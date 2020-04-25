// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Taken from Betaflight.
 *
 * Driver for IBUS (Flysky) receiver
 *   - initial implementation for MultiWii by Cesco/Pl¸schi
 *   - implementation for BaseFlight by Andreas (fiendie) Tacke
 *   - ported to CleanFlight by Konstantin (digitalentity) Sharlaimov
 *   - ported to BluePill + optimization and tweaks by Roman P.
 */

#include "stm32f1xx_hal.h"
#include "ibus.h"

static int is_valid_ia6_length(uint8_t length)
{
    return length == IBUS_TELEMETRY_PACKET_LENGTH ||
		   length == IBUS_SERIAL_RX_PACKET_LENGTH;
}

/* Returns next index frame in round-robin manner */
static int next_frame_index(struct ibus *ibus)
{
	return ibus->frame_ind++ % (sizeof(ibus->frames)/sizeof(ibus->frames[0]));
}

static void set_current_frame(struct ibus *ibus)
{
	int i = next_frame_index(ibus);

	/*
	 * For new ibus frame we select possibly next one in round-robin
	 * in order not to block consumer waiting this frame to become
	 * ready. When consumer starts reading data from a ready frame
	 * he marks frame as active, thus producer must select another
	 * frame for new data not to corrupt data while consumer reads.
	 */

	ibus->frame = (ibus->active_frame != &ibus->frames[i] ?
				   &ibus->frames[i] : &ibus->frames[next_frame_index(ibus)]);
	if (ibus->ready_frame == ibus->frame)
		/* We do not want a consumer to observe frame in progress */
		ibus->ready_frame = NULL;

	ibus->frame->pos = 0;
}

static void ibus_handle_rx(struct ibus *ibus, uint8_t byte)
{
    if (!ibus->frame) {
        if (is_valid_ia6_length(byte)) {
            ibus->model = IBUS_MODEL_IA6B;
            ibus->sync_byte = byte;
            ibus->frame_size = byte;
            ibus->channel_off = 2;
            ibus->checksum = 0xffff;
        } else if (!ibus->sync_byte && byte == 0x55) {
            ibus->model = IBUS_MODEL_IA6;
            ibus->sync_byte = 0x55;
            ibus->frame_size = 31;
            ibus->checksum = 0x0000;
            ibus->channel_off = 1;
        } else if (ibus->sync_byte != byte) {
            return;
        }
		set_current_frame(ibus);
    }

    ibus->frame->data[ibus->frame->pos++] = byte;
    if (ibus->frame->pos == ibus->frame_size) {
		ibus->ready_frame = ibus->frame;
		ibus->frame = NULL;
    }
}

void handle_ibus_input(struct ibus *ibus)
{
	if (!__HAL_UART_GET_FLAG(ibus->huart, UART_FLAG_ORE) &&
	    !__HAL_UART_GET_FLAG(ibus->huart, UART_FLAG_FE)) {
		ibus_handle_rx(ibus, ibus->rx_byte);
		HAL_UART_Receive_IT(ibus->huart, &ibus->rx_byte, 1);
	}
}

static int is_valid_checksum_ia6(struct ibus *ibus, struct ibus_frame *frame)
{
    uint16_t checksum, rxsum;
	uint8_t offset, i;

    checksum = ibus->checksum;
    rxsum = frame->data[ibus->frame_size - 2] +
		(frame->data[ibus->frame_size - 1] << 8);

    for (i = 0, offset = ibus->channel_off; i < IBUS_MAX_SLOTS;
		 i++, offset += 2) {
        checksum += frame->data[offset] + (frame->data[offset + 1] << 8);
    }
    return (checksum == rxsum);
}

static uint16_t calculate_checksum_ia6b(struct ibus *ibus,
										struct ibus_frame *frame)
{
    uint16_t i, checksum;
    uint8_t size;

	checksum = ibus->checksum;
	size = frame->data[0] - IBUS_CHECKSUM_SIZE;

    for (i = 0; i < size; i++)
        checksum -= frame->data[i];

    return checksum;
}

static int is_valid_checksum_ia6b(struct ibus *ibus, struct ibus_frame *frame)
{
	uint8_t size = ibus->frame_size;
	uint16_t checksum;

	checksum = calculate_checksum_ia6b(ibus, frame);

    /* Note that there's a byte order swap to little endian here */
    return (checksum >> 8) == frame->data[size - 1] &&
		   (checksum & 0xff) == frame->data[size - 2];
}

static int is_valid_checksum(struct ibus *ibus, struct ibus_frame *frame)
{
    if (ibus->model == IBUS_MODEL_IA6)
		return is_valid_checksum_ia6(ibus, frame);
	return is_valid_checksum_ia6b(ibus, frame);
}

static uint8_t get_channel_data(struct ibus *ibus, struct ibus_frame *frame,
								uint16_t *channels, uint8_t nr)
{
    uint8_t i, max_slots, offset;

	max_slots = (nr < IBUS_MAX_SLOTS ? nr : IBUS_MAX_SLOTS);
	offset = ibus->channel_off;
    for (i = 0; i < max_slots; i++, offset += 2) {
        channels[i] = frame->data[offset] |
					 ((frame->data[offset + 1] & 0x0F) << 8);
	}

    /*
	 * Latest IBUS recievers are using prviously not used 4 bits
	 * on every channel to incresse total channel count.
	 */
	nr = (nr < IBUS_MAX_CHANNELS ? nr : IBUS_MAX_CHANNELS);
	offset = ibus->channel_off + 1;
    for (; i < nr; i++, offset += 6) {
        channels[i] = ((frame->data[offset] & 0xF0) >> 4) |
					  (frame->data[offset + 2] & 0xF0) |
					  ((frame->data[offset + 4] & 0xF0) << 4);
    }

	return i;
}

int ibus_get_channels(struct ibus *ibus, uint16_t *channels, uint8_t nr)
{
	struct ibus_frame *frame;
	int ret;

	/* Get ready frame and lock it while reading data from it */
	__disable_irq();
	frame = ibus->ready_frame;
	ibus->active_frame = frame;
	__enable_irq();

	if (!frame) {
		ret = -IBUS_ERR_NODATA;
		goto out;
	}

	if (!is_valid_checksum(ibus, frame)) {
		ret = -IBUS_ERR_CHECKSUM;
		goto out;
	}

	ret = get_channel_data(ibus, frame, channels, nr);

out:
	/* We are done with this frame */
	ibus->active_frame = NULL;

    return ret;
}

void ibus_init(struct ibus *ibus, UART_HandleTypeDef *huart)
{
	/* Start receiving bytes immediately */
	ibus->huart = huart;
	ibus->frame = NULL;
	ibus->ready_frame = NULL;
	ibus->active_frame = NULL;
	HAL_UART_Receive_IT(huart, &ibus->rx_byte, 1);
}
