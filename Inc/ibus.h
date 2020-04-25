/* SPDX-License-Identifier: GPL-2.0-or-later */

enum {
	IBUS_MAX_CHANNELS  = 18,
	/* In AFHDS there is 18 channels encoded in 14 slots (each slot is 2 byte long) */
	IBUS_MAX_SLOTS     = 14,
	IBUS_BUFFSIZE      = 32,
	IBUS_CHECKSUM_SIZE = 2,

	IBUS_MODEL_IA6B  = 0,
	IBUS_MODEL_IA6   = 1,

	IBUS_TELEMETRY_PACKET_LENGTH = 4,
	IBUS_SERIAL_RX_PACKET_LENGTH = 32,

	IBUS_ERR_OK       = 0,
	IBUS_ERR_NODATA   = 1,
	IBUS_ERR_CHECKSUM = 2,
};

struct ibus_frame {
	uint8_t data[IBUS_BUFFSIZE];
	uint8_t pos;
};

struct ibus {
	uint8_t  model;
	uint8_t  sync_byte;
	uint8_t  frame_size;
	uint8_t  channel_off;
	uint16_t checksum;
	uint8_t  frame_ind;

	uint16_t channels[IBUS_MAX_CHANNELS];

	uint8_t  rx_byte;

	/* Use 2 frames for consumer and producer in parallel */
	struct ibus_frame frames[2];
	struct ibus_frame *ready_frame;  /* last ready frame set by producer */
	struct ibus_frame *active_frame; /* frame from which consumer reads data */
	struct ibus_frame *frame;        /* current frame in work for producer */

	UART_HandleTypeDef *huart;
};

void ibus_init(struct ibus *ibus, UART_HandleTypeDef *);
int ibus_get_channels(struct ibus *ibus, uint16_t *channels, uint8_t nr);
void handle_ibus_input(struct ibus *ibus);
