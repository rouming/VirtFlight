/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "stm32_hal_legacy.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct channel {
	uint16_t       width;
};

struct pwm {
	uint16_t       pin;
	GPIO_TypeDef   *port;
	struct channel *channel;
	uint16_t       rising_cnt;
};

enum ppm_state {
	PPM_SYNCING_INIT, /* Waits for the first falling or rising edge */
	PPM_SYNCING,      /* Waits for long delay between impulses */
	PPM_COUNTING,     /* Counts channels */
	PPM_READY         /* PPM ready */
};

enum {
	PPM_IDLE_MS = 4,  /* Pause between PPM frames */
	NO_INPUT_MS = 20, /* If transmitter is off */

	NR_AXIS     = 10, /* How many axis joystick supports */
	NR_CHANNELS = 10, /* How many channels do we handle */
	NR_PWM_PINS = 6,  /* How many PWM pins do we have */
};

struct ppm {
	uint16_t       pin;
	GPIO_TypeDef   *port;
	struct channel *channels;
	unsigned int   nr_channels;
	unsigned int   nr_edges;

	enum ppm_state state;
	uint16_t       edge_ms;
	uint16_t       edge_cnt;
};

enum input_state {
	PROBING,
	PPM_INPUT,
	PWM_INPUT,
	UNKNOWN_INPUT,
};

struct input {
	enum input_state state;
	struct pwm       pwm_pins[NR_PWM_PINS];
	struct ppm       ppm_pin;
	uint16_t         last_input_ms;
};

static struct channel channels[NR_CHANNELS];

static struct input input = {
	.state = PROBING,
	.pwm_pins = {
		[0] = {.pin = CH1_Pin, .port = CH1_GPIO_Port, .channel = &channels[0]},
		[1] = {.pin = CH2_Pin, .port = CH2_GPIO_Port, .channel = &channels[1]},
		[2] = {.pin = CH3_Pin, .port = CH3_GPIO_Port, .channel = &channels[2]},
		[3] = {.pin = CH4_Pin, .port = CH4_GPIO_Port, .channel = &channels[3]},
		[4] = {.pin = CH5_Pin, .port = CH5_GPIO_Port, .channel = &channels[4]},
		[5] = {.pin = CH6_Pin, .port = CH6_GPIO_Port, .channel = &channels[5]},
	},
	.ppm_pin = {
		.pin          = CH1_Pin,
		.port         = CH1_GPIO_Port,
		.channels     = channels,
		.nr_channels  = NR_CHANNELS,
	},
};

static inline void set_channel_width(struct channel *ch, uint16_t width)
{
	/* Clamp to [0, 4095] */
	if (width < 3900) {
		width = 0;
	} else {
		width -= 3900;
		if (width > 4095)
			width = 4095;
	}

	ch->width = width;
}

static void handle_pwm_input(uint16_t pin)
{
	uint16_t cnt;
	int i;

	cnt = __HAL_TIM_GetCounter(&htim3);

	for (i = 0; i < NR_PWM_PINS; i++) {
		struct pwm *pwm = &input.pwm_pins[i];

		if (pwm->pin != pin)
			continue;

		if (HAL_GPIO_ReadPin(pwm->port, pwm->pin)) {
			pwm->rising_cnt = cnt;
		} else {
			/* Update channel */
			uint16_t width = cnt - pwm->rising_cnt;
			set_channel_width(pwm->channel, width);
		}
	}
}

static void handle_ppm_input(uint16_t pin)
{
	struct ppm *ppm = &input.ppm_pin;
	uint16_t diff_ms, cnt, ms;

	if (ppm->pin != pin)
		/* Not a PPM pin, just ignore */
		return;

	ms = __HAL_TIM_GetCounter(&htim2);
	cnt = __HAL_TIM_GetCounter(&htim3);

	switch (ppm->state) {
	case PPM_SYNCING_INIT:
		ppm->edge_ms = ms;
		ppm->state = PPM_SYNCING;
		break;
	case PPM_SYNCING:
		diff_ms = ms - ppm->edge_ms;
		ppm->edge_ms = ms;

		if (diff_ms < PPM_IDLE_MS)
			break;

		/* Got long enough idle, now count channels */
		ppm->nr_edges = 0;
		ppm->state = PPM_COUNTING;
		break;
	case PPM_COUNTING:
		diff_ms = ms - ppm->edge_ms;
		ppm->edge_ms = ms;
		ppm->nr_edges += 1;

		if (diff_ms < PPM_IDLE_MS)
			break;

		/* Got long enough idle, now decide what input we have */

		if (ppm->nr_edges & 1) {
			/* Can't be odd for any input */
			input.state = UNKNOWN_INPUT;
			ppm->state = PPM_SYNCING_INIT;
			break;
		}
		if (ppm->nr_edges == 2) {
			input.state = PWM_INPUT;
			ppm->state = PPM_SYNCING_INIT;
			break;
		}
		input.state = PPM_INPUT;
		ppm->state = PPM_READY;
		ppm->nr_edges = 0;
		ppm->edge_cnt = cnt;
		break;

	case PPM_READY:
		diff_ms = ms - ppm->edge_ms;
		ppm->edge_ms = ms;
		ppm->nr_edges += 1;

		if (diff_ms >= PPM_IDLE_MS) {
			ppm->nr_edges = 0;
			ppm->edge_cnt = cnt;
			break;
		}
		if (ppm->nr_edges % 2) {
			int i_ch = (ppm->nr_edges / 2) - 1;
			if (i_ch < ppm->nr_channels) {
				uint16_t width = cnt - ppm->edge_cnt;
				set_channel_width(&ppm->channels[i_ch], width);
			}
			ppm->edge_cnt = cnt;
			break;
		}
		break;

	default:
		Error_Handler();
		break;
	}
}

static void reset_input_to_probing(void)
{
	input.state = PROBING;
	input.ppm_pin.state = PPM_SYNCING_INIT;
};

static void check_no_input(void)
{
	uint16_t cnt_ms = __HAL_TIM_GetCounter(&htim2);

	/* Switch back to PROBING if no input has happened */
	if (input.state != PROBING) {
		uint16_t diff_ms = cnt_ms - input.last_input_ms;
		if (diff_ms >= NO_INPUT_MS)
			reset_input_to_probing();
	}
	input.last_input_ms = cnt_ms;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	check_no_input();

	switch (input.state) {
	case PROBING:
	case PPM_INPUT:
		handle_ppm_input(pin);
		break;
	case PWM_INPUT:
		handle_pwm_input(pin);
		break;
	default:
		Error_Handler();
		break;
	}
}

static void USBD_HID_WaitForSend(USBD_HandleTypeDef *dev)
{
	volatile USBD_HID_HandleTypeDef *hhid =
		(volatile USBD_HID_HandleTypeDef *)dev->pClassData;

	while (hhid->state == HID_BUSY)
		;
}

struct joystick_report {
	uint8_t  report_id;
	uint16_t axis[NR_AXIS];
	uint8_t  buttons:3;
	uint8_t  padding:5;
} __attribute__((packed));

static void Send_JoystickReport(void)
{
	  struct joystick_report joystick = {
		  .report_id = HID_JOYSTICK_REPORT_ID
	  };
	  int i, nr;

	  nr = NR_AXIS < NR_CHANNELS ? NR_AXIS : NR_CHANNELS;
	  for (i = 0; i < nr; i++)
		  joystick.axis[i] = channels[i].width;

	  USBD_HID_WaitForSend(&hUsbDeviceFS);
	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&joystick,
						  sizeof(joystick));
}

struct keyboard_report {
	uint8_t report_id;
	uint8_t keys[8];
};

static void Send_Key(uint8_t keycode)
{
	struct keyboard_report keyboard = {
		.report_id = HID_KEYBOARD_REPORT_ID
	};

	keyboard.keys[2] = keycode;

	USBD_HID_WaitForSend(&hUsbDeviceFS);
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyboard,
						sizeof(keyboard));
}

static void PressAndRelease_Key(struct channel *ch, uint8_t *sent,
								uint8_t keycode)
{
	if (ch->width >= 1500 && !*sent) {
		/* Press a key */
		Send_Key(keycode);
		/* Release a key */
		Send_Key(0x00);

		*sent = 1;

	} else if (ch->width < 1500 && *sent) {
		*sent = 0;
	}
}

static void PressOrRelease_Key(struct channel *ch, uint8_t *pressed,
							   uint8_t keycode)
{
	if (ch->width >= 1500) {
		/* Press a key many times */
		Send_Key(keycode);

		*pressed = 1;

	} else if (ch->width < 1500 && *pressed) {
		/* Release a key once */
		Send_Key(0x00);

		*pressed = 0;
	}
}

/*
 * We map 5'th channel for 't' key
 */
static void Handle_T_Key(void)
{
	static uint8_t t_sent;

	PressAndRelease_Key(&channels[4], &t_sent, 0x17);
}

/*
 * We map 5'th channel for 'y' key
 */
static void Handle_Y_Key(void)
{
	static uint8_t y_pressed;

	PressOrRelease_Key(&channels[4], &y_pressed, 0x1c);
}

/*
 * We map 6'th channel for 'r' key
 */
static void Handle_R_Key(void)
{
	static uint8_t r_sent;

	PressAndRelease_Key(&channels[5], &r_sent, 0x15);
}

static void Send_KeyboardReport(void)
{
	/* Either T, either Y */
	Handle_T_Key();
	(void)Handle_Y_Key;

	Handle_R_Key();
}

static void Status_Blink(void)
{
	static uint16_t prev_ms;
	static int init;

	uint16_t ms, diff_ms;

	if (!init) {
		prev_ms = __HAL_TIM_GetCounter(&htim2);
		init = 1;
	}
	ms = __HAL_TIM_GetCounter(&htim2);
	diff_ms = ms - prev_ms;

	if (input.state == PROBING) {
		/* 0.2s on, 0.2s off */

		if (diff_ms >= 200) {
			HAL_GPIO_TogglePin(BP_LED_GPIO_Port, BP_LED_Pin);
			prev_ms = ms;
		}

	} else if (input.state == PWM_INPUT) {
		/* 1s on, 1s off */

		if (diff_ms >= 1000) {
			HAL_GPIO_TogglePin(BP_LED_GPIO_Port, BP_LED_Pin);
			prev_ms = ms;
		}
	} else if (input.state == PPM_INPUT) {
		/* 1s off, 0.2s on, 0.2s off, 0.2s on */

		static int state;

		if (!state) {
			/* 1s off */
			HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_SET);
			state = 1;
		} else if (state == 1 && diff_ms >= 1000) {
			/* 0.2s on */
			HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_RESET);
			prev_ms = ms;
			state = 2;
		} else if (state == 2 && diff_ms >= 200) {
			/* 0.2s off */
			HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_SET);
			prev_ms = ms;
			state = 3;
		} else if (state == 3 && diff_ms >= 200) {
			/* 0.2s on */
			HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_RESET);
			prev_ms = ms;
			state = 4;
		} else if (state == 4 && diff_ms >= 200) {
			/* repeat */
			state = 0;
			prev_ms = ms;
		}
	} else {
		Error_Handler();
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_TIM_Base_Start(&htim3))
	  Error_Handler();

  if (HAL_TIM_Base_Start_IT(&htim2))
	  Error_Handler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Turn led off */
  HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_SET);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Send_JoystickReport();
	  /* Send_KeyboardReport(); */
	  (void)Send_KeyboardReport;
	  Status_Blink();
	  HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BP_LED_Pin */
  GPIO_InitStruct.Pin = BP_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BP_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CH1_Pin CH2_Pin CH3_Pin CH4_Pin 
                           CH5_Pin CH6_Pin */
  GPIO_InitStruct.Pin = CH1_Pin|CH2_Pin|CH3_Pin|CH4_Pin 
                          |CH5_Pin|CH6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	while (1) {
		/* Fast blinking means error */
		HAL_GPIO_TogglePin(BP_LED_GPIO_Port, BP_LED_Pin);
		HAL_Delay(70);
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
