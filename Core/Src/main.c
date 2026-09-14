/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
#define NTC_CHANNELS   13     // physical channel count per board
#define BOARD_ID       0      // 0-5, UNIQUE per board, sets CAN ID offset

/* NTC divider + ADC physical constants (R1=NTC->3.3V, R2=10k->GND) */
#define R25         10000.0f  // NTC resistance at 25C
#define BETA        3950.0f   // NTC beta coefficient
#define T25_K       298.15f   // 25C in Kelvin
#define R_FIXED     10000.0f  // fixed reference resistor (R2)
#define VREF        3.3f      // ADC reference voltage
#define ADC_MAX     4095.0f   // 12-bit ADC full scale

/*Global variables */
volatile uint16_t adc_vals[NTC_CHANNELS];
volatile int16_t temp_buffer_A[NTC_CHANNELS];   // ISR writes here (raw ADC counts)
volatile int16_t temp_buffer_B[NTC_CHANNELS];   // main loop's private snapshot



volatile uint16_t OT_TRIP_RAW       = 3155;  // over-temp trip threshold (raw ADC count)
volatile uint16_t OT_CLEAR_RAW      = 2854;  // over-temp hysteresis clear threshold
volatile uint16_t SENSOR_SHORT_RAW  = 50;    // near-0V -> open NTC / broken wire
volatile uint16_t SENSOR_OPEN_RAW   = 4045;  // near-3.3V -> shorted NTC / broken ref resistor

volatile uint8_t OT_DEBOUNCE_N     = 5;
volatile uint8_t SENSOR_DEBOUNCE_N = 1;

typedef enum {
    FAULT_NONE = 0,
    FAULT_OVERTEMP,
    FAULT_SENSOR_SHORT,   // reads near 0V
    FAULT_SENSOR_OPEN,    // reads near 3.3V
} fault_type_t;

volatile fault_type_t fault_latched_type[NTC_CHANNELS];  // latched fault, if any, per channel
volatile uint8_t      fault_debounce_count[NTC_CHANNELS]; // consecutive-sample counter, shared

#define FAULT_GPIO_Port   GPIOB
#define FAULT_GPIO_Pin    GPIO_PIN_0

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

HAL_StatusTypeDef CAN_SendFault(uint8_t channel, fault_type_t type);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    for (uint8_t ch = 0; ch < NTC_CHANNELS; ch++)
    {
        uint16_t raw = adc_vals[ch];

        if (raw <= SENSOR_SHORT_RAW)
        {
            if (fault_debounce_count[ch] < SENSOR_DEBOUNCE_N)
                fault_debounce_count[ch]++;

            if (fault_debounce_count[ch] >= SENSOR_DEBOUNCE_N &&
                fault_latched_type[ch] == FAULT_NONE)
            {
                fault_latched_type[ch] = FAULT_SENSOR_SHORT;
                CAN_SendFault(ch, FAULT_SENSOR_SHORT);
            }
        }
        else if (raw >= SENSOR_OPEN_RAW)
        {
            if (fault_debounce_count[ch] < SENSOR_DEBOUNCE_N)
                fault_debounce_count[ch]++;

            if (fault_debounce_count[ch] >= SENSOR_DEBOUNCE_N &&
                fault_latched_type[ch] == FAULT_NONE)
            {
                fault_latched_type[ch] = FAULT_SENSOR_OPEN;
                CAN_SendFault(ch, FAULT_SENSOR_OPEN);
            }
        }
        else if (raw >= OT_TRIP_RAW)
        {
            if (fault_debounce_count[ch] < OT_DEBOUNCE_N)
                fault_debounce_count[ch]++;

            if (fault_debounce_count[ch] >= OT_DEBOUNCE_N &&
                fault_latched_type[ch] == FAULT_NONE)
            {
                fault_latched_type[ch] = FAULT_OVERTEMP;
                CAN_SendFault(ch, FAULT_OVERTEMP);
            }
        }
        else if (raw <= OT_CLEAR_RAW)
        {
            fault_debounce_count[ch] = 0;
        }

        temp_buffer_A[ch] = (int16_t)raw;
    }
}


HAL_StatusTypeDef CAN_SendFault(uint8_t channel, fault_type_t type)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t tx_mailbox;

    tx_header.StdId = BOARD_ID;              // 0x000-0x005: dominant arbitration
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    tx_data[0] = channel;                    // which of 13 channels faulted
    tx_data[1] = (uint8_t)type;               // fault type enum value

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        /* Mailbox full: correctness requirement is to NOT block here.
           TODO: implement retry-pending flag + retry on next cycle,
           rather than silently dropping (flagged as still open). */
        return HAL_ERROR;
    }

    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

void Fault_Reset(uint8_t channel)
{
    fault_latched_type[channel] = FAULT_NONE;
    fault_debounce_count[channel] = 0;
    /* No GPIO to manage - master decides all downstream action based on CAN fault frames */
}

static float ADC_ToTemperature(uint16_t adc_count)
{
    float v_out, r_ntc, temp_k;

    /* Guard against divide-by-zero / log(0) at the extremes */
    if (adc_count == 0) adc_count = 1;
    if (adc_count >= (uint16_t)ADC_MAX) adc_count = (uint16_t)ADC_MAX - 1;

    v_out = (adc_count * VREF) / ADC_MAX;

    /* R1 (NTC) -> 3.3V, R2 (fixed) -> GND, ADC reads the divider node:
       Vout = VREF * R2 / (R_ntc + R2)
       Rearranged for R_ntc: R_ntc = R2 * (VREF - Vout) / Vout */
    r_ntc = R_FIXED * (VREF - v_out) / v_out;

    /* Beta equation (simplified Steinhart-Hart) */
    temp_k = 1.0f / ((1.0f / T25_K) + (1.0f / BETA) * logf(r_ntc / R25));

    return temp_k - 273.15f;
}

static int16_t Temp_ToFixedPoint(float temp_c)
{
    float scaled = temp_c * 100.0f;

    if (scaled >= 0.0f)
        return (int16_t)(scaled + 0.5f);
    else
        return (int16_t)(scaled - 0.5f);
}

void Send_Telemetry_Frames(int16_t *raw_snapshot)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;
    int16_t temp_fixed[NTC_CHANNELS];

    /* Convert all 13 raw ADC snapshots to fixed-point degC x100 */
    for (uint8_t i = 0; i < NTC_CHANNELS; i++)
    {
        float temp_c = ADC_ToTemperature((uint16_t)raw_snapshot[i]);
        temp_fixed[i] = Temp_ToFixedPoint(temp_c);
    }

    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.TransmitGlobalTime = DISABLE;

    /* Frames 0-2: 4 channels each, big-endian int16 packing */
    for (uint8_t frame = 0; frame < 3; frame++)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            int16_t val = temp_fixed[frame * 4 + i];
            tx_data[2*i]     = (uint8_t)((val >> 8) & 0xFF);
            tx_data[2*i + 1] = (uint8_t)(val & 0xFF);
        }

        tx_header.StdId = 0x100 + (BOARD_ID * 0x10) + frame;
        tx_header.DLC = 8;

        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
    }

    /* Frame 3: last channel only */
    {
        int16_t val = temp_fixed[12];
        tx_data[0] = (uint8_t)((val >> 8) & 0xFF);
        tx_data[1] = (uint8_t)(val & 0xFF);

        tx_header.StdId = 0x100 + (BOARD_ID * 0x10) + 3;
        tx_header.DLC = 2;

        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  uint32_t last_tx_time = 0;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_vals, NTC_CHANNELS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

      if (HAL_GetTick() - last_tx_time >= 100)
      {
          last_tx_time = HAL_GetTick();

          /* Atomic snapshot - shortest possible critical section */
          __disable_irq();
          memcpy((void*)temp_buffer_B, (void*)temp_buffer_A, sizeof(temp_buffer_B));
          __enable_irq();

          /* Convert + pack + transmit telemetry - always, fault or not */
          Send_Telemetry_Frames(temp_buffer_B);
      }

      /* Room for other non-blocking background tasks here later,
         e.g. CAN error/bus-off monitoring, watchdog kick, etc. */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 13;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
