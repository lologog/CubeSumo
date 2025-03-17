/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adc_value_1 = 0;
uint32_t adc_value_2 = 0;
uint32_t adc_value_3 = 0;
uint32_t adc_value_4 = 0;
uint32_t adc_value_5 = 0;
uint32_t adc_value_6 = 0;
uint32_t adc_value_7 = 0;
uint32_t adc_value_8 = 0;
uint32_t adc_value_9 = 0;
uint8_t button_value = 0;

void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = AdcChannel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_SET);

	  HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_SET);

	  HAL_Delay(3000);

	  HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_RESET);

	  HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_RESET);

	  HAL_Delay(3000);
	  /* Start ADC Conversion */
	  if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_1 = HAL_ADC_GetValue(&hadc1); // Get X value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_16);
	        HAL_ADC_Start(&hadc1);
	      }

	      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_2 = HAL_ADC_GetValue(&hadc1); // Get Y value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_11);
	        HAL_ADC_Start(&hadc1);
	      }

	      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_3 = HAL_ADC_GetValue(&hadc1); // Get Y value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_12);
	        HAL_ADC_Start(&hadc1);
	      }

	      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_4 = HAL_ADC_GetValue(&hadc1); // Get Y value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_7);
	        HAL_ADC_Start(&hadc1);
	      }

	      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_5 = HAL_ADC_GetValue(&hadc1); // Get Y value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_15);
	        HAL_ADC_Start(&hadc1);
	      }

	      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_6 = HAL_ADC_GetValue(&hadc1); // Get Y value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_9);
	        HAL_ADC_Start(&hadc1);
	      }

	      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_7 = HAL_ADC_GetValue(&hadc1); // Get Y value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_6);
	        HAL_ADC_Start(&hadc1);
	      }

	      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_8 = HAL_ADC_GetValue(&hadc1); // Get Y value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_5);
	        HAL_ADC_Start(&hadc1);
	      }

	      if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	      {
	        adc_value_9 = HAL_ADC_GetValue(&hadc1); // Get Y value
	        ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_8);
	        HAL_ADC_Start(&hadc1);
	      }

	      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(LED3_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

	      if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin) == GPIO_PIN_RESET)
	      {
	    	  button_value = 1;
	    	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	      }
	      else
	      {
	    	  button_value = 0;
	    	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	      }



    /* USER CODE END WHILE */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED3_Pin|LED2_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|EN_LEFT_Pin|PHASE_LEFT_Pin|EN_RIGHT_Pin
                          |PHASE_RIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED3_Pin LED2_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED3_Pin|LED2_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_LEFT_Pin PHASE_LEFT_Pin EN_RIGHT_Pin PHASE_RIGHT_Pin */
  GPIO_InitStruct.Pin = EN_LEFT_Pin|PHASE_LEFT_Pin|EN_RIGHT_Pin|PHASE_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
