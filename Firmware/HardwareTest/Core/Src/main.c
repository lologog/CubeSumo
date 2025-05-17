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
#define NUM_SENSORS (9) //how many analog sensors are used in the robot
#define SOFTWARE_PWM_PERIOD (5) //software PWM period in ticks - the less the fester is the PWM freq
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

//struct that associates an ADC channel with a pointer to its output variable
typedef struct
{
    uint32_t channel;
    uint32_t* value_ptr;
} Sensor;

//enum representing current motor direction state
typedef enum {
    DIR_STOP,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT
} Direction_t;

//variables used to store analog values read from sensors
uint32_t adc_value_1, adc_value_2, adc_value_3, adc_value_4;
uint32_t adc_value_5, adc_value_6, adc_value_7, adc_value_8, adc_value_9;

static uint8_t pwm_counter = 0; //PWM counter incremented every 1ms from SysTick

//variable tracking current movement direction
Direction_t current_direction = DIR_STOP;

//global variables storing duty cycle 0-100% for left and right motors
volatile uint8_t pwm_left = 0;
volatile uint8_t pwm_right = 0;

//button
uint8_t button_value = 0;

//array that maps ADC channels to their corresponding sensor value variables
Sensor sensors[NUM_SENSORS] = {
    {ADC_CHANNEL_16, &adc_value_1},
    {ADC_CHANNEL_11, &adc_value_2},
    {ADC_CHANNEL_12, &adc_value_3},
    {ADC_CHANNEL_7,  &adc_value_4},
    {ADC_CHANNEL_15, &adc_value_5},
    {ADC_CHANNEL_9,  &adc_value_6},
    {ADC_CHANNEL_6,  &adc_value_7},
    {ADC_CHANNEL_5,  &adc_value_8},
    {ADC_CHANNEL_8,  &adc_value_9}
};

//selects the specified ADC channel and prepares it for conversion
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

//reads values from all configured ADC channels and stores the results
void UpdateAllSensors(void)
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        ADC_SetActiveChannel(&hadc1, sensors[i].channel);
        HAL_ADC_Start(&hadc1);

        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
        	//store the value directly in the variable pointed to by the sensor
            *(sensors[i].value_ptr) = HAL_ADC_GetValue(&hadc1);
        }
    }
}

//stops both motors by disabling the enable pins
void StopMotors(void)
{
    HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, GPIO_PIN_RESET);
    pwm_left = 0;
    pwm_right = 0;
    current_direction = DIR_STOP;
}

//stops motors briefly if direction is changing to avoid shoot-through
void SafeDirectionChange(Direction_t new_dir)
{
    if (current_direction != DIR_STOP && current_direction != new_dir)
    {
        StopMotors();
        HAL_Delay(100); //dead time to allow safe switching
    }
}

//moves robot forward at specified speed (0–100%)
void MoveBack(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_FORWARD);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_SET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = duty;

    current_direction = DIR_FORWARD;
}

//moves robot backward at specified speed (0–100%)
void MoveForward(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_BACKWARD);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_RESET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = duty;

    current_direction = DIR_BACKWARD;
}

//spins robot left so it turns in place at specified speed
void TurnLeft(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_LEFT);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_SET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = duty;

    current_direction = DIR_LEFT;
}

//spins robot right so it turns in place at specified speed
void TurnRight(uint8_t speed_percent)
{
    SafeDirectionChange(DIR_RIGHT);

    HAL_GPIO_WritePin(PHASE_LEFT_GPIO_Port, PHASE_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PHASE_RIGHT_GPIO_Port, PHASE_RIGHT_Pin, GPIO_PIN_RESET);

    uint8_t duty;

    if (speed_percent > 100)
    {
        duty = SOFTWARE_PWM_PERIOD;
    }
    else
    {
        duty = (speed_percent * SOFTWARE_PWM_PERIOD) / 100;
    }

    pwm_left = duty;
    pwm_right = duty;

    current_direction = DIR_RIGHT;
}

//software based PWM controller - should be called every 1ms from SysTick_Handler :)
void SoftwarePWM_Loop(void)
{
    pwm_counter++;
    if (pwm_counter >= SOFTWARE_PWM_PERIOD)
    {
    	pwm_counter = 0;
    }


    //left motor PWM
    if (pwm_counter < pwm_left)
    {
    	HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, GPIO_PIN_SET);
    }
    else
    {
    	HAL_GPIO_WritePin(EN_LEFT_GPIO_Port, EN_LEFT_Pin, GPIO_PIN_RESET);
    }

    //right motor PWM
    if (pwm_counter < pwm_right)
    {
    	HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, GPIO_PIN_SET);
    }
    else
    {
    	HAL_GPIO_WritePin(EN_RIGHT_GPIO_Port, EN_RIGHT_Pin, GPIO_PIN_RESET);
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
	  UpdateAllSensors();

	  MoveForward(20); //this is minimum

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
