/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DIGIT_ON GPIO_PIN_SET
#define DIGIT_OFF GPIO_PIN_RESET
#define SEG_ON GPIO_PIN_RESET
#define SEG_OFF GPIO_PIN_SET

uint8_t curDigitIdx = 0;

char valToDigit(uint8_t idx, double val){
	int shownVal = round(val);
	int shownMag = abs(shownVal);


	if(shownVal > 100){
		shownVal = 100;
	} else if(shownVal < -100){
		shownVal = -100;
	}

	switch(idx){
	case 3:
		// top just shows negative sign
		if(shownVal < 0){
			return '-';
		} else {
			return  ' ';
		}
	break;
	case 2:
		if(shownMag >= 100){
			int tmp = shownMag % 1000;
			tmp = tmp / 100;
		    return floor(tmp) + '0';
		} else {
			return ' ';
		}
	break;
	case 1:
		if(shownMag >= 10){
			int tmp = shownMag % 100;
			tmp = tmp / 10;
		    return floor(tmp) + '0';
		} else {
			return ' ';
		}
	break;
	case 0:
		int tmp = shownMag % 10;
	    return floor(tmp) + '0';
	break;
	default:
		return ' ';
	break;
	}

}

void writeDigit(uint8_t idx, char val, bool hasDP){

	//Select the digit source line. SET provides power to the digit, RESET removes power
	HAL_GPIO_WritePin(LED_V0_GPIO_Port, LED_V0_Pin, (idx == 0) ? DIGIT_ON:DIGIT_OFF);
	HAL_GPIO_WritePin(LED_V1_GPIO_Port, LED_V1_Pin, (idx == 1) ? DIGIT_ON:DIGIT_OFF);
	HAL_GPIO_WritePin(LED_V2_GPIO_Port, LED_V2_Pin, (idx == 2) ? DIGIT_ON:DIGIT_OFF);
	HAL_GPIO_WritePin(LED_V3_GPIO_Port, LED_V3_Pin, (idx == 3) ? DIGIT_ON:DIGIT_OFF);

	// Always switch dp
	HAL_GPIO_WritePin(LED_DP_GPIO_Port, LED_DP_Pin, hasDP?SEG_ON:SEG_OFF);


	//Switch on supported characters on 7 segment display
	switch(val){
		case '0':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_OFF);
		  break;
		case '1':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_OFF);
		  break;
		case '2':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_ON);
		  break;
		case '3':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_ON);
		  break;
		case '4':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_ON);
		  break;
		case '5':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_ON);
		  break;
		case '6':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_ON);
		  break;
		case '7':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_OFF);
		  break;
		case '8':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_ON);
		  break;
		case '9':
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_ON);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_ON);
		  break;
		default:
		  HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_C_GPIO_Port, LED_C_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_D_GPIO_Port, LED_D_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_E_GPIO_Port, LED_E_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_F_GPIO_Port, LED_F_Pin, SEG_OFF);
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, SEG_OFF);
		  break;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(5); //world's worst rtos
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 25;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_B_Pin|LED_V3_Pin|LED_V2_Pin|LED_V4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_G_Pin|LED_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_DP_Pin|LED_D_Pin|LED_E_Pin|LED_F_Pin
                          |LED_A_Pin|LED_V1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CTRL_A_Pin CTRL_B_Pin */
  GPIO_InitStruct.Pin = CTRL_A_Pin|CTRL_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CTRL_SW_Pin */
  GPIO_InitStruct.Pin = CTRL_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTRL_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_B_Pin LED_V3_Pin LED_V2_Pin LED_V4_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_V3_Pin|LED_V2_Pin|LED_V4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G_Pin LED_C_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_DP_Pin LED_D_Pin LED_E_Pin LED_F_Pin
                           LED_A_Pin LED_V1_Pin */
  GPIO_InitStruct.Pin = LED_DP_Pin|LED_D_Pin|LED_E_Pin|LED_F_Pin
                          |LED_A_Pin|LED_V1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RUN_SW_Pin */
  GPIO_InitStruct.Pin = RUN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RUN_SW_GPIO_Port, &GPIO_InitStruct);

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

