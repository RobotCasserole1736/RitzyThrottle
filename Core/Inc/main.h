/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CTRL_A_Pin GPIO_PIN_0
#define CTRL_A_GPIO_Port GPIOC
#define CTRL_A_EXTI_IRQn EXTI0_IRQn
#define CTRL_B_Pin GPIO_PIN_1
#define CTRL_B_GPIO_Port GPIOC
#define CTRL_B_EXTI_IRQn EXTI1_IRQn
#define CTRL_SW_Pin GPIO_PIN_2
#define CTRL_SW_GPIO_Port GPIOC
#define PWM_OUT_Pin GPIO_PIN_0
#define PWM_OUT_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOA
#define LED_V3_Pin GPIO_PIN_5
#define LED_V3_GPIO_Port GPIOA
#define LED_V2_Pin GPIO_PIN_6
#define LED_V2_GPIO_Port GPIOA
#define LED_V4_Pin GPIO_PIN_7
#define LED_V4_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOC
#define LED_C_Pin GPIO_PIN_5
#define LED_C_GPIO_Port GPIOC
#define LED_DP_Pin GPIO_PIN_0
#define LED_DP_GPIO_Port GPIOB
#define LED_D_Pin GPIO_PIN_1
#define LED_D_GPIO_Port GPIOB
#define LED_E_Pin GPIO_PIN_2
#define LED_E_GPIO_Port GPIOB
#define LED_F_Pin GPIO_PIN_10
#define LED_F_GPIO_Port GPIOB
#define LED_A_Pin GPIO_PIN_12
#define LED_A_GPIO_Port GPIOB
#define LED_V1_Pin GPIO_PIN_13
#define LED_V1_GPIO_Port GPIOB
#define RUN_SW_Pin GPIO_PIN_14
#define RUN_SW_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
