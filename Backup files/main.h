/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

TIM_OC_InitTypeDef sConfigOC;

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
#define S2_D3_Pin GPIO_PIN_13
#define S2_D3_GPIO_Port GPIOC
#define S2_A1_Pin GPIO_PIN_0
#define S2_A1_GPIO_Port GPIOC
#define S1_A1_Pin GPIO_PIN_1
#define S1_A1_GPIO_Port GPIOC
#define S1_A2_Pin GPIO_PIN_0
#define S1_A2_GPIO_Port GPIOA
#define S1_A3_Pin GPIO_PIN_1
#define S1_A3_GPIO_Port GPIOA
#define S2_A0_Pin GPIO_PIN_2
#define S2_A0_GPIO_Port GPIOA
#define S1_A0_Pin GPIO_PIN_3
#define S1_A0_GPIO_Port GPIOA
#define LED_nR_Pin GPIO_PIN_7
#define LED_nR_GPIO_Port GPIOA
#define S1_D0_Pin GPIO_PIN_0
#define S1_D0_GPIO_Port GPIOA
#define S1_D1_Pin GPIO_PIN_1
#define S1_D1_GPIO_Port GPIOA
#define S1_D3_Pin GPIO_PIN_12
#define S1_D3_GPIO_Port GPIOB
#define LED_nG_Pin GPIO_PIN_14
#define LED_nG_GPIO_Port GPIOB
#define LED_nB_Pin GPIO_PIN_15
#define LED_nB_GPIO_Port GPIOB
#define S1_D2_Pin GPIO_PIN_11
#define S1_D2_GPIO_Port GPIOA
#define S2_D0_Pin GPIO_PIN_15
#define S2_D0_GPIO_Port GPIOA
#define S2_D2_Pin GPIO_PIN_7
#define S2_D2_GPIO_Port GPIOB
#define S2_D1_Pin GPIO_PIN_9
#define S2_D1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
