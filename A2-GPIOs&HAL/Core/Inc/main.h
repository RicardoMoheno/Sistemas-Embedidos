/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define switchBit0_Pin GPIO_PIN_0
#define switchBit0_GPIO_Port GPIOA
#define switchBit1_Pin GPIO_PIN_1
#define switchBit1_GPIO_Port GPIOA
#define switchBit2_Pin GPIO_PIN_2
#define switchBit2_GPIO_Port GPIOA
#define switchBit3_Pin GPIO_PIN_3
#define switchBit3_GPIO_Port GPIOA
#define dispPinB_Pin GPIO_PIN_4
#define dispPinB_GPIO_Port GPIOA
#define dispPinA_Pin GPIO_PIN_5
#define dispPinA_GPIO_Port GPIOA
#define dispPinF_Pin GPIO_PIN_6
#define dispPinF_GPIO_Port GPIOA
#define dispPinG_Pin GPIO_PIN_7
#define dispPinG_GPIO_Port GPIOA
#define dispPinE_Pin GPIO_PIN_8
#define dispPinE_GPIO_Port GPIOA
#define dispPinD_Pin GPIO_PIN_9
#define dispPinD_GPIO_Port GPIOA
#define dispPinC_Pin GPIO_PIN_10
#define dispPinC_GPIO_Port GPIOA
#define dispPinPD_Pin GPIO_PIN_11
#define dispPinPD_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
