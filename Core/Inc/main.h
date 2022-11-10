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
#include "stm32f0xx_hal.h"

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
#define SA_Pin GPIO_PIN_0
#define SA_GPIO_Port GPIOA
#define SB_Pin GPIO_PIN_1
#define SB_GPIO_Port GPIOA
#define SC_Pin GPIO_PIN_2
#define SC_GPIO_Port GPIOA
#define SD_Pin GPIO_PIN_3
#define SD_GPIO_Port GPIOA
#define Se_Pin GPIO_PIN_4
#define Se_GPIO_Port GPIOA
#define STATUS_LED_Pin GPIO_PIN_5
#define STATUS_LED_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_6
#define B1_GPIO_Port GPIOA
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define B2_Pin GPIO_PIN_7
#define B2_GPIO_Port GPIOA
#define B2_EXTI_IRQn EXTI4_15_IRQn
#define B3_Pin GPIO_PIN_1
#define B3_GPIO_Port GPIOB
#define B3_EXTI_IRQn EXTI0_1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
