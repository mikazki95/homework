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
// Generales
typedef enum {
	false = 0, true = 1
} bool;
typedef bool bolean;

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
#define TEST_USART_Pin GPIO_PIN_13
#define TEST_USART_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define PIN_OUT_INTERR_Pin GPIO_PIN_0
#define PIN_OUT_INTERR_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define AFE_PWDN_Pin GPIO_PIN_10
#define AFE_PWDN_GPIO_Port GPIOB
#define AFE_DIAG_END_Pin GPIO_PIN_11
#define AFE_DIAG_END_GPIO_Port GPIOB
#define AFE_PD_ALM_Pin GPIO_PIN_3
#define AFE_PD_ALM_GPIO_Port GPIOB
#define AFE_LED_ALM_Pin GPIO_PIN_4
#define AFE_LED_ALM_GPIO_Port GPIOB
#define AFE_DRDY_Pin GPIO_PIN_8
#define AFE_DRDY_GPIO_Port GPIOB
#define AFE_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define AFE_START_Pin GPIO_PIN_9
#define AFE_START_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
