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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/// Generales
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
int8_t findStr (const char *str1, const char *str2);
int32_t map (int32_t valor, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RGB_R1_Pin GPIO_PIN_2
#define RGB_R1_GPIO_Port GPIOE
#define RGB_B2_Pin GPIO_PIN_3
#define RGB_B2_GPIO_Port GPIOE
#define RGB_G2_Pin GPIO_PIN_4
#define RGB_G2_GPIO_Port GPIOE
#define RGB_R2_Pin GPIO_PIN_5
#define RGB_R2_GPIO_Port GPIOE
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI1_DRDY_Pin GPIO_PIN_4
#define SPI1_DRDY_GPIO_Port GPIOC
#define SPI1_DRDY_EXTI_IRQn EXTI4_IRQn
#define SPI1_START_Pin GPIO_PIN_5
#define SPI1_START_GPIO_Port GPIOC
#define ADS_RST_Pin GPIO_PIN_0
#define ADS_RST_GPIO_Port GPIOB
#define INTERUPT1_Pin GPIO_PIN_1
#define INTERUPT1_GPIO_Port GPIOB
#define INTERUPT1_EXTI_IRQn EXTI1_IRQn
#define IN_3_Pin GPIO_PIN_8
#define IN_3_GPIO_Port GPIOD
#define MK_Reset_Pin GPIO_PIN_0
#define MK_Reset_GPIO_Port GPIOD
#define IN_1_Pin GPIO_PIN_5
#define IN_1_GPIO_Port GPIOD
#define IN_4_Pin GPIO_PIN_6
#define IN_4_GPIO_Port GPIOD
#define IN_2_Pin GPIO_PIN_7
#define IN_2_GPIO_Port GPIOD
#define INT_BATT_Pin GPIO_PIN_9
#define INT_BATT_GPIO_Port GPIOB
#define INT_BATT_EXTI_IRQn EXTI9_5_IRQn
#define RGB_B1_Pin GPIO_PIN_0
#define RGB_B1_GPIO_Port GPIOE
#define RGB_G1_Pin GPIO_PIN_1
#define RGB_G1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define TASK_COUNT "9"
float filter(float cutofFreq);
int32_t DerivadaFuncion (int derivacion);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
