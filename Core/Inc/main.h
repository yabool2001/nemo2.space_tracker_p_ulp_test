/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define ACC_CS_Pin GPIO_PIN_4
#define ACC_CS_GPIO_Port GPIOA
#define ACC_SCK_Pin GPIO_PIN_5
#define ACC_SCK_GPIO_Port GPIOA
#define ACC_MISO_Pin GPIO_PIN_6
#define ACC_MISO_GPIO_Port GPIOA
#define ACC_MOSI_Pin GPIO_PIN_7
#define ACC_MOSI_GPIO_Port GPIOA
#define ASTRO_RST_Pin GPIO_PIN_1
#define ASTRO_RST_GPIO_Port GPIOB
#define ASTRO_EVT_Pin GPIO_PIN_2
#define ASTRO_EVT_GPIO_Port GPIOB
#define GNSS_RST_Pin GPIO_PIN_13
#define GNSS_RST_GPIO_Port GPIOB
#define GNSS_PWR_SW_Pin GPIO_PIN_15
#define GNSS_PWR_SW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
