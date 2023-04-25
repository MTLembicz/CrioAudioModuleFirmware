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
#define ERROR_LED_Pin GPIO_PIN_0
#define ERROR_LED_GPIO_Port GPIOA
#define SD_LED_Pin GPIO_PIN_5
#define SD_LED_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_11
#define STATUS_LED_GPIO_Port GPIOB
#define DAC_LED_Pin GPIO_PIN_7
#define DAC_LED_GPIO_Port GPIOC
#define SPKR1_MIC2_RLY_Pin GPIO_PIN_4
#define SPKR1_MIC2_RLY_GPIO_Port GPIOB
#define SPKR1_DAC_RLY_Pin GPIO_PIN_6
#define SPKR1_DAC_RLY_GPIO_Port GPIOB
#define SPKR2_DAC_RLY_Pin GPIO_PIN_7
#define SPKR2_DAC_RLY_GPIO_Port GPIOB
#define SPKR2_JACK_RLY_Pin GPIO_PIN_8
#define SPKR2_JACK_RLY_GPIO_Port GPIOB
#define SPKR2_MIC1_RLY_Pin GPIO_PIN_9
#define SPKR2_MIC1_RLY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
