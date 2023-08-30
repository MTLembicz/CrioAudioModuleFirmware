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
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERROR_LED_Pin GPIO_PIN_0
#define ERROR_LED_GPIO_Port GPIOA
#define JACK_VOL_INFO_Pin GPIO_PIN_1
#define JACK_VOL_INFO_GPIO_Port GPIOA
#define MIC1_VOL_INFO_Pin GPIO_PIN_2
#define MIC1_VOL_INFO_GPIO_Port GPIOA
#define MIC2_VOL_INFO_Pin GPIO_PIN_3
#define MIC2_VOL_INFO_GPIO_Port GPIOA
#define SD_CD_Pin GPIO_PIN_4
#define SD_CD_GPIO_Port GPIOC
#define SD_LED_Pin GPIO_PIN_5
#define SD_LED_GPIO_Port GPIOC
#define JACK_VOL_PLC_Pin GPIO_PIN_0
#define JACK_VOL_PLC_GPIO_Port GPIOB
#define JACK_VOL_PLC_EXTI_IRQn EXTI0_IRQn
#define MIC1_VOL_PLC_Pin GPIO_PIN_1
#define MIC1_VOL_PLC_GPIO_Port GPIOB
#define MIC1_VOL_PLC_EXTI_IRQn EXTI1_IRQn
#define MIC2_VOL_PLC_Pin GPIO_PIN_2
#define MIC2_VOL_PLC_GPIO_Port GPIOB
#define MIC2_VOL_PLC_EXTI_IRQn EXTI2_IRQn
#define STATUS_LED_Pin GPIO_PIN_11
#define STATUS_LED_GPIO_Port GPIOB
#define SPI3_CS4_Pin GPIO_PIN_14
#define SPI3_CS4_GPIO_Port GPIOB
#define DAC_LED_Pin GPIO_PIN_7
#define DAC_LED_GPIO_Port GPIOC
#define MIC1_SELECT_Pin GPIO_PIN_8
#define MIC1_SELECT_GPIO_Port GPIOA
#define MIC1_SELECT_EXTI_IRQn EXTI9_5_IRQn
#define WAV_ALARM_Pin GPIO_PIN_9
#define WAV_ALARM_GPIO_Port GPIOA
#define WAV_ALARM_EXTI_IRQn EXTI9_5_IRQn
#define WAV_START_Pin GPIO_PIN_10
#define WAV_START_GPIO_Port GPIOA
#define WAV_START_EXTI_IRQn EXTI15_10_IRQn
#define STATUS_Pin GPIO_PIN_11
#define STATUS_GPIO_Port GPIOA
#define VOL_CTRL_SELECT_Pin GPIO_PIN_12
#define VOL_CTRL_SELECT_GPIO_Port GPIOA
#define MIC1_VOL_CS_Pin GPIO_PIN_10
#define MIC1_VOL_CS_GPIO_Port GPIOC
#define MIC2_VOL_CS_Pin GPIO_PIN_11
#define MIC2_VOL_CS_GPIO_Port GPIOC
#define JACK_VOL_CS_Pin GPIO_PIN_12
#define JACK_VOL_CS_GPIO_Port GPIOC
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
