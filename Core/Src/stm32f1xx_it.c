/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "volumeControl.h"
#include "audioRelays.h"
#include "wavPlayer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern WavFileSelect wavFileSelect;
extern WavPlayerState wavPlayerState;
extern MiniJackPlcSignal miniJackPlcSignal;
extern Mic1PlcSignal mic1PlcSignal;
extern Mic2PlcSignal mic2PlcSignal;
extern MiniJackVolume miniJackVolume;
extern Mic1Volume mic1Volume;
extern Mic2Volume mic2Volume;
extern Mic1State mic1State;

extern uint8_t miniJackVolumeActual;
extern uint8_t mic1VolumeActual;
extern uint8_t mic2VolumeActual;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t fatFsCounter = 0;
volatile uint8_t mic1AndAlarmCounter = 0;
volatile uint8_t volumeControlCounter = 0;
volatile uint8_t Timer1, Timer2;
volatile uint8_t miniJackVolumeTimer = 0;
volatile uint8_t mic1VolumeTimer = 0;
volatile uint8_t mic2VolumeTimer = 0;
volatile uint16_t miniJackVolumeInfoTimer = 0;
volatile uint16_t mic1VolumeInfoTimer = 0;
volatile uint16_t mic2VolumeInfoTimer = 0;

void PlayAlarm_Handler(void)
{
	if (wavFileSelect == WAV_FILE_ALARM && HAL_GPIO_ReadPin(WAV_ALARM_GPIO_Port, WAV_ALARM_Pin) == GPIO_PIN_SET)
	{
		// Change file and state to idle to stop playing alarm
		wavFileSelect = WAV_FILE_START;
		wavPlayerState = WAV_STATE_IDLE;
	}
}

void Mic1_Handler(void)
{
	switch (mic1State)
	{
	case MIC1_STARTUP:
		RelaySPKR2(MIC1);
		mic1State = MIC1_ON;
		break;

	case MIC1_ON:
		if (HAL_GPIO_ReadPin(MIC1_SELECT_GPIO_Port, MIC1_SELECT_Pin) == GPIO_PIN_SET)
		{
			RelaySPKR2(JACK);
			mic1State = MIC1_OFF;
		}
		break;

	case MIC1_OFF:
		break;
	}
}

void VolumeControl_Handler(void)
{
	if (miniJackPlcSignal == JACK_VOLUME_PLC_ACTIVE)
	{
		miniJackVolumeTimer++;
		// If signal is ended - check duration and change volume
		if (HAL_GPIO_ReadPin(JACK_VOL_PLC_GPIO_Port, JACK_VOL_PLC_Pin) != GPIO_PIN_RESET)
		{
			// Volume DOWN if signal < 500 ms
			// Volume UP if signal > 500 ms
			miniJackVolume = (miniJackVolumeTimer <= 5) ? JACK_VOLUME_DOWN : JACK_VOLUME_UP;
			miniJackVolumeTimer = 0;
			miniJackPlcSignal = JACK_VOLUME_PLC_IDLE;
		}
	}
	if (mic1PlcSignal == MIC1_VOLUME_PLC_ACTIVE)
	{
		mic1VolumeTimer++;
		// If signal is ended - check duration and change volume
		if (HAL_GPIO_ReadPin(MIC1_VOL_PLC_GPIO_Port, MIC1_VOL_PLC_Pin) != GPIO_PIN_RESET)
		{
			// Volume DOWN if signal < 500 ms
			// Volume UP if signal > 500 ms
			mic1Volume = (mic1VolumeTimer <= 5) ? MIC1_VOLUME_DOWN : MIC1_VOLUME_UP;
			mic1VolumeTimer = 0;
			mic1PlcSignal = MIC1_VOLUME_PLC_IDLE;
		}
	}
	if (mic2PlcSignal == MIC2_VOLUME_PLC_ACTIVE)
	{
		mic2VolumeTimer++;
		// If signal is ended - check duration and change volume
		if (HAL_GPIO_ReadPin(MIC2_VOL_PLC_GPIO_Port, MIC2_VOL_PLC_Pin) != GPIO_PIN_RESET)
		{
			// Volume DOWN if signal < 500 ms
			// Volume UP if signal > 500 ms
			mic2Volume = (mic2VolumeTimer <= 5) ? MIC2_VOLUME_DOWN : MIC2_VOLUME_UP;
			mic2VolumeTimer = 0;
			mic2PlcSignal = MIC2_VOLUME_PLC_IDLE;
		}
	}
}

void SDTimer_Handler(void)
{
	if(Timer1 > 0)
		Timer1--;

	if(Timer2 > 0)
		Timer2--;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi2_tx;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	fatFsCounter++;
	mic1AndAlarmCounter++;
	volumeControlCounter++;
	miniJackVolumeInfoTimer++;
	mic1VolumeInfoTimer++;
	mic2VolumeInfoTimer++;

	if (fatFsCounter >= 10)
	{
		fatFsCounter = 0;
		SDTimer_Handler();
	}
	if (mic1AndAlarmCounter >= 50)
	{
		mic1AndAlarmCounter = 0;
		Mic1_Handler();
		PlayAlarm_Handler();
	}
	if (volumeControlCounter >= 100)
	{
		volumeControlCounter = 0;
		VolumeControl_Handler();
	}

	uint16_t miniJackVolumePulse = (miniJackVolumeActual == 0) ? 20 : miniJackVolumeActual * 20;
	if (miniJackVolumeInfoTimer >= 0)
	{
		HAL_GPIO_WritePin(JACK_VOL_INFO_GPIO_Port, JACK_VOL_INFO_Pin, GPIO_PIN_SET);
	}
	if (miniJackVolumeInfoTimer >= miniJackVolumePulse)
	{
		HAL_GPIO_WritePin(JACK_VOL_INFO_GPIO_Port, JACK_VOL_INFO_Pin, GPIO_PIN_RESET);
	}
	if (miniJackVolumeInfoTimer >= 2 * miniJackVolumePulse)
	{
		miniJackVolumeInfoTimer = 0;
	}

	uint16_t mic1VolumePulse = (mic1VolumeActual == 0) ? 20 : mic1VolumeActual * 20;
	if (mic1VolumeInfoTimer >= 0)
	{
		HAL_GPIO_WritePin(MIC1_VOL_INFO_GPIO_Port, MIC1_VOL_INFO_Pin, GPIO_PIN_SET);
	}
	if (mic1VolumeInfoTimer >= mic1VolumePulse)
	{
		HAL_GPIO_WritePin(MIC1_VOL_INFO_GPIO_Port, MIC1_VOL_INFO_Pin, GPIO_PIN_RESET);
	}
	if (mic1VolumeInfoTimer >= 2 * mic1VolumePulse)
	{
		mic1VolumeInfoTimer = 0;
	}

	uint16_t mic2VolumePulse = (mic2VolumeActual == 0) ? 20 : mic2VolumeActual * 20;
	if (mic2VolumeInfoTimer >= 0)
	{
		HAL_GPIO_WritePin(MIC2_VOL_INFO_GPIO_Port, MIC2_VOL_INFO_Pin, GPIO_PIN_SET);
	}
	if (mic2VolumeInfoTimer >= mic2VolumePulse)
	{
		HAL_GPIO_WritePin(MIC2_VOL_INFO_GPIO_Port, MIC2_VOL_INFO_Pin, GPIO_PIN_RESET);
	}
	if (mic2VolumeInfoTimer >= 2 * mic2VolumePulse)
	{
		mic2VolumeInfoTimer = 0;
	}

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(JACK_VOL_PLC_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(MIC1_VOL_PLC_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(MIC2_VOL_PLC_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(MIC1_SELECT_Pin);
  HAL_GPIO_EXTI_IRQHandler(WAV_ALARM_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(WAV_FINISH_Pin);
  HAL_GPIO_EXTI_IRQHandler(WAV_START_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
