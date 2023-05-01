/*
 * volumeControl.c
 *
 *  Created on: May 1, 2023
 *      Author: Teo
 */

#include "volumeControl.h"

MiniJackVolume miniJackVolume;

extern SPI_HandleTypeDef hspi3;

uint8_t uPotRegisterAddress = 0x00;
uint8_t miniJackVolumeActual = 0;

void MiniJackVolumeProcess(void)
{
	/*
	LM1971 data register - attentuation:
	0x00 - no attenuation,
	0x03 (dec 3) - 3dB,
	0x13 (dec 19) - 19dB,
	0x3E (dec 62) - 62 dB,
	0x3F - mute
	*/

	switch(miniJackVolume)
	{
	case JACK_VOLUME_UP:
		if (miniJackVolumeActual > 0)
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			miniJackVolumeActual--;
			uint8_t sendData[2] = {uPotRegisterAddress, miniJackVolumeActual};
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				miniJackVolume = JACK_VOLUME_IDLE;
			}
			HAL_GPIO_WritePin(GPIOC, JACK_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
		else
		{
			miniJackVolume = JACK_VOLUME_IDLE;
		}
		break;

	case JACK_VOLUME_DOWN:
		if (miniJackVolumeActual < 67)
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			miniJackVolumeActual++;
			uint8_t sendData[2] = {uPotRegisterAddress, miniJackVolumeActual};
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				miniJackVolume = JACK_VOLUME_IDLE;
			}
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
		else
		{
			miniJackVolume = JACK_VOLUME_IDLE;
		}
		break;

	case JACK_VOLUME_INIT:
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
		uint8_t sendData[2] = {uPotRegisterAddress, miniJackVolumeActual};
		HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_RESET);
		while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
		if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
		{
			miniJackVolume = JACK_VOLUME_IDLE;
		}
		HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		break;

	case JACK_VOLUME_IDLE:
		break;
	}
}
