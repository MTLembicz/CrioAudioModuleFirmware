/*
 * volumeControl.c
 *
 *  Created on: May 1, 2023
 *      Author: Teo
 */

#include "volumeControl.h"
#include "audioRelays.h"

MiniJackVolume miniJackVolume;
Mic1Volume mic1Volume;
Mic2Volume mic2Volume;

extern SPI_HandleTypeDef hspi3;

uint8_t uPotRegisterAddress = 0x00;
uint8_t miniJackVolumeActual = 0;
uint8_t mic1VolumeActual = 0;
uint8_t mic2VolumeActual = 0;

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
		RelaySPKR2(JACK);
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
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
		else
		{
			miniJackVolume = JACK_VOLUME_IDLE;
		}
		break;

	case JACK_VOLUME_DOWN:
		if (miniJackVolumeActual < 62)
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
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
		else
		{
			miniJackVolume = JACK_VOLUME_IDLE;
			RelaySPKR2(NO_RELAY);
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
		else
		{
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
		}
		while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		break;

	case JACK_VOLUME_IDLE:
		break;
	}
}

void Mic1VolumeProcess(void)
{
	/*
	LM1971 data register - attentuation:
	0x00 - no attenuation,
	0x03 (dec 3) - 3dB,
	0x13 (dec 19) - 19dB,
	0x3E (dec 62) - 62 dB,
	0x3F - mute
	*/

	switch(mic1Volume)
	{
	case MIC1_VOLUME_UP:
		if (mic1VolumeActual > 0)
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			mic1VolumeActual--;
			uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
			HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				mic1Volume = MIC1_VOLUME_IDLE;
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
		else
		{
			mic1Volume = MIC1_VOLUME_IDLE;
		}
		break;

	case MIC1_VOLUME_DOWN:
		if (mic1VolumeActual < 62)
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			mic1VolumeActual++;
			uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
			HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				mic1Volume = MIC1_VOLUME_IDLE;
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
		else
		{
			mic1Volume = MIC1_VOLUME_IDLE;
		}
		break;

	case MIC1_VOLUME_INIT:
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
		uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
		HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
		while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
		if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
		{
			mic1Volume = MIC1_VOLUME_IDLE;
		}
		else
		{
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
		}
		while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		break;

	case MIC1_VOLUME_IDLE:
		break;
	}
}

void Mic2VolumeProcess(void)
{
	/*
	LM1971 data register - attentuation:
	0x00 - no attenuation,
	0x03 (dec 3) - 3dB,
	0x13 (dec 19) - 19dB,
	0x3E (dec 62) - 62 dB,
	0x3F - mute
	*/

	switch(mic2Volume)
	{
	case MIC2_VOLUME_UP:
		if (mic2VolumeActual > 0)
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			mic2VolumeActual--;
			uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
			HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				mic2Volume = MIC2_VOLUME_IDLE;
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
		else
		{
			mic2Volume = MIC2_VOLUME_IDLE;
		}
		break;

	case MIC2_VOLUME_DOWN:
		if (mic2VolumeActual < 62)
		{
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			mic2VolumeActual++;
			uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
			HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				mic2Volume = MIC2_VOLUME_IDLE;
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
		else
		{
			mic2Volume = MIC2_VOLUME_IDLE;
		}
		break;

	case MIC2_VOLUME_INIT:
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
		uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
		HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
		while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
		if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
		{
			mic2Volume = MIC2_VOLUME_IDLE;
		}
		else
		{
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
		}
		while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		break;

	case MIC2_VOLUME_IDLE:
		break;
	}
}

