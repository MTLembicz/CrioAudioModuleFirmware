/*
 * volumeControl.c
 *
 *  Created on: May 1, 2023
 *      Author: Teo
 */

#include "volumeControl.h"
#include "audioRelays.h"
#include "sdConfigFile.h"

VolumeControlState miniJackVolumeState;
VolumeControlState mic1VolumeState;
VolumeControlState mic2VolumeState;

extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

uint8_t uPotRegisterAddress = 0x00;
uint8_t miniJackVolumeActual = 25;
uint8_t mic1VolumeActual = 25;
uint8_t mic2VolumeActual = 25;

void MiniJackVolumeProcess(void)
{
	// Set mini-jack volume if potentiometer is active
	if (HAL_GPIO_ReadPin(VOL_CTRL_SELECT_GPIO_Port, VOL_CTRL_SELECT_Pin) == GPIO_PIN_RESET)
	{
		uint32_t miniJackAdcVolume = 0;
		
		if (HAL_ADC_PollForConversion(&hadc1, 40) == HAL_OK)
		{
			miniJackAdcVolume = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Start(&hadc1);
		}
		else
		{
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
		}
		
		miniJackAdcVolume = ConvertAdcValue(miniJackAdcVolume);

		if (abs(miniJackAdcVolume - miniJackVolumeActual) > 1)
		{
			miniJackVolumeActual = miniJackAdcVolume;
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			uint8_t sendData[2] = {uPotRegisterAddress, miniJackVolumeActual};
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) != HAL_OK)
			{
				HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
	}
	// Set mini-jack volume based on PLC signals
	else
	{
		/*
		LM1971 data register - attenuation:
		0x00 - no attenuation,
		0x03 (dec 3) - 3dB,
		0x13 (dec 19) - 19dB,
		0x3E (dec 62) - 62 dB,
		0x3F - mute
		*/

		switch(miniJackVolumeState)
		{
		case VOLUME_UP:
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
					miniJackVolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				miniJackVolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_DOWN:
			if (miniJackVolumeActual < 63)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				miniJackVolumeActual++;
				uint8_t sendData[2] = {uPotRegisterAddress, miniJackVolumeActual};
				HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					miniJackVolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				miniJackVolumeState = VOLUME_IDLE;
				RelaySPKR2(NO_RELAY);
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_MIN:
			if (miniJackVolumeActual != 63)
			{
				RelaySPKR2(NO_RELAY);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				miniJackVolumeActual = 63;
				uint8_t sendData[2] = {uPotRegisterAddress, miniJackVolumeActual};
				HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					miniJackVolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				miniJackVolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_MAX:
			RelaySPKR2(JACK);
			if (miniJackVolumeActual > 0)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				miniJackVolumeActual = 0;
				uint8_t sendData[2] = {uPotRegisterAddress, miniJackVolumeActual};
				HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					miniJackVolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				miniJackVolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_INIT:
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			uint8_t sendData[2] = {uPotRegisterAddress, miniJackVolumeActual};
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				miniJackVolumeState = VOLUME_IDLE;
			}
			else
			{
				HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(JACK_VOL_CS_GPIO_Port, JACK_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_IDLE:
			break;
		}
	}
}

void Mic1VolumeProcess(void)
{
	// Set mic1 volume if potentiometer is active
	if (HAL_GPIO_ReadPin(VOL_CTRL_SELECT_GPIO_Port, VOL_CTRL_SELECT_Pin) == GPIO_PIN_RESET)
	{
		uint32_t mic1AdcVolume = 0;

		if (HAL_ADC_PollForConversion(&hadc2, 40) == HAL_OK)
		{
			mic1AdcVolume = HAL_ADC_GetValue(&hadc2);
			HAL_ADC_Start(&hadc2);
		}
		else
		{
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
		}

		mic1AdcVolume = (uint8_t)(mic1AdcVolume / 65);

		if (abs(mic1AdcVolume - mic1VolumeActual) > 1)
		{
			mic1VolumeActual = mic1AdcVolume;
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
			HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) != HAL_OK)
			{
				HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
	}
	// Set mic1 volume based on PLC signals
	else
	{
		/*
		LM1971 data register - attentuation:
		0x00 - no attenuation,
		0x03 (dec 3) - 3dB,
		0x13 (dec 19) - 19dB,
		0x3E (dec 62) - 62 dB,
		0x3F - mute
		*/

		switch(mic1VolumeState)
		{
		case VOLUME_UP:
			if (mic1VolumeActual > 0)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				mic1VolumeActual--;
				uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					mic1VolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				mic1VolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_DOWN:
			if (mic1VolumeActual < 63)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				mic1VolumeActual++;
				uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					mic1VolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				mic1VolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_MIN:
			if (mic1VolumeActual != 63)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				mic1VolumeActual = 63;
				uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					mic1VolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				mic1VolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_MAX:
			if (mic1VolumeActual > 0)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				mic1VolumeActual = 0;
				uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					mic1VolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				mic1VolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_INIT:
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			uint8_t sendData[2] = {uPotRegisterAddress, mic1VolumeActual};
			HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				mic1VolumeState = VOLUME_IDLE;
			}
			else
			{
				HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_IDLE:
			break;
		}
	}
}

void Mic2VolumeProcess(void)
{
	// Set mic2 volume if potentiometer is active
	if (HAL_GPIO_ReadPin(VOL_CTRL_SELECT_GPIO_Port, VOL_CTRL_SELECT_Pin) == GPIO_PIN_RESET)
	{
		uint32_t mic2AdcVolume = 0;

		if (HAL_ADC_PollForConversion(&hadc3, 40) == HAL_OK)
		{
			mic2AdcVolume = HAL_ADC_GetValue(&hadc3);
			HAL_ADC_Start(&hadc3);
		}
		else
		{
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
		}

		mic2AdcVolume = (uint8_t)(mic2AdcVolume / 65);

		if (abs(mic2AdcVolume - mic2VolumeActual) > 1)
		{
			mic2VolumeActual = mic2AdcVolume;
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
			HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) != HAL_OK)
			{
				HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
		}
	}
	// Set mic2 volume based on PLC signals
	else
	{
		/*
		LM1971 data register - attentuation:
		0x00 - no attenuation,
		0x03 (dec 3) - 3dB,
		0x13 (dec 19) - 19dB,
		0x3E (dec 62) - 62 dB,
		0x3F - mute
		*/

		switch(mic2VolumeState)
		{
		case VOLUME_UP:
			if (mic2VolumeActual > 0)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				mic2VolumeActual--;
				uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
				HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					mic2VolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				mic2VolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_DOWN:
			if (mic2VolumeActual < 63)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				mic2VolumeActual++;
				uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
				HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					mic2VolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				mic2VolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_MIN:
			if (mic2VolumeActual != 63)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				mic2VolumeActual = 63;
				uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
				HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					mic2VolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(MIC1_VOL_CS_GPIO_Port, MIC1_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				mic2VolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_MAX:
			if (mic2VolumeActual > 0)
			{
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
				mic2VolumeActual = 0;
				uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
				HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
				{
					mic2VolumeState = VOLUME_IDLE;
				}
				while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
				HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			}
			else
			{
				mic2VolumeState = VOLUME_IDLE;
			}
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_INIT:
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
			uint8_t sendData[2] = {uPotRegisterAddress, mic2VolumeActual};
			HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_RESET);
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500) == HAL_OK)
			{
				mic2VolumeState = VOLUME_IDLE;
			}
			else
			{
				HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
			}
			while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(MIC2_VOL_CS_GPIO_Port, MIC2_VOL_CS_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
			sdConfigFile_saveFullConfig();
			break;

		case VOLUME_IDLE:
			break;
		}
	}
}

uint8_t ConvertAdcValue(uint32_t adcValue)
{
	/*
	 * 12-bit ADC = 4095 max value
	 * Min voltage = 0,09 V = 0,09 * 4095 = 369
	 * Max voltage = 2,88 V = 3,3 V / 1,146 (1.24 is based on experiments)
	 * 4095 / 63 attenuation levels = 65
	 */
	if (adcValue < 369)
	{
		adcValue = 0;
	}

	uint8_t convertedValue = (uint8_t)(adcValue * 1.24 / 65);

	if (convertedValue > 63)
	{
		convertedValue = 63;
	}
	return convertedValue;
}

