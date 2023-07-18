/*
 * statusInfo.c
 *
 *  Created on: Jul 5, 2023
 *      Author: Teo
 */

#include "statusInfo.h"
#include "wavPlayer.h"

SDcardStatus sdCardStatus;

volatile uint8_t ledCounter = 0;
volatile uint8_t pinCounter = 0;
uint8_t statusPinPulse = 0;

extern WavPlayerState wavPlayerState;

void StatusLedProcess(void)
{
	if (ledCounter > 10)
	{
		ledCounter = 0;
	}
	switch(sdCardStatus)
	{
		case OK:
			HAL_GPIO_WritePin(SD_LED_GPIO_Port, SD_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
			if (wavPlayerState != WAV_STATE_PLAY)
			{
				HAL_GPIO_WritePin(DAC_LED_GPIO_Port, DAC_LED_Pin, GPIO_PIN_SET);
			}
			break;

		case NO_SD:
			HAL_GPIO_WritePin(SD_LED_GPIO_Port, SD_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
			if (wavPlayerState != WAV_STATE_PLAY)
			{
				HAL_GPIO_WritePin(DAC_LED_GPIO_Port, DAC_LED_Pin, GPIO_PIN_SET);
			}
			break;

		case ERROR_SD:
			if (ledCounter >= 5)
			{
				HAL_GPIO_WritePin(SD_LED_GPIO_Port, SD_LED_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
			}
			else
			{
				if (ledCounter >= 0)
				{
					HAL_GPIO_WritePin(SD_LED_GPIO_Port, SD_LED_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
				}
			}
			break;
	}
	switch (wavPlayerState)
	{
		case WAV_STATE_ERROR:
			if (ledCounter >= 5)
			{
					HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(DAC_LED_GPIO_Port, DAC_LED_Pin, GPIO_PIN_RESET);
			}
			else
			{
				if (ledCounter >= 0)
				{
					HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(DAC_LED_GPIO_Port, DAC_LED_Pin, GPIO_PIN_SET);
				}
			}
			break;

		case WAV_STATE_PLAY:
			HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DAC_LED_GPIO_Port, DAC_LED_Pin, GPIO_PIN_RESET);
			break;

		default:
			HAL_GPIO_WritePin(DAC_LED_GPIO_Port, DAC_LED_Pin, GPIO_PIN_SET);
	}

	ledCounter++;
}

void StatusPinProcess(void)
{
	// choose pulse length in ms depending on board status
	switch (sdCardStatus)
	{
	case OK:
		statusPinPulse = 50;
		break;

	case NO_SD:
		statusPinPulse = 100;
		break;

	case ERROR_SD:
		statusPinPulse = 150;
		break;
	}
	if (wavPlayerState == WAV_STATE_ERROR)
	{
		statusPinPulse = 200;
	}

	// clear counter if pulse sequence is finished (LOW + HIGH)
	if (pinCounter * 10 >= statusPinPulse * 2)
	{
		pinCounter = 0;
	}

	// generate HIGH and LOW pulse
	// one counter is equal to 10 ms
	if (pinCounter * 10 >= statusPinPulse)
	{
		HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_SET);

	}
	pinCounter++;
}
