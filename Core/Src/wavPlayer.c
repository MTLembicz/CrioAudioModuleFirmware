/*
 * wav_player.c
 *
 * Created on: 30 Mar 2023
 *     Author: Mat L
 */
#define SPI_TIMEOUT 500

#include <wavPlayer.h>
#include "fatfs.h"
#include <math.h>
#include "audioRelays.h"
#include "volumeControl.h"
#include "statusInfo.h"
#include "sdConfigFile.h"

FATFS fatFs;							// file system
FIL wavFile, logFile, configFile;		// files
FRESULT fresult;						// to store the result

UINT br, bw;

/* capacity related variables */
FATFS *pfatFs;
DWORD fre_clust;
uint32_t total, free_space;

UINT* nullptr = NULL;

uint8_t bufferState = 0;
uint32_t wavFileBytesReaded = 0;
uint32_t wavFileSize = 0;
int16_t audioBuffer[AUDIO_BUFFER_SIZE];

struct WavFilesInfo wavFileInfo;
WavPlayerState wavPlayerState;
WavFileSelect wavFileSelect;

extern SPI_HandleTypeDef hspi3;
extern Mic1State mic1State;
extern uint8_t miniJackVolumeActual;
extern uint8_t dacVolume;
extern SDcardStatus sdCardStatus;

// unused
void DACConfigureI2SFormat(SPI_HandleTypeDef *hspi)
{
	uint8_t dacRegisterAddress = 0x14;
	uint8_t dacRegisterData = 0x4;
	uint8_t sendData[2] = {dacRegisterAddress, dacRegisterData};

	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(GPIOB, SPI3_CS4_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 20) == HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOB, STATUS_LED_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(GPIOB, SPI3_CS4_Pin, GPIO_PIN_SET);
}

// volume value should be 0 to 99
// function called in WAVPlayerPlay
void DACSetVolume()
{
	/*
	 * PCM1870 register values:
	 * 128 = Mute, 129 = -63 dB, 254 = -0.5 dB, 255 = no attenuation
	 */
	uint8_t volume = dacVolume * 127 / 100 + 128;
	uint8_t dacRegisterAddress = 0x10;
	uint8_t dacRegisterData = volume;
	uint8_t sendData[2] = {dacRegisterAddress, dacRegisterData};

	HAL_GPIO_WritePin(GPIOB, SPI3_CS4_Pin, GPIO_PIN_RESET);
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500);
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(GPIOB, SPI3_CS4_Pin, GPIO_PIN_SET);
	HAL_Delay(10);

	dacRegisterAddress = 0x11;
	dacRegisterData = volume;
	sendData[0] = dacRegisterAddress;
	sendData[1] = dacRegisterData;

	HAL_GPIO_WritePin(GPIOB, SPI3_CS4_Pin, GPIO_PIN_RESET);
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi3, &sendData[0], 2, 500);
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(GPIOB, SPI3_CS4_Pin, GPIO_PIN_SET);
	// wait to apply setting
	HAL_Delay(10);
}

bool SDMount(void)
{
  /* Mount SD Card */
  if (f_mount(&fatFs, "", 0) == FR_OK)
  {
	  // SD card mounted successfully
	  // Check for config file

	  if (f_open(&configFile, "config.txt", FA_OPEN_ALWAYS) == FR_OK)
	  {
		  f_close(&configFile);
		  return true;
	  }
	  else
	  {
		  return false;
	  }

	  //return sdConfigFile_saveFirmwareVersion();
  }
  else
  {
	  // SD card mount failed
	  return false;
  }
}

bool SDUnmount(void)
{
	if (f_mount(0, "", 0) == FR_OK)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool WAVPlayerFileSelect(const char* filePath)
{
	UINT readBytes = 0;
	uint32_t chunkId;
	uint32_t chunkSize;
	uint32_t format;
	uint32_t subchunk1Id;
	uint32_t subchunk1Size;
	uint16_t audioFormat;
	uint16_t numChannels;
	uint32_t sampleRate;
	uint32_t byteRate;
	uint16_t blockAlign;
	uint16_t bitsPerSample;
	uint32_t subchunk2Id;
	uint32_t subchunk2Size;

	/* Open WAV file to read */
	if (f_open(&wavFile, filePath, FA_READ) != FR_OK)
	{
		wavPlayerState = WAV_STATE_ERROR;
		return false;
	}

	f_read(&wavFile, &chunkId, sizeof(chunkId), &readBytes);
	f_read(&wavFile, &chunkSize, sizeof(chunkSize), &readBytes);
	f_read(&wavFile, &format, sizeof(format), &readBytes);
	f_read(&wavFile, &subchunk1Id, sizeof(subchunk1Id), &readBytes);
	f_read(&wavFile, &subchunk1Size, sizeof(subchunk1Size), &readBytes);
	f_read(&wavFile, &audioFormat, sizeof(audioFormat), &readBytes);
	f_read(&wavFile, &numChannels, sizeof(numChannels), &readBytes);
	f_read(&wavFile, &sampleRate, sizeof(sampleRate), &readBytes);
	f_read(&wavFile, &byteRate, sizeof(byteRate), &readBytes);
	f_read(&wavFile, &blockAlign, sizeof(blockAlign), &readBytes);
	f_read(&wavFile, &bitsPerSample, sizeof(bitsPerSample), &readBytes);
	f_read(&wavFile, &subchunk2Id, sizeof(subchunk2Id), &readBytes);
	f_read(&wavFile, &subchunk2Size, sizeof(subchunk2Size), &readBytes);

	wavFileSize = subchunk2Size;

	//if(CHUNK_ID_CONST == wavFileInfo.chunkId && FORMAT_CONST == wavFileInfo.format && CHANNEL_STEREO == wavFileInfo.numChannels)
	if(CHUNK_ID_CONST == chunkId && FORMAT_CONST == format)
	{
		//SendStringUART("Header is correct. WAV file\r\n");
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		//HAL_Delay(1000);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else
	{
		wavPlayerState = WAV_STATE_ERROR;
		return false;
	}
	return true;
}

void WAVPlayerPlay(I2S_HandleTypeDef* i2s)
{
	DACSetVolume();
	if (wavPlayerState != WAV_STATE_ERROR)
	{
		wavFileBytesReaded = 0;

		/* Fill buffer first time */
		fresult = f_read(&wavFile, &audioBuffer, AUDIO_BUFFER_SIZE * 2, &br);
		/*
		if (fresult == FR_TIMEOUT)
		{
			HAL_GPIO_WritePin(GPIOC, DAC_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, ERROR_LED_Pin, GPIO_PIN_RESET);
		}
		if (fresult == FR_DENIED)
		{
			HAL_GPIO_WritePin(GPIOC, DAC_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, ERROR_LED_Pin, GPIO_PIN_RESET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOC, DAC_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, ERROR_LED_Pin, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOC, DAC_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, ERROR_LED_Pin, GPIO_PIN_RESET);
		}
		*/
		wavFileBytesReaded += br;

		// Start circular DMA
		RelaySPKR1(EXTERNAL_DAC);
		RelaySPKR2(EXTERNAL_DAC);
		HAL_I2S_Transmit_DMA(i2s, (uint16_t *)&audioBuffer, AUDIO_BUFFER_SIZE);
		//HAL_GPIO_WritePin(GPIOC, DAC_LED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		if (SDMount())
		{
			HAL_GPIO_WritePin(GPIOA, ERROR_LED_Pin, GPIO_PIN_SET);
			wavPlayerState = WAV_STATE_START;
		}

	}
}

void WAVPlayerBufferState(uint8_t bs)
{
	bufferState = bs;
}

void WAVPlayerProcess(I2S_HandleTypeDef* i2s)
{
	if (wavPlayerState == WAV_STATE_START)
	{
		/*
		// maybe needed after error and card change
		wavFileBytesReaded = 0;
		wavFileSize = 0;
		memset(audioBuffer, 0, sizeof audioBuffer);
		*/
		switch (wavFileSelect)
		{
		case WAV_FILE_START:
			WAVPlayerFileSelect("001_start_22khz.wav");
			break;

		case WAV_FILE_ALARM:
			WAVPlayerFileSelect("003_alarm_22khz.wav");
			break;
		}
		WAVPlayerPlay(i2s);
		wavPlayerState = WAV_STATE_PLAY;
	}

	if (wavPlayerState == WAV_STATE_PLAY)
	{
		if(wavFileBytesReaded >= wavFileSize)
		{
			HAL_I2S_DMAStop(i2s);
			WAVPlayerStopAndCloseFile();
		}

		if(bufferState == 1)
		{
			if (f_read(&wavFile, &audioBuffer[0], AUDIO_BUFFER_SIZE, &br) != FR_OK)
			{
				wavPlayerState = WAV_STATE_ERROR;
			}
			wavFileBytesReaded += br;
			bufferState = 0;
		}
		if(bufferState == 2)
		{
			if (f_read(&wavFile, &audioBuffer[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE, &br) != FR_OK)
			{
				wavPlayerState = WAV_STATE_ERROR;
			}
			wavFileBytesReaded += br;
			bufferState = 0;
		}
	}

	if (wavPlayerState == WAV_STATE_IDLE && mic1State == MIC1_OFF)
	{
		//HAL_GPIO_WritePin(GPIOC, DAC_LED_Pin, GPIO_PIN_SET);
		RelaySPKR1(MIC2);
		// Turn on JACK or NO_RELAY if mini-jack is muted
		if (miniJackVolumeActual != 63)
		{
			RelaySPKR2(JACK);
		}
		else
		{
			RelaySPKR2(NO_RELAY);
		}
	}
}

bool WAVPlayerStopAndCloseFile(void)
{
	if(f_close(&wavFile) == FR_OK)
	{
		//HAL_GPIO_WritePin(GPIOC, DAC_LED_Pin, GPIO_PIN_SET);
		if (wavFileSelect == WAV_FILE_ALARM)
		{
			wavPlayerState = WAV_STATE_START;
		}
		else
		{
			wavPlayerState = WAV_STATE_IDLE;
		}
		return true;
	}
	else
	{
		wavPlayerState = WAV_STATE_ERROR;
		RelaySPKR1(MIC2);
		// Turn on JACK or NO_RELAY if mini-jack is muted
		if (miniJackVolumeActual != 63)
		{
			RelaySPKR2(JACK);
		}
		else
		{
			RelaySPKR2(NO_RELAY);
		}
		return false;
	}
}
/*
void WAVPlayerFillBuffer(I2S_HandleTypeDef* i2s, bool bufferHalf)
{
	if (bufferHalf == false)
	{
		f_read(&wavFile, &audioBuffer[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE / 2, &br);
	}
	else if (bufferHalf == true)
	{
		f_read(&wavFile, &audioBuffer[0], AUDIO_BUFFER_SIZE / 2, &br);
	}
}
*/
void WAVPlayerFillBufferHalf(void)
{
	//f_read(&wavFile, &audioBuffer[0], AUDIO_BUFFER_SIZE, &br);
    for (int i = 0; i < AUDIO_BUFFER_SIZE / 4; i++)
    {
        int16_t value = (int16_t)(32000.0 * sin(2.0 * M_PI * i / 44.1));
        audioBuffer[i * 2] = value;
        audioBuffer[i * 2 + 1] = value;
    }
}
void WAVPlayerFillBufferFull(void)
{
	//f_read(&wavFile, &audioBuffer[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE, &br);
    for (int i = AUDIO_BUFFER_SIZE / 4; i < AUDIO_BUFFER_SIZE / 2; i++)
    {
        int16_t value = (int16_t)(32000.0 * sin(2.0 * M_PI * i / 44.1));
        audioBuffer[i * 2] = value;
        audioBuffer[i * 2 + 1] = value;
    }
}
