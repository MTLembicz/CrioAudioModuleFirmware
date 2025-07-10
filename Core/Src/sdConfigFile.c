/*
 * sdConfigFile.c
 *
 *  Created on: 29 sie 2023
 *      Author: Teo
 */

#include "sdConfigFile.h"
#include "fatfs.h"
#include "version.h"

char miniJackVolumeString[] = "Mini-jack volume";
char mic1VolumeString[] = "Mic1 volume";
char mic2VolumeString[] = "Mic2 volume";
char dacVolumeString[] = "Info signals volume";
char separator = ':';

extern FATFS fatFs;			// file system from wavPlayer
extern FIL configFile;		// files
extern FRESULT fresult;		// to store the result

extern uint8_t miniJackVolumeActual;
extern uint8_t mic1VolumeActual;
extern uint8_t mic2VolumeActual;
uint8_t dacVolume = 75;

bool sdConfigFile_saveFirmwareVersion()
{
	if (f_open(&configFile, "config.txt", FA_WRITE) == FR_OK)
	{
		char firmwareVersionString[25];
		snprintf(firmwareVersionString, sizeof(firmwareVersionString), "Firmware version: %s\n", MAIN_VERSION);

		UINT bw;
		f_write(&configFile, firmwareVersionString, (UINT)sizeof(firmwareVersionString), &bw);
		f_close(&configFile);
		return true;
	}
	else
	{
		return false;
	}
}

bool sdConfigFile_readVolume()
{
	if (f_open(&configFile, "config.txt", FA_READ) == FR_OK)
	{
		char line[50];

		/* Read file line by line */
		while (f_gets(line, sizeof(line), &configFile) != 0)
		{
			/* Look for Mini-jack volume */
			if (strstr(line, miniJackVolumeString))
			{
				char * p = strchr(line, separator);
				// Look for terminating character
				// If 3 positions above ':' is not '\0' - volume has 2 digits
				if (*(p + 3) != '\0')
				{
					char volumeString[3];
					strncpy(volumeString, p + 2, 2);
					miniJackVolumeActual = atoi(volumeString);
				}
				// If there is '\0' - volume has 1 digit
				else
				{
					char * p = strchr(line, separator);
					char volumeChar = *(p + 2);
					// convert ASCI character to int
					miniJackVolumeActual = volumeChar - '0';
				}
			}

			/* Look for Mic1 volume */
			if (strstr(line, mic1VolumeString))
			{
				char * p = strchr(line, separator);
				// Look for terminating character
				// If 3 positions above ':' is not '\0' - volume has 2 digits
				if (*(p + 3) != '\0')
				{
					char volumeString[3];
					strncpy(volumeString, p + 2, 2);
					mic1VolumeActual = atoi(volumeString);
				}
				// If there is '\0' - volume has 1 digit
				else
				{
					char * p = strchr(line, separator);
					char volumeChar = *(p + 2);
					// convert ASCI character to int
					mic1VolumeActual = volumeChar - '0';
				}
			}

			/* Look for Mic2 volume */
			if (strstr(line, mic2VolumeString))
			{
				char * p = strchr(line, separator);
				// Look for terminating character
				// If 3 positions above ':' is not '\0' - volume has 2 digits
				if (*(p + 3) != '\0')
				{
					char volumeString[3];
					strncpy(volumeString, p + 2, 2);
					mic2VolumeActual = atoi(volumeString);
				}
				// If there is '\0' - volume has 1 digit
				else
				{
					char * p = strchr(line, separator);
					char volumeChar = *(p + 2);
					// convert ASCI character to int
					mic2VolumeActual = volumeChar - '0';
				}
			}

			/* Look for DAC volume */
			if (strstr(line, dacVolumeString))
			{
				char * p = strchr(line, separator);
				// Look for terminating character
				// If 3 positions above ':' is not '\0' - volume has 2 digits
				if (*(p + 3) != '\0')
				{
					char volumeString[3];
					strncpy(volumeString, p + 2, 2);
					dacVolume = atoi(volumeString);
				}
				// If there is '\0' - volume has 1 digit
				else
				{
					char * p = strchr(line, separator);
					char volumeChar = *(p + 2);
					// convert ASCI character to int
					dacVolume = volumeChar - '0';
				}
			}
		}
		f_close(&configFile);
		return true;
	}
	else
	{
		return false;
	}
}

bool sdConfigFile_saveFullConfig()
{
	/* Function override config file */
	if (f_open(&configFile, "config.txt", (FA_WRITE)) == FR_OK)
	{
		UINT bw;

		// write firmware version
		char firmwareVersionString[30];
		int firmwareVersionStringLength = sprintf(firmwareVersionString, "Firmware version: %s\n", MAIN_VERSION);
		f_write(&configFile, firmwareVersionString, firmwareVersionStringLength, &bw);

		// write mini-jack volume
		char volumeString[30];
		int volumeStringLength = sprintf(volumeString, "%s%c %d\n", miniJackVolumeString, separator, miniJackVolumeActual);
		f_write(&configFile, volumeString, volumeStringLength, &bw);

		// write mic1 volume
		volumeStringLength = sprintf(volumeString, "%s%c %d\n", mic1VolumeString, separator, mic1VolumeActual);
		f_write(&configFile, volumeString, volumeStringLength, &bw);

		// write mic2 volume
		volumeStringLength = sprintf(volumeString, "%s%c %d\n", mic2VolumeString, separator, mic2VolumeActual);
		f_write(&configFile, volumeString, volumeStringLength, &bw);

		// write dac volume
		volumeStringLength = sprintf(volumeString, "%s%c %d\n", dacVolumeString, separator, dacVolume);
		f_write(&configFile, volumeString, volumeStringLength, &bw);

		if (f_close(&configFile) == FR_OK)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}
