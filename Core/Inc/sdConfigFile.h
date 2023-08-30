/*
 * sdConfigFile.h
 *
 *  Created on: 29 sie 2023
 *      Author: Teo
 */

#ifndef INC_SDCONFIGFILE_H_
#define INC_SDCONFIGFILE_H_

#include "main.h"
#include "stm32f1xx_hal.h"

bool sdConfigFile_saveFirmwareVersion();
bool sdConfigFile_readVolume();
bool sdConfigFile_saveFullConfig();

#endif /* INC_SDCONFIGFILE_H_ */
