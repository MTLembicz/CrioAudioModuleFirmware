/*
 * volumeControl.h
 *
 *  Created on: May 1, 2023
 *      Author: Teo
 */

#ifndef INC_VOLUMECONTROL_H_
#define INC_VOLUMECONTROL_H_

#include "main.h"
#include "stm32f1xx_hal.h"

typedef enum {
	VOLUME_INIT = 0,
	VOLUME_IDLE,
	VOLUME_UP,
	VOLUME_DOWN,
	VOLUME_MIN,
	VOLUME_MAX,
}VolumeControlState;

typedef enum {
	VOLUME_PLC_IDLE = 0,
	VOLUME_PLC_ACTIVE,
}VolumeControlPlcSignal;

void MiniJackVolumeProcess(void);
void Mic1VolumeProcess(void);
void Mic2VolumeProcess(void);
uint8_t ConvertAdcValue(uint32_t adcValue);

#endif /* INC_VOLUMECONTROL_H_ */
