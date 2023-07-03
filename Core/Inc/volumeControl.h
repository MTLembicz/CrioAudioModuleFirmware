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
	JACK_VOLUME_INIT = 0,
	JACK_VOLUME_IDLE,
	JACK_VOLUME_UP,
	JACK_VOLUME_DOWN,
	JACK_VOLUME_MIN,
	JACK_VOLUME_MAX,
}MiniJackVolume;

typedef enum {
	MIC1_VOLUME_INIT = 0,
	MIC1_VOLUME_IDLE,
	MIC1_VOLUME_UP,
	MIC1_VOLUME_DOWN,
	MIC1_VOLUME_MIN,
	MIC1_VOLUME_MAX,
}Mic1Volume;

typedef enum {
	MIC2_VOLUME_INIT = 0,
	MIC2_VOLUME_IDLE,
	MIC2_VOLUME_UP,
	MIC2_VOLUME_DOWN,
	MIC2_VOLUME_MIN,
	MIC2_VOLUME_MAX,
}Mic2Volume;

typedef enum {
	JACK_VOLUME_PLC_IDLE = 0,
	JACK_VOLUME_PLC_ACTIVE,
}MiniJackPlcSignal;

typedef enum {
	MIC1_VOLUME_PLC_IDLE = 0,
	MIC1_VOLUME_PLC_ACTIVE,
}Mic1PlcSignal;

typedef enum {
	MIC2_VOLUME_PLC_IDLE = 0,
	MIC2_VOLUME_PLC_ACTIVE,
}Mic2PlcSignal;


/*
 * TODO
struct MiniJackVolumeStruct
{

};
*/

void MiniJackVolumeProcess(void);
void Mic1VolumeProcess(void);
void Mic2VolumeProcess(void);

#endif /* INC_VOLUMECONTROL_H_ */
