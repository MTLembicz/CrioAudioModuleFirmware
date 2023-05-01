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
}MiniJackVolume;

typedef enum {
	JACK_VOLUME_PLC_IDLE = 0,
	JACK_VOLUME_PLC_ACTIVE,
}MiniJackPlcSignal;

/*
 * TODO
struct MiniJackVolumeStruct
{

};
*/

void MiniJackVolumeProcess(void);

#endif /* INC_VOLUMECONTROL_H_ */
