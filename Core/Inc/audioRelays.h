/*
 * audioRelays.h
 *
 *  Created on: May 1, 2023
 *      Author: Teo
 */

#ifndef INC_AUDIORELAYS_H_
#define INC_AUDIORELAYS_H_

#include "main.h"
#include "stm32f1xx_hal.h"

typedef enum {
	JACK = 0,
	MIC1,
	MIC2,
	EXTERNAL_DAC,
	NO_RELAY,
}RelayInput;

typedef enum {
	MIC1_OFF = 0,
	MIC1_ON,
	MIC1_STARTUP,
}Mic1State;

void RelaySPKR1(RelayInput input);
void RelaySPKR2(RelayInput input);

#endif /* INC_AUDIORELAYS_H_ */
