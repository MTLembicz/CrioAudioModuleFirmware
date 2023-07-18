/*
 * statusInfo.h
 *
 *  Created on: Jul 5, 2023
 *      Author: Teo
 */

#ifndef INC_STATUSINFO_H_
#define INC_STATUSINFO_H_

#include "main.h"

typedef enum {
	OK = 0,
	NO_SD,
	ERROR_SD,
}SDcardStatus;

void StatusLedProcess(void);
void StatusPinProcess(void);

#endif /* INC_STATUSINFO_H_ */
