/*
 * timer.h
 *
 *  Created on: Aug 20, 2025
 *      Author: mcaplan
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "fsl_gpt.h"

void TIMER_Init(void);

int TIMER_GetCurrentUs(void);


#endif /* TIMER_H_ */
