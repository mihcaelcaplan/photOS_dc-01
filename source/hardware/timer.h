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

void TIMER_ScheduleInterrupt(int interruptTime);
void TIMER_TurnOnInterrupts(void);


// GPT interrupt timing
#define GPT_INTERRUPT_PERIOD_US 1000

extern volatile int last_scheduled_interrupt;

#endif /* TIMER_H_ */
