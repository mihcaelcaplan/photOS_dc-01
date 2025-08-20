/*
 * timer.c
 *
 *  Created on: Aug 20, 2025
 *      Author: mcaplan
 */

#include "timer.h"
#include "fsl_gpt.h"


/*inspired by https://community.nxp.com/t5/i-MX-RT-Crossover-MCUs/imxrt-1050-Millisecond-ticks/m-p/1204336*/


void TIMER_Init(void){

	gpt_config_t gpt_init;

	GPT_GetDefaultConfig(&gpt_init);

	gpt_init.enableFreeRun = true;                // Free run
	gpt_init.enableMode  = false;                 // No reset of counter

	GPT_Init(GPT1, &gpt_init);
    GPT_SetClockSource(GPT1, kGPT_ClockSource_Osc); // 24 MHz
    GPT_SetClockDivider(GPT1, 24);                  // Prescaler = 24

	GPT_StartTimer(GPT1);                           // Start Timer
}

int TIMER_GetCurrentUs(void){
	return GPT_GetCurrentTimerCount(GPT1);
}
