/*
 * utils.c
 *
 *  Created on: Jun 9, 2025
 *      Author: mcaplan
 */
#include "utils.h"

//uint8_t g_systickCounter = 0U;
//
//extern void SysTick_Handler(void)
//{
//   g_systickCounter++;
//}

void simpleDelay(uint8_t ms){

	uint32_t msToUs = ms*1000;

	SDK_DelayAtLeastUs(msToUs, SystemCoreClock);

//	//config for ms
//	SysTick_Config(SystemCoreClock / 1000U);
//
//
//	/* Enable Systick which might be disabled by system init */
//    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 1U)
//    {
//        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
//    }
//
//	uint8_t countNow = g_systickCounter;
//
//
//	while (g_systickCounter < (countNow + ms)){}
}
