/*
 * utils.h
 *
 *  Created on: Jun 9, 2025
 *      Author: mcaplan
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "fsl_common.h"

// redefine the entry vector for the on tick interrupt to make g_systickCounter* 1 ms resolution timer
//void SysTick_Handler(void);

void simpleDelay(uint8_t ms);


#endif /* UTILS_H_ */
