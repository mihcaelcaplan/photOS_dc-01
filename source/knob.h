/*
 * knob.h
 *
 *  Created on: Oct 16, 2025
 *      Author: mcaplan
 */

#ifndef KNOB_H_
#define KNOB_H_

#include "fsl_gpio.h"

// set up some knobs
#define KNOB_A_GPIO GPIO2
#define KNOB_A_GPIO_A 26
#define KNOB_A_GPIO_B 27

// 
typedef enum {
    CW = 0xA,
    CCW = 0xB,
} encoder_state_t;



#endif /* KNOB_H_ */
