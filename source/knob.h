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
#define KNOB_GPIO GPIO2

#define KNOB_A_GPIO_A 26
#define KNOB_A_GPIO_B 27

#define KNOB_B_GPIO_A 25
#define KNOB_B_GPIO_B 24

#define KNOB_C_GPIO_A 23
#define KNOB_C_GPIO_B 22

// 
typedef enum {
    CW = -1,
    CCW = 1,
} encoder_state_t;


typedef struct encoder {
	GPIO_Type *gpio;
	uint8_t pin_a;
	uint8_t pin_b;
	int last_edge_ms;
    uint8_t last_state;
    bool pending;
} encoder_t;

//encoder_t encoder_a;
//encoder_t encoder_b;
//encoder_t encoder_c;

#endif /* KNOB_H_ */
