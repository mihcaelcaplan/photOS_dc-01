/*
 * state_adjust.h
 *
 *  Created on: Sep 2, 2025
 *      Author: mcaplan
 */

#ifndef STATE_ADJUST_H_
#define STATE_ADJUST_H_

#include "state.h"
#include "hardware/knob.h"

void STATE_Adjust_Enter(void);
void STATE_Adjust_Exit(void);
void STATE_Adjust_Init(void);

// set up a mailbox for passing around information
struct inputMailbox{
	encoder_state_t encoder_a;
	encoder_state_t encoder_b;
	encoder_state_t encoder_c;
	uint8_t multifunction_toggle;
};

extern volatile struct inputMailbox adjustMailbox;



#endif /* STATE_ADJUST_H_ */
