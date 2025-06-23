/*
 * STATE.h
 *
 *  Created on: Jun 23, 2025
 *      Author: mcaplan
 */
#ifndef STATE_H_
#define STATE_H_

// working on an event driven loop with WFI in between processing

// maybe i want to OR them together so i space out the bits
typedef enum _state {
    // MAIN UX LOOP
    COMPOSE = 0x01,
    ADJUST = 0x02,
    CAPTURE = 0x04,
    REVIEW = 0x08,

    // DEAL WITH IMAGES
    BROWSE = 0x10,
    TRANSFER = 0x20,

    // TRANSITION OUT OF USE
    STOW = 0x40,

} state_t;


// declarations
void STATE_Init(void);

void STATE_transition(state_t new_state);


#endif STATE_H_