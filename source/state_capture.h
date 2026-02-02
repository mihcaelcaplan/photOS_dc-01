/*
 * state_capture.h
 *
 *  Created on: Aug 25, 2025
 *      Author: mcaplan
 */

#ifndef STATE_CAPTURE_H_
#define STATE_CAPTURE_H_

// capture type enum
enum {
    STILL,
    MOVIE
};

typedef struct {
	bool capture_end;
} capture_control_t;

extern capture_control_t cap_control;

/* TODO: move to run-time mode switch :) */
#define CAPTURE_MODE MOVIE


void STATE_Capture_Enter(void);


#endif /* STATE_CAPTURE_H_ */
