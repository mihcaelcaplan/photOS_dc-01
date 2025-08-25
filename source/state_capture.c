/*
 * state_capture.c
 *
 *  Created on: Aug 25, 2025
 *      Author: mcaplan
 */

#include "state.h"
#include "fsl_debug_console.h"

void STATE_Capture_Enter(void){
    PRINTF("CAPTURE`: Entering capture loop\r\n");

    PRINTF("Capture...\r\n");
    // capture loop
    STATE_set_current(COMPOSE);
    STATE_transition();

}


