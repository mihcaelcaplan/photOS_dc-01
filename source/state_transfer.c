/*
 * state_transfer.c
 *
 *  Created on: Aug. 28, 2025
 *      Author: mcaplan
 */

#include "state_transfer.h"
#include "fsl_debug_console.h"

void STATE_Transfer_Enter(void) {

	// wait for the usb detach to set back to compose
    while (STATE_get_current() == TRANSFER) {
        __WFI(); // Wait for interrupt
    }
    
    STATE_transition();
}

void STATE_Transfer_Exit(void) {
    // TODO: Implement transfer exit functionality
}
