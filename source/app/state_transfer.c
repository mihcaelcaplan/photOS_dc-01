/*
 * state_transfer.c
 *
 *  Created on: Aug. 28, 2025
 *      Author: mcaplan
 */

#include "state_transfer.h"
#include "fsl_debug_console.h"
#include "hardware/display_interface.h"

void STATE_Transfer_Enter(void) {
    DISPLAY_Off();


	// wait for the usb detach to set back to compose
    while (STATE_get_current() == TRANSFER) {
        __WFI(); // Wait for interrupt
    }
    STATE_transition();
}

void STATE_Transfer_Exit(void) {
        DISPLAY_On();

}
