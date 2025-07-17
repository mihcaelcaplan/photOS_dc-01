/*
 * state.c
 *
 *  Created on: Jul. 17, 2025
 *      Author: mcaplan
 */

#include "state.h"
#include "fsl_debug_console.h"
#include "state_compose.h"
#include "battery_interface.h"


static state_t current_state = COMPOSE;

void STATE_Init(void) {
    current_state = COMPOSE;
    PRINTF("STATE: Initialized to COMPOSE\r\n");
    if(BATTERY_Get_Level() < 4){
        STATE_Compose_Loop();
    }
    else{
        PRINTF("STATE: Battery level too low, connect to charger\r\n");
    }

}

void STATE_transition(state_t new_state) {
    state_t previous_state = current_state;
    current_state = new_state;
    
    PRINTF("STATE: Transitioning from %d to %d\r\n", previous_state, new_state);
    
    switch (new_state) {
        case COMPOSE:
            PRINTF("STATE: Entering COMPOSE mode\r\n");
            if(BATTERY_Get_Level() < 4){
                STATE_Compose_Loop();
            }
            else{
                PRINTF("STATE: Battery level too low, connect to charger\r\n");
            }
            break;
        case ADJUST:
            // PRINTF("STATE: Entering ADJUST mode\r\n");
            break;
        case CAPTURE:
            // PRINTF("STATE: Entering CAPTURE mode\r\n");
            break;
        case REVIEW:
            // PRINTF("STATE: Entering REVIEW mode\r\n");
            break;
        case BROWSE:
            // PRINTF("STATE: Entering BROWSE mode\r\n");
            break;
        case TRANSFER:
            // PRINTF("STATE: Entering TRANSFER mode\r\n");
            break;
        case STOW:
            // PRINTF("STATE: Entering STOW mode\r\n");
            break;
        default:
            // PRINTF("STATE: Unknown state %d\r\n", new_state);
            break;
    }
}

state_t STATE_get_current(void) {
    return current_state;
}