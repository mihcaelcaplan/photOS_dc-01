/*
 * state.c
 *
 *  Created on: Jul. 17, 2025
 *      Author: mcaplan
 */

#include "state.h"
#include "fsl_debug_console.h"
#include "hardware/battery_interface.h"
#include "hardware/storage_usb_device.h"
#include "app/state_compose.h"
#include "app/state_capture.h"
#include "app/state_transfer.h"
#include "app/state_adjust.h"



// the idea here is that interrupt handlers add to the event queue, the event queue is parsed in the state management periodic function 
// could be in the get_state that runs at each state while condition check
// or could be in a tick interrupt for more deterministic response
// then the queued events are translated into state change that the state transition manages by exiting current (if necessary) and entering the new state
// each state is a loop, and they are transitioned between by events
// battery gating may be at the event parsing level or at the state transition level, depending on timing

static state_t current_state = COMPOSE;
state_t last_state = 0;

// set up the initial state
void STATE_Init(void) {
    // TODO: refactor to set_current_state -> state_transition flow
    current_state = COMPOSE;
    PRINTF("STATE: Initialized to COMPOSE\r\n");
        STATE_Compose_Enter();
}

// goes after the state checking while loop in inner state

void STATE_transition() {

    if (last_state != current_state){
        PRINTF("STATE: Transitioning from %d to %d\r\n", last_state, current_state);
        
        // exit handlers
        switch (last_state){
            case COMPOSE:
                // clean the rowbuffers
                STATE_Compose_Exit();

                break;
            case TRANSFER:
               PRINTF("STATE: Leaving TRANSFER mode\r\n");
               STATE_Transfer_Exit();
//                break;
            default:
                PRINTF("STATE: Leaving %d mode, no exit handler\r\n", last_state);
        }

        // entrance handlers
        switch (current_state) {
            case COMPOSE:
                PRINTF("STATE: Entering COMPOSE mode\r\n");
                STATE_Compose_Enter();
                break;
            case ADJUST:
                 PRINTF("STATE: Entering ADJUST mode\r\n");
                 STATE_Adjust_Enter();
                break;
            case CAPTURE:
                PRINTF("STATE: Entering CAPTURE mode\r\n");
                STATE_Capture_Enter();
                break;
            case REVIEW:
                // PRINTF("STATE: Entering REVIEW mode\r\n");
                break;
            case BROWSE:
                // PRINTF("STATE: Entering BROWSE mode\r\n");
                break;
            case TRANSFER:
                PRINTF("STATE: Entering TRANSFER mode\r\n");
                STATE_Transfer_Enter();
                break;
            case STOW:
                // PRINTF("STATE: Entering STOW mode\r\n"); // low power sleep
                break;
            default:
                // PRINTF("STATE: Unknown state %d\r\n", new_state);
                break;
        }
}
}

state_t STATE_get_current(void) {
    return current_state;
}


void STATE_set_current(state_t new_state){
    // push states down
	if(!(new_state == current_state)){
		last_state = current_state;
		current_state = new_state;
	}

}



void STATE_Queue_Init(state_queue_t *queue){
    memset(queue, 0, sizeof(state_queue_t));
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

bool STATE_Queue_Pop(state_queue_t *queue, state_t *state){
    if (STATE_Queue_IsEmpty(queue)) {
        return false;
    }
    
    *state = queue->states[queue->head];
    queue->head = (queue->head + 1) % STATE_QUEUE_SIZE;
    queue->count--;
    
    return true;
}

bool STATE_Queue_Push(state_queue_t *queue, state_t state){
     if (STATE_Queue_IsFull(queue)) {
        return false;
    }
    
   queue->states[queue->tail] = state;
    
    queue->tail = (queue->tail + 1) % STATE_QUEUE_SIZE;
    queue->count++;
    
    return true;

}
bool STATE_Queue_IsEmpty(state_queue_t *queue){
     return queue->count == 0;

    }
bool STATE_Queue_IsFull(state_queue_t *queue){
    return queue->count >= STATE_QUEUE_SIZE;
}

uint8_t STATE_Queue_Count(state_queue_t *queue){
    return queue->count;
}

void STATE_Queue_Clear(state_queue_t *queue){
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}
