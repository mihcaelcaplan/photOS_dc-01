/*
 * STATE.h
 *
 *  Created on: Jun 23, 2025
 *      Author: mcaplan
 */
#ifndef STATE_H_
#define STATE_H_

#include <stdbool.h>
#include <stdint.h>

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

// state queue functions
#define STATE_QUEUE_SIZE 16

typedef struct _state_queue {
    state_t states[STATE_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} state_queue_t;

void STATE_Queue_Init(state_queue_t *queue);
bool STATE_Queue_Pop(state_queue_t *queue, state_t *state);
bool STATE_Queue_Push(state_queue_t *queue, state_t state);
bool STATE_Queue_IsEmpty(state_queue_t *queue);
bool STATE_Queue_IsFull(state_queue_t *queue);
uint8_t STATE_Queue_Count(state_queue_t *queue);
void STATE_Queue_Clear(state_queue_t *queue);

#endif STATE_H_
