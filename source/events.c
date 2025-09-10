#include "events.h"
#include <string.h>


void EVENT_Init(event_queue_t *queue) {
    memset(queue, 0, sizeof(event_queue_t));
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

bool EVENT_Push(event_queue_t *queue, event_type_t type, uint32_t data) {
    if (EVENT_IsFull(queue)) {
        return false;
    }
    
    queue->events[queue->tail].type = type;
    queue->events[queue->tail].data = data;    
    
    queue->tail = (queue->tail + 1) % EVENT_QUEUE_SIZE;
    queue->count++;
    
    return true;
}

bool EVENT_Pop(event_queue_t *queue, event_t *event) {
    if (EVENT_IsEmpty(queue)) {
        return false;
    }
    
    *event = queue->events[queue->head];
    queue->head = (queue->head + 1) % EVENT_QUEUE_SIZE;
    queue->count--;
    
    return true;
}

bool EVENT_IsEmpty(event_queue_t *queue) {
    return queue->count == 0;
}

bool EVENT_IsFull(event_queue_t *queue) {
    return queue->count >= EVENT_QUEUE_SIZE;
}

uint8_t EVENT_Count(event_queue_t *queue) {
    return queue->count;
}

void EVENT_Clear(event_queue_t *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}