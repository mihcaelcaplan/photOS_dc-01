#ifndef EVENTS_H_
#define EVENTS_H_

#include <stdint.h>
#include <stdbool.h>

#define EVENT_QUEUE_SIZE 16

typedef enum {
    EVENT_TYPE_BUTTON_PRESS,
    EVENT_TYPE_BUTTON_RELEASE,
    EVENT_TYPE_USB_CONNECT,
    EVENT_TYPE_USB_DISCONNECT,
    EVENT_TYPE_BATTERY_LOW,
    EVENT_TYPE_TIMER_EXPIRED,
} event_type_t;

typedef struct {
    event_type_t type;
    uint32_t data;
} event_t;

typedef struct {
    event_t events[EVENT_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} event_queue_t;


void EVENT_Init(event_queue_t *queue);
bool EVENT_Push(event_queue_t *queue, event_type_t type, uint32_t data);
bool EVENT_Pop(event_queue_t *queue, event_t *event);
bool EVENT_IsEmpty(event_queue_t *queue);
bool EVENT_IsFull(event_queue_t *queue);
uint8_t EVENT_Count(event_queue_t *queue);
void EVENT_Clear(event_queue_t *queue);

#endif // EVENTS_H_