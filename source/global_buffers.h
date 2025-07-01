/*
 * global_buffers.h
 *
 *  Created on: Jun 16, 2025
 *      Author: mcaplan
 */

#ifndef GLOBAL_BUFFERS_H_
#define GLOBAL_BUFFERS_H_

#include "fsl_common.h"

// set this to 0 because we not using caches
#define APP_CACHE_LINE_SIZE 0U

//same as elcdif_rgb.h for now
#define APP_IMG_HEIGHT 480
#define APP_IMG_WIDTH  480

#define APP_FB_HEIGHT APP_IMG_HEIGHT
#define APP_FB_WIDTH  APP_IMG_WIDTH
#define APP_FB_BPP 3
#define APP_FB_STRIDE_BYTE (APP_FB_WIDTH * APP_FB_BPP)
#define FRAME_BUFFER_ALIGN 64

// number of types of framebuffers
#define s_frameBufferCount 3
#define c_frameBufferCount 4


//for display specifically TODO: move?
#define APP_HSW        200
#define APP_HFP        100
#define APP_HBP        80
#define APP_VSW        200
#define APP_VFP        40
#define APP_VBP        20
#define APP_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnFallingClkEdge)

//declare buffers
extern uint32_t s_frameBuffer[s_frameBufferCount][APP_IMG_HEIGHT*APP_IMG_WIDTH * APP_FB_BPP];
extern uint32_t c_frameBuffer[c_frameBufferCount][APP_IMG_HEIGHT*APP_IMG_WIDTH];

// buffer structure and ring pool structure
typedef struct {
    void* data;
    uint32_t size;    
} buffer_t;

typedef struct {
    buffer_t* buffers;
    uint8_t head;
    uint8_t tail;
    uint8_t size;
} pool_t;

// declare public buffers
extern buffer_t screen_buffers[s_frameBufferCount];
extern buffer_t camera_buffers[c_frameBufferCount];

static inline buffer_t* address_to_buf(pool_t* pool, void* address){
	for(int i = 0; i<pool->size; i++){
		buffer_t* buf = &pool->buffers[i];
		void* bufAddress = buf->data;
		if (bufAddress == address){
			return buf;
		}
	}
	return NULL;
}

// declare pools
extern pool_t stale_screen_buffers;
extern pool_t fresh_screen_buffers;

// 2 possible dirty buffers, 1 possible clean one - allow triple buffering
extern pool_t clean_camera_buffers;
extern pool_t dirty_camera_buffers;
extern pool_t empty_camera_buffers;

// buffer get and put
//   returns buffer pointer if there is one, otherwise NULL
static inline buffer_t* pool_get(pool_t* pool) {
    uint8_t tail = pool->tail;
    if (tail == pool->head) return NULL;  // empty
    
    buffer_t* buf = &pool->buffers[tail];
    pool->tail = (tail + 1 == pool->size+1) ? 0 : tail + 1; // 0 if at capacity otherwise increment
    return buf;
}

// returns 0 if the put is successful, -1 if no space
static inline int pool_put(pool_t* pool, buffer_t* buf) {
    uint8_t head = pool->head;
    uint8_t next_head = (head + 1 == pool->size + 1) ? 0 : head + 1; // if capacity otherwise increment
    
    if (next_head == pool->tail) return -1;  // full
    
    pool->buffers[head] = *buf;
    pool->head = next_head;
    return 0;
}

static inline int pool_empty(pool_t* pool) {
    return pool->head == pool->tail;
}

static inline int pool_full(pool_t* pool) {
    uint8_t next_head = (pool->head + 1 == pool->size + 1) ? 0 : pool->head + 1;
    return next_head == pool->tail;
}

void GLOBAL_buffersInit(void);


#endif /* GLOBAL_BUFFERS_H_ */
