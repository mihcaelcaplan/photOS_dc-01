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
#define C_IMG_HEIGHT 1920
#define C_IMG_WIDTH  1920

#define D_IMG_HEIGHT 480
#define D_IMG_WIDTH  480
#define APP_FB_BPP 3
#define APP_FB_STRIDE_BYTE (D_IMG_WIDTH * APP_FB_BPP)
#define FRAME_BUFFER_ALIGN 64

    
//declare buffers extern so the actual memory only lives in one place =)
extern uint8_t c_frameBuffer[3][C_IMG_HEIGHT*C_IMG_WIDTH];
extern uint8_t p_frameBuffer[C_IMG_HEIGHT*C_IMG_WIDTH * APP_FB_BPP];
extern uint8_t s_frameBuffer[2][D_IMG_HEIGHT*D_IMG_WIDTH * APP_FB_BPP];

// external buffer for 3 rows for interpolation
extern uint8_t rowBuffer[3][C_IMG_WIDTH];
extern uint8_t scanlineBuffer[C_IMG_WIDTH*APP_FB_BPP];

    //need a camera buffer valid manager
    typedef struct {
        uint32_t dma_buffer0_sa;
        uint32_t dma_buffer1_sa;
        uint32_t procesing_buffer_sa;

    } camera_buffer_manager_t;

    //need a display buffer ready manage
    typedef struct {
        // addresses
        uint32_t buffer0_sa;
	    uint32_t buffer1_sa;
    } display_buffer_manager_t;

// declare the global managers
extern camera_buffer_manager_t camera_buffer_manager;
extern display_buffer_manager_t display_buffer_manager;

struct mailbox {
	bool full;
	uint32_t data;
};

extern volatile struct mailbox cameraMailbox;
extern volatile struct mailbox lcdMailbox;

extern uint32_t global_dcim_counter;

typedef enum {
    zoom_level_1 = 4,
    zoom_level_2 = 2,
    zoom_level_3 = 1
} zoom_level_t;
extern zoom_level_t zoom_level;




#endif /* GLOBAL_BUFFERS_H_ */
