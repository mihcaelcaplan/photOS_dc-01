/*
 * global_buffers.c
  *  Created on: Jun 24, 2025
 *      Author: mcaplan
 */

#include "global_buffers.h"


// underlying memory
// for display -  3 bytes per pixel
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint8_t s_frameBuffer[2][D_IMG_HEIGHT*D_IMG_WIDTH * APP_FB_BPP], //480X480X3
	FRAME_BUFFER_ALIGN
);

// for raw cam data - 1 byte per pixel
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint8_t c_frameBuffer[3][C_IMG_HEIGHT*C_IMG_WIDTH], //1920X1920
	FRAME_BUFFER_ALIGN
);

//for saving full size
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint8_t p_frameBuffer[C_IMG_HEIGHT*C_IMG_WIDTH*APP_FB_BPP], //1920X1920
	FRAME_BUFFER_ALIGN
);


// init the manager
camera_buffer_manager_t camera_buffer_manager = {
	.dma_buffer0_sa = &c_frameBuffer[0],
	.dma_buffer1_sa = &c_frameBuffer[1],
};

// init the manager
display_buffer_manager_t display_buffer_manager = {
	// addresses
	.buffer0_sa = &s_frameBuffer[0],
	.buffer1_sa = &s_frameBuffer[1],
};

// init the mailbox with data = framebuffer that returned
struct mailbox cameraMailbox = {false, 0};
struct mailbox lcdMailbox = {false, 0};
