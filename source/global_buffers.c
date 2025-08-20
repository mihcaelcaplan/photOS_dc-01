/*
 * global_buffers.c
  *  Created on: Jun 24, 2025
 *      Author: mcaplan
 */

#include "global_buffers.h"


// underlying memory
// for display - 24 bpp or 3 Bpp
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint8_t s_frameBuffer[2][D_IMG_HEIGHT*D_IMG_WIDTH * APP_FB_BPP], //480X480X3
	FRAME_BUFFER_ALIGN
);

// for raw cam data - 8 bpp or 1Bpp
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint8_t c_frameBuffer[3][C_IMG_HEIGHT*C_IMG_WIDTH], //1920X1920
	FRAME_BUFFER_ALIGN
);


// status variables, 32 bit flag variables with register syntax?
volatile uint32_t transfer_status = 0; //0x0 = no transfer 0x1 = transfer in progress, 0x2 = ....
volatile uint32_t camera_status = 0;  // 0x0 = not valid, 0x1 = valid
volatile uint32_t display_status = 0;// 0x0 = not ready, 0x1 = ready

// init the manager
camera_buffer_manager_t camera_buffer_manager = {
	.status = &camera_status,
	.dma_buffer0_sa = &c_frameBuffer[0],
	.dma_buffer1_sa = &c_frameBuffer[1],
	.clean_buffer_sa = NULL,
	.empty_buffer_sa = &c_frameBuffer[2],
	.last_fb_i = 0,

	.buffer1_clean = false,
	.buffer0_clean = false,

	.data_valid = false,
	.data_valid_sa = NULL,
//	.drain_callback = drain_valid,
//
//	// need to put in callbacks before using
//	.start_of_frame = camera_sof_callback,
//	.dma_done = camera_dmadone_callback
};

// init the manager
display_buffer_manager_t display_buffer_manager = {
	// addresses
	.buffer0_sa = s_frameBuffer[0],
	.buffer1_sa = s_frameBuffer[1],
	
	// flags
	.buffer0_ready = false,
	.buffer0_processing = false,
	.buffer0_full = false,
	.buffer0_active = false,
	.buffer1_ready = false,
	.buffer1_processing = false,
	.buffer1_full = false,
	.buffer1_active = false,

	// interfaces
	.ready_sa = 0
};

// init the mailbox with data = framebuffer that returned
struct mailbox cameraMailbox = {false, 0};
struct mailbox lcdMailbox = {false, 0};
