/*
 * global_buffers.c
 *
 *  Created on: Jun 24, 2025
 *      Author: mcaplan
 */

#include "global_buffers.h"


// underlying memory
// for display - 24 bpp or 3 Bpp
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint32_t s_frameBuffer[s_frameBufferCount][APP_IMG_HEIGHT*APP_IMG_WIDTH * APP_FB_BPP],
	FRAME_BUFFER_ALIGN
);

// for raw cam data - 8 bpp or 1Bpp
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint32_t c_frameBuffer[c_frameBufferCount][APP_IMG_HEIGHT*APP_IMG_WIDTH],
	FRAME_BUFFER_ALIGN
);

//buffer_t screen_buffers[s_frameBufferCount];
//buffer_t camera_buffers[c_frameBufferCount];
//
//// 1 pool of available buffers to take from
//pool_t stale_screen_buffers = {screen_buffers[0], 0, 0, 1};
//pool_t fresh_screen_buffers = {screen_buffers[1], 0, 0, 2};
//
//// 2 possible dirty buffers, 1 possible clean one , 1 empty - allow triple buffering with insurance
//pool_t clean_camera_buffers = {camera_buffers[0], 0, 0, 1};
//pool_t dirty_camera_buffers = {camera_buffers, 0, 0, 2};
//pool_t empty_camera_buffers = {camera_buffers, 0, 0, 1};
//
//// map memory to data structure
//void GLOBAL_buffersInit(void) {
//    // Initialize screen descriptors
//    for (int i = 0; i < s_frameBufferCount; i++) {
//        screen_buffers[i].data = s_frameBuffer[i];
//        screen_buffers[i].size = sizeof(s_frameBuffer[i]);
//    }
//
//    // Initialize camera descriptors
//    for (int i = 0; i < c_frameBufferCount; i++) {
//        camera_buffers[i].data = c_frameBuffer[i];
//        camera_buffers[i].size = sizeof(c_frameBuffer[i]);
//    }
//}
