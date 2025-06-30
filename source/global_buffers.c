/*
 * global_buffers.c
 *
 *  Created on: Jun 24, 2025
 *      Author: mcaplan
 */

#include "global_buffers.h"

// for display - 24 bpp or 3 Bpp
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint32_t s_frameBuffer[2][APP_IMG_HEIGHT*APP_IMG_WIDTH * APP_FB_BPP],
	FRAME_BUFFER_ALIGN
);

// for raw cam data - 8 bpp or 1Bpp
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint32_t c_frameBuffer[2][APP_IMG_HEIGHT*APP_IMG_WIDTH],
	FRAME_BUFFER_ALIGN
);
