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


//for display specifically (move somewhere nicer)
#define APP_HSW        41
#define APP_HFP        4
#define APP_HBP        8
#define APP_VSW        10
#define APP_VFP        4
#define APP_VBP        2
#define APP_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)


AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_frameBuffer[1][APP_IMG_HEIGHT*APP_IMG_WIDTH * APP_FB_BPP], FRAME_BUFFER_ALIGN);



#endif /* GLOBAL_BUFFERS_H_ */
