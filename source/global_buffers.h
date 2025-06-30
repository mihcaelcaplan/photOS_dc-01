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


//for display specifically TODO: move?
#define APP_HSW        100
#define APP_HFP        100
#define APP_HBP        80
#define APP_VSW        4
#define APP_VFP        40
#define APP_VBP        20
#define APP_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnFallingClkEdge)


//declare
extern uint32_t s_frameBuffer[3][APP_IMG_HEIGHT*APP_IMG_WIDTH * APP_FB_BPP];
extern uint32_t c_frameBuffer[3][APP_IMG_HEIGHT*APP_IMG_WIDTH];


#endif /* GLOBAL_BUFFERS_H_ */
