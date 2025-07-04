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
#define APP_HSW        20
#define APP_HFP        40
#define APP_HBP        50
 
#define APP_VSW        10
#define APP_VFP        5
#define APP_VBP        5
#define APP_POL_FLAGS \
(kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnFallingClkEdge)
//    ( kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)
    
    //declare buffers extern so the actual memory only lives in one place =)
    extern uint32_t c_frameBuffer[3][APP_IMG_HEIGHT*APP_IMG_WIDTH];
    //need a camera buffer valid manager
    typedef struct {
        uint32_t* status;
        uint32_t dma_buffer0_sa;
        uint32_t dma_buffer1_sa;
        uint32_t clean_buffer_sa;
        uint32_t empty_buffer_sa;
        int last_fb_i;
        
        // must be set before data valid status set
        uint32_t data_valid_sa;
        void (*drain_callback)(struct camera_buffer_manager_t* self);
        
        // hook functions for state machine advance on frame edges
        void (*start_of_frame)(struct camera_buffer_manager_t* self); // start of frame callback
        void (*dma_done)(struct camera_buffer_manager_t* self, int fb_i); // dma done callback
        
    } camera_buffer_manager_t;

    
    #define DISPLAY_BUFFER_READY 0X1
    #define DISPLAY_BUFFER0_FULL 0X2
    #define DISPLAY_BUFFER1_FULL 0X4
    #define DISPLAY_BUFFER0_LOCK 0X8
    #define DISPLAY_BUFFER1_LOCK 0X10

    extern uint32_t s_frameBuffer[2][APP_IMG_HEIGHT*APP_IMG_WIDTH * APP_FB_BPP];
    //need a display buffer ready manage
    typedef struct {
        uint32_t* status; 
        uint32_t buffer0_sa;
	    uint32_t buffer1_sa;
        
        // must be set before ready status set
        uint32_t ready_sa;
        void (*fill_callback)(struct display_buffer_manager_t* self);

        // hook functions for state machine advance on frame edges
        void (*vsync_edge)(struct display_buffer_manager_t* self); // start of frame callback
        void (*cur_frame_done)(struct display_buffer_manager_t* self); // data out done callback

    } display_buffer_manager_t;

// need transfer manager that watches both
typedef struct {
    uint32_t* status;
    void (*transfer_callback)(void);
    uint32_t* ready_block_status; //ready bit must be [0]
    uint32_t* data_valid_block_status; //ready bit must be [0]
} transfer_manager_t;

// declare the global managers
extern camera_buffer_manager_t camera_buffer_manager;
extern display_buffer_manager_t display_buffer_manager;
extern transfer_manager_t transfer_manager;



// static inline
// all this does is check if both blocks are ready and valid and callback and set status bits
// returns status = 0xOOOO, 0x0 = no transfer 0x1 = transfer in progress, 0x2 = ....
static inline void manage_transfer(transfer_manager_t* mgr){
    uint32_t ready = *mgr->ready_block_status;
    uint32_t data_valid = *mgr->data_valid_block_status;
    
    if(ready == 0x1 && data_valid == 0x1){
        // commence transfer
        *mgr->status |= 0x01;
        mgr->transfer_callback();
        
        // reset the block status
        *mgr->ready_block_status &= ~0x1;
        *mgr->data_valid_block_status &= ~0x1;
}
}


#endif /* GLOBAL_BUFFERS_H_ */
