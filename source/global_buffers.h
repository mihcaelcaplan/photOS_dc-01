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
//#define C_IMG_HEIGHT 960
//#define C_IMG_WIDTH  960

#define C_IMG_HEIGHT 1920
#define C_IMG_WIDTH  1920

#define D_IMG_HEIGHT 480
#define D_IMG_WIDTH  480
#define APP_FB_BPP 3
#define APP_FB_STRIDE_BYTE (D_IMG_WIDTH * APP_FB_BPP)
#define FRAME_BUFFER_ALIGN 64


    
    //declare buffers extern so the actual memory only lives in one place =)
    extern uint8_t c_frameBuffer[3][C_IMG_HEIGHT*C_IMG_WIDTH];

    //need a camera buffer valid manager
    typedef struct {
        uint32_t* status;
        uint32_t dma_buffer0_sa;
        uint32_t dma_buffer1_sa;
        uint32_t clean_buffer_sa;
        uint32_t empty_buffer_sa;
        int last_fb_i;

        bool data_valid;
        bool buffer0_clean;
        bool buffer1_clean;
        
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

    extern uint8_t s_frameBuffer[2][D_IMG_HEIGHT*D_IMG_WIDTH * APP_FB_BPP];
    //need a display buffer ready manage
    typedef struct {
        // addresses
        uint32_t buffer0_sa;
	    uint32_t buffer1_sa;
        
        // lifecycle: ready, processing, full, active
        bool buffer0_ready;
        bool buffer0_processing;
        bool buffer0_full;
        bool buffer0_active;
        
        bool buffer1_ready;
        bool buffer1_processing;
        bool buffer1_full;
        bool buffer1_active;

        // interfaces
        uint32_t ready_sa;

        
        // callbacks
        // void (*fill_callback)(struct display_buffer_manager_t* self);
        // void (*fill_callback)(struct display_buffer_manager_t* self);
        
        // hook functions for state machine advance on frame edges
        void (*vsync_edge)(struct display_buffer_manager_t* self); // start of frame callback
        void (*cur_frame_done)(struct display_buffer_manager_t* self); // data out done callback

    } display_buffer_manager_t;

// declare the global managers
extern camera_buffer_manager_t camera_buffer_manager;
extern display_buffer_manager_t display_buffer_manager;

struct mailbox {
	bool full;
	uint32_t data;
};

extern struct mailbox cameraMailbox;
extern struct mailbox lcdMailbox;



#endif /* GLOBAL_BUFFERS_H_ */
