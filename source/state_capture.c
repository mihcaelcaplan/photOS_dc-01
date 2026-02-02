/*
 * state_capture.c
 *
 *  Created on: Aug 25, 2025
 *      Author: mcaplan
 */

#include "state.h"
#include "state_capture.h"
#include "fsl_debug_console.h"
#include "global_buffers.h"
#include "image_processing.h"
#include "jpeg_interface.h"
#include "storage_sd_device.h"
#include "display_file.h"
#include "fsl_csi.h"
#include "image_processing.h"
#include "avi.h"
#include "timer.h"



// declare some objects
//AT_NONCACHEABLE_SECTION(static FATFS g_fileSystem); /* File system object */
AT_NONCACHEABLE_SECTION(static FIL jpgFil);
AT_NONCACHEABLE_SECTION(static FIL attributeFil);

AT_NONCACHEABLE_SECTION(static FIL indxFil); // for avi index tmp file
bool continuing_capture = 0;

// set up a control struct for movie
capture_control_t cap_control = {
		.capture_end = false
};

// set up filename and iterate counter and open jpg file
FRESULT error;
UINT br;
TCHAR filename[64];


static void iterateDCIMCounter(void){
//	 iterate the counter
	global_dcim_counter++;

	// write the file
	error = f_open(&attributeFil, "DCIM/DC01/attrib.dat", FA_OPEN_EXISTING | FA_WRITE);
		if (error != FR_OK){
			PRINTF("counter update fail");
		}
	f_write(&attributeFil, &global_dcim_counter, sizeof(global_dcim_counter), &br);
	f_close(&attributeFil);
}


void STATE_Capture_Enter(void){

    // make options 
    // demosaic_options_t db_options;
	// #define ZOOM_LEVEL zoom_level_2
    PROCESSING_MakeOptions(zoom_level, &db_options);
    
    unsigned char* jpg_buffer = p_frameBuffer;
    unsigned long jpg_size = sizeof(p_frameBuffer);

    while(1){
        
        // capture
        // wait for frame_ready
        while(!cameraMailbox.full){
                    __NOP();
        }
        uint8_t* last_processed;    
        assert(cameraMailbox.data == 1 || cameraMailbox.data == 2);

        // rotate the buffers
        if(cameraMailbox.data == 1){
            // store processing address
            last_processed = camera_buffer_manager.procesing_buffer_sa;
            // 1 to processing
            camera_buffer_manager.procesing_buffer_sa = camera_buffer_manager.dma_buffer0_sa;

            // last processed to 1
            camera_buffer_manager.dma_buffer0_sa = last_processed;
            CSI->DMASA_FB1 = camera_buffer_manager.dma_buffer0_sa;
        }
        else if(cameraMailbox.data == 2){
            // store processing address
            last_processed = camera_buffer_manager.procesing_buffer_sa;
            // 1 to processing
            camera_buffer_manager.procesing_buffer_sa = camera_buffer_manager.dma_buffer1_sa;

            // last processed to 2
            camera_buffer_manager.dma_buffer1_sa = last_processed;
            CSI->DMASA_FB2 = camera_buffer_manager.dma_buffer1_sa;
        }

        //	reset mailbox
        cameraMailbox.full = false;
        cameraMailbox.data = 0;

        //process the image from the framebuffer
        uint32_t processing_start = TIMER_GetCurrentUs();

        PROCESSING_DebayerJPEG(camera_buffer_manager.procesing_buffer_sa, &jpg_buffer, &jpg_size, &db_options);

        uint32_t processing_elapsed = TIMER_GetCurrentUs() - processing_start;

        PRINTF("CAPTURE: Processing time %d ms\r\n", processing_elapsed/1000);
    

        /* runs once and exits the loop */
        if(CAPTURE_MODE == STILL){
            snprintf(filename, sizeof(filename), "DCIM/DC01/dci_%d.jpg", global_dcim_counter);
            //	 open the jpg file
            error = f_open(&jpgFil, filename, FA_OPEN_ALWAYS | FA_WRITE);
            if (error != FR_OK){
                PRINTF("file %s open fail with error %d", filename, error);
            }
            //	write buffer to a file
            UINT bytesWritten = 0;
            unsigned char *write_pos = jpg_buffer;
            UINT bytesRemain = jpg_size;
            while (bytesRemain > 0){
                f_write(&jpgFil, write_pos, bytesRemain, &bytesWritten);
                bytesRemain -= bytesWritten;
                write_pos += bytesWritten;
            }
            //	close the file
            jpeg_destroy_compress(&cinfo);
            f_close(&jpgFil);

            iterateDCIMCounter();

            // signal that we will go back to compose
            STATE_set_current(COMPOSE);
        
        }
        else if(CAPTURE_MODE == MOVIE){
            // if this our first loop, init things
            if (continuing_capture == false) {
                snprintf(filename, sizeof(filename), "DCIM/DC01/dci_%d.avi", global_dcim_counter);

                error = f_open(&jpgFil, filename, FA_OPEN_ALWAYS | FA_WRITE);
                if (error != FR_OK){
                    PRINTF("file %s open fail with error %d", filename, error);
                }
                error = f_open(&indxFil, "index.tmp", FA_OPEN_ALWAYS | FA_WRITE);
                if (error != FR_OK){
                    PRINTF("file %s open fail with error %d", "index.tmp", error);
                }


                
                // init the handle structure with fixed headers and placeholders for patching
                avi_handle.index_pointer = &indxFil;
                AVI_Init(&jpgFil, db_options.roi_width, db_options.roi_height, jpg_size);

                AVI_AddFrame(jpg_buffer, jpg_size);

                continuing_capture = true;
            }
            else if(continuing_capture == true){
                AVI_AddFrame(jpg_buffer, jpg_size);

                if(cap_control.capture_end == true){ //TODO: this needs to be set by another button press i think
                    PRINTF("Capture end, patching...\r\n");
                    AVI_Patch();
                    
                    continuing_capture == false;
                    
                    // shut down the compressor
                    jpeg_destroy_compress(&cinfo);
                    
                    // flush everything to file
                    f_close(&jpgFil);

                    iterateDCIMCounter();

                    // go back to compose
                    STATE_set_current(COMPOSE);
                }
            }        
        }
        
        // switch back to compose if necessary, does nothing if we are in continuing capture
        STATE_transition();
    }
}


