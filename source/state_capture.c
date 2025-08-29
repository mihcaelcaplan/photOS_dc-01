/*
 * state_capture.c
 *
 *  Created on: Aug 25, 2025
 *      Author: mcaplan
 */

#include "state.h"
#include "fsl_debug_console.h"
#include "global_buffers.h"
#include "image_processing.h"
#include "jpeg_interface.h"
#include "storage_sd_device.h"
#include "display_file.h"
#include "fsl_csi.h"

// declare some objects
//AT_NONCACHEABLE_SECTION(static FATFS g_fileSystem); /* File system object */
AT_NONCACHEABLE_SECTION(static FIL jpgFil);


void STATE_Capture_Enter(void){

    PRINTF("CAPTURE: Entering capture loop\r\n");

    // process the raw_image in the last_processed buffer
    	//	mount the filesystem
	FRESULT error;

	error = MOUNT_SDCard();
    if (error != FR_OK){
        PRINTF("mount SD faillll...");
    }

    //	 open the file
        error = f_open(&jpgFil, _T("DCIM/tcs01.jpg"), FA_OPEN_ALWAYS | FA_WRITE);
    if (error != FR_OK){
        PRINTF("file write fail");
    }
    
    // make options 
    demosaic_options_t db_options;
	#define ZOOM_LEVEL zoom_level_3
    PROCESSING_MakeOptions(ZOOM_LEVEL, &db_options);
    
        // capture
    // wait for
    while(!cameraMailbox.full){
				__NOP();
    }
    // buffer swap
    //lcdmailbox.data is 1 or 2 to rep. framebuffer 1 or
    uint8_t* last_processed;
    
    assert(cameraMailbox.data == 1 || cameraMailbox.data == 2);
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


        //	Debayer
    PROCESSING_DebayerJPEG(camera_buffer_manager.procesing_buffer_sa, &db_options);

    //	write buffer to a file
	UINT bytesWritten = 0;
    unsigned char *write_pos = p_frameBuffer;
	UINT bytesRemain = sizeof(p_frameBuffer);


	while (bytesRemain > 0){
        f_write(&jpgFil, write_pos, bytesRemain, &bytesWritten);
        bytesRemain -= bytesWritten;
        write_pos += bytesWritten;
    }

	 
    //	close the file
    jpeg_destroy_compress(&cinfo);
    f_close(&jpgFil);

    // switch back to compose
    STATE_set_current(COMPOSE);
    STATE_transition();

}


