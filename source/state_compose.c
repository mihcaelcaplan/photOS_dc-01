/*
 * state_compose.c
 *
 *  Created on: Jul. 17, 2025
 *      Author: mcaplan
 */

#include "state_compose.h"

#include "fsl_debug_console.h"
#include "fsl_csi.h"
#include "fsl_elcdif.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include <stdbool.h>
#include "global_buffers.h"
#include "lcd_st7701.h"
#include "camera_ov5640.h"
#include "utils.h"
#include "timer.h"
#include "image_processing.h"



void STATE_Compose_Enter(void) {
    PRINTF("COMPOSE: Entering compose loop\r\n");
    
    while (STATE_get_current() == COMPOSE) {

        CSI->DMASA_FB1 = camera_buffer_manager.dma_buffer0_sa;
		CSI->DMASA_FB2 = camera_buffer_manager.dma_buffer1_sa;

	//	csi enable base address witch on
//		CSI->CR18 |=~~` CSI_CR18_BASEADDR_SWITCH_EN_MASK | CSI_CR18_BASEADDR_SWITCH_SEL(1);

	//	start csi subsystem
		CSI->CR1 |= CSI_CR1_FB1_DMA_DONE_INTEN_MASK; // enable interrupt fb1 full
		CSI->CR1 |= CSI_CR1_FB2_DMA_DONE_INTEN_MASK; // enable interrupt fb2 full
		CSI->CR1 |= CSI_CR1_SOF_INTEN_MASK;; // enable interrupt sof
		CSI->CR3 |= CSI_CR3_DMA_REQ_EN_RFF_MASK; 	 // enable dma reqs from receive fifo
		CSI->CR18|= CSI_CR18_CSI_ENABLE_MASK; 		 // enable csi module

		ST7701_SPIWrite(0x13, COMMAND); // normal display mode on

		simpleDelay(10);

		//	enable lcdif and interrupts
		ELCDIF_RgbModeStart(LCDIF);
		ELCDIF_EnableInterrupts(LCDIF, kELCDIF_VsyncEdgeInterruptEnable | kELCDIF_CurFrameDoneInterruptEnable); //seems like don't need vsync
		GPIO_PinWrite(GPIO1, 9U, 1U); //display backlight enable

		int c = 0;

		CSI_Start(CSI);


		// init image processing
		#define ZOOM_LEVEL zoom_level_3
		#define TILE_SIZE 48

		demosaic_options_t db_options;
		PROCESSING_MakeOptions(ZOOM_LEVEL, TILE_SIZE, &db_options);

		while(1){
//			wait until frameshown
			while(lcdMailbox.full){
				__NOP();
			}

	//		wait until camera receives a frame
			while(!cameraMailbox.full){
				__NOP();
			}

//			reset mailbox
			cameraMailbox.full = false;
			cameraMailbox.data = 0;
	
			int i = c%2; //ping pong var for framebuffers

			uint8_t* source_buf = c_frameBuffer[i];
			uint8_t* processing_buf = p_frameBuffer;
			uint8_t* dest_buf = s_frameBuffer[i];
			
//			get timer
			int process_start = TIMER_GetCurrentUs();


//			binAndInterpolateForDisplay(c_frameBuffer[i], dest_buf, ZOOM_LEVEL);
//			fill_framebuffer_gradient(dest_buf, 480, 480);

			/* IMAGE PIPELINE*/

			// debayer
			PROCESSING_DebayerLiveView(source_buf, processing_buf, &db_options);

			// downscale if necessary
			if(ZOOM_LEVEL == zoom_level_1){
//				// downscale 1:4

				int start_row = 0;
				int start_col = 0;
				// copy directly (row by row)
				int source_width = 1920;
				int dest_width = 480;
				for(int i = 0; i< D_IMG_HEIGHT; i++){
					// start address + row_offset+i*width + col_offset

//					i*2 skips rows across the 960 high source
					uint8_t* copy_src = processing_buf + ((start_row + i*4)* source_width + start_col)*3;

//					copy in a line to the buffer
					uint8_t* copy_processing =rowBuffer[0];
					memcpy(copy_processing, copy_src, 1920*3);

					uint8_t* dest_row = dest_buf+ i*dest_width*3;

//					walk through the rowbuffer and condense
					for(int j = 0; j<dest_width; j++){
						int row_i = 4*j*3;
						uint8_t* row_pixel = rowBuffer[0] + row_i;
						uint8_t* dest = dest_row + j*3;

						memcpy(dest, row_pixel, 3);

					}
				}

			}
			if(ZOOM_LEVEL == zoom_level_2){
				// downscale 1:2
				
				int start_row = 480;
				int start_col = 480;
				// copy directly (row by row)
				int source_width = 1920;
				int dest_width = 480;
				for(int i = 0; i< D_IMG_HEIGHT; i++){
					// start address + row_offset+i*width + col_offset

//					i*2 skips rows across the 960 high source
					uint8_t* copy_src = processing_buf + ((start_row + i*2)* source_width + start_col)*3;

//					copy in a line to the buffer
					uint8_t* copy_processing =rowBuffer[0];
					memcpy(copy_processing, copy_src, 960*3);

					uint8_t* dest_row = dest_buf+ i*dest_width*3;

//					walk through the rowbuffer and condense
					for(int j = 0; j<dest_width; j++){
						int row_i = 2*j*3;
						uint8_t* row_pixel = rowBuffer[0] + row_i;
						uint8_t* dest = dest_row + j*3;

						memcpy(dest, row_pixel, 3);

					}
				}

			}
			if (ZOOM_LEVEL == zoom_level_3){
				// copy directly (row by row)
				int source_width = 1920;
				int dest_width = 480;
				int start_row = 720;
				int start_col = 720;
				for(int i = 0; i< D_IMG_HEIGHT; i++){
					// start address + row_offset+i*width + col_offset
//					uint8_t* copy_src = processing_buf +i*source_width*3;
					uint8_t* copy_src = processing_buf + ((start_row + i)* source_width + start_col)*3;
					// start address + row_offset
					uint8_t* copy_dest = dest_buf + i*dest_width*3;

					memcpy(copy_dest, copy_src, dest_width*3);
				}
			}
			int processing_time = TIMER_GetCurrentUs() - process_start;
			PRINTF("TIME processing:  %d uS\r\n", processing_time);


			// fill lcd mailbox
			lcdMailbox.data = (uint32_t)dest_buf;
//			lcdMailbox.data= processing_buf + 720*1920 +720;
			lcdMailbox.full = true;
			

			c++;
		}

   
        
        __WFI(); // Wait for interrupt
    }
    
    PRINTF("COMPOSE: Exiting compose loop\r\n");
}
