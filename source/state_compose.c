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
		// #define ZOOM_LEVEL zoom_level_2
//		#define TILE_SIZE 48

		demosaic_options_t db_options;
		PROCESSING_MakeOptions(zoom_level, &db_options);

		while (STATE_get_current() == COMPOSE) {
//			wait until frameshown
			while(lcdMailbox.full){
				__NOP();
			}

	//		wait until camera receives a frame
			while(!cameraMailbox.full){
				__NOP();
			}
			uint8_t* source_buf;
			uint8_t* dest_buf;
			if(cameraMailbox.data==1){
				source_buf = camera_buffer_manager.dma_buffer0_sa;
				dest_buf = display_buffer_manager.buffer0_sa;
			}
			else if(cameraMailbox.data==2){
				source_buf = camera_buffer_manager.dma_buffer1_sa;
				dest_buf = display_buffer_manager.buffer1_sa;
			}

//			reset mailbox
			cameraMailbox.full = false;
			cameraMailbox.data = 0;
			
//			get timer
			int process_start = TIMER_GetCurrentUs();

			/* IMAGE PIPELINE*/

			// debayer
			 PROCESSING_DebayerLiveView(source_buf, dest_buf, &db_options);
//			 PROCESSING_DebayerLiveView_Raw(source_buf, dest_buf, &db_options);
			

			int processing_time = TIMER_GetCurrentUs() - process_start;
//			PRINTF("TIME processing:  %d uS\r\n", processing_time);


			// fill lcd mailbox
			lcdMailbox.data = (uint32_t)dest_buf;
			lcdMailbox.full = true;
			

			c++;
			__WFI(); // Wait for interrupt
		}

   
    PRINTF("COMPOSE: Exiting compose loop\r\n");
	// TODO: branch to new state
    STATE_transition();
}
