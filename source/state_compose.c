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





void STATE_Compose_Init(void) {
    PRINTF("COMPOSE: Initializing compose state\r\n");
    // TODO: Initialize live view, autofocus, etc.
}

void STATE_Compose_Loop(void) {
    PRINTF("COMPOSE: Entering compose loop\r\n");
    
    while (1) {

        CSI->DMASA_FB1 = camera_buffer_manager.dma_buffer0_sa;
		CSI->DMASA_FB2 = camera_buffer_manager.dma_buffer1_sa;

	//	csi enable base address witch on
//		CSI->CR18 |= CSI_CR18_BASEADDR_SWITCH_EN_MASK | CSI_CR18_BASEADDR_SWITCH_SEL(1);

	//	start csi subsystem
		CSI->CR1 |= CSI_CR1_FB1_DMA_DONE_INTEN_MASK; // enable interrupt fb1 full
		CSI->CR1 |= CSI_CR1_FB2_DMA_DONE_INTEN_MASK; // enable interrupt fb2 full
		CSI->CR1 |= CSI_CR1_SOF_INTEN_MASK;; // enable interrupt fb2 full
		CSI->CR3 |= CSI_CR3_DMA_REQ_EN_RFF_MASK; 	 // enable dma reqs from receive fifo
		CSI->CR18|= CSI_CR18_CSI_ENABLE_MASK; 		 // enable csi module

		ST7701_SPIWrite(0x13, COMMAND); // normal display mode on
		simpleDelay(10);


		//	enable lcdif and interrupts
		ELCDIF_RgbModeStart(LCDIF);
		ELCDIF_EnableInterrupts(LCDIF, kELCDIF_VsyncEdgeInterruptEnable | kELCDIF_CurFrameDoneInterruptEnable); //seems like don't need vsync

//		one shot run
		GPIO_PinWrite(GPIO1, 9U, 1U); //display backlight enable

		int c = 0;

		while(1){
			CSI_Start(CSI);
	//		wait until ready & valid
//			while(!((*transfer_manager.data_valid_block_status & 0x1) && (*transfer_manager.ready_block_status & 0x1))){
			while(!(*camera_buffer_manager.status & 0x1)){
				__NOP();
			}


//			processForDisplay((uint32_t*)camera_buffer_manager.data_valid_sa, (uint32_t*)display_buffer_manager.ready_sa);

			int i  = c%2;
			interpolateForDisplay((uint32_t*)camera_buffer_manager.data_valid_sa, (uint32_t*)s_frameBuffer[i]);
//			processForDisplay((uint32_t*)camera_buffer_manager.data_valid_sa, (uint32_t*)s_frameBuffer[i]);
			if(pending_frame == false){
//				LCDIF->NEXT_BUF = pending_frame_sa;
				pending_frame_sa = (uint32_t)s_frameBuffer[i];
				pending_frame = true;
			}
			
			// display_buffer_manager.next(s_frameBuffer[i]);

//			LCDIF->NEXT_BUF = s_frameBuffer[i];


			camera_buffer_manager.drain_callback(&camera_buffer_manager);
			display_buffer_manager.fill_callback(&display_buffer_manager);
			c++;
		}

   
        
        __WFI(); // Wait for interrupt
    }
    
    PRINTF("COMPOSE: Exiting compose loop\r\n");
}
