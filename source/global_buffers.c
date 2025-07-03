/*
 * global_buffers.c
  *  Created on: Jun 24, 2025
 *      Author: mcaplan
 */

#include "global_buffers.h"


// underlying memory
// for display - 24 bpp or 3 Bpp
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint32_t s_frameBuffer[2][APP_IMG_HEIGHT*APP_IMG_WIDTH * APP_FB_BPP],
	FRAME_BUFFER_ALIGN
);

// for raw cam data - 8 bpp or 1Bpp
AT_NONCACHEABLE_SECTION_ALIGN
(
	uint32_t c_frameBuffer[3][APP_IMG_HEIGHT*APP_IMG_WIDTH],
	FRAME_BUFFER_ALIGN
);


// status variables, 32 bit flag variables with register syntax?
volatile uint32_t transfer_status = 0; //0x0 = no transfer 0x1 = transfer in progress, 0x2 = ....
volatile uint32_t camera_status = 0;  // 0x0 = not valid, 0x1 = valid
volatile uint32_t display_status = 0;// 0x0 = not ready, 0x1 = ready

// void callbacks
void camera_sof_callback(camera_buffer_manager_t* self){
//	 if empty buffer,
//	 detach
	if(self->clean_buffer_sa== 00 && self->empty_buffer_sa != 0){
		if(self->last_fb_i == 0){
			CSI->DMASA_FB1 = self->empty_buffer_sa; //ideally repoint the last completed fb
			self->clean_buffer_sa = self->dma_buffer0_sa; // clean -> dmaN
			self->dma_buffer0_sa = self->empty_buffer_sa; // dmaN -> empty

		}
		else{
			CSI->DMASA_FB2 = self->empty_buffer_sa; //ideally repoint the last completed fb
			self->clean_buffer_sa = self->dma_buffer1_sa; // clean -> dmaN
			self->dma_buffer1_sa = self->empty_buffer_sa; // dmaN -> empty

		}
		// rotate buffers
		self->empty_buffer_sa = NULL; //no empty buffer until the clean one is drained
		self->data_valid_sa = self->clean_buffer_sa;
		*self->status |= 0x1;

		__DSB();
	}
}
void camera_dmadone_callback(camera_buffer_manager_t* self, int fb_i){
	//swap fb done to to empty (full?)
	self->last_fb_i = fb_i;
	__DSB();
}

void drain_valid(camera_buffer_manager_t* self){
	if(self->data_valid_sa != 0){
		self->data_valid_sa = 0;
		self->clean_buffer_sa = 0;
		*self->status &= ~0x1;
	}
}

// init the manager
camera_buffer_manager_t camera_buffer_manager = {
	.status = &camera_status,
	.dma_buffer0_sa = &c_frameBuffer[0],
	.dma_buffer1_sa = &c_frameBuffer[1],
	.clean_buffer_sa = NULL,
	.empty_buffer_sa = &c_frameBuffer[2],
	.last_fb_i = 0,

	.data_valid_sa = NULL,
	.drain_callback = drain_valid,

	// need to put in callbacks before using
	.start_of_frame = camera_sof_callback,
	.dma_done = camera_dmadone_callback
};

// define callbacks
void display_vsync_callback(display_buffer_manager_t* self){
	// if full unlocked buffer, send it to the display and lock it
	uint32_t status = *self->status;

	if(status & DISPLAY_BUFFER0_FULL && !(status & DISPLAY_BUFFER0_LOCK)){
		*self->status |= DISPLAY_BUFFER0_LOCK;
		LCDIF->NEXT_BUF = self->buffer0_sa;
		
	}
	else if(status & DISPLAY_BUFFER1_FULL && !(status & DISPLAY_BUFFER1_LOCK)){
		*self->status |= DISPLAY_BUFFER1_LOCK;
		LCDIF->NEXT_BUF = self->buffer1_sa;
	}

}
void display_framedone_callback(display_buffer_manager_t* self){
	 uint32_t status = *self->status;
	// try to unlock buffers
	if(LCDIF->CUR_BUF == self->buffer0_sa && status & DISPLAY_BUFFER0_LOCK){
		status &= ~(DISPLAY_BUFFER0_LOCK);
		status &= ~(DISPLAY_BUFFER0_FULL);
	}
	else if(LCDIF->CUR_BUF == self->buffer1_sa && status & DISPLAY_BUFFER1_LOCK){
		status &= ~DISPLAY_BUFFER1_LOCK;
		status &= ~DISPLAY_BUFFER1_FULL;
	}
	
	// if empty buffer, make ready
	if(!(status & DISPLAY_BUFFER0_FULL)){
		// set framebuf ready for transfer address
		self->ready_sa = self->buffer0_sa;

		// set ready for transfer
		*self->status |= 0x1;
	}
	else if(!(status & DISPLAY_BUFFER1_FULL)){
		*self->status |= 0x1;
		self->ready_sa = self->buffer1_sa;
	}
	else{
		*self->status &= ~0x1;
		self->ready_sa = NULL;
	}
}

void fill_ready(display_buffer_manager_t* self){
//	just mark whatever buffer the ready refers to as full
	if(self->ready_sa != 0){
		if(self->ready_sa == self->buffer0_sa){
			*self->status |= DISPLAY_BUFFER0_FULL;
			self->ready_sa = 0;
		}
		else{
			*self->status |= DISPLAY_BUFFER1_FULL;
			self->ready_sa = 0;
		}
	}
}

// init the manager
display_buffer_manager_t display_buffer_manager = {
	.status = &display_status,
	.buffer0_sa = s_frameBuffer[0],
	.buffer1_sa = s_frameBuffer[1],
	.vsync_edge = display_vsync_callback,
	.cur_frame_done = display_framedone_callback,
	.fill_callback = fill_ready,
};


// init the manager
transfer_manager_t transfer_manager = {
	.status = 0,
	.ready_block_status = &display_status,
	.data_valid_block_status = &camera_status
//	.transfer_callback = NULL, // add before using
};
