/*
 * image_processing.h
 *
 *  Created on: Aug 20, 2025
 *      Author: mcaplan
 */

#ifndef IMAGE_PROCESSING_H_
#define IMAGE_PROCESSING_H_

#include <stdint.h>
#include "camera_ov5640.h"
#include "global_buffers.h"

typedef struct {
	uint32_t start_row;
	uint32_t start_col;
	uint32_t roi_width;
	uint32_t roi_height;
	uint8_t tile_size;
	uint32_t source_buffer_stride;
	zoom_level_t zoom;

	int r_gain; // r_channel = r * r_gain/255
	int g_gain; // g_channel = g * g_gain/255
	int b_gain; // b_channel = b * b_gain/255
	int pixel_gain; // brightness (iso?) = pixel * pixel_gain/255
} demosaic_options_t;

extern struct jpeg_compress_struct cinfo;
extern struct jpeg_error_mgr jerr;

void PROCESSING_MakeOptions(zoom_level_t zoom, demosaic_options_t* options);

void PROCESSING_Debayer(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options);

void processOnePixel_CheapBilinear(int row_i,  int col_i, demosaic_options_t* options, uint8_t* r, uint8_t* g, uint8_t* b);

void PROCESSING_DebayerLiveView(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options);
void PROCESSING_DebayerLiveView_Raw(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options);

void PROCESSING_DebayerJPEG(uint8_t* source_buffer, demosaic_options_t* options);


#endif /* IMAGE_PROCESSING_H_ */
