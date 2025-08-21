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

typedef struct {
	uint32_t start_row;
	uint32_t start_col;
	uint32_t roi_width;
	uint32_t roi_height;
	uint8_t tile_size;
	uint32_t source_buffer_stride;
} demosaic_options_t;

void PROCESSING_MakeOptions(zoom_level_t zoom, uint8_t tile_size, demosaic_options_t* options);

void PROCESSING_Debayer(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options);

void processOnePixel_CheapBilinear(uint8_t* row, int row_i,  int col_i, demosaic_options_t* options, uint8_t* r, uint8_t* g, uint8_t* b);



#endif /* IMAGE_PROCESSING_H_ */
