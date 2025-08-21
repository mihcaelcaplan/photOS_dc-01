/*
 * image_processing.c
 *
 *  Created on: Aug 20, 2025
 *      Author: mcaplan
 */

#include "image_processing.h"
#include "camera_ov5640.h"
#include "fsl_debug_console.h"
#include "global_buffers.h"

void PROCESSING_MakeOptions(zoom_level_t zoom, uint8_t tile_size, demosaic_options_t* options){

	switch (zoom) {
		case zoom_level_1:
			options->roi_width = 1920;
			options->roi_height = 1920;
			options->start_col = 0;
			options->start_row = 0;
			break;
		case zoom_level_2:
			options->roi_width = 960;
			options->roi_height = 960;
			options->start_col = 480;
			options->start_row = 480;
			break;
		case zoom_level_3:
			options->roi_width = 480;
			options->roi_height = 480;
			options->start_col = 720;
			options->start_row = 720;
			break;
		default:
			options->roi_width = 1920;
			options->roi_height = 1920;
			options->start_col = 0;
			options->start_row = 0;
			break;
	}

	assert((options->roi_width % tile_size) == 0);
	options->tile_size = tile_size;

	options->source_buffer_stride = 1920;
}

static inline int clampi(int v, int low, int high){
	if( v < low ) return low;
	if (v > high) return high;
	return v;
}



void processOnePixel_CheapBilinear(int row_i,  int col_i, demosaic_options_t* options, uint8_t* r, uint8_t* g, uint8_t* b){
		//get options
		int source_buffer_stride = options->source_buffer_stride; //processing in place
		int tile_size = options->tile_size;
		int roi_width = options->roi_width;
		int roi_height = options->roi_height;
		int start_col = options->start_col;
		int start_row = options->start_row;

		// clamps the offsets so edges mirror themselves across the edge
		int left_offset = clampi(col_i - 1, 0, roi_width);
		int right_offset = clampi(col_i + 1, 0, roi_width);

		uint8_t* top_row = rowBuffer[2];
		uint8_t* center_row = rowBuffer[1];
		uint8_t* bottom_row = rowBuffer[0];
//
		// load relevant neighbors
		uint8_t center = center_row[col_i];
		uint8_t up = top_row[col_i]; 
		uint8_t right = center_row[right_offset];
		uint8_t down_right = bottom_row[right_offset];
		uint8_t down_left = bottom_row[left_offset]; 

		// even rows: BG..
		// even col, B 
		if((row_i & 1) == 0 && (col_i & 1) == 0){
			*b = center;
			*r = (down_left + down_right) >> 1;// diagonals
			*g = (right + up) >> 1; // straight
		}
		
		// odd col, G 
		else if((row_i & 1) == 0 && (col_i & 1) == 1){
			*g = center;
			*b = right;
			*r = up;
		}

		// odd cols: GR
		// even col: G
		else if((row_i & 1) == 1 && (col_i & 1) == 0){
			*g = center;
			*r = right;
			*b = up;

		}
		
		// odd col, R
		else if((row_i & 1) == 1 && (col_i & 1) == 1){
			*r = center;
			*b = (down_left + down_right) >> 1;// diagonals
			*g = (right + up) >> 1; // straight
		}
}


void PROCESSING_Debayer(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options){
	// set up tiles

	int source_buffer_stride = options->source_buffer_stride; //processing in place
	int tile_size = options->tile_size;
	int roi_width = options->roi_width;
	int roi_height = options->roi_height;
	int start_col = options->start_col;
	int start_row = options->start_row;

	for( int pixel_row = 0 ; pixel_row < roi_height; pixel_row++ ){

		// get row indices
		int row_i = start_row + pixel_row; //absolute row index

		int row_above_i = clampi(row_i + 1, 0, 1919);
		int row_below_i = clampi(row_i - 1, 0, 1919);
		
		// get row pointers
		uint8_t* row = source_buffer + row_i * source_buffer_stride;
		uint8_t* row_above = source_buffer + row_above_i * source_buffer_stride;
		uint8_t* row_below = source_buffer + row_below_i * source_buffer_stride;

		//copy to row-processing buffer (only copy roi width)
		memcpy(rowBuffer[0], row_below + start_col, roi_width);
		memcpy(rowBuffer[1], row + start_col, roi_width);
		memcpy(rowBuffer[2], row_above + start_col, roi_width);

		
		for( int pixel_col = 0; pixel_col < roi_width; pixel_col++ ){
			// inner loop
			
			// no offset needed because copied rows always start at 0
			int col_i = pixel_col;
			
			// declare output pixel values
			uint8_t r,g,b;
			
			// process and set rgb
			processOnePixel_CheapBilinear(row_i, col_i, options, &r, &g, &b);
			
			// write to processing buffer - downsampling will happen later
			// write as packed rgb888
			uint8_t* out_row = dest_buffer + row_i * source_buffer_stride * 3; //row_i has start col baked in TODO: this should be dest_buffer_stride (the same)
			uint8_t* out_px = out_row + (start_col + col_i ) * 3;
			out_px[0] = r;
			out_px[1] = g;
			out_px[2] = b;
		}
	}

}

// NOTES

// void processOnePixel_CheapBilinear(uint8_t* row, int row_i,  int col_i, demosaic_options_t* options, uint8_t* r, uint8_t* g, uint8_t* b){
// 		//get options
// 		int source_buffer_stride = options->source_buffer_stride; //processing in place
// 		int tile_size = options->tile_size;
// 		int roi_width = options->roi_width;
// 		int roi_height = options->roi_height;
// 		int start_col = options->start_col;
// 		int start_row = options->start_row;

// 		// clamps the offsets so edges mirror themselves across the edge
// 		// zero or 1 to toggle row stride offsets for bottom and top row
// //		int row_above_i = clampi(row_i + 1, 0, 1919);
// //		int row_below_i = clampi(row_i - 1, 0, 1919);

// 		 int row_up = (0 < row_i + 1 < 1919) ? 1 : 0;
// 		 int row_down = (0 < row_i - 1 < 1919) ? 1 : 0;

// 		// clamp col offsets to left
// 		int left_offset = clampi(col_i - 1, 0, 1919);
// 		int right_offset = clampi(col_i + 1, 0, 1919);
// //
// 		// load relevant neighbors
// 		uint8_t center = row[col_i];
// 		uint8_t up = row[col_i + row_up*source_buffer_stride]; 
// 		uint8_t right = row[right_offset];
// 		uint8_t down_right = row[right_offset - row_down*source_buffer_stride];
// 		uint8_t down_left = row[left_offset- row_down*source_buffer_stride]; 

// 		// even rows: BG..
// 		// even col, B 
// 		if((row_i & 1) == 0 && (col_i & 1) == 0){
// 			*b = center;
// 			*r = (down_left + down_right) >> 1;// diagonals
// 			*g = (right + up) >> 1; // straight
// 		}
		
// 		// odd col, G 
// 		else if((row_i & 1) == 0 && (col_i & 1) == 1){
// 			*g = center;
// 			*b = right;
// 			*r = up;
// 		}

// 		// odd cols: GR
// 		// even col: G
// 		else if((row_i & 1) == 1 && (col_i & 1) == 0){
// 			*g = center;
// 			*r = right;
// 			*b = up;

// 		}
		
// 		// odd col, R
// 		else if((row_i & 1) == 1 && (col_i & 1) == 1){
// 			*r = center;
// 			*b = (down_left + down_right) >> 1;// diagonals
// 			*g = (right + up) >> 1; // straight
// 		}
// }

//void PROCESSING_Debayer(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options){
//	// set up tiles
//
//	int source_buffer_stride = 1920; //processing in place
//	int tile_size = options->tile_size;
//	int roi_width = options->roi_width;
//	int roi_height = options->roi_height;
//	int start_col = options->start_col;
//	int start_row = options->start_row;
//
//	// tile increments
//	for( int tile_row = 0; tile_row < roi_height; tile_row += tile_size ){
//		for( int tile_col = 0; tile_col < roi_width; tile_col += tile_size ){
//
//			for( int pixel_row = 0 ; pixel_row < tile_size && pixel_row + tile_row < roi_height; pixel_row++ ){
//
//				// get row indices
//				int row_i = start_row + tile_row + pixel_row; //absolute row index
//				int row_above_i = clampi(row_i + 1, 0, 1919);
//				int row_below_i = clampi(row_i - 1, 0, 1919);
//
//				// load 3 rows
//				uint8_t* row = source_buffer + row_i * source_buffer_stride;
//				uint8_t* row_above = source_buffer + row_above_i * source_buffer_stride;
//				uint8_t* row_below = source_buffer + row_below_i * source_buffer_stride;
//
//				uint8_t out_row = dest_buffer + ((start_row + row_i) * source_buffer_stride + tile_col) * 3;
//
//				for( int pixel_col = 0; pixel_col < tile_size && pixel_col + tile_col < roi_width; pixel_col++ ){
//					// inner loop
//
//					int col_i = start_col + tile_col + pixel_col;
//
//					// clamps the offsets so edges mirror themselves across the edge
//					int left_offset = clampi(col_i - 1, 0, 1919);
//					int right_offset = clampi(col_i + 1, 0, 1919);
//
//					// load relevant neighbors
//					uint8_t center = row[col_i];
//					uint8_t up = row_above[col_i];
//					uint8_t down = row_below[col_i];
//					uint8_t right = row[right_offset];
//					uint8_t left = row[left_offset];
//					uint8_t up_center = row_above[col_i];
//					uint8_t up_right = row_above[right_offset];
//					uint8_t up_left = row_above[left_offset];
//					uint8_t down_center = row_below[col_i];
//					uint8_t down_right = row_below[right_offset];
//					uint8_t down_left = row_below[left_offset];
//
//					// declare output pixel values
//					uint8_t r,g,b;
//
//					// even rows: BG..
//					// even col, B
//					if((row_i & 1) == 0 && (col_i & 1) == 0){
//						b = center;
//						r = (up_left + up_right + down_left + down_right) >> 2;// diagonals
//						g = (left + right + up + down) >> 2; // straight
//					}
//
//					// odd col, G
//					else if((row_i & 1) == 0 && (col_i & 1) == 1){
//						g = center;
//						b = (left + right) >> 1;
//						r = (up + down) >> 1;
//					}
//
//					// odd cols: GR
//					// even col: G
//					else if((row_i & 1) == 1 && (col_i & 1) == 0){
//						g = center;
//						r = (left + right) >> 1;
//						b = (up + down) >> 1;
//
//					}
//
//					// odd col, R
//					else if((row_i & 1) == 1 && (col_i & 1) == 1){
//						r = center;
//						b = (up_left + up_right + down_left + down_right) >> 2;// diagonals
//						g = (left + right + up + down) >> 2; // straight
//					}
//
//					// write to processing buffer - downsampling will happen later if necessary
//					// write as packed rgb888
//					uint8_t* out_px = out_row + (pixel_col * 3);
//					out_px[0] = r;
//					out_px[1] = g;
//					out_px[2] = b;
//				}
//			}
//
//
//		}
//
//	}
//
//}
