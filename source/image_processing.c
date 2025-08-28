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
#include "jpeglib.h"

// create row buffer pointers for use in pointer rotations
uint8_t* buffered_row[3] = {rowBuffer[0], rowBuffer[1], rowBuffer[2]};


void PROCESSING_MakeOptions(zoom_level_t zoom, demosaic_options_t* options){

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

	options->source_buffer_stride = 1920;

	options->zoom = zoom;
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

		uint8_t* top_row = buffered_row[2];
		uint8_t* center_row = buffered_row[1];
		uint8_t* bottom_row = buffered_row[0];
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

void processOnePixel_Bilinear(int row_i,  int col_i, demosaic_options_t* options, uint8_t* r, uint8_t* g, uint8_t* b){

		// clamps the offsets so edges mirror themselves across the edge
		int left_offset = clampi(col_i - 1, 0, roi_width-1);
		int right_offset = clampi(col_i + 1, 0, roi_width-1);

		uint8_t* top_row = buffered_row[2];
		uint8_t* center_row = buffered_row[1];
		uint8_t* bottom_row = buffered_row[0];
//
		// load relevant neighbors
		uint8_t center = center_row[col_i];
		uint8_t right = center_row[right_offset];
		uint8_t left = center_row[left_offset];
		uint8_t up = top_row[col_i]; 
		uint8_t up_left= top_row[left_offset]; 
		uint8_t up_right = top_row[right_offset]; 
		uint8_t down = bottom_row[col_i];
		uint8_t down_right = bottom_row[right_offset];
		uint8_t down_left = bottom_row[left_offset]; 

		// even rows: BG..
		// even col, B 
		if((row_i & 1) == 0 && (col_i & 1) == 0){
			*b = center;
			*r = (down_left + down_right + up_left + up_right) >> 2;// diagonals
			*g = (right + up + left +down) >> 2; // straight
		}
		
		// odd col, G 
		else if((row_i & 1) == 0 && (col_i & 1) == 1){
			*g = center;
			*b = (right + left) >> 1;
			*r = (up + down) >> 1;
		}

		// odd cols: GR
		// even col: G
		else if((row_i & 1) == 1 && (col_i & 1) == 0){
			*g = center;
			*r = (right + left) >> 1;
			*b = (up + down) >> 1;

		}
		
		// odd col, R
		else if((row_i & 1) == 1 && (col_i & 1) == 1){
			*r = center;
			*b = (down_left + down_right + up_left + up_right) >> 2;// diagonals
			*g = (right + up + left + down) >> 2; // straight
		}
}


// FOR FULL RES IN PROCESSING BUFFER 1920x1920x3 bytes
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
		memcpy(buffered_row, row_below + start_col, roi_width);
		memcpy(rowBuffer[1], row + start_col, roi_width);
		memcpy(rowBuffer[2], row_above + start_col, roi_width);

		
		for( int pixel_col = 0; pixel_col < roi_width; pixel_col++ ){
			// inner loop
			
			// no offset needed because copied rows always start at 0
			int col_i = pixel_col;
			
			// declare output pixel values
			uint8_t r,g,b;
			
			// process and set rgb
			processOnePixel_Bilinear(row_i, col_i, options, &r, &g, &b);
			
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

// for scaled down buffering, ideally into display framebuffer directly
void PROCESSING_DebayerLiveView(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options){
	// set up tiles

	int source_buffer_stride = options->source_buffer_stride; //processing in place
	int tile_size = options->tile_size;
	int roi_width = options->roi_width;
	int roi_height = options->roi_height;
	int start_col = options->start_col;
	int start_row = options->start_row;
	zoom_level_t zoom = options->zoom;

	// for 480x480 dest buf, will be < 1920, +=8 at zoom level 1
	// 960 +=4 at zoom_level 2
	// 480 +=1 at zoom_level 3

	int dest_width = 480;
	int dest_height = 480;

	int row_num = 1;
	int offset_step = 1;

	switch(zoom){
		case zoom_level_1: 
			row_num = 2;
			offset_step = 8;
			break;
		case zoom_level_2: 
			row_num = 2;
			offset_step = 4;
			break;
		case zoom_level_3: 
		// default 1,1
			break;
		default:
			break;
	};

	// will get 2 adjacent rows for scaling down to live view at larger input sizes
	int dest_row_c = 0;
	
	for( int pixel_row = 0 ; pixel_row < dest_width/row_num; pixel_row++ ){
		
		int dest_col_c = 0;
		// get row indices
		for(int inner_row = 0; inner_row < row_num; inner_row++){
			int row_i = start_row + pixel_row*offset_step + inner_row; //absolute row index
			
			int row_above_i = clampi(row_i + 1, 0, 1919);
			int row_below_i = clampi(row_i - 1, 0, 1919);
			
			// get row pointers
			uint8_t* row = source_buffer + row_i * source_buffer_stride;
			uint8_t* row_above = source_buffer + row_above_i * source_buffer_stride;
			uint8_t* row_below = source_buffer + row_below_i * source_buffer_stride;

			//copy to row-processing buffer (only copy roi width)
			if(pixel_row == 0){
				// get all buffers
				memcpy(buffered_row[2], row_above + start_col, roi_width);
				memcpy(buffered_row[1], row + start_col, roi_width);
				memcpy(buffered_row[0], row_below + start_col, roi_width);
			}
			else{
				// rotate and get 1 buf
				uint8_t* temp = buffered_row[0];
				buffered_row[0] = buffered_row[1]; //center to bottom
				buffered_row[1] = buffered_row[2]; //top to center
				buffered_row[2] = temp;
	//
				memcpy(buffered_row[2], row_above + start_col, roi_width);
			}
			
			
			for( int pixel_col = 0; pixel_col < dest_height/row_num; pixel_col++ ){
				// inner loop
				
				for (int inner_col = 0; inner_col < row_num; inner_col++){
					
					int col_i = pixel_col*offset_step + inner_row;
					// no offset needed because copied rows always start at 0
					// declare output pixel values
					uint8_t r,g,b;
					
					// process and set rgb
					processOnePixel_CheapBilinear(row_i, col_i, options, &r, &g, &b);
					
					// write to processing buffer - downsampling will happen later
					// write as packed rgb888
					uint8_t* out_row = dest_buffer + dest_row_c * dest_width * 3; //row_i has start col baked in TODO: this should be dest_buffer_stride (the same)
					uint8_t* out_px = out_row + (dest_col_c * 3);
					
					out_px[0] = r;
					out_px[1] = g;
					out_px[2] = b;

					// increment dest_col_counter
					dest_col_c++;
				}
			}
			// increment dest_row_counter
			dest_row_c++;
		}
	}

}

// globalize cinfo
struct jpeg_compress_struct cinfo;
struct jpeg_error_mgr jerr;
/*
* debayer 3 rows at a time, straight to JPEG buffer
*/
// create jpeg
void PROCESSING_DebayerJPEG(uint8_t* source_buffer, demosaic_options_t* options){
	// set up variables
	int source_buffer_stride = options->source_buffer_stride; //processing in place
	// int tile_size = options->tile_size;
	int roi_width = options->roi_width;
	int roi_height = options->roi_height;
	int start_col = options->start_col;
	int start_row = options->start_row;


	unsigned char* jpg_buffer = p_frameBuffer;
	unsigned long jpg_size = sizeof(p_frameBuffer);

	JSAMPROW row_pointer[1] = {scanlineBuffer}; /* Output row buffer */

	cinfo.err = jpeg_std_error(&jerr);


	jpeg_create_compress(&cinfo);

	jpeg_mem_dest(&cinfo, &jpg_buffer, &jpg_size);

	cinfo.image_width = roi_width;
	cinfo.image_height = roi_height;
	cinfo.input_components = APP_FB_BPP;
	cinfo.in_color_space = JCS_RGB; 

	jpeg_set_defaults( &cinfo );

	// Absolutely minimal settings:
//	jpeg_set_quality(&cinfo, 50, TRUE);        // Very low quality
//	cinfo.dct_method = JDCT_IFAST;             // Fastest/least memory DCT
//	cinfo.smoothing_factor = 0;                 // No smoothing
//	cinfo.optimize_coding = FALSE;              // No optimization
//	cinfo.progressive_mode = FALSE;             // No progressive

	jpeg_start_compress( &cinfo, TRUE );

	while( cinfo.next_scanline < cinfo.image_height )
	{
		// get raw data rows
		int row_i = start_row + cinfo.next_scanline; //absolute row index
		
		int row_above_i = clampi(row_i + 1, 0, 1919);
		int row_below_i = clampi(row_i - 1, 0, 1919);
		
		// get row pointers
		uint8_t* row = source_buffer + row_i * source_buffer_stride;
		uint8_t* row_above = source_buffer + row_above_i * source_buffer_stride;
		uint8_t* row_below = source_buffer + row_below_i * source_buffer_stride;

		// buffer row neighborhood
		if(cinfo.next_scanline == 0){
				// get all buffers
				memcpy(buffered_row[2], row_above + start_col, roi_width);
				memcpy(buffered_row[1], row + start_col, roi_width);
				memcpy(buffered_row[0], row_below + start_col, roi_width);
			}
			else{
				// rotate and get 1 buf
				uint8_t* temp = buffered_row[0];
				buffered_row[0] = buffered_row[1]; //center to bottom
				buffered_row[1] = buffered_row[2]; //top to center
				buffered_row[2] = temp;
				memcpy(buffered_row[2], row_above + start_col, roi_width);
			}
			
		// create destination row pointer
		uint8_t* out_row = scanlineBuffer; //row_i has start col baked in TODO: this should be dest_buffer_stride (the same)
			
		// debayer 1 row to processing buffer
		for( int pixel_col = 0; pixel_col < roi_width; pixel_col++ ){
			// inner loop
			
			// no offset needed because copied rows always start at 0
			int col_i = pixel_col;
			
			// declare output pixel values
			uint8_t r,g,b;
			
			// process and set rgb, assumes rowbuffers are memcopyed to and rotated
			processOnePixel_CheapBilinear(row_i, col_i, options, &r, &g, &b);

			uint8_t* out_px = out_row + col_i * 3;
			out_px[2] = r;
			out_px[1] = g;
			out_px[0] = b;
		}

		// set row pointer and write; iterates the scanline counter
		jpeg_write_scanlines( &cinfo, row_pointer, 1 );
	}
	
	// finish compression when whole image is done
	jpeg_finish_compress(&cinfo);
	
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
