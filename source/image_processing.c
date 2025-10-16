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

demosaic_options_t db_options = {
	// set some default gains - gain = gain/255
	.r_gain = 300,
	.g_gain = 280,
	.b_gain = 360,
	.pixel_gain = 256

};


// TODO: should be named set_zoom really
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
		int roi_width = options->roi_width;

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
			*r = ((int)down_left + (int)down_right) >> 1;// diagonals
			*g = ((int)right + (int)up) >> 1; // straight
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
			*b = ((int)down_left + (int)down_right) >> 1;// diagonals
			*g = ((int)right + (int)up) >> 1; // straight
		}
}

processOnePixel_Luma(int row_i,  int col_i, demosaic_options_t* options, uint8_t* r, uint8_t* g, uint8_t* b){
	int roi_width = options->roi_width;

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
//			uint8_t up_left= top_row[left_offset];
//			uint8_t up_right = top_row[right_offset];
			uint8_t down = bottom_row[col_i];
//			uint8_t down_right = bottom_row[right_offset];
//			uint8_t down_left = bottom_row[left_offset];

			uint8_t luminance = 0;

			// even rows: BG..
			// even col, B
			if((row_i & 1) == 0 && (col_i & 1) == 0){
				luminance = ((int)right + (int)up + (int)left +(int)down) >> 2; // greens
			}

			// odd col, G
			else if((row_i & 1) == 0 && (col_i & 1) == 1){
				luminance = center; //green
			}

			// odd cols: GR
			// even col: G
			else if((row_i & 1) == 1 && (col_i & 1) == 0){
				luminance = center;

			}

			// odd col, R
			else if((row_i & 1) == 1 && (col_i & 1) == 1){
				luminance = ((int)right + (int)up + (int)left + (int)down) >> 2; // straight
			}

			*r = luminance;
			*g = luminance;
			*b = luminance;

}

processOnePixel_Raw(int row_i,  int col_i, demosaic_options_t* options, uint8_t* r, uint8_t* g, uint8_t* b){
			
			uint8_t* center_row = buffered_row[1]; //raw processing must always just use the first rowbuffer for center row
			uint8_t center = center_row[col_i];
			
			uint8_t _r = 0;
			uint8_t _g = 0;
			uint8_t _b = 0;

			// even rows: BG..
			// even col, B
			if((row_i & 1) == 0 && (col_i & 1) == 0){
				_b = center;
			}
			// odd col, G
			else if((row_i & 1) == 0 && (col_i & 1) == 1){
				_g = center; //green
			}
			// odd cols: GR
			// even col: G
			else if((row_i & 1) == 1 && (col_i & 1) == 0){
				_g = center;
			}

			// odd col, R
			else if((row_i & 1) == 1 && (col_i & 1) == 1){
				_r = center;
			}

			*r = _r;
			*g = _g;
			*b = _b;

}

void processOnePixel_Bilinear(int row_i,  int col_i, demosaic_options_t* options, uint8_t* r, uint8_t* g, uint8_t* b){

	int roi_width = options->roi_width;

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
			*r = ((int)down_left + (int)down_right + (int)up_left + (int)up_right) >> 2;// diagonals
			*g = ((int)right + (int)up + (int)left +(int)down) >> 2; // straight
		}
		
		// odd col, G 
		else if((row_i & 1) == 0 && (col_i & 1) == 1){
			*g = center;
			*b = ((int)right + (int)left) >> 1;
			*r = ((int)up + (int)down) >> 1;
		}

		// odd cols: GR
		// even col: G
		else if((row_i & 1) == 1 && (col_i & 1) == 0){
			*g = center;
			*r = ((int)right + (int)left) >> 1;
			*b = ((int)up + (int)down) >> 1;

		}
		
		// odd col, R
		else if((row_i & 1) == 1 && (col_i & 1) == 1){
			*r = center;
			*b = ((int)down_left + (int)down_right + (int)up_left + (int)up_right) >> 2;// diagonals
			*g = ((int)right + (int)up + (int)left + (int)down) >> 2; // straight
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

	// Combine gains beforehand (do this once, not per pixel)
	int combined_r_gain = (options->r_gain * options->pixel_gain) >> 8;  // Pre-scale down
	int combined_g_gain = (options->g_gain * options->pixel_gain) >> 8;
	int combined_b_gain = (options->b_gain * options->pixel_gain) >> 8;
	
	int dest_width = 480;
	int dest_height = 480;
	
	int row_num = 1;
	int offset_step = 1;
//	PRINTF("zoom_level: %d\r\n", zoom_level);
	
	// for 480x480 dest buf, will be < 1920, +=8 at zoom level 1
	// 960 +=4 at zoom_level 2
	// 480 +=1 at zoom_level 3
	switch(zoom){
		case zoom_level_1: 
			offset_step = 4;
			break;
		case zoom_level_2: 
			offset_step = 2;
			break;
		case zoom_level_3: 
		// default 1,1
			break;
		default:
			break;
	};

	// clear display buffer
	// memset(dest_buffer, 0x80, dest_height*dest_width*3);
	
	// set up outside loop
	int row_offset = 0;
	int col_offset = 0;
	
	// will get 2 adjacent rows for scaling down to live view at larger input sizes	
	for( int pixel_row = 0 ; pixel_row < dest_height; pixel_row++ ){
		
		// calculate an offset for sampling pattern at downscale
		if(offset_step > 1){ // if downsampling
			row_offset = pixel_row & 1 ; //0, 1, 0, 1, ...
		}
		
		// get row indices
		int row_i = start_row + pixel_row*offset_step + row_offset; //absolute row index
		
		int row_above_i = clampi(row_i - 1, 0, source_buffer_stride-1);
		int row_below_i = clampi(row_i + 1, 0, source_buffer_stride-1);
		
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
		 	uint8_t* temp = buffered_row[2];

		 	buffered_row[2] = buffered_row[1];
		 	buffered_row[1] = buffered_row[0];
		 	buffered_row[0] = temp;

		 	memcpy(buffered_row[0], row_below +start_col, roi_width);
		 }
		
		uint8_t* out_row = dest_buffer + pixel_row * dest_width * 3; 

		for( int pixel_col = 0; pixel_col < dest_width; pixel_col++ ){
			
			// set up column offsets for bayer pattern completeness 
			if(offset_step > 1){ // if downsampling
				col_offset = pixel_col & 1 ; //0, 1, 0, 1, ...
			}
			
			int col_i = pixel_col*offset_step + col_offset;
			
			// declare output pixel values
			uint8_t r,g,b = 0;
			
			// process and set rgb
			processOnePixel_CheapBilinear(row_i, col_i, options, &r, &g, &b);
			
			// write as packed rgb888
			uint8_t* out_px = out_row + (pixel_col * 3);
			
			// out_px[0] = clampi(b + (b >> options->zoom_gain_div), 0, 255);
			out_px[0] = clampi(( b * combined_b_gain ) >> 8, 0, 255);
			out_px[1] = clampi(( g * combined_g_gain ) >> 8, 0, 255);
			out_px[2] = clampi(( r * combined_r_gain ) >> 8, 0, 255);
			}		
	}

}

void PROCESSING_DebayerLiveView_Raw(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options){
	for(int i = 0; i< 480; i++){
		uint8_t* in_row = source_buffer + i*1920;
		
		memcpy(buffered_row[1], in_row, 480 );
		
		uint8_t* out_row = dest_buffer + i * 480 * 3;
		
		for(int j = 0; j< 480; j++){
			uint8_t r = 0;
			uint8_t g = 0;
			uint8_t b = 0;
			
			processOnePixel_Raw(i,j,options,&r,&g,&b);
			
			uint8_t* out_px = out_row + (j * 3);
			out_px[0] = b;
			out_px[1] = g;
			out_px[2] = r;


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
void PROCESSING_DebayerJPEG(uint8_t* source_buffer, unsigned char** jpg_buffer_ptr, unsigned long* jpg_size_ptr, demosaic_options_t* options){
	// set up variables
	int source_buffer_stride = options->source_buffer_stride; //processing in place
	// int tile_size = options->tile_size;
	int roi_width = options->roi_width;
	int roi_height = options->roi_height;
	int start_col = options->start_col;
	int start_row = options->start_row;

	int combined_r_gain = (options->r_gain * options->pixel_gain) >> 8;  // Pre-scale down
	int combined_g_gain = (options->g_gain * options->pixel_gain) >> 8;
	int combined_b_gain = (options->b_gain * options->pixel_gain) >> 8;

	// unsigned char* jpg_buffer = NULL;
	// unsigned long jpg_size = 0;
//	unsigned char* jpg_buffer = p_frameBuffer;
//	unsigned long jpg_size = sizeof(p_frameBuffer);
//	 unsigned long jpg_size = roi_height * roi_width * APP_FB_BPP;

	JSAMPROW row_pointer[1] = {scanlineBuffer}; /* Output row buffer */

	cinfo.err = jpeg_std_error(&jerr);


	jpeg_create_compress(&cinfo);

	jpeg_mem_dest(&cinfo, jpg_buffer_ptr, jpg_size_ptr);

	cinfo.image_width = roi_width;
	cinfo.image_height = roi_height;
	cinfo.input_components = APP_FB_BPP;
	cinfo.in_color_space = JCS_RGB; 

	jpeg_set_defaults( &cinfo );

	 jpeg_set_quality(&cinfo, 95, TRUE);        // High quality (85-95 range)
//	 cinfo.dct_method = JDCT_ISLOW;             // Most accurate DCT method
	 cinfo.dct_method = JDCT_FLOAT;             // Most accurate DCT method
	 cinfo.smoothing_factor = 0;                 // Keep at 0 (smoothing reduces quality)
//	 cinfo.optimize_coding = TRUE;               // Enable Huffman optimization
//	 cinfo.progressive_mode = TRUE;              // Optional: enables progressive JPEG

	jpeg_start_compress( &cinfo, TRUE );

	while( cinfo.next_scanline < cinfo.image_height )
	{
		// get raw data rows
		int row_i = start_row + cinfo.next_scanline; //absolute row index
		
		int row_above_i = clampi(row_i - 1, 0, source_buffer_stride-1);
		int row_below_i = clampi(row_i + 1, 0, source_buffer_stride-1);
		
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
				// rotate next row is row below aka row_i+1
				uint8_t* temp = buffered_row[2]; //throw away the top row and move down
				buffered_row[2] = buffered_row[1]; //move center up
				buffered_row[1] = buffered_row[0]; // move bottom up
				buffered_row[0] = temp; // rotate pointer

				memcpy(buffered_row[0], row_below + start_col, roi_width );
			}
			
		// create destination row pointer
		uint8_t* out_row = scanlineBuffer; 
			
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
			out_px[0] = clampi(( b * combined_b_gain ) >> 8, 0, 255);
			out_px[1] = clampi(( g * combined_g_gain ) >> 8, 0, 255);
			out_px[2] = clampi(( r * combined_r_gain ) >> 8, 0, 255);
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

// // FOR FULL RES IN PROCESSING BUFFER 1920x1920x3 bytes
// void PROCESSING_Debayer(uint8_t* source_buffer, uint8_t* dest_buffer, demosaic_options_t* options){
// 	// set up tiles

// 	int source_buffer_stride = options->source_buffer_stride; //processing in place
// 	int tile_size = options->tile_size;
// 	int roi_width = options->roi_width;
// 	int roi_height = options->roi_height;
// 	int start_col = options->start_col;
// 	int start_row = options->start_row;


// 	for( int pixel_row = 0 ; pixel_row < roi_height; pixel_row++ ){

// 		// get row indices
// 		int row_i = start_row + pixel_row; //absolute row index

// 		int row_above_i = clampi(row_i - 1, 0, 1919);
// 		int row_below_i = clampi(row_i + 1, 0, 1919);

// 		// get row pointers
// 		uint8_t* row = source_buffer + row_i * source_buffer_stride;
// 		uint8_t* row_above = source_buffer + row_above_i * source_buffer_stride;
// 		uint8_t* row_below = source_buffer + row_below_i * source_buffer_stride;

// 		//copy to row-processing buffer (only copy roi width)
// 		memcpy(buffered_row[0], row_below + start_col, roi_width);
// 		memcpy(buffered_row[1], row + start_col, roi_width);
// 		memcpy(buffered_row[2], row_above + start_col, roi_width);


// 		for( int pixel_col = 0; pixel_col < roi_width; pixel_col++ ){
// 			// inner loop

// 			// no offset needed because copied rows always start at 0
// 			int col_i = pixel_col;

// 			// declare output pixel values
// 			uint8_t r,g,b;

// 			// process and set rgb
// 			processOnePixel_Bilinear(row_i, col_i, options, &r, &g, &b);

// 			// write to processing buffer - downsampling will happen later
// 			// write as packed rgb888
// 			uint8_t* out_row = dest_buffer + row_i * source_buffer_stride * 3; //row_i has start col baked in TODO: this should be dest_buffer_stride (the same)
// 			uint8_t* out_px = out_row + (start_col + col_i ) * 3;
// 			out_px[0] = r;
// 			out_px[1] = g;
// 			out_px[2] = b;
// 		}
// 	}

// }
