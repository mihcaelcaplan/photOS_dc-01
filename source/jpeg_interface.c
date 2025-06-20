/*
 * jpeg_interface.c
 *
 *  Created on: Jun 19, 2025
 *      Author: mcaplan
 */

#include "jpeg_interface.h"
#include "jpeglib.h"
#include "global_buffers.h"

#include "fsl_debug_console.h"


/* This struct represents a JPEG error handler */
static struct jpeg_error_mgr jerr;


// decode a File and put it in the buffer.
	void jpeg_decode(FIL *file, uint8_t *buffer)
	{

		/* This struct contains the JPEG decompression parameters */
		static struct jpeg_decompress_struct cinfo;

		uint8_t *jpg_buffer;
	    uint8_t *jpg_buffer_aligned;
	    uint8_t *read_pos;
	    UINT jpg_size;
	    UINT bytesRead;
	    UINT bytesRemain;

	    // Decode JPEG Image
	    JSAMPROW row_pointer[1] = {0}; /* Output row buffer */
	    uint32_t row_stride     = 0;   /* physical row width in image buffer */

	    // Step 1: allocate and initialize JPEG decompression object
	    cinfo.err = jpeg_std_error(&jerr);

	    // Step 2: Initialize the JPEG decompression object
	    jpeg_create_decompress(&cinfo);

	    jpg_size = f_size(file);

		jpg_buffer = (unsigned char *)malloc(jpg_size + 2 * APP_CACHE_LINE_SIZE);
		if (jpg_buffer == NULL)
		{
			PRINTF("Error: memory allocation error\r\n");
			assert(false);
		}
		jpg_buffer_aligned = jpg_buffer;

		bytesRemain = jpg_size;
		read_pos    = jpg_buffer_aligned;

		while (bytesRemain > 0)
		{
			f_read(file, read_pos, bytesRemain, &bytesRead);
			bytesRemain -= bytesRead;
			read_pos += bytesRead;
		}

		jpeg_mem_src(&cinfo, jpg_buffer_aligned, jpg_size);

		// Step 4: read image parameters with jpeg_read_header()
		jpeg_read_header(&cinfo, true);

	    // Step 4: set parameters for decompression
	    cinfo.dct_method = JDCT_FLOAT;
	    cinfo.output_gamma = 1;
	    cinfo.dither_mode = JDITHER_NONE;
	    cinfo.do_fancy_upsampling = FALSE;   // Simpler processing
	    cinfo.do_block_smoothing = FALSE;    // No smoothing



	    //    cinfo.out_color_space = ;
	    /*
	     * Resize to fit the screen, the actual resize rate is:
	     * cinfo.scale_num / 8, the cinfo.scale_num must be in the range of 1 ~ 16
	     */
	//    if ((cinfo.image_width * APP_FB_HEIGHT) > (cinfo.image_height * APP_FB_WIDTH))
	//    {
	//        cinfo.scale_num = APP_FB_WIDTH * 8 / cinfo.image_width;
	//    }
	//    else
	//    {
	//        cinfo.scale_num = APP_FB_HEIGHT * 8 / cinfo.image_height;
	//    }
	//
	//    if (cinfo.scale_num < 1)
	//    {
	//        cinfo.scale_num = 1;
	//    }
	//    else if (cinfo.scale_num > 16)
	//    {
	//        cinfo.scale_num = 16;
	//    }

	    // Step 5: start decompressor
	    jpeg_start_decompress(&cinfo);

	    row_stride = APP_FB_STRIDE_BYTE;

	    /* Place the output image to the center of the screen. */
	    buffer += row_stride * ((APP_FB_HEIGHT - cinfo.output_height) / 2);
	    buffer += APP_FB_BPP * ((APP_FB_WIDTH - cinfo.output_width) / 2);

	    while (cinfo.output_scanline < cinfo.output_height)
	    {
	        row_pointer[0] = &buffer[cinfo.output_scanline * row_stride];
	        jpeg_read_scanlines(&cinfo, row_pointer, 1);
	    }

	    // Step 6: Finish decompression
	    jpeg_finish_decompress(&cinfo);

	    // Step 7: Release JPEG decompression object
	    jpeg_destroy_decompress(&cinfo);

	    free(jpg_buffer);
	}

// encode the contents of a buffer and put it in a file
void jpeg_encode(uint8_t *buffer, FIL *file){

	static struct jpeg_compress_struct cinfo;

	unsigned char *jpg_buffer;
	unsigned char *write_pos;

	UINT bytesWritten;
	UINT bytesRemain;

	unsigned long jpg_size = 0; //let the jpeg lib tell us the size

	JSAMPROW row_pointer[1] = {0}; /* Output row buffer */

	cinfo.err = jpeg_std_error( &jerr );

	jpeg_create_compress(&cinfo);


	jpeg_mem_dest(&cinfo, &jpg_buffer, &jpg_size);

	cinfo.image_width = APP_FB_WIDTH;
	cinfo.image_height = APP_FB_HEIGHT;
	cinfo.input_components = APP_FB_BPP;
	cinfo.in_color_space = JCS_RGB; // TODO: this may change from the camera

	jpeg_set_defaults(&cinfo );

	jpeg_start_compress(&cinfo, TRUE );

	while( cinfo.next_scanline < cinfo.image_height )
	{
		row_pointer[0] = &buffer[ cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
		jpeg_write_scanlines( &cinfo, row_pointer, 1 );
	}

    jpeg_finish_compress(&cinfo);

//	now the mem dest should be filled so just write that to a file
	bytesRemain = jpg_size; //updated with size since we gave a pointer to mem_dest
	write_pos = jpg_buffer; // pointer to beginning of the dest buf

	//	write buffer to a file
	while (bytesRemain > 0)
		{
			f_write(file, write_pos, bytesRemain, &bytesWritten);
			bytesRemain -= bytesWritten;
			write_pos += bytesWritten;
		}

	jpeg_destroy_compress( &cinfo );
	free(jpg_buffer);

}



