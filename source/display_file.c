/*
 * display_file.c
 *
 *  Created on: Jun 16, 2025
 *      Author: mcaplan
 */

#include "display_file.h"
#include "elcdif_rgb.h"

#include <stdio.h>
#include <string.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_cache.h"
#include "ff.h"
#include "diskio.h"
//#include "fsl_sd_disk.h"
#include "jpeglib.h"
//#include "display_support.h"
//#include "pin_mux.h"
//#include "clock_config.h"
//#include "board.h"
//#include "sdmmc_config.h"
//#include "fsl_gpio.h"

#include "lcd_st7701.h"
#include "fsl_elcdif.h"
#include "global_buffers.h"
#include "storage_sd_device.h"
#include "fsl_elcdif.h"



//// set this to 0 because we not using caches
//#define APP_CACHE_LINE_SIZE 0U
//
////same as elcdif_rgb.h for now
//#define APP_IMG_HEIGHT 480
//#define APP_IMG_WIDTH  480
//
//#define APP_FB_HEIGHT APP_IMG_HEIGHT
//#define APP_FB_WIDTH  APP_IMG_WIDTH
//#define APP_FB_BPP 3
//#define APP_FB_STRIDE_BYTE (APP_FB_WIDTH * APP_FB_BPP)
//#define FRAME_BUFFER_ALIGN 64


/*Variables and fun things*/

AT_NONCACHEABLE_SECTION(static FATFS g_fileSystem); /* File system object */
AT_NONCACHEABLE_SECTION(static FIL jpgFil);
//AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_frameBuffer[2][APP_IMG_HEIGHT][APP_IMG_WIDTH], FRAME_BUFFER_ALIGN);


/* This struct contains the JPEG decompression parameters */
static struct jpeg_decompress_struct cinfo;
/* This struct represents a JPEG error handler */
static struct jpeg_error_mgr jerr;

/*Helpers*/

void Browse_ELCDIF_Init(void)
{
    const elcdif_rgb_mode_config_t config = {
        .panelWidth    = APP_IMG_WIDTH,
        .panelHeight   = APP_IMG_HEIGHT,
        .hsw           = APP_HSW,
        .hfp           = APP_HFP,
        .hbp           = APP_HBP,
        .vsw           = APP_VSW,
        .vfp           = APP_VFP,
        .vbp           = APP_VBP,
        .polarityFlags = APP_POL_FLAGS,
        .bufferAddr    = (uint32_t)s_frameBuffer[0],
        .pixelFormat   = kELCDIF_PixelFormatRGB888,
        .dataBus       = kELCDIF_DataBus18Bit,
    };

#if (defined(APP_ELCDIF_HAS_DISPLAY_INTERFACE) && APP_ELCDIF_HAS_DISPLAY_INTERFACE)
    BOARD_InitDisplayInterface();
#endif
    ELCDIF_RgbModeInit(LCDIF, &config);

}


//need to decode a jpeg
void jpeg_decode(FIL *file, uint8_t *buffer)
{
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

//    // Step 3: Set up file input source (reads directly from file)
//	jpeg_stdio_src(&cinfo, &file);

    jpg_size = f_size(file);

        jpg_buffer = (unsigned char *)malloc(jpg_size + 2 * APP_CACHE_LINE_SIZE);
        if (jpg_buffer == NULL)
        {
            PRINTF("Error: memory allocation error\r\n");
            assert(false);
        }
    #if APP_CACHE_LINE_SIZE
        jpg_buffer_aligned = (void *)(SDK_SIZEALIGN((uint32_t)jpg_buffer, APP_CACHE_LINE_SIZE));
    #else
        jpg_buffer_aligned = jpg_buffer;
    #endif

        bytesRemain = jpg_size;
        read_pos    = jpg_buffer_aligned;

        DCACHE_CleanInvalidateByRange((uint32_t)jpg_buffer_aligned, jpg_size);

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
//    cinfo.out_color_space = ;
    /*
     * Resize to fit the screen, the actual resize rate is:
     * cinfo.scale_num / 8, the cinfo.scale_num must be in the range of 1 ~ 16
     */
    if ((cinfo.image_width * APP_FB_HEIGHT) > (cinfo.image_height * APP_FB_WIDTH))
    {
        cinfo.scale_num = APP_FB_WIDTH * 8 / cinfo.image_width;
    }
    else
    {
        cinfo.scale_num = APP_FB_HEIGHT * 8 / cinfo.image_height;
    }

    if (cinfo.scale_num < 1)
    {
        cinfo.scale_num = 1;
    }
    else if (cinfo.scale_num > 16)
    {
        cinfo.scale_num = 16;
    }

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
}


// mount the sd card as a file system
static int MOUNT_SDCard(void)
{
    FRESULT error;
    const TCHAR driverName[3U] = {SDDISK + '0', ':', '/'};

    const TCHAR directoryName[4U] = {'D', 'C', 'I', 'M'};


    // clear FATFS manually
    memset((void *)&g_fileSystem, 0, sizeof(g_fileSystem));

//	init the sd card
   error =  USB_DeviceMscDiskStorageInit();
   if(error != kStatus_USB_Success){
	   PRINTF("SD init failed");
	   return -1;
   }

    // Mount the driver
    error = f_mount(&g_fileSystem, driverName, 1);
    if (error != FR_OK)
    {
        PRINTF("Mount volume failed.\r\n");
        return -2;
    }

    error = f_chdrive((char const *)&driverName[0U]);
    if (error != FR_OK)
    {
        PRINTF("Change drive failed.\r\n");
        return -3;
    }

#define TEST_PHOTO _T("/DCIM/000.jpg")

    char cwd_buffer[256] = {0};  // Buffer to store current working directory
    // Get current working directory
    error = f_getcwd(cwd_buffer, sizeof(cwd_buffer));
    if (error == FR_OK) {
        PRINTF("Current directory: %s\n", cwd_buffer);
    } else {
        PRINTF("f_getcwd failed with error: %d\n", error);
    }


    // Open file to check
    error = f_open(&jpgFil, TEST_PHOTO, FA_OPEN_EXISTING);
    if (error != FR_OK)
    {
        PRINTF("No demo jpeg file!\r\n");
        return -4;
    }

    f_close(&jpgFil);

    return 0;
}



void DISPLAY_showStoredFile(){
//	mount the filesystem
	FRESULT error;

	error = MOUNT_SDCard();
	 if (error != FR_OK)
		    {
		        PRINTF("mount SD faillll...");
		    }


//	open the jpeg to somewhere
	error = f_open(&jpgFil, TEST_PHOTO, FA_READ);
	    if (error != FR_OK)
	    {
	        PRINTF("FILE OPEN FAILLLLL");
	    }

/* Clear the frame buffer. */
	memset(s_frameBuffer, 0, sizeof(s_frameBuffer));

//	decode the jpeg to an empty framebuffer
	jpeg_decode(&jpgFil, (void*)&s_frameBuffer[0]);

//	close the file
	f_close(&jpgFil);

// make sure to set up the lcd if
	BOARD_InitLcdifPixelClock();
//	BOARD_InitLcd();

//	and the display drivers
	GPIO_PinWrite(GPIO1, 9U, 1U);

//	???
	ST7701_SPIWrite(0x13, COMMAND);
	simpleDelay(1);

	PRINTF("LCDIF PHOTO DISPLAY start...\r\n");

	Browse_ELCDIF_Init();

	BOARD_EnableLcdInterrupt();

	ELCDIF_EnableInterrupts(LCDIF, kELCDIF_CurFrameDoneInterruptEnable);
	ELCDIF_RgbModeStart(LCDIF);

// set the lcdif next frame buf to pick up the decoded jpeg
	ELCDIF_SetNextBufferAddr(LCDIF, (uint32_t)s_frameBuffer[0]);

}

