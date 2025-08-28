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
#include "jpeglib.h"

#include "lcd_st7701.h"
#include "fsl_elcdif.h"
#include "global_buffers.h"
#include "storage_sd_device.h"
#include "fsl_elcdif.h"
#include "jpeg_interface.h"



/*Variables and fun things*/

#define TEST_PHOTO _T("/DCIM/000.jpg")


AT_NONCACHEABLE_SECTION(static FATFS g_fileSystem); /* File system object */
AT_NONCACHEABLE_SECTION(static FIL jpgFil);


/*Helpers*/

//void Browse_ELCDIF_Init(void)
//{
//    const elcdif_rgb_mode_config_t config = {
//        .panelWidth    = D_IMG_WIDTH,
//        .panelHeight   = D_IMG_HEIGHT,
//        .hsw           = APP_HSW,
//        .hfp           = APP_HFP,
//        .hbp           = APP_HBP,
//        .vsw           = APP_VSW,
//        .vfp           = APP_VFP,
//        .vbp           = APP_VBP,
//        .polarityFlags = APP_POL_FLAGS,
//        .bufferAddr    = (uint32_t)s_frameBuffer[0],
//        .pixelFormat   = kELCDIF_PixelFormatRGB888,
//        .dataBus       = kELCDIF_DataBus18Bit,
//    };
//
//#if (defined(APP_ELCDIF_HAS_DISPLAY_INTERFACE) && APP_ELCDIF_HAS_DISPLAY_INTERFACE)
//    BOARD_InitDisplayInterface();
//#endif
//    ELCDIF_RgbModeInit(LCDIF, &config);
//
//}


// mount the sd card as a file system
int MOUNT_SDCard(void)
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



    char cwd_buffer[256] = {0};  // Buffer to store current working directory
    // Get current working directory
    error = f_getcwd(cwd_buffer, sizeof(cwd_buffer));
    if (error == FR_OK) {
        PRINTF("Current directory: %s\n", cwd_buffer);
    } else {
        PRINTF("f_getcwd failed with error: %d\n", error);
    }


//    // Open file to check
//    error = f_open(&jpgFil, TEST_PHOTO, FA_OPEN_EXISTING);
//    if (error != FR_OK)
//    {
//        PRINTF("No demo jpeg file!\r\n");
//        return -4;
//    }
//
//    f_close(&jpgFil);

    return 0;
}

//void fill_framebuffer_gradient(uint8_t *framebuffer, int width, int height) {
////    for (int y = 0; y < height; ++y) {
////        for (int x = 0; x < width; ++x) {
////            int idx = (y * width + x) * 3;
////            framebuffer[idx + 0] = (uint8_t)((x * 255) / (width - 1));  // Red gradient across X
////            framebuffer[idx + 1] = (uint8_t)((y * 255) / (height - 1)); // Green gradient across Y
////            framebuffer[idx + 2] = 128;                                 // Constant blue
////        }
////    }
//}

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

//	fill_framebuffer_gradient((void*)&s_frameBuffer[0], APP_FB_WIDTH, APP_FB_HEIGHT);

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


void BROWSE_storeFile(){
	//	mount the filesystem
	FRESULT error;

	error = MOUNT_SDCard();
	 if (error != FR_OK)
		{
			PRINTF("mount SD faillll...");
		}

//	 open the file
	 	 error = f_open(&jpgFil, _T("test.jpg"), FA_OPEN_ALWAYS | FA_WRITE);
	 	if (error != FR_OK)
		{
			PRINTF("file write fail");
		}
	/* fill the frame buffer. */
		fill_framebuffer_gradient(s_frameBuffer[0], D_IMG_WIDTH, D_IMG_HEIGHT);

		jpeg_encode((void*)&s_frameBuffer[0], &jpgFil);

	 //	close the file
		f_close(&jpgFil);

}
