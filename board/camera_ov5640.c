/*
 * camera_ov5640.c
 *
 *  Created on: Jun 11, 2025
 *      Author: mcaplan
 */

#include "camera_ov5640.h"
#include "fsl_lpi2c.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "global_buffers.h"
#include "fsl_csi.h"
#include "fsl_debug_console.h"
#include "fsl_elcdif.h"
#include "lcd_st7701.h"

#define CAMERA_I2C LPI2C1
#define OV5460_I2C_ADDRESS_WRITE 0x3C
#define OV5460_I2C_ADDRESS_READ 0x3C
#define RESET_GPIO 14U


int lcdirqc = 0;
int csiirqc = 0;

// buffer_t* buffer_to_display = NULL; // variable to use inside the irq
// //buffer_t* last_buffer = NULL; // variable to use inside the irq
// uint32_t last_buffer_addr;

// flags/buffer pointers
//volatile uint32_t* camera_dirty_buffer0;
//volatile uint32_t* camera_dirty_buffer1;
//volatile uint32_t* camera_empty_buffer;
//volatile uint32_t* camera_clean_buffer;
//volatile uint32_t* display_fresh_buffer;
//volatile uint32_t* display_stale_buffer;
//volatile uint32_t* display_empty_buffer;


void LCDIF_IRQHandler(void){
	uint32_t flags = (LCDIF->CTRL1 & ELCDIF_CTRL1_IRQ_MASK);
    LCDIF->CTRL1_CLR = ELCDIF_CTRL1_IRQ_MASK; //  clear all int masks
	
    if (flags & kELCDIF_VsyncEdge)
	{
		display_buffer_manager.vsync_edge(&display_buffer_manager);
	}
	if (flags & kELCDIF_CurFrameDone)
	{
		display_buffer_manager.cur_frame_done(&display_buffer_manager);
	}
	
	lcdirqc++;
    __DSB();
}

volatile uint32_t* last_fb_done = c_frameBuffer[0];
volatile uint32_t* last_fb_done_i = 0;

void CSI_IRQHandler(void){
	uint32_t status = CSI_GetStatusFlags(CSI);

	if(status & kCSI_StartOfFrameFlag){
		CSI->SR |= CSI_SR_SOF_INT_MASK; //clear flag by writing 1
		camera_buffer_manager.start_of_frame(&camera_buffer_manager);
	}

	if(status & kCSI_RxBuffer0DmaDoneFlag){
		CSI->SR |= CSI_SR_DMA_TSF_DONE_FB1_MASK; //clear flag by writing 1
		camera_buffer_manager.dma_done(&camera_buffer_manager, 0);
	}
	if(status & kCSI_RxBuffer1DmaDoneFlag){
		CSI->SR |= CSI_SR_DMA_TSF_DONE_FB2_MASK; //clear flag by writing 1
		camera_buffer_manager.dma_done(&camera_buffer_manager, 1);
		
	}
	
	csiirqc++;
	__DSB();
}

// all the addresses are 16 bit
//uint32_t OV5640_I2CRead8(uint16_t regAddr){
//
//	uint8_t rcv = 0;
//
//	//	send the register address to read
//	LPI2C_MasterStart(CAMERA_I2C, OV5460_I2C_ADDRESS_WRITE, kLPI2C_Write );
//	LPI2C_MasterSend(CAMERA_I2C, &regAddr, 2);
//	LPI2C_MasterStop(CAMERA_I2C);
//
////	restart (turn around bus) and receive the data
//	LPI2C_MasterStart(CAMERA_I2C, OV5460_I2C_ADDRESS_READ, kLPI2C_Read );
//	LPI2C_MasterReceive(CAMERA_I2C, &rcv, 1);
//	LPI2C_MasterStop(CAMERA_I2C);
//
//	return rcv;
//
//}

// all the addresses are 16 bit, reg is 8 bit wide
void OV5640_I2CWrite8(uint16_t regAddr, uint32_t data){
	LPI2C_MasterStart(CAMERA_I2C, OV5460_I2C_ADDRESS_WRITE, kLPI2C_Write );

	uint8_t low_byte = (regAddr >> 8) & 0xFF;
	uint8_t high_byte = regAddr & 0xFF;

//	send address (swapped)
	LPI2C_MasterSend(CAMERA_I2C, &low_byte, 1);
	LPI2C_MasterSend(CAMERA_I2C, &high_byte, 1);
//	send data
	LPI2C_MasterSend(CAMERA_I2C, &data, 1);

	LPI2C_MasterStop(CAMERA_I2C);

}

void registerInit(void){
// sysclk from pad
OV5640_I2CWrite8(0x3103, 0x11);

// software reset and wait
OV5640_I2CWrite8(0x3008, 0x82);
simpleDelay(5);


// init the settings we want
// gonna skip the driver code from the app notes and configure a very minimal data path that just passes off raw 8 bit data from the sensor
// Power & Clock Tree:
// Wake from standby/software reset
    OV5640_I2CWrite8(0x3008, 0x42); // put in standby to set everything up

// sysclk from pll
	OV5640_I2CWrite8(0x3103, 0x03);


// IO pad control 1 = out, 0 = in
    // Bit[7] Bit[6] Bit[5] Bit[4] Bit[3:0]
    // FREX   VSYNC  HREF   PCLK   D[9:6]
    OV5640_I2CWrite8(0x3017, 0xff); //frex on (not use), d[9:6] out
    // Bit[7:2] Bit[1] Bit[0]
    // D[5:0]   GPIO1  GPIO0
    OV5640_I2CWrite8(0x3018, 0xff); //d[5:2] high, no gpio

// PLL settings for pixclk out
    OV5640_I2CWrite8(0x3034, 0x18); // set to 8 bit mode

// pixclk = mclk/root_div/pre-div)*multiply/sysclk divider
// (24Mhz/2/4)*100/6 = 50Mhz  idek anymore
    OV5640_I2CWrite8(0x3035, 0x41);  // [7:4] sysclk divider, [3:0] mipi divider
    OV5640_I2CWrite8(0x3036, 0xA0);  // multiplier (Can be any integer from 4~127 and only even integers from 128~252)
    OV5640_I2CWrite8(0x3037, 0x01);  // [4] root divider (1 = root/2), [3:0] pre-divider (1,2,3,4,6,8)

	OV5640_I2CWrite8(0x3108, 0x01); //set system dividers


	OV5640_I2CWrite8(0x3000, 0x00); //take core blocks out of reset
	OV5640_I2CWrite8(0x3002, 0x1C); //put jfifo, sfifo, jpg in reset
	OV5640_I2CWrite8(0x3004, 0xFF); // turn MCU clocks on
	OV5640_I2CWrite8(0x3006, 0xc3); // disable clock of JPEG2x, JPEG
	OV5640_I2CWrite8(0x300e, 0x58); //mipi off, DVP on (should be default)



// Timing Generator:
// FPS = pixclk/(line_width*total lines)
/*
 * OV5640 Sensor Windowing Configuration
 * =====================================
 *
 * Full Sensor Array: 2624×1952 pixels
 *
 *     0                    1312                    2623
 *     ┌─────────────────────┼─────────────────────┐  0
 *     │                     │                     │
 *     │        ┌────────────┼────────────┐        │
 *     │        │  (x_st,y_st)            │        │
 *     │        │            │            │        │
 *     │        │            │            │        │
 *     ├────────┼────────────●────────────┼────────┤  976 (mid_y)
 *     │        │        (mid_x,mid_y)    │        │
 *     │        │            │            │        │
 *     │        │            │  (x_end,y_end)      │
 *     │        └────────────┼────────────┘        │
 *     │                     │                     │
 *     └─────────────────────┼─────────────────────┘  1951
 *                           │
*                   width = x_end - x_st + 1
*                   height = y_end - y_st + 1
 *
 * Centering Calculations:
 * ----------------------
 * mid_x = 1312           // (2624 - 1) / 2
 * mid_y = 976            // (1952 - 1) / 2
 *
 * x_st = mid_x - (width/2)
 * y_st = mid_y - (height/2)
 * x_end = x_st + width - 1
 * y_end = y_st + height - 1
 */

uint16_t x_start = 1312 - ( APP_FB_WIDTH/2 );
uint16_t x_end = x_start + APP_FB_WIDTH - 1;
uint16_t y_start = 976 -  (APP_FB_HEIGHT/2 );
uint16_t y_end = y_start + APP_FB_HEIGHT - 1;

uint16_t width = APP_FB_WIDTH;
uint16_t height = APP_FB_HEIGHT;

// set up image window constraints
    OV5640_I2CWrite8(0x3800, (x_start >> 8) & 0xFF); // X start [11:8]
    OV5640_I2CWrite8(0x3801, x_start & 0xFF); // X start [7:0]
    OV5640_I2CWrite8(0x3802, (y_start >> 8) & 0xFF); // Y start [11:8]
    OV5640_I2CWrite8(0x3803, y_start & 0xFF); // Y start [7:0]
    OV5640_I2CWrite8(0x3804, (x_end >> 8) & 0xFF); // X end [11:8]
    OV5640_I2CWrite8(0x3805, x_end & 0xFF); // X end [7:0]
    OV5640_I2CWrite8(0x3806, (y_end >> 8) & 0xFF); // Y end [11:8]
    OV5640_I2CWrite8(0x3807, y_end & 0xFF); // Y end [7:0]

//    TODO: add in DVP output settings :) probably mirror the others
    OV5640_I2CWrite8(0x3808, (width >> 8) & 0xFF); //DVP output horizontal width [11:8]
    OV5640_I2CWrite8(0x3809, width & 0xFF); //DVP output horizontal width [7:0]
    OV5640_I2CWrite8(0x380A, (height >> 8) & 0xFF); //DVP output vertical height [11:8]
    OV5640_I2CWrite8(0x380B, height & 0xFF); //DVP output vertical height [7:0]

//    OV5640_I2CWrite8(0x380C, ((width+221) >> 8) &0XFF ) ; // total horizontal size [11:8]
//    OV5640_I2CWrite8(0x380D, (width+221) & 0XFF ); // total horizontal size [7:0]
//    OV5640_I2CWrite8(0x380E, ((height+17) >> 8) &0XFF ); // total vertical size[11:8]
//    OV5640_I2CWrite8(0x380F, (height+17) & 0xFF ); // total vertical size[ 7:0]

    OV5640_I2CWrite8(0x4709, (uint8_t)50); // vsync line width 10
    OV5640_I2CWrite8(0x4713, 0x03); // JPEG (compression?) mode 3

    
    // these only operate in the ISP stage, i think not if i disable it all
    // 0x3810 //horizontal offset [11:8]
    // 0x3811 //horizontal offset [7:0]
    // 0x3812 //vertical offset [11:8]
    // 0x3813 //vertical offset [7:0]


//  X/Y increment (binning/subsampling)
    OV5640_I2CWrite8(0x3814,0x11); // X [7:4]odd subsampling increment, [3:0] even subsampling increment
    OV5640_I2CWrite8(0x3815,0x11);  // Y [7:4]odd subsampling increment, [3:0] even subsampling increment

// Timing control (flip/mirror)
    OV5640_I2CWrite8(0x3820,0x41);  // [2] ISP flip, [1] sensor flip
    OV5640_I2CWrite8(0x3821, 0x02);  // [5] jpeg en, [2]isp mirror, [1] sensor mirror, [0] horizontal binning enable (vertical binning auto-enable on Y inc.)

// Output Interface:
// Raw format output (bypass ISP)
    OV5640_I2CWrite8(0x4300, 0xf8); //raw, default pixel order

// ISP disable
    OV5640_I2CWrite8(0x5000, 0x00); // all isp off
    OV5640_I2CWrite8(0x5001, 0x00);

    OV5640_I2CWrite8(0x3008, 0x02);// wake up from standby
}

void OV5640_Init(void){
	GPIO_PinWrite(GPIO1,RESET_GPIO, 1U);

/* Enable the master function and disable the slave function. */
	LPI2C_MasterEnable(CAMERA_I2C, true);
	LPI2C_SlaveEnable(CAMERA_I2C, false);

	//init registers
	registerInit();
	simpleDelay(5);

//	test pattern
//	 OV5640_I2CWrite8(0x503d, 0x80);
//	 OV5640_I2CWrite8(0x4741, 0x07);
}



void CAMERA_Init(void){

//	// 1ms reset pulse
		gpio_pin_config_t ov5640_reset = {
				kGPIO_DigitalOutput,
				1U,
				kGPIO_NoIntmode
		};
	//	set up gpio
		GPIO_PinInit(GPIO1, RESET_GPIO, &ov5640_reset);
		GPIO_PinWrite(GPIO1,RESET_GPIO, 0U);
//
////	init the CSI clock
//	/* CSI MCLK select 24M. */
//	    /*
//	     * CSI clock source:
//	     *
//	     * 00 derive clock from osc_clk (24M)
//	     * 01 derive clock from PLL2 PFD2
//	     * 10 derive clock from pll3_120M
//	     * 11 derive clock from PLL3 PFD1
//	     */
	    CLOCK_SetMux(kCLOCK_CsiMux, 0);
//	    /*
//	     * CSI clock divider:
//	     *
//	     * 000 divide by 1
//	     * 001 divide by 2
//	     * 010 divide by 3
//	     * 011 divide by 4
//	     * 100 divide by 5
//	     * 101 divide by 6
//	     * 110 divide by 7
//	     * 111 divide by 8
//	     */
	    CLOCK_SetDiv(kCLOCK_CsiDiv, 0);
//
//
////	init csi which will ungate the clock
csi_config_t ov5640_config  = {
	480, //width
	480, //height
	kCSI_HsyncActiveHigh | kCSI_DataLatchOnRisingEdge | kCSI_VsyncActiveLow,
	1, // byte per pixel (raw 8 bits per pixel)
	480*1, //linepitch in bytes
	kCSI_GatedClockMode, // use hsync
	kCSI_DataBus8Bit, //data bus width
};

//	init the camera (turning on mclk so that the chip wakes up)
	CSI_Reset(CSI);

	CSI_Init(CSI, &ov5640_config);// register the frame buffer addresses
	CSI->CR2  = (CSI->CR2& ~!CSI_CR2_BTS_MASK) | 0x2 << CSI_CR2_BTS_SHIFT;




	CSI_ReflashFifoDma(CSI, kCSI_RxFifo);


	CSI_SetRxBufferAddr(CSI, 0, (uint32_t)c_frameBuffer[0]);
	CSI_SetRxBufferAddr(CSI, 1, (uint32_t)c_frameBuffer[1]);


	NVIC_ClearPendingIRQ(CSI_IRQn);
	EnableIRQ(CSI_IRQn);
//
	//	//	configure the control regs once mclk up from csi init (24 MHz)
	OV5640_Init();

	memset(&c_frameBuffer[0], 0, sizeof(c_frameBuffer[0]));



///* Initialize the LCD_DISP. */
//   /*
//    * The desired output frame rate is 60Hz. So the pixel clock frequency is:
//    * (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2) * 60 = 9.2M.
//    *
//    * Here use the video pll (93MHz) as pixel clock source,
//    * pixel clock = F_video_pll / (prediv + 1) / (div + 1) = 93 / 5 / 2 = 9.3M.
//    */
uint32_t videoPllFreq;
//   videoPllFreq = CLOCK_GetPllFreq(kCLOCK_PllVideo);
//   PRINTF("video pll freq: %i \r\n");
//
//   if (videoPllFreq != 93000000)
//   {
//       PRINTF("Error: Invalid LCDIF pixel clock source.\r\n");
//       while (1)
//           ;
//   }
//
//   /*
//    * 000 derive clock from PLL2
//    * 001 derive clock from PLL3 PFD3
//    * 010 derive clock from PLL5
//    * 011 derive clock from PLL2 PFD0
//    * 100 derive clock from PLL2 PFD1
//    * 101 derive clock from PLL3 PFD1
////    */
   CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

   CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 4);

   CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);

 /* Reset the LCDIF, this is only used for flash target project debug.
 *
 * Hardware reset through debugger could not reset the ELCDIF, when reset
 * through debugger, the previous ELCDIF status retains, especially the
 * interrupt pending status. So we need to reset the ELCDIF before enabling
 * interrupt in NVIC. If the application only works with POR (Power on reset),
 * then this could be removed.
 */
   CLOCK_EnableClock(kCLOCK_Lcd);
   CLOCK_EnableClock(kCLOCK_LcdPixel);
   ELCDIF_Reset(LCDIF);
//   CLOCK_DisableClock(kCLOCK_LcdPixel);
//   CLOCK_DisableClock(kCLOCK_Lcd);
//

//	// 2. Reset the LCDIF block
//	LCDIF->CTRL_SET = LCDIF_CTRL_SFTRST_MASK;
//	LCDIF->CTRL_SET = LCDIF_CTRL_CLKGATE_MASK;
//	LCDIF->CTRL_CLR = LCDIF_CTRL_SFTRST_MASK;
//	LCDIF->CTRL_CLR = LCDIF_CTRL_CLKGATE_MASK;


//		initdisplay :)
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
			.bufferAddr    = NULL,
			.pixelFormat   = kELCDIF_PixelFormatRGB888,
			.dataBus       = kELCDIF_DataBus18Bit,
		};

	ELCDIF_RgbModeInit(LCDIF, &config);
//
////	disable the block
//	LCDIF->CTRL_CLR = LCDIF_CTRL_DOTCLK_MODE_MASK;
//
	NVIC_ClearPendingIRQ(LCDIF_IRQn);
	NVIC_SetPriority(LCDIF_IRQn, 3);
	EnableIRQ(LCDIF_IRQn);
//
//	//add recover on underflow
	LCDIF->CTRL1_SET = LCDIF_CTRL1_RECOVER_ON_UNDERFLOW_MASK;
//
//	NVIC_ClearPendingIRQ(LCDIF_IRQn);
//	NVIC_SetPriority(LCDIF_IRQn, 3);
//	EnableIRQ(LCDIF_IRQn);
//
}

void processForDisplay(uint32_t* source_buffer, uint32_t* dest_buffer){
    // 3 byte per pixel packed format, 24 bpp
    // set processing flag with buffer num
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

//    memset(&s_frameBuffer[processBuffer], 0, sizeof(s_frameBuffer[processBuffer]));

    for(int i=0; i < 480; i++){
        for(int j=0; j < 480; j++){
            int src_i = i*480 + j; // line number* line width + column number
            int dst_i = i*480*3 + 3*j; // line number* line width*3 + column number*3

            uint32_t pixel_value = source_buffer[src_i];

            // Reset RGB for each pixel
            r = 0;
            g = 0;
            b = 0;

            if(i % 2 == 0) { // Even rows: BGBGBG...
                if(j % 2 == 0) { // B pixel
                    b = pixel_value;
                } else { // G pixel
                    g = pixel_value;
                }
            } else { // Odd rows: GRGRGR...
                if(j % 2 == 0) { // G pixel
                    g = pixel_value;
                } else { // R pixel
                    r = pixel_value;
                }
            }

            dest_buffer[dst_i + 0] = r;
            dest_buffer[dst_i + 1] = g;
            dest_buffer[dst_i + 2] = b;
        }
    }
}

void fill_framebuffer_gradient(uint8_t *framebuffer, int width, int height) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 3;
            framebuffer[idx + 0] = (uint8_t)((x * 255) / (width - 1));  // Red gradient across X
            framebuffer[idx + 1] = (uint8_t)((y * 255) / (height - 1)); // Green gradient across Y
            framebuffer[idx + 2] = 128;                                 // Constant blue
        }
    }
}

void LCDtest(void){

////	enable lcdif and interrupts
//	ELCDIF_RgbModeStart(LCDIF);
//	ELCDIF_EnableInterrupts(LCDIF, kELCDIF_VsyncEdgeInterruptEnable | kELCDIF_CurFrameDoneInterruptEnable);
//
////	display on
//	ST7701_SPIWrite(0x13, COMMAND); // normal display mode on
//
//
//	// fill a display framebuffer with test pattern
//
////	"wait" for a stale buffer (maybe a
//	while(pool_empty(&stale_screen_buffers)){
//		pool_put(&stale_screen_buffers, &screen_buffers[0]);
//	}
////
////	// get a stale buffer and add data to it
//	buffer_t* buffer_to_process = pool_get(&stale_screen_buffers);
//	fill_framebuffer_gradient(s_frameBuffer[0], 480, 480);
//
//	pool_put(&fresh_screen_buffers, buffer_to_process);
//
////	wait
//	while(!pool_empty(&fresh_screen_buffers)){
//		__NOP();
//	}
//
//	// when the pool is empty we can turn on the screen
//	GPIO_PinWrite(GPIO1, 9U, 1U); //display backlight enable

}

void CAMERA_Run(void){
//	ST7701_SPIWrite(0x13, COMMAND); // normal display mode on
//
////  set up buffers
//	display_fresh_buffer = NULL;
//	display_stale_buffer = s_frameBuffer[0];
//	display_empty_buffer = s_frameBuffer[1];
//
//	LCDIF->CUR_BUF = display_empty_buffer; // so that display data out won't update until the stale is overwritten and made fresh
//
//	//	enable lcdif and interrupts
//	ELCDIF_RgbModeStart(LCDIF);
//	ELCDIF_EnableInterrupts(LCDIF, kELCDIF_VsyncEdgeInterruptEnable | kELCDIF_CurFrameDoneInterruptEnable); //seems like don't need vsync
//
//
//// enable csi and interrupts
////  set up buffers
//	camera_dirty_buffer0 = c_frameBuffer[0];
//	camera_dirty_buffer1 = c_frameBuffer[1];
//	camera_empty_buffer = c_frameBuffer[2];
//	camera_clean_buffer = NULL;
//
//	CSI->DMASA_FB1 = camera_dirty_buffer0;
//	CSI->DMASA_FB2 = camera_dirty_buffer1;
//
////	csi enable base address witch on
//	CSI->CR18 |= CSI_CR18_BASEADDR_SWITCH_EN_MASK | CSI_CR18_BASEADDR_SWITCH_SEL(1);
//
////	start csi subsystem
//	CSI->CR1 |= CSI_CR1_FB1_DMA_DONE_INTEN_MASK; // enable interrupt fb1 full
//	CSI->CR1 |= CSI_CR1_FB2_DMA_DONE_INTEN_MASK; // enable interrupt fb2 full
//	CSI->CR1 |= CSI_CR1_SOF_INTEN_MASK;; // enable interrupt fb2 full
//	CSI->CR3 |= CSI_CR3_DMA_REQ_EN_RFF_MASK; 	 // enable dma reqs from receive fifo
//	CSI->CR18|= CSI_CR18_CSI_ENABLE_MASK; 		 // enable csi module
//
//	GPIO_PinWrite(GPIO1, 9U, 1U); //display backlight enable
//	while(1){
//		//display driven event loop
//
//	//	wait for a "clean" holdout from the triple buffer
//		while(camera_clean_buffer == NULL){
//			__NOP();
//		}
//
//		//	wait for lcd screen to have displayed something
//		while(display_empty_buffer==NULL){ // | <- filled in frame boundary
//			__NOP();
//		}
//
//	//	now i have a clean camera buffer and an stale screen buffer to be refreshed
//	//	should probably have some size guards in here
//		processForDisplay((uint32_t)camera_clean_buffer, (uint32_t)display_empty_buffer);
//
//
//		camera_empty_buffer = camera_clean_buffer; //  (empty <- clean) |  -> dirty on frame boundary
//		camera_clean_buffer = NULL; // clean <- NULL  | -> clean on frame boundary
//
//	//	after processing, put the dest buffer on the fresh plate
//		display_fresh_buffer = display_empty_buffer; // (fresh <- empty) | -> stale on frame done
//		display_empty_buffer = NULL; // empty <- null | -> empty in frame boundary
//		// fresh goes to stale, and stale goes to empty?
//	}
}


void camera_buffer_test(void){
	CSI->DMASA_FB1 = camera_buffer_manager.dma_buffer0_sa;
	CSI->DMASA_FB2 = camera_buffer_manager.dma_buffer1_sa;

//	csi enable base address witch on
//	CSI->CR18 |= CSI_CR18_BASEADDR_SWITCH_EN_MASK | CSI_CR18_BASEADDR_SWITCH_SEL(1);

//	start csi subsystem
	CSI->CR1 |= CSI_CR1_FB1_DMA_DONE_INTEN_MASK; // enable interrupt fb1 full
	CSI->CR1 |= CSI_CR1_FB2_DMA_DONE_INTEN_MASK; // enable interrupt fb2 full
	CSI->CR1 |= CSI_CR1_SOF_INTEN_MASK;; // enable interrupt fb2 full
	CSI->CR3 |= CSI_CR3_DMA_REQ_EN_RFF_MASK; 	 // enable dma reqs from receive fifo
	CSI->CR18|= CSI_CR18_CSI_ENABLE_MASK; 		 // enable csi module

//	wait for data valid

	while(1){
		while(*camera_buffer_manager.status != 0x1){
			__NOP();
		}
		PRINTF("camera data valid\r\n");
		camera_buffer_manager.empty_buffer_sa = camera_buffer_manager.clean_buffer_sa;
		camera_buffer_manager.clean_buffer_sa = 0; //reset
		*camera_buffer_manager.status = 0x0;
	}

}

void lcd_buffer_test(void){
	ST7701_SPIWrite(0x13, COMMAND); // normal display mode on
	simpleDelay(10);


	//	enable lcdif and interrupts
	ELCDIF_RgbModeStart(LCDIF);
	ELCDIF_EnableInterrupts(LCDIF, kELCDIF_VsyncEdgeInterruptEnable | kELCDIF_CurFrameDoneInterruptEnable); //seems like don't need vsync

//	wait for ready
	while(*display_buffer_manager.status != 0x1){
		__NOP();
	}

	fill_framebuffer_gradient((uint32_t*)display_buffer_manager.ready_sa, 480, 480);
	display_buffer_manager.fill_callback(&display_buffer_manager);


	PRINTF("display ready\r\n");
	GPIO_PinWrite(GPIO1, 9U, 1U); //display backlight enable
}

void transfer_test(void){
	CSI->DMASA_FB1 = camera_buffer_manager.dma_buffer0_sa;
		CSI->DMASA_FB2 = camera_buffer_manager.dma_buffer1_sa;

	//	csi enable base address witch on
	//	CSI->CR18 |= CSI_CR18_BASEADDR_SWITCH_EN_MASK | CSI_CR18_BASEADDR_SWITCH_SEL(1);

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

//		wait until ready & valid
		while(!((*transfer_manager.data_valid_block_status & 0x1) && (*transfer_manager.ready_block_status & 0x1))){
			__NOP();
		}

//		make sure data
		processForDisplay((uint32_t*)camera_buffer_manager.data_valid_sa, (uint32_t*)display_buffer_manager.ready_sa);
		
		camera_buffer_manager.drain_callback(&camera_buffer_manager);
		display_buffer_manager.fill_callback(&display_buffer_manager);

		GPIO_PinWrite(GPIO1, 9U, 1U); //display backlight enable

}



//void CAMERA_Stop(void){}
