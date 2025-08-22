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

// counters for irqs to catch overfiring
int lcdirqc = 0;
int lcdirqbufchangec = 0;
int csiirqc = 0;

// only enable cur_frame_done
volatile bool pending_frame = false;
volatile uint32_t pending_frame_sa;
//volatile uint32_t active_frame_sa;

volatile uint32_t last_frame_shown = 0;


void LCDIF_IRQHandler(void){
	uint32_t flags = (LCDIF->CTRL1 & ELCDIF_CTRL1_IRQ_MASK);
    LCDIF->CTRL1_CLR = ELCDIF_CTRL1_IRQ_MASK; //  clear all interrupt

     if (flags & kELCDIF_VsyncEdge){
		// if (lcdMailbox.full){
		// 	LCDIF->NEXT_BUF = lcdMailbox.data;
		// 	lcdMailbox.full = false;
		// 	lcdMailbox.data = 0;
			
		// 	lcdirqbufchangec++;
		// }
	 }
	 
	if (flags & kELCDIF_CurFrameDone)
	 {
		if (lcdMailbox.full){
		LCDIF->NEXT_BUF = lcdMailbox.data;
		lcdMailbox.full = false;
		lcdMailbox.data = 0;
		
		lcdirqbufchangec++;
		}
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
//		camera_buffer_manager.start_of_frame(&camera_buffer_manager);
	}

	if(status & kCSI_RxBuffer0DmaDoneFlag){
		CSI->SR |= CSI_SR_DMA_TSF_DONE_FB1_MASK; //clear flag by writing 1
		cameraMailbox.full = true;
		cameraMailbox.data = 1;

	}
	if(status & kCSI_RxBuffer1DmaDoneFlag){
		CSI->SR |= CSI_SR_DMA_TSF_DONE_FB2_MASK; //clear flag by writing 1
		cameraMailbox.full = true;
		cameraMailbox.data = 2;

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
    OV5640_I2CWrite8(0x3035, 0x21);  // [7:4] sysclk divider, [3:0] mipi divider
    OV5640_I2CWrite8(0x3036, 0x64);  // multiplier (Can be any integer from 4~127 and only even integers from 128~252)
    OV5640_I2CWrite8(0x3037, 0x11);  // [4] root divider (1 = root/2), [3:0] pre-divider (1,2,3,4,6,8)

	OV5640_I2CWrite8(0x3108, 0x16); //set system dividers


	OV5640_I2CWrite8(0x3000, 0x00); //take core blocks out of reset
	OV5640_I2CWrite8(0x3002, 0x1C); //put jfifo, sfifo, jpg in reset
	OV5640_I2CWrite8(0x3004, 0xFF); // turn MCU clocks on
	OV5640_I2CWrite8(0x3006, 0xc3); // disable clock of JPEG2x, JPEG
	OV5640_I2CWrite8(0x300e, 0x58); //mipi off, DVP on (should be default)

//set up image window constraints
   OV5640_I2CWrite8(0x3800, 0x00 );// X start [11:8]
   OV5640_I2CWrite8(0x3801, 0x00 );// X start [7:0]
   OV5640_I2CWrite8(0x3802, 0x00 );// Y start [11:8]
   OV5640_I2CWrite8(0x3803, 0x00 );// Y start [7:0]

   OV5640_I2CWrite8(0x3804, 0x0a);// X end [11:8] (start + width +offset)
   OV5640_I2CWrite8(0x3805, 0x3f);// X end [7:0]
   OV5640_I2CWrite8(0x3806, 0x07);// Y end [11:8] (start + width +offset)
   OV5640_I2CWrite8(0x3807, 0x9f);// Y end [7:0]

//    TODO: add in DVP output settings, the actual output width
    OV5640_I2CWrite8(0x3808, 0x07); //DVP output horizontal width [11:8]
    OV5640_I2CWrite8(0x3809, 0x80); //DVP output horizontal width [7:0]
    OV5640_I2CWrite8(0x380A, 0x07); //DVP output vertical height [11:8]
    OV5640_I2CWrite8(0x380B, 0x80); //DVP output vertical height [7:0]
//
//    OV5640_I2CWrite8(0x380C, 0x02) ; // total horizontal size [11:8]
//    OV5640_I2CWrite8(0x380D, 0xf8); // total horizontal size [7:0]
//    OV5640_I2CWrite8(0x380E, 0x02); // total vertical size[11:8]
//    OV5640_I2CWrite8(0x380F, 0xe0); // total vertical size[ 7:0]
//    	0x380c, 0x0b, // HTS 		//
//    	0x380d, 0x1c, // HTS
//    	0x380e, 0x07, // VTS 		//
//    	0x380f, 0xb0, // VTS

	OV5640_I2CWrite8(0x3810, 0x00); //horizontal offset [11:8]
	OV5640_I2CWrite8(0x3811, 0x10); //horizontal offset [7:0]
	OV5640_I2CWrite8(0x3812, 0x00); //vertical offset [11:8]
	OV5640_I2CWrite8(0x3813, 0x04); //vertical offset [7:0]


    //    OV5640_I2CWrite8(0x4709, (uint8_t)50); // vsync line width n*lu
	//    OV5640_I2CWrite8(0x471B, 0x03); // Hsync mode enable
    OV5640_I2CWrite8(0x4713, 0x03); // JPEG (compression?) mode 3
	
	OV5640_I2CWrite8(0x471D, 0x00); // vsync mode not use width  
    
	// OV5640_I2CWrite8(0x470A, 0x0f); //vsync width [15:8]
    // OV5640_I2CWrite8(0x470B, 0x00); //vsync width [7:0]
	
    // OV5640_I2CWrite8(0x4740, 0x28); //POlarity control!


//  X/Y increment (binning/subsampling)
    OV5640_I2CWrite8(0x3814,0x11); // X [7:4]odd subsampling increment, [3:0] even subsampling increment
    OV5640_I2CWrite8(0x3815,0x11);  // Y [7:4]odd subsampling increment, [3:0] even subsampling increment

// Timing control (flip/mirror)
//    OV5640_I2CWrite8(0x3820, 0x41);  // [2] ISP flip, [1] sensor flip
//    OV5640_I2CWrite8(0x3821, 0x02);  // [5] jpeg en, [2]isp mirror, [1] sensor mirror, [0] horizontal binning enable (vertical binning auto-enable on Y inc.)

// Output Interface:
// Raw format output (bypass ISP)
//    OV5640_I2CWrite8(0x4300, 0xf8); //raw, default pixel order
    OV5640_I2CWrite8(0x4300, 0x00);
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
	1920, //width
	1920, //height
	kCSI_HsyncActiveHigh | kCSI_DataLatchOnRisingEdge | kCSI_VsyncActiveHigh,
	1, // byte per pixel (raw 8 bits per pixel)
	1920*1, //linepitch in bytes
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

//	memset(&c_frameBuffer[0], 0, sizeof(c_frameBuffer[0]));//
}

#define R_GAIN 256
#define G_GAIN 256
#define B_GAIN 256

// diagonal whitebalance adjust
int16_t wb_adjust[3][3] = {
    {256, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};

int16_t blue_adjust[3][3] = {
	{230,   0,  26},
	{  0, 230,  26},
	{  0,   0, 307}
};

const int16_t color_matrix[3][3] = {
	    {256,   -12, -32},  // Red channel
	    {  0, 200, -16},  // Green channel
	    {  0,   -12, 205}   // Blue channel
};


//if 0s everwhere then its diagonal
bool isDiagonal(int16_t matrix[3][3]) {
    return (matrix[0][1] == 0 && matrix[0][2] == 0 &&
            matrix[1][0] == 0 && matrix[1][2] == 0 &&
            matrix[2][0] == 0 && matrix[2][1] == 0);
}
// apply a gain by right shifting, so a*b/2^8 = a*b/256 where b{1:256}, so a scaled by the fraction b/256
void applyColorMatrix(uint8_t *r, uint8_t *g, uint8_t *b, int16_t matrix[3][3]) {
	int16_t r_out, g_out, b_out;
	if (isDiagonal(matrix)){
		 r_out = (*r * matrix[0][0]) >> 8;
		 g_out = (*g * matrix[1][1]) >> 8;
		 b_out = (*b * matrix[2][2]) >> 8;
	}
	else{
		 r_out = (matrix[0][0] * (*r) + matrix[0][1] * (*g) + matrix[0][2] * (*b)) >> 8;
		 g_out = (matrix[1][0] * (*r) + matrix[1][1] * (*g) + matrix[1][2] * (*b)) >> 8;
		 b_out = (matrix[2][0] * (*r) + matrix[2][1] * (*g) + matrix[2][2] * (*b)) >> 8;
	}

    *r = (r_out > 255) ? 255 : (r_out < 0) ? 0 : r_out;  // Clamp!
    *g = (g_out > 255) ? 255 : (g_out < 0) ? 0 : g_out;
    *b = (b_out > 255) ? 255 : (b_out < 0) ? 0 : b_out;
}



/* interpolateForDisplay
 * assumes a 480x480 ("full zoom") image size input
 * always copying to a 480x480 display buffer 
*/
void interpolateForDisplay(uint8_t* source_buffer, uint8_t* dest_buffer){
	uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

	for(int i=0; i < 480; i++){
        for(int j=0; j < 480; j++){
            int src_i = i*480 + j; // line number* line width + column number
			
			// border region
			if( i == 0 || i == 480-1 || j == 0 || j == 480-1 ){
				// leave them black haha
			}
			
			// uint32_t pixel_value = source_buffer[src_i];
			
			// inner region
			else if(i % 2 == 0) { // Even rows: BGBGBG...
                if(j % 2 == 0) { // B pixel
                    // estimate r and g
					r = (source_buffer[src_i - 480 - 1]+ source_buffer[src_i - 480 + 1] + source_buffer[src_i + 480 - 1] + source_buffer[src_i + 480 + 1]) /4; 
					g = (source_buffer[src_i - 480] + source_buffer[src_i + 480] + source_buffer[src_i - 1] + source_buffer[src_i + 1])/4;
					b = source_buffer[src_i];
                } else { // G pixel
					// estimate r and b
					r = (source_buffer[src_i - 480] + source_buffer[src_i + 480]) / 2;
					b = (source_buffer[src_i - 1] + source_buffer[src_i + 1]) / 2;
                    g = source_buffer[src_i];;
                }
            } else { // Odd rows: GRGRGR...
                if(j % 2 == 0) { // G pixel
					// estimate r and b
					r = (source_buffer[src_i - 1] + source_buffer[src_i + 1]) / 2;
					b = (source_buffer[src_i - 480] + source_buffer[src_i + 480]) / 2;
                    g = source_buffer[src_i];;
                } else { // R pixel
					// estimate b and g
					b = (source_buffer[src_i - 480 - 1]+ source_buffer[src_i - 480 + 1] + source_buffer[src_i + 480 - 1] + source_buffer[src_i + 480 + 1]) /4; 
					g = (source_buffer[src_i - 480] + source_buffer[src_i + 480] + source_buffer[src_i - 1] + source_buffer[src_i + 1])/4;
                    r = source_buffer[src_i];
                }
            }
			
			// assign out to 24bpp lcd buffer
			int dst_i = i*480*3 + 3*j; // line number* line width*3 + column number*3
			applyColorMatrix(&r, &g, &b, color_matrix);

			dest_buffer[dst_i + 0] = r;
            dest_buffer[dst_i + 1] = g;
            dest_buffer[dst_i + 2] = b;
			
		}
	}
}

/* binAndInterpolateForDisplay
 * assumes a 1920x1920 ("full square") image size input
 * always copying to a 480x480 display buffer 
*/
void binAndInterpolateForDisplay(uint8_t* source_buffer, uint8_t* dest_buffer, zoom_level_t level){
	uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

	
	int source_w = 1920;
	int source_h = 1920;

	// linear = rows*row_len + columns
//	int roi_linear_start_1 = 0; 				 // h = 1920, w = 1920 	-> 4:1
//	int roi_linear_start_2 = 480*source_w + 480; // h = 960,  w = 960 	-> 2:1
//	int roi_linear_start_3 = 720*source_w + 720; // h = 480,  w = 480  	-> 1:1

	int roi_linear_start_1 = 0;
	int roi_linear_start_2 = 0;
	int roi_linear_start_3 = 0;



	int width = source_w;

	// sweep over the output array for consistency
	for(int i=0; i < 480; i++){
        for(int j=0; j < 480; j++){

			// for full image, 4x4 window
            if (level == zoom_level_1){

				// linear block index: start + row*step*row_width + col*step
				int block_i = roi_linear_start_1 + i*zoom_level_1*width + j*zoom_level_1;
				
				// Names       ... Index Offset
				// b3 g7 b4 g8 ... 3*width+0 3*width+1 3*width+2 3*width+3
				// g5 r3 g6 r4 ... 2*width+0 2*width+1 2*width+2 2*width+3
				// b1 g3 b2 g4 ... width+0   width+1   width+2   width+3
				// g1 r1 g2 r2 ... 0  	   	 1  	   2  	   	 3
				
				// offset variables 
				int b3 = block_i + 3*width+0; int g7 = block_i + 3*width+1;  int b4 = block_i + 3*width+2; int g8 = block_i + 3*width+3;
				int g5 = block_i + 2*width+0; int r3 = block_i + 2*width+1;  int g6 = block_i + 2*width+2; int r4 = block_i + 2*width+3;
				int b1 = block_i + width+0;   int g3 = block_i + width+1;    int b2 = block_i + width+2;   int g4 = block_i + width+3;
				int g1 = block_i + 0; 	      int r1 = block_i + 1; 	     int g2 = block_i + 2; 		   int r2 = block_i + 3;

				r = (source_buffer[r1] + source_buffer[r2] + source_buffer[r3] + source_buffer[r4])/4;
				b = (source_buffer[b1] + source_buffer[b2] + source_buffer[b3] + source_buffer[b4])/4;
				g = (source_buffer[g1] + source_buffer[g2] + source_buffer[g3] + source_buffer[g4] + source_buffer[g5] + source_buffer[g6] + source_buffer[g7] + source_buffer[g8])/8;

			// intermediate zoom, 2x2 window
			}
			else if (level == zoom_level_2){

				
				// linear block index:  start + row*step*row_width + col*step
				int block_i = roi_linear_start_2 + i*zoom_level_2*source_w + j*zoom_level_2;
				
				// Names ... Index Offset
				// b1 g2 ... width+0 width+1
				// g1 r1 ... 0       1
				
				// offset variables 
				int b1 = block_i + width+0; int g2 = block_i + width+1;
				int g1 = block_i + 0; 	    int r1 = block_i + 1;

				r = source_buffer[r1];
				b = source_buffer[b1];
				g = (source_buffer[g1] + source_buffer[g2])/2;

			}
		
			// for full zoom, indexing as linear array:  start + row*row_width + col
			else if(level == zoom_level_3){
//				int src_i = i*source_w + j; // line number* line width + column number
				 int src_i = roi_linear_start_3 + i*source_w + j; // line number* line width + column number
				
				// border region
				if( i == 0 || i == 480-1 || j == 0 || j == 480-1 ){
					// leave them black haha
				}
				// inner region
				else if(i % 2 == 0) { // Even rows: BGBGBG...
					if(j % 2 == 0) { // B pixel
						// estimate r and g
						r = (source_buffer[src_i - source_w - 1]+ source_buffer[src_i - source_w + 1] + source_buffer[src_i + source_w - 1] + source_buffer[src_i + source_w + 1]) /4; 
						g = (source_buffer[src_i - source_w] + source_buffer[src_i + source_w] + source_buffer[src_i - 1] + source_buffer[src_i + 1])/4;
						b = source_buffer[src_i];
					} else { // G pixel
						// estimate r and b
						r = (source_buffer[src_i - source_w] + source_buffer[src_i + source_w]) / 2;
						b = (source_buffer[src_i - 1] + source_buffer[src_i + 1]) / 2;
						g = source_buffer[src_i];
					}
				} else { // Odd rows: GRGRGR...
					if(j % 2 == 0) { // G pixel
						// estimate r and b
						r = (source_buffer[src_i - 1] + source_buffer[src_i + 1]) / 2;
						b = (source_buffer[src_i - source_w] + source_buffer[src_i + source_w]) / 2;
						g = source_buffer[src_i];
					} else { // R pixel
						// estimate b and g
						b = (source_buffer[src_i - source_w - 1]+ source_buffer[src_i - source_w + 1] + source_buffer[src_i + source_w - 1] + source_buffer[src_i + source_w + 1]) /4; 
						g = (source_buffer[src_i - source_w] + source_buffer[src_i + source_w] + source_buffer[src_i - 1] + source_buffer[src_i + 1])/4;
						r = source_buffer[src_i];
					}
				}
			}
			
			// assign out to 24bpp lcd buffer
			int dst_i = i*480*3 + 3*j; // line number* line width*3 + column number*3
//			applyColorMatrix(&r, &g, &b, color_matrix);

			dest_buffer[dst_i + 0] = r;
            dest_buffer[dst_i + 1] = g;
            dest_buffer[dst_i + 2] = b;
			
		}
	}
}


// display raw data
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
