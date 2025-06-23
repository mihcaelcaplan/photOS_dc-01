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

#define CAMERA_I2C LPI2C1
#define OV5460_I2C_ADDRESS_WRITE 0x3C
#define OV5460_I2C_ADDRESS_READ 0x3C
#define RESET_GPIO 14U


// all the addresses are 16 bit
uint32_t OV5640_I2CRead8(uint16_t regAddr){

	uint8_t rcv = 0;

	//	send the register address to read
	LPI2C_MasterStart(CAMERA_I2C, OV5460_I2C_ADDRESS_WRITE, kLPI2C_Write );
	LPI2C_MasterSend(CAMERA_I2C, &regAddr, 2);
	LPI2C_MasterStop(CAMERA_I2C);

//	restart (turn around bus) and receive the data
	LPI2C_MasterStart(CAMERA_I2C, OV5460_I2C_ADDRESS_READ, kLPI2C_Read );
	LPI2C_MasterReceive(CAMERA_I2C, &rcv, 1);
	LPI2C_MasterStop(CAMERA_I2C);

	return rcv;

}

// all the addresses are 16 bit, reg is 8 bit wide
void OV5640_I2CWrite8(uint16_t regAddr, uint32_t data){
	LPI2C_MasterStart(CAMERA_I2C, OV5460_I2C_ADDRESS_WRITE, kLPI2C_Write );

	LPI2C_MasterSend(CAMERA_I2C, &regAddr, 2);
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
    OV5640_I2CWrite8(0x3018, 0xf0); //d[5:2] high, no gpio

// PLL settings for pixclk out
    OV5640_I2CWrite8(0x3034, 0x18); // set to 8 bit mode

// pixclk = mclk/root_div/pre-div)*multiply/sysclk divider
// (24Mhz/2/4)*100/6 = 50Mhz
    OV5640_I2CWrite8(0x3035, 0x61);  // [7:4] sysclk divider, [3:0] mipi divider
    OV5640_I2CWrite8(0x3036, 0x64);  // multiplier (Can be any integer from 4~127 and only even integers from 128~252)
    OV5640_I2CWrite8(0x3037, 0x14);  // [4] root divider (1 = root/2), [3:0] pre-divider (1,2,3,4,6,8)

	OV5640_I2CWrite8(0x3108, 0x01); //turn on clocks



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

// set up image window constraints
    OV5640_I2CWrite8(0x3800, (x_start >> 8) & 0xFF); // X start [11:8]
    OV5640_I2CWrite8(0x3801, x_start & 0xFF); // X start [7:0]
    OV5640_I2CWrite8(0x3802, (y_start >> 8) & 0xFF); // Y start [11:8]
    OV5640_I2CWrite8(0x3803, y_start & 0xFF); // Y start [7:0]
    OV5640_I2CWrite8(0x3804, (x_end >> 8) & 0xFF); // X end [11:8]
    OV5640_I2CWrite8(0x3805, x_end & 0xFF); // X end [7:0]
    OV5640_I2CWrite8(0x3806, (y_end >> 8) & 0xFF); // Y end [11:8]
    OV5640_I2CWrite8(0x3807, y_end & 0xFF); // Y end [7:0]
    
    // these only operate in the ISP stage, i think not if i disable it all
    // 0x3808 //horizontal width [11:8]
    // 0x3809 //horizontal width [7:0]
    // 0x380A //vertical height [11:8]
    // 0x380B //vertical height [7:0]
    
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
    OV5640_I2CWrite8(0x4300, 0x00); //raw, default pixel orde

// ISP disable
    OV5640_I2CWrite8(0x5000, 0x00); // all isp off
    OV5640_I2CWrite8(0x5001, 0x00);

    OV5640_I2CWrite8(0x3008, 0x02);// wake up from standby
}

void OV5640_Init(void){

// 1ms reset pulse
	gpio_pin_config_t ov5640_reset = {
			kGPIO_DigitalOutput,
			1U,
			kGPIO_NoIntmode
	};
//	set up gpio
	GPIO_PinInit(GPIO1, RESET_GPIO, &ov5640_reset);
	GPIO_PinWrite(GPIO1,RESET_GPIO, 0U);
	simpleDelay(2);
//	turn on
	GPIO_PinWrite(GPIO1,RESET_GPIO, 1U);

/* Enable the master function and disable the slave function. */
	LPI2C_MasterEnable(CAMERA_I2C, true);
	LPI2C_SlaveEnable(CAMERA_I2C, false);
	
	//init registers
	registerInit();
}



void CAMERA_Init(void){

//	init the CSI receiver


	OV5640_Init();

}

void CAMERA_Run(void);
void CAMERA_Stop(void);
