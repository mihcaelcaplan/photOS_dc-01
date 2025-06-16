/*
 * camera_ov5640.c
 *
 *  Created on: Jun 11, 2025
 *      Author: mcaplan
 */

#include "camera_ov5640.h"
#include "fsl_lpi2c.h"
#include "fsl_gpio.h"

#define CAMERA_I2C LPI2C1
#define OV5460_I2C_ADDRESS 0x78
#define RESET_GPIO 14U


void OV5640_Init(void){
/* Enable the master function and disable the slave function. */
	LPI2C_MasterEnable(base, true);
	LPI2C_SlaveEnable(base, false);

}

uint32_t OV5640_I2CRead8(uint32_t value){

	uint8_t rcv = 0;


	//	restart (turn around bus)
	LPI2C_MasterStart(CAMERA_I2C, OV5460_I2C_ADDRESS, kLPI2C_Read );
//	read 1 byte = 8b
	LPI2C_MasterReceive(CAMERA_I2C, &rcv, 1);

	LPI2C_MasterStop(CAMERA_I2C);

	return rcv;

}

void OV5640_I2CWrite8(uint32_t data){
	LPI2C_MasterStart(CAMERA_I2C, OV5460_I2C_ADDRESS, kLPI2C_Write );

//	send 1 byte = 8b
	LPI2C_MasterSend(CAMERA_I2C, &data, 1);

	LPI2C_MasterStop(CAMERA_I2C);



}


void CAMERA_Init(void){
	gpio_pin_config_t ov5640_reset = {
			kGPIO_DigitalOutput,
			0U,
			kGPIO_NoIntmode
	};

//	set up gpio
	GPIO_PinInit(GPIO1, RESET_GPIO, &ov5640_reset);
	simpleDelay(1);

//	turn on
	GPIO_PinWrite(GPIO1,RESET_GPIO, 1U);


	OV5640_Init();

}

void CAMERA_Run(void);
void CAMERA_Stop(void);
