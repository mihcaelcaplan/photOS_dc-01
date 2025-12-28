/*
 * battery_mp2733.c
 *
 *  Created on: Jul 17, 2025
 *      Author: mcaplan
 */

#include "hardware/battery_interface.h"
#include "fsl_lpi2c.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include <stdint.h>

#define PMIC_I2C LPI2C1
#define PMIC_I2C_ADDRESS 0x4B
#define BATT_PMIC_INT 31U
int gpio_c = 0;
// set up input handler
bool int_triggered = false;

//void GPIO2_Combined_16_31_IRQHandler(void){
//		int_triggered = true;
//		gpio_c++;
//}


uint32_t BATTERY_I2CRead8(uint8_t address, uint8_t* rcv_buf){
	status_t status;

	// 7 bit i2c address
	status = LPI2C_MasterStart(PMIC_I2C, PMIC_I2C_ADDRESS, kLPI2C_Write);
	assert(status ==0);

	//	send address (swapped)
	status = LPI2C_MasterSend(PMIC_I2C, &address, 1);
	assert(status ==0);

	status = LPI2C_MasterRepeatedStart(PMIC_I2C, PMIC_I2C_ADDRESS, kLPI2C_Read);

	//receive data in reg
	status = LPI2C_MasterReceive(PMIC_I2C, rcv_buf, 1); //1 byte transfer to rcv_buf
	assert(status ==0);

	status = LPI2C_MasterStop(PMIC_I2C);
//	assert(status ==0);
}

void BATTERY_I2CWrite8(uint8_t address, uint8_t* data){
	status_t status;
	// 7 bit i2c address
	status = LPI2C_MasterStart(PMIC_I2C, PMIC_I2C_ADDRESS, kLPI2C_Write);
	assert(status ==0);
	
	//	send address (swapped)
	status = LPI2C_MasterSend(PMIC_I2C, &address, 1);
	assert(status ==0);

	//	send data
	status = LPI2C_MasterSend(PMIC_I2C, &data, 1);
	assert(status ==0);

	status = LPI2C_MasterStop(PMIC_I2C);
	assert(status ==0);

}

gpio_pin_config_t pmic_int_config = {
	kGPIO_DigitalInput,
	0,
	kGPIO_IntRisingEdge,
};



// returns the vin_status
pmic_connected_t BATTERY_Init(void){
	uint8_t pmic_status = 0;
	BATTERY_I2CRead8(0x0C, &pmic_status);

	uint8_t vin_status = (pmic_status & 0xE0) >> 5;

	switch (vin_status) {
		case PMIC_Not_Connected:
			PRINTF("BATTERY: Not Connected\r\n");
			break;
		case PMIC_Connected_SDP:
			PRINTF("BATTERY: Connected to SDP\r\n");
			break;
		case PMIC_Connected_CDP:
			PRINTF("BATTERY: Connected to CDP\r\n");
			break;
		case PMIC_Connected_DCP:
			PRINTF("BATTERY: Connected to DCP\r\n");
			break;
		default:
			if(vin_status > 0){
				PRINTF("BATTERY: Connected to non-standard\r\n");
			}
			else{
				PRINTF("BATTERY: bad PMIC status\r\n");
			}
			break;
	}

	PRINTF("BATTERY: Init completed\r\n");
	return vin_status;
}

battery_level_t BATTERY_Get_Level(void){
	return CHARGED;
}

uint8_t BATTERY_Get_Status(void){
	uint8_t pmic_status = 0;
	BATTERY_I2CRead8(0x0C, &pmic_status);
	return pmic_status;
}
