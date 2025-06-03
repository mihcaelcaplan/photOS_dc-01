/*
 * usb_mux.c
 *
 *  Created on: Jun 3, 2025
 *      Author: mcaplan
 */

#include "usb_mux.h"
#include "fsl_gpio.h"
#include "MIMXRT1062.h"

#define SWITCH_TO_PMIC 0U
#define SWITCH_TO_USBC 1U



// sets the mux up, including configuring the gpio to be the correct state after power up
void MUX_Init(void)
{
	gpio_pin_config_t mux_config =
	{
			kGPIO_DigitalOutput,
			SWITCH_TO_PMIC,
			kGPIO_NoIntmode
	};

	//	init the switch pin to route to pmic initially
	GPIO_PinInit(GPIO2, 30U, &mux_config);


};

//main interface
void MUX_ToPMIC(void)
{
	GPIO_PinWrite(GPIO2, 30U, SWITCH_TO_PMIC);
};

void MUX_ToUSBC(void)
{
	GPIO_PinWrite(GPIO2, 30U, SWITCH_TO_USBC);
};
