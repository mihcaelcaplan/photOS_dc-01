/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    hello_world_dc-01.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"


#include "storage_usb_device.h"


#include "sdmmc_config.h"

/* TODO: insert other include files here. */
#include "usb_mux.h"
#include "debug_shell.h"
#include "display_interface.h"
#include "utils.h"
#include "camera_interface.h"
#include "battery_interface.h"
#include "state.h"
#include "events.h"
#include "timer.h"

// state and event queues
state_queue_t state_q;
event_queue_t event_q;


// set up button interrupts
/* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
        kGPIO_IntRisingEdge,
    };

int zoom_c = 0;

void GPIO5_Combined_0_15_IRQHandler(void){
    GPIO_PortClearInterruptFlags(GPIO5, 1U << 0);
    // set state flag 
    STATE_set_current(CAPTURE);
	SDK_ISR_EXIT_BARRIER;
}


/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
    TIMER_Init();
    
    // set up pmic
    MUX_Init(); //switch mux to pmic
    BATTERY_Init(); // turn on dp/dm detection
    // 
    MUX_ToUSBC();
    BOARD_USB_Disk_Config(USB_DEVICE_INTERRUPT_PRIORITY);
    USB_DeviceApplicationInit();
    
    PRINTF("Hello World, I'm photOS, the operating system for the DC-0x cameras.\r\n");
    
    DISPLAY_Init(); // very minimal display function, basically reset pulse
//
    CAMERA_Init();
	
    /* Enable GPIO pin interrupt */
    GPIO_PinInit(GPIO5, 0, &sw_config);
    GPIO_PortEnableInterrupts(GPIO5, 1U << 0);

    EnableIRQ(GPIO5_Combined_0_15_IRQn);
    
    
    USB_DeviceAppStart();
    STATE_Init(); //start the state machine

    
    // shouldn't get here but will let debugger hook
    PRINTF("IDLE... \r\n");
    while(1){
//    	if (1 == GPIO_PinRead(GPIO5, 0)){
//    		PRINTF("SW: hi");
//    	}
//    	else PRINTF("SW: lo");
//    	simpleDelay(150);

    	__NOP();
    }
    // shouldn't get here but would love to know if it does
    PRINTF("Somehow reached the return :'(\r\n");
    return 0 ;
}
