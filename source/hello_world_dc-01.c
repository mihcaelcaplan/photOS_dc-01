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
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_msc.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "usb_phy.h"

#include "sdmmc_config.h"

/* TODO: insert other include files here. */
#include "usb_mux.h"
#include "debug_shell.h"
#include "display_interface.h"
#include "utils.h"
#include "camera_interface.h"
#include "battery_interface.h"
#include "state.h"


/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    MUX_Init();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_USB_Disk_Config(USB_DEVICE_INTERRUPT_PRIORITY);
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
    PRINTF("Hello World, I'm photOS, the operating system for the DC-0x cameras.\r\n");
    MUX_ToUSBC();
    USB_DeviceApplicationInit();
    DISPLAY_Init(); // very minimal display function, basically reset pulse
    BATTERY_Init(); // initialize battery monitoring
    

    
    CAMERA_Init(); // right now doing the display control for camera live view

    simpleDelay(1); //TODO: needed?

//    transfer_test();

    STATE_Init(); //enter compose state

    while(1) {
        // enter the compose state and listen for events 

     }
    return 0 ;
}
