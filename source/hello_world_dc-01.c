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
#include "usb_mux.h"
#include "debug_shell.h"
#include "display_interface.h"
#include "utils.h"
#include "camera_interface.h"
#include "battery_interface.h"
#include "state.h"
#include "events.h"
#include "timer.h"
#include "display_file.h"
#include "button.h"
#include "knob.h"
#include "ff.h"

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
    BUTTON_Init();
    TIMER_Init();
    KNOB_Init();
    
    // set up pmic
    MUX_Init(); //switch mux to pmic
    pmic_connected_t usb_connected = BATTERY_Init(); // turn on dp/dm detection
    MUX_ToUSBC();
    
    BOARD_USB_Disk_Config(USB_DEVICE_INTERRUPT_PRIORITY);
    MOUNT_SDCard(); //necessary :)
    USB_DeviceApplicationInit();
    USB_DeviceAppStart();
    
    DISPLAY_Init();
    CAMERA_Init();
    
    
    PRINTF("Hello World, I'm photOS, the operating system for the DC-0x cameras.\r\n");
//    TIMER_TurnOnInterrupts();
    STATE_Init(); //start the state machine


    // shouldn't get here but will let debugger hook if it does
    PRINTF("IDLE... \r\n");
    while(1){
    	__NOP();
    }
    // shouldn't get here ever but would love to know if we do somehow
    PRINTF("Somehow reached the return :'(\r\n");
    return 0 ;
}
