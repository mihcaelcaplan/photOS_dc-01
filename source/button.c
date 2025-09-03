/*
 * button.c
 *
 *  Created on: Sep 1, 2025
 *      Author: mcaplan
 */

 #include "button.h"
 #include "state.h"
#include "timer.h"


volatile int button_press_start = 0;
volatile bool button_pressed = false;
long time_depressed = 0;
long debounce_time = 0;


 void GPIO5_Combined_0_15_IRQHandler(void){
    GPIO_PortClearInterruptFlags(GPIO5, 1U << 0); //clear

    // check if this is the press or release
    if(button_pressed == false){
	    // start the timer
    	button_press_start = TIMER_GetCurrentUs();
    	button_pressed = true;
    }
    else if(button_pressed == true){

//    	debounce 50 ms
    	debounce_time = TIMER_GetCurrentUs() - button_press_start;

    	if(debounce_time > 50000){
			//stop the timer, clear the flag and switch
			time_depressed = debounce_time;
			button_pressed = false;

			if(time_depressed < 400000) STATE_set_current(CAPTURE);
			if(time_depressed > 400000) STATE_set_current(ADJUST);
    	}

    }
    // set state flag
    // STATE_set_current(CAPTURE);
     SDK_ISR_EXIT_BARRIER;
}

gpio_pin_config_t main_sw_config = {
    kGPIO_DigitalInput,
    0,
	kGPIO_IntRisingOrFallingEdge,
};


void BUTTON_Init(void){
     /* Enable GPIO pin interrupt */
    GPIO_PinInit(GPIO5, 0, &main_sw_config);
    GPIO_PortEnableInterrupts(GPIO5, 1U << 0);

    EnableIRQ(GPIO5_Combined_0_15_IRQn);
    
}
