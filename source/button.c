/*
 * button.c
 *
 *  Created on: Sep 1, 2025
 *      Author: mcaplan
 */

 #include "button.h"
 #include "state.h"


 void GPIO5_Combined_0_15_IRQHandler(void){
    GPIO_PortClearInterruptFlags(GPIO5, 1U << 0);
    // set state flag
    STATE_set_current(CAPTURE);
	SDK_ISR_EXIT_BARRIER;
}

gpio_pin_config_t main_sw_config = {
    kGPIO_DigitalInput,
    0,
    kGPIO_IntRisingEdge,
};


void BUTTON_Init(void){
     /* Enable GPIO pin interrupt */
    GPIO_PinInit(GPIO5, 0, &main_sw_config);
    GPIO_PortEnableInterrupts(GPIO5, 1U << 0);

    EnableIRQ(GPIO5_Combined_0_15_IRQn);
    
}