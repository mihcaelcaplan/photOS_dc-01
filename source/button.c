/*
 * button.c
 *
 *  Created on: Sep 1, 2025
 *      Author: mcaplan
 */

 #include "button.h"
 #include "state.h"
#include "timer.h"

volatile bool button_pressed = 0;
volatile long now = 0;
volatile long last_irq_time = 0;
volatile long button_press_start = 0;
#define DEBOUNCE_TIME 50000

 void GPIO5_Combined_0_15_IRQHandler(void){
    GPIO_PortClearInterruptFlags(GPIO5, 1U << 0); //clear
    uint32_t pin_state = GPIO_PinReadPadStatus(GPIO5, 0);
    uint32_t now = TIMER_GetCurrentUs();

    if (now - last_irq_time < DEBOUNCE_TIME) {
          return;  // Ignore bounces
      }
      last_irq_time = now;

    // Now process the clean state change
    if (pin_state == 0 && !button_pressed) {
        // Clean press event
        button_pressed = true;           // SET THE FLAG
        button_press_start = now;        // START TIMING
    }
    else if (pin_state == 1 && button_pressed) {
        // Clean release event
        button_pressed = false;          // CLEAR THE FLAG
        long duration = now - button_press_start;  //   CALCULATE DURATION

        // Handle state transitions based on duration
        if (duration < 400000) STATE_set_current(CAPTURE);
        else STATE_set_current(ADJUST);
      }

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
