/*
 * knob.c
 *
 *  Created on: Oct 16, 2025
 *      Author: mcaplan
 */
#include "knob.h"
#include "state_adjust.h"
#include "state.h"

/* This probably should be abstracted to like a hardware implementation
So problem is that the interrupt strategy is driven by the hardware implementation slightly
For ex, my encoder is both high in between detent positions and then drives the quadrature pulses on each line with an offset
so if i put interrupts on both falling edges, i can read pin state and check which one is first
*/

// void GPIO2_Combined_0_15_IRQHandler(void){

//  }

volatile encoder_state_t current_e_state = 0;
volatile uint8_t last_combined = 0;
volatile uint8_t combined = 0;
volatile uint8_t code = 0;

void GPIO2_Combined_16_31_IRQHandler(void){
    GPIO_PortClearInterruptFlags(GPIO2, 1U << KNOB_A_GPIO_A); //clear A
    GPIO_PortClearInterruptFlags(GPIO2, 1U << KNOB_A_GPIO_B); //clear B


    // read pin states
    if (adjustMailbox.encoder_a == 0){
		uint32_t pin_state_a = GPIO_PinReadPadStatus(KNOB_A_GPIO, KNOB_A_GPIO_A);
		uint32_t pin_state_b = GPIO_PinReadPadStatus(KNOB_A_GPIO, KNOB_A_GPIO_B);

		//
		combined = (pin_state_a << 1) | pin_state_b;
		code = (last_combined << 2) | combined;

		if(code == 0b1101 || code == 0b0100 || code == 0b0010 || code == 0b1011){
			current_e_state = CCW;
		}
		else if (code == 0b1110 || code == 0b0111 || code == 0b0001 || code == 0b1000){
			current_e_state = CW;
		}

		adjustMailbox.encoder_a = current_e_state;
    }

    // switch state
    STATE_set_current(ADJUST);

    __DSB;
 }

 gpio_pin_config_t knob_encoder_config = {
    kGPIO_DigitalInput,
    0,
	kGPIO_IntFallingEdge,
};


 void KNOB_Init(void){
     /* Enable GPIO pin interrupt */
    GPIO_PinInit(KNOB_A_GPIO, KNOB_A_GPIO_A, &knob_encoder_config);
    GPIO_PinInit(KNOB_A_GPIO, KNOB_A_GPIO_B, &knob_encoder_config);
     GPIO_PortEnableInterrupts(KNOB_A_GPIO, 1U << KNOB_A_GPIO_A);
   GPIO_PortEnableInterrupts(KNOB_A_GPIO, 1U << KNOB_A_GPIO_B);

    EnableIRQ(GPIO2_Combined_16_31_IRQn);
    
}
