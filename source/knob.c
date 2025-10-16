/*
 * knob.c
 *
 *  Created on: Oct 16, 2025
 *      Author: mcaplan
 */
#include "knob.h"

// void GPIO2_Combined_0_15_IRQHandler(void){

//  }

volatile quad_state_t last_e_state;

void GPIO2_Combined_16_31_IRQHandler(void){
    GPIO_PortClearInterruptFlags(GPIO2, 1U << KNOB_A_GPIO_A); //clear A
    GPIO_PortClearInterruptFlags(GPIO2, 1U << KNOB_A_GPIO_B); //clear B

    // read pin states
    uint32_t pin_state_a = GPIO_PinReadPadStatus(KNOB_A_GPIO, KNOB_A_GPIO_A);
    uint32_t pin_state_b = GPIO_PinReadPadStatus(KNOB_A_GPIO, KNOB_A_GPIO_B);

    quad_state_t current_e_state;
    // pins -> quad state
    if((pin_state_a & pin_state_b) == 1){ // 11
        current_e_state = hahb;
    }
    else{
        if(pin_state_a == 1){ // 10
            current_e_state = halb;
        }
        else if(pin_state_b == 1){ // 01
            current_e_state = lahb;
        }
        else{ // 00
            current_e_state = lalb;
        }
    }

    output_state_t encoder_output = no_change;
    // quad_state -> rotation state
    // basically if a changes first it's forward if b changes first it's reverse
    if(last_e_state == hahb){
        if(current_e_state == lahb){
            encoder_output = forward;
        }
        else if(current_e_state == halb){
            encoder_output = reverse;
        }
    }
    else if(last_e_state == halb){
        if(current_e_state == hahb){
            encoder_output = forward;
        }
        else if(current_e_state == lalb){
            encoder_output = reverse;
        }
    }
    else if(last_e_state == lahb){
        if(current_e_state == lalb){
            encoder_output = forward;
        }
        else if(current_e_state == hahb){
            encoder_output = reverse;
        } 
    }
    else if(last_e_state == lalb){
        if(current_e_state == halb){
            encoder_output = forward;
        }
        else if(current_e_state == lahb){
            encoder_output = reverse;
        } 
    }

    // set the new last state before exit
    last_e_state = current_e_state;
    __DSB;
 }

 gpio_pin_config_t knob_encoder_config = {
    kGPIO_DigitalInput,
    0,
	kGPIO_IntRisingEdge,
};


 void KNOB_Init(void){
     /* Enable GPIO pin interrupt */
    GPIO_PinInit(KNOB_A_GPIO, KNOB_A_GPIO_A, &knob_encoder_config);
    GPIO_PinInit(KNOB_A_GPIO, KNOB_A_GPIO_B, &knob_encoder_config);
    GPIO_PortEnableInterrupts(KNOB_A_GPIO, 1U << KNOB_A_GPIO_A);
    GPIO_PortEnableInterrupts(KNOB_A_GPIO, 1U << KNOB_A_GPIO_B);

    EnableIRQ(GPIO2_Combined_16_31_IRQn);
    
}
