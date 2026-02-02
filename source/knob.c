/*
 * knob.c
 *
 *  Created on: Oct 16, 2025
 *      Author: mcaplan
 */
#include "knob.h"
#include "state_adjust.h"
#include "state.h"
//#include "fsl_pit.h"
#include "fsl_gpt.h"
#include "timer.h"


/* This probably should be abstracted to like a hardware implementation
So problem is that the interrupt strategy is driven by the hardware implementation slightly
For ex, my encoder is both high in between detent positions and then drives the quadrature pulses on each line with an offset
so if i put interrupts on both falling edges, i can read pin state and check which one is first
*/

// set up the encoder structs
encoder_t encoder_a = {
		.gpio =  KNOB_GPIO,
		.pin_a = KNOB_A_GPIO_A,
		.pin_b = KNOB_A_GPIO_B
};

encoder_t encoder_b = {
		.gpio =  KNOB_GPIO,
		.pin_a = KNOB_B_GPIO_A,
		.pin_b = KNOB_B_GPIO_B
};

encoder_t encoder_c = {
		.gpio =  KNOB_GPIO,
		.pin_a = KNOB_C_GPIO_A,
		.pin_b = KNOB_C_GPIO_B
};


volatile encoder_state_t current_e_state = 0;
volatile uint8_t last_combined = 0;
volatile uint8_t combined = 0;
volatile uint8_t code = 0;

// for debounce
volatile int e_now = 0;
volatile int encoder_last_us = 0;

#define ENCODER_DEBOUNCE_TIME_US 500

static const int8_t delta_table[16] = {
        0,  1, -1, 127,
       -1,  0, 127, 1,
        1, 127,  0, -1,
       127, -1,  1,  0
    };

uint8_t last_a_state;
uint8_t last_b_state;
uint8_t last_c_state;

int gpt1_int_c = 0;
uint32_t knob_gpio_state;

void GPT1_IRQHandler(){
	GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare1Flag);

	knob_gpio_state = KNOB_GPIO->DR;
	uint8_t a_pin_a = (knob_gpio_state >> encoder_a.pin_a) & 1u;
	uint8_t a_pin_b = (knob_gpio_state >> encoder_a.pin_b) & 1u;
	uint8_t a_state = (a_pin_a << 1) | a_pin_b;

	uint8_t b_pin_a = (knob_gpio_state >> encoder_b.pin_a) & 1u;
	uint8_t b_pin_b = (knob_gpio_state >> encoder_b.pin_b) & 1u;
	uint8_t b_state = (b_pin_a << 1) | b_pin_b;

	uint8_t c_pin_a = (knob_gpio_state >> encoder_c.pin_a) & 1u;
	uint8_t c_pin_b = (knob_gpio_state >> encoder_c.pin_b) & 1u;
	uint8_t c_state = (c_pin_a << 1) | c_pin_b;

	int8_t a_delta = 0;
	int8_t b_delta = 0;
	int8_t c_delta = 0;

	if(a_state != last_a_state){
	  // update a
	  code = (last_a_state << 2)| a_state;
	  a_delta = delta_table[code];
	  last_a_state = a_state;
	}
	if(b_state != last_b_state){
	  // update b
	  code = (last_b_state << 2)| b_state;
	  b_delta = delta_table[code];
	  last_b_state = b_state;
	}
	if(c_state != last_c_state){
	  // update c
	  code = (last_c_state << 2)| c_state;
	  c_delta = delta_table[code];
	  last_c_state = c_state;
	}

	int pending = 0;
	if(a_delta!= 127 && a_delta != 0){
	  // update mailbox
	  adjustMailbox.encoder_a = a_delta;
	  pending = 1;
	}
	if(b_delta!= 127 && b_delta != 0){
	  // update mailbox
	  adjustMailbox.encoder_b = b_delta;
	  pending = 1;
	}
	if(c_delta!= 127 && c_delta != 0){
	  // update mailbox
	  adjustMailbox.encoder_c = c_delta;
	  pending = 1;
	}

	if(pending){
		STATE_set_current(ADJUST);
	}

	TIMER_ScheduleInterrupt(TIMER_GetCurrentUs() + GPT_INTERRUPT_PERIOD_US);
	gpt1_int_c++;
}

void GPIO2_Combined_16_31_IRQHandler(void){
//	create a full interrupt mask
uint32_t mask = (1u << encoder_a.pin_a) | (1u << encoder_a.pin_b) |
        (1u << encoder_b.pin_a) | (1u << encoder_b.pin_b) |
        (1u << encoder_c.pin_a) |  (1u << encoder_c.pin_b);

//	need to clear all the flags
  GPIO_PortClearInterruptFlags(GPIO2, mask); //clear all interrupt sources

  // ignore spurious edges (debounce)
  e_now = TIMER_GetCurrentUs();
  if((e_now - encoder_last_us) < ENCODER_DEBOUNCE_TIME_US){
    return;
  }

//    read the gpio
uint32_t knob_gpio_state = KNOB_GPIO->DR;

uint8_t a_pin_a = (knob_gpio_state >> encoder_a.pin_a) & 1u;
uint8_t a_pin_b = (knob_gpio_state >> encoder_a.pin_b) & 1u;
uint8_t a_state = (a_pin_a << 1) | a_pin_b;

uint8_t b_pin_a = (knob_gpio_state >> encoder_b.pin_a) & 1u;
uint8_t b_pin_b = (knob_gpio_state >> encoder_b.pin_b) & 1u;
uint8_t b_state = (b_pin_a << 1) | b_pin_b;

uint8_t c_pin_a = (knob_gpio_state >> encoder_c.pin_a) & 1u;
uint8_t c_pin_b = (knob_gpio_state >> encoder_c.pin_b) & 1u;
uint8_t c_state = (c_pin_a << 1) | c_pin_b;

int8_t a_delta = 0;
int8_t b_delta = 0;
int8_t c_delta = 0;

if(a_state != last_a_state){
  // update a
  code = (last_a_state << 2)| a_state;
  a_delta = delta_table[code];
  last_a_state = a_state;
}
if(b_state != last_b_state){
  // update b
  code = (last_b_state << 2)| b_state;
  b_delta = delta_table[code];
  last_b_state = b_state;
}
if(c_state != last_c_state){
  // update c
  code = (last_c_state << 2)| c_state;
  c_delta = delta_table[code];
  last_c_state = c_state;
}

if(a_delta!= 127){
  // update mailbox
  adjustMailbox.encoder_a = a_delta;
}
if(b_delta!= 127){
  // update mailbox
  adjustMailbox.encoder_b = b_delta;
}
if(c_delta!= 127){
  // update mailbox
  adjustMailbox.encoder_c = c_delta;
}

STATE_set_current(ADJUST);

}

gpio_pin_config_t knob_encoder_config = {
  kGPIO_DigitalInput,
  0,
kGPIO_IntFallingEdge,
};


void KNOB_Init(void){
    /* Enable GPIO pin interrupt */
  GPIO_PinInit(encoder_a.gpio, encoder_a.pin_a, &knob_encoder_config);
  GPIO_PinInit(encoder_a.gpio, encoder_a.pin_b, &knob_encoder_config);
  GPIO_PinInit(encoder_b.gpio, encoder_b.pin_a, &knob_encoder_config);
  GPIO_PinInit(encoder_b.gpio, encoder_b.pin_b, &knob_encoder_config);
  GPIO_PinInit(encoder_c.gpio, encoder_c.pin_a, &knob_encoder_config);
  GPIO_PinInit(encoder_c.gpio, encoder_c.pin_b, &knob_encoder_config);
  
	// init encoder pin state and times
	int now = TIMER_GetCurrentUs();
	encoder_a.last_edge_ms = now;
	encoder_b.last_edge_ms = now;
	encoder_c.last_edge_ms = now;

	uint32_t encoder_port = KNOB_GPIO->DR;

	encoder_a.last_state = (((encoder_port >> encoder_a.pin_a) & 0x1U) << 1) | ((encoder_port >> encoder_a.pin_b) & 0x1U);
	encoder_b.last_state = (((encoder_port >> encoder_b.pin_a) & 0x1U) << 1) | ((encoder_port >> encoder_b.pin_b) & 0x1U);
	encoder_c.last_state = (((encoder_port >> encoder_c.pin_a) & 0x1U) << 1) | ((encoder_port >> encoder_c.pin_b) & 0x1U);

//   GPIO_PortEnableInterrupts(encoder_a.gpio, 1U << encoder_a.pin_a);
//   GPIO_PortEnableInterrupts(encoder_a.gpio, 1U << encoder_a.pin_b);
//   GPIO_PortEnableInterrupts(encoder_b.gpio, 1U << encoder_b.pin_a);
//   GPIO_PortEnableInterrupts(encoder_b.gpio, 1U << encoder_b.pin_b);
//   GPIO_PortEnableInterrupts(encoder_c.gpio, 1U << encoder_c.pin_a);
//   GPIO_PortEnableInterrupts(encoder_c.gpio, 1U << encoder_c.pin_b);

//   EnableIRQ(GPIO2_Combined_16_31_IRQn);

//set up PIT timer
//	pit_config_t pit_config;
//	PIT_GetDefaultConfig(&pit_config);
//	PIT_Init(PIT, &pit_config);
//	PIT_SetTimerPeriod(PIT, 0, 75000);
//
//	PIT_EnableInterrupts(PIT, 0, kPIT_TimerInterruptEnable);
//
//	PIT_StartTimer(PIT, 0);
//	EnableIRQ(PIT_IRQn);

	TIMER_TurnOnInterrupts();

}
