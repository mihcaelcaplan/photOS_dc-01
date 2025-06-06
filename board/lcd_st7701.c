/*
 * lcd_st7701s.h
 *
 *  Created on: Jun 5, 2025
 *      Author: mcaplan
 */

#include "display_interface.h"

#include "fsl_lpspi.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"


#define SPI_LCD_CONTROL LPSPI3

// internal prototypes
void ST7701_Init(void);
uint8_t ST7701_SPIReadReg(uint8_t value);

// set up transfer and receive buffers (same size as SPI FIFO word)
//uint32_t TxData;
//uint32_t RxData;

// for ST7701 differentiation, nice syntax
typedef enum {
	COMMAND = 0u,
	DATA = 1u
} transmission_t;

uint32_t txCount = 0;

// reset the spi just to be robust
void ST7701_Init(void){

	PRINTF("VERID: 0x%08X\n", SPI_LCD_CONTROL->VERID);

////	enable
//
	if ((LPSPI_GetStatusFlags(SPI_LCD_CONTROL) & (uint32_t)kLPSPI_ModuleBusyFlag) != 0U)
		    {
				PRINTF("LPSPI Busy...");
		        return;
		    }
	LPSPI_Enable(SPI_LCD_CONTROL, true);
////	clear common flags and configure bit width (0=1b) and chip select (0=cs0)
	SPI_LCD_CONTROL->TCR = (SPI_LCD_CONTROL->TCR & ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK |
													 LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_TXMSK_MASK |
													 LPSPI_TCR_PCS_MASK))
	                                             | LPSPI_TCR_WIDTH(0)
												 | LPSPI_TCR_PCS(0) ;

//	wait for empty tx fifo (shared btw. command and data)
	while( SPI_LCD_CONTROL->FSR & LPSPI_FSR_TXCOUNT_MASK >> LPSPI_FSR_TXCOUNT_SHIFT != 0 ){
		PRINTF(".");
	}

}

// this will always be 9 bits with a d/c bit as the first part of the packet
uint8_t ST7701_SPIReadReg(uint8_t value){

//	this sets the data/command bit for the ST7701
	uint32_t data_out = value | (COMMAND << 8); // Build 9-bit frame: D/C=0 (command)

	if ((LPSPI_GetStatusFlags(SPI_LCD_CONTROL) & (uint32_t)kLPSPI_ModuleBusyFlag) != 0U)
	    {
			PRINTF("LPSPI Busy...");
	        return 0xFF;
	    }

//	reset the tcr by clearing relevant bits
	SPI_LCD_CONTROL->TCR = (SPI_LCD_CONTROL->TCR & ~(LPSPI_TCR_CONT_MASK |  LPSPI_TCR_CONTC_MASK |
												     LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_TXMSK_MASK |
												     LPSPI_TCR_PCS_MASK |   LPSPI_TCR_WIDTH_MASK))
		                                         |   LPSPI_TCR_WIDTH(0)| LPSPI_TCR_PCS(0) ;

//	wait for empty tx fifo (shared btw. command and data)
	while( ((SPI_LCD_CONTROL->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT) != 0U){}


//  set up the first part of the half duplex transfer
// 	frame size for 9 bit transmit
//	cont on to hold cs down
//	tx mask 0 so transfer starts when data added to fifo
//	rx mask 1 so no data rcv
	SPI_LCD_CONTROL->TCR = (SPI_LCD_CONTROL->TCR & ~(LPSPI_TCR_FRAMESZ_MASK | LPSPI_TCR_CONT_MASK | LPSPI_TCR_TXMSK_MASK | LPSPI_TCR_RXMSK_MASK))
												 |   LPSPI_TCR_FRAMESZ(8) |   LPSPI_TCR_CONT(1) |   LPSPI_TCR_TXMSK(0) |   LPSPI_TCR_RXMSK(1);

//	wait for empty tx fifo (shared btw. command and data)
	while( ((SPI_LCD_CONTROL->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT) != 0U){}

//send the data to transmit to the command/transmit fifo
	SPI_LCD_CONTROL->TDR = data_out;


//	wait for empty tx fifo (shared btw. command and data)
		while( ((SPI_LCD_CONTROL->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT) != 0U){}

//  set up the second part of the half duplex transfer
// 	frame size for 8 bit receive
//	cont off to release cs after receive
//  contc on to keep assert cs
//	tx mask 1 so transfer starts when data added to fifo
//	rx mask 0
	SPI_LCD_CONTROL->TCR = (SPI_LCD_CONTROL->TCR & ~(LPSPI_TCR_FRAMESZ_MASK | LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK | LPSPI_TCR_TXMSK_MASK | LPSPI_TCR_RXMSK_MASK))
												 |   LPSPI_TCR_FRAMESZ(7) |   LPSPI_TCR_CONT(0) |   LPSPI_TCR_CONTC(1) |   LPSPI_TCR_TXMSK(1) |   LPSPI_TCR_RXMSK(0);

//	wait for empty tx fifo (shared btw. command and data)
//	while( ((SPI_LCD_CONTROL->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT) != 0U){}

//  double check that rx fifo got filled correctly
	while(LPSPI_GetRxFifoCount(SPI_LCD_CONTROL) != 1U){}

//	get the data from rx fifo
	uint32_t data_in = SPI_LCD_CONTROL->RDR;

	return (uint8_t)data_in;

}


// definition of the main interface methods

void DISPLAY_Init(void){

//	turn backlight off
	gpio_pin_config_t lcd_pwr_enable_init = {
			kGPIO_DigitalOutput,
			0U,
			kGPIO_NoIntmode
	};

//	set up gpio
	GPIO_PinInit(GPIO1, 9U, &lcd_pwr_enable_init);


	ST7701_Init();

//	read an easy register
	uint8_t response;
	response = ST7701_SPIReadReg(0xDA);
	PRINTF("Read 0x%02x, received 0x%02x\r\n", 0xDA, response);

	 response = ST7701_SPIReadReg(0xDB);
	PRINTF("Read 0x%02x, received 0x%02x\r\n", 0xDB, response);

	 response = ST7701_SPIReadReg(0xDC);
	PRINTF("Read 0x%02x, received 0x%02x\r\n", 0xDC, response);

 // manufacturer init code


// set up mode correctly

}


void DISPLAY_Run(void){}



