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
#include"lcd_st7701.h"
#include "utils.h"


uint32_t txCount = 0;

//the magic sequence from https://cdn-shop.adafruit.com/product-files/5826/GX_ST7701S+HSD4.0_480480_RGB_V0.txt
void ST7701_Manufacturer_Init(void){


	//PAGE1
	ST7701_SPIWrite(0xFF, COMMAND);
	ST7701_SPIWrite(0x77, DATA);
	ST7701_SPIWrite(0x01, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x10, DATA);



	ST7701_SPIWrite(0xC0, COMMAND);
	ST7701_SPIWrite(0x3B, DATA);
	ST7701_SPIWrite(0x00, DATA);

	ST7701_SPIWrite(0xC1, COMMAND);
	ST7701_SPIWrite(0x0D, DATA);
	ST7701_SPIWrite(0x02, DATA);

	ST7701_SPIWrite(0xC2, COMMAND);
	ST7701_SPIWrite(0x31, DATA);
	ST7701_SPIWrite(0x05, DATA);

	ST7701_SPIWrite(0xCd, COMMAND);
	ST7701_SPIWrite(0x08, DATA);

	ST7701_SPIWrite(0xB0, COMMAND);
	ST7701_SPIWrite(0x00, DATA); //Positive Voltage Gamma Control
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0x18, DATA);
	ST7701_SPIWrite(0x0E, DATA);
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0x06, DATA);
	ST7701_SPIWrite(0x07, DATA);
	ST7701_SPIWrite(0x08, DATA);
	ST7701_SPIWrite(0x07, DATA);
	ST7701_SPIWrite(0x22, DATA);
	ST7701_SPIWrite(0x04, DATA);
	ST7701_SPIWrite(0x12, DATA);
	ST7701_SPIWrite(0x0F, DATA);
	ST7701_SPIWrite(0xAA, DATA);
	ST7701_SPIWrite(0x31, DATA);
	ST7701_SPIWrite(0x18, DATA);


	ST7701_SPIWrite(0xB1, COMMAND);
	ST7701_SPIWrite(0x00, DATA); //Negative Voltage Gamma Control
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0x19, DATA);
	ST7701_SPIWrite(0x0E, DATA);
	ST7701_SPIWrite(0x12, DATA);
	ST7701_SPIWrite(0x07, DATA);
	ST7701_SPIWrite(0x08, DATA);
	ST7701_SPIWrite(0x08, DATA);
	ST7701_SPIWrite(0x08, DATA);
	ST7701_SPIWrite(0x22, DATA);
	ST7701_SPIWrite(0x04, DATA);
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0xA9, DATA);
	ST7701_SPIWrite(0x32, DATA);
	ST7701_SPIWrite(0x18, DATA);

	//PAGE1
	ST7701_SPIWrite(0xFF, COMMAND);
	ST7701_SPIWrite(0x77, DATA);
	ST7701_SPIWrite(0x01, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x11, DATA);


	ST7701_SPIWrite(0xB0, COMMAND);
	ST7701_SPIWrite(0x60, DATA); //Vop=4.7375v

	ST7701_SPIWrite(0xB1, COMMAND);
	ST7701_SPIWrite(0x32, DATA); //VCOM=32

	ST7701_SPIWrite(0xB2, COMMAND);
	ST7701_SPIWrite(0x07, DATA); //VGH=15v

	ST7701_SPIWrite(0xB3, COMMAND);
	ST7701_SPIWrite(0x80, DATA);

	ST7701_SPIWrite(0xB5, COMMAND);
	ST7701_SPIWrite(0x49, DATA); //VGL=-10.17v

	ST7701_SPIWrite(0xB7, COMMAND);
	ST7701_SPIWrite(0x85, DATA);

	ST7701_SPIWrite(0xB8, COMMAND);
	ST7701_SPIWrite(0x21, DATA); //AVDD=6.6 & AVCL=-4.6

	ST7701_SPIWrite(0xC1, COMMAND);
	ST7701_SPIWrite(0x78, DATA);

	ST7701_SPIWrite(0xC2, COMMAND);
	ST7701_SPIWrite(0x78, DATA);

	ST7701_SPIWrite(0xE0, COMMAND);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x1B, DATA);
	ST7701_SPIWrite(0x02, DATA);

	ST7701_SPIWrite(0xE1, COMMAND);
	ST7701_SPIWrite(0x08, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x07, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x44, DATA);
	ST7701_SPIWrite(0x44, DATA);



	ST7701_SPIWrite(0xE2, COMMAND);
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0x44, DATA);
	ST7701_SPIWrite(0x44, DATA);
	ST7701_SPIWrite(0xED, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0xEC, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);


	ST7701_SPIWrite(0xE3, COMMAND);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0x11, DATA);


	ST7701_SPIWrite(0xE4, COMMAND);
	ST7701_SPIWrite(0x44, DATA);
	ST7701_SPIWrite(0x44, DATA);

	ST7701_SPIWrite(0xE5, COMMAND);
	ST7701_SPIWrite(0x0A, DATA);
	ST7701_SPIWrite(0xE9, DATA);
	ST7701_SPIWrite(0xD8, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x0C, DATA);
	ST7701_SPIWrite(0xEB, DATA);
	ST7701_SPIWrite(0xD8, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x0E, DATA);
	ST7701_SPIWrite(0xED, DATA);
	ST7701_SPIWrite(0xD8, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x10, DATA);
	ST7701_SPIWrite(0xEF, DATA);
	ST7701_SPIWrite(0xD8, DATA);
	ST7701_SPIWrite(0xA0, DATA);




	ST7701_SPIWrite(0xE6, COMMAND);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x11, DATA);
	ST7701_SPIWrite(0x11, DATA);

	ST7701_SPIWrite(0xE7, COMMAND);
	ST7701_SPIWrite(0x44, DATA);
	ST7701_SPIWrite(0x44, DATA);


	ST7701_SPIWrite(0xE8, COMMAND);
	ST7701_SPIWrite(0x09, DATA);
	ST7701_SPIWrite(0xE8, DATA);
	ST7701_SPIWrite(0xD8, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x0B, DATA);
	ST7701_SPIWrite(0xEA, DATA);
	ST7701_SPIWrite(0xD8, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x0D, DATA);
	ST7701_SPIWrite(0xEC, DATA);
	ST7701_SPIWrite(0xD8, DATA);
	ST7701_SPIWrite(0xA0, DATA);
	ST7701_SPIWrite(0x0F, DATA);
	ST7701_SPIWrite(0xEE, DATA);
	ST7701_SPIWrite(0xD8, DATA);
	ST7701_SPIWrite(0xA0, DATA);


	ST7701_SPIWrite(0xEB, COMMAND);
	ST7701_SPIWrite(0x02, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0xE4, DATA);
	ST7701_SPIWrite(0xE4, DATA);
	ST7701_SPIWrite(0x88, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x40, DATA);


	ST7701_SPIWrite(0xEC, COMMAND);
	ST7701_SPIWrite(0x3C, DATA);
	ST7701_SPIWrite(0x00, DATA);


	ST7701_SPIWrite(0xED, COMMAND);
	ST7701_SPIWrite(0xAB, DATA);
	ST7701_SPIWrite(0x89, DATA);
	ST7701_SPIWrite(0x76, DATA);
	ST7701_SPIWrite(0x54, DATA);
	ST7701_SPIWrite(0x02, DATA);
	ST7701_SPIWrite(0xFF, DATA);
	ST7701_SPIWrite(0xFF, DATA);
	ST7701_SPIWrite(0xFF, DATA);
	ST7701_SPIWrite(0xFF, DATA);
	ST7701_SPIWrite(0xFF, DATA);
	ST7701_SPIWrite(0xFF, DATA);
	ST7701_SPIWrite(0x20, DATA);
	ST7701_SPIWrite(0x45, DATA);
	ST7701_SPIWrite(0x67, DATA);
	ST7701_SPIWrite(0x98, DATA);
	ST7701_SPIWrite(0xBA, DATA);

	ST7701_SPIWrite(0x36, COMMAND);
	ST7701_SPIWrite(0x00, DATA);


	//-----------VAP & VAN---------------
	ST7701_SPIWrite(0xFF, COMMAND);
	ST7701_SPIWrite(0x77, DATA);
	ST7701_SPIWrite(0x01, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x13, DATA);

	ST7701_SPIWrite(0xE5, COMMAND);
	ST7701_SPIWrite(0xE4, DATA);

	ST7701_SPIWrite(0xFF, COMMAND);
	ST7701_SPIWrite(0x77, DATA);
	ST7701_SPIWrite(0x01, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);
	ST7701_SPIWrite(0x00, DATA);

	ST7701_SPIWrite(0x3a, COMMAND);
	ST7701_SPIWrite(0x66, DATA);
	ST7701_SPIWrite(0x21, COMMAND);
	simpleDelay(1U);
	ST7701_SPIWrite(0x11, COMMAND);
	simpleDelay(120U);
	ST7701_SPIWrite(0x29, COMMAND);


}


// reset the spi just to be robust
void ST7701_Init(void){

	ST7701_Manufacturer_Init();

//	enable
	if ((LPSPI_GetStatusFlags(SPI_LCD_CONTROL) & (uint32_t)kLPSPI_ModuleBusyFlag) != 0U)
		    {
				PRINTF("LPSPI Busy...");
		        return;
		    }
	LPSPI_Enable(SPI_LCD_CONTROL, true);

//	clear common flags and configure bit width (0=1b) and chip select (0=cs0)
	SPI_LCD_CONTROL->TCR = (SPI_LCD_CONTROL->TCR & ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK |
													 LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_TXMSK_MASK |
													 LPSPI_TCR_PCS_MASK))
	                                             | LPSPI_TCR_WIDTH(0)
												 | LPSPI_TCR_PCS(0) ;

//	wait for empty tx fifo (shared btw. command and data)
	while( (SPI_LCD_CONTROL->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT != 0 ){
		PRINTF(".");
	}

}
void ST7701_SPIWriteReg(uint8_t address, uint32_t data){

	ST7701_SPIWrite(address, COMMAND);
	ST7701_SPIWrite(data, DATA);
}

void ST7701_SPIWrite(uint32_t data, transmission_t type){

	uint32_t data_out;

	//	this sets the data/command bit for the ST7701
		if(type == COMMAND){
			data_out = data | (COMMAND << 8); // Build 9-bit frame: D/C=0 (command)
		}
		else if(type == DATA){
			data_out = data | (DATA << 8); // Build 9-bit frame: D/C=1 (data)
		}
		else{
			PRINTF("incorrect type, error");
		}

		if ((LPSPI_GetStatusFlags(SPI_LCD_CONTROL) & (uint32_t)kLPSPI_ModuleBusyFlag) != 0U)
		    {
				PRINTF("LPSPI Busy...");
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
	//	cont off to relax cs between transmits
	//	tx mask 0 so transfer starts when data added to fifo
	//	rx mask 1 so no data rcv
		SPI_LCD_CONTROL->TCR = (SPI_LCD_CONTROL->TCR & ~(LPSPI_TCR_FRAMESZ_MASK | LPSPI_TCR_CONT_MASK | LPSPI_TCR_TXMSK_MASK | LPSPI_TCR_RXMSK_MASK))
													 |   LPSPI_TCR_FRAMESZ(8) |   LPSPI_TCR_CONT(0) |   LPSPI_TCR_TXMSK(0) |   LPSPI_TCR_RXMSK(1);

	//	wait for empty tx fifo (shared btw. command and data)
		while( ((SPI_LCD_CONTROL->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT) != 0U){}

	//send the data to transmit to the command/transmit fifo
		SPI_LCD_CONTROL->TDR = data_out;

//	wait for empty tx fifo (shared btw. command and data)
		while( ((SPI_LCD_CONTROL->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT) != 0U){}

}



// this will always be 9 bits with a d/c bit as the first part of the packet
uint32_t ST7701_SPIReadReg(uint32_t address){

//	this sets the data/command bit for the ST7701
	uint32_t data_out = address | (COMMAND << 8); // Build 9-bit frame: D/C=0 (command)

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

	return data_in;

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
	PRINTF("Read 0x%02x, received 0x%02X\r\n", 0xDA, response);

	 response = ST7701_SPIReadReg(0xDB);
	PRINTF("Read 0x%02x, received 0x%02X\r\n", 0xDB, response);

	 response = ST7701_SPIReadReg(0xDC);
	PRINTF("Read 0x%02x, received 0x%02X\r\n", 0xDC, response);

 // manufacturer init code


// set up mode correctly

}


void DISPLAY_Run(void){}



