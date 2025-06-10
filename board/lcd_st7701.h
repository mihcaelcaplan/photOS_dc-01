/*
 * lcd_st7701.h
 *
 *  Created on: Jun 9, 2025
 *      Author: mcaplan
 */

#ifndef LCD_ST7701_H_
#define LCD_ST7701_H_

#define SPI_LCD_CONTROL LPSPI3

// for ST7701 differentiation, nice syntax
typedef enum {
	COMMAND = 0u,
	DATA = 1u
} transmission_t;



// internal prototypes
void ST7701_Init(void);
uint32_t ST7701_SPIRead(uint32_t value);
void ST7701_SPIWriteReg(uint8_t address, uint32_t data);
void ST7701_SPIWrite(uint32_t data, transmission_t type);
uint32_t ST7701_SPIRead_SDK(uint32_t command);

#endif /* LCD_ST7701_H_ */
