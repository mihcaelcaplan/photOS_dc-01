/*
 * display_interface.h
 *
 *  Created on: Jun 6, 2025
 *      Author: mcaplan
 */

#ifndef DISPLAY_INTERFACE_H_
#define DISPLAY_INTERFACE_H_


// these get actually defined in board/lcd_xxxx.c

void DISPLAY_Init(void);
void DISPLAY_Run(void);
void DISPLAY_Stop(void);

#endif /* DISPLAY_INTERFACE_H_ */
