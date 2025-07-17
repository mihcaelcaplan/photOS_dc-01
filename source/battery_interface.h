/*
 * battery_interface.h
 *
 *  Created on: Jul. 17, 2025
 *      Author: mcaplan
 */

#ifndef BATTERY_INTERFACE_H_
#define BATTERY_INTERFACE_H_

// 
typdef enum {
    CHARGED = 0x00,
    CHARGING = 0x01,
    LOW = 0x02,
    TOO_LOW = 0x04,
} battery_level_t;


// these get actually defined in board/lcd_xxxx.c

void BATTERY_Init(void);
void BATTERY_Get_Level(void);

#endif /* BATTERY_INTERFACE_H_ */
