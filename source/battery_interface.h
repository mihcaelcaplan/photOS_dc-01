/*
 * battery_interface.h
 *
 *  Created on: Jul. 17, 2025
 *      Author: mcaplan
 */

#ifndef BATTERY_INTERFACE_H_
#define BATTERY_INTERFACE_H_

// 
typedef enum {
    CHARGED = 1,
    CHARGING = 2,
    LOW = 3,
    TOO_LOW = 4,
} battery_level_t;

typedef enum  {
	PMIC_Not_Connected = 0x00,
	PMIC_Connected_SDP = 0x05, // 0101
	PMIC_Connected_CDP = 0x06, // 0110
	PMIC_Connected_DCP = 0x07, // 0111
} pmic_connected_t;



// these get actually defined in board/lcd_xxxx.c

pmic_connected_t BATTERY_Init(void);
battery_level_t BATTERY_Get_Level(void);

#endif /* BATTERY_INTERFACE_H_ */
