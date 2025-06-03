/*
 * usb_mux.h
 *
 *  Created on: Jun 3, 2025
 *      Author: mcaplan
 */

#ifndef USB_MUX_H_
#define USB_MUX_H_

// sets the mux up, including configuring the gpio to be the correct state after power up
void MUX_Init(void);

//main interface
void MUX_ToPMIC(void);

void MUX_ToUSBC(void);

#endif /* USB_MUX_H_ */
