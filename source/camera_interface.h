/*
 * camera_interface.h
 *
 *  Created on: Jun 11, 2025
 *      Author: mcaplan
 */

#ifndef CAMERA_INTERFACE_H_
#define CAMERA_INTERFACE_H_

// these get actually defined in board/camera_xxxx.c

void CAMERA_Init(void);
void CAMERA_Run(void);
void CAMERA_Stop(void);


#endif /* CAMERA_INTERFACE_H_ */
