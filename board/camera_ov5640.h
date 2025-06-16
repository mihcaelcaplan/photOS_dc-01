/*
 * camera_ov5640.h
 *
 *  Created on: Jun 11, 2025
 *      Author: mcaplan
 */

#ifndef CAMERA_OV5640_H_
#define CAMERA_OV5640_H_

#define I2C_CAMERA_CONTROL

#include "fsl_common.h"



// internal prototypes
void OV5640_Init(void);

uint32_t OV5640_I2CRead(uint32_t value);

void OV5640_I2CWrite(uint32_t data);


void CAMERA_Init(void);
void CAMERA_Run(void);
void CAMERA_Stop(void);




#endif /* CAMERA_OV5640_H_ */
