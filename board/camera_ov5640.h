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

typedef enum {
    zoom_level_1 = 4,
    zoom_level_2 = 2,
    zoom_level_3 = 1
} zoom_level_t;

void binAndInterpolateForDisplay(uint8_t* source_buffer, uint8_t* dest_buffer, zoom_level_t level);
void interpolateForDisplay(uint8_t* source_buffer, uint8_t* dest_buffer);
void processForDisplay(uint32_t* source_buffer, uint32_t* dest_buffer);
void transfer_test(void);

// Volatile variables accessible from other modules
extern volatile bool pending_frame;
extern volatile uint32_t pending_frame_sa;
extern volatile uint32_t active_frame_sa;

#endif /* CAMERA_OV5640_H_ */
