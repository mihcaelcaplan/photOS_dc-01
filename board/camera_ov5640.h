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
#include "global_buffers.h"

// internal prototypes
void OV5640_Init(void);

uint32_t OV5640_I2CRead(uint32_t value);

void OV5640_I2CWrite(uint32_t data);

void CAMERA_Init(void);
void CAMERA_Run(void);
void CAMERA_Stop(void);


void binAndInterpolateForDisplay(uint8_t* source_buffer, uint8_t* dest_buffer, zoom_level_t level);
void interpolateForDisplay(uint8_t* source_buffer, uint8_t* dest_buffer);
void processForDisplay(uint32_t* source_buffer, uint32_t* dest_buffer);
void transfer_test(void);

// Volatile variables accessible from other modules
extern volatile bool pending_frame;
extern volatile uint32_t pending_frame_sa;
extern volatile uint32_t active_frame_sa;

typedef struct {
    uint8_t exp_width;
    uint32_t exposure;
    uint8_t gain;
    uint32_t vts;
} ov5640_settings_t;

extern ov5640_settings_t ov5640_settings;


// access and adjustment for external world
void OV5640_SetExposure(uint32_t exposure_midpoint);
void OV5640_SetGain(uint8_t gain);


#endif /* CAMERA_OV5640_H_ */
