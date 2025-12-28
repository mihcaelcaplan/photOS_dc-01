/*
 * jpeg_interface.h
 *
 *  Created on: Jun 19, 2025
 *      Author: mcaplan
 */

#ifndef JPEG_INTERFACE_H_
#define JPEG_INTERFACE_H_

#include "ff.h"
#include "fsl_common.h"

// decode a File and put it in the buffer.
void jpeg_decode(FIL *file, uint8_t *buffer);

// encode the contents of a buffer and put it in a file
void jpeg_encode(uint8_t *buffer, FIL *file);



#endif /* JPEG_INTERFACE_H_ */
