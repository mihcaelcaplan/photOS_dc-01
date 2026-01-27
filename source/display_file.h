/*
 * display_file.h
 *
 *  Created on: Jun 16, 2025
 *      Author: mcaplan
 */

#ifndef DISPLAY_FILE_H_
#define DISPLAY_FILE_H_

#include "ff.h"

// filesystem globals
extern FATFS g_fileSystem;
extern const TCHAR driverName[3U];


void DISPLAY_showStoredFile(void);

int MOUNT_SDCard(void);


#endif /* DISPLAY_FILE_H_ */
