/*
 * storage_sd_device.h
 *
 *  Created on: Jun 16, 2025
 *      Author: mcaplan
 */

#include "sdmmc_config.h"
#include "fsl_common.h"
#include "usb.h"


#ifndef STORAGE_SD_DEVICE_H_
#define STORAGE_SD_DEVICE_H_


/*SD Card related */
#define USB_DEVICE_DISK_BLOCK_SIZE_POWER (9U)


void BOARD_USB_Disk_Config(uint8_t usbPriorty);

/*!
 * @brief device msc card init function.
 *
 * This function initialize the card.
 * @return kStatus_USB_Success or error.
 */
uint8_t USB_DeviceMscDiskStorageInit(void);

status_t USB_Disk_WriteBlocks(const uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);
status_t USB_Disk_ReadBlocks(uint8_t *buffer, uint32_t startBlock, uint32_t blockCount);
uint32_t USB_Disk_GetBlockSize();
uint32_t USB_Disk_GetBlockCount();




#endif /* STORAGE_SD_DEVICE_H_ */
