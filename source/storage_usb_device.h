/*
 * storage_usb_device.h
 *
 *  Created on: Jun 16, 2025
 *      Author: mcaplan
 */

#ifndef USB_DEVICE_H_
#define USB_DEVICE_H_

#include "sdmmc_config.h"
#include "fsl_common.h"
#include "board.h"
#include "storage_sd_device.h"
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_msc.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "usb_phy.h"

/*USB related prototypes and defines*/
#define USB_DEVICE_MSC_WRITE_BUFF_SIZE (512 * 8U)
#define USB_DEVICE_MSC_READ_BUFF_SIZE  (512 * 8U)
#define CONTROLLER_ID kUSB_ControllerEhci0
#define USB_DEVICE_INTERRUPT_PRIORITY                                                                    \
    (3U) /*! @brief enable the write task. 1U supported, 0U not supported . if this macro is enable ,the \
USB_DEVICE_CONFIG_USE_TASK macro should also be enable.*/
#define USB_DEVICE_MSC_USE_WRITE_TASK (0U)
#define USB_DEVICE_MSC_BUFFER_NUMBER  (3U)
/* application define logical unit number, if LOGICAL_UNIT_SUPPORTED > USB_DEVICE_MSC_MAX_LUN, update
 * USB_DEVICE_MSC_MAX_LUN in class driver usb_device_msc.h*/
#define LOGICAL_UNIT_SUPPORTED (1U)

// Type definitions
typedef struct _usb_msc_buffer_struct
{
    uint32_t offset; /*!< Offset of the block need to access*/
    uint32_t size;   /*!< Size of the transfered data*/
    struct _usb_msc_buffer_struct *next;
    uint8_t *buffer; /*!< Buffer address of the transferred data*/
} usb_msc_buffer_struct_t;

typedef struct _usb_msc_struct
{
    usb_device_handle deviceHandle;
    class_handle_t mscHandle;
    uint8_t diskLock;
    uint8_t read_write_error;
    uint8_t currentConfiguration;
    uint8_t currentInterfaceAlternateSetting[USB_MSC_INTERFACE_COUNT];
    uint8_t speed;
    uint8_t attach;
    uint8_t stop; /* indicates this media keeps stop or not, 1: stop, 0: start */
    usb_msc_buffer_struct_t *headlist;
    usb_msc_buffer_struct_t *taillist;
    usb_msc_buffer_struct_t *transferlist;
} usb_msc_struct_t;

// External variable declarations
extern usb_device_inquiry_data_fromat_struct_t g_InquiryInfo;
extern usb_device_mode_parameters_header_struct_t g_ModeParametersHeader;
extern usb_msc_struct_t g_msc;
extern uint32_t g_mscReadRequestBuffer[USB_DEVICE_MSC_READ_BUFF_SIZE >> 2];
extern uint32_t g_mscWriteRequestBuffer[USB_DEVICE_MSC_WRITE_BUFF_SIZE >> 2];
extern usb_device_class_config_struct_t msc_config[1];
extern usb_device_class_config_list_struct_t msc_config_list;

// Function declarations
void USB_OTG1_IRQHandler(void);
void USB_OTG2_IRQHandler(void);
void USB_DeviceIsrEnable(void);
void USB_DeviceClockInit(void);
usb_status_t USB_DeviceMscCallback(class_handle_t handle, uint32_t event, void *param);
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);
void USB_DeviceApplicationInit(void);
void USB_DeviceAppStart(void);
void USB_DeviceAppStop(void);
void USB_DeviceMscApp(void);
void USB_DeviceMscAppTask(void);

#endif /* USB_DEVICE_H_ */