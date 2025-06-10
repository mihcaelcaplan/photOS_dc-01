/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    hello_world_dc-01.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"


#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_msc.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "usb_phy.h"

#include "sdmmc_config.h"

/* TODO: insert other include files here. */
#include "usb_mux.h"
#include "debug_shell.h"
#include "display_interface.h"
#include "utils.h"

/* TODO: insert other definitions and declarations here. */




/*SD Card related */
#define USB_DEVICE_DISK_BLOCK_SIZE_POWER (9U)


/* State in Disk driver. */
sd_card_t g_sd;
sd_card_t *usbDeviceMscSdcard;

void BOARD_USB_Disk_Config(uint8_t usbPriorty)
{
    BOARD_SD_Config(&g_sd, NULL, (usbPriorty - 1U), NULL);
}

/*!
 * @brief device msc card init function.
 *
 * This function initialize the card.
 * @return kStatus_USB_Success or error.
 */
uint8_t USB_DeviceMscDiskStorageInit(void)
{
    usb_status_t error = kStatus_USB_Success;
    usbDeviceMscSdcard = &g_sd;

    /* Init card. */
    if (SD_Init(usbDeviceMscSdcard))
    {
        PRINTF("\n SD card init failed \n");
        error = kStatus_USB_Error;
    }

    return error;
}

status_t USB_Disk_WriteBlocks(const uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
    return SD_WriteBlocks(usbDeviceMscSdcard, buffer, startBlock, blockCount);
}

status_t USB_Disk_ReadBlocks(uint8_t *buffer, uint32_t startBlock, uint32_t blockCount)
{
    return SD_ReadBlocks(usbDeviceMscSdcard, buffer, startBlock, blockCount);
}

uint32_t USB_Disk_GetBlockSize()
{
    return usbDeviceMscSdcard->blockSize;
}

uint32_t USB_Disk_GetBlockCount()
{
    return usbDeviceMscSdcard->blockCount;
}



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


void BOARD_InitHardware(void);
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
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




USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
usb_device_inquiry_data_fromat_struct_t g_InquiryInfo = {
    (USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER << USB_DEVICE_MSC_UFI_PERIPHERAL_QUALIFIER_SHIFT) |
        USB_DEVICE_MSC_UFI_PERIPHERAL_DEVICE_TYPE,
    (uint8_t)(USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT << USB_DEVICE_MSC_UFI_REMOVABLE_MEDIUM_BIT_SHIFT),
    USB_DEVICE_MSC_UFI_VERSIONS,
    0x02,
    USB_DEVICE_MSC_UFI_ADDITIONAL_LENGTH,
    {0x00, 0x00, 0x00},
    {'N', 'X', 'P', ' ', 'S', 'E', 'M', 'I'},
    {'N', 'X', 'P', ' ', 'M', 'A', 'S', 'S', ' ', 'S', 'T', 'O', 'R', 'A', 'G', 'E'},
    {'0', '0', '0', '1'}};
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
usb_device_mode_parameters_header_struct_t g_ModeParametersHeader = {
    /*refer to ufi spec mode parameter header*/
    0x0000, /*!< Mode Data Length*/
    0x00,   /*!<Default medium type (current mounted medium type)*/
    0x00,   /*!MODE SENSE command, a Write Protected bit of zero indicates the medium is write enabled*/
    {0x00, 0x00, 0x00, 0x00} /*!<This bit should be set to zero*/
};
/* Data structure of msc device, store the information ,such as class handle */
usb_msc_struct_t g_msc;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint32_t g_mscReadRequestBuffer[USB_DEVICE_MSC_READ_BUFF_SIZE >> 2];

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint32_t g_mscWriteRequestBuffer[USB_DEVICE_MSC_WRITE_BUFF_SIZE >> 2];

void USB_OTG1_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(g_msc.deviceHandle);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
    __DSB();
}

void USB_OTG2_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction(g_msc.deviceHandle);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
    __DSB();
}

void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber                  = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];

    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}


void USB_DeviceClockInit(void)
{
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    if (CONTROLLER_ID == kUSB_ControllerEhci0)
    {
        CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    }
    else
    {
        CLOCK_EnableUsbhs1PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
        CLOCK_EnableUsbhs1Clock(kCLOCK_Usb480M, 480000000U);
    }
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

/*!
 * @brief device msc callback function.
 *
 * This function handle the disk class specified event.
 * @param handle          The USB class  handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the class specific event.
 * @return kStatus_USB_Success or error.
 */
usb_status_t USB_DeviceMscCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Success;
    status_t errorCode = kStatus_Success;
    usb_device_lba_information_struct_t *lbaInformation;
    usb_device_lba_app_struct_t *lba;
    usb_device_ufi_app_struct_t *ufi;
    usb_device_capacity_information_struct_t *capacityInformation;
    switch (event)
    {
        case kUSB_DeviceMscEventReadResponse:
            lba = (usb_device_lba_app_struct_t *)param;
            break;
        case kUSB_DeviceMscEventWriteResponse:
            lba = (usb_device_lba_app_struct_t *)param;
            /*write the data to disk */
            if (0 != lba->size)
            {
                errorCode =
                    USB_Disk_WriteBlocks(lba->buffer, lba->offset, lba->size >> USB_DEVICE_DISK_BLOCK_SIZE_POWER);
                if (kStatus_Success != errorCode)
                {
                    g_msc.read_write_error = 1;
                    usb_echo(
                        "Write error, error = 0xx%x \t Please check write request buffer size(must be less than 128 "
                        "sectors)\r\n",
                        error);
                    error = kStatus_USB_Error;
                }
            }
            else
            {
                error = kStatus_USB_InvalidRequest;
            }
            break;
        case kUSB_DeviceMscEventWriteRequest:
            lba = (usb_device_lba_app_struct_t *)param;
/*get a buffer to store the data from host*/
            lba->buffer = (uint8_t *)&g_mscWriteRequestBuffer[0];
            break;
        case kUSB_DeviceMscEventReadRequest:
            lba         = (usb_device_lba_app_struct_t *)param;
            lba->buffer = (uint8_t *)&g_mscReadRequestBuffer[0];

            /*read the data from disk, then store these data to the read buffer*/
            errorCode = USB_Disk_ReadBlocks(lba->buffer, lba->offset, lba->size >> USB_DEVICE_DISK_BLOCK_SIZE_POWER);

            if (kStatus_Success != errorCode)
            {
                g_msc.read_write_error = 1;
                usb_echo(
                    "Read error, error = 0xx%x \t Please check read request buffer size(must be less than 128 "
                    "sectors)\r\n",
                    error);
                error = kStatus_USB_Error;
            }
            break;
        case kUSB_DeviceMscEventGetLbaInformation:
            lbaInformation                                             = (usb_device_lba_information_struct_t *)param;
            lbaInformation->logicalUnitNumberSupported                 = LOGICAL_UNIT_SUPPORTED;
            lbaInformation->logicalUnitInformations[0].lengthOfEachLba = USB_Disk_GetBlockSize();
            lbaInformation->logicalUnitInformations[0].totalLbaNumberSupports = USB_Disk_GetBlockCount();
            lbaInformation->logicalUnitInformations[0].bulkInBufferSize       = USB_DEVICE_MSC_READ_BUFF_SIZE;
            lbaInformation->logicalUnitInformations[0].bulkOutBufferSize      = USB_DEVICE_MSC_WRITE_BUFF_SIZE;
            break;
        case kUSB_DeviceMscEventTestUnitReady:
            /*change the test unit ready command's sense data if need, be careful to modify*/
            if (1U == g_msc.stop)
            {
                ufi                                    = (usb_device_ufi_app_struct_t *)param;
                ufi->requestSense->senseKey            = USB_DEVICE_MSC_UFI_NOT_READY;
                ufi->requestSense->additionalSenseCode = USB_DEVICE_MSC_UFI_ASC_MEDIUM_NOT_PRESENT;
            }
            break;
        case kUSB_DeviceMscEventInquiry:
            ufi         = (usb_device_ufi_app_struct_t *)param;
            ufi->size   = sizeof(usb_device_inquiry_data_fromat_struct_t);
            ufi->buffer = (uint8_t *)&g_InquiryInfo;
            break;
        case kUSB_DeviceMscEventModeSense:
            ufi         = (usb_device_ufi_app_struct_t *)param;
            ufi->size   = sizeof(usb_device_mode_parameters_header_struct_t);
            ufi->buffer = (uint8_t *)&g_ModeParametersHeader;
            break;
        case kUSB_DeviceMscEventModeSelectResponse:
            ufi = (usb_device_ufi_app_struct_t *)param;
            break;
        case kUSB_DeviceMscEventModeSelect:
        case kUSB_DeviceMscEventFormatComplete:
        case kUSB_DeviceMscEventRemovalRequest:
            error = kStatus_USB_InvalidRequest;
            break;
        case kUSB_DeviceMscEventRequestSense:
            break;
        case kUSB_DeviceMscEventReadCapacity:
            capacityInformation                         = (usb_device_capacity_information_struct_t *)param;
            capacityInformation->lengthOfEachLba        = USB_Disk_GetBlockSize();
            capacityInformation->totalLbaNumberSupports = USB_Disk_GetBlockCount();
            break;
        case kUSB_DeviceMscEventReadFormatCapacity:
            capacityInformation                         = (usb_device_capacity_information_struct_t *)param;
            capacityInformation->lengthOfEachLba        = USB_Disk_GetBlockSize();
            capacityInformation->totalLbaNumberSupports = USB_Disk_GetBlockCount();
            break;
        case kUSB_DeviceMscEventStopEjectMedia:
            ufi = (usb_device_ufi_app_struct_t *)param;
            if (0x00U == (ufi->cbwcb[4] & 0x01U)) /* check start bit */
            {
                g_msc.stop = 1U; /* stop command */
            }
            break;
        default:
            error = kStatus_USB_InvalidRequest;
            break;
    }
    return error;
}
/*!
 * @brief device callback function.
 *
 * This function handle the usb standard event. more information, please refer to usb spec chapter 9.
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 * @return  A USB error code or kStatus_USB_Success..
 */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint16_t *temp16   = (uint16_t *)param;
    uint8_t *temp8     = (uint8_t *)param;
    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            g_msc.attach               = 0U;
            g_msc.currentConfiguration = 0U;
            g_msc.stop                 = 0U;
            error                      = kStatus_USB_Success;
			/* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
			if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &g_msc.speed))
			{
				USB_DeviceSetSpeed(handle, g_msc.speed);
			}
        }
        break;
        case kUSB_DeviceEventSetConfiguration:
            if (0U == (*temp8))
            {
                g_msc.attach               = 0;
                g_msc.currentConfiguration = 0U;
                error                      = kStatus_USB_Success;
            }
            else if (USB_MSC_CONFIGURE_INDEX == (*temp8))
            {
                g_msc.attach               = 1;
                g_msc.currentConfiguration = *temp8;
                error                      = kStatus_USB_Success;
            }
            else
            {
                /* no action, return kStatus_USB_InvalidRequest */
            }
            break;
        case kUSB_DeviceEventSetInterface:
            if (g_msc.attach)
            {
                uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if (interface < USB_MSC_INTERFACE_COUNT)
                {
                    if (alternateSetting < USB_MSC_INTERFACE_ALTERNATE_COUNT)
                    {
                        g_msc.currentInterfaceAlternateSetting[interface] = alternateSetting;
                        error                                             = kStatus_USB_Success;
                    }
                }
            }
            break;
        case kUSB_DeviceEventGetConfiguration:
            if (param)
            {
                *temp8 = g_msc.currentConfiguration;
                error  = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceEventGetInterface:
            if (param)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                if (interface < USB_INTERFACE_COUNT)
                {
                    *temp16 = (*temp16 & 0xFF00U) | g_msc.currentInterfaceAlternateSetting[interface];
                    error   = kStatus_USB_Success;
                }
            }
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param)
            {
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param)
            {
                error = USB_DeviceGetConfigurationDescriptor(handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetStringDescriptor:
            if (param)
            {
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
        default:
            break;
    }
    return error;
}


/* USB device class information */
usb_device_class_config_struct_t msc_config[1] = {{
    USB_DeviceMscCallback,
    0,
    &g_UsbDeviceMscConfig,
}};
/* USB device class configuration information */
usb_device_class_config_list_struct_t msc_config_list = {
    msc_config,
    USB_DeviceCallback,
    1,
};


void USB_DeviceApplicationInit(void)
{
    USB_DeviceClockInit();

//    PRINTF("Please insert disk \r\n");

    if (kStatus_USB_Success != USB_DeviceMscDiskStorageInit())
    {
        PRINTF("Disk init failed\r\n");
        return;
    }

    g_msc.speed        = USB_SPEED_FULL;
    g_msc.attach       = 0;
    g_msc.mscHandle    = (class_handle_t)NULL;
    g_msc.deviceHandle = NULL;
    if (kStatus_USB_Success != USB_DeviceClassInit(CONTROLLER_ID, &msc_config_list, &g_msc.deviceHandle))
    {
        PRINTF("USB device init failed\r\n");
    }
    else
    {
//        PRINTF("USB device mass storage demo\r\n");
        g_msc.mscHandle = msc_config_list.config->classHandle;
    }

    USB_DeviceIsrEnable();

    /*Add one delay here to make the DP pull down long enough to allow host to detect the previous disconnection.*/
    SDK_DelayAtLeastUs(5000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
//    USB_DeviceRun(g_msc.deviceHandle);
}

void USB_DeviceAppStart(void){
	USB_DeviceRun(g_msc.deviceHandle);
}
void USB_DeviceAppStop(void){
	USB_DeviceStop(g_msc.deviceHandle);
}

void USB_DeviceMscApp(void)
{
    /*TO DO*/
    /*add user code*/
    return;
}

void USB_DeviceMscAppTask(void)
{
    if (g_msc.read_write_error)
    {
        return;
    }
    USB_DeviceMscApp();
}

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    MUX_Init();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_USB_Disk_Config(USB_DEVICE_INTERRUPT_PRIORITY);
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
    PRINTF("Hello World, I'm photOS, the operating system for the DC-0x cameras.\r\n");
    MUX_ToUSBC();
    USB_DeviceApplicationInit();
    DISPLAY_Init();

    while(1) {

    	SHELL_WaitForInput();

    }
    return 0 ;
}
