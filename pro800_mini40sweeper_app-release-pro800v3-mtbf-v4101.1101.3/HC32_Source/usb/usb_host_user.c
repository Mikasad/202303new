/**
 *******************************************************************************
 * @file  usb/usb_host_msc/source/usb_host_user.c
 * @brief user application layer.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdio.h>
#include "usb_host_user.h"
//#include "usb_host.h"
#ifdef USB_MSC_FAT_VALID
#include "ff.h"       /* FATFS */
#endif
#include "usb_host_msc_class.h"
#include "usb_host_msc_scsi.h"
#include "usb_host_msc_bot.h"
#include "usb_host_driver.h"
 
/**
 * @addtogroup HC32F4A0_DDL_Applications
 * @{
 */

/**
 * @addtogroup USB_Host_Msc
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define BUTTON_PORT                 (GPIO_PORT_A)
#define BUTTON_PIN                  (GPIO_PIN_00)

#define GET_BUTTON_KEY()            (GPIO_ReadInputPins(BUTTON_PORT, BUTTON_PIN))

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
/* USBH_USR_Private_Macros */
extern usb_core_instance usb_app_instance;

/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */
usb_host_user_callback_func USR_cb = {
    &host_user_init,
    &host_user_denint,
    &host_user_devattached,
    &host_user_devreset,
    &host_user_devdisconn,
    &host_user_overcurrent,
    &host_user_devspddetected,
    &host_user_devdescavailable,
    &host_user_devaddrdistributed,
    &host_user_cfgdescavailable,
    &host_user_mfcstring,
    &host_user_productstring,
    &host_user_serialnum,
    &host_user_enumcompl,
    &host_user_userinput,
    &host_user_msc_app,
    &host_user_devunsupported,
    &host_user_unrecoverederror

};

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
#ifdef USB_MSC_FAT_VALID
static uint8_t Explore_Disk(const char *path, uint8_t recu_level);
static void Toggle_Leds(void);
#endif

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint8_t USB_HOST_USER_AppState = USH_USR_FS_INIT;
#ifdef USB_MSC_FAT_VALID
static FATFS fatfs;
static FIL file;
static uint8_t line_idx = 0U;
#endif



/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @brief  Displays the message on terminal for host lib initialization
 * @param  None
 * @retval None
 */
void host_user_init(void)
{
    static uint8_t startup = 0U;

    if (startup == 0U) {
        startup = 1U;
#if (LL_PRINT_ENABLE == DDL_ON)
        DDL_Printf("> USB Host library started.\r\n");
        DDL_Printf("     USB Host Library v2.1.0\r\n");
#endif
    }
}

/**
 * @brief  Displays the message on terminal via DDL_Printf
 * @param  None
 * @retval None
 */
void host_user_devattached(void)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    DDL_Printf(MSG_DEV_ATTACHED);
#endif
}

/**
 * @brief  host_user_unrecoverederror
 * @param  None
 * @retval None
 */
void host_user_unrecoverederror(void)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    DDL_Printf(MSG_UNREC_ERROR);
#endif
}

/**
 * @brief  Device disconnect event
 * @param  None
 * @retval None
 */
//void host_user_devdisconn(void)
//{
//#if (LL_PRINT_ENABLE == DDL_ON)
//    DDL_Printf(MSG_DEV_DISCONNECTED);
//#endif
//}

/**
 * @brief  USBH_USR_ResetUSBDevice
 * @param  None
 * @retval None
 */
void host_user_devreset(void)
{
    /* callback for USB-Reset */
}

/**
 * @brief  host_user_devspddetected
 * @param  [in] DeviceSpeed      USB speed
 * @retval None
 */
void host_user_devspddetected(uint8_t DeviceSpeed)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    if (DeviceSpeed == PRTSPD_FULL_SPEED) {
        DDL_Printf(MSG_DEV_FULLSPEED);
    } else if (DeviceSpeed == PRTSPD_LOW_SPEED) {
        DDL_Printf(MSG_DEV_LOWSPEED);
    } else {
        DDL_Printf(MSG_DEV_ERROR);
    }
#endif
}

/**
 * @brief  host_user_devdescavailable
 * @param  [in] DeviceDesc       device descriptor
 * @retval None
 */
void host_user_devdescavailable(void *DeviceDesc)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    usb_host_devdesc_typedef *hs;
    hs = DeviceDesc;
    DDL_Printf("VID : %04lXh\r\n", (uint32_t)(*hs).idVendor);
    DDL_Printf("PID : %04lXh\r\n", (uint32_t)(*hs).idProduct);
#endif
}

/**
 * @brief  host_user_devaddrdistributed
 * @param  None
 * @retval None
 */
void host_user_devaddrdistributed(void)
{
}

/**
 * @brief  host_user_cfgdescavailable
 * @param  [in] cfgDesc          Configuration desctriptor
 * @param  [in] itfDesc          Interface desctriptor
 * @param  [in] epDesc           Endpoint desctriptor
 * @retval None
 */
void host_user_cfgdescavailable(usb_host_cfgdesc_typedef *cfgDesc,
                                usb_host_itfdesc_typedef *itfDesc,
                                USB_HOST_EPDesc_TypeDef *epDesc)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    usb_host_itfdesc_typedef *id;

    id = itfDesc;
    if ((*id).bInterfaceClass  == 0x08U) {
        DDL_Printf(MSG_MSC_CLASS);
    } else if ((*id).bInterfaceClass  == 0x03U) {
        DDL_Printf(MSG_HID_CLASS);
    } else {
        ;
    }
#endif
}

/**
 * @brief  Displays the message on terminal for Manufacturer String
 * @param  [in] ManufacturerString
 * @retval None
 */
void host_user_mfcstring(void *ManufacturerString)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    DDL_Printf("Manufacturer : %s\r\n", (char *)ManufacturerString);
#endif
}

/**
 * @brief  Displays the message on terminal for product String
 * @param  [in] ProductString
 * @retval None
 */
void host_user_productstring(void *ProductString)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    DDL_Printf("Product : %s\r\n", (char *)ProductString);
#endif
}

/**
 * @brief  Displays the message on terminal for SerialNum_String
 * @param  [in] SerialNumString
 * @retval None
 */
void host_user_serialnum(void *SerialNumString)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    DDL_Printf("Serial Number : %s\r\n", (char *)SerialNumString);
#endif
}

/**
 * @brief  User response request is displayed to ask application jump to class
 * @param  None
 * @retval None
 */
void host_user_enumcompl(void)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    /* Enumeration complete */
    DDL_Printf(MSG_DEV_ENUMERATED);
    DDL_Printf("To see the root content of the disk : \r\n");
 #endif
}

/**
 * @brief  Device is not supported
 * @param  None
 * @retval None
 */
void host_user_devunsupported(void)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    DDL_Printf("> Device not supported.\r\n");
#endif
}

/**
 * @brief  User Action for application state entry
 * @param  None
 * @retval HOST_USER_STATUS     User response for key button
 */
HOST_USER_STATUS host_user_userinput(void)
{
    HOST_USER_STATUS usbh_usr_status;

    usbh_usr_status = USER_NONE_RESP;
    /*Key B3 is in polling mode to detect user action */
//    if (GET_BUTTON_KEY() == PIN_RESET) {
//        usbh_usr_status = USER_HAVE_RESP;
//    }
	DDL_Printf("> host_user_userinput.\r\n");
    return usbh_usr_status;
}

/**
 * @brief  Over Current Detected on VBUS
 * @param  None
 * @retval None
 */
void host_user_overcurrent(void)
{
#if (LL_PRINT_ENABLE == DDL_ON)
    DDL_Printf("Overcurrent detected.\r\n");
#endif
}

/**
 * @brief  Demo application for mass storage
 * @param  None
 * @retval None
 */


#ifdef USB_MSC_FAT_VALID
/**
 * @brief  Displays disk content
 * @param  [in] path         pointer to root path
 * @param  [in] recu_level
 * @retval uint8_t
 */
static uint8_t Explore_Disk(const char *path, uint8_t recu_level)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    char *fn;
    char tmp[14];
    uint32_t u32DevConnectTmp;

    res = f_opendir(&dir, path);
    if (res == FR_OK) {
        while ((0UL != host_driver_ifdevconnected(&usb_app_instance))) {
            res = f_readdir(&dir, &fno);
            if ((res != FR_OK) || (fno.fname[0] == (char)0)) {
                break;
            }
            if (fno.fname[0] == '.') {
                continue;
            }

            fn = fno.fname;
            (void)strcpy(tmp, fn);

            line_idx++;
            if (line_idx > 9U) {
                line_idx = 0U;
#if (LL_PRINT_ENABLE == DDL_ON)
                DDL_Printf("Press USER KEY to continue...\r\n");
#endif

                /*Key B3 in polling*/
                u32DevConnectTmp = host_driver_ifdevconnected(&usb_app_instance);
                while ((GET_BUTTON_KEY() != PIN_RESET) && (0UL != u32DevConnectTmp)) {
                    Toggle_Leds();
                }
            }
#if (LL_PRINT_ENABLE == DDL_ON)
            if (recu_level == 1U) {
                DDL_Printf("   |__");
            }
            /* else if(recu_level == 2U) */
            else {
                DDL_Printf("   |   |__");
            }
            if ((fno.fattrib & AM_DIR) == AM_DIR) {
                (void)strcat(tmp, "\r\n");
                DDL_Printf((void *)tmp);
            } else {
                (void)strcat(tmp, "\r\n");
                DDL_Printf((void *)tmp);
            }
#endif
            if (((fno.fattrib & AM_DIR) == AM_DIR) && (recu_level == 1U)) {
                (void)Explore_Disk(fn, 2U);
            }
        }
    }
    return (uint8_t)res;
}

/**
 * @brief  Toggle leds to shows user input state
 * @param  None
 * @retval None
 */
static void Toggle_Leds(void)
{
	#if 0
    static uint32_t i;
    if (i++ == 0x10000U) {
        BSP_LED_Toggle(LED_BLUE);
        BSP_LED_Toggle(LED_YELLOW);
        i = 0U;
    }
	#endif
}
#endif

/**
 * @brief  Deint User state and associated variables
 * @param  None
 * @retval None
 */
void host_user_denint(void)
{
    USB_HOST_USER_AppState = USH_USR_FS_INIT;
}

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
