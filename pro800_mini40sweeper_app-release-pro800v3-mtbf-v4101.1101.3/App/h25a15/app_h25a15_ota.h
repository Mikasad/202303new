/*
 * @Author: Jack Yi
 * @Date: 2023-07-01 15:28:40
 * @LastEditors: Jack Yi
 * @LastEditTime: 2023-07-01 15:31:01
 * @FilePath: \pro800_mini40sweeper_app\App\h25a15\app_h25a15_ota.h
 * @Description: 
 * 
 * Copyright (c) 2023 by Jack Yi, All Rights Reserved. 
 */
#ifndef _APP_H25A15_OTA_H_
#define _APP_H25A15_OTA_H_


#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "h25a15/drv_h25a15.h"
#include "drv_zlac8015.h"
#define CAN_OTA_DATA_LEN    8

#define CAN_OTA_ORDER		0x10
#define CAN_OTA_ORDER_ACK	0x20
#define CAN_OTA_DATA		0x11
#define CAN_OTA_CRC_DATA	0x12
#define CAN_OTA_FW_DATA		0x13
#define CAN_OTA_ERASE_DATA	0x14
#define CAN_OTA_DATA_ACK	0x21
#define CAN_OTA_ERROR_ACK	0xFF
#define CAN_OTA_END_ACK		(0x700+H25A_ID)
#define CAN_OTA_VER_ACK   (0x580+H25A_ID)
#define CAN_OTA_END_WHEEL_ACK	(0x700+5)
#define CAN_OTA_VER_WHEEL_ACK   (0x580+5)

#define CAN_OTA_END_WHEEL_ACK  (0x700+ZLAC_ID)
#define CAN_OTA_VER_WHEEL_ACK   (0x580+ZLAC_ID)

#define CAN_OTA_END_BMS_ACK	(0x700+4)
#define CAN_OTA_VER_BMS_ACK    (0x580+4)
/* �?�?件版�?号index�?0x3000，sub_index：软�?0x02，硬件：0x03 */
#define CAN_OTA_SW_VER_IDX   0x02300000
#define CAN_OTA_HW_VER_IDX   0x03300000

/* 驱动器提供的OTA所有指令返回的最大超时时间为7s */
#define CAN_OTA_TIMEOUT_MS  7000

void H25A15_Ota_Can_Write(uint32_t ID,uint8_t *Data,uint16_t Len);
void H25A15_Ota_Can_Read(uint32_t ID,uint8_t *pData,uint16_t Len);

#endif


