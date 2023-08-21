/*
 * @Author: Jack Yi
 * @Date: 2022-09-05 16:58:24
 * @LastEditors: Jack Yi
 * @LastEditTime: 2022-11-17 18:34:23
 * @FilePath: \gssa-s2.0-app\User_Code\ota\app_ota.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Jack Yi, All Rights Reserved. 
 */
#ifndef _APP_OTA_H
#define _APP_OTA_H

#include <stdint.h>
#include <stdbool.h>
//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"

#include "pb.h"
#include "drv_ota.h"


#define OTA_TASK

typedef enum
{
	APP_OTA_TASK_SUSPEND = 1,
    APP_OTA_TASK_RESUME,
    APP_OTA_TASK_DEFAULT,
}app_ota_manage_task_t;


extern uint8_t app_ota_type;

void set_recv_can_cmd_tick(uint32_t tick);
uint32_t get_recv_can_cmd_tick(void);
void set_ota_process_flag(bool flag);
bool get_ota_process_flag(void);
void set_ota_current_msg_id(uint32_t msg_id);
uint32_t get_ota_current_msg_id(void);
void copy_ota_package(pb_byte_t *buf, uint32_t len);
void post_to_ota_task(uint32_t msg_id, uint8_t* buf, uint16_t* msg_len, uint16_t buf_len);
void post_canbus_to_ota_task(uint32_t msg_id);

void app_ota_task(void *pvParameters);
void app_ota_msg_check(void *pvParameters);

#endif

