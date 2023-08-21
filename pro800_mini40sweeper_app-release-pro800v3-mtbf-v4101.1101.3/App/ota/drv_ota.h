#ifndef __APP_MAIN_MCU_OTA_H_
#define __APP_MAIN_MCU_OTA_H_
//标准头文件
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "main.h"
#include "version.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "service.pb.h"

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/
#define OTA_DBG             0
#define OTA_PACKAGE_SIZE    1024

#if OTA_DBG
#define OTA_PRT         printf
#else
#define OTA_PRT
#endif

/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/
typedef enum
{
	MAIN_MCU,
	SUB_MCU,
	IMU,
	UNKNOWN_MCU
}OTA_TARGET;

typedef enum
{
	OTA_CMD_START,
	OTA_CMD_FINISH,
	OTA_CMD_ROLLBACK,
	OTA_CMD_REBOOT,
	OTA_CMD_NONE
}OTA_CMD;

typedef enum
{
	MCU_4TH,
	MCU_6TH,
	MCU_PRO800,
	MCU_LINKS,
	MCU_S_V2 = 5,
	MCU_S_V3 = 6,
	MCU_S_V4 = 7,
}MCU_TYPE;

typedef enum
{
	MCU_GD,
	MCU_UNKNOWN
}SUB_MCU_TYPE;

typedef enum
{
	JC310,
	IMU_UNKNOWN
}IMU_TYPE;

typedef enum
{
	OTA_FAILED,
	OTA_SUCCEED
}OTA_STATUS;

typedef struct
{
	OTA_TARGET target;
	OTA_CMD    cmd;
	OTA_STATUS status;
	uint8_t    backupNum;
	uint32_t   total_num;
	uint32_t   total_size;
}OTA_INFO;

/************************************************************************************************************************************
																变量extern声明
*************************************************************************************************************************************/


/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口声明
*************************************************************************************************************************************/



/************************************************************************************************************************************
											对外提供的任务、硬件初始化、软件初始化声明
*************************************************************************************************************************************/
uint32_t get_phantas_mcu_upgrade_command_status(OTA_TARGET target);
void set_phantas_mcu_upgrade_command_status(OTA_TARGET target, OTA_STATUS status);
uint32_t get_phantas_mcu_upgrade_package_status(OTA_TARGET target);
void set_phantas_mcu_upgrade_package_status(OTA_TARGET target, OTA_STATUS status);
bool phantas_mcu_upgrade_package_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
bool mcu_types_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
bool canbus_update_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);

#endif
