#ifndef __RFID_H_
#define __RFID_H_
//标准头文件
#include <stdint.h>
//stm32头文件
#include "stm32f4xx.h"
//freertos头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"
//APP头文件
#include "security_state.h"
#include "version.h"

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/



/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/



/************************************************************************************************************************************
																变量extern声明
*************************************************************************************************************************************/
extern TaskHandle_t taskHandleRfid;


/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口声明
*************************************************************************************************************************************/
void GetRfidData(uint8_t *fp_id,uint8_t *fp_rssi);
uint8_t GetRfidHealthState(void);
void WriteRfidPowerCfgPara(uint32_t f_rfidPower);


/************************************************************************************************************************************
											对外提供的任务、硬件初始化、软件初始化声明
*************************************************************************************************************************************/
void RfidTask(void *pvParameters);
//0=成功
int InitHardwareRfid(void);
int InitSoftwareRfid(void);

#endif
