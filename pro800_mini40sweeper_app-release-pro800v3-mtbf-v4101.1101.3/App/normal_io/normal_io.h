#ifndef __NORMAL_IO_H_
#define __NORMAL_IO_H_
//标准头文件

//stm32头文件
#include "hc_gpio.h"
//freertos头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"
//APP头文件


/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/



/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/


typedef struct{
	uint8_t state;
	uint8_t reverse;//实际状态与管脚高低是否相反
	uint16_t gpioPin;
	uint32_t gpio;
	uint32_t startTime;//用来消抖
	uint32_t holdTime;//用来消抖
}keySt_t;

typedef struct{
	keySt_t din[4];
	keySt_t in[16];
	keySt_t alm[2];
	keySt_t hardVer[2];
	keySt_t mcuKey;
	keySt_t stop;
}allKeysSt_t;


/************************************************************************************************************************************
																变量extern声明
*************************************************************************************************************************************/
extern TaskHandle_t taskHandleNormalIo;
extern allKeysSt_t m_allKeysSt;
/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口声明
*************************************************************************************************************************************/

void SetEmergency(uint8_t f_cmd);
void ClearEmergencyStop(void);
void ConfigEmergencyParam(uint8_t f_type, uint32_t f_time );
uint8_t GetEmergencyButtonStatus(void);

uint8_t GetEmergencyStopState(void);

uint32_t CheckForbidMoveDirByEmergencyStop(void);

uint8_t GetTrolleyModeStatusByButton(void);

uint32_t CheckForbidMoveDirByKey(void);//钥匙开关对运动方向的限制

uint8_t GetKeyState(void);

uint16_t GetNormalIoInputInfo(void);

uint8_t GetExInputStatus(void);

uint16_t GetChargeDetectionInputInfo(void);

//获取清水箱污水箱浮标状态
uint8_t GetBuoyStatus(void);
//获取清水箱水位
uint8_t GetCleanWaterLevel(void);
//获取污水箱水位
uint8_t GetSewageLevel(void);

uint8_t GetDin1Sum(void);

//HEPA网及袋未安装告警
uint8_t GetHepaDustBagAlarm(void);

uint8_t GetEmergencyType(void);

uint32_t GetEmergencyMinTime(void);

uint8_t GetDisinfectLevel(void);

uint8_t GetSideDoorOpenStatus(void);

/************************************************************************************************************************************
											对外提供的任务、硬件初始化、软件初始化声明
*************************************************************************************************************************************/
void NormalIoTask(void const *pvParameters);


#endif
