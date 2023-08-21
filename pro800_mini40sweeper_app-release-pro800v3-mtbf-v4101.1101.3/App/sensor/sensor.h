#ifndef __SENSOR_H_
#define __SENSOR_H_

//标准头文件

//stm32头文件
//FreeRTOS
//APP头文件
#include "security_state.h"
#include "version.h"


/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/



/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/

typedef struct{
	uint32_t front;
	uint32_t back;
	uint32_t left;
	uint32_t right;
	uint32_t leftFront;
	uint32_t rightFront;
	uint32_t leftBack;
	uint32_t rightBack;
}sensorDirCfg_t;//上位机配置，一个方向可能会有多个位，表示多个口都代表着这个方向





/************************************************************************************************************************************
																变量extern声明
*************************************************************************************************************************************/


/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口声明
*************************************************************************************************************************************/

//各种weak函数在这里统一声明
uint32_t CheckForbidMoveDirByCollision(void);
uint32_t CheckForbidMoveDirByDrop(void);
void GetUltrasonicVersion(version_t *fp_version);
void GetAntidropVersion(version_t *fp_version);
void GetAntiCollisionVersion(version_t *fp_version);
uint8_t GetTelecontrolRawData(void);
uint8_t GetUltrasonicHealthState(void);
void GetUltrasonicDistance(uint32_t * fp_dest);
void GetUltrasonicCfgPara(sensorDirCfg_t f_ultrasonicDirCfg, uint32_t f_enableCfg);
void GetAntiDropCfgPara(sensorDirCfg_t f_sensorDirCfg,uint32_t *fp_threshold);
void GetAntiCollisionCfgPara(sensorDirCfg_t f_sensorDirCfg,uint32_t *fp_threshold);
void KS136_IRQHandler(void);
/************************************************************************************************************************************
											对外提供的任务、硬件初始化、软件初始化声明
*************************************************************************************************************************************/




#endif

