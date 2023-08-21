#ifndef __MOTOR_DRIVER_H_
#define __MOTOR_DRIVER_H_
//标准头文件
#include <stdint.h>
//stm32头文件
//freertos头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"
//APP头文件
#include "ctr_move_task.h"
#include "hc32_ll.h"


/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/
//#define MOTOR_DRIVER_TYPE_BLDC
#define USE_HOLLYSYS_MOTOR

/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/



/************************************************************************************************************************************
																变量extern声明
*************************************************************************************************************************************/
extern TaskHandle_t taskHandleMotorDriver;

/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口声明
*************************************************************************************************************************************/
void WriteSpeedVal(int32_t f_leftSpeed,int32_t f_rightSpeed);
void GetSpeedValToMotorDriver(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed);
void CfgDriverBrk(brkStatus_t f_brkStatus);
void CfgDriverEnable(en_functional_state_t f_newState);
en_functional_state_t GetLeftDriverAlarmState(void);
en_functional_state_t GetRightDriverAlarmState(void);
void GetLeftDecoderInfo(int32_t *fp_cnt, int32_t *fp_relativeCnt);
void GetRightDecoderInfo(int32_t *fp_cnt, int32_t *fp_relativeCnt);
uint8_t GetMotorDriverHealthState(void);
uint8_t GetLeftMotorHealthState(void);
uint8_t GetRightMotorHealthState(void);
uint8_t GetMotorBrakeDownState(void);
void ReadMotorStatus( uint8_t* fp_enable, int32_t speed[], int32_t count[], int32_t dcount[], uint16_t fp_max_cur[]);
#ifdef MOTOR_DRIVER_TYPE_DA_NENG
int DaNengDriver_CAN_IRQHandler(CanRxMsg f_canRxMes);
#endif

#ifdef USE_HOLLYSYS_MOTOR
void ClearHollySysMotorAlarm(void);
uint32_t GetLeftMotorErrorCode(void);
uint32_t GetRightMotorErrorCode(void);
void SetHollySysDriverType( uint8_t f_type );
void GetHollySysDriverType( uint32_t fp_type[] );
void SetHollySysPidIncharge(uint32_t f_p, uint32_t f_i );
void SetHollySysMotorTemperature(uint32_t f_left, uint32_t f_right );
void SetHollySysMotorCurrent(uint32_t f_left, uint32_t f_right );
void SetHollySysControlMode(uint8_t f_mode );
#endif
uint16_t GetLeftMotorLoadFactor(void);
uint16_t GetRightMotorLoadFactor(void);
int16_t GetLeftMotorCurrent(void);
int16_t GetRightMotorCurrent(void);
int16_t GetLeftMotorTemperature(void);
int16_t GetRightMotorTemperature(void);
uint8_t GetLRTemperatureAlarm(void);
uint8_t GetLRCurrentAlarm(void);
int GetCarMoveStatus(void);//非零表示在运动
uint8_t GetHollySysControlMode(void);
void SetMotorMaxCurrentInCharge( uint32_t f_current );
/************************************************************************************************************************************
											对外提供的任务、硬件初始化、软件初始化声明
*************************************************************************************************************************************/
void MotorDriverTask(void const *pvParameters);

int InitHardwareMotorDriver(void);
int InitSoftwareMotorDriver(void);

int InitHardwareDecoder(void);
int InitSoftwareDecoder(void);




#endif
