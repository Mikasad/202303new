#ifndef _XD510_H
#define _XD510_H

#include <stdint.h>
#define ABS(a)  (((a)>0)?(a):-(a))

typedef struct
{
	uint8_t enable;                 //1:使能自适应控制
	uint8_t target_current;         //目标电流，默认2A，单位0.1A
	uint8_t last_current;           //上一次目标电流
	uint8_t new_cmd_flag;           //新命令标志
	uint16_t real_current;          //当前电流 
	uint8_t deviation;              //允许偏差，单位百分之1，最小百分之5
	uint16_t debounce_time;         //自适应电流去抖时间
	uint16_t min_current;           //自适应最小电流
	uint16_t max_current;           //自适应最小电流
	uint8_t  beyond_status ;        //自适应状态 0未超出  1小于阈值 2大于阈值
	uint8_t  response_status;       //反馈给上位机的状态 0抬起 12放下中 1放下

}Xd510_Self_Adaption_t;   //推杆自适应控制

extern Xd510_Self_Adaption_t Xd510_Self_Adaption;

void SetDustMotor(  uint32_t f_pwm );
void SetVacuumMotor( uint32_t f_enable, uint32_t f_pwm );
void SetSprayMotor( uint32_t f_enable, uint32_t f_pwm );
void SetDustClearMotor( uint32_t f_pwm );
void SetBrushLift( uint32_t f_cmd );
void SetSideBrushLift( uint32_t f_cmd );
void SetSqueegeeLift( uint32_t f_cmd );
void SetBrushDownLevel( uint32_t f_level );
void SetSqueegeeDownLevel( uint32_t f_level );
void SetVacuumCloseDelay( uint32_t f_time );
void SetRobotWorkMode( uint32_t f_mode );
void SetOutletSewageCmd( uint32_t f_cmd );
void SetHullLiftTravelTable(uint16_t f_low, uint16_t f_median, uint16_t f_high);
void SetSideLiftTravelTable(uint16_t f_low, uint16_t f_median, uint16_t f_high);
void SetVacuumMaxCurrent(uint32_t f_max);
void SetBrushLiftMaxCurrent(uint32_t f_max);
void SetSqueegeeLiftMaxCurrent(uint32_t f_max);
uint8_t GetDustMotorWorkStatus( void );
uint8_t GetVacuumMotorWorkStatus(void);
uint8_t GetSprayMotorWorkStatus(void);
uint8_t GetDustClearMotorWorkStatus(void);
uint16_t GetBrushLiftStatus(void);
uint16_t GetSideBrushLiftStatus(void);
uint8_t GetSqueegeeLiftStatus(void);
uint32_t GetXd510SysErrBits1(void);
uint32_t GetXd510SysErrBits2(void);
uint32_t GetXd510SysErrSelf(void);
int16_t GetXd510Temperature(void);
int16_t GetDustMotorCurrent(void);
int16_t GetVacuumMotorCurrent(void);
int16_t GetSprayMotorCurrent(void);
int16_t GetDustClearMotorCurrent(void);
int16_t GetBrushLiftMotorCurrent(void);
int16_t GetSqueegeeLiftMotorCurrent(void);
uint16_t GetDeviceMotorCurrentAlarm(void);
uint8_t GetValveStatus(void);
void GetHullTravelTable(uint16_t fp_travel[]);
void GetSideTravelTable(uint16_t fp_travel[]);
void GetXd510Version(uint32_t *fp_version );
void GetXd510motorPwm( int16_t f_pwm[] );
int GetXd510HealthStatus(void);
void GetXd510OutputPwm( uint8_t f_pwm[] );
int16_t GetBrushLiftHullCountReal(void);

void Xd510Task(void const *pvParameters);
int InitSoftwareXd510(void);
void ClearXd510Overcurrent(void);
void SetBrushLiftPosition( uint32_t f_position);



#endif
