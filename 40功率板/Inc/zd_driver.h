#ifndef _ZD_DRIVER_H
#define _ZD_DRIVER_H

#include "stm32f4xx_hal.h"
extern uint8_t RXC_states;
void BmSetPwm(uint8_t f_addr, int16_t speed,int16_t Acce);
void BmState_Inquire(uint8_t id);
void BmMode_Set(uint8_t id,uint8_t mode);
void BmID_Set(uint8_t id);
void BmID_Inquire(void);
void Bm_state( uint8_t fp_buff[], uint16_t len );

void B86SetPwm(uint8_t f_addr, int16_t speed );
void B86_state( uint8_t fp_buff[], uint16_t len );

void ZdDriverReceive( uint8_t fp_buff[], uint16_t len );

void SetRollerMotorFront( uint32_t f_pwm );
void SetRollerMotorRear( uint32_t f_pwm );
void SetLeftBrushMotor(uint32_t f_enable, uint32_t f_pwm );
void SetRightBrushMotor(uint32_t f_enable, uint32_t f_pwm );
void SetRollerMotorDir(uint32_t dir);
void SetRollerMaxCurrentFront(uint16_t f_max);
void SetRollerMaxCurrentRear(uint16_t f_max);
void ClearZdOverCurrent(void);

uint16_t GetFrontRollerCurrent(void);
uint16_t GetRearRollerCurrent(void);
uint16_t GetLeftBrushCurrent(void);
uint16_t GetRightBrushCurrent(void);
uint8_t GetFrontRollerPWM(void);
uint8_t GetRearRollerPWM(void);
uint8_t GetFrontRollerOvercurrentAlarm(void);
uint8_t GetRearRollerOvercurrentAlarm(void);
uint8_t GetSideBrushOvercurrentAlarm(void);
uint8_t GetFrontRollerConnectStatus(void);
uint8_t GetRearRollerConnectStatus(void);
uint8_t GetLeftBrushConnectStatus(void);
uint8_t GetRightBrushConnectStatus(void);
uint8_t GetRollerDriverType(void);
uint32_t GetFrontRollerAlarm(void);
uint32_t GetRearRollerAlarm(void);
uint32_t GetLeftSideAlarm(void);
uint32_t GetRightSideAlarm(void);

void SetZdMcuerrorcode(uint16_t f_code, uint8_t f_flag);

void ZdDriverTask(void const *pvParameters);
void GetZdDebugData( uint16_t fp_data[] );


#endif
