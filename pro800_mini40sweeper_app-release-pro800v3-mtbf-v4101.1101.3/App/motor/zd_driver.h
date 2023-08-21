#ifndef _ZD_DRIVER_H
#define _ZD_DRIVER_H

#include <stdint.h>



void ZdDriverReceive( uint8_t fp_buff[], uint16_t len );

void SetRollerMotor(uint32_t f_enable, uint32_t f_pwm );
void SetLeftBrushMotor(uint32_t f_enable, uint32_t f_pwm );
void SetRightBrushMotor(uint32_t f_enable, uint32_t f_pwm );
void SetRollerMaxCurrent(uint16_t f_max);
void ClearZdOverCurrent(void);

uint16_t GetRollerCurrent(void);
uint16_t GetRollerSpeed(void);
uint16_t GetLeftBrushCurrent(void);
uint16_t GetRightBrushCurrent(void);
uint8_t GetRollerConnectStatus(void);
uint8_t GetLeftBrushConnectStatus(void);
uint8_t GetRightBrushConnectStatus(void);
uint8_t GetRollerOvercurrentAlarm(void);
uint8_t GetSideBrushOvercurrentAlarm(void);
uint8_t GetRollerWorkStatus(void);
uint8_t GetLeftSideBrushWorkStatus(void);
uint8_t GetRightSideBrushWorkStatus(void);
uint8_t GetRollerDriverType(void);
uint32_t GetZdDriverAlarm(void);
void ZdDriverTask(void const *pvParameters);
void GetZdDebugData( uint16_t fp_data[] );

uint32_t GetLeftSideAlarm(void);
uint32_t GetRightSideAlarm(void);
void SetRollerMotorDir(uint32_t dir);
#endif
