#ifndef __PC_CORRESPOND_H_
#define __PC_CORRESPOND_H_


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"

#include "security_state.h"
#include "bms.h"

extern TaskHandle_t taskHandlePcCorrespond;
extern BatteryMsg_t g_battery_data;

int InitSoftwarePcCorrespond(void);
void PcCorrespondTask(void const *pvParameters);
void GetPcSpeed(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed);
void SetPcSpeed(int32_t f_leftSpeed,int32_t f_rightSpeed);
int GetPcConnectStatus(void);
void ClearPcDisconnect(void);
void ClearSpeedTimeout(void);

int GetPcConnectStatus(void);

#endif
