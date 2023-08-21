#ifndef _DISINFECT_BAG_H
#define _DISINFECT_BAG_H

#include "main.h"
#include "can_task.h"



void HandleDisinfectData(CanRxMsg_t* fp_rx_msg);
void SetDisinfectMistSpray(uint8_t f_cmd);
void SetDisinfectSprayDistance( uint8_t f_cmd);
void SetDisinfectBoxBeep( uint8_t f_cmd );
void SetDisinfectBoxPump( uint8_t f_cmd );
void SetDisinfect2_0Work(uint8_t f_cmd);
void ConfigDisinfectEnable( uint8_t f_cmd );
uint8_t GetDisinfectMistSprayLevel(void);
uint8_t GetDisinfectSprayDistance(void);
uint8_t GetDisinfectDisinfectantLevel(void);
uint8_t GetDisinfectBeepStatus(void);
uint8_t GetDisinfectPumpStatus(void);
uint16_t GetDisinfectTemperature(void);
uint8_t GetDisinfectDisconnectStatus(void);
void GetDisinfectDebugData(uint8_t* fp_data);
void DisinfectDebug(uint8_t f_cmd);

void DisinfectTask(void const *pvParameters);




#endif
