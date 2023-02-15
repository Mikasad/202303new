#ifndef XDS_BLVACCUM_H
#define XDS_BLVACCUM_H

#include "stm32f4xx_hal.h"



void HaldleXdsData(uint8_t fp_data[], uint16_t f_len);

void XdsTask(void);

void SetXdsVacuumPwm(uint8_t f_enable, uint32_t f_pwm );

uint16_t GetXdsVacuumCur(void);

uint8_t GetXdsVacuumSt(void);

uint8_t GetXdsVacuumHealth(void);

uint8_t GetXdsVacuumOC(void);

void ClearXdsVacuumOC(void);

uint32_t GetXdsVacuumVersion(void);

uint32_t GetXdsAlarm(void);

void GetXdsDebugData(uint16_t fp_data[]);

void XdsDebug(uint8_t f_cmd);




#endif
