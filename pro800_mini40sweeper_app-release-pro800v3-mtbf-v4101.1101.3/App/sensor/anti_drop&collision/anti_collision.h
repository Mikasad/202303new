#ifndef __ANTI_COLLISION_H_
#define __ANTI_COLLISION_H_

#include <stdint.h>
#include <main.h>

typedef union{
  float fData;
  uint8_t memData[4]; 
}floatMemory_t;

typedef struct{
  uint32_t cfgRate;                    //上位机配置斜率阈值,单位V/s
  uint32_t cfgPeriodTime_ms;           //上位机下发的采样间隔,单位ms
  uint32_t readCfgRate;                //读取到的配置斜率阈值
  uint32_t readCfgPeriodTime_ms;       //读取到的配置采样间隔
  uint32_t rate;                       //实际斜率  v/s
}antiCollisionInfo_t;

extern uint32_t g_all_rate[8];
extern uint8_t  g_all_rate_idx[8];

void HandleAntiCollisionData(void);
void AntiCollisionTask(void);
void receiveAntiCollisionData(uint32_t ExtId, uint8_t* Data);
void ConfigAntiCollision( uint16_t f_enable, uint32_t fp_period_time[], uint32_t fp_rate[], uint16_t f_voltage );
void ClearAntiCollisionTrigger(void);
uint32_t GetAntiCollisionDisconectStatus(void);
uint32_t GetAntiCollisionReat(uint8_t f_idx);
uint8_t GetAntiCollisionStop(void);
uint16_t GetAntiCollisionEnable(void);
uint8_t GetAntiCollisionTriggle(void);
void GetAntiCollisionData(antiCollisionInfo_t fp_data[] );
void GetGcubVersion(uint32_t *fp_version );


#endif

