#ifndef __CANOPEN_TIMER_H
#define __CANOPEN_TIMER_H
#include "stm32f4xx_hal.h"
#include "canopen_od.h"
#include "timerscfg.h"
extern uint8_t CAN1_Send_Msg(uint32_t stdId,uint32_t Ide,uint32_t Rtr,uint8_t* msg,uint8_t len);

/* MACRO DEFINITION END */

/* TYPEDEF STRUCT BEGIN */

/* 定时事件回调函数指针 */
typedef void (*TimerCallback_t)(CO_Data* d, unsigned int id);

struct struct_s_timer_entry {
	unsigned char state;        
	CO_Data* d;                 
	TimerCallback_t callback;   
	unsigned int id;            
	TIMEVAL val;                
	TIMEVAL interval;           
};
typedef struct struct_s_timer_entry s_timer_entry;
/* TYPEDEF STRUCT END */

//Var Definition
extern TIM_HandleTypeDef TIM5_Handler; 
//Function Definition

TIMER_HANDLE SetAlarm(CO_Data* d, unsigned int id, TimerCallback_t callback, TIMEVAL value, TIMEVAL Timecycle);
TIMER_HANDLE DelAlarm(TIMER_HANDLE handle);
void TimeDispatch(void);
void TimerForCan(void);
void setTimer(TIMEVAL value);
TIMEVAL getElapsedTime(void);
void TIM6_Init(void);

void Callbacktest(CO_Data* d, unsigned int id);
void Callbacktest1(CO_Data* d, unsigned int id);
void Callbacktest2(CO_Data* d, unsigned int id);

#endif
