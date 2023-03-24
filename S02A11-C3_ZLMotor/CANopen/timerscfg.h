#ifndef __TIMERSCFG_H__
#define __TIMERSCFG_H__

/* MACRO DEFINITION BEGIN */
// Whatever your microcontroller, the timer wont work if
// TIMEVAL is not at least on 32 bits
#define TIMEVAL unsigned int

// The timer of the STM32 counts from 0000 to 0xFFFF
#define TIMEVAL_MAX 0xFFFFFFFF

// The timer is incrementing every 100 us.
#define MS_TO_TIMEVAL(ms) ((ms) * 10)
#define US_TO_TIMEVAL(us) ((us) * 1) 

#define TIMER6_DIV            5
#define TIMER6_PERIOD         1399
#define TIM_IT                0x00000001

#define CANOPEN_TIMER_FREQ    14000000
#define CANOPEN_TIMER_PERIOD  0.0001

#define CANOPEN_TIMER_CNT_MAX 70000
#define MAX_NB_TIMER 15                    //最大可设定时间管理事件数  8

#define TIMER_HANDLE short
/* 定时事件入口状态 */
#define TIMER_FREE 0					/* 空闲，即该入口没有被定时事件占用 */
#define TIMER_ARMED 1					/* 被占用，即该入口已经定时事件占用 */
#define TIMER_TRIG 2					/* 单次触发 */
#define TIMER_TRIG_PERIOD 3		/* 周期触发 */

#define TIMER_NONE -1					/* 错误返回 */

/* 求最小值 */
#define min_val(a,b) ((a<b)?a:b)



#endif
