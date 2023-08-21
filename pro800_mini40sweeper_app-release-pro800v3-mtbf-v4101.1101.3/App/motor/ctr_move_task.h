#ifndef __CTR_MOVE_TASK_H_
#define __CTR_MOVE_TASK_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"

#define MODE_SPEED  0
#define MODE_TORQUE 1

typedef enum 
{
    CTR_MODE_TROLLEY=0,//手推
	CTR_MODE_AUTO,//自动
}controlMode_t;
typedef enum 
{
	BRK_STATUS_NO_BRK,//不刹车
	BRK_STATUS_ALL_BRK,
	BRK_STATUS_NO_OPS,//不动作
	BRK_STATUS_LEFT_BRK,
	BRK_STATUS_RIGHT_BRK,
}brkStatus_t;

uint8_t GetTrolleyModeStatus(void);

void CtrMoveTask(void const *pvParameters);
void SetRobotBrkInAutoMode(brkStatus_t f_brkStatus);
void SetRobotBrkInCmdLineMode(brkStatus_t f_brkStatus);
void SetRobotTrolleyMode( uint8_t f_mode );
controlMode_t GetCurCtrMode(void);

#endif
