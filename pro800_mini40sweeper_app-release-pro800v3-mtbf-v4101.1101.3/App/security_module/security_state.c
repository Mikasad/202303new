//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"
//#include "croutine.h"
//#include "semphr.h"
//#include "event_groups.h"

//APP
#include "security_state.h"

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 通过安全传感器判断禁止移动的方向,目前只有前后和原地打转
*	形    参: 
*	返 回 值:禁止移动的方向
*********************************************************************************************************
*/
uint32_t CheckForbidMoveDirBySafetySensor(safetySensorTriggerStatus_t f_safetySensorTriggerStatus)
{
	uint32_t ret = 0;
	if(f_safetySensorTriggerStatus.dir.front == 1 || f_safetySensorTriggerStatus.dir.leftFront == 1 || f_safetySensorTriggerStatus.dir.rightFront == 1)
		ret |= ~ROBOT_MOVE_DIR_BACK; //只能向后
	if(f_safetySensorTriggerStatus.dir.back == 1 || f_safetySensorTriggerStatus.dir.leftBack == 1 || f_safetySensorTriggerStatus.dir.rightBack == 1)
		ret |= ~ROBOT_MOVE_DIR_FRONT;//只能向前
	if(f_safetySensorTriggerStatus.dir.left == 1 || f_safetySensorTriggerStatus.dir.right == 1)
		ret |= ~(ROBOT_MOVE_DIR_SPOTTURN | ROBOT_MOVE_DIR_BACK | ROBOT_MOVE_DIR_FRONT);
	return ret;
}
/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 获取连接状态
*	形    参: 
*	返 回 值:1=未失联，0=失联
*********************************************************************************************************
*/
int GetLinkStatus(ConnectionStatus_t *fp_connStatus)
{
    if( fp_connStatus == NULL )
    {
        return 0;
    }
    if( xTaskGetTickCount() - fp_connStatus->last_connect_time >= pdMS_TO_TICKS(fp_connStatus->timeout_ms) )
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 重置与最后与设备连接的时间
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
void ResetConnectedTime(ConnectionStatus_t *fp_connStatus)
{
    if( fp_connStatus != NULL )
    {
        fp_connStatus->last_connect_time = xTaskGetTickCount();
    }
	
}







