#include "mutex_can_buf.h"
//#include "info_store.h"
#include <string.h>

/*
*********************************************************************************************************
*	专门用于CAN数据的发送
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 初始化发送结构，主要是创建互斥量
*	形    参:  
*	返 回 值: -1=错误；0=成功；1=失败
*********************************************************************************************************
*/
int InitMutexCanTxBuf(mutexCanTxBuf_t *fp_mutexCanTxBuf)
{
	
	mutexCanTxBuf_t *p_mutexCanTxBuf = fp_mutexCanTxBuf;

	p_mutexCanTxBuf->mutex = xSemaphoreCreateMutex();
	if(p_mutexCanTxBuf->mutex == NULL)
		return 1;
	else
		return 0;        
}

/*
*********************************************************************************************************
*	函 数 名: GetLenMutexBuf1
*	功能说明: 返回数据缓冲区有效数据的长度
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
uint16_t GetLenMutexCanTxBuf(mutexCanTxBuf_t *fp_mutexCanTxBuf)
{
    uint16_t len = 0;
	
	mutexCanTxBuf_t *p_mutexCanTxBuf = fp_mutexCanTxBuf;
	
    xSemaphoreTake(p_mutexCanTxBuf->mutex, portMAX_DELAY);
    len = p_mutexCanTxBuf->len;
    xSemaphoreGive(p_mutexCanTxBuf->mutex);
    
    return len;
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 
*	形    参: 
*	返 回 值:-1=错误，1=无数据或数据不足 ，0=获取到了数据
*********************************************************************************************************
*/
int ReadMutexCanTxBuf(mutexCanTxBuf_t *fp_mutexCanTxBuf,CanTxMsg_t *fp_canTxMsg)
{
    int ret = 0;
    uint16_t posOut = 0;
	mutexCanTxBuf_t *p_mutexCanTxBuf = fp_mutexCanTxBuf;
        
    xSemaphoreTake(p_mutexCanTxBuf->mutex, portMAX_DELAY);
    if(p_mutexCanTxBuf->len == 0)
    {
        ret = 1;
    }
    else
    {
        posOut = p_mutexCanTxBuf->posOut;
		*fp_canTxMsg = p_mutexCanTxBuf->canTxMsg[posOut];
        p_mutexCanTxBuf->len--;
        p_mutexCanTxBuf->posOut++;
        p_mutexCanTxBuf->posOut %= MUTEX_CAN_TX_BUF_SIZE;
        ret = 0;
    }
    xSemaphoreGive(p_mutexCanTxBuf->mutex);
    return ret;
}
/*
*********************************************************************************************************
*	函 数 名: WriteMutexBuf1
*	功能说明: 
*	形    参: 
*	返 回 值:-1=错误，1=剩余空间不足，0=写入成功
*********************************************************************************************************
*/
int WriteMutexCanTxBuf(mutexCanTxBuf_t *fp_mutexCanTxBuf,CanTxMsg_t f_canTxMsg)
{
    int ret = 0;
    uint16_t posIn = 0;
    mutexCanTxBuf_t *p_mutexCanTxBuf = fp_mutexCanTxBuf;
	
    xSemaphoreTake(p_mutexCanTxBuf->mutex, portMAX_DELAY);
    if(p_mutexCanTxBuf->len >= MUTEX_CAN_TX_BUF_SIZE)//剩余空间不足
    {
        ret = 1;
    }
    else
    {
        posIn = p_mutexCanTxBuf->posIn;
		p_mutexCanTxBuf->canTxMsg[posIn] = f_canTxMsg;

        p_mutexCanTxBuf->len++;
        p_mutexCanTxBuf->posIn++;
        p_mutexCanTxBuf->posIn %= MUTEX_CAN_TX_BUF_SIZE;
        ret = 0;
    }
	xSemaphoreGive(p_mutexCanTxBuf->mutex);

    return ret;
}
void ClearMutexCanTxBuf(mutexCanTxBuf_t *fp_mutexCanTxBuf)
{
	mutexCanTxBuf_t *p_mutexCanTxBuf = fp_mutexCanTxBuf;

	xSemaphoreTake(p_mutexCanTxBuf->mutex, portMAX_DELAY);
    p_mutexCanTxBuf->len = 0;
    p_mutexCanTxBuf->posIn = 0;
    p_mutexCanTxBuf->posOut = 0;
	xSemaphoreGive(p_mutexCanTxBuf->mutex);

}

