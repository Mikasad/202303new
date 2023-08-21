#include "can_task.h"
#include "sensor_hub.h"
#include "motor_driver.h"
#include "objdict.h"
#include "main.h"
#include "anti_drop_collision.h"
#include "hc_can.h"

/*
*********************************************************************************************************
*
*    模块名称 : 发送Can数据
*    文件名称 : can_task.c
*    版    本 :     V1.0.2
*    说    明 :
*            1，因CAN总线的特性，可能会有多个模块都是用同一路CAN，在多任务中就会导致CAN数据的发送出现混乱，
因此在使用CAN功能发送数据时要注意在各个任务中仅仅是写入CANbuf中，然后在这里进行数据发送
*            2,接收中断也在这里处理
*    修改记录 :
*        版本号          日期        作者     说明
*        V1.0.1    2018-11-14  杨臣       正式发布
*        V1.0.2    2018-11-26  杨臣       添加了can发送错误码的设置和清除
*        V1.0.3    2018-11-30  杨臣       CAN发送充分利用三个邮箱，使能发送优先级，即先请求先发送
*       V1.1.0
*    外设资源占用：
*        GPIO：
*            PD0,PD1,PB6,PB12
*        CAN：
*            CAN1,CAN_FIFO0
*            CAN2,CAN_FIFO1
*        NVIC：
*            CAN1_RX0_IRQn，优先级（2，0）
*            CAN2_RX1_IRQn，优先级（6，0）
*    待优化：（需要优化但是未优化之前请保留该提示）
*
*********************************************************************************************************
*/

mutexCanTxBuf_t can1TxBuf;
mutexCanTxBuf_t can2TxBuf;

static void HandleCan1Tx(void);
static void HandleCan2Tx(void);

/**
 * @brief 将被放入发送队列的数据顺序读取出来进行发送
 */
static void HandleCan1Tx(void)
{
    static CanTxMsg_t s_canTxMsg = {0};
    uint8_t status = LL_OK;
    static uint8_t s_needSendOldFlag = 0;
    TickType_t stayWorkingTime = xTaskGetTickCount(); // 在死循环中保持的时间不能太长
    const TickType_t c_statyTimeOut = pdMS_TO_TICKS(2);

    while (1)
    {
        if (s_needSendOldFlag == 1) // 上一次读取了数据，但是没有空邮箱可以发送
        {
            status = CAN_FillTxFrame(CAN1_UNIT, CAN_TX_BUF_PTB, &s_canTxMsg.head);
            CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_PTB);
            CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_STB_ALL);
        }
        else if (ReadCan1TxBuf(&s_canTxMsg) == 0) // 没有老数据，则读取新数据,读到了数据就发送
        {
            status = CAN_FillTxFrame(CAN1_UNIT, CAN_TX_BUF_PTB, &s_canTxMsg.head);
            CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_PTB);
            CAN_StartTx(CAN1_UNIT, CAN_TX_REQ_STB_ALL);
        }
        else // 没有任何数据需要发送
            break;

        if (status == LL_OK)
            s_needSendOldFlag = 0;
        else
        {
            s_needSendOldFlag = 1;
            break;
        }
        if (c_statyTimeOut < xTaskGetTickCount() - stayWorkingTime) // 如果呆的时间太久就强制退出
            break;
    }
}

/**
 * @brief 将被放入发送队列的数据顺序读取出来进行发送
 */
static void HandleCan2Tx(void)
{
    static CanTxMsg_t s_canTxMsg = {0};
    uint8_t status = LL_OK;
    static uint8_t s_needSendOldFlag = 0;
    TickType_t stayWorkingTime = xTaskGetTickCount(); // 在死循环中保持的时间不能太长
    const TickType_t c_statyTimeOut = pdMS_TO_TICKS(2);

    while (1)
    {
        if (s_needSendOldFlag == 1) // 上一次读取了数据，但是没有空邮箱可以发送
        {
            status = CAN_FillTxFrame(CAN2_UNIT, CAN_TX_BUF_PTB, &s_canTxMsg.head);
            CAN_StartTx(CAN2_UNIT, CAN_TX_REQ_PTB);
            CAN_StartTx(CAN2_UNIT, CAN_TX_REQ_STB_ALL);
        }
        else if (ReadCan2TxBuf(&s_canTxMsg) == 0) // 没有老数据，则读取新数据
        {
            status = CAN_FillTxFrame(CAN2_UNIT, CAN_TX_BUF_PTB, &s_canTxMsg.head);
            CAN_StartTx(CAN2_UNIT, CAN_TX_REQ_PTB);
            CAN_StartTx(CAN2_UNIT, CAN_TX_REQ_STB_ALL);
        }
        else // 没有任何数据需要发送
            break;

        if (status == LL_OK)
            s_needSendOldFlag = 0;
        else
        {
            s_needSendOldFlag = 1;
            break;
        }

        if (c_statyTimeOut < xTaskGetTickCount() - stayWorkingTime) // 如果呆的时间太久就强制退出
            break;
    }
}

int ReadCan1TxBuf(CanTxMsg_t *fp_canTxMsg)
{
    return ReadMutexCanTxBuf(&can1TxBuf, fp_canTxMsg);
}

int ReadCan2TxBuf(CanTxMsg_t *fp_canTxMsg)
{
    return ReadMutexCanTxBuf(&can2TxBuf, fp_canTxMsg);
}

/**
 * @brief can发送任务
 * @param  pvParameters      Param doc
 */
void CanTask(void const *pvParameters)
{
    TickType_t lastWakeTime;
    const TickType_t c_frequency = pdMS_TO_TICKS(2); // xms进行一次任务处理
    uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    InitSoftwareCanTask();
    /* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        /* 将缓存内的can数据发送出去 */
        HandleCan1Tx();
        HandleCan2Tx();

        vTaskDelay(c_frequency);
    }
}

/**
 * @brief 创建两个队列,放需要tx的can数据
 * @return int 
 */
int InitSoftwareCanTask(void)
{
    if (InitMutexCanTxBuf(&can1TxBuf) != 0)
        return 1;
    if (InitMutexCanTxBuf(&can2TxBuf) != 0)
        return 1;
    return 0;
}
void ota_send(   CanTxMsg_t *bms_pack)
{
    uint8_t status = LL_OK;
    
	status = CAN_FillTxFrame(CAN2_UNIT, CAN_TX_BUF_PTB, &bms_pack->head);

	if(status != LL_OK)
	{
		printf("can tx error:%d\r\n",status);
	}
	CAN_StartTx(CAN2_UNIT, CAN_TX_REQ_PTB);
    #if 1
    printf("ota_send data: 0x%X\trtr: %d\tide: %d\r\n", bms_pack->head.u32ID, bms_pack->head.RTR, bms_pack->head.IDE);
    for (int i = 0; i < bms_pack->head.DLC ; i++)
        printf("%02X ", bms_pack->head.au8Data[i]);
    printf("\r\n");
    #endif
    memset(bms_pack, 0, sizeof(CanTxMsg_t));
    vTaskDelay(5);
}