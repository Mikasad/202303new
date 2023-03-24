#include "canopen_lifegrd.h"
#include "main.h"
#include "stdbool.h"
/*------------------------------------------------
Function:消费者跳包入口定时事件回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void ConsumerHeartbeatAlarm(CO_Data* d, unsigned int id)
{

    unsigned char nodeId = (unsigned char)(((d->ConsumerHeartbeatEntries[id]) & (unsigned int)0x00FF0000) >> (unsigned char)16);

    d->ConsumerHeartBeatTimers[id] = TIMER_NONE;

    d->NMTable[nodeId] = DIS_CONNECTED;

    (*d->heartbeatError)(d, nodeId);
}
/*------------------------------------------------
Function:处理节点守护报文
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HeartBeat_Message_Handle(CO_Data *d, CanRxMsgTypeDef *m)
{
    unsigned char nodeId = (unsigned char)GET_NODE_ID(m->StdId);

    if(m->RTR == RTR_VALID)
    {
        if(nodeId == d->Node_ID)
        {
            CanTxMsgTypeDef msg;

            /* MNT报文(数据帧表示响应，远程帧表示请求) */
            unsigned short tmp = d->Node_ID + 0x700;
            /* cob-id */
            msg.StdId = UNS16_LE(tmp);
            /* 长度 */
            msg.DLC = (unsigned char)0x01;
            /* 数据帧：响应 */
            msg.RTR = RTR_INVALID;
            /* 节点状态 */
            msg.Data[0] = d->Node_state;
            /* 同步标志 */
            if(pHeartBeatPar.toggle)
            {
                msg.Data[0] |= 0x80;
                pHeartBeatPar.toggle = 0;
            }
            else
                pHeartBeatPar.toggle = 1;

            /* 发送报文 */
            CAN2_Send_Msg(msg.StdId,CAN_ID_STD,msg.RTR,msg.Data,msg.DLC);
        }
    }
    else
    {
        unsigned char New_NodeState = (unsigned char)((*m).Data[0] & 0x7F);

        if(d->NMTable[nodeId] != New_NodeState)
        {
            (*d->post_SlaveStateChange)(d, nodeId, New_NodeState);
            d->NMTable[nodeId] = New_NodeState;
        }
        /* 如果该节点上报过来的不是未知状态，则要对该节点进行重新倒计时 */
        if(d->NMTable[nodeId] != UNKNOWN_STATE)
        {
            unsigned char i,ConsumerHeartBeat_nodeId;

            for(i = (unsigned char)0x00; i < *d->ConsumerHeartbeatCount; i++)
            {
                ConsumerHeartBeat_nodeId = (unsigned char)(((d->ConsumerHeartbeatEntries[i]) & (unsigned int)0x00FF0000) >> (unsigned char)16);
                if(nodeId == ConsumerHeartBeat_nodeId)
                {
                    TIMEVAL time = ((d->ConsumerHeartbeatEntries[i]) & (unsigned int)0x0000FFFF);
                    DelAlarm(d->ConsumerHeartBeatTimers[i]);
                    /* 重新设置消费者(主站)心跳包入口(从站)定时事件 */
                    d->ConsumerHeartBeatTimers[i] = SetAlarm(d, i, &ConsumerHeartbeatAlarm, MS_TO_TIMEVAL(time), 0);
                }
            }
        }
    }
}

/*------------------------------------------------
Function:生产者(从站)心跳包定时事件回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void ProducerHeartbeatAlarm(CO_Data *d, unsigned int id)
{
    if(pHeartBeatPar.Timeout_server)
    {
        CanTxMsgTypeDef msg;

        /* MNT报文(数据帧表示响应，远程帧表示请求) */
        unsigned short tmp = d->Node_ID + 0x700;

        /* cob-id */
        msg.StdId = UNS16_LE(tmp);
        /* 长度 */
        msg.DLC = (unsigned char)0x01;
        /* 数据帧：响应 */
        msg.RTR = RTR_INVALID;
        /* 节点运行状态 */
        msg.Data[0] = d->Node_state;

        /* 发送该帧 */
        CAN2_Send_Msg(msg.StdId,CAN_ID_STD,msg.RTR,msg.Data,msg.DLC);
    }
    else
    {
        pHeartBeatPar.Timeout_server_timer = DelAlarm(pHeartBeatPar.Timeout_server_timer);
    }
}

/*------------------------------------------------
Function:初始化心跳报文
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void heartbeatInit(CO_Data *d)
{
//    unsigned char i = 0;

    RegisterSetOnDentry_Callback(d, 0x1017, 0x00, &OnHeartbeatProducerUpdate);

    /* 同步标志位置0 */
    pHeartBeatPar.toggle = 0;

#ifdef _INITCONSUME
    for(unsigned char i = 0x00; i < *d->ConsumerHeartbeatCount; i++)//不要写的过大，因为会占用 定时事件的个数
    {
        TIMEVAL ConsumeTime = (unsigned short int)((d->ConsumerHeartbeatEntries[i]) & (unsigned int)0x0000FFFF);

        if(ConsumeTime)
        {
            d->ConsumerHeartBeatTimers[i] = SetAlarm(d, i, &ConsumerHeartbeatAlarm, MS_TO_TIMEVAL(ConsumeTime), 0);
        }
    }
#endif
    /* 生产者(从站)注册一个定时事件，在规定时间内上报心跳包 */
    if(pHeartBeatPar.Timeout_server)
    {
        TIMEVAL time = pHeartBeatPar.Timeout_server;
        pHeartBeatPar.Timeout_server_timer = SetAlarm(d, 0, &ProducerHeartbeatAlarm, MS_TO_TIMEVAL(time), MS_TO_TIMEVAL(time));
    }
}

/*------------------------------------------------
Function:停止心跳包
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void heartbeatStop(CO_Data *d)
{

    /* 删除生产者(从站)心跳包定时器 */
    pHeartBeatPar.Timeout_server_timer = DelAlarm( pHeartBeatPar.Timeout_server_timer);
}

/*------------------------------------------------
Function:如果字典中索引为0x1017的条目发生变化，调用该函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int OnHeartbeatProducerUpdate(CO_Data *d, const indextable *unused_indextable, unsigned char unused_bSubindex)
{
    heartbeatStop(d);
    heartbeatInit(d);

    return 0;
}
/*------------------------------------------------
Function:生命守护机制初始化
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void lifeGuardInit(CO_Data *d)
{
    heartbeatInit(d);
}

/*------------------------------------------------
Function:生命守护机制停止
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void lifeGuardStop(CO_Data *d)
{
    heartbeatStop(d);
}


/*------------------------------------------------
Function:心跳错误回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u8 _heartbeatErrorFlag;
extern  bool MC_StopMotor1(void), MC_StopMotor2(void);
void _heartbeatError(CO_Data *d, unsigned char heartbeatID)
{
    MC_StopMotor1();
//    for(u8 i=0; i< MOTOR_NUM ; i++)
//    {
//        MotorControl[i].Motor_Start_Stop = DISABLE;
    MC_StopMotor2();
    _heartbeatErrorFlag = 2;
//	MotorState = RUN;
}
/*------------------------------------------------
Function:监控的节点状态改变回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void _post_SlaveStateChange(CO_Data* d, unsigned char nodeId, unsigned char newNodeState)
{
//  ATLAS_PRINT("Node:%d  State Change to:%d\r\n",nodeId,newNodeState);
}
/*------------------------------------------------
Function:消费者心跳超时子索引改变回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char test1016 = 0;
unsigned int  ODCallback_t_Index1016_Subindex(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
    unsigned int IndexOffset = 19;
    unsigned int newConsumerNodeID = 0;
    unsigned int pastConsumerNodeID = 0;
    unsigned char i = 0;
test1016 = *(unsigned char *)index->pSubindex[bSubindex].lpParam;
    /* 消费者心跳超时子索引0：0-5 */
    if(bSubindex == 0) //子索引0中写入的数据只能是 0---5
    {
        if((*(unsigned char *)index->pSubindex[bSubindex].lpParam > 5))//{(*(unsigned char *)index->pSubindex[bSubindex].lpParam < 0)||}因为子索引的数据类型为 unsigned char 所以 与0比较会告警
        {
					
            memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
            return SDOABT_GENERAL_ERROR;
        }
    }
    else
    {
        newConsumerNodeID = (unsigned char)(((*(unsigned int *)index->pSubindex[bSubindex].lpParam) & (unsigned int)0x00FF0000) >> (unsigned char)16);
        for(i=0; i<5; i++)
        {
            pastConsumerNodeID = (bSubindex != (i+1))?(unsigned char)(((*(unsigned int *)d->objdict[IndexOffset].pSubindex[i+1].lpParam) & (unsigned int)0x00FF0000) >> (unsigned char)16):0;
            if(newConsumerNodeID == pastConsumerNodeID)
            {
                memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
                return GEERAL_PARA_INCOMPATIBILITY_REASON;
            }
        }
    }

    return 0;
}
