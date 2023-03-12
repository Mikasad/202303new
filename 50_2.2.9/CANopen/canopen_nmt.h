#ifndef __CANOPEN_NMT_H__
#define __CANOPEN_NMT_H__
#include "stm32f4xx_hal.h"
#include "canopen_def.h"
#include "string.h"
#include "canopen_od.h"
#include "canopen_objacces.h"

/*NMT_FSM_Define*/ 
#define INITIALIZATION    0x0000
#define DIS_CONNECTED			0x0001
#define CONNECTING				0x0002
#define PREPARING					0x0002
#define STOP              0x0004
#define OPERATION_STATE   0x0005
#define PRE_OPERATIONG    0x007F
#define UNKNOWN_STATE			0x000F

//最高优先级COB-ID
#define NMT_COB_ID        0x0000
//Exist in byte0(NMT报文指令命令符)
#define NMT_CS_START_REMODE_NODE      0x01    //进入OPERATION
#define NMT_CS_STOP_REMODE_NODE       0x02    //进入STOP
#define NMT_CS_ENTER_PRE_OPERAT_STAT  0x80    //进入PRE_OPERATIONG
#define NMT_CS_RESET_NODE             0x81    //进入INITIALIZATION
#define NMT_CS_RESET_COMMUNICATION    0x82    //进入INITIALIZATION
//下列用于差错控制服务(监控从站状态)
//心跳协议(COB-ID & byte0)
#define HEART_COB_ID_BASE     0x700 //Add NodeID
#define HEART_STAT_BOOTUP     0x00  //心跳监控：从站状态
#define HEART_STAT_STOP       0x04
#define HEART_STAT_OPERAT     0x05
#define HEART_STAT_PRE_OPERAT 0x7F
//报文类型(COB-ID高4位)
#define TRANS_TYPE_NMT            0x0 //0000
#define TRANS_TYPE_SYNC           0x1 //0001  Node-ID 为空
#define TRANS_TYPE_EMCY           0x1 //0001  Node-ID 不为空
#define TRANS_TYPE_TIME           0x2 //0010
#define TRANS_TYPE_TPDO1          0x3 //0011
#define TRANS_TYPE_RPDO1          0x4 //0100
#define TRANS_TYPE_TPDO2          0x5 //0101
#define TRANS_TYPE_RPDO2          0x6 //0110
#define TRANS_TYPE_TPDO3          0x7 //0111
#define TRANS_TYPE_RPDO3          0x8 //1000
#define TRANS_TYPE_TPDO4          0x9 //1001
#define TRANS_TYPE_RPDO4          0xA //1010
#define TRANS_TYPE_TSDO           0xB //1011 从站发送，主站接收
#define TRANS_TYPE_RSDO           0xC //1100 主站发送，从站接收
#define TRANS_TYPE_HEART_BEAT     0xE //1110
//CAN-ID
#define CANID_NMT                 0x00
#define CANID_SYNC                0x80
#define CANID_EMCY                0x100
#define CANID_TIME                0x80h
#define CANID_TPDO1               0x00000182
#define CANID_RPDO1               0x00000202
#define CANID_TPDO2               0x80000282
#define CANID_RPDO2               0x80000302
#define CANID_TPDO3               0x80000382
#define CANID_RPDO3               0x80000402
#define CANID_TPDO4               0x00000482
#define CANID_RPDO4               0x80000502
#define CANID_TSDO                0x580
#define CANID_RSDO                0x600
#define CANID_HEART_BEAT          0x700

/*Timing Base*/
#define CLK_FREQUENCY   10000
#define CLK_TIME        0.0001f

/* Status of the node during the SDO transfer : */
#define SDO_SERVER  0x01
#define SDO_CLIENT  0x02
#define SDO_UNKNOWN 0x03

/* 进行几种服务功能的开启or停止 */
//入口参数：服务功能类型、功能开启函数、功能停止函数
#define StartOrStop(CommType, FuncStart, FuncStop) \
	if(newCommunicationState->CommType && d->CurrentCommunicationState.CommType == 0){\
    d->CurrentCommunicationState.CommType = 1;\
		FuncStart;\
	}else if(!newCommunicationState->CommType && d->CurrentCommunicationState.CommType == 1){\
    d->CurrentCommunicationState.CommType = 0;\
		FuncStop;\
	}
#define None

#define GET_NODE_ID(m)       (m & 0x7f )
#define GET_FUNCION_CODE(m)  (m >> 7)
#define GET_IF_IN_MAPPLE(index) (((index>=0x1600&&index<=0x1603)||(index>=0x1A00&&index<=0x1A03))?1:0)

/*Function Define*/
void SwitchCommunication_State(CO_Data *d, s_state_communication *newCommunicationState);
unsigned char SetState(CO_Data *d,unsigned  int newstate);
void Net_Manager_Handle(CO_Data *d);
unsigned char MessageType_Check(CO_Data *d,unsigned int Cob_id,unsigned char rtr);
unsigned char getNodeId(CO_Data* d);
void SetNodeId(CO_Data *d,unsigned char nodeId); 
unsigned char SendBootUp(CO_Data *d);
void _initialisation(CO_Data* d);
void _preOperational(CO_Data* d);
void _operational(CO_Data* d);
void _stopped(CO_Data* d);  

#endif

