#include "canopen_nmt.h"
#include "bsp_can.h"
#include "canopen_objacces.h"
#include "canopen_sdo.h"
#include "canopen_pdo.h"
#include "canopen_sync.h"
#include "canopen_lifegrd.h"
#include "canopen_emcy.h"

/*------------------------------------------------
Function:Switch CommunicationState
Input   :d  *newCommunicationState
Output  :No
Explain :No
------------------------------------------------*/
void SwitchCommunication_State(CO_Data* d, s_state_communication *newCommunicationState)
{
  /* NMT State Run or Not BEGIN*/
    /* Boot-up State */
    StartOrStop(csBoot_Up,	None,	SendBootUp(d))	
    /* SDO State */
    StartOrStop(csSDO,	None,		resetSDO(d))
    /* EMCY State */
    StartOrStop(csEmergency,	emergencyInit(d),	emergencyStop(d))
		/* SYNC State */																																									 
    StartOrStop(csSYNC,	None,		StopSync(d))
    /* LIFE-GUARD State */
    StartOrStop(csLifeGuard,	lifeGuardInit(d),	lifeGuardStop(d))
    /* PDO State */
    StartOrStop(csPDO,	PDOInit(d),	PDOStop(d))
    #ifdef CO_ENABLE_LSS
    /* LSS State  */
    StartOrStop(csLSS,	None,None)
    #endif
  /* NMT State Run or Not END*/
}

/*------------------------------------------------
Function:NMT set state
Input   :d  newstate
Output  :Result
Explain :No
------------------------------------------------*/
unsigned char SetState(CO_Data *d,unsigned  int newstate)
{
	if(newstate != d->NMT_state)
	{
		switch(newstate)
		{
			case INITIALIZATION:
			{
				/* Boot-up  SDO  EMCY  Sync  LifeGuard  PDO  Lss */
				s_state_communication newCommunicationState = {1, 0, 0, 0, 0, 0, 0};//设置当前状态下所允许服务
        d->Node_state = HEART_STAT_BOOTUP;
        d->NMT_state = INITIALIZATION;
				SwitchCommunication_State(d, &newCommunicationState);
        (*d->initialisation)(d);
			}
			case PRE_OPERATIONG:
			{
				s_state_communication newCommunicationState = {0, 1, 1, 1, 1, 0, 1};
        d->Node_state = HEART_STAT_PRE_OPERAT;
        d->NMT_state = PRE_OPERATIONG;
				SwitchCommunication_State(d, &newCommunicationState);
        (*d->preOperational)(d);
			}break;
			case OPERATION_STATE:
			{
				s_state_communication newCommunicationState = {0, 1, 1, 1, 1, 1, 0};
        d->Node_state = HEART_STAT_OPERAT;
        d->NMT_state = OPERATION_STATE;
				SwitchCommunication_State(d, &newCommunicationState);
        (*d->operational)(d);
			}break;
			case STOP:
			{
				s_state_communication newCommunicationState = {0, 0, 0, 0, 1, 0, 1};
        d->Node_state = HEART_STAT_STOP;
        d->NMT_state = STOP;
				SwitchCommunication_State(d, &newCommunicationState);
        (*d->stopped)(d);
			}break;
			default:
        return 0xFF;
		}
	}
    return d->NMT_state;
}


/*------------------------------------------------
Function:网络管理对象
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void Net_Manager_Handle(CO_Data *d)
{
  
  if(d->NMT_state == PRE_OPERATIONG || \
				 d->NMT_state == OPERATION_STATE || \
				 d->NMT_state == STOP \
				)
	{
    /* 节点ID为0则“Commad Specifier”被广播至所有CANopen从站 */
    if((d->CanRx_Buffer->Data[1] == d->Node_ID)||(d->CanRx_Buffer->Data[1] == 0)) 
    {
      switch(d->CanRx_Buffer->Data[0])
      {
        case NMT_CS_START_REMODE_NODE:
        {
            if(d->NMT_state == PRE_OPERATIONG || d->NMT_state == STOP)
            {
              SetState(d,OPERATION_STATE);
            }
        }break;
        /* Not allowed in Motor Enable */
        case NMT_CS_STOP_REMODE_NODE:
        {
//            if(pAxisPar.motorEn[A_AXIS] != MOTOR_OFF)
//            {
//              /* 电机使能时NMT转向STOP */
//              Device_Error_Handler(DEVICE_ERR_SET,ERR_TURNTOSTOP_INEN,A_AXIS);
//              break;
//            }
            if(d->NMT_state == PRE_OPERATIONG || d->NMT_state == OPERATION_STATE)
            {
              SetState(d,STOP);
            }
        }break;
        case NMT_CS_ENTER_PRE_OPERAT_STAT:
        {
            if(d->NMT_state == OPERATION_STATE || d->NMT_state == STOP)
            {
              SetState(d,PRE_OPERATIONG);
            }
        }break;
        /* Not allowed in Motor Enable */
        case NMT_CS_RESET_NODE:
        {   
//            if(pAxisPar.motorEn[A_AXIS] != MOTOR_OFF)
//            {
//              /* 电机使能时NMT转向INIT */
//              Device_Error_Handler(DEVICE_ERR_SET,ERR_TURNTOINIT_INEN,A_AXIS);
//              break;
//            }
            /* 如果重启节点回调函数不为空则调用 */
            if(d->NMT_Slave_Node_Reset_Callback != NULL)
                d->NMT_Slave_Node_Reset_Callback(d);
            SetState(d,INITIALIZATION);
        }break;
        /* Not allowed in Motor Enable */
        case NMT_CS_RESET_COMMUNICATION:
        {
            unsigned char CurrNode_Id = getNodeId(d);
          
 //            if(pAxisPar.motorEn[A_AXIS] != MOTOR_OFF)
//            {
//              /* 电机使能时NMT转向INIT */
//              Device_Error_Handler(DEVICE_ERR_SET,ERR_TURNTOINIT_INEN,A_AXIS);
//              break;
//            }
            /* 如果重启通信回调函数不为空则调用 */
            if(d->NMT_Slave_Communications_Reset_Callback != NULL)
                d->NMT_Slave_Communications_Reset_Callback(d);
            SetNodeId(d, CurrNode_Id);
            SetState(d,INITIALIZATION);
        }break;
        default:break;
      }
    }
  }
}
/*------------------------------------------------
Function:报文类型检查
Input   :d  COB-ID  RTR
Output  :No
Explain :No
------------------------------------------------*/
extern int xintiao_flag;
extern int heartcnt;
unsigned char MessageType_Check(CO_Data *d,unsigned int Cob_id,unsigned char rtr)
{
  unsigned char funcCmd;
  
   funcCmd = GET_FUNCION_CODE(Cob_id);

  switch(funcCmd)
  {
    case TRANS_TYPE_NMT:  
    {
			Net_Manager_Handle(d);
    }break;
    case (TRANS_TYPE_EMCY & TRANS_TYPE_SYNC):
    { 
      //SYNC
      if(GET_NODE_ID(Cob_id) == 0)
      {
        proceedSYNC(d);
      }
      //EMCY
      else
      {
        
      }
    }break;
    case TRANS_TYPE_TIME: 
    {
      
    }break;
    case TRANS_TYPE_TPDO1:
    case TRANS_TYPE_TPDO2:
    case TRANS_TYPE_TPDO3:
    case TRANS_TYPE_TPDO4:
    case TRANS_TYPE_RPDO1:
    case TRANS_TYPE_RPDO2:
    case TRANS_TYPE_RPDO3:
    case TRANS_TYPE_RPDO4:
    {
      if(d->CurrentCommunicationState.csPDO)
      {
        proceedPDO(d,d->CanRx_Buffer);
      }
    }break;
    /* RSDO报文 */  
    case TRANS_TYPE_RSDO: 
    {
      if(rtr == RTR_VALID)
        break;
      if(d->CurrentCommunicationState.csSDO)
      {
        SDO_Email_Handler(d,d->CanRx_Buffer);
      }
      
    }break;
    /* Heart_beat */
    case TRANS_TYPE_HEART_BEAT: 
		xintiao_flag=1;
//	     heartcnt=0;
    {
      if(d->CurrentCommunicationState.csLifeGuard)
        HeartBeat_Message_Handle(d,d->CanRx_Buffer);
    }break;
    default:break;
  }
  
  return 0;
}
/*------------------------------------------------
Function:getNodeId
Input   :d
Output  :Node_ID
Explain :No
------------------------------------------------*/
unsigned char getNodeId(CO_Data* d)
{
  return d->Node_ID;
}
/*------------------------------------------------
Function:Set Nodeid
Input   :d  nodeId
Output  :No
Explain :No
------------------------------------------------*/
void SetNodeId(CO_Data *d, unsigned char nodeId)
{
  unsigned short offset = d->firstIndex->SDO_SVR;
  
  if(!(nodeId > 0 && nodeId <= 127))
	{
//	  ATLAS_PRINT("Invalid NodeID：%d\r\n",nodeId);
	  return;
  }
  /* SDO Cob_ID */
  if(offset)
	{
    /* Adjust COB-ID Client->Server (rx) only id already set to default value or id not valid (id==0xFF)*/
    if((*(unsigned int*)d->objdict[offset].pSubindex[1].lpParam == 0x600 + d->Node_ID)||(d->Node_ID==0xFF))
		{
      /* cob_id_client = 0x600 + nodeId; */
      *(unsigned int*)d->objdict[offset].pSubindex[1].lpParam = 0x600 + nodeId;
    }
    /* Adjust COB-ID Server -> Client (tx) only id already set to default value or id not valid (id==0xFF)*/
    if((*(unsigned int*)d->objdict[offset].pSubindex[2].lpParam == 0x580 + d->Node_ID)||(d->Node_ID==0xFF))
		{
      /* cob_id_server = 0x580 + nodeId; */
      *(unsigned int*)d->objdict[offset].pSubindex[2].lpParam = 0x580 + nodeId;
    }
  }
  /* rPDO Cob_ID */
  {
    unsigned char i = 0;
    unsigned short offset = d->firstIndex->PDO_RCV;
    unsigned short lastIndex = d->lastIndex->PDO_RCV;
    unsigned int cobID[] = {0x200, 0x300, 0x400, 0x500};
    if(offset)
		{
			while((offset <= lastIndex) && (i < 4)) 
			{
				if((*(unsigned int*)d->objdict[offset].pSubindex[1].lpParam == cobID[i] + d->Node_ID)||(d->Node_ID==0xFF))
					*(unsigned int*)d->objdict[offset].pSubindex[1].lpParam = cobID[i] + nodeId;
				i ++;
				offset ++;
			}
		}
  }
  /* Initialize the TPDO communication parameters. Only for 0x1800 to 0x1803 */
  {
    unsigned char i = 0;
    unsigned short offset = d->firstIndex->PDO_TRS;
    unsigned short lastIndex = d->lastIndex->PDO_TRS;
    unsigned int cobID[] = {0x180, 0x280, 0x380, 0x480};
    i = 0;
    if( offset ) while ((offset <= lastIndex) && (i < 4)) 
		{
      if((*(unsigned int*)d->objdict[offset].pSubindex[1].lpParam == cobID[i] + d->Node_ID)||(d->Node_ID==0xFF))
	      *(unsigned int*)d->objdict[offset].pSubindex[1].lpParam = cobID[i] + nodeId;
      i ++;
      offset ++;
    }
  }

  /* Update EMCY COB-ID if already set to default*/
  if((*d->error_cobid == d->Node_ID + 0x80)||(d->Node_ID==0xFF))
    *d->error_cobid = nodeId + 0x80;

  /* bDeviceNodeId is defined in the object dictionary. */
  d->Node_ID = nodeId;
}
/*------------------------------------------------
Function:SendBootUp
Input   :d
Output  :Msg Sent Result
Explain :No
------------------------------------------------*/
unsigned char SendBootUp(CO_Data *d)
{
  /*Boot-up*/
  d->CanTx_Buffer->StdId = d->Node_ID + HEART_COB_ID_BASE;//Boot-up
  d->CanTx_Buffer->RTR = RTR_INVALID;
  d->CanTx_Buffer->DLC = 0x01;
  d->CanTx_Buffer->Data[0] = HEART_STAT_BOOTUP;
  
  return CAN1_Send_Msg(d->CanTx_Buffer->StdId,CAN_ID_STD,CAN_RTR_DATA,d->CanTx_Buffer->Data,d->CanTx_Buffer->DLC);
}
/*------------------------------------------------
Function:进入初始化状态回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void _initialisation(CO_Data* d){}
/*------------------------------------------------
Function:进入预运行状态回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void _preOperational(CO_Data* d){}
/*------------------------------------------------
Function:进入运行状态回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void _operational(CO_Data* d){}
/*------------------------------------------------
Function:进入停止状态回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void _stopped(CO_Data* d){}


