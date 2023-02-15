#include "canopen_pdo.h"
#include "ds402.h"
//void Device_Error_Handler(unsigned char HandleType,long errcode,short axisNum)
//{
//  obj6040_Control_word = CONTROLWORD_COMMAND_DISABLEVOLTAGE;
//  /* Inadition diffent Errcode */
//  switch(errcode)
//  {
//    /* Over Current error */
//    case ERR_CURR_OVER_MAX:
//    case ERR_CURR_SAT_OVER_PEAK_TIME:  
//    case ERR_PHASE_A_CURR:
//    case ERR_PHASE_B_CURR:
//    case ERR_PHASE_C_CURR:
//    {
//      /* Handle Entry */
//      if(HandleType)
//      {
//        /* Device Disabled */
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        /* Emcy record - Error Flag Set In -> Status Word */
//        EmcyMsg_TransSet(&EmcyTransData,OC1_ERR,ERR_REG_BIT1_SET,errcode);
//      }
//      else
//      {
//        /*  Err Recovered - Reset Fault with ->Control Word  */
//        EMCY_errorRecovered(&CANopen_Drive, OC1_ERR);
//      }
//    }break;  
//    case ERR_CURR_12_OVER_LOAD:
//    case ERR_CURR_15_OVER_LOAD:
//    case ERR_CURR_20_OVER_LOAD:
//    case ERR_CURR_25_OVER_LOAD:
//    case ERR_CURR_30_OVER_LOAD:
//    {
//      /* Handle Entry */
//      if(HandleType)
//      {
//        /* Device Disabled */
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        /* Emcy record - Error Flag Set In -> Status Word */
//        EmcyMsg_TransSet(&EmcyTransData,MOTOR_OVERLOAD,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        /*  Err Recovered - Reset Fault with ->Control Word  */
//        EMCY_errorRecovered(&CANopen_Drive, MOTOR_OVERLOAD);
//      }
//    }break;
//    case ERR_OVER_VEL_ERR:
//    {
//      /* Status Word Bit13 */
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        /* Emcy record */
//        EmcyMsg_TransSet(&EmcyTransData,VEL_OVER_MAX_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        /*  Err Recovered */
//        EMCY_errorRecovered(&CANopen_Drive, VEL_OVER_MAX_ERR);
//      }
//    }break;
//    case ERR_OVER_POS_ERR:
//    case ERR_POS_FOLLOWING_ERR:
//    {
//      /* Status Word Bit13 */
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = QUICKSTOP_RAMP;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;//Enter Fault Reaction
//        /* Emcy record */
//        EmcyMsg_TransSet(&EmcyTransData,FLOWING_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        /*  Err Recovered */
//        EMCY_errorRecovered(&CANopen_Drive, FLOWING_ERR);
//      }
//    }break;
//    case ERR_ENC_DISCNNET_A:
//    case ERR_ENC_DISCNNET_B:
//    case ERR_ENC_DISCNNET_Z:
//    {
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        /* TODO:here should execute Pos Clear */
//        EmcyMsg_TransSet(&EmcyTransData,POS_SENSOR_BREACH_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive,POS_SENSOR_BREACH_ERR);
//      }
//    }break;
//    case ERR_HARD_LIM_STATUS:
//    {
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        EmcyMsg_TransSet(&EmcyTransData,NEG_LIMIT_SWITCH_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, NEG_LIMIT_SWITCH_ERR);
//      }
//    }break;
//    case ERR_HOMING_UNEXPECTED_MOTOR_OFF:
//    case ERR_DI_HOMING_FUNC_NOT_ASSIGNED:
//    case ERR_HOMING_TIME_OUT:
//    case ERR_HOMING_START_IN_MOTION:
//    case ERR_HOMING_UNEXPECTED_STOP_REQ:
//    {
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        EmcyMsg_TransSet(&EmcyTransData,HOMING_PROCESS_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, HOMING_PROCESS_ERR);
//        if(pBaseCtrlPar.controlMode[A_AXIS] == HOMING_OP_MODE)
//        {
//          obj6041_Status_word &= ~Bit_Homing_Error;
//        }
//      }
//    }break;
//    case ERR_125_TIMES_OVER_VELOCITY:
//    {
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        EmcyMsg_TransSet(&EmcyTransData,VEL_125_OVERLOAD_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, VEL_125_OVERLOAD_ERR);
//      }
//    }break;
//    case ERR_HALL_STUDY_ERR:
//    {
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        /* TODO:here should execute Pos Clear */
//        EmcyMsg_TransSet(&EmcyTransData,AUTO_TUNING_HALL_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, AUTO_TUNING_HALL_ERR);
//      }
//    }break;
//    case ERR_SERVO_MAX_LIMIT_VOLTAGE:
//    case ERR_REGEN_RES_OVER_LOAD:
//    {
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        EmcyMsg_TransSet(&EmcyTransData,OV1_ERR,ERR_REG_BIT2_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, OV1_ERR);
//      }
//    }break;
//    case ERR_MOTOR_STUCK_ERR:
//    {
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        EmcyMsg_TransSet(&EmcyTransData,MOTOR_STUCK_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, MOTOR_STUCK_ERR);
//      }
//    }break;
//    case ERR_TURNTOINIT_INEN:
//    {
//      if(HandleType)
//      {
//        EmcyMsg_TransSet(&EmcyTransData,TURN_TO_INIT_INENABLE_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, TURN_TO_INIT_INENABLE_ERR);
//      }
//    }break;
//    case ERR_TURNTOSTOP_INEN:
//    {
//      if(HandleType)
//      {
//        EmcyMsg_TransSet(&EmcyTransData,TURN_TO_STOP_INENABLE_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, TURN_TO_STOP_INENABLE_ERR);
//      }
//    }break;
//    case ERR_PDO_TRANS_LEN:
//    {
//      if(HandleType)
//      {
//        EmcyMsg_TransSet(&EmcyTransData,PDO_TRANS_LEN_ERR,ERR_REG_BIT4_SET,errcode);
//      }
//      else
//      {
//        EMCY_errorRecovered(&CANopen_Drive, PDO_TRANS_LEN_ERR);
//      }
//    }break;
//    case ERR_HEARTBEAT_TIMEOUT:
//    {
//      /* Handle Entry */
//      if(HandleType)
//      {
//        /* Device Disabled */
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        /* Emcy record - Error Flag Set In -> Status Word */
//        EmcyMsg_TransSet(&EmcyTransData,HEARTBEAT_TIMEOUT_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        /*  Err Recovered - Reset Fault with ->Control Word  */
//        EMCY_errorRecovered(&CANopen_Drive, HEARTBEAT_TIMEOUT_ERR);
//      }
//    }break;
//    case ERR_UNDERVOLTAGE_LOCKOUT     :
//    case ERR_CHARGE_PUMP_UNDERVOLTAGE : 
//    case ERR_VDS_OVERCURRENT          :
//    case ERR_SENSE_OVERCURRENT        :
//    case ERR_GATE_DRIVE_FAULT         :
//    case ERR_OVERTEMPERATURE_SHUTDOWN :
//    case ERR_OVERTEMPERATURE_WARNING  :
//    {
//      if(HandleType)
//      {
//        pBaseCtrlPar.tStopOpPar.faultStopOp[A_AXIS] = DISABLE_DRIVE;
//        Driver_Data.drivesytemState = State_Fault_reaction_active;
//        /* TODO:here should execute Pos Clear */
//        EmcyMsg_TransSet(&EmcyTransData,SERVO_DRIVER_ERR,ERR_REG_BIT0_SET,errcode);
//      }
//      else
//      {
//        DRV8323S_SPI_WriteRead(0x1001);
//        EMCY_errorRecovered(&CANopen_Drive, SERVO_DRIVER_ERR);
//      }
//    }break;
//    default:break;
//  }
//}

/*------------------------------------------------
Function:构建pdo报文
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char buildPDO(CO_Data *d, unsigned char numPdo, CanTxMsgTypeDef * pdo)
{
  const indextable *TPDO_com = d->objdict + d->firstIndex->PDO_TRS + numPdo;

  const indextable *TPDO_map = d->objdict + d->firstIndex->PDO_TRS_MAP + numPdo;

  unsigned char prp_j = 0x00;
  unsigned int offset = 0x00000000;

  const unsigned char *pMappingCount = (unsigned char *)TPDO_map->pSubindex[0].lpParam;


  pdo->StdId = (unsigned short int) UNS16_LE(*(unsigned int*)TPDO_com->pSubindex[1].lpParam & 0x7FF);
	/* 数据帧 */
  pdo->RTR = RTR_INVALID;

//  ATLAS_PRINT("  PDO CobId is : %d\r\n", *(unsigned int *) TPDO_com->pSubindex[1].lpParam);
//  ATLAS_PRINT("  Number of objects mapped : %d\r\n", *pMappingCount);

	/* 将所有映射的数据拷贝到pdo缓冲区 */
  do
	{
		unsigned char dataType;
		unsigned char tmp[] = {0, 0, 0, 0, 0, 0, 0, 0};


		unsigned int *pMappingParameter = (unsigned int *)TPDO_map->pSubindex[prp_j + 1].lpParam;

		unsigned short int index = (unsigned short int)((*pMappingParameter) >> 16);

		unsigned int Size = (unsigned int)(*pMappingParameter & (unsigned int) 0x000000FF);


    /* 最多8byte */
		if(Size && ((offset + Size) <= 64))
		{

			unsigned int ByteSize = 1 + ((Size - 1) >> 3);

			unsigned char subIndex = (unsigned char)(((*pMappingParameter) >> (unsigned char)8) & (unsigned int)0x000000FF);

//			ATLAS_PRINT ("  got mapping parameter : %d\r\n", *pMappingParameter);
//			ATLAS_PRINT ("    at index : %d\r\n", TPDO_map->index);
//			ATLAS_PRINT ("    sub-index : %d\r\n", prp_j + 1);


			if(getODentry(d, index, subIndex, tmp, &ByteSize, &dataType, 0) != OD_SUCCESSFUL)
			{
//				ATLAS_ERR(" Couldn't find mapped variable at index-subindex-size : %d\r\n", (unsigned int)(*pMappingParameter));
				return 0xFF;
			}
			
			CopyBits((unsigned char)Size, ((unsigned char *)tmp), 0, 0, (unsigned char *)&pdo->Data[offset >> 3], (unsigned char)(offset % 8), 0);

			/* 位数偏移 */
			offset += Size;
		}
		
		/* 子索引加一 */
		prp_j++;
	}
  while(prp_j < *pMappingCount);

  pdo->DLC = (unsigned char)(1 + ((offset - 1) >> 3));

//  ATLAS_PRINT("  End scan mapped variable\r\n");

  return 0;
}

/*------------------------------------------------
Function:发送pdo请求
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char sendPDOrequest(CO_Data *d, unsigned short int RPDOIndex)
{
  unsigned short int *pwCobId;

  unsigned short int offset = d->firstIndex->PDO_RCV;

  unsigned short int lastIndex = d->lastIndex->PDO_RCV;


  if(!d->CurrentCommunicationState.csPDO)
  {
		return 0;
  }

//  ATLAS_PRINT("sendPDOrequest RPDO Index : %d\r\n", RPDOIndex);


  if(offset && RPDOIndex >= 0x1400)
	{

		offset += RPDOIndex - 0x1400;

		if(offset <= lastIndex)
		{
			/* 取出cob-id */
			pwCobId = d->objdict[offset].pSubindex[1].lpParam;

//			ATLAS_PRINT("sendPDOrequest cobId is : %d\r\n", *pwCobId);
			
			{
				CanTxMsgTypeDef pdo;
				pdo.StdId = UNS16_LE(*pwCobId);
				pdo.RTR = RTR_VALID;
				pdo.DLC = 0;
				
				/* 发送报文 */
				return CAN1_Send_Msg(pdo.StdId,CAN_ID_STD,pdo.RTR,pdo.Data,pdo.DLC);
			}
		}
	}
//  ATLAS_ERR("sendPDOrequest : RPDO Index not found : %d\r\n", RPDOIndex);
  
  return 0xFF;
}
/*------------------------------------------------
Function:pdo报文处理函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char proceedPDO(CO_Data *d, CanRxMsgTypeDef *m)
{
  unsigned char numPdo;
  unsigned char numMap;
  unsigned char *pMappingCount = NULL;
  unsigned int *pMappingParameter = NULL;
  unsigned char *pTransmissionType = NULL;
  unsigned short *pwCobId = NULL;
  unsigned char Size;
  unsigned char offset;
  unsigned char status;
  unsigned int objDict;
  unsigned short offsetObjdict;
  unsigned short lastIndex;
  unsigned int TimeCheck;
  
  status = state2;//初始化为state2

  offset = 0x00;
  numPdo = 0;
  numMap = 0;
	
  testGl[12] = (TIM1->CNT/84)+testGl[15]*100;
  testGl[15] = 0;
  TimeCheck = TIM6->CNT; 
  
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);

  if((*m).RTR == RTR_INVALID)
	{
		offsetObjdict = d->firstIndex->PDO_RCV;
		lastIndex = d->lastIndex->PDO_RCV;

		if(offsetObjdict)
		{
			while(offsetObjdict <= lastIndex)
			{
				switch(status)
				{
					case state2:
          {
						pwCobId = d->objdict[offsetObjdict].pSubindex[1].lpParam;
						if(*pwCobId == UNS16_LE(m->StdId))
						{
							status = state4;
//							ATLAS_PRINT("cobId found at index :%d\r\n", 0x1400 + numPdo);
							break;
						}
						else
						{
							numPdo++;
							offsetObjdict++;
							status = state2;
							break;
						}
          }
					case state4:
          {
						offsetObjdict = d->firstIndex->PDO_RCV_MAP;
						lastIndex = d->lastIndex->PDO_RCV_MAP;
                    pTransmissionType = (unsigned char *)d->objdict[d->firstIndex->PDO_RCV + numPdo].pSubindex[2].lpParam;
						
                    if((*pTransmissionType == SYNC_NOCYCLE)\
                            ||((*pTransmissionType >= SYNC_CYCLE_BEGIN) && (*pTransmissionType <= SYNC_CYCLE_END)))
                    {
                        if(*pTransmissionType == SYNC_NOCYCLE)
                        {
                            if(pRpdoDirectPar[numPdo].lastmessage.StdId == m->StdId &&
                                    pRpdoDirectPar[numPdo].lastmessage.DLC == m->DLC &&
                                    memcmp(pRpdoDirectPar[numPdo].lastmessage.Data, m->Data, 8) == 0)
                            {
                                return 0;
                            }
                        }
                        pRpdoDirectPar[numPdo].lastmessage = *m;
                        pRpdoDirectPar[numPdo].TransStatus |= PDD_REC_DATA_READY;
//              
                        return 0;
                    }
                    if((*pTransmissionType != ASYNC_TRIGGER_A)&&(*pTransmissionType != ASYNC_TRIGGER_B))
                    {
                        pRpdoDirectPar[numPdo].TransStatus &= ~PDD_REC_DATA_READY;
                        return 0xFF;
                    }


						pMappingCount = (unsigned char *)(d->objdict + offsetObjdict + numPdo)->pSubindex[0].lpParam;
						numMap = 0;
						while(numMap < *pMappingCount)
						{
							unsigned char tmp[] = {0, 0, 0, 0, 0, 0, 0, 0};
							unsigned int ByteSize;
							
							pMappingParameter = (unsigned int *)(d->objdict + offsetObjdict + numPdo)->pSubindex[numMap + 1].lpParam;
							if(*pMappingParameter == NULL)
							{
								return 0xFF;
							}

							/* 从映射参数中取出位数[size] */
							Size = (unsigned char)(*pMappingParameter & (unsigned int)0x000000FF);
              
							if(Size && ((offset + Size) <= (m->DLC << 3)))
							{
								CopyBits(Size, (unsigned char *)&m->Data[offset >> 3], offset % 8, 0, ((unsigned char *)tmp), 0, 0);

								ByteSize = (unsigned int)(1 + ((Size - 1) >> 3));
//								objDict = setODentry(d, (unsigned short)((*pMappingParameter) >> 16), (unsigned char)(((*pMappingParameter) >> 8) & 0xFF), tmp, &ByteSize, 0);//12.14us
                if(pRpdoDirectPar[numPdo].active != TRUE)
                  return 0xFF;
                objDict = RPDO_Direct_SetODentry(d,pRpdoDirectPar,numPdo,numMap,tmp,1);//8.57us         
								if(objDict != OD_SUCCESSFUL)
								{
									return 0xFF;
								}
								offset += Size;
							}
              else
              {
                /* PDO传输长度错误 */
//                Device_Error_Handler(DEVICE_ERR_SET,ERR_PDO_TRANS_LEN,A_AXIS);
                return 0xFF;
              }
							/* 映射值进行偏移，开始拷贝第二个数据 */
							numMap++;
						}
						
						/* RPDO事件定时事件 */
						if(d->RxPDO_EventTimers)
						{
							TIMEVAL EventTimerDuration = *(unsigned short *)d->objdict[offsetObjdict].pSubindex[5].lpParam;
							
							/* RPDO事件定时事件 */
							if(EventTimerDuration)
							{
								DelAlarm(d->RxPDO_EventTimers[numPdo]);
								d->RxPDO_EventTimers[numPdo] = SetAlarm(d, numPdo, d->RxPDO_EventTimers_Handler, MS_TO_TIMEVAL(EventTimerDuration), 0);
							}
						}
          }
//          TimeConsume.RPDO = ((TIM5->CNT - TimeCheck)>(TIMER5_PERIOD/2))?(TIM5->CNT - TimeCheck + TIMER5_PERIOD):(TIM5->CNT - TimeCheck);
       //   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
						return 0;
				}
			}
		}
	}
	/* pdo请求 */
  else if((*m).RTR == RTR_VALID)
	{
		
		status = state1;
		
		offsetObjdict = d->firstIndex->PDO_TRS;
		lastIndex = d->lastIndex->PDO_TRS;
		
		if(offsetObjdict)
		{
			/* 遍历所有的TPDO */
			while(offsetObjdict <= lastIndex)
			{
				/* 判断状态 */
				switch(status)
				{
					case state1:
          {
						pwCobId = (d->objdict + offsetObjdict)->pSubindex[1].lpParam;
						if(*pwCobId == UNS16_LE(m->StdId))
						{
							status = state4;
							break;
						}
						else
						{
							numPdo++;
							offsetObjdict++;
						}
						status = state1;
						break;
          }

					case state4:
          {
						pTransmissionType = (unsigned char *)d->objdict[offsetObjdict].pSubindex[2].lpParam;
						
						if(*pTransmissionType == ASYNC_TR_ONLY)
						{
							status = state5;
							break;
						}
						else if(*pTransmissionType == SYNC_RTR_ONLY)
						{
							if(d->PDO_status[numPdo].transmit_type_parameter & PDO_RTR_SYNC_READY)
							{
                CAN1_Send_Msg(d->PDO_status[numPdo].last_message.StdId,CAN_ID_STD,d->PDO_status[numPdo].last_message.RTR,d->PDO_status[numPdo].last_message.Data,d->PDO_status[numPdo].last_message.DLC);
								return 0;
							}
							else
							{
//								ATLAS_PRINT("Not ready RTR_SYNC TPDO send current data : %d\r\n", UNS16_LE(m->StdId));
								status = state5;
							}
							break;
						}
						else if((*pTransmissionType == ASYNC_TRIGGER_B) || (*pTransmissionType == ASYNC_TRIGGER_A))
						{
							d->PDO_status[numPdo].event_timer = DelAlarm(d->PDO_status[numPdo].event_timer);
							d->PDO_status[numPdo].inhibit_timer = DelAlarm(d->PDO_status[numPdo].inhibit_timer);
							d->PDO_status[numPdo].transmit_type_parameter &= ~PDO_INHIBITED;
							/* pdo事件定时事件 */
							PDOEventTimerAlarm(d, numPdo);
							return 0;
						}
						else
						{
//							ATLAS_PRINT("PDO is not to send on request : %d\r\n", UNS16_LE(m->StdId));
							return 0xFF;
						}
          }

					/* 构建pdo报文，并且发送 */
					case state5:
					{
						CanTxMsgTypeDef pdo;
						
						/* 构建pdo报文 */
						if(buildPDO(d, numPdo, &pdo))
						{
//							ATLAS_PRINT(" Couldn't build TPDO number : %d\r\n", numPdo);
							return 0xFF;
						}
            CAN1_Send_Msg(pdo.StdId,CAN_ID_STD,pdo.RTR,pdo.Data,pdo.DLC);
						return 0;
					}
				}
			}
		}
	}

  return 0;
}
/*------------------------------------------------
Function:将小端数据从源地址拷贝到目标地址
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
//CopyBits(Size, (unsigned char *)&m->Data[offset >> 3], offset % 8, 0, ((unsigned char *)tmp), 0, 0);
void CopyBits(unsigned char NbBits, unsigned char *SrcByteIndex, unsigned char SrcBitIndex, unsigned char SrcBigEndian, 
							unsigned char *DestByteIndex, unsigned char DestBitIndex, unsigned char DestBigEndian)
{
  while(NbBits > 0)
	{
		char Vect = DestBitIndex - SrcBitIndex;
		unsigned char Aligned = Vect > 0 ? *SrcByteIndex << Vect : *SrcByteIndex >> -Vect;
		unsigned char BoudaryLimit = (Vect > 0 ? 8 - DestBitIndex : 8 - SrcBitIndex);
		unsigned char BitsToCopy = BoudaryLimit > NbBits ? NbBits : BoudaryLimit;
		unsigned char Mask = ((0xff << (DestBitIndex + BitsToCopy)) | (0xff >> (8 - DestBitIndex)));
		unsigned char Filtered = Aligned & ~Mask;

		*DestByteIndex &= Mask;

		*DestByteIndex |= Filtered;

		if((SrcBitIndex += BitsToCopy) > 7)
		{
			SrcBitIndex = 0;
			SrcByteIndex += (SrcBigEndian ? -1 : 1);
		}

		if((DestBitIndex += BitsToCopy) > 7)
		{
			DestBitIndex = 0;
			DestByteIndex += (DestBigEndian ? -1 : 1);
		}

		NbBits -= BitsToCopy;
	}
}
/*------------------------------------------------
Function:发送pdo报文
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
static void sendPdo(CO_Data * d, unsigned int pdoNum, CanTxMsgTypeDef *pdo)
{

  d->PDO_status[pdoNum].last_message = *pdo;

//  ATLAS_PRINT ("sendPDO cobId :%d\r\n", UNS16_LE(pdo->StdId));
//  ATLAS_PRINT ("     Nb octets  : %d\r\n", pdo->DLC);

	/* 发送数据包 */
  CAN1_Send_Msg(pdo->StdId,CAN_ID_STD,pdo->RTR,pdo->Data,pdo->DLC);
}
/*------------------------------------------------
Function:启动异步特殊事件
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char sendPDOevent(CO_Data *d)
{
  return _sendPDOevent(d,pRpdoDirectPar,0);
}
/*------------------------------------------------
Function:启动指定通道的pdo事件
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char sendOnePDOevent(CO_Data *d, unsigned char pdoNum)
{
  unsigned short offsetObjdict;
  CanTxMsgTypeDef pdo;
	
  if(!d->CurrentCommunicationState.csPDO || (d->PDO_status[pdoNum].transmit_type_parameter & PDO_INHIBITED))
	{
		return 0;
	}

	/* 通讯pdo在字典中的下标 */
  offsetObjdict = (unsigned short)(d->firstIndex->PDO_TRS + pdoNum);

//  ATLAS_PRINT("  PDO is on EVENT. Trans type : %d\r\n", *((unsigned char *) d->objdict[offsetObjdict].pSubindex[2].lpParam));

	/* 将pdo数据清空 */
  memset(&pdo, 0, sizeof(pdo));

	/* 构建pdo报文 */
  if(buildPDO(d, pdoNum, &pdo))
	{
//		ATLAS_PRINT(" Couldn't build TPDO number : %d\r\n", pdoNum);
		return 0;
	}

  if(d->PDO_status[pdoNum].last_message.StdId == pdo.StdId && 
		 d->PDO_status[pdoNum].last_message.DLC == pdo.DLC && 
		 memcmp(d->PDO_status[pdoNum].last_message.Data, pdo.Data, 8) == 0)
	{
		return 0;
	}
  else
	{
		TIMEVAL EventTimerDuration;
		TIMEVAL InhibitTimerDuration;

//		ATLAS_PRINT("Changes TPDO number : %d\r\n", pdoNum);


		EventTimerDuration = *(unsigned short *)d->objdict[offsetObjdict].pSubindex[5].lpParam;

		InhibitTimerDuration = *(unsigned short *)d->objdict[offsetObjdict].pSubindex[3].lpParam;


		if(EventTimerDuration)
		{

			DelAlarm(d->PDO_status[pdoNum].event_timer);

			d->PDO_status[pdoNum].event_timer = SetAlarm(d, pdoNum, &PDOEventTimerAlarm, US_TO_TIMEVAL(EventTimerDuration*10), 0);
		}

		/* 两次pdo事件最小禁止时间不为0 */
		if(InhibitTimerDuration)
		{

			DelAlarm(d->PDO_status[pdoNum].inhibit_timer);


			d->PDO_status[pdoNum].inhibit_timer = SetAlarm(d, pdoNum, &PDOInhibitTimerAlarm, US_TO_TIMEVAL(InhibitTimerDuration), 0);

			d->PDO_status[pdoNum].transmit_type_parameter |= PDO_INHIBITED;
		}

		/* 发送pdo报文 */
		sendPdo(d, pdoNum, &pdo);
	}
    
	return 1;
}
/*------------------------------------------------
Function:pdo定时事件回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void PDOEventTimerAlarm(CO_Data *d, unsigned int pdoNum)
{
  /* 将pdo定时事件句柄置空 */
  d->PDO_status[pdoNum].event_timer = TIMER_NONE;
  /* cob_id置0 */
  
  //此处将last_message.cob_id清除是为了防止下一次bulid报文message后和last_message相同而不进行第二次发送
  d->PDO_status[pdoNum].last_message.StdId = 0;
	

  sendOnePDOevent(d, (unsigned char)pdoNum);
}
/*------------------------------------------------
Function:pdo禁止定时事件回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void PDOInhibitTimerAlarm(CO_Data *d, unsigned int pdoNum)
{

  d->PDO_status[pdoNum].inhibit_timer = TIMER_NONE;

  d->PDO_status[pdoNum].transmit_type_parameter &= ~PDO_INHIBITED;
	
	/* 发送一个pdo事件 */
  sendOnePDOevent(d, (unsigned char)pdoNum);
}
/*------------------------------------------------
Function:接收pdo事件定时事件，在接收到pdo响应之后，定时调用该函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void _RxPDO_EventTimers_Handler(CO_Data *d, unsigned int pdoNum)
{
}
/*------------------------------------------------
Function:启动所有需要触发的PDO事件  isSyncEvent：1-同步事件 2-异步事件
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char _sendPDOevent(CO_Data *d,PDO_DrctTrans_Struct *DrctPdo, unsigned char isSyncEvent)
{
  unsigned char pdoNum = 0x00;
  unsigned char *pTransmissionType = NULL;
  unsigned char status = state3;

  unsigned short offsetObjdict = d->firstIndex->PDO_TRS;

  unsigned short offsetObjdictMap = d->firstIndex->PDO_TRS_MAP;

  unsigned short lastIndex = d->lastIndex->PDO_TRS;
  

  unsigned short offsetRPOD = d->firstIndex->PDO_RCV;

  unsigned short offsetRPODlast = d->lastIndex->PDO_RCV;

  unsigned short offsetRPODMap = d->firstIndex->PDO_RCV_MAP;

  unsigned short offsetRPODMaplast = d->lastIndex->PDO_RCV_MAP;
  
  unsigned char *pMappingCount = NULL;
  unsigned int *pMappingParameter = NULL;
  unsigned char numMap;
  unsigned char Size;
  unsigned char offset;
  unsigned int objDict;

	/* pdo报文使能 */
  if(!d->CurrentCommunicationState.csPDO)
	{
		return 0;
	}

  if(offsetObjdict)
	{
		CanTxMsgTypeDef pdo;
		
		memset(&pdo, 0, sizeof(pdo));
		
		while(offsetObjdict <= lastIndex)
		{
			switch(status)
			{
				/* 状态3：构建pdo报文 */
				case state3:
        {
					if(*(unsigned int *)d->objdict[offsetObjdict].pSubindex[1].lpParam & 0x80000000)
					{
//						ATLAS_PRINT("Not a valid PDO %d\r\n", 0x1800 + pdoNum);
						status = state11;
						break;
					}

					pTransmissionType = (unsigned char *)d->objdict[offsetObjdict].pSubindex[2].lpParam;
//					ATLAS_PRINT("Reading PDO at index : %d\r\n", 0x1800 + pdoNum);

					if(isSyncEvent && (*pTransmissionType >= SYNC_CYCLE_BEGIN) && (*pTransmissionType <= SYNC_CYCLE_END) &&
						 (++d->PDO_status[pdoNum].transmit_type_parameter == *pTransmissionType))
					{
						d->PDO_status[pdoNum].transmit_type_parameter = 0;
						
//						ATLAS_PRINT("  PDO is on SYNCHRO. Trans type : %d\r\n", *pTransmissionType);
						
						/* 清空pdo报文 */
						memset(&pdo, 0, sizeof(pdo));
						
						/* 构建pdo报文 */
						if(buildPDO(d, pdoNum, &pdo))
						{
//							ATLAS_PRINT(" Couldn't build TPDO number : %d\r\n", pdoNum);
							status = state11;
							break;
						}
						status = state5;
					}
					else if(isSyncEvent && (*pTransmissionType == SYNC_RTR_ONLY))
					{

						if(buildPDO(d, pdoNum, &d->PDO_status[pdoNum].last_message))
						{
//							ATLAS_PRINT(" Couldn't build TPDO number : %d\r\n", pdoNum);
							d->PDO_status[pdoNum].transmit_type_parameter &= ~PDO_RTR_SYNC_READY;
						}
						/* 构建成功，将状态设置为准备好 */
						else
						{

							d->PDO_status[pdoNum].transmit_type_parameter |= PDO_RTR_SYNC_READY;
						}
						status = state11;
						break;
					}
					else
					{
						if((isSyncEvent && (*pTransmissionType == SYNC_NOCYCLE)) ||
							 /* 非同步模式 */
							 (!isSyncEvent &&
						    (*pTransmissionType == ASYNC_TRIGGER_B || *pTransmissionType == ASYNC_TRIGGER_A) && //一些特定事件：报文特殊，直接解除禁止并启动pdo事件
								!(d->PDO_status[pdoNum].transmit_type_parameter & PDO_INHIBITED)))	//不在禁止时间
						{
							/* 启动指定通道的pdo事件 */
							sendOnePDOevent(d, pdoNum);
							status = state11;
						}
						else
						{
//							ATLAS_PRINT("  PDO is not on EVENT or synchro or not at this SYNC. Trans type : %d\r\n", *pTransmissionType);
							status = state11;
						}
					}
        }
					break;
					
				/* 状态5：发送pdo报文 */
				case state5:
        {
					sendPdo(d, pdoNum, &pdo);
					status = state11;
        }
					break;
				
				/* 状态11：pdo操作结束继续下一个 */
				case state11:
        {
					pdoNum++;
					offsetObjdict++;
					offsetObjdictMap++;
//					ATLAS_PRINT("next pdo index : %d\r\n", pdoNum);
					status = state3;
        }
					break;

				default:
//					ATLAS_PRINT ("Unknown state has been reached :%d\r\n", status);
					return 0xFF;
			}
		}
	}
	
  return 0;
}
/*------------------------------------------------
Function:启动所有需要触发的RPDO事件  isSyncEvent：1-同步事件 2-异步事件
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned char _RPOD_SyncEvent(CO_Data *d,PDO_DrctTrans_Struct *DrctPdo, unsigned char isSyncEvent)
{
  unsigned char pdoNum = 0x00;
  unsigned char *pTransmissionType = NULL;
  unsigned char status = state3;
  unsigned char *pMappingCount = NULL;
  unsigned int *pMappingParameter = NULL;
  unsigned char numMap = 0;
  unsigned char Size = 0;
  unsigned char offset = 0;
  unsigned int objDict = 0;

  unsigned short offsetRPOD = d->firstIndex->PDO_RCV;

  unsigned short offsetRPODlast = d->lastIndex->PDO_RCV;
  /* 第一个RPDO映射参数在字典中的下标 */
  unsigned short offsetRPODMap = d->firstIndex->PDO_RCV_MAP;

  unsigned short offsetRPODMaplast = d->lastIndex->PDO_RCV_MAP;
  
  
  
  /* pdo报文使能 */
  if(!d->CurrentCommunicationState.csPDO)
	{
		return 0;
	}

  if(offsetRPOD)
  {
    while(offsetRPOD <= offsetRPODlast)
    {
      switch(status)
      {
        case state3:
        {
          /* cob-id必须合法，pdo配置字典时为bit31不能为1 */
					if(*(unsigned int *)d->objdict[offsetRPOD].pSubindex[1].lpParam & 0x80000000)
					{
  						status = state11;
						break;
					}
          /* 获取当前rpdo传输方式 */
					pTransmissionType = (unsigned char *)d->objdict[offsetRPOD].pSubindex[2].lpParam;

          if(isSyncEvent && ((DrctPdo + pdoNum)->TransStatus & PDD_REC_DATA_READY)\
            &&(((*pTransmissionType >= SYNC_CYCLE_BEGIN) && (*pTransmissionType <= SYNC_CYCLE_END)\
						  &&(++(DrctPdo + pdoNum)->transmit_type_parameter == *pTransmissionType))\
              ||(*pTransmissionType == SYNC_NOCYCLE))\
          )
          {

            (DrctPdo + pdoNum)->TransStatus &= ~PDD_REC_DATA_READY;
            (DrctPdo + pdoNum)->transmit_type_parameter = 0;

						pMappingCount = (unsigned char *)(d->objdict + offsetRPODMap + pdoNum)->pSubindex[0].lpParam;
						/* 遍历所有映射参数子索引 */
						numMap = 0;
            offset = 0;
						while(numMap < *pMappingCount)
						{
							unsigned char tmp[] = {0, 0, 0, 0, 0, 0, 0, 0};
							unsigned int ByteSize;
							

							pMappingParameter = (unsigned int *)(d->objdict + offsetRPODMap + pdoNum)->pSubindex[numMap + 1].lpParam;
 
							if(*pMappingParameter == NULL)
							{
								status = state11;
                break;
							}
							/* 从映射参数中取出位数[size] */
							Size = (unsigned char)(*pMappingParameter & (unsigned int)0x000000FF);
							if(Size && ((offset + Size) <= ((DrctPdo + pdoNum)->lastmessage.DLC << 3)))
							{

								CopyBits(Size, (unsigned char *)&(DrctPdo + pdoNum)->lastmessage.Data[offset >> 3], offset % 8, 0, ((unsigned char *)tmp), 0, 0);
								/* 将位数转换为字节数 */
								ByteSize = (unsigned int)(1 + ((Size - 1) >> 3));

                objDict = RPDO_Direct_SetODentry(d,pRpdoDirectPar,pdoNum,numMap,tmp,1);//8.57us         
								if(objDict != OD_SUCCESSFUL)
								{
									return 0xFF;
								}
								offset += Size;
							}
							/* 映射值进行偏移，开始拷贝第二个数据 */
							numMap++;
						}
          }
          status = state11;
            }
            break;
        /* 状态11：pdo操作结束继续下一个 */
				case state11:
        {
					pdoNum++;
					offsetRPOD++;
					status = state3;
            }
            break;
            default:
                break;
      }
    }
  }
}
/*------------------------------------------------
Function:pdo通信参数改变回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int TPDO_Communication_Parameter_Callback(CO_Data *d, const indextable *OD_entry, unsigned char bSubindex)
{
	/* 如果当前状态不支持pdo报文，则直接退出 */
  if(d->CurrentCommunicationState.csPDO)
	{
    switch(bSubindex)
		{

			case 2:

			case 3:

			case 5:
			{

				const indextable *TPDO_com = d->objdict + d->firstIndex->PDO_TRS;
        /* 获取当前TPDO NUM */
				unsigned char numPdo = (unsigned char)(OD_entry - TPDO_com);
				
				/* 重新初始化该通道的pdo事件 */
				d->PDO_status[numPdo].event_timer = DelAlarm(d->PDO_status[numPdo].event_timer);
				d->PDO_status[numPdo].inhibit_timer = DelAlarm(d->PDO_status[numPdo].inhibit_timer);
				d->PDO_status[numPdo].transmit_type_parameter = 0;
				PDOEventTimerAlarm(d, numPdo);

				return 0;
			}

			default:
				break;
		}
	}
  return 0;
}
/*------------------------------------------------
Function:PDO报文初始化
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void PDOInit(CO_Data *d)
{
	/* TPDO在字典中的索引 */
  unsigned short int pdoIndex = 0x1800;


  unsigned short int offsetObjdict = d->firstIndex->PDO_TRS;

  unsigned short int lastIndex = d->lastIndex->PDO_TRS;
 

	if(offsetObjdict)
	{
		while(offsetObjdict <= lastIndex)
		{
			unsigned int errorCode;
			ODCallback_t *CallbackList;

			/* 按索引查字典，获取回调函数指针列表 */
			OD_Index_san(d, pdoIndex, &errorCode, &CallbackList);
			if(errorCode == OD_SUCCESSFUL && CallbackList)
			{
        //TODO:回调函数已重定义
//				/* 传输类型改变，回调函数 */
//				CallbackList[2] = &TPDO_Communication_Parameter_Callback;
//				/* pdo最小禁止时间改变，回调函数 */
//				CallbackList[3] = &TPDO_Communication_Parameter_Callback;
//				/* 两次pdo最大间隔时间改变，回调函数 */
//				CallbackList[5] = &TPDO_Communication_Parameter_Callback;
			}
			
			/* 索引值加一 */
			pdoIndex++;
			/* TPDO在字典中的下标加一 */
			offsetObjdict++;
		}
	}

	RPDO_Direct_Trans_Init(d,pRpdoDirectPar);

  _sendPDOevent(d,pRpdoDirectPar,0);
}
/*------------------------------------------------
Function:RPDO通讯参数回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_RPDO_Communicate(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
  unsigned int InCob_id = 0;
  

  if(d->Node_state != HEART_STAT_PRE_OPERAT)
  {
    memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
    return OD_VALUE_RANGE_EXCEEDED;
  }
  switch(bSubindex)
  {
    /* 改变COB-ID */
    case 1:
    {
      InCob_id = *(unsigned int *)index->pSubindex[bSubindex].lpParam;
      /* COB-ID validity Check */
      if((InCob_id & SLAVE_NODE_NUM_MAX) != d->Node_ID)
      {
        memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
        return OD_VALUE_RANGE_EXCEEDED;
      }
    }
    break;
    /* 改变传输类型 */
    case 2:
        break;
    default:
        break;
  }
  
  return 0;
}
/*------------------------------------------------
Function:TPDO通讯参数回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_TPDO_Communicate(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
  unsigned int InCob_id = 0;
  
  if(d->Node_state != HEART_STAT_PRE_OPERAT)
  {
    memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
    return OD_VALUE_RANGE_EXCEEDED;
  }
  /* Handle Base SubIndex */
  switch(bSubindex)
  {
    /* 改变COB-ID */
    case 1:
    {
      InCob_id = *(unsigned int *)index->pSubindex[bSubindex].lpParam;
      /* COB-ID validity Check */
      if((InCob_id & SLAVE_NODE_NUM_MAX) != d->Node_ID)
      {
        memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
        return OD_VALUE_RANGE_EXCEEDED;
      }
    }
    break;

    case 2:
        break;
    case 3:
        break;
    case 5:
        break;
    default:
        break;
  }
  const indextable *TPDO_com = d->objdict + d->firstIndex->PDO_TRS;
  /* 获取当前TPDO NUM */
  unsigned char numPdo = (unsigned char)(index - TPDO_com);
  
  d->PDO_status[numPdo].event_timer = DelAlarm(d->PDO_status[numPdo].event_timer);
  d->PDO_status[numPdo].inhibit_timer = DelAlarm(d->PDO_status[numPdo].inhibit_timer);
  d->PDO_status[numPdo].transmit_type_parameter = 0;
  PDOEventTimerAlarm(d, numPdo);
  
  return 0;
}
/*------------------------------------------------
Function:TPDO映射参数入口参数数目子索引0改变回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_TPDO_MapSubindex0(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
  unsigned char offsetObjdict_MAP = 0;
  unsigned char InNum_Value = 0;
  unsigned char MapDataSize = 0;
  unsigned char offset = 0;
  unsigned char Size = 0;
  unsigned int *pMappingParameter = NULL;
  /* 第一个RPDO映射参数在字典中的下标 */
  unsigned short int offsetTPDOMAP = d->firstIndex->PDO_TRS_MAP;
  
  /* 拿到当前映射参数ID - quickIndex */
  switch(index->index)
  {
    case 0x1A00:
        offsetObjdict_MAP = 0;
        break;
    case 0x1A01:
        offsetObjdict_MAP = 1;
        break;
    case 0x1A02:
        offsetObjdict_MAP = 2;
        break;
    case 0x1A03:
        offsetObjdict_MAP = 3;
        break;
    default:
      return 0;
  }
  /* 非预操作态不可用 */
  if(d->Node_state != HEART_STAT_PRE_OPERAT)
  {
    memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
    return OD_VALUE_RANGE_EXCEEDED;
  }
  InNum_Value = *(unsigned char *)index->pSubindex[bSubindex].lpParam;
  if(InNum_Value != NULL)
  {
    while(offset<InNum_Value)
    {
      /* pdo映射参数子索引值 */
      pMappingParameter = (unsigned int *)(d->objdict + offsetTPDOMAP + offsetObjdict_MAP)->pSubindex[offset + 1].lpParam;
      if(*pMappingParameter == NULL)
      {
        return 0;
      }
			Size = (unsigned char)(*pMappingParameter & (unsigned int)0x000000FF);
      MapDataSize += Size;
      if(MapDataSize > 64)
      {
        memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
        return LENGTH_NUMBER_BEYOND_PDO_LEN;
      }
      offset++;
    }
  }
   
  return 0;
}
/*------------------------------------------------
Function:TPDO映射参数改变回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_TPDO_Map(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
  unsigned char offsetObjdict_MAP = 0;
  unsigned short MapIndex = 0;
  unsigned char MapSubindex = 0;
  unsigned int MapszData = 0;
  unsigned int ObjId = 0;
  unsigned char TpdoMapinNum = 0;
  unsigned int TpdoCobId = 0;
  unsigned short int offsetTPDOCmct = d->firstIndex->PDO_TRS;
  unsigned short int offsetTPDOMAP = d->firstIndex->PDO_TRS_MAP;
  
  
  switch(index->index)
  {
    case 0x1A00:
        offsetObjdict_MAP = 0;
        break;
    case 0x1A01:
        offsetObjdict_MAP = 1;
        break;
    case 0x1A02:
        offsetObjdict_MAP = 2;
        break;
    case 0x1A03:
        offsetObjdict_MAP = 3;
        break;
    default:
      return 0;
  }

  TpdoCobId = *(unsigned int *)d->objdict[offsetTPDOCmct+offsetObjdict_MAP].pSubindex[1].lpParam;
  TpdoMapinNum = *(unsigned int *)d->objdict[offsetTPDOMAP+offsetObjdict_MAP].pSubindex[0].lpParam;
  if(!(TpdoCobId & PDO_EXIST_BIT_CLR)\
    || TpdoMapinNum != NULL\
    ||d->Node_state != HEART_STAT_PRE_OPERAT)
  {
    /* 恢复映射参数 */
    memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
    return OD_VALUE_RANGE_EXCEEDED;
  }
  MapIndex = ((*(unsigned int *)index->pSubindex[bSubindex].lpParam)>>16)&0xFFFF;
  MapSubindex = ((*(unsigned int *)index->pSubindex[bSubindex].lpParam)>>8)&0xFF;
  MapszData = ((*(unsigned int *)index->pSubindex[bSubindex].lpParam)>>0)&0xFF;
  if((ObjDict_Get_Id(MapIndex,&ObjId) != OD_SUCCESSFUL) || (MapSubindex >= d->objdict[ObjId].bSubcount))
  {
    memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
    return OD_NO_SUCH_OBJECT;
  }
  #ifndef PDO_MAP_PERMISON
    /* PDO Map Check */
    if(d->objdict[ObjId].pSubindex[MapSubindex].pdoMap != PDO_ALLOW)
    {
      memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
      return OD_NOT_MAPPABLE;
    }
  #endif
  
	return 0;
}
/*------------------------------------------------
Function:RPDO映射参数入口参数数目子索引0改变回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_RPDO_MapSubindex0(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
  unsigned char offsetObjdict_MAP = 0;
  unsigned char InNum_Value = 0;
  unsigned char MapDataSize = 0;
  unsigned char offset = 0;
  unsigned char Size = 0;
  unsigned int *pMappingParameter = NULL;
  /* 第一个RPDO映射参数在字典中的下标 */
  unsigned short int offsetRPDOMAP = d->firstIndex->PDO_RCV_MAP;
  
  switch(index->index)
  {
    case 0x1600:
        offsetObjdict_MAP = 0;
        break;
    case 0x1601:
        offsetObjdict_MAP = 1;
        break;
    case 0x1602:
        offsetObjdict_MAP = 2;
        break;
    case 0x1603:
        offsetObjdict_MAP = 3;
        break;
    default:
      return 0;
  }

  if(d->Node_state != HEART_STAT_PRE_OPERAT)
  {
    memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
    return OD_VALUE_RANGE_EXCEEDED;
  }
  InNum_Value = *(unsigned char *)index->pSubindex[bSubindex].lpParam;

  if(InNum_Value != NULL)
  {
    while(offset<InNum_Value)
    {
      /* pdo映射参数子索引值 */
      pMappingParameter = (unsigned int *)(d->objdict + offsetRPDOMAP + offsetObjdict_MAP)->pSubindex[offset + 1].lpParam;
      if(*pMappingParameter == NULL)
      {
        return 0;
      }
      /* 从映射参数中取出位数[size] */
			Size = (unsigned char)(*pMappingParameter & (unsigned int)0x000000FF);
      MapDataSize += Size;

      if(MapDataSize > 64)
      {
        memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
        return LENGTH_NUMBER_BEYOND_PDO_LEN;
      }
      offset++;
    }
  }
   
  return 0;
}
/*------------------------------------------------
Function:RPDO映射参数改变回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int  ODCallback_t_RPDO_Map(CO_Data *d, const indextable *index, unsigned char bSubindex)
{
  unsigned char offsetObjdict_MAP = 0;
  unsigned short MapIndex = 0;
  unsigned char MapSubindex = 0;
  unsigned int MapszData = 0;
  unsigned int ObjId = 0;
  unsigned char RpdoMapinNum = 0;
  unsigned int RpdoCobId = 0;
  /* 第一个RPDO通信参数在字典中的下标 */
  unsigned short int offsetRPDOCmct = d->firstIndex->PDO_RCV;
  /* 第一个RPDO映射参数在字典中的下标 */
  unsigned short int offsetRPDOMAP = d->firstIndex->PDO_RCV_MAP;
  
  
  switch(index->index)
  {
    case 0x1600:
        offsetObjdict_MAP = 0;
        break;
    case 0x1601:
        offsetObjdict_MAP = 1;
        break;
    case 0x1602:
        offsetObjdict_MAP = 2;
        break;
    case 0x1603:
        offsetObjdict_MAP = 3;
        break;
    default:
      return 0;
  }

  RpdoCobId = *(unsigned int *)d->objdict[offsetRPDOCmct+offsetObjdict_MAP].pSubindex[1].lpParam;
  RpdoMapinNum = *(unsigned int *)d->objdict[offsetRPDOMAP+offsetObjdict_MAP].pSubindex[0].lpParam;
  if(!(RpdoCobId & PDO_EXIST_BIT_CLR)\
    || RpdoMapinNum != NULL\
    ||d->Node_state != HEART_STAT_PRE_OPERAT)
  {
    /* 恢复映射参数 */
    memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
    return OD_VALUE_RANGE_EXCEEDED;
  }

  MapIndex = ((*(unsigned int *)index->pSubindex[bSubindex].lpParam)>>16)&0xFFFF;
  MapSubindex = ((*(unsigned int *)index->pSubindex[bSubindex].lpParam)>>8)&0xFF;
  MapszData = ((*(unsigned int *)index->pSubindex[bSubindex].lpParam)>>0)&0xFF;
  if((ObjDict_Get_Id(MapIndex,&ObjId) != OD_SUCCESSFUL) || (MapSubindex >= d->objdict[ObjId].bSubcount))
  {
    pRpdoDirectPar[offsetObjdict_MAP].skip[bSubindex - 1] = 0;
    pRpdoDirectPar[offsetObjdict_MAP].Pointer[0][bSubindex - 1] = 0;
    pRpdoDirectPar[offsetObjdict_MAP].Pointer[1][bSubindex - 1] = 0;
    memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
    return OD_NO_SUCH_OBJECT;
  }
  #ifndef PDO_MAP_PERMISON
    /* PDO Map Check */
    if(d->objdict[ObjId].pSubindex[MapSubindex].pdoMap != PDO_ALLOW)
    {
      pRpdoDirectPar[offsetObjdict_MAP].skip[bSubindex - 1] = 0;
      pRpdoDirectPar[offsetObjdict_MAP].Pointer[0][bSubindex - 1] = 0;
      pRpdoDirectPar[offsetObjdict_MAP].Pointer[1][bSubindex - 1] = 0;
      memcpy(index->pSubindex[bSubindex].lpParam, d->LastObj.Data, d->LastObj.size);
      return OD_NOT_MAPPABLE;
    }
  #endif
  pRpdoDirectPar[offsetObjdict_MAP].skip[bSubindex - 1] = (unsigned char)(1 + ((MapszData - 1) >> 3));;
  pRpdoDirectPar[offsetObjdict_MAP].Pointer[0][bSubindex - 1] = ObjId;
  pRpdoDirectPar[offsetObjdict_MAP].Pointer[1][bSubindex - 1] = MapSubindex;
  
	return 0;
}
/*------------------------------------------------
Function:PDO直接传输初始化
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int RPDO_Direct_Trans_Init(CO_Data *d,PDO_DrctTrans_Struct *DrctPdo)
{
  unsigned char MapParaNum = 0;
  unsigned char MapParaNum_offset = 0;
  unsigned char DrctData_offset = 0;
  unsigned short Index = 0;
  unsigned char Subindex = 0;
  unsigned int szData = 0;
  unsigned int ObjId = 0;
  
  /* 第一个RPDO映射参数在字典中的下标 */
  unsigned short int offsetObjdict_MAP = d->firstIndex->PDO_RCV_MAP;
	/* 最后一个RPDO映射参数在字典中的下标 */
  unsigned short int lastIndex_MAP = d->lastIndex->PDO_RCV_MAP;
  
  if(offsetObjdict_MAP)
  {
    DrctData_offset = 0;
    while(offsetObjdict_MAP <= lastIndex_MAP)
    {
      MapParaNum  = *(unsigned char *)d->objdict[offsetObjdict_MAP].pSubindex[0].lpParam;
      (DrctPdo + DrctData_offset)->active = 1;
      /* 子索引为空 */
      if(MapParaNum == NULL)
      {
        offsetObjdict_MAP++;
        DrctData_offset++;
        continue;
      }
      MapParaNum_offset = 0;
      
      while(MapParaNum_offset < MapParaNum)
      {
        if(*(unsigned int *)d->objdict[offsetObjdict_MAP].pSubindex[MapParaNum_offset + 1].lpParam != NULL)
        {
          /* 获取Index & Subindex */
          Index = ((*(unsigned int *)d->objdict[offsetObjdict_MAP].pSubindex[MapParaNum_offset + 1].lpParam)>>16)&0xFFFF;
          Subindex = ((*(unsigned int *)d->objdict[offsetObjdict_MAP].pSubindex[MapParaNum_offset + 1].lpParam)>>8)&0xFF;
          szData = ((*(unsigned int *)d->objdict[offsetObjdict_MAP].pSubindex[MapParaNum_offset + 1].lpParam)>>0)&0xFF;
          /* 查字典 */
          if(ObjDict_Get_Id(Index,&ObjId) != OD_SUCCESSFUL)
          {
            return 0xFF;
          }
          if(Subindex >= d->objdict[ObjId].bSubcount)
          {
            return 0xFF;
          }
          #ifndef PDO_MAP_PERMISON
            /* PDO Map Check */
            if(d->objdict[ObjId].pSubindex[Subindex].pdoMap != PDO_ALLOW)
            {
              MapParaNum_offset++;
              continue;
            }
          #endif
          /* 得到该地址ID & Subindex */
          (DrctPdo + DrctData_offset)->Pointer[0][MapParaNum_offset] = ObjId;
          (DrctPdo + DrctData_offset)->Pointer[1][MapParaNum_offset] = Subindex;
          (DrctPdo + DrctData_offset)->skip[MapParaNum_offset] = (unsigned char)(1 + ((szData - 1) >> 3));
          MapParaNum_offset++;
        }
        else
        {
          MapParaNum_offset++;
        }          
      }
      DrctData_offset++;
      offsetObjdict_MAP++;
    }
  }
  
  return 0;
}
/*------------------------------------------------
Function:RPDO直接设置对象字典函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
unsigned int RPDO_Direct_SetODentry(CO_Data *d,PDO_DrctTrans_Struct *DrctPdo,unsigned int pdoNum,unsigned int rpdoSubindex,unsigned char *pSourceData,unsigned char checkAccess)
{
  unsigned int MapDataID = 0;
  unsigned int MapDataSubindex = 0;
  unsigned char MapDataszData = 0;
  
  /* This PDO not exist */
  if((DrctPdo + pdoNum)->active != 1)
  {
//    ATLAS_ERR("This RPDO not Exist .RPDONum:%d\r\n",pdoNum);
    return 0xFF;
  }
  MapDataID = (DrctPdo + pdoNum)->Pointer[0][rpdoSubindex];
  MapDataSubindex = (DrctPdo + pdoNum)->Pointer[1][rpdoSubindex];
  MapDataszData = (DrctPdo + pdoNum)->skip[rpdoSubindex];

  if(MapDataSubindex >= d->objdict[MapDataID].bSubcount)
  {
//    ATLAS_ERR("This RPDO' SubIndex not Exist .RPDONum:%d  SubIndex:%d\r\n",pdoNum,MapDataSubindex);
    return 0xFF;
  }
  if(checkAccess && (d->objdict[MapDataID].pSubindex[MapDataSubindex].accessType & RO)) 
  {
//    ATLAS_ERR("This RPDO'SubIndex Can't Write .RPDONum:%d\r\n",pdoNum);
    return 0xFF;
  }
  if(MapDataszData != d->objdict[MapDataID].pSubindex[MapDataSubindex].size)
  {
//    ATLAS_ERR("This RPDO'SubIndex Size Err .RPDONum:%d\r\n",pdoNum);
    return 0xFF;
  }
  memcpy(d->objdict[MapDataID].pSubindex[MapDataSubindex].lpParam, pSourceData, MapDataszData);
  
  return 0;
}
/*------------------------------------------------
Function:停止pdo报文
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void PDOStop(CO_Data *d)
{
  unsigned char pdoNum = 0x00;
	
  unsigned short int offsetObjdict = d->firstIndex->PDO_TRS;
  unsigned short int lastIndex = d->lastIndex->PDO_TRS;
	
  if(offsetObjdict)
	{
    while(offsetObjdict <= lastIndex)
		{
			d->PDO_status[pdoNum].event_timer = DelAlarm(d->PDO_status[pdoNum].event_timer);
			d->PDO_status[pdoNum].inhibit_timer = DelAlarm (d->PDO_status[pdoNum].inhibit_timer);
			d->PDO_status[pdoNum].transmit_type_parameter = 0;
			d->PDO_status[pdoNum].last_message.StdId = 0;
			/* pdo号加一 */
			pdoNum++;
			offsetObjdict++;
		}
	}
}
