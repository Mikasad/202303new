#include "canopen_emcy.h"

/*------------------------------------------------
Function:预定义错误域错误数变化回调函数
Input   :d  OD对象 子索引
Output  :No
Explain :No
------------------------------------------------*/
unsigned int OnNumberOfErrorsUpdate(CO_Data *d, const indextable *unsused_indextable, unsigned char unsused_bSubindex)
{
	unsigned char index;

	if(*d->error_number == 0)
	{
		for(index = 0; index < d->error_history_size; ++index)
			*(d->error_first_element + index) = 0;
	}
  /* Return SDO Abort code：06090030 */
	else
	{
		;
	}
	
  return 0;
}
/*------------------------------------------------
Function:紧急报文初始化配置
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void emergencyInit(CO_Data *d)
{
  RegisterSetOnDentry_Callback(d, 0x1003, 0x00, &OnNumberOfErrorsUpdate);

	/* 错误个数 */
  *d->error_number = 0;
}
/*------------------------------------------------
Function:紧急报文停止配置
Input   :d
Output  :No
Explain :No
------------------------------------------------*/
void emergencyStop(CO_Data* d)
{
  
}
/*------------------------------------------------
Function:设置新的错误
Input   :d  ERR_CODE  ERR_MASK ADDINFO
Output  :RESULT
Explain :No
------------------------------------------------*/
unsigned char EMCY_setError(CO_Data *d, unsigned short errCode, unsigned char errRegMask, unsigned short addInfo)
{
	unsigned char index;
	unsigned char errRegister_tmp;
  unsigned char Specific[sizeof(unsigned short)];
	
	for(index = 0; index < EMCY_MAX_ERRORS; ++index)
	{
		if(d->error_data[index].errCode == errCode)
		{
			if(d->error_data[index].active)
			{
//				ATLAS_PRINT("EMCY message already sent\r\n");
				return 0;
			}
			else
				d->error_data[index].active = 1;
			
			break;
		}
	}
	
	if(index == EMCY_MAX_ERRORS)
	{
		for(index = 0; index < EMCY_MAX_ERRORS; ++index) 
		{
			if(d->error_data[index].active == 0) 
				break;
		}
	}
	
	if(index == EMCY_MAX_ERRORS)
	{
//		ATLAS_ERR("error_data full\r\n");
		return 1;
	}
	
	d->error_data[index].errCode = errCode;
	d->error_data[index].errRegMask = errRegMask;
	d->error_data[index].active = 1;
	
	d->error_state = Error_occurred;

	for(index = 0, errRegister_tmp = 0; index < EMCY_MAX_ERRORS; ++index)
	{
		if(d->error_data[index].active == 1)
		{
			errRegister_tmp |= d->error_data[index].errRegMask;
		}
	}
	*d->error_register = errRegister_tmp;
	
	for(index = d->error_history_size - 1; index > 0; --index)
	{
		*(d->error_first_element + index) = *(d->error_first_element + index - 1);
	}
	*(d->error_first_element) = errCode | ((unsigned int)addInfo << 16);
  *Specific = (unsigned char)addInfo;
	if(*d->error_number < d->error_history_size) 
	{
		++(*d->error_number);
	}
	
	if(d->CurrentCommunicationState.csEmergency)
		return SendEmcy(d, errCode, *d->error_register, Specific, 1);
	else 
		return 1;
}

/*------------------------------------------------
Function:恢复一个错误
Input   :d  ERR_CODE  
Output  :No
Explain :No
------------------------------------------------*/
void EMCY_errorRecovered(CO_Data* d, unsigned short errCode)
{
	unsigned char index;
	unsigned char errRegister_tmp;
	unsigned char anyActiveError = 0;
	
	for(index = 0; index < EMCY_MAX_ERRORS; ++index)
	{
		if(d->error_data[index].errCode == errCode)
		{			
			break;
		}
	}
	
	if((index != EMCY_MAX_ERRORS) && (d->error_data[index].active == 1))
	{
		d->error_data[index].active = 0;
		
		for(index = 0, errRegister_tmp = 0; index < EMCY_MAX_ERRORS; ++index)
		{
			if(d->error_data[index].active == 1)
			{
				anyActiveError = 1;
				errRegister_tmp |= d->error_data[index].errRegMask;
			}
		}

		if(anyActiveError == 0)
		{
			d->error_state = Error_free;

			if(d->CurrentCommunicationState.csEmergency)
        EmcyMsg_TransSet(&EmcyTransData,0x0000,0x00,0x0000);
		}

		*d->error_register = errRegister_tmp;
	}
	else
  {}
//		ATLAS_ERR("Recovered error was not active\r\n");
}
/*------------------------------------------------
Function:Send Emcy message
Input   :d  ERR_CODE  ERR_REG  *Specific  SpecificLength
Output  :No
Explain :No
------------------------------------------------*/
unsigned char EmcyMsg_TransSet(Emcy_Trans_Data_status *e,unsigned short errCode,unsigned char errRegMask,unsigned short addInfo)
{
  e->errCode = errCode;
  e->errRegMask = errRegMask;
  e->addInfo = addInfo;
  /* 记录驱动器最近一次错误代码 */
  e->TransReq = TRANS_EN;
  return 0;
}
/*------------------------------------------------
Function:EmcyMsg_TransREQ
Input   :d  e
Output  :No
Explain :No
------------------------------------------------*/
unsigned char EmcyMsg_TransREQ(CO_Data *d,Emcy_Trans_Data_status *e)
{
  if(e->TransReq == TRANS_EN)
  {
    e->TransReq = TRANS_DIS;
    if(e->errCode == 0x00 \
      &&e->errRegMask == 0x00 \
      &&e->addInfo == 0x00 \
      )
    {
      /* Recovered Msg */
      SendEmcy(d, 0x0000, 0x00, NULL, 0);
    }
    /* Set New Error */
    else
      return EMCY_setError(d,e->errCode,e->errRegMask,e->addInfo);
  }
  
  return 0;
}

/*------------------------------------------------
Function:Send Emcy message
Input   :d  ERR_CODE  ERR_REG  *Specific  SpecificLength
Output  :No
Explain :No
------------------------------------------------*/
unsigned char SendEmcy(CO_Data *d,unsigned int short errCode,unsigned char errReg,const void *Specific, unsigned char SpecificLength)
{
  /*cob-id*/
  d->CanTx_Buffer->StdId = *d->error_cobid;
  /*data frame*/
  d->CanTx_Buffer->RTR = BIT_INVALID;
  /*errcode*/
  d->CanTx_Buffer->Data[0] = (errCode>>0) &0xFF;
  d->CanTx_Buffer->Data[1] = (errCode>>8) &0xFF;
  /*err maske*/
  d->CanTx_Buffer->Data[2] = errReg;
  /* Reserved */
  d->CanTx_Buffer->Data[3] = 0;
  if(Specific == NULL)
  {
    d->CanTx_Buffer->Data[4] = 0;
    d->CanTx_Buffer->Data[5] = 0;
    d->CanTx_Buffer->Data[6] = 0;
    d->CanTx_Buffer->Data[7] = 0;
    SpecificLength = 4;
  }
  else
  {
    if(SpecificLength > 4)
      SpecificLength = 4;
    memcpy(&d->CanTx_Buffer->Data[4], Specific, SpecificLength);	  
  }
  /* 数据包长度 */
  d->CanTx_Buffer->DLC = SpecificLength + 4;
  
  
  return CAN1_Send_Msg(d->CanTx_Buffer->StdId,CAN_ID_STD,d->CanTx_Buffer->RTR,d->CanTx_Buffer->Data,d->CanTx_Buffer->DLC);
}
