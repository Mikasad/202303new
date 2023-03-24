#ifndef __DriverCTRL_Com_C
#define __DriverCTRL_Com_C
/**
  ******************************************************************************
  * @file    DriverCTRL_Com.c 
  * @author  Liupeng 
  * @version V1.0.0
  * @date    3/30/2020
  * @brief   ¨ª¡§??D-¨°¨¦¡À¨¤?a??
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stdlib.h"
#include <stdint.h>
//#include "stm32f10x.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "DriverCTRL_Com.h"
#include "tcp_echoserver.h"
#include "mc_api.h"
#include "math.h"
#include "motorcontrol.h"

#define StraightSpeedMax 100
#define RotatingSpeedMax 35
//#define pi acos(-1)
//#define pi 3.1415926
//#define WheelDist    40    //403 mm 42.5   *10    460-35    ?¨¢?¨¤??¦Ì?2?¨ºy¨º?42.5 ¨º¦Ì?¨º¦Ì¡Â¨º?DT???a44
//#define WheelDia 17     //170mm  ??¡Á¨®?¡À??

#define LiftVelMax 180*10
#define LiftVelMin -180*10

volatile u8 u8LwipR_Buf[1024];
volatile u16 u16LwipR_BufId;
 u16 u16ComRead_BufId;
extern u16  LWIP_CNT_1ms ;
extern int32_t SetSpeedMotor1 ,SetSpeedMotor2 ,hDurationms1 ,hDurationms2 ;
extern STM_Handle_t STM[2];
extern uint16_t Countrer_1ms,Countrer1_1ms,Countrer2_1ms;
//s16 Speed_Right,Speed_Left;
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
double Speed_Right = 0;
double Speed_Left = 0;
double Spd[2];

double deltaX=1;
double deltaY=1;
double deltaZ=1; 
double XAxisSum = 1;
double YAxisSum = 1;
double ZAxisSum = 1;

float SpeedFactor = 100;

short  Axis_Pos[3]={1,1,1};

extern uint8_t MC_StartStopFlag ,MC_StartStopFlag1,PositionModeFlag1,PositionModeFlag2;
extern int32_t SetSpeedMotor1 ,SetSpeedMotor2 ,hDurationms1 ,hDurationms2 ,Countrer_1ms_CNT,CurrentPosition11,CurrentPosition22,hDurationms11 ,hDurationms22;

extern float CurrentPosition1,TargetPosition1,MoveDuration1;
extern float CurrentPosition2,TargetPosition2,MoveDuration2;
extern float SetPositionMotor1,SetPositionMotor2;

/*******************************************************************************
//--o¡¥¨ºy??3?¡êoCom_Init
//--o¡¥¨ºy1|?¨¹¡êo¨ª¡§D??¡ê?¨¦3?¨º??¡¥
//--o¡¥¨ºy2?¨ºy¡êo?T
//--¡¤¦Ì??¨ºy?Y¡êo?T
//--?????¦Ì?¡Â¡êo?T
*******************************************************************************/
void Com_Init (void)
{	
	SDCom.RXStep = Com_RXStep_StartByte1_Receive;	
	SDCom.CrcStateFlag = 0;
}

/***************************************************************************
CheckSum????D¡ê?¨¦o¨ª
*****************************************************************************/
u8 CheckSum(u8* buf,u8 num)//D¡ê?¨¦o¨ª
{
	int cnt=0;
	u16 sum=0;
	
	for(cnt=0;cnt<num;cnt++)
	{
		sum+=buf[cnt];
	}
	
	return (u8)sum;
}

/*******************************************************************************
//--o¡¥¨ºy??3?¡êoCom_SendData
//--o¡¥¨ºy1|?¨¹¡êo¨ª¡§D? ¡¤¡é?¨ª3¨¬D¨°
//--o¡¥¨ºy2?¨ºy¡êoInstruct: 		¡¤¡é?¨ª??¨¢?
//					DataPoint:	¨ºy?Y?o¡ä?????
//					Length:		¡¤¡é?¨ª¨ºy?Y3¡è?¨¨
//--¡¤¦Ì??¨ºy?Y¡êo?T
//--?????¦Ì?¡Â¡êo?T
*******************************************************************************/
void Com_SendData (Com_Instruct_TypeDef Instruct,u8 * DataPoint,u8 Length)
{
	u8 i;
	u8 CRCTmp=0;
	
	//??¨ºy?Y¨¬¨ª?¨®????¡Á??¨²D?3¨¦¨°???
	SDCom.LwipSendBuf[0] = (u8)COM_FRAME_Start_Byte1;
	SDCom.LwipSendBuf[1]=(u8)COM_FRAME_Start_Byte2;
	SDCom.LwipSendBuf[2]=(u8)COM_FRAME_DCU_Address;
	SDCom.LwipSendBuf[3]=Length;
	SDCom.LwipSendBuf[4]=Instruct;		
	if (Length>1)
	{
		memcpy((SDCom.LwipSendBuf+5),DataPoint,Length-1);
	}

	for(i=0; i<Length; i++)
	{
		CRCTmp = CRCTmp+SDCom.LwipSendBuf[i+4];
	}
	SDCom.LwipSendBuf[Length+4]=(u8)(CRCTmp);
	
	SDCom.LwipSendBuf[Length+5]=COM_FRAME_End_Byte1;
	SDCom.LwipSendBuf[Length+6]=COM_FRAME_End_Byte2;

//	TCP_Server_Send(SDCom.LwipSendBuf, Length+7);	

}


/*******************************************************************************
//--o¡¥¨ºy??3?¡êoCom_Pack_Process
//--o¡¥¨ºy1|?¨¹¡êo¡ä|¨¤¨ª¨°?¡ã¨¹¨ª¡§D?¨ºy?Y
//--o¡¥¨ºy2?¨ºy¡êo?T
//--¡¤¦Ì??¨ºy?Y¡êo?T
//--?????¦Ì?¡Â¡êo?T
*******************************************************************************/
u8 CrcTemp;

u16 cntTest = 0;

//int16_t PreSetSpeedMotor1 ,PreSetSpeedMotor2 ;

s16 TempSetSpeedMotor1[100],TempSetSpeedMotor2[100];
void Com_Pack_Process (void)
{
	u16 Temp1;
	u8 Temp2 = 0;
	u8 Length;
	static s8 val_x,val_y,val_z;
	u8 STMStateMotor1,STMStateMotor2;
	u16 CurrentFaultsMotor12,CurrentFaultsMotor1,CurrentFaultsMotor2;
  uint32_t tickstart = 0U;
	
	Temp2 = CheckSum(SDCom.DataLen_Com,SDCom.DataLength);
	CrcTemp = Temp2;

	if(SDCom.DataLen_Com[SDCom.DataLength] == Temp2)
	{
		SDCom.CrcStateFlag = 1;
		LWIP_CNT_1ms = 0;
	}
	else
	{
		SDCom.CrcStateFlag = 0;
		
		return;
	}

	switch (SDCom.DataLen_Com[0])
	{//??¨¢?¡¤???
//		case Com_Instruct_IDLE:
//			break;
		

		case Com_Instruct_CheckData:
			if(SDCom.DataLen_Com[1] == 0x01)
			{
			    deltaZ=0.001*(Speed_Right-Speed_Left)/WheelDist;       //
                deltaX=((Speed_Right+Speed_Left)/2)*cos(deltaZ)*0.001; //*0.001
                deltaY=((Speed_Right+Speed_Left)/2)*sin(deltaZ)*0.001;
        //        printf("Speed_Right-Speed_Left: %x Speed_Right+Speed_Left: %x - Speed_Right:%x  Speed_Left:%x \n",(unsigned int)(Speed_Right-Speed_Left),(unsigned int)(Speed_Right+Speed_Left),(unsigned int)Speed_Right,(unsigned int)Speed_Left);
        //        printf("deltaZ: %04x deltaX:%04x deltaY:%04x\n",(unsigned int)deltaZ,(unsigned int)deltaX,(unsigned int)deltaY);
                XAxisSum += deltaX*1000;           //  *2.27536462
                YAxisSum += deltaY*1000;
                ZAxisSum += (deltaZ*180/pi)*1000;  //   *1.2327121
//                printf("XAxisSum:%04x  YAxisSum:%04x  ZAxisSum:%04x\n",(unsigned int)XAxisSum,(unsigned int)YAxisSum,(unsigned int)ZAxisSum);
        
				Axis_Pos[0] = (short)XAxisSum;
				Axis_Pos[1] = (short)YAxisSum;
				Axis_Pos[2] = (short)ZAxisSum;
				
				SDCom.SendBufTemp[0]= (short)Axis_Pos[0]>>8;	//?¨¢x¡¤¡ä¨¤???¨°?	
				SDCom.SendBufTemp[1]= (short)Axis_Pos[0];	
				
				SDCom.SendBufTemp[2]= (short)Axis_Pos[1]>>8; //?¨¢y¡¤¡ä¨¤???¨°?
				SDCom.SendBufTemp[3]= (short)Axis_Pos[1];
				
				SDCom.SendBufTemp[4]= (short)Axis_Pos[2]>>8;  //?¨¢z¡¤¡ä¨¤????¨¨¡ê?0.01?¨¨
				SDCom.SendBufTemp[5] =(short)Axis_Pos[2];
				
				/*¦Ì¨²7¡Á??¨²¡êo0x01¡êo??¨¬¡À¦Ì??¨²?¡ê?¨¦¨ª¡§D?¡ê?¨°¨¬3¡ê¡ê?
					¦Ì¨²8?¡é9¡Á??¨²¡êo
					¡ä¨ª?¨®¡¤¡ä¨¤?¡êo
					0x0001¡êo¡Á¨®¦Ì??¨²?y?¡¥¨°¨¬3¡ê¡ê?
					0x0002¡êo¡Á¨®¦Ì??¨²¡ä??¨²¨ª¡§??¨°¨¬3¡ê¡ê?
					0x0004¡êo¡Á¨®¦Ì??¨²¦Ì?¨¢¡Â¨°¨¬3¡ê¡ê?
					0x0008¡êo¡Á¨®¦Ì??¨²¡À¨¤???¡Â¨°¨¬3¡ê¡ê?

					0x0010¡êo¨®¨°¦Ì??¨²?y?¡¥¨°¨¬3¡ê¡ê?
					0x0020¡êo¨®¨°¦Ì??¨²¡ä??¨²¨ª¡§??¨°¨¬3¡ê¡ê?
					0x0040¡êo¨®¨°¦Ì??¨²¦Ì?¨¢¡Â¨°¨¬3¡ê¡ê?
					0x0080¡êo¨®¨°¦Ì??¨²¡À¨¤???¡Â¨°¨¬3¡ê */
				STMStateMotor1 = MC_GetSTMStateMotor1();
				STMStateMotor2 = MC_GetSTMStateMotor2();
				
				if(STMStateMotor1 == 6 && STMStateMotor2 == 6)
				{
				  SDCom.SendBufTemp[6] =  0x00 ;  //¨ª¡§D??y3¡ê
				}
				else
				{
				  SDCom.SendBufTemp[6] =  0x01 ;  //¨ª¡§D?¨°¨¬3¡ê
				}
				
				CurrentFaultsMotor1 = MC_GetCurrentFaultsMotor1();
				CurrentFaultsMotor2 = MC_GetCurrentFaultsMotor2();
				if(CurrentFaultsMotor1 == MC_BREAK_IN || CurrentFaultsMotor2 == MC_BREAK_IN)
				{
				  CurrentFaultsMotor12 |= 0x04;//1y¨¢¡Â MC_OVER_VOLT
				}
				else if(CurrentFaultsMotor1 == MC_OVER_VOLT || CurrentFaultsMotor2 == MC_OVER_VOLT)
				{
				  CurrentFaultsMotor12 |= 0x01;// MC_OVER_VOLT
				}
				else if(CurrentFaultsMotor1 == MC_UNDER_VOLT || CurrentFaultsMotor2 == MC_UNDER_VOLT)
				{
				  CurrentFaultsMotor12 |= 0x02;//MC_UNDER_VOLT
				}
				else if(CurrentFaultsMotor1 == MC_SPEED_FDBK || CurrentFaultsMotor2 == MC_SPEED_FDBK)
				{
				  CurrentFaultsMotor12 |= 0x08;//MC_SPEED_FDBK
				}
			
			   
				SDCom.SendBufTemp[7] =  CurrentFaultsMotor12 >>8; //¡Á¨®¨®¨°¦Ì??¨²¨°¨¬3¡ê¡ä¨²??
				SDCom.SendBufTemp[8] =  CurrentFaultsMotor12;
				
				SDCom.SendBufTemp[9] =  0; //¡À¡ê¨¢?	
				SDCom.SendBufTemp[10] = 0;
			

				Com_SendData (SDCom.DataLen_Com[0], SDCom.SendBufTemp, 12);
				
				return;
			
			}
			else if(SDCom.DataLen_Com[1] == 0x02)
			{
				MC_AcknowledgeFaultMotor1();
				MC_AcknowledgeFaultMotor2();
			    deltaZ=0.001*(Speed_Right-Speed_Left)/WheelDist;       //
                deltaX=((Speed_Right+Speed_Left)/2)*cos(deltaZ)*0.001; //*0.001
                deltaY=((Speed_Right+Speed_Left)/2)*sin(deltaZ)*0.001;
        //        printf("Speed_Right-Speed_Left: %x Speed_Right+Speed_Left: %x - Speed_Right:%x  Speed_Left:%x \n",(unsigned int)(Speed_Right-Speed_Left),(unsigned int)(Speed_Right+Speed_Left),(unsigned int)Speed_Right,(unsigned int)Speed_Left);
        //        printf("deltaZ: %04x deltaX:%04x deltaY:%04x\n",(unsigned int)deltaZ,(unsigned int)deltaX,(unsigned int)deltaY);
                XAxisSum += deltaX*1000;           //  *2.27536462
                YAxisSum += deltaY*1000;
                ZAxisSum += (deltaZ*180/pi)*1000;  //   *1.2327121  1rad¡ê¡§?¡ä1???¨¨¡ê?=|D??180?¨¨    1rad?¨¢¡ê¡§180??|D¡ê?=???¨¨
                //printf("XAxisSum:%04x  YAxisSum:%04x  ZAxisSum:%04x\n",(unsigned int)XAxisSum,(unsigned int)YAxisSum,(unsigned int)ZAxisSum)
        
				Axis_Pos[0] = (short)XAxisSum;
				Axis_Pos[1] = (short)YAxisSum;
				Axis_Pos[2] = (short)ZAxisSum;
				
				SDCom.SendBufTemp[0]= (short)Axis_Pos[0]>>8;	//?¨¢x¡¤¡ä¨¤???¨°?	
				SDCom.SendBufTemp[1]= (short)Axis_Pos[0];	
				
				SDCom.SendBufTemp[2]= (short)Axis_Pos[1]>>8; //?¨¢y¡¤¡ä¨¤???¨°?
				SDCom.SendBufTemp[3]= (short)Axis_Pos[1];
				
				SDCom.SendBufTemp[4]= (short)Axis_Pos[2]>>8;  //?¨¢z¡¤¡ä¨¤????¨¨¡ê?0.01?¨¨
				SDCom.SendBufTemp[5] =(short)Axis_Pos[2];
		
				/*¦Ì¨²7¡Á??¨²¡êo0x01¡êo??¨¬¡À¦Ì??¨²?¡ê?¨¦¨ª¡§D?¡ê?¨°¨¬3¡ê¡ê?
					¦Ì¨²8?¡é9¡Á??¨²¡êo
					¡ä¨ª?¨®¡¤¡ä¨¤?¡êo
					0x0001¡êo¡Á¨®¦Ì??¨²?y?¡¥¨°¨¬3¡ê¡ê?
					0x0002¡êo¡Á¨®¦Ì??¨²¡ä??¨²¨ª¡§??¨°¨¬3¡ê¡ê?
					0x0004¡êo¡Á¨®¦Ì??¨²¦Ì?¨¢¡Â¨°¨¬3¡ê¡ê?
					0x0008¡êo¡Á¨®¦Ì??¨²¡À¨¤???¡Â¨°¨¬3¡ê¡ê?

					0x0010¡êo¨®¨°¦Ì??¨²?y?¡¥¨°¨¬3¡ê¡ê?
					0x0020¡êo¨®¨°¦Ì??¨²¡ä??¨²¨ª¡§??¨°¨¬3¡ê¡ê?
					0x0040¡êo¨®¨°¦Ì??¨²¦Ì?¨¢¡Â¨°¨¬3¡ê¡ê?
					0x0080¡êo¨®¨°¦Ì??¨²¡À¨¤???¡Â¨°¨¬3¡ê */
				STMStateMotor1 = MC_GetSTMStateMotor1();
				STMStateMotor2 = MC_GetSTMStateMotor2();
				
				if(STMStateMotor1 == 6 && STMStateMotor2 == 6)
				{
				  SDCom.SendBufTemp[6] =  0x00 ;  //¨ª¡§D??y3¡ê
				}
				else
				{
				  SDCom.SendBufTemp[6] =  0x01 ;  //¨ª¡§D?¨°¨¬3¡ê
				}
				
				CurrentFaultsMotor1 = MC_GetCurrentFaultsMotor1();
				CurrentFaultsMotor2 = MC_GetCurrentFaultsMotor2();
				if(CurrentFaultsMotor1 == MC_BREAK_IN )
				{
				  CurrentFaultsMotor12 |= 0x04;//1y¨¢¡Â MC_OVER_VOLT
				}
				else if(CurrentFaultsMotor1 == MC_OVER_VOLT)
				{
				  CurrentFaultsMotor12 |= 0x01;// MC_OVER_VOLT
				}
				else if(CurrentFaultsMotor1 == MC_UNDER_VOLT)
				{
				  CurrentFaultsMotor12 |= 0x02;//MC_UNDER_VOLT
				}
				else if(CurrentFaultsMotor1 == MC_SPEED_FDBK )
				{
				  CurrentFaultsMotor12 |= 0x08;//MC_SPEED_FDBK
				}
				
				
				if( CurrentFaultsMotor2 == MC_BREAK_IN)
				{
				  CurrentFaultsMotor12 |= 0x40;//1y¨¢¡Â MC_OVER_VOLT
				}
				else if( CurrentFaultsMotor2 == MC_OVER_VOLT)
				{
				  CurrentFaultsMotor12 |= 0x10;// MC_OVER_VOLT
				}
				else if( CurrentFaultsMotor2 == MC_UNDER_VOLT)
				{
				  CurrentFaultsMotor12 |= 0x20;//MC_UNDER_VOLT
				}
				else if( CurrentFaultsMotor2 == MC_SPEED_FDBK)
				{
				  CurrentFaultsMotor12 |= 0x80;//MC_SPEED_FDBK
				}
				
				if(CurrentFaultsMotor1 == MC_NO_ERROR && CurrentFaultsMotor2 == MC_NO_ERROR)
				{
				  CurrentFaultsMotor12 = 0x00;//MC_NO_ERROR
				}
				
			
			   
				SDCom.SendBufTemp[7] =  CurrentFaultsMotor12 >>8; //¡Á¨®¨®¨°¦Ì??¨²¨°¨¬3¡ê¡ä¨²??
				SDCom.SendBufTemp[8] =  CurrentFaultsMotor12;
				
				SDCom.SendBufTemp[9] =  0; //¡À¡ê¨¢?	
				SDCom.SendBufTemp[10] = 0;
			

				Com_SendData (SDCom.DataLen_Com[0], SDCom.SendBufTemp, 12);
				
				return;
			
			}
			break;

		case Com_Instruct_CheckVersion:
			
			SDCom.SendBufTemp[0] = 	(u8)(Soft_Version>>16);		
			SDCom.SendBufTemp[1] =  (u8)(Soft_Version>>8);
			SDCom.SendBufTemp[2] = 	(u8)Soft_Version;		
			SDCom.SendBufTemp[3] =  (u8)(Hard_Version>>16);
			SDCom.SendBufTemp[4] =  (u8)(Hard_Version>>8);
			SDCom.SendBufTemp[5] =  (u8)Hard_Version;	

			Com_SendData (SDCom.DataLen_Com[0], SDCom.SendBufTemp, 7);
			return;


			break;

		case Com_Instruct_MotorCTRL:
			
//		    if(SDCom.DataLen_Com[1] == 0x00)
//			{
//			
//			}
//			else if(SDCom.DataLen_Com[1] == 0x01)
//			{
//			
//			}
//			else if(SDCom.DataLen_Com[1] == 0x02)
//			{
//			
//			}
//			__disable_irq();
			val_x  =  SDCom.DataLen_Com[2] ; //?¨¢X¦Ì??¨²¨ª????¡¥?¨´?¨¨ cm/s
			val_y  =  SDCom.DataLen_Com[3] ; //?¨¢Y¦Ì??¨²¨ª????¡¥?¨´?¨¨ cm/s
			val_z  =  SDCom.DataLen_Com[4] ; //?¨¢Z¦Ì??¨²¨ª?Dy¡Áa?¨´?¨¨ ??/s
			
//			Speed_Right =(double) (val_x + (val_z*pi/180)*WheelDist/2.0);//¨¦¨´¨¢?2¡À?
//			Speed_Left  =(double) (val_x - (val_z*pi/180)*WheelDist/2.0);
			
		    Speed_Right =(double) (val_x + (val_z*pi/180)*WheelDist/2.0);
			Speed_Left  =(double) (val_x - (val_z*pi/180)*WheelDist/2.0);

//		    Spd[0] =(double)(Speed_Left*60*20/(pi*WheelDia))*SpeedFactor ;     //         VelEval_xp[0] =Speed_Left*60*20/(pi*WheelDia) ;  25 = (10000/4000*10) 10000¨º?1¨¬???y?¡¥?¡Â¡Á?¡ä¨®??¡¤¡é??3?¨ºy4000 ¨º?¦Ì??¨²¡Á?¡ä¨®¡Áa?¨´  10 ¡À?¨º???3yD?¨ºy¦Ì?2.5
//			Spd[1] =(double)(Speed_Right*60*20/(pi*WheelDia))*SpeedFactor ;
			Spd[0] =(double)(Speed_Left/(pi*WheelDia))*SpeedFactor ;     //         VelEval_xp[0] =Speed_Left*60*20/(pi*WheelDia) ;  25 = (10000/4000*10) 10000¨º?1¨¬???y?¡¥?¡Â¡Á?¡ä¨®??¡¤¡é??3?¨ºy4000 ¨º?¦Ì??¨²¡Á?¡ä¨®¡Áa?¨´  10 ¡À?¨º???3yD?¨ºy¦Ì?2.5
			Spd[1] =(double)(Speed_Right/(pi*WheelDia))*SpeedFactor ;
			Spd[1] = -Spd[1];
			
//			__enable_irq();
			
			if(Spd[0]>0)
			{
			  SetSpeedMotor1 = (s16)(Spd[0]+0.5) ;     //         VelEval_xp[0] =Speed_Left*60*20/(pi*WheelDia) ;  25 = (10000/4000*10) 10000¨º?1¨¬???y?¡¥?¡Â¡Á?¡ä¨®??¡¤¡é??3?¨ºy4000 ¨º?¦Ì??¨²¡Á?¡ä¨®¡Áa?¨´  10 ¡À?¨º???3yD?¨ºy¦Ì?2.5
			}
			else
			{
			   SetSpeedMotor1 = (s16)(Spd[0]-0.5) ; 
			}
			
			if(Spd[1]>0)
			{
			  SetSpeedMotor2 = (s16)(Spd[1]+0.5) ;     //         VelEval_xp[0] =Speed_Left*60*20/(pi*WheelDia) ;  25 = (10000/4000*10) 10000¨º?1¨¬???y?¡¥?¡Â¡Á?¡ä¨®??¡¤¡é??3?¨ºy4000 ¨º?¦Ì??¨²¡Á?¡ä¨®¡Áa?¨´  10 ¡À?¨º???3yD?¨ºy¦Ì?2.5
			}
			else
			{
			   SetSpeedMotor2 = (s16)(Spd[1]-0.5) ; 
			}
		    
			if(SetSpeedMotor1>500)
			{
			  SetSpeedMotor1 = 500;
			}
			else if(SetSpeedMotor1<-500)
			{
			  SetSpeedMotor1 = -500;
			}
			
			if(SetSpeedMotor2>500)
			{
			  SetSpeedMotor2 = 500;
			}
			else if(SetSpeedMotor2<-500)
			{
			  SetSpeedMotor2 = -500;
			}
			MC_StartStopFlag = 6;
			if(SetSpeedMotor1 != 0)
			{
				MC_ProgramSpeedRampMotor1( SetSpeedMotor1, hDurationms1 );
				CurrentPosition1 =  MC_GetCurrentPosition1();
//				TC_FollowCommand(pMCI[M1]->pPosCtrl, CurrentPosition1);
				pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
				Countrer_1ms = 0;
				PositionModeFlag1 = 0;
			
			}
			else
			{
				if(Countrer_1ms >= Countrer_1ms_CNT)
				{
					PositionModeFlag1 = 1;
					Countrer_1ms = Countrer_1ms_CNT;
					pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;
				}
				else
				{
					if(Countrer_1ms <( Countrer_1ms_CNT-hDurationms1))
					{
						MC_ProgramSpeedRampMotor1( SetSpeedMotor1, hDurationms1 );
					}
					CurrentPosition1 =  MC_GetCurrentPosition1();
//					TC_FollowCommand(pMCI[M1]->pPosCtrl, CurrentPosition1);
				}
				
				if(PositionModeFlag1 == 1)
				{
					SetPositionMotor1 =  CurrentPosition1;
					MC_ProgramPositionCommandMotor1( SetPositionMotor1, hDurationms11 );						
					CurrentPosition11 =  MC_GetCurrentPosition1();
					TargetPosition1 =  MC_GetTargetPosition1();	
				}
			
			}
			
			if(SetSpeedMotor2 != 0)
			{
				MC_ProgramSpeedRampMotor2( SetSpeedMotor2, hDurationms2 );
				CurrentPosition2 =  MC_GetCurrentPosition2();
//				TC_FollowCommand(pMCI[M2]->pPosCtrl, CurrentPosition2);
				pMCI[M2]->pPosCtrl->PositionControlRegulation = 0;
				Countrer1_1ms = 0;
				PositionModeFlag2 = 0;
			
			}
			else
			{
				if(Countrer1_1ms >= Countrer_1ms_CNT)
				{
					PositionModeFlag2 = 1;
					Countrer1_1ms = Countrer_1ms_CNT;
					pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;
				}
				else
				{
					if(Countrer1_1ms <( Countrer_1ms_CNT-hDurationms2))
					{
						MC_ProgramSpeedRampMotor2( SetSpeedMotor2, hDurationms2 );
					}
					CurrentPosition2 =  MC_GetCurrentPosition2();
//					TC_FollowCommand(pMCI[M2]->pPosCtrl, CurrentPosition2);
				}
				
				if(PositionModeFlag2 == 1)
				{
					SetPositionMotor2 =  CurrentPosition2;
					MC_ProgramPositionCommandMotor2( SetPositionMotor2, hDurationms22 );						
					CurrentPosition22 =  MC_GetCurrentPosition2();
					TargetPosition2 =  MC_GetTargetPosition2();	
				}
			
			}
			
//			if(STM[0].bState != 6) 
//			{
//				MC_StartMotor1();
//				MC_ProgramSpeedRampMotor1( SetSpeedMotor1, 500 );	
//        PreSetSpeedMotor1 = SetSpeedMotor1;				
//			}
//			
//			if(STM[1].bState != 6) 
//			{
//				MC_StartMotor2();
//				MC_ProgramSpeedRampMotor2( SetSpeedMotor2, 500 );	
//				PreSetSpeedMotor2 = SetSpeedMotor2;
//			}
//			
//		 if(STM[0].bState == 6 && STM[1].bState == 6) 
//		 {
////			 	MC_ProgramSpeedRampMotor1( SetSpeedMotor1, 0 );
////			  PreSetSpeedMotor1 = SetSpeedMotor1;
//			
//			 if( PreSetSpeedMotor1 != SetSpeedMotor1)
//			 {	
////         cntTest ++;	
//         TempSetSpeedMotor1[cntTest]	= 	SDCom.DataLen_Com[2];		 
//		     PreSetSpeedMotor1 = SetSpeedMotor1;
//				 MC_ProgramSpeedRampMotor1( SetSpeedMotor1, 500 );	
//				 
////				 if(SetSpeedMotor1 == 0)
////				 {
////				    MC_ProgramSpeedRampMotor1( Spd[0], 0 );
////				 }
////				 else
////				 {
////				    MC_ProgramSpeedRampMotor1( Spd[0], 500 );	
////				 }
//			 }
//			 
//			 if( PreSetSpeedMotor2 != SetSpeedMotor2)
//			 {
////				 cntTest ++;
////				 TempSetSpeedMotor1[cntTest]	= 	Speed_Right;		
////				 TempSetSpeedMotor2[cntTest]	= 	Speed_Left;		
//				 MC_ProgramSpeedRampMotor2( SetSpeedMotor2, 500 );	 
//		     PreSetSpeedMotor2 = SetSpeedMotor2;
//				 
////         if(SetSpeedMotor2 == 0)
////				 {
////				    MC_ProgramSpeedRampMotor2( Spd[1], 500 );
////				 }
////				 else
////				 {
////				    MC_ProgramSpeedRampMotor2( Spd[1], 500 );	
////				 }
//				 cntTest ++;
//			 }
//        if(cntTest>99)cntTest=0;
//			}
			
			Com_SendData (SDCom.DataLen_Com[0], SDCom.SendBufTemp, 1);
			return;
			
			break;

		case Com_Instruct_StutterStop:
			
			SetSpeedMotor1 = 0;
		    SetSpeedMotor2 = 0;
		
			if(STM[0].bState == 6 && STM[1].bState == 6) 
			{
//				MC_ProgramSpeedRampMotor1( SetSpeedMotor1, 100 );
//				MC_ProgramSpeedRampMotor2( SetSpeedMotor2, 100 );
//				
//				PreSetSpeedMotor1 = SetSpeedMotor1;
//				PreSetSpeedMotor2 = SetSpeedMotor2;
//				MC_StopMotor1();
//				MC_StopMotor2();
//				
//				if( PreSetSpeedMotor1 != SetSpeedMotor1)
//				 {
//					 if(PreSetSpeedMotor1 != 0)
//					 {
////						 MC_ProgramSpeedRampMotor1( 0, 0 );	
//					 }
//					 
//					 cntTest ++;
//					 
//					 MC_StopMotor1();
//					 PreSetSpeedMotor1 = SetSpeedMotor1;
//					
//				 }
//				 
//				 if( PreSetSpeedMotor2 != SetSpeedMotor2)
//				 {			 	 
//					
//					 if(PreSetSpeedMotor2 != 0)
//					 {
////						 MC_ProgramSpeedRampMotor2( 0, 0 );	
//					 }
//					 
//					 MC_StopMotor2();
//					 PreSetSpeedMotor2 = SetSpeedMotor2;
//				 }
			 
			}
			
		
			
//			MC_ProgramSpeedRampMotor1( SetSpeedMotor1, 0 );
//			MC_ProgramSpeedRampMotor2( SetSpeedMotor2, 0 );
//		  MC_StopMotor1();
//			MC_StopMotor2();
//		
//		  tickstart = HAL_GetTick();
//			 do
//			{
//				/* Check for the Timeout */
//				if((HAL_GetTick() - tickstart ) > 500)
//				{
//					MC_StopMotor1();
//					MC_StopMotor2();
//				  Com_SendData (SDCom.DataLen_Com[0], SDCom.SendBufTemp, 1);
//					
//					return ;
//				}
//				
//			} while (1);
		 
			
		    Com_SendData (SDCom.DataLen_Com[0], SDCom.SendBufTemp, 1);
			return;
		
			break;
		
		case Com_Instruct_hDurationmsValue:
			
		        hDurationms1 = (int16_t)((int16_t)SDCom.DataLen_Com[1]*256 + SDCom.DataLen_Com[2]) ;  
		        hDurationms2 = (int16_t)((int16_t)SDCom.DataLen_Com[3]*256 + SDCom.DataLen_Com[4]) ;  
			
		        SDCom.SendBufTemp[0]= (short)hDurationms1/256;	//?¨¢x¡¤¡ä¨¤???¨°?	
				SDCom.SendBufTemp[1]= (short)hDurationms1%256;	
				
				SDCom.SendBufTemp[2]= (short)hDurationms2/256; //?¨¢y¡¤¡ä¨¤???¨°?
				SDCom.SendBufTemp[3]= (short)hDurationms2%256;
		
		        Com_SendData (SDCom.DataLen_Com[0], SDCom.SendBufTemp, 5);
			return;
		
			break;
//		case  Com_Instruct_ACK:
//			break;

		default:
			
			break;
	
	}
	
//	Com_SendData (SDCom.DataLen_Com[0], SDCom.SendBufTemp, 1);
	
}





/**
  * @brief  ??LWIP?¨®¨º?¦Ì?¨ºy?Y?e¡Á??¨²¡ä|¨¤¨ª
  * @param  None
  * @retval None
  */
void LWIP_Data_Process(void)
{
//	static u8 CRC_data4=0;
//	static u8 CRC_dataSum4 =0;
//	u8 writime=5;
	
  /* ?D???o3???¨º?¡¤??a???¨°¨°??¨² */
	if(u16LwipR_BufId != u16ComRead_BufId)
	{
		switch (SDCom.RXStep)
		{
			case Com_RXStep_StartByte1_Receive:
				if(COM_FRAME_Start_Byte1 == u8LwipR_Buf[u16ComRead_BufId])
				{//?a¨º?¡Á??¨²?£¤??
					SDCom.RXStep = Com_RXStep_StartByte2_Receive;
					u16ComRead_BufId++;
				}

			break;

			case Com_RXStep_StartByte2_Receive:
				if(u8LwipR_Buf[u16ComRead_BufId] == COM_FRAME_Start_Byte2)
				{//¡Á??¨²?£¤??3¨¦1|
					SDCom.RXStep = Com_RXStep_Address_Receive;
					u16ComRead_BufId++;
				}
				else
				{
					if (u8LwipR_Buf[u16ComRead_BufId] != COM_FRAME_Start_Byte1)
					{//?a¨º?¡Á??¨²?£¤??					
						SDCom.RXStep = Com_RXStep_StartByte1_Receive;
						u16ComRead_BufId++;
					}
				}

			break;

			case Com_RXStep_Address_Receive:
				if (u8LwipR_Buf[u16ComRead_BufId] == COM_FRAME_DriverCTRL_Address)
				{//¦Ì??¡¤?£¤??
					SDCom.RXStep = Com_RXStep_Length_Receive;
					u16ComRead_BufId++;
				}
				else
				{
					SDCom.RXStep = Com_RXStep_StartByte1_Receive;
					u16ComRead_BufId++;
				}

			break;

			case Com_RXStep_Length_Receive:
				//??¨¨?¨ºy?Y3¡è?¨¨??			
				if (u8LwipR_Buf[u16ComRead_BufId] > COM_Buff_MaxDataSize)
				{
					SDCom.DataLength = COM_Buff_MaxDataSize;
					u16ComRead_BufId++;
				}
				else
				{
					SDCom.DataLength = u8LwipR_Buf[u16ComRead_BufId];
					u16ComRead_BufId++;
				}

				SDCom.RXStep = Com_RXStep_Instruct_Receive;
				SDCom.DataCount=0;
								
			break;

			case Com_RXStep_Instruct_Receive:
				//?¨®¨º???¨¢???
				SDCom.DataLen_Com[SDCom.DataCount] = u8LwipR_Buf[u16ComRead_BufId];
				SDCom.DataCount++;
				if (SDCom.DataLength>1)
				{
					SDCom.RXStep = Com_RXStep_Data_Receive;	
					u16ComRead_BufId++;
				}
				else 
				{
					SDCom.RXStep = Com_RXStep_CRC_Receive;		
					u16ComRead_BufId++;
				}
				
			break;

			case Com_RXStep_Data_Receive:
				//?¨®¨º?¨ºy?YD??¡é
				SDCom.DataLen_Com[SDCom.DataCount] = u8LwipR_Buf[u16ComRead_BufId];
				SDCom.DataCount++;
				u16ComRead_BufId++;
				if (SDCom.DataCount >= SDCom.DataLength)
				{//¡Á?o¨®¨°???¨ºy?Y
					SDCom.RXStep = Com_RXStep_CRC_Receive;
				}
				
			break;

			case Com_RXStep_CRC_Receive:
				//?¨®¨º?CRC ??
				SDCom.DataLen_Com[SDCom.DataCount] = u8LwipR_Buf[u16ComRead_BufId];
				
				SDCom.RXStep = Com_RXStep_StartByte1_Receive;
			
				u16ComRead_BufId = 0;
			  u16LwipR_BufId = 0;
				Com_Pack_Process();
						
			break;

		}

//		u16ComRead_BufId++;
				
	}  
}


#endif   //__DriverCTRL_Com_C

