#include "bsp_BDCMotor.h"
#include "main.h"
#include "function.h"
#include "Sensorless bldc.h"
void Sensorless_Start(void);
void SensorlessBLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty);
void SensorlessBLDC1_HALLSTUDY(u8 bHallState);
void Sensorless_ChangePwm(u8 Mortor_NUM);
unsigned long BEMF(void);
extern u16 PWM_Complementary_Switch_M1 ,PWM_Complementary_Switch_M2 ;
void Mul_change(u8 Motor_NUM);
s32 s_abs(s32 a);
u8 bHallStartStep=1;
int16_t VoltBEMF[13]={0};
u8 dir=0;
u8 ucMotorAD=0;
int32_t usOZTimeS=0;
int32_t usOZTimeS1=0;
u8 ClockDir=0;
int16_t BEMF_Cnt;
int16_t hallmin;
int sensorless[100];
s32 Ack_time[9]={0};           //过零点时间
s32 Record_time[9]={0};           //过零点时间
s16 Ack_Flag[9]={0};
s32 OverZerotime=0.0;
s32 DriveTime=0.0;
float RiseMul=1; //0.8   0.8
float DowmMul=1;  //1.3   1.2
#define M1      (uint8_t)(0x0)  /*!< Motor 1.*/
s32 s_abs(s32 a)
{
    s32 temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}



void Sensorless_ChangePwm(u8 Mortor_NUM)
{
    if(Sensorless[Mortor_NUM].PWM_DutySet > Sensorless[Mortor_NUM].PWM_Duty)
    {
        Sensorless[Mortor_NUM].PWM_Duty += Sensorless[Mortor_NUM].Acceleration;
        if(Sensorless[Mortor_NUM].PWM_DutySet < Sensorless[Mortor_NUM].PWM_Duty)
        {
            Sensorless[Mortor_NUM].PWM_Duty = Sensorless[Mortor_NUM].PWM_DutySet;
        }
    }
    else if(Sensorless[Mortor_NUM].PWM_DutySet< Sensorless[Mortor_NUM].PWM_Duty)
    {
        Sensorless[Mortor_NUM].PWM_Duty -= Sensorless[Mortor_NUM].Deceleration;
        if(Sensorless[Mortor_NUM].PWM_DutySet > Sensorless[Mortor_NUM].PWM_Duty)
        {
            Sensorless[Mortor_NUM].PWM_Duty = Sensorless[Mortor_NUM].PWM_DutySet;
        }
    }

}
/*无感换向函数*/
void SensorlessBLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
	  if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;
	Sensorless[0].PWMTicksPre = Sensorless[0].PWMTicks;
    Sensorless[0].FLAGBEMF=0;
    Sensorless[0].PWMTicks = 0;

   if(bHallState == 3)     //BC
    {
		Sensorless[0].Speed_Total_Cnt=Sensorless[0].Speed_Cnt;
		Sensorless[0].Speed_Cnt=0;
		Sensorless[0].Speed_Real=SPEED_COEFF/Sensorless[0].Speed_Total_Cnt;	
		TIM1->CCER = 0x1410;
		TIM1->CCR1 = PWM_Duty;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR2 = PWM_Duty;
    }

    else if(bHallState == 5)  //AB
    {
		TIM1->CCER = 0x1041;
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
        TIM1->CCR1 = PWM_Duty;
    }
    else if(bHallState == 2) //BA
    {
		TIM1->CCER =  0x1014;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
        TIM1->CCR2 = PWM_Duty;
    }
    else if(bHallState == 4) //CB
    {
		TIM1->CCER = 0x1140;
		TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
        TIM1->CCR3 = PWM_Duty;
    }
    else if(bHallState == 6) //CA
    {
		TIM1->CCER = 0x1104;
		TIM1->CCR2 = PWM_Duty;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
    }

    else if(bHallState == 1) //AC
    {

		TIM1->CCER = 0x1401;
		TIM1->CCR2 = PWM_Duty;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
    }
}



void SensorlessBLDC1_HALLSTUDY(u8 bHallState)
{
    switch (bHallState)//HPWMLpwm
    {
    case 0:
        break;

    case 6:
        TIM1->CCER = 0x1414;//  B->AC
	    TIM1->CCR1 = PWM_PERIOD;
	    TIM1->CCR3 = PWM_PERIOD;      
        TIM1->CCR2 = PWM_PERIOD;
	    TIM1->CCR2 = 4000;
        break;

    case 4:
        TIM1->CCER = 0x1114;//BC->A
	    TIM1->CCR1 = PWM_PERIOD;
	    TIM1->CCR3 = PWM_PERIOD;      
        TIM1->CCR2 = PWM_PERIOD;
	    TIM1->CCR2 = 4000;
	    TIM1->CCR3 = 4000;
		
        break;

    case 5:
        TIM1->CCER = 0x1144;//C->AB
	    TIM1->CCR1 = PWM_PERIOD;
	    TIM1->CCR3 = PWM_PERIOD;      
        TIM1->CCR2 = PWM_PERIOD;
	    TIM1->CCR3 = 4000;

        break;

    case 1:
        TIM1->CCER = 0x1141;//AC->B
	    TIM1->CCR1 = PWM_PERIOD;
	    TIM1->CCR3 = PWM_PERIOD;      
        TIM1->CCR2 = PWM_PERIOD;
	    TIM1->CCR3 = 4000;
	    TIM1->CCR1 = 4000;
        break;

    case 3:
        TIM1->CCER = 0x1441;//A->BC
	    TIM1->CCR1 = PWM_PERIOD;
	    TIM1->CCR3 = PWM_PERIOD;      
        TIM1->CCR2 = PWM_PERIOD;
	    TIM1->CCR1 = 4000;
        break;

    case 2:
        TIM1->CCER = 0x1411;//AB->C
	    TIM1->CCR1 = PWM_PERIOD;
	    TIM1->CCR3 = PWM_PERIOD;      
        TIM1->CCR2 = PWM_PERIOD;
	    TIM1->CCR1 = 4000;
	    TIM1->CCR2 = 4000;
        break;

    case 7:
        break;
    default:
        break;
    }
}

u8 change_temp = 0;
u8 Hall_Change_Temp=0;
u8 change_flag=0;

int delaymove=3;//2
int i=0;
int caixiangzhi=20;
s32 Sensorcnt=0;
s32 Sensorlesstime=140;
extern s32 Sensorlesstime;
extern s32 DriveTime;
int MeneFlag=0;
uint8_t HallchangeValue=0;
int8_t Sensorless_Time[7]={0};
int8_t Fileter_cnt[7]={0};
int Delay_ZeroTime=100;
int Zeta_Flag=0;
int Angel_Orient=0;
int Self_Start_Cnt=0;
int Three_Conduction_Count=0;
int Self_Start_Value=1;
int PWM_Change_Flag=0;
u16 Bemf_FailTime=0;
int bemf_increase=24;
/*无感驱动程序*/
void Sensorless_Start(void)                 //三步无感驱动。程序在main中定时器中断回调函数中运行
{
	 Sensorless[0].PWMTicks++;
	 Sensorless[0].Speed_Cnt++;
	switch (Sensorless[0].State)
	 {
          case 0:
			 if(++Sensorless[0].IdleCountCnt1 <1000)
			 {
				 SensorlessBLDC1_PhaseChange(1,1000);
				
			 }
			 else
			 {
				 Sensorless[0].State++;
				  Sensorless[0].IdleCountCnt1=0;
			 }
			 break;		 
			  
		 case 1:
		 if(Sensorless[0].CountSectorCnt < 50)
			{
				if(++Sensorless[0].CountSectorCnt2 >Sensorless[0].CountSectorCnt3 )
				{
					BEMF();
					Sensorless[0].CountSectorCnt3-=10;
					Sensorless[0].CountSectorCnt2 =0;
					if(Sensorless[0].CountSectorCnt3<Sensorlesstime)           //换向时间
					{
						Sensorless[0].CountSectorCnt3=Sensorlesstime-1;
					}		
					 Sensorless[0].SenlessHallSector ++;

					if(Sensorless[0].SenlessHallSector>6)
					{
						Sensorless[0].SenlessHallSector = 1;
						Sensorless_Time[1]=0;
						Sensorless_Time[2]=0;
						Sensorless_Time[3]=0;
						Sensorless_Time[4]=0;
						Sensorless_Time[5]=0;
						Sensorless_Time[6]=0;
					}		
					if(Sensorless[0].SenlessHallSector == 1) change_temp = 4;
					if(Sensorless[0].SenlessHallSector == 2) change_temp = 6;
					if(Sensorless[0].SenlessHallSector == 3) change_temp = 2;
					if(Sensorless[0].SenlessHallSector == 4) change_temp = 3;
					if(Sensorless[0].SenlessHallSector == 5) change_temp = 1;
					if(Sensorless[0].SenlessHallSector == 6) change_temp = 5;
     				Sensorless[0].CountSectorCnt++;
					SensorlessBLDC1_PhaseChange(change_temp,1000);  //5000对应37
					
				}
			}
				if(Sensorless[0].CountSectorCnt>=50)
			{
				Sensorless[0].State++;
				Sensorless[0].FirstFlag=1;
				Sensorless[0].CountSectorCnt=0;
			}
				break;	  
		
		 case 2:	
			  if(Sensorless[0].FLAGBEMF==0)
		    {  
              if(BEMF()==1)               //检测到过零点
				{
					RED_LED_TOGGLE;	
					Sensorless[0].FlagSwitchStep = Sensorless[0].Speed_Total_Cnt/bemf_increase ;///2;//>>1;     //延迟时间修改为一整个周期除6在除4
					Sensorless[0].FLAGBEMF = 1;  //检测到过零事件之后，不再检测
					Bemf_FailTime=0;
				}
			  else if(BEMF()==0) 	//100ms捕捉不到过零点就停止
			  {
				  Bemf_FailTime++;
				  if(Bemf_FailTime>=10000)
				  {
					  MotorState=3;
					  Sensorless[0].State=0;
					  Sensorless[0].PWM_Duty=1000;
					  Sensorless[0].CountSectorCnt3=250;
					  Sensorless[0].Speed_Set=0;
					  Bemf_FailTime=0;
					  Sensorless_Mototr_Stop();
				  }
			  }
			}
			else if(Sensorless[0].FLAGBEMF==1)   //检测到过零点
			  {
                  if(Sensorless[0].FlagSwitchStep==0)   //延时时间到
			     { 
				   Sensorless[M1].SenlessHallSector++;      //换向次数累加
				   if(Sensorless[M1].SenlessHallSector>6)
					{
						Sensorless[M1].SenlessHallSector = 1;
						Sensorless_Time[1]=0;                   //每次周期过零点标志位清0
						Sensorless_Time[2]=0;
						Sensorless_Time[3]=0;
						Sensorless_Time[4]=0;
						Sensorless_Time[5]=0;
						Sensorless_Time[6]=0;
					}				
					if(Sensorless[M1].SenlessHallSector == 1) change_temp = 4;
					if(Sensorless[M1].SenlessHallSector == 2) change_temp = 6;
					if(Sensorless[M1].SenlessHallSector == 3) change_temp = 2;
					if(Sensorless[M1].SenlessHallSector == 4) change_temp = 3;
					if(Sensorless[M1].SenlessHallSector == 5) change_temp = 1;
					if(Sensorless[M1].SenlessHallSector == 6) change_temp = 5;		
					Sensorless[M1].HallState = change_temp;          //当前Hall状态
					Sensorless[M1].Turn_Into_Closed_Cnt++;
					if(Sensorless[0].Turn_Into_Closed_Cnt>=200)
					{
						Sensorless[0].Turn_Into_Flag=1;
					}
                                  			
					SensorlessBLDC1_PhaseChange(change_temp,Sensorless[0].PWM_Duty);	
//                    RED_LED_TOGGLE;					
                    Sensorless[0].FLAGBEMF=0;        //过零点标志清0   				
					Sensorless[0].FirstFlag=2;		//调试用			
					sensorless[i]=change_temp;    //记录每次换向
			      }
				  else
			      {
				     Sensorless[0].FlagSwitchStep--;        //延迟
			      }			 
		    }
			 
			  break;
		default:
			break;
	}
}
int16_t VoltBEMF1[3]={0};
int16_t VoltBEMF2[3]={0};
int16_t VoltBEMF3[3]={0};
int16_t VoltBEMF4[3]={0};
int16_t VoltBEMF5[3]={0};
int16_t Ack_Max;
int16_t Ack_Min;
int16_t Ack_Sum;

int16_t Sensorless_Time1=0;
int16_t Change_cnt=0;

//int8_t Ack_Flag[7]={0};
extern s32 OverZerotime;


unsigned long BEMF(void)      //程序在main中定时器中断回调函数中运行
{
	int cnt;
	Mul_change(0);
	Bemf_Delay(0);
	switch(change_temp)//正转  645132  change_temp      MotorControl[5].Hall.HallState_CCW
	{
		case 2:    //BA
			 VoltBEMF[1]=Sensorless[0].PhaseWCurrent;
		  if(Sensorless[0].PhaseWCurrent <=RiseMul*VoltVar.BUS/2&&Sensorless_Time[1]==0)
		  {
			  Fileter_cnt[1]++;    //计数
		  }
		  else if(Sensorless[0].PhaseWCurrent > RiseMul*VoltVar.BUS/2&&Sensorless_Time[1]==0)
		  {
			  if(Fileter_cnt[1]>0)
			  {
			     Fileter_cnt[1]--;
			  }
		  }
		  if(Fileter_cnt[1]==1)     //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[1]=1;
			  Fileter_cnt[1]=0;
//		      RED_LED_TOGGLE;
			  Record_time[6]=Ack_time[6];      //记录前一次换向的时间
			  Ack_time[6]=OverZerotime;       //记录此时的换向时间
			  OverZerotime=0;                //计数清0
			   Ack_Flag[1]=1;
			  return 1;
		  }
		  else
		  {
			  return 0;
		  }
		   break;
		case 4:  //CB
		   VoltBEMF[2]=Sensorless[0].PhaseUCurrent;
		  if(Sensorless[0].PhaseUCurrent <=RiseMul*VoltVar.BUS/2&&Sensorless_Time[2]==0)
		  {
			  Fileter_cnt[2]++;      //计数
		  }
		  else if(Sensorless[0].PhaseUCurrent > RiseMul*VoltVar.BUS/2&&Sensorless_Time[2]==0)
		  {
			    if(Fileter_cnt[2]>0)
			  {
			     Fileter_cnt[2]--;
			  }
//			  Fileter_cnt[2]--;
		  }
		  if(Fileter_cnt[2]==1)     //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[2]=1;
			  Fileter_cnt[2]=0;
//		      RED_LED_TOGGLE;
			  Record_time[2]=Ack_time[2];   //记录前一次换向的时间
			  Ack_time[2]=OverZerotime;        //记录此时的换向时间
			  OverZerotime=0;
			  Ack_Flag[3]=1;
			  return 1;
		  }
		    else
		  {
			  return 0;
		  }
		   break;
		case 6:  //CA
		   VoltBEMF[3]=Sensorless[0].PhaseVCurrent;
		  if(Sensorless[0].PhaseVCurrent >=DowmMul*VoltVar.BUS/2&&Sensorless_Time[3]==0)
		  {
			  Fileter_cnt[3]++;       //计数
		  }
		  else if(Sensorless[0].PhaseVCurrent < DowmMul*VoltVar.BUS/2&&Sensorless_Time[3]==0)
		  {
			    if(Fileter_cnt[3]>0)
			  {
			     Fileter_cnt[3]--;
			  }
//			  Fileter_cnt[3]--;
		  }
		   if(Fileter_cnt[3]==1)       //先后2次都达到母线电压的一半
		  {
			 Sensorless_Time[3]=1;
			    Fileter_cnt[3]=0;
//		      RED_LED_TOGGLE;
			  Record_time[1]=Ack_time[1];         //记录前一次的换向时间
			  Ack_time[1]=OverZerotime;           //记录此时的换向时间
			  OverZerotime=0;
			  Ack_Flag[2]=1;
			    return 1;
		  }
		    else
		  {
			  return 0;
		  }
		
		    dir=1;
		   break;
		case 1:  //AC
		   VoltBEMF[4]=Sensorless[0].PhaseVCurrent;
		  if(Sensorless[0].PhaseVCurrent <=RiseMul*VoltVar.BUS/2&&Sensorless_Time[4]==0)
		  {
			  Fileter_cnt[4]++;
		  }
		  else if(Sensorless[0].PhaseVCurrent > RiseMul*VoltVar.BUS/2&&Sensorless_Time[4]==0)
		  {
			    if(Fileter_cnt[4]>0)
			  {
			     Fileter_cnt[4]--;
			  }
//			  Fileter_cnt[4]--;
		  }
		   if(Fileter_cnt[4]==1)           //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[4]=1;
			  Fileter_cnt[4]=0;
//		      RED_LED_TOGGLE;
			  Record_time[4]=Ack_time[4];             //记录前一次的换向时间
			  Ack_time[4]=OverZerotime;                 //记录此时的换向时间
			  OverZerotime=0; 
			  Ack_Flag[5]=1;
			    return 1;
		  }
		    else
		  {
			  return 0;
		  }
		   break;
		case 3: //BC
		   VoltBEMF[5]=Sensorless[0].PhaseUCurrent;
		  if(Sensorless[0].PhaseUCurrent >=DowmMul*VoltVar.BUS/2&&Sensorless_Time[5]==0)
		  {
			  Fileter_cnt[5]++;
		  }
		  else if(Sensorless[0].PhaseUCurrent <DowmMul*VoltVar.BUS/2&&Sensorless_Time[5]==0)
		  {
			    if(Fileter_cnt[5]>0)
			  {
			     Fileter_cnt[5]--;
			  }
//			  Fileter_cnt[5]--;
		  }
		   if(Fileter_cnt[5]==1)            //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[5]=1;
			   Fileter_cnt[5]=0;
//		      RED_LED_TOGGLE;
			  Record_time[5]=Ack_time[5];           //记录前一次的换向时间
			  Ack_time[5]=OverZerotime;             //记录此时的换向时间
			  OverZerotime=0;
			  Ack_Flag[6]=1;
		        return 1;
		  }
		    else
		  {
			  return 0;
		  }
		   break;
		case 5:  //AB
		   VoltBEMF[6]=Sensorless[0].PhaseWCurrent;
		  if(Sensorless[0].PhaseWCurrent >= DowmMul*VoltVar.BUS/2&&Sensorless_Time[6]==0)
		  {
			  Fileter_cnt[6]++;
		  }
		  else if(Sensorless[0].PhaseWCurrent <DowmMul*VoltVar.BUS/2&&Sensorless_Time[6]==0)
		  {
			    if(Fileter_cnt[6]>0)
			  {
			     Fileter_cnt[6]--;
			  }
//			  Fileter_cnt[6]--;
		  }
		   if(Fileter_cnt[6]==1)        //先后2次都达到母线电压的一半
		  {
			  Sensorless_Time[6]=1;
			  Fileter_cnt[6]=0;
//		      RED_LED_TOGGLE;
			  Record_time[3]=Ack_time[3];           //记录前一次的换向时间
			  Ack_time[3]=OverZerotime;               //记录此时的换向时间
			  OverZerotime=0;
			  Ack_Flag[4]=1;
			  return 1;
		  }
		    else
		  {
			  return 0;
		  }
		   break;
		default:
			break;
	}
}
extern int16_t CPWM_Test;


/*修改过零点位置程序*/
//void Mul_change(u8 Motor_NUM)   //根据实际的电流波形，对过零点进行处理。
//{
////	if(Sensorless[Motor_NUM].PWM_Duty >= 6300)
////	{
////		if(Sensorless[0].Current.FilterValue >=400 )  //400
////		{
////		   RiseMul=0.75;
////		   DowmMul=1.2;
////		   delaymove=3;
//////			RiseMul=0.98;
//////		   DowmMul=0.8;
//////		   delaymove=3;
////		}
////		else
////		{
////			RiseMul=1;
////			DowmMul=1;
////			delaymove=2;
////		}
////    }
//	if(Sensorless[Motor_NUM].PWM_Duty >= 6000&& Sensorless[Motor_NUM].PWM_Duty < 6300)
//	{
//		if(Sensorless[0].Current.FilterValue >=500 )  //700
//		{
//		   RiseMul=0.8;
//		   DowmMul=1;
//		   delaymove=3;
////			RiseMul=0.98;
////		   DowmMul=0.8;
////		   delaymove=3;
//		}
//		else
//		{
//			RiseMul=1;
//			DowmMul=1;
//			delaymove=2;
//		}
//    }
//	  if(Sensorless[Motor_NUM].PWM_Duty < 6000 && Sensorless[Motor_NUM].PWM_Duty >= 4500)
//   {
//	   if(Sensorless[0].Current.FilterValue >=500 )
//	   {
//		   RiseMul=0.9;
//		   DowmMul=1;
//		   delaymove=3;
//	   }
//	   else
//	   {
//		   RiseMul=1;
//		   DowmMul=1;
//		   delaymove=2;
//	   }
//   }
//    if(Sensorless[Motor_NUM].PWM_Duty < 4500 && Sensorless[Motor_NUM].PWM_Duty >= 3500)
//   {
//	   if(Sensorless[0].Current.FilterValue >=500 )
//	   {
//		   RiseMul=0.9;
//		   DowmMul=1;
//		   delaymove=2;
//	   }
//	   else
//	   {
//		   RiseMul=1;
//		   DowmMul=1;
//		   delaymove=2;
//	   }
//   }
//	
//	
//   if(Sensorless[Motor_NUM].PWM_Duty < 3500 && Sensorless[Motor_NUM].PWM_Duty >=2500)
//    {
//	   if(Sensorless[0].Current.FilterValue >=500 )
//	   {
//		   RiseMul=1;
//		   DowmMul=0.9;
//		   delaymove=2;
//	   }
//	   else
//	   {
//		   RiseMul=1;
//		   DowmMul=1;
//		   delaymove=2;
//	   }
//    }
//	if(Sensorless[Motor_NUM].PWM_Duty >=2000 && Sensorless[Motor_NUM].PWM_Duty< 2500)
//    {
//		RiseMul=1;
//		DowmMul=1;	
//	}
//	else if(1350<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 2000)
//	{
//		RiseMul=0.9;
//		DowmMul=0.8;
//	}
//	 else if(1300<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1350)
//	{
//		RiseMul=1;
//		DowmMul=0.9;
//	}
//	 else if(1255<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1300)
//	{
//		RiseMul=1;
//		DowmMul=1;
//	}
//	 else if(1200<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1255)
//	{
//		RiseMul=1.05;
//		DowmMul=1;
//	}
//	 else if(1100<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1200)
//	{
//		RiseMul=1.135;
//		DowmMul=1.1;
//	}
//	 else if(1020<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1100)
//	{
//		RiseMul=1.2;
//		DowmMul=1.15;
//	}
//	else if(950<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1020)
//	{
//		RiseMul=1.25;//1.25
//		DowmMul=1.2;
//	}
//	else if(800<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 950)
//	{
//		RiseMul=1.4;
//		DowmMul=1;
//		
//	}
//	else if(600<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 800)
//	{
//		RiseMul=1.3;//1.25
//		DowmMul=1;
//	}
//	else if(500<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 600)
//	{
//		RiseMul=1.1;//1.25
//		DowmMul=1;
//	}
//}




//void Mul_change(u8 Motor_NUM)   //根据实际的电流波形，对过零点进行处理。另外一个方向转动
//{
//    if(3500<=Sensorless[Motor_NUM].PWM_Duty)
//	{
//		RiseMul=0.6;
//		DowmMul=1.4;
//	}
//	else if(3000<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 3500)
//	{
//		RiseMul=0.7;
//		DowmMul=1.3;
//	}
//	else if(2000<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=3000)
//	{
//		RiseMul=0.8;
//		DowmMul=1.3;
//	}
//	else if(1500<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=2000)
//	{
//		RiseMul=0.9;//1.25
//		DowmMul=1.1;
//	}
//	else if(1000<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1500)
//	{
//		RiseMul=1;//1.25
//		DowmMul=1;
//	}
//	else if(700<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1000)
//	{
//		RiseMul=1.05;//1.25
//		DowmMul=1;
//	}
//		else if(0<Sensorless[Motor_NUM].PWM_Duty &&Sensorless[Motor_NUM].PWM_Duty< 700)
//	{
//		RiseMul=1.1;//1.25
//		DowmMul=1;
//	}
//}

void Mul_change(u8 Motor_NUM)   //根据实际的电流波形，对过零点进行处理。
{
    if(3200<Sensorless[Motor_NUM].PWM_Duty)
	{
		RiseMul=1.4;
		DowmMul=0.65;
	}
	else if(2400<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=3200)
	{
		RiseMul=1.3;
		DowmMul=0.8;
	}
	else if(1700<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=2400)
	{
		RiseMul=1.15;
		DowmMul=0.9;
	}
	else if(1000<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<1700)
	{
		RiseMul=1;//1.25
		DowmMul=1.05;
	}
	else if(900<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty< 1000)
	{
		RiseMul=1;//1.25
		DowmMul=1.05;
	}
	else if(700<Sensorless[Motor_NUM].PWM_Duty &&Sensorless[Motor_NUM].PWM_Duty<=900)
	{
		RiseMul=0.9;//1.25
		DowmMul=1.1;
	}
	else if(550<=Sensorless[Motor_NUM].PWM_Duty &&Sensorless[Motor_NUM].PWM_Duty<=700)
	{
		RiseMul=0.95;//1.25
		DowmMul=1.03;
	}
		else if(450<=Sensorless[Motor_NUM].PWM_Duty &&Sensorless[Motor_NUM].PWM_Duty<550)
	{
		RiseMul=0.8;//1.25
		DowmMul=1.1;
	}
}

void Bemf_Delay(u8 Motor_NUM) 
{
	if(2600<=Sensorless[Motor_NUM].PWM_Duty)
	{
		bemf_increase=20;
	}
//	if(3500<Sensorless[Motor_NUM].PWM_Duty)
//	{
//		bemf_increase=13;
//	}
//	else if(3200<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=3500)
//	{
//		bemf_increase=13;
//	}
//	else if(2800<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=3200)
//	{
//		bemf_increase=14;
//	}
//	else if(2600<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=2800)
//	{
//		bemf_increase=13;
//	}
	else if(2400<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<2600)
	{
		bemf_increase=17;//14
	}
	else if(2300<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=2400)
	{
		bemf_increase=22;//new add
	}
	else if(1800<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<2300)
	{
		bemf_increase=20;//15
	}
	else if(1700<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=1800)
	{
		bemf_increase=18;//13
	}
	else if(1400<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<1700)
	{
		bemf_increase=22;//17
	}
	else if(1200<Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=1400)
	{
		bemf_increase=17;//15
	}
	else if(1100<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<=1200)
	{
		bemf_increase=16; //14
	}
	else if(900<=Sensorless[Motor_NUM].PWM_Duty && Sensorless[Motor_NUM].PWM_Duty<1100)
	{
		bemf_increase=16; //14
	}
    else if(700<Sensorless[Motor_NUM].PWM_Duty &&Sensorless[Motor_NUM].PWM_Duty< 900)
	{
		bemf_increase=13;
	}	
     else if(0<Sensorless[Motor_NUM].PWM_Duty &&Sensorless[Motor_NUM].PWM_Duty<=700)
	{
		bemf_increase=11;
	}	
}

void Sensorless_Mototr_Stop(void)
{
	TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;	
    TIM1->CCER = CLOSE_ALL_TUBE;
	Sensorless_PID_PWM.wIntegral=0;
	PID_Current_InitStructure[0].wIntegral = 0;
}