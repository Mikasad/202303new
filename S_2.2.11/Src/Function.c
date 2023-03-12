/************************************************************

************************************************************/

/*
***********************************************
* Include headfile
***********************************************
*/
#include "function.h"
#include "stm32f4xx_hal.h"
#include "bsp_BDCMotor.h"
#include "main.h"
#include "canopen_pdo.h"
#include "Agreement.h"
#include "flash.h"
/*
***********************************************
* Private Const Table
***********************************************
*/

VoltVar_TypeDef VoltVar;
u32 wGlobal_Flags;
u32 prewGlobal_Flags;
u32 wGlobal_Flags1;
u32 FaultOccurred = 0;
u32 FaultOccurred1 = 0;
uint32_t DisplayCounter = 0;
u16 BLDC_SPEED_COEEFFOCIENT=30;
DefaultPhaseCheekCurTemp Motor5_Current_temp;
DefaultPhaseCheekCurTemp Motor6_Current_temp;

/*------------------------------------------------
Function:MCU版本号
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver)
{
	pVersion->uVersionPartOfRobotType = robotype;				//项目类型	S线
	pVersion->uVersionPartOfMainVersion = main_ver;			//大版本号
	pVersion->uVersionFullVersion = fun_ver;						//功能版本号
	pVersion->uVersionPartOfSmallVersion = small_ver;		//小版本号，代码bug以及参数微调
	pVersion->uVersionFullVersion = (robotype<<24)+(main_ver<<16)+(fun_ver<<8)+small_ver;
}


/*------------------------------------------------
Function:硬件版本号
Input   :&MCU_Version ,funtype，vol，cur_max，update_ver
Output  :No
Explain :No
------------------------------------------------*/
s8 funtype,vol,cur_max,update_ver=0;
void HardVersion_Init(MCU_Version *pVersion)
{
	s8 funtype,vol,cur_max,update_ver=0;
	vol=0;
	  funtype=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13);
	cur_max=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12);
	update_ver=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11);
	if(update_ver==1&&cur_max==1&&funtype==0)
	{
		update_ver=4;
	}
	else
	{
		update_ver=2;
	}
	pVersion->HardwareFullVersion = ('H'<<24)+('A'<<16)+(15<<8)+update_ver;
}




	

/*------------------------------------------------
Function:上电自锁和下电解锁
Input   :
Output  :No
Explain :No

------------------------------------------------*/
void SelfLock_Ctl(void)
{
	if(HAL_GPIO_ReadPin(POWER_EN_GPIO_Port,POWER_EN_Pin)==SET)
	{
		SELF_LOCK;
	}
	else 
	{
		SELF_UNLOCK;
	}
}
/*------------------------------------------------
Function:电流的实际值转化 转化后1000-1A
Input   :No
Output  :No
Explain :No
------------------------------------------------*/

void Current_Real_Trans(void)
{
	u8 i;
	float cur_coefficient[9] = {1.23f,1.23f,1.23f,1.23f,1.23f,0.121f,0.121f,1.23f,0.121f};
	for(i=0;i<9;i++)
	{
		MotorControl[i].Current.Cur_Real = MotorControl[i].Current.FilterValue/cur_coefficient[i];
	}
}

/*------------------------------------------------
Function:电流保护限制
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void CurrentLimit(u8 Motor_NUM)
{
    if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue1)		/* 1阶段对于无刷暂时没用，会被PID调节回来 */
    {
//        MotorControl[Motor_NUM].Current.OFCnt1++;
//        if(MotorControl[Motor_NUM].Current.OFCnt1 > MotorControl[Motor_NUM].Current.OFCnt1_T)
//        {
//            if(MotorControl[Motor_NUM].PWM_Duty>0)
//            {
//                MotorControl[Motor_NUM].PWM_Duty -= 1;
//            }
//            else if(MotorControl[Motor_NUM].PWM_Duty<0)
//            {
//                MotorControl[Motor_NUM].PWM_Duty += 1;
//            }
//        }
        if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue2)
        {
            MotorControl[Motor_NUM].Current.OFCnt2++;
            if(MotorControl[Motor_NUM].Current.OFCnt2 > MotorControl[Motor_NUM].Current.OFCnt2_T)
            {
				if(Motor_NUM == 8)
				{
					VOTAGE_OFF;
					SetMotorStop(Motor_NUM);
					MC_SetFault(FAN_SHORT_CIRCUIT);
					MotorControl[Motor_NUM].Fault_Flag = 1;
					MotorControl[Motor_NUM].Current.OFCnt2 = 0;
				}
				else if(Motor_NUM == 7)    //原来的坑，过流判断弄成电机号左移1位，因此按照原来的话边刷过流会报成过压，因此单独分开设置
				{
					SetMotorStop(Motor_NUM);
					MC_SetFault(SIDE_BRUSH_OVER_CUR);
					MotorControl[Motor_NUM].Fault_Flag = 1;
					MotorControl[Motor_NUM].Current.OFCnt2 = 0;
				}
				else 
				{
					SetMotorStop(Motor_NUM);
					MC_SetFault(OVER_CURRENT(Motor_NUM));
					MotorControl[Motor_NUM].Fault_Flag = 1;
					MotorControl[Motor_NUM].Current.OFCnt2 = 0;
				}
            }
            if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue3)
            {
                MotorControl[Motor_NUM].Current.OFCnt3++;
                if(MotorControl[Motor_NUM].Current.OFCnt3 > MotorControl[Motor_NUM].Current.OFCnt3_T)
                {
                    SetMotorStop(Motor_NUM);
                    MC_SetFault(OVER_CURRENT(Motor_NUM));
                    MotorControl[Motor_NUM].Fault_Flag = 1;
                    MotorControl[Motor_NUM].Current.OFCnt3 = 0;
                }
                if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue4)
                {
                    MotorControl[Motor_NUM].Current.OFCnt4++;
                    if(MotorControl[Motor_NUM].Current.OFCnt4 > MotorControl[Motor_NUM].Current.OFCnt4_T)
                    {
                        SetMotorStop(Motor_NUM);
                        MC_SetFault(OVER_CURRENT(Motor_NUM));
                        MotorControl[Motor_NUM].Fault_Flag = 1;
                        MotorControl[Motor_NUM].Current.OFCnt4 = 0;
                    }
                }
                else if(MotorControl[Motor_NUM].Current.OFCnt4 > 0)
                {
                    MotorControl[Motor_NUM].Current.OFCnt4--;
                }
            }
            else if(MotorControl[Motor_NUM].Current.OFCnt3 > 0)
            {
                MotorControl[Motor_NUM].Current.OFCnt3--;
            }
        }
        else if(MotorControl[Motor_NUM].Current.OFCnt2 > 0)
        {
            MotorControl[Motor_NUM].Current.OFCnt2--;
        }
    }
    else if(MotorControl[Motor_NUM].Current.OFCnt1 > 0)
    {
        MotorControl[Motor_NUM].Current.OFCnt1--;
        if(MotorControl[Motor_NUM].Current.OFCnt2 > 0)
        {
            MotorControl[Motor_NUM].Current.OFCnt2--;
        }
        if(MotorControl[Motor_NUM].Current.OFCnt3 > 0)
        {
            MotorControl[Motor_NUM].Current.OFCnt3--;
        }
        if(MotorControl[Motor_NUM].Current.OFCnt4 > 0)
        {
            MotorControl[Motor_NUM].Current.OFCnt4--;
        }
    }

}

/*------------------------------------------------
Function:设置错误标志
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_SetFault(u32 hFault_type)
{
    wGlobal_Flags |= hFault_type;
	/*这一部分flash相关的数组移到main函数内*/

    if(FaultOccurred !=0)
    {
        FaultOccurred |=hFault_type;
        FaultOccurred &=hFault_type;
    }
    else
        FaultOccurred  |= hFault_type; //只报警当前错误类型
}
void MC_SetFault1(u32 hFault_type)
{
    wGlobal_Flags1 |= hFault_type;
    if(FaultOccurred1 !=0)
    {
        FaultOccurred1 |=hFault_type;
        FaultOccurred1 &=hFault_type;
    }
    else
        FaultOccurred1  |= hFault_type; //只报警当前错误类型
}
/*------------------------------------------------
Function:清除错误标志
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_ClearFault(u32 hFault_type)
{
    wGlobal_Flags &= ~hFault_type;
    sendPDOevent(&CANopen_Drive);
    FaultOccurred &= ~hFault_type; //清除当前错误类型

}
/*------------------------------------------------
Function:错误清除
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Motor_Fault_Clear(void)
{
		if(((wGlobal_Flags&(PUSH_MOTOR1_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL))==0)&&MotorControl[0].Fault_Flag == 1)//推杆0错误清除
		{
				MotorControl[0].Fault_Flag = 0;
		}
		if(((wGlobal_Flags&(PUSH_MOTOR2_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL))==0)&&MotorControl[1].Fault_Flag == 1)//推杆1错误清除
		{
				MotorControl[1].Fault_Flag = 0;
		}
		if(((wGlobal_Flags&(ONEWYA_MOTOR1_OVER_CUR))==0)&&MotorControl[3].Fault_Flag == 1)//电机3错误清除
		{
				MotorControl[3].Fault_Flag = 0;
		}
		if(((wGlobal_Flags&(ONEWYA_MOTOR2_OVER_CUR))==0)&&MotorControl[4].Fault_Flag == 1)//电机4错误清除
		{
				MotorControl[4].Fault_Flag = 0;
		}
    if(OverFlow_Cnt[0]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(BLDC1_OVER_CUR|HALL5_SENSOR_ERR|MOTOR5_PHASE_ERROR|MOTOR5_OVER_SPEED|MOTOR5_MISSING_PHASE|MOTOR5_BREAK))==0)&&MotorControl[5].Fault_Flag == 1)//电机5错误清除 无刷电机
        {
            MotorControl[5].Fault_Flag = 0;
        }
    }

//    if(OverFlow_Cnt[1]<BREAK_CNT_MAX)
//    {
//        if(((wGlobal_Flags&(BLDC2_OVER_CUR|HALL6_SENSOR_ERR|MOTOR6_PHASE_ERROR|MOTOR6_OVER_SPEED|MOTOR6_MISSING_PHASE|MOTOR6_BREAK))==0)&&MotorControl[6].Fault_Flag == 1)//电机6错误清除 无刷电机
//        {
//            MotorControl[6].Fault_Flag = 0;
//        }
//    }
    if(((wGlobal_Flags&SIDE_BRUSH_ERROR)==0)&&MotorControl[7].Fault_Flag == 1)//电机7错误清除 边刷
    {
        MotorControl[7].Fault_Flag = 0;
    }
    if(((wGlobal_Flags&FAN_ERROR)==0)&&MotorControl[8].Fault_Flag == 1)//电机8错误清除 风机
    {
        MotorControl[8].Fault_Flag = 0;
    }

}
/*----------------------------
*Fuction：绝对值函数
*Explain：类型：float
----------------------------*/
float f_abs(float a)
{
    float temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}
long l_abs(long a)
{
    long temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}

//堵转结构体
typedef struct OLCKED_ROTOR_Ttag
{
    long pStuckCurr;
    long pStuckTime;
    long pStuckVel;
} OLCKED_ROTOR_T;

void MotorStuckCheck(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
		if(i!=6&&i!=4)
		{
           CurrentLimit(i);
		}
    }

}
void Over_VoltageCheck(void)  //10ms
{
    if(VoltVar.BUS>VoltVar.BusHigh)
    {
        VoltVar.BusHighCnt++;
        if(VoltVar.BusHighCnt>VoltVar.HighGuardTime)
        {
            VoltVar.BusHighCnt   = VoltVar.HighGuardTime+50;
            for(u8 i=0; i < MOTOR_NUM; i++)
            {
                MotorControl[i].Motor_Start_Stop = DISABLE;
            }
            MC_SetFault(OVER_VOLTAGE);
        }
    }
    else if(VoltVar.BUS < VOLT_300V )   //OVR
    {
        if(VoltVar.BusHighCnt>0)
        {
            VoltVar.BusHighCnt--;
            if(VoltVar.BusHighCnt < VoltVar.HighGuardTime)
            {
                MC_ClearFault(OVER_VOLTAGE);
            }
        }
    }

    if(VoltVar.BUS<VoltVar.BusLow)
    {
        if(++VoltVar.BusLowCnt>VoltVar.LowGuardTime)
        {
            VoltVar.BusLowCnt = VoltVar.LowGuardTime+50;
            for(u8 i=0; i < MOTOR_NUM; i++)
            {
                MotorControl[i].Motor_Start_Stop = DISABLE;
            }
            MC_SetFault(UNDER_VOLTAGE);
        }
    }
    else if(VoltVar.BUS > VOLT_200V)
    {
        if(VoltVar.BusLowCnt>0) //UVR
        {
            VoltVar.BusLowCnt--;
            if(VoltVar.BusLowCnt < VoltVar.LowGuardTime)
            {
                MC_ClearFault(UNDER_VOLTAGE);
            }
        }
    }

}
/*------------------------------------------------
Function:参数初始化
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Init_Drive_Para(void)
{
    VoltVar.HighGuardTime  = 50;
    VoltVar.LowGuardTime  = 50;
    VoltVar.BusHigh = VOLT_360V ;
    VoltVar.BusLow =  VOLT_160V ;
		TIM4->CCR3= PWM_PERIOD+PWM_PeriodOFFSET;
		TIM4->CCR1 = PWM_PERIOD+PWM_PeriodOFFSET ;
		TIM4->CCR2 = PWM_PERIOD+PWM_PeriodOFFSET ;
		TIM11->CCR1 = 2*PWM_PERIOD+PWM_PeriodOFFSET ;
		TIM10->CCR1 = 2*PWM_PERIOD+PWM_PeriodOFFSET ;
/************************推杆初始化***************************/
		MotorControl[0].Frequency = 10000/(TIM2->PSC+1);
    MotorControl[0].Current.MaxValue1 = MOTOR0_CurrentValue(1.8) ;
    MotorControl[0].Current.MaxValue2 = MOTOR0_CurrentValue(2.5) ;
    MotorControl[0].Current.MaxValue3 = MOTOR0_CurrentValue(3) ;
    MotorControl[0].Current.MaxValue4 = MOTOR0_CurrentValue(3.5) ;
    MotorControl[0].Current.OFCnt1_T = 20000;
    MotorControl[0].Current.OFCnt2_T = 20000;
    MotorControl[0].Current.OFCnt3_T = 8000;
    MotorControl[0].Current.OFCnt4_T = 500;

		MotorControl[1].Frequency = 10000/(TIM9->PSC+1);
    MotorControl[1].Current.MaxValue1 = MOTOR1_CurrentValue(1) ;
    MotorControl[1].Current.MaxValue2 = MOTOR1_CurrentValue(1) ;
    MotorControl[1].Current.MaxValue3 = MOTOR1_CurrentValue(1) ;
    MotorControl[1].Current.MaxValue4 = MOTOR1_CurrentValue(1)	;
    MotorControl[1].Current.OFCnt1_T = 20000;
    MotorControl[1].Current.OFCnt2_T = 10000;
    MotorControl[1].Current.OFCnt3_T = 10000;
    MotorControl[1].Current.OFCnt4_T = 10000;

    MotorControl[2].Current.MaxValue1 = MOTOR2_CurrentValue(3) ;
    MotorControl[2].Current.MaxValue2 = MOTOR2_CurrentValue(4) ;
    MotorControl[2].Current.MaxValue3 = MOTOR2_CurrentValue(5) ;
    MotorControl[2].Current.MaxValue4 = MOTOR2_CurrentValue(6) ;
    MotorControl[2].Current.OFCnt1_T = 200;
    MotorControl[2].Current.OFCnt2_T = 20;
    MotorControl[2].Current.OFCnt3_T = 8;
    MotorControl[2].Current.OFCnt4_T = 5;

/*********************推杆初始化end***************************/

/*************************单向有刷3_4***************************/
		MotorControl[3].Frequency = 10000/(TIM10->PSC+1);
    MotorControl[3].Current.MaxValue1 = MOTOR3_CurrentValue(2.5) ;  //过滤电机
    MotorControl[3].Current.MaxValue2 = MOTOR3_CurrentValue(3) ;
    MotorControl[3].Current.MaxValue3 = MOTOR3_CurrentValue(3.5) ;  //过滤电机
    MotorControl[3].Current.MaxValue4 = MOTOR3_CurrentValue(4) ;
    MotorControl[3].Current.OFCnt1_T = 20000;
    MotorControl[3].Current.OFCnt2_T = 2000;
    MotorControl[3].Current.OFCnt3_T = 800;
    MotorControl[3].Current.OFCnt4_T = 200;

		MotorControl[4].Frequency = 10000/(TIM11->PSC+1);
    MotorControl[4].Current.MaxValue1 = MOTOR4_CurrentValue(3) ;  //喷水电机
    MotorControl[4].Current.MaxValue2 = MOTOR4_CurrentValue(3) ;
    MotorControl[4].Current.MaxValue3 = MOTOR4_CurrentValue(3.5) ;  //喷水电机
    MotorControl[4].Current.MaxValue4 = MOTOR4_CurrentValue(4) ;
    MotorControl[4].Current.OFCnt1_T = 20000;
    MotorControl[4].Current.OFCnt2_T = 2000;
    MotorControl[4].Current.OFCnt3_T = 800;
    MotorControl[4].Current.OFCnt4_T = 200;

/*************************单向有刷3_4 end***************************/

/*************************无刷5，6***************************/

		MotorControl[5].Frequency = 10000/(TIM1->PSC+1);
    MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(2) ;
    MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(8) ;
    MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(9) ;
    MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(10.5) ;
		MotorControl[5].Current.CurOffset = 30;		//默认0.3A
    MotorControl[5].Current.OFCnt1_T = 30000;
    MotorControl[5].Current.OFCnt2_T = 30000;
    MotorControl[5].Current.OFCnt3_T = 10000;
    MotorControl[5].Current.OFCnt4_T = 200;
    MotorControl[5].Pole_Paires = 4;
    MotorControl[5].Acceleration = 10;
    MotorControl[5].Deceleration = 10;
//		MotorControl[5].Motor_Start_Stop = ENABLE;
//		MotorControl[5].PercentOfPWM = 100;

		MotorControl[6].Frequency = 10000/(TIM8->PSC+1);
    MotorControl[6].Current.MaxValue1 = MOTOR6_CurrentValue(2) ;
    MotorControl[6].Current.MaxValue2 = MOTOR6_CurrentValue(5) ;
    MotorControl[6].Current.MaxValue3 = MOTOR6_CurrentValue(7) ;
    MotorControl[6].Current.MaxValue4 = MOTOR6_CurrentValue(8.5) ;
		MotorControl[6].Current.CurOffset = 30;		//默认0.3A
    MotorControl[6].Current.OFCnt1_T = 30000;
    MotorControl[6].Current.OFCnt2_T = 30000;
    MotorControl[6].Current.OFCnt3_T = 10000;
    MotorControl[6].Current.OFCnt4_T = 200;
    MotorControl[6].Pole_Paires = 4;
    MotorControl[6].Acceleration = 5;
    MotorControl[6].Deceleration = 5;
		
		MotorControl[7].Frequency = 10000/(TIM8->PSC+1);
		MotorControl[7].Current.MaxValue1 = MOTOR7_CurrentValue(1) ;  //边刷电机
    MotorControl[7].Current.MaxValue2 = MOTOR7_CurrentValue(1) ;
    MotorControl[7].Current.MaxValue3 = MOTOR7_CurrentValue(1) ;  //边刷电机
    MotorControl[7].Current.MaxValue4 = MOTOR7_CurrentValue(1) ;
    MotorControl[7].Current.OFCnt1_T = 10000;
    MotorControl[7].Current.OFCnt2_T = 10000;
    MotorControl[7].Current.OFCnt3_T = 10000;
    MotorControl[7].Current.OFCnt4_T = 10000;
		
		MotorControl[8].Frequency = 10000/(TIM4->PSC+1);
		MotorControl[8].Current.MaxValue1 = MOTOR8_CurrentValue(13) ;  //吸风电机
    MotorControl[8].Current.MaxValue2 = MOTOR8_CurrentValue(20) ;
    MotorControl[8].Current.MaxValue3 = MOTOR8_CurrentValue(20) ;  //吸风电机
    MotorControl[8].Current.MaxValue4 = MOTOR8_CurrentValue(20) ;
    MotorControl[8].Current.OFCnt1_T = 20000;
    MotorControl[8].Current.OFCnt2_T = 10;
    MotorControl[8].Current.OFCnt3_T = 20000;
    MotorControl[8].Current.OFCnt4_T = 20000;
	TIM4->CCR4= 0;
/*************************无刷5，6end***************************/
		PID_Init (&PID_Speed_InitStruct[0],&PID_Current_InitStructure[0]);		//电机5的PID
		PID_Init (&PID_Speed_InitStruct[1],&PID_Current_InitStructure[1]);		//电机6的PID
		PID_PWM_Init(&PID_PWM);																								//推杆0的PID
/*	加减速度初始化	*/
    MotorControl[0].Acceleration = 10;
    MotorControl[0].Deceleration = 10;
    MotorControl[0].LastMotorDirection = 2;
    MotorControl[0].Hall.HALL_CaptureValue = 10000; //用于上电标定
    MotorControl[1].Acceleration = 10;
    MotorControl[1].Deceleration = 10;
    MotorControl[1].LastMotorDirection = 2;
    MotorControl[1].Hall.HALL_CaptureValue = 10000; //用于上电标定
    MotorControl[2].Acceleration = 10;
    MotorControl[2].Deceleration = 10;
    MotorControl[2].LastMotorDirection = 2;
    MotorControl[2].Hall.HALL_CaptureValue = 10000; //用于上电标定
		MotorControl[0].Push_Location_model=0;
    MotorControl[1].Push_Location_model=1; //该推杆电机需要标定，标定成功才可以位置控制，其余电机不需要标定默认开启位置控制，只是伸出和收缩的控制没有位置控制
		MotorControl[0].Hall.HALL_CaptureValue = 10000; //用于上电标定    
		MotorControl[1].Hall.HALL_CaptureValue = 0; //由于推杆1需要位置控制，不标定不会打开Push_Location_model开关，所以在直接给定标定命令的时候当前值为0
 
		MotorControl[3].Acceleration = 3;
    MotorControl[3].Deceleration = 3;

    MotorControl[4].Acceleration = 3;
    MotorControl[4].Deceleration = 3;

    MotorControl[7].Acceleration = 10;
    MotorControl[7].Deceleration = 10;

    MotorControl[8].Acceleration = 10;
    MotorControl[8].Deceleration = 10;


    MotorControl[5].Hall.HallState = HALL_GetPhase1();
    MotorControl[6].Hall.HallState = HALL_GetPhase2();

/*           霍尔学习参数                */
    HALL_Study[1].StudySectorCnt3 = 200;/*换向一次时间1代表2ms*/
    HALL_Study[1].HallSector = 1;
    HALL_Study[1].StudySectorCnt = 0;
    HALL_Study[1].HallCommPWM = 4500;/*霍尔换向PWM值*/

    HALL_Study[0].StudySectorCnt3 = 200;/*换向一次时间1代表2ms*/
    HALL_Study[0].HallSector = 1;
    HALL_Study[0].StudySectorCnt = 0;
    HALL_Study[0].HallCommPWM = 4500;/*霍尔换向PWM值*/

    HALL_Study[0].HallTab[0] = 6;
    HALL_Study[0].HallTab[1] = 4;
    HALL_Study[0].HallTab[2] = 5;
    HALL_Study[0].HallTab[3] = 1;
    HALL_Study[0].HallTab[4] = 3;
    HALL_Study[0].HallTab[5] = 2;
		
		HALL_Study[0].HallTab[0] = 1;
    HALL_Study[0].HallTab[1] = 3;
    HALL_Study[0].HallTab[2] = 2;
    HALL_Study[0].HallTab[3] = 6;
    HALL_Study[0].HallTab[4] = 4;
    HALL_Study[0].HallTab[5] = 5;
		

    HALL_Study[1].HallTab[0] = 6;// 546231
    HALL_Study[1].HallTab[1] = 4;
    HALL_Study[1].HallTab[2] = 5;
    HALL_Study[1].HallTab[3] = 1;
    HALL_Study[1].HallTab[4] = 3;
    HALL_Study[1].HallTab[5] = 2;
		
		HALL_Study[1].HallTab[0] = 1;
    HALL_Study[1].HallTab[1] = 3;
    HALL_Study[1].HallTab[2] = 2;
    HALL_Study[1].HallTab[3] = 6;
    HALL_Study[1].HallTab[4] = 4;
    HALL_Study[1].HallTab[5] = 5;

		
}
/*------------------------------------------------
Function:led灯设置
Input   :led1: 1/下面红灯亮  0/下面红灯不亮
				 led2: 1/上面红灯亮  0/上面红灯不亮
				 led_time:闪烁的次数
				 led_mode:1/慢闪一次 0/快闪
				 led_speed:闪烁的速度
Output  :No
Explain :No
------------------------------------------------*/
void LEDSet(u8 led1,u8 led2,u8 led_time,u8 led_mode,u8 led_speed)
{
		//led1
		if(led1==0)
		RED_LED_OFF;
		else if(led_mode)	//慢闪
		{
			if(DisplayCounter<2*led_speed)
				RED_LED_ON;
			else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
				RED_LED_TOGGLE;
		}
		else if(DisplayCounter<=(2*led_time)*led_speed)	//快闪
		{
				if((DisplayCounter-1)%led_speed==0)
				{
						if((DisplayCounter/led_speed)%2==0)
						{
								RED_LED_ON;
						}
						else
								RED_LED_OFF;
				}
		}
		else if(DisplayCounter<2*(led_time+1)*led_speed)
		{
				RED_LED_OFF;
		}
//		
//		//led2
//		if(led2==0)
//		RED_LED2_OFF;
//		else if(led_mode)
//		{
//			if(DisplayCounter<2*led_speed)
//				RED_LED2_ON;
//			else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
//				RED_LED2_TOGGLE;
//		}
//		else if(DisplayCounter<=(2*led_time)*led_speed)
//		{
//				if((DisplayCounter-1)%led_speed==0)
//				{
//						if((DisplayCounter/led_speed)%2==0)
//						{
//								RED_LED2_ON;
//						}
//						else 
//								RED_LED2_OFF;
//				}
//		}
//		else if(DisplayCounter<2*(led_time+1)*led_speed)
//		{
//				RED_LED2_OFF;
//		}
//		
		//time clear
		if(led_mode)
		{
			if(DisplayCounter>(4*led_time+2)*led_speed)
				DisplayCounter=0;
		}
		else if(DisplayCounter>(2*led_time+2)*led_speed)
				DisplayCounter=0;
}
/*------------------------------------------------
Function:驱动器错误指示
Input   :No
Output  :No
Explain :10ms运行一次
------------------------------------------------*/

void DisplayErrLed(void)
{
    DisplayCounter++;
    if(wGlobal_Flags==NO_ERROR)
    {
				wGlobal_Flags1 =0;
				FaultOccurred1 = 0;
        FaultOccurred = wGlobal_Flags;
        DisplayCounter = 0;
				RED_LED_OFF;
    }
    else
    {
        switch (FaultOccurred)
        {
				case OVER_VOLTAGE:  //常亮
				case UNDER_VOLTAGE:   
						RED_LED_ON;
            DisplayCounter=0;
            break;
        case PUSH_MOTOR1_OVER_CUR: //推杆0闪烁1次
				case PUSH_BRAKE_LINE:
				case PUSH_LOST_HALL:
				case PUSHMOTOR_INT_ERROR:
						LEDSet(LED_ON,LED_OFF,1,0,LED_HIGH_SPEED);
            break;
				
				case PUSH_OVER_TIME:
        case PUSH_MOTOR2_OVER_CUR: //推杆1闪烁2次
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case ONEWYA_MOTOR1_OVER_CUR: //电机3闪烁3次
//				case MOTOR3_BRAKE_LINE:
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case ONEWYA_MOTOR2_OVER_CUR: //电机4闪烁4次
//				case MOTOR4_BRAKE_LINE:
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;
				
        case BLDC1_OVER_CUR: //BLDC1  5次
				case MOTOR5_PHASE_ERROR:
				case MOTOR5_BREAK:
				case MOTOR5_MISSING_PHASE:
				case MOTOR5_OVER_SPEED:
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;
				
//        case BLDC2_OVER_CUR: //BLDC2  6次
//				case MOTOR6_BREAK:
//				case MOTOR6_PHASE_ERROR:
//				case MOTOR6_MISSING_PHASE:
//				case MOTOR6_OVER_SPEED:
//            LEDSet(LED_ON,LED_OFF,6,0,LED_HIGH_SPEED);
//            break;
				
				case SIDE_BRUSH_ERROR:	//边刷7 次
						LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
					break;
				case FAN_ERROR:					//风机 8次
						LEDSet(LED_ON,LED_OFF,8,0,LED_HIGH_SPEED);
					break;

				
				case CAN_COMMUNICATION_ERR:	//慢闪1次
            LEDSet(LED_ON,LED_OFF,1,1,LED_LOW_SPEED);
            break;
        case HALL5_SENSOR_ERR:    //慢闪2次
						LEDSet(LED_ON,LED_OFF,2,1,LED_LOW_SPEED);		//慢闪

            break;

        case HALL6_SENSOR_ERR:		// 慢闪3次
						LEDSet(LED_ON,LED_OFF,3,1,LED_LOW_SPEED);
            break;
		

        default:
						RED_LED_ON;
            break;
        }
    }

}
/*------------------------------------------------
Function:推杆电机位置环
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Push_Motor_Location_Control(u8 motornum)
{
    if(MotorControl[motornum].Push_Location_model==1)//标定正确，开始执行位置控制
    {
        if(motornum == 0) //50机器滚刷推杆，后期预留滚刷自适应
        {
            if(MotorControl[5].Current.SetValue ==0)	//没有做电流自适应的情况，进行位置控制
            {
                if(MotorControl[motornum].Hall.HALL_CaptureValue - MotorControl[motornum].Location_Set >5)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
                {
                    MotorControl[motornum].PercentOfPWM = 95;
                }
                else if(MotorControl[motornum].Hall.HALL_CaptureValue - MotorControl[motornum].Location_Set < -5)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
                {
                    MotorControl[motornum].PercentOfPWM = -95;
                }
                else //||fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) < 5)
                {
                    MotorControl[motornum].PercentOfPWM = 0;
                }
            }
        }
        else
        {
            if(MotorControl[motornum].Hall.HALL_CaptureValue > MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
            {
                MotorControl[motornum].PercentOfPWM = MAX_PERCENT;
            }
            else if(MotorControl[motornum].Hall.HALL_CaptureValue < MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
            {
                MotorControl[motornum].PercentOfPWM = -MAX_PERCENT;
            }
            else if(MotorControl[motornum].Hall.HALL_CaptureValue == MotorControl[motornum].Location_Set)//||fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) < 5)
            {
                MotorControl[motornum].PercentOfPWM = 0;
            }
        }
    }
}
/*------------------------------------------------

Function:4路推杆电机霍尔检测和断联检测

Input   :No

Output  :No

Explain :No

------------------------------------------------*/
u8 NeedToCheekPush[MOTOR_NUM] = {0,3,0,0,0,0,0,0,0}; //如果该推杆电机需要霍尔错误或者推杆断联检测 0 1 2 9号电机默认都打开检测
u8 NeedToMissingHallPush[MOTOR_NUM] = {1,0,0,0,0,0,0,0}; //该数组来确保推杆是否需要霍尔保护
uint16_t PushMaxPulse[MOTOR_NUM] = {260,4000,4000,0,0,0,0,0}; //这个数组需要确定整机推杆的最大值
u32 Push_Error_Cheek_Cnt[MOTOR_NUM] = {0},Push_Error_Cheek_Cnt_1[MOTOR_NUM] = {0};
int Push_curr_temp[MOTOR_NUM] = {0},Push_curr_temp_1[MOTOR_NUM] = {0};
u8 CurChkArray[2]= {0};
#define PUSH_PROTECT_MIN_VAL 5999
#define PUSH_PROTECT_MIN_CUR  100
#define MOTOR34_PROTECT_MIN_CUR 50
#define MOTOR34_PROTECT_MIN_VAL 2999
void Push_Motor_Cheek(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        if(NeedToCheekPush[i]==1) //如果该推杆电机需要霍尔错误或者推杆断联检测 0 1 2 9号电机默
        {
            /*占空比大于5999才保护*/
             if(MotorControl[i].Motor_Start_Stop == 1 && (l_abs(MotorControl[i].Location_Set-MotorControl[i].Hall.HALL_CaptureValue))>20 && \
                    l_abs(MotorControl[i].PWM_Duty) > PUSH_PROTECT_MIN_VAL && MotorControl[i].Hall.HALL_CaptureValue < PushMaxPulse[i])
            {
							/*计数值和电流值累计应该放在电机运行时*/
                Push_Error_Cheek_Cnt[i]++;
                Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
								if(Push_Error_Cheek_Cnt[i]%2002000==0)
                {
                    Push_Error_Cheek_Cnt[i]++;
                }
                if(Push_Error_Cheek_Cnt[i]%1001==0) //100.1ms将捕获值更新一次，并进行一次断线检测
                {
                    MotorControl[i].Hall.HALL_CaptureValueDelta = MotorControl[i].Hall.HALL_CaptureValue;
                    if(Push_curr_temp[i] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
                    {
												Push_Error_Cheek_Cnt[i] = 0;
                        Push_curr_temp[i] = 0;
                        MotorControl[i].Motor_Start_Stop = DISABLE;   //断线错误
//											  MotorControl[i].Fault_Flag = 1;  //断线错误可以不置错误标志位，可以继续使能
                        if(i == 0)
                        {
                            MC_SetFault1(PUSH0_BRAKE_LINE);//具体哪一个电机错误查看 第二个32位全局错误 wGlobal_Flags1
                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
                        }
                        if(i == 1)
                        {
                            MC_SetFault1(PUSH1_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
                        }
                    }
										Push_curr_temp[i] = 0;
                }
                if(NeedToMissingHallPush[i]) //需要霍尔保护的推杆，默认都需要
                {
                    if((Push_Error_Cheek_Cnt[i]%2000==0)&&(MotorControl[i].Hall.HALL_CaptureValueDelta == \
											MotorControl[i].Hall.HALL_CaptureValue)&&Push_curr_temp[i] >= PUSH_PROTECT_MIN_CUR) //如果200ms内推杆的霍尔值没有改变，那就说明推杆霍尔异常
                    {
												Push_Error_Cheek_Cnt[i] = 0;
                        Push_curr_temp[i] = 0;
                        MotorControl[i].Motor_Start_Stop = DISABLE;   //霍尔错误
                        MotorControl[i].Fault_Flag = 1; //之后需要重新进行推杆标定，所以需要手动清除
												MotorControl[i].Push_motor_calibrationFLAG = 3; //霍尔出错之后需要重新标定否则不能控制  默认为0，1为开始标定 2 为标定成功，3为标定失败
                        if(i == 0)
                        {
                            MC_SetFault1(PUSH0_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
                        }
                        if(i == 1)
                        {
                            MC_SetFault1(PUSH1_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
                        }
                    }
                }
            }
            else
            {
                Push_Error_Cheek_Cnt[i] = 0;
								Push_curr_temp[i] = 0;
            }
        }
        else if(NeedToCheekPush[i]==2)  //电机3电机4断线检测
        {
            if(MotorControl[i].Motor_Start_Stop == 1 && l_abs(MotorControl[i].PWM_Duty) > MOTOR34_PROTECT_MIN_VAL) //占空比大于2999才保护
            {
                Push_Error_Cheek_Cnt[i]++;
                Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
                if(Push_Error_Cheek_Cnt[i]%10000 == 0)//1s
                {
                    if(Push_curr_temp[i] < MOTOR34_PROTECT_MIN_CUR)
                    {
                        MotorControl[i].Motor_Start_Stop = 0;
                        if(i == 3)
                        {
                            MC_SetFault(MOTOR3_BRAKE_LINE);
                        }
                        if(i == 4)
                        {
                            MC_SetFault(MOTOR4_BRAKE_LINE);
                        }
                    }
                    Push_curr_temp[i] = 0;
                    Push_Error_Cheek_Cnt[i] = 0;
                }
            }
            else
            {
                Push_Error_Cheek_Cnt[i] = 0;
                Push_curr_temp[i] = 0;
            }
        }
				else if(NeedToCheekPush[i]==3)
				{
					if(MotorControl[i].Motor_Start_Stop == 1)
					{
						if(MotorControl[i].PWM_Duty>PUSH_PROTECT_MIN_VAL)
						{
							Push_Error_Cheek_Cnt[i]++;
              Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
							Push_Error_Cheek_Cnt_1[i] =0;
							Push_curr_temp_1[i] = 0; 
							if(Push_Error_Cheek_Cnt[i]%10000==0)
							{
								if(Push_curr_temp[i] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
								{
									CurChkArray[0] =1;
								}
								else 
								{
									CurChkArray[0] =0;
									CurChkArray[1] =0;
								}
								Push_Error_Cheek_Cnt[i]=0;
								Push_curr_temp[i] =0;
							}
						}
						else if(MotorControl[i].PWM_Duty<-PUSH_PROTECT_MIN_VAL)
						{
							Push_Error_Cheek_Cnt_1[i]++;
              Push_curr_temp_1[i] += MotorControl[i].Current.FilterValue;
							Push_Error_Cheek_Cnt[i] =0;
							Push_curr_temp[i] = 0;
							if(Push_Error_Cheek_Cnt_1[i]%10000==0)
							{
								if(Push_curr_temp_1[i] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
								{
									CurChkArray[1] =1;
								}
								else 
								{
									CurChkArray[1] =0;
									CurChkArray[0] =0;
								}
								Push_Error_Cheek_Cnt_1[i]=0;
								Push_curr_temp_1[i] =0;
							}
						}
						if(CurChkArray[0]==1&&CurChkArray[1]==1)
						{
							MotorControl[i].Motor_Start_Stop = DISABLE;   //断线错误
							MotorControl[i].Fault_Flag = 1;  //断线错误
							MC_SetFault1(PUSH1_BRAKE_LINE);
              MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
						}
					}
				}
    }
}
/*------------------------------------------------
Function:推杆电机标定
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
uint32_t Sys_Ticktemp[10] = {0};
extern u8 First_PowerOn;
int push_test_1=1000;
void Push_Motor_Calibrate(u8 motornum)
{
    u8 caputernumsetfinsh = 0;
    if(MotorControl[motornum].Push_Location_model ==1)      //每次标定完毕会将该标志位置1，标定过程中Push_Location_model为0
    {
        MotorControl[motornum].Fault_Flag =0;//标定前把错误清零在标定中堵转或者全部缩回都判断为标定成功
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PercentOfPWM = 0;
        MotorControl[motornum].Push_Location_model = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 10000;  //用于标定，在不同的位置标定开始之后都需要自减
    }
    if(MotorControl[motornum].Fault_Flag == 0)
    {
        MotorControl[motornum].PercentOfPWM=-MAX_PERCENT;//不能删除否则不能进捕获中断
        MotorControl[motornum].PWM_Duty+=10;
        if(MotorControl[motornum].PWM_Duty<100)
        {
          MotorControl[motornum].PWM_Duty=1000;
        }
    }
    else
    {
        MotorControl[motornum].PWM_Duty=0;
    }
    if(MotorControl[motornum].PWM_Duty > PERCENT_95_OF_PWM_PERIOD)
    {
        MotorControl[motornum].PWM_Duty = PERCENT_95_OF_PWM_PERIOD;
    }
    else if(MotorControl[motornum].PWM_Duty < -PERCENT_95_OF_PWM_PERIOD)
    {
        MotorControl[motornum].PWM_Duty = -PERCENT_95_OF_PWM_PERIOD;
    }
    if(motornum == 0)
    {
        MOTOR0CW();
    }
    else if(motornum == 1)
    {
        MOTOR1CCW();
    }
    Sys_Ticktemp[motornum]++; //计时
    if(Sys_Ticktemp[motornum]%501 == 0) // 501赋值一次 1000检查一次  1002次赋值一次 1503赋值一次  2000检查一次 2004赋值一次 2505赋值一次 3000检查一次 3006赋值一次 3057赋值一次 4000检查一次
    {
        MotorControl[motornum].Hall.HALL_PreCaptureValue = MotorControl[motornum].Hall.HALL_CaptureValue ;
    }
    else if(Sys_Ticktemp[motornum]%1000 == 0) //1000个值检查一次
    {
        caputernumsetfinsh = 1;//局部变量来保证霍尔值更新之后进行到位判断
    }
    else
    {
        caputernumsetfinsh = 0;
    }

    if((caputernumsetfinsh == 1 )&&
            MotorControl[motornum].Hall.HALL_PreCaptureValue == MotorControl[motornum].Hall.HALL_CaptureValue
//            MotorControl[motornum].Fault_Flag ==0
      ) //如果时间到，计数值没有改变，电机没有过流，
    {
        MotorControl[motornum].Fault_Flag = 0; //标定中如果过流则清除错误
        MotorControl[motornum].Motor_Start_Stop = 0;
        MotorControl[motornum].Location_Set = -100;     //929修改
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PercentOfPWM = 0; //必须清除，因为没有位置环还可以按照PWM控制方式来控制，这种情况不允许
        MotorControl[motornum].Hall.HALL_PreCaptureValue = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 0;
        MotorControl[motornum].Push_motor_calibrationFLAG = 2;//标定正确退出标定
        MotorControl[motornum].Push_Location_model=1; //开启推杆位置环，该参数用来启用位置控制，需要赋值默认值来启用那些不需要位置控制的推杆电机
        Sys_Ticktemp[motornum] = 0;//计数清零
        if(motornum == 0)
        {
            MOTOR0STOP();
            MC_ClearFault(PUSH_MOTOR1_OVER_CUR);
        }
        else if(motornum == 1)
        {
            MOTOR1STOP();
            MC_ClearFault(PUSH_MOTOR2_OVER_CUR);
        }

    }
    else if(Sys_Ticktemp[motornum] > 7000) //14秒 标定失败
    {
        if(motornum == 0)
        {
            MOTOR0STOP();
            MC_SetFault1(PUSH0_CALIBRATE);
        }
        else if(motornum == 1)
        {
            MOTOR1STOP();
            MC_SetFault1(PUSH1_CALIBRATE);
        }
        MotorControl[motornum].Motor_Start_Stop = 0;
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PercentOfPWM = 0;
        MotorControl[motornum].Hall.HALL_PreCaptureValue = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 0;
				MotorControl[motornum].Push_Location_model = 1;
        MotorControl[motornum].Push_motor_calibrationFLAG = 3; //关闭推杆1位置环，并且退出本次标定
        Sys_Ticktemp[motornum] = 0;
        MC_SetFault(PUSHMOTOR_INT_ERROR);
        MotorState = START;
    }
}



void swap(float *x,float *y)
{
    uint32_t temp= 0;
    temp=*x;
    *x=*y;
    *y=temp;
}
void BubbleSort(float a[], float n)        // 本算法将a[]中的元素从小到到大进行排序
{
    uint32_t j= 0;
    for(uint32_t i = 0; i < n - 1; i++)
    {
        for(j = n - 1; j > i; j--)
        {
            if(a[j - 1]>a[j])
            {
                swap(&a[j - 1],&a[j]);      // 为交换函数，将a[j] 与 a[j - 1] 进行交换
            }
            else
            {}
        }
    }
}
/*------------------------------------------------
Function:电机5缺相检测
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
#define ac 0
#define ab 1
#define bc 2
#define ac_ab_bc_index 3
float M5_100us_ac_ab_bc_cur[ac_ab_bc_index] = {0};
float M6_100us_ac_ab_bc_cur[ac_ab_bc_index] = {0};
float testM5_100us_ac_ab_bc_cur = 0;
float testM6_100us_ac_ab_bc_cur = 0;
float filter_cur = 0.8f;
int Miss_Phase_Count=0;
void Motor5_Default_Phase_Cheek(void)
{
    if(MotorControl[5].Speed_Real < -1000 && MotorControl[5].Motor_Start_Stop == 1 && \
            MotorControl[5].Speed_Ref<-1000  )  //1010号修改
    {
        Motor5_Current_temp.cnt++;
        if(Motor5_Current_temp.cnt>30000)   //6.10修改缺相计数由2秒变成3s
        {
            Motor5_Current_temp.cnt = 0;
            BubbleSort(M5_100us_ac_ab_bc_cur,ac_ab_bc_index);
			if(M5_100us_ac_ab_bc_cur[2]>100)
			{
					 testM5_100us_ac_ab_bc_cur =(M5_100us_ac_ab_bc_cur[2]-M5_100us_ac_ab_bc_cur[0])/M5_100us_ac_ab_bc_cur[2];//测试观察值
				
				
				if(testM5_100us_ac_ab_bc_cur>0.93) //从小到大排序之后最大值-最小值，0.6为测试估计值
				{
					Miss_Phase_Count++;
					if(Miss_Phase_Count>1)
					{
						MC_SetFault(MOTOR5_MISSING_PHASE); //单纯为了指示灯
						MotorControl[5].Fault_Flag =1;
						MotorControl[5].Motor_Start_Stop = 0;
					}
				}

            else
            {
                if(Miss_Phase_Count>0)
                {
                    Miss_Phase_Count--;
                }

				}
			}
            M5_100us_ac_ab_bc_cur[ab] = 0;
            M5_100us_ac_ab_bc_cur[ac] = 0;
            M5_100us_ac_ab_bc_cur[bc] = 0;
        }
        if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[4]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[1])      //BC  CB
        {
            M5_100us_ac_ab_bc_cur[bc] += MotorControl[5].Current.FilterValue;
        }
        else if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[5]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[2]) //BA  AB
        {
            M5_100us_ac_ab_bc_cur[ab] += MotorControl[5].Current.FilterValue;
        }
        else if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[0]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[3]) //CA  AC
        {
            M5_100us_ac_ab_bc_cur[ac] += MotorControl[5].Current.FilterValue;
        }
    }
    else
    {
        M5_100us_ac_ab_bc_cur[ab] = 0;
        M5_100us_ac_ab_bc_cur[ac] = 0;
        M5_100us_ac_ab_bc_cur[bc] = 0;
        if(Motor5_Current_temp.cnt > 0)
        {
            Motor5_Current_temp.cnt--;
        }
    }
}

/*------------------------------------------------
Function:电机6缺相检测
Input   :No
Output  :No
Explain :
------------------------------------------------*/
void Motor6_Default_Phase_Cheek(void)
{
    if(MotorControl[6].Speed_Real != 0 && MotorControl[6].Motor_Start_Stop == 1 && \
            MotorControl[6].Speed_Set != 0  )
    {
        Motor6_Current_temp.cnt++;
        if(Motor6_Current_temp.cnt>20000)
        {
            Motor6_Current_temp.cnt = 0;
            BubbleSort(M6_100us_ac_ab_bc_cur,ac_ab_bc_index);
            testM6_100us_ac_ab_bc_cur =  (M6_100us_ac_ab_bc_cur[2]-M6_100us_ac_ab_bc_cur[0])/M6_100us_ac_ab_bc_cur[2];//测试观察值
            if(testM6_100us_ac_ab_bc_cur>0.5) //从小到大排序之后最大值-最小值，0.5为测试估计值
            {
//                if(MotorControl[6].Current.DeepFilterVAL>50)
                {   
//                    MC_SetFault(MOTOR6_MISSING_PHASE); //单纯为了指示灯
                    MotorControl[6].Fault_Flag =1;
                    MotorControl[6].Motor_Start_Stop = 0;

                }
            }
            M6_100us_ac_ab_bc_cur[ab] = 0;
            M6_100us_ac_ab_bc_cur[ac] = 0;
            M6_100us_ac_ab_bc_cur[bc] = 0;
        }
        if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[4]||\
                MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[1])      //BC  CB
        {
            M6_100us_ac_ab_bc_cur[bc] += MotorControl[6].Current.FilterValue;
        }
        else if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[5]||\
                MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[2]) //BA  AB
        {
            M6_100us_ac_ab_bc_cur[ab] += MotorControl[6].Current.FilterValue;
        }
        else if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[0]||\
                MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[3]) //CA  AC
        {
            M6_100us_ac_ab_bc_cur[ac] += MotorControl[6].Current.FilterValue;
        }
    }
    else
    {
        M6_100us_ac_ab_bc_cur[ab] = 0;
        M6_100us_ac_ab_bc_cur[ac] = 0;
        M6_100us_ac_ab_bc_cur[bc] = 0;
				Motor6_Current_temp.cnt=0;
    }
}

/*************************
*Function Name 		:Hardware_flowCheck
*Description   		:Check Hardware flow cnt if >=BREAK_CNT_MAX setfault the motor
* Input           : None
* Output          : None
* Return          : None		2021.11.25	by diamond
*************************/
void Hardware_flowCheck(void)
{
	if(OverFlow_Cnt[0]>=BREAK_CNT_MAX)	/* 无刷5 */
	{
		MC_SetFault(MOTOR5_BREAK);
		MotorControl[5].Fault_Flag = 1;
		MotorControl[5].Motor_Start_Stop = DISABLE;
	}
	if(OverFlow_Cnt[1]>=BREAK_CNT_MAX)	/* 无刷6 */
	{
		MC_SetFault(MOTOR6_BREAK);
		MotorControl[6].Fault_Flag = 1;
		MotorControl[6].Motor_Start_Stop = DISABLE;
	}
}
 /*------------------------------------------------
Function:偏置电压校准
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u8 Voltage_offset_calibration[8] = {0,1,3,4,5,6,7,8};//上下对起来看
u8 ADC_ConvertedValueindex[8] =    {1,2,9,11,6,13,3,4};//上下对起来看
#define OFFSET_NUM 100
u8 offset_cnt = 0;
void Voltage_offset_cali(void)		
{
    offset_cnt++;
    for(int i = 0; i < 7; i++)
    {
        MotorControl[Voltage_offset_calibration[i]].Current.offset += ADC_ConvertedValue[ADC_ConvertedValueindex[i]];
    }
    if(offset_cnt == OFFSET_NUM)		/* 取100次的均值 */
    {
        for(int k = 0; k < 7; k++)
        {
            MotorControl[Voltage_offset_calibration[k]].Current.offset = (MotorControl[Voltage_offset_calibration[k]].Current.offset/OFFSET_NUM);
        }
        MotorState = INIT;
    }
		else if(offset_cnt>OFFSET_NUM)
		{
			offset_cnt = OFFSET_NUM+1;
			MotorState = INIT;
		}
}
/*************************
*Function Name 		:BLDC5_Phase_Check &&BLDC6_Phase_Check
*Description   		:Check PAHSE  
* Input           : None
* Output          : None
* Return          : None		2021.11.30	by diamond
*************************/
void BLDC5_Phase_Check(void)
{
	static u8 motor_err_cnt5 = 0;
	
	if(ABS(MotorControl[5].PWM_Duty)>2000&&MotorControl[5].Motor_Start_Stop==1)
    {
		if(MotorControl[5].Direction!=-1&&MotorControl[5].Direction!=1)
		{
			motor_err_cnt5++;
			if(motor_err_cnt5>60)		//累计10次报警
			{
				motor_err_cnt5=60;
				MotorControl[5].Motor_Start_Stop = DISABLE;
				MotorControl[5].Fault_Flag =1;
				MC_SetFault(MOTOR5_PHASE_ERROR);
			}
		}
		else if(ABS(MotorControl[5].Speed_Set-MotorControl[5].Speed_Real)<200)
		{
			if(motor_err_cnt5>0)
			motor_err_cnt5--;
		
		}
	}
}
void BLDC6_Phase_Check(void)
{
	static u8 motor_err_cnt6 = 0;
	if(MotorControl[6].PWM_Duty!=0)
	{
		if(MotorControl[6].Direction!=-1&&MotorControl[6].Direction!=1)
		{
			motor_err_cnt6++;
			if(motor_err_cnt6>10)//10次报警
			{
				motor_err_cnt6=10;
				MotorControl[6].Motor_Start_Stop = DISABLE;
				MotorControl[6].Fault_Flag =1;
				MC_SetFault(MOTOR6_PHASE_ERROR);
			}
		}
		else if(ABS(MotorControl[6].Speed_Set-MotorControl[6].Speed_Real)<200)
		{
			if(motor_err_cnt6>0)
			motor_err_cnt6--;
		}
	}
}
/*************************
*Function Name 		:Push_Motor_CurSelfAdapt
*Description   		:推杆电流根据无刷电机实现高度自适应  
* Input           : u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min 
										无刷电机号，有刷电机号，自适应最大高度，自适应最小高度
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
float cur_ratio[7] = {0,0,0,0,0,1.21,1.21f};
s16 motottest=0;
u16 hight_err_cnt = 0;
u8 adjust_flag = 0;
u16 Inplacecnt=0;
void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min)
{
	s16 cur_temp;
	if(MotorControl[bldc_num].Current.DeepFilterVAL<0) 
	{
		cur_temp = (s16)(-MotorControl[bldc_num].Current.DeepFilterVAL);
	}
	else
	{
	   cur_temp = 0;
	}
	if(MotorControl[bldc_num].Current.SetValue!=0)
	{
		if(MotorControl[bldc_num].Fault_Flag==0)
		{
			Adaptive_Rolling_Brush_Detection();
			if(MotorControl[push_num].Motor_Start_Stop ==DISABLE)MotorControl[push_num].Motor_Start_Stop = ENABLE;
			
			if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])<MotorControl[5].Current.CurOffset*cur_ratio[bldc_num])
			{
				MotorControl[push_num].PercentOfPWM = 0;
			    MotorControl[bldc_num].Adaptive_Start_Flag=2;
				if(hight_err_cnt>0)
				hight_err_cnt--;
				else if(hight_err_cnt==0)
					adjust_flag=1;
			}
			else if(adjust_flag==1)
			{
				if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])>(MotorControl[5].Current.CurOffset+20)*cur_ratio[bldc_num])
				{
					hight_err_cnt++;
					if(hight_err_cnt>150)
					{
						hight_err_cnt=150;
						adjust_flag = 0;
						MotorControl[bldc_num].Adaptive_Start_Flag=1;
						MotorControl[push_num].PercentOfPWM = PID_Regulator(cur_temp,MotorControl[bldc_num].Current.SetValue,&PID_PWM)/PWM_COEFFICIENT;
						if(f_abs(MotorControl[push_num].PercentOfPWM)<50)    
						{	
							 MotorControl[push_num].PercentOfPWM = 0;
						}
					}
				}
			}
			else if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])>MotorControl[5].Current.CurOffset*cur_ratio[bldc_num])
			{
				hight_err_cnt++;
				if(hight_err_cnt>150)
				{
					hight_err_cnt=150;
					adjust_flag=0;
					MotorControl[bldc_num].Adaptive_Start_Flag=1;
//					MotorControl[push_num].PercentOfPWM = PID_Regulator(cur_temp,MotorControl[bldc_num].Current.SetValue,&PID_PWM);
					motottest=PID_Regulator(cur_temp,MotorControl[bldc_num].Current.SetValue,&PID_PWM);
					MotorControl[push_num].PercentOfPWM =motottest/PWM_COEFFICIENT;
//					MotorControl[push_num].PercentOfPWM =MotorControl[push_num].PercentOfPWM /PWM_COEFFICIENT;
					if(f_abs(MotorControl[push_num].PercentOfPWM)<50)    
					{	
						 MotorControl[push_num].PercentOfPWM = 0;
					}
				}
			}
			if(MotorControl[push_num].Hall.HALL_CaptureValue>hall_max&&MotorControl[push_num].PercentOfPWM<0)
				MotorControl[push_num].PercentOfPWM = 0;
			else if(MotorControl[push_num].Hall.HALL_CaptureValue<hall_min&&MotorControl[push_num].PercentOfPWM>0)
				MotorControl[push_num].PercentOfPWM = 0;
		}
		else 
		{
			MotorControl[push_num].PercentOfPWM = 0;
		}
		if(MotorControl[bldc_num].Speed_Real<-100)         //转起来的情况
		{
			MotorControl[bldc_num].Current.SetValue_Min=MotorControl[bldc_num].Current.SetValue-150;
			if((Push_Type==0&&MotorControl[push_num].Hall.HALL_CaptureValue>120)||(Push_Type==1&&MotorControl[push_num].Hall.HALL_CaptureValue>350))
			{
				MotorControl[push_num].Inplace=1;
			}
			
			else if((Push_Type==0 && MotorControl[push_num].Hall.HALL_CaptureValue>60 && cur_temp>MotorControl[bldc_num].Current.SetValue_Min)||(Push_Type==1 && MotorControl[push_num].Hall.HALL_CaptureValue>250 && cur_temp>MotorControl[bldc_num].Current.SetValue_Min))
			{
				MotorControl[push_num].Inplace=1;
			}
		
	   }
		
   }
	else
	{
		if(MotorControl[0].Location_Set<0)
		{
			Inplacecnt++;
			if((Inplacecnt>800&&MotorControl[0].Hall.HALL_CaptureValue<30)||(MotorControl[0].Hall.HALL_CaptureValue<10))
			{
				 MotorControl[0].Inplace=0;
				MotorControl[0].Hall.HALL_CaptureValue=0;
//				MotorControl[0].Hall.HALL_PreCaptureValue=0;
				Inplacecnt=0;
			}
	    }
	}
}
/*************************
*Function Name 		:Motor7_Err_Chk
*Description   		:边刷电机错误检测  2S没速度报警 
* Input           : None
* Output          : None	
* Return          : None		2021.12.16	by diamond
*************************/
void Motor7_Err_Chk(void)		/* 200ms循环里 */
{
	static u8 err_cur_cnt = 0;
	if(MotorControl[7].PWM_Duty>500&&MotorControl[7].Speed_Real==0)
	{
		err_cur_cnt++;
		if(err_cur_cnt==20)		
		{
			MotorControl[7].Fault_Flag =1;
			MotorControl[7].Motor_Start_Stop = DISABLE;
			MC_SetFault(SIDE_BRUSH_ERROR);
		}
	}
	else 
		err_cur_cnt=0;
}
/*************************
*Function Name     :BLDC_Stuck_Chk
*Description       :BLDC堵转保护  500ms无动作报警
* Input           : None
* Output          : None
* Return          : None    2021.12.16  by diamond
*************************/
u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
void BLDC_Stuck_Chk(void)      /* 1ms循环里 */
{
//    static u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
    if(GetMotorSpeed(Motor5)==0&&MotorControl[5].PWM_Duty!=0)  //BLDC1带载时缺相，电流为0
    {
        if(MotorControl[5].Current.DeepFilterVAL < 20) //基本可以判断为带载时缺相，因为那一相是断开的没有电流
        {
            M5phase_check_cnt++;//带载缺相计数
        }
        else
        {
            M5stuck_check_cnt++;//堵转计数
        }

        if(M5phase_check_cnt>PHASE_ERR_MAX)
        {
            MC_SetFault(MOTOR5_MISSING_PHASE);//M5缺相
            M5phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[5].Fault_Flag =1;
            MotorControl[5].Motor_Start_Stop = 0;

        }
        if(M5stuck_check_cnt>SUTCK_ERR_MAX)
        {
            M5stuck_check_cnt = SUTCK_ERR_MAX;
            MC_SetFault(MOTOR5_PHASE_ERROR); //M5相序错误
            MotorControl[5].Fault_Flag =1;
            MotorControl[5].Motor_Start_Stop = 0;

        }
    }
    else
    {
        if(M5phase_check_cnt>0)
        {
            M5phase_check_cnt--;
        }
        if(M5stuck_check_cnt>0)
        {
            M5stuck_check_cnt--;
        }
    }

    if(GetMotorSpeed(Motor6)==0&&MotorControl[6].PWM_Duty!=0)  //BLDC1带载时缺相，电流为0
    {
        if(MotorControl[6].Current.DeepFilterVAL < 20) //基本可以判断为带载时缺相，因为那一相是断开的没有电流
        {
            M6phase_check_cnt++;//带载缺相计数
        }
        else
        {
            M6stuck_check_cnt++;//堵转计数
        }

        if(M6phase_check_cnt>PHASE_ERR_MAX)
        {
//            MC_SetFault(MOTOR6_MISSING_PHASE);//M6缺相
            M6phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[6].Fault_Flag =1;
            MotorControl[6].Motor_Start_Stop = 0;

        }
        if(M6stuck_check_cnt>SUTCK_ERR_MAX)
        {
            MC_SetFault(MOTOR6_PHASE_ERROR); //M6相序错误
            M6stuck_check_cnt = SUTCK_ERR_MAX;
            MotorControl[6].Fault_Flag =1;
            MotorControl[6].Motor_Start_Stop = 0;

        }
    }
    else
    {
        if(M6phase_check_cnt>0)
        {
            M6phase_check_cnt--;
        }
        if(M6stuck_check_cnt>0)
        {
            M6stuck_check_cnt--;
        }
    }

}
/*************************
*Function Name 		:BLDC1_OverSpdChk
*Description   		:BLDC1失速报警  4s连续速度异常报警
* Input           : None
* Output          : None
* Return          : None		2021.12.17	by diamond
*************************/
void BLDC1_OverSpdChk(void)
{
	static u16 speed_err_cnt = 0;
	if((MotorControl[5].Speed_Ref>0&&MotorControl[5].Speed_Real<0)||(MotorControl[5].Speed_Ref<0&&MotorControl[5].Speed_Real>0))		/* 反向转 */
		speed_err_cnt++;
	else if(MotorControl[5].Speed_Ref==0&&ABS(MotorControl[5].Speed_Set)<500)
		speed_err_cnt=0;			
	else if(ABS(MotorControl[5].Speed_Real)-ABS(MotorControl[5].Speed_Ref)>500&&MotorControl[5].Speed_Real<-3000)																								/* 实际>设定 */
		speed_err_cnt++;
	else if(speed_err_cnt>0)
		speed_err_cnt--;
	if(speed_err_cnt>2000)
	{
		speed_err_cnt = 0;
		MotorControl[5].Fault_Flag = 1;
		MotorControl[5].Motor_Start_Stop = DISABLE;
		MC_SetFault(MOTOR5_OVER_SPEED);
	}

}
void BLDC2_OverSpdChk(void)
{
	static u16 speed_err_cnt = 0;
	if((MotorControl[6].Speed_Ref>0&&MotorControl[6].Speed_Real<0)||(MotorControl[6].Speed_Ref<0&&MotorControl[6].Speed_Real>0))		/* 反向转 */
		speed_err_cnt++;			
	else if(MotorControl[6].Speed_Ref==0&&ABS(MotorControl[6].Speed_Set)<500)
		speed_err_cnt=0;		
	else if(ABS(MotorControl[6].Speed_Real)-ABS(MotorControl[6].Speed_Ref)>500)																										/* 实际>设定 */
		speed_err_cnt++;
	else if(speed_err_cnt>0)
		speed_err_cnt--;
	if(speed_err_cnt>2000)
	{
		speed_err_cnt = 0;
		MotorControl[6].Fault_Flag = 1;
		MotorControl[6].Motor_Start_Stop = DISABLE;
		MC_SetFault(MOTOR6_OVER_SPEED);
	}
}

/*************************
*Function Name 		:Push_OverRunChk
*Description   		:推杆动作时间限制
* Input           : u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3
										push 1-4:	0关闭 1开启	timer:0-256 多少秒后停止
* Output          : None
* Return          : None		2022.1.27	by diamond
*************************/
u8 timer_cnt[4]=0;
void Push_OverRunChk(u8 push1,u8 timer0,u8 push2,u8 timer1)
{
	
	if(push1)
	{
		if(MotorControl[0].Motor_Start_Stop==ENABLE&&ABS(MotorControl[0].PWM_Duty)>6000&&MotorControl[5].Current.SetValue==0)
		{
			timer_cnt[0]++;
			if(timer_cnt[0]>timer0*2)
			{
				MotorControl[0].Motor_Start_Stop=DISABLE;
//				MotorControl[0].Fault_Flag =1;
//				MC_SetFault(PUSH_OVER_TIME);
//				MC_SetFault1(PUSH0_OVERTIME);
				timer_cnt[0]=0;
			}
		}
		else timer_cnt[0]=0;
	}
	if(push2)
	{
		if(MotorControl[1].Motor_Start_Stop==ENABLE&&ABS(MotorControl[1].PWM_Duty)>500)
		{
			timer_cnt[1]++;
			if(timer_cnt[1]>timer1*2)
			{
				MotorControl[1].Motor_Start_Stop=DISABLE;
//				MotorControl[1].Fault_Flag =1;
				MC_SetFault(PUSH_OVER_TIME);
				MC_SetFault1(PUSH1_OVERTIME);
				timer_cnt[1]=0;
			}
		}
		else timer_cnt[1]=0;
	}
}
extern int8_t ad_angel;
void Motor_Choose(void)
{
	if(Motor_Type==0)                  //滚刷电机选择，0：联谊滚刷电机，超前角12    1：和泰滚刷电机  超前角6
	{
		ad_angel=12;
		MotorControl[5].Current.MaxValue1 = MOTOR6_CurrentValue(2) ;
		MotorControl[5].Current.MaxValue2 = MOTOR6_CurrentValue(5) ;
		MotorControl[5].Current.MaxValue3 = MOTOR6_CurrentValue(7) ;
		MotorControl[5].Current.MaxValue4 = MOTOR6_CurrentValue(8.5) ;
		BLDC_SPEED_COEEFFOCIENT=30;
		PID_Current_InitStructure[0].hLower_Limit_Output=-4000;    //定时器1频率改成了20khz，PWM最大输出4200
		PID_Current_InitStructure[0].hUpper_Limit_Output=4000;
		PID_Current_InitStructure[0].wLower_Limit_Integral = -4000 * 512;
        PID_Current_InitStructure[0].wUpper_Limit_Integral = 4000 * 512;
	}
	else if(Motor_Type==1)
	{
		ad_angel=6;
		MotorControl[5].Current.MaxValue1 = MOTOR6_CurrentValue(2) ;
		MotorControl[5].Current.MaxValue2 = MOTOR6_CurrentValue(6) ;
		MotorControl[5].Current.MaxValue3 = MOTOR6_CurrentValue(7) ;
		MotorControl[5].Current.MaxValue4 = MOTOR6_CurrentValue(8.5) ;
		BLDC_SPEED_COEEFFOCIENT=36;
		PID_Current_InitStructure[0].hLower_Limit_Output=-4000;    //定时器1频率改成了20khz，PWM最大输出4200
		PID_Current_InitStructure[0].hUpper_Limit_Output=4000;
		PID_Current_InitStructure[0].wLower_Limit_Integral = -4000 * 512;
        PID_Current_InitStructure[0].wUpper_Limit_Integral = 4000 * 512;
	}
}

u8 Select_Flag=0;
u8 Flash_cnt=0;
u8 Adjust_Flag1=0;
void Push_Type_Select(void)
{
	 u8 Hall_Finsh = 0;
	if(MotorControl[0].Adjust_Flag==1)
	{
		MotorControl[0].Motor_Start_Stop=1;
		MotorControl[0].Location_Set=600;
		 Sys_Ticktemp[5]++; //计时
		if(Sys_Ticktemp[5]%100 == 0) // 1s检测一次
		{
			MotorControl[0].Hall.HALL_PreCaptureValue = MotorControl[0].Hall.HALL_CaptureValue ;
		}
		if(Sys_Ticktemp[5]%800 == 0) // 8s检测一次
		{
			Hall_Finsh=1;
		}
		if(MotorControl[0].Hall.HALL_PreCaptureValue == MotorControl[0].Hall.HALL_CaptureValue&&Hall_Finsh==1)
		{
//			MotorControl[0].Hall.HALL_PreCaptureValue=0;
			MotorControl[0].Hall.HALL_CaptureValue_Max=MotorControl[0].Hall.HALL_CaptureValue;
			Sys_Ticktemp[5]=0;
			Adjust_Flag1=1;
			if(MotorControl[0].Hall.HALL_CaptureValue>300)
			{
				Push_Type=1;
				MotorControl[0].Adjust_Flag=2;//校准完成
				MotorControl[0].Location_Set=-600;
			}
			else
			{
				Push_Type=0;
				MotorControl[0].Adjust_Flag=2;  //校准完成
				MotorControl[0].Location_Set=-600;
			}
		}	
	}
	if((MotorControl[0].Adjust_Flag==2)&&(Adjust_Flag1==1))
	{
		if((Push_Type==0&&MotorControl[0].Hall.HALL_CaptureValue_Max>150)||(Push_Type==1&&MotorControl[0].Hall.HALL_CaptureValue_Max>300))
		{
			Select_Flag=1;    //校准成功
		}
		else
		{
			Select_Flag=2;    //校准失败
			MC_SetFault1(PUSH0_LOST_HALL);
            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
		}
			transbuf[80]=MotorControl[0].Adjust_Flag; // 校准完成                          
			transbuf[81]=Push_Type; // 推杆类型                                          
			transbuf[82]=Select_Flag; // 校准成功                                  
			transbuf[83]=MotorControl[0].Hall.HALL_CaptureValue_Max; //最大hall记录 
		     Flash_cnt++;
		     if(Flash_cnt==5)
			 { 
				 Flash_Writesign=1;
				 Flash_cnt=0;
			 }
			Sys_Ticktemp[6]++;
		   if(Sys_Ticktemp[6]>800)
		   {
              Flash_WriteCheck();	
			   Sys_Ticktemp[6]=0;
			   Adjust_Flag1=0;
		   }			   
	}
}


void Bldc_Hall_Check(void)   //hall错误函数
{
	 if(MotorControl[5].Hall.HallState == 0||MotorControl[5].Hall.HallState == 7 )
		{
				MotorControl[5].Hall_Error_Syscnt++;
				if(MotorControl[5].Hall_Error_Syscnt>3)   //2.5s内hall不低则上报霍尔错误
				{
				   MC_SetFault(HALL5_SENSOR_ERR);
				   MotorControl[5].Hall_Error_Syscnt=0;
				}
		}
		else
		{
			if(MotorControl[5].Hall_Error_Syscnt>0)
			{
				MotorControl[5].Hall_Error_Syscnt--;
			}
		}
		
}

void  Motor3_Temperation_Protect(void)
{
	if(MotorControl[3].Motor_Start_Stop == 1&&MotorControl[3].PercentOfPWM>30)
		{
			if(MotorControl[3].Current.GetADC_Temp < 1050)  //1100差不多在96°左右，2000在70°左右，2800在40°左右
			{
			    MotorControl[3].Hall_Error_Syscnt++;
				if(MotorControl[3].Hall_Error_Syscnt>50)   //1S温度达到100度以上
				{
					MotorControl[3].Motor_Start_Stop=0;
	        		MC_SetFault(PUMP_OVER_TEMP);		
				}
			}
			else
			{
				if(MotorControl[3].Hall_Error_Syscnt>0)
				{
					MotorControl[3].Hall_Error_Syscnt--;
				}
			}		
		}
}


void Adaptive_Rolling_Brush_Detection(void)   //滚刷自适应调节失败
{
	if(MotorControl[5].Current.DeepFilterVAL-MotorControl[5].Current.SetValue>100)  //深度滤波的电流-实际电流>100
	{
		if(MotorControl[5].Adaptive_Start_Flag=1)    //滚刷自适应已经调节了的情况
		{
			MotorControl[5].Adaptive_Fail_Cnt++;
			if(MotorControl[5].Adaptive_Fail_Cnt>150)   //3s中调节不过来报自适应失败
			{
				MC_SetFault(PUSH_SELF_CHK_ERR);	
                MotorControl[5].Adaptive_Fail_Cnt=150;				
			}
		}
        else
		{
            if(MotorControl[5].Adaptive_Fail_Cnt>0)   //调节清0操作
			{
				MotorControl[5].Adaptive_Fail_Cnt--;
			}
		}			
	}	
}


void Motor5_Auto_Check(void)  //滚刷自检  20ms
{
	if(MotorControl[5].Auto_Check==1)  //收到下位机的指令
	{
		if(MotorControl[0].Push_motor_calibrationFLAG == 2 && MotorControl[0].Hall.HALL_CaptureValue<20)  //如果此时推杆已标定并且实际位置<20
		{
			MotorControl[5].Motor_Start_Stop=1;
			MotorControl[5].PercentOfPWM=100;
			MotorControl[5].Auto_Check_Cnt1++;
			if(MotorControl[5].Auto_Check_Cnt1>400)   //转8s
			{
				MotorControl[5].Auto_Check_Cnt1=0;  //停止
				MotorControl[5].PercentOfPWM=0;
				Ramp_Speed(5);
				MotorControl[5].Motor_Start_Stop=0;		    
				MotorControl[5].Auto_Check=2;
			}
			if(f_abs(MotorControl[5].Current.DeepFilterVAL)>150)  //电流>1.5A则代表此时电机存在卡住现象
			{
				MotorControl[5].Auto_Check_Cnt++;
				if(MotorControl[5].Auto_Check_Cnt>200)  //超过4s
				{
					MC_SetFault(MOTOR5_AUTOCHECK_ERR);
					MotorControl[5].Auto_Check_Flag=2;   //滚刷自检标志位变成2，代表自检失败
					MotorControl[5].Auto_Check_Cnt=0;
				}			
			}
			else
			{
				MotorControl[5].Auto_Check_Flag=1;        //滚刷自检标志位变成1，代表自检成功
				if(MotorControl[5].Auto_Check_Cnt>0)
				{
					MotorControl[5].Auto_Check_Cnt--;
				}		
				
			}		
		}
		else if(MotorControl[0].Hall.HALL_CaptureValue>20)  //重新标定
		{
			MotorControl[0].Push_motor_calibrationFLAG = 1;
		    Push_Motor_Calibrate(0);			
		}
	}
}