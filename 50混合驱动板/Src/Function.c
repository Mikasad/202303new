#include "function.h"
#include "stm32f4xx_hal.h"
#include "bsp_BDCMotor.h"
#include "main.h"
#include "canopen_pdo.h"
#include "Agreement.h"
#include "flash.h"


VoltVar_TypeDef VoltVar;
u32 wGlobal_Flags;
u32 prewGlobal_Flags;
u32 wGlobal_Flags1;
u32 FaultOccurred = 0;
u32 FaultOccurred1 = 0;
uint32_t DisplayCounter = 0;
DefaultPhaseCheekCurTemp Motor5_Current_temp;
DefaultPhaseCheekCurTemp Motor6_Current_temp;
//堵转结构体
typedef struct OLCKED_ROTOR_Ttag
{
    long pStuckCurr;
    long pStuckTime;
    long pStuckVel;
} OLCKED_ROTOR_T;
/*------------------------------------------------
Function:MCU版本号
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver)
{
    pVersion->uVersionPartOfRobotType = robotype;				//项目类型
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
void HardVersion_Init(MCU_Version *pVersion,u8 funtype,u8 vol,u8 cur_max,u8 update_ver)
{
    pVersion->HardwarePartOfMotorType = funtype;        //电机类型：混合H,伺服S,步进B
    pVersion->HardwarePartOfVotage = vol;                //工作电压  A：24V
    pVersion->HardwarePartOfCurrent = cur_max;          //最大电流
    pVersion->HardwarePartOfVersion = update_ver;        //更新的版本号
    pVersion->HardwareFullVersion = (funtype<<24)+(vol<<16)+(cur_max<<8)+update_ver;
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
        MotorControl[Motor_NUM].Current.OFCnt1++;
        if(MotorControl[Motor_NUM].Current.OFCnt1 > MotorControl[Motor_NUM].Current.OFCnt1_T)
        {
//            if(MotorControl[Motor_NUM].PWM_Duty>0)
//            {
//                MotorControl[Motor_NUM].PWM_Duty -= 1;
//            }
//            else if(MotorControl[Motor_NUM].PWM_Duty<0)
//            {
//                MotorControl[Motor_NUM].PWM_Duty += 1;
//            }
        }
        if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue2)
        {
            MotorControl[Motor_NUM].Current.OFCnt2++;
            if(MotorControl[Motor_NUM].Current.OFCnt2 > MotorControl[Motor_NUM].Current.OFCnt2_T)
            {
                SetMotorStop(Motor_NUM);
                MC_SetFault(OVER_CURRENT(Motor_NUM));

                MotorControl[Motor_NUM].Fault_Flag = 1;
                MotorControl[Motor_NUM].Current.OFCnt2 = 0;
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
    u8 i;
    wGlobal_Flags |= hFault_type;
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
    if(OverFlow_Cnt[3]<BREAK_CNT_MAX) //如果brake报警5次之后就不能清除了
    {
        if(((wGlobal_Flags&(PUSH_MOTOR1_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2_9))==0)&&MotorControl[0].Fault_Flag == 1)//推杆0错误清除
        {
            MotorControl[0].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR2_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2_9))==0)&&MotorControl[1].Fault_Flag == 1)//推杆1错误清除
        {
            MotorControl[1].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR3_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2_9))==0)&&MotorControl[2].Fault_Flag == 1)//推杆2错误清除
        {
            MotorControl[2].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR4_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2_9))==0)&&MotorControl[9].Fault_Flag == 1)//推杆9错误清除
        {
            MotorControl[9].Fault_Flag = 0;
        }
		
    }
    if(OverFlow_Cnt[2]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(ONEWYA_MOTOR1_OVER_CUR|MOTOR3_BRAKE_LINE|BRAKE_3_4))==0)&&MotorControl[3].Fault_Flag == 1)//电机3错误清除
        {
            MotorControl[3].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(ONEWYA_MOTOR2_OVER_CUR|MOTOR4_BRAKE_LINE|BRAKE_3_4))==0)&&MotorControl[4].Fault_Flag == 1)//电机4错误清除
        {
            MotorControl[4].Fault_Flag = 0;
        }
    }
    if(OverFlow_Cnt[0]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(BLDC1_OVER_CUR|HALL5_SENSOR_ERR|MOTOR5_PHASE_ERROR|MOTOR5_OVER_SPEED|MOTOR5_MISSING_PHASE|MOTOR5_BREAK))==0)&&MotorControl[5].Fault_Flag == 1)//电机5错误清除 无刷电机
        {
            MotorControl[5].Fault_Flag = 0;
        }
    }

    if(OverFlow_Cnt[1]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(BLDC2_OVER_CUR|HALL6_SENSOR_ERR|MOTOR6_PHASE_ERROR|MOTOR6_OVER_SPEED|MOTOR6_MISSING_PHASE|MOTOR6_BREAK))==0)&&MotorControl[6].Fault_Flag == 1)//电机6错误清除 无刷电机
        {
            MotorControl[6].Fault_Flag = 0;
        }
    }
    if(((wGlobal_Flags&SIDE_BRUSH_ERROR)==0)&&MotorControl[7].Fault_Flag == 1)//电机7错误清除 边刷
    {
        MotorControl[7].Fault_Flag = 0;
    }
    if(((wGlobal_Flags&FAN_ERROR)==0)&&MotorControl[8].Fault_Flag == 1)//电机8错误清除 风机
    {
        MotorControl[8].Fault_Flag = 0;
    }
    if(OverFlow_Cnt[4]<BREAK_CNT_MAX)
    {
        if((wGlobal_Flags&BRAKE10_11_12_13)==0&&MotorControl[10].Fault_Flag == 1)
        {
            MotorControl[10].Fault_Flag = 0;
            MotorControl[11].Fault_Flag = 0;
            MotorControl[12].Fault_Flag = 0;
            MotorControl[13].Fault_Flag = 0;
        }
    }
}
/*------------------------------------------------
Function:绝对值函数（float类型）
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
float f_abs(float a)
{
    float temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}
/*------------------------------------------------
Function:绝对值函数（long类型）
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
long l_abs(long a)
{
    long temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}
/*------------------------------------------------
Function:过流保护
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MotorStuckCheck(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        CurrentLimit(i);
    }
}
/*------------------------------------------------
Function:过压检测
Input   :No
Output  :No
Explain :低压恢复：20V  高压恢复：30V  过压：36V 欠压：16V 500ms告警
------------------------------------------------*/
void Over_VoltageCheck(void)  //10ms
{
    if(VoltVar.BUS>VoltVar.BusHigh)//过压
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
Function:5,6号电机选择
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
int kp_Value=500;
int ki_Value=10;
void Motor_Choose(void)
{
	if(Motor_Type==0)                  //滚刷电机选择，0：滚刷电机   1：盘刷电机
	{
		MotorControl[5].Pole_Paires=4;
		MotorControl[6].Pole_Paires=4;
		MotorControl[5].Speed_Set=MotorControl[5].Speed_PWN_Select;
		MotorControl[6].Speed_Set=MotorControl[6].Speed_PWN_Select;
		MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(12) ;
		MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(14) ;
		MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(15) ;
		MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(16) ;
		
			
		MotorControl[6].Current.MaxValue1 = MOTOR6_CurrentValue(12) ;
		MotorControl[6].Current.MaxValue2 = MOTOR6_CurrentValue(14) ;
		MotorControl[6].Current.MaxValue3 = MOTOR6_CurrentValue(15) ;
		MotorControl[6].Current.MaxValue4 = MOTOR6_CurrentValue(16) ;
	}
	else if(Motor_Type==1)
	{
		MotorControl[5].Pole_Paires=3;
		MotorControl[6].Pole_Paires=3;
		if(MotorControl[5].Speed_PWN_Select>=0)
		{
			MotorControl[5].Speed_Set=-MotorControl[5].Speed_PWN_Select; //盘刷电机5号需要反转
		}
		else
		{
			MotorControl[5].Speed_Set=0;//下位机下发反向转速则为0
		}
		MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(12) ;
		MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(14) ;
		MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(15) ;
		MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(16) ;
	
        MotorControl[6].Speed_Set=MotorControl[6].Speed_PWN_Select;		//盘刷电机则直接赋值给速度
		MotorControl[6].Current.MaxValue1 = MOTOR6_CurrentValue(12) ;
		MotorControl[6].Current.MaxValue2 = MOTOR6_CurrentValue(14) ;
		MotorControl[6].Current.MaxValue3 = MOTOR6_CurrentValue(15) ;
		MotorControl[6].Current.MaxValue4 = MOTOR6_CurrentValue(16) ;
		if(MotorControl[6].Speed_Real>1500)
		{
		  PID_Current_InitStructure[1].hLower_Limit_Output= -8399;   //Lower Limit for Output limitation
          PID_Current_InitStructure[1].hUpper_Limit_Output= 8399 ;   //Upper Limit for Output limitation
          PID_Current_InitStructure[1].wLower_Limit_Integral = -8399 * 1024;
          PID_Current_InitStructure[1].wUpper_Limit_Integral = 8399 * 1024;
		}
			if(MotorControl[5].Speed_Real<-1500)   //5号电机反向转速
		{
		  PID_Current_InitStructure[0].hLower_Limit_Output= -8399;   //Lower Limit for Output limitation
          PID_Current_InitStructure[0].hUpper_Limit_Output= 8399 ;   //Upper Limit for Output limitation
          PID_Current_InitStructure[0].wLower_Limit_Integral = -8399 * 1024;
          PID_Current_InitStructure[0].wUpper_Limit_Integral = 8399 * 1024;
		}
		PID_Speed_InitStruct[0].hKi_Gain=1;
		PID_Speed_InitStruct[0].hKp_Gain=500;
		PID_Current_InitStructure[0].hKi_Gain=1;
		PID_Current_InitStructure[0].hKp_Gain=200;	
		PID_Speed_InitStruct[1].hKi_Gain=1;
		PID_Speed_InitStruct[1].hKp_Gain=500;
		PID_Current_InitStructure[1].hKi_Gain=1;
		PID_Current_InitStructure[1].hKp_Gain=200;	
		
	}
	
	else if(Motor_Type==2)
	{
		MotorControl[5].PWM_DutySet=MotorControl[5].Speed_PWN_Select*14/5;  //1月3号修改
		MotorControl[6].PWM_DutySet=MotorControl[6].Speed_PWN_Select*14/5;
		if(MotorControl[6].PWM_DutySet>8000)
		{
			MotorControl[6].PWM_DutySet=8000;
		}
		if(MotorControl[5].PWM_DutySet>8000)
		{
			MotorControl[5].PWM_DutySet=8000;
		}
		 MotorControl[5].Acceleration = 1;
		 MotorControl[6].Acceleration = 1;
		MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(7) ;
		MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(12) ;
		MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(13) ;
		MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(15) ;
			
		MotorControl[6].Current.MaxValue1 = MOTOR6_CurrentValue(7) ;
		MotorControl[6].Current.MaxValue2 = MOTOR6_CurrentValue(12) ;
		MotorControl[6].Current.MaxValue3 = MOTOR6_CurrentValue(13) ;
		MotorControl[6].Current.MaxValue4 = MOTOR6_CurrentValue(15) ;
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
//    TIM3->CCR1 = 0;
//    TIM8->CCR4 = 8400;
//	   MotorControl[10].PWM_DutySet = 8400;		/* S线单向有刷，供电24V */   50不需要
//    MotorControl[12].PWM_DutySet = 8400;		/* S线单向有刷，供电24V */
//    MotorControl[10].Motor_Start_Stop = 1;
//    MotorControl[12].Motor_Start_Stop = 1;
    /************************推杆初始化***************************/
    MotorControl[0].Current.MaxValue1 = MOTOR0_CurrentValue(2) ;
    MotorControl[0].Current.MaxValue2 = MOTOR0_CurrentValue(3) ;
    MotorControl[0].Current.MaxValue3 = MOTOR0_CurrentValue(4) ;
    MotorControl[0].Current.MaxValue4 = MOTOR0_CurrentValue(5) ;
    MotorControl[0].Current.OFCnt1_T = 200;
    MotorControl[0].Current.OFCnt2_T = 30000;
    MotorControl[0].Current.OFCnt3_T = 10000;
    MotorControl[0].Current.OFCnt4_T = 2000;

    MotorControl[1].Current.MaxValue1 = MOTOR1_CurrentValue(2) ;
    MotorControl[1].Current.MaxValue2 = MOTOR1_CurrentValue(1) ;
    MotorControl[1].Current.MaxValue3 = MOTOR1_CurrentValue(2) ;
    MotorControl[1].Current.MaxValue4 = MOTOR1_CurrentValue(3) ;
    MotorControl[1].Current.OFCnt1_T = 200;
    MotorControl[1].Current.OFCnt2_T = 30000;
    MotorControl[1].Current.OFCnt3_T = 10000;
    MotorControl[1].Current.OFCnt4_T = 2000;

    MotorControl[2].Current.MaxValue1 = MOTOR2_CurrentValue(2) ;
    MotorControl[2].Current.MaxValue2 = MOTOR2_CurrentValue(3) ;
    MotorControl[2].Current.MaxValue3 = MOTOR2_CurrentValue(4) ;
    MotorControl[2].Current.MaxValue4 = MOTOR2_CurrentValue(5) ;
    MotorControl[2].Current.OFCnt1_T = 200;
    MotorControl[2].Current.OFCnt2_T = 30000;
    MotorControl[2].Current.OFCnt3_T = 10000;
    MotorControl[2].Current.OFCnt4_T = 2000;

    MotorControl[9].Current.MaxValue1 = MOTOR9_CurrentValue(2) ;
    MotorControl[9].Current.MaxValue2 = MOTOR9_CurrentValue(3) ;
    MotorControl[9].Current.MaxValue3 = MOTOR9_CurrentValue(4) ;
    MotorControl[9].Current.MaxValue4 = MOTOR9_CurrentValue(5) ;
    MotorControl[9].Current.OFCnt1_T = 200;
    MotorControl[9].Current.OFCnt2_T = 30000;
    MotorControl[9].Current.OFCnt3_T = 10000;
    MotorControl[9].Current.OFCnt4_T = 2000;
    /*********************推杆初始化end***************************/

    /*************************单向有刷3_4***************************/
    MotorControl[3].Current.MaxValue1 = MOTOR3_CurrentValue(3) ;  //过滤电机
    MotorControl[3].Current.MaxValue2 = MOTOR3_CurrentValue(4) ;
    MotorControl[3].Current.MaxValue3 = MOTOR3_CurrentValue(5) ;  //过滤电机
    MotorControl[3].Current.MaxValue4 = MOTOR3_CurrentValue(6) ;
    MotorControl[3].Current.OFCnt1_T = 2000;
    MotorControl[3].Current.OFCnt2_T = 30000;
    MotorControl[3].Current.OFCnt3_T = 10000;
    MotorControl[3].Current.OFCnt4_T = 2000;

    MotorControl[4].Current.MaxValue1 = MOTOR4_CurrentValue(3) ;  //喷水电机
    MotorControl[4].Current.MaxValue2 = MOTOR4_CurrentValue(2) ;
    MotorControl[4].Current.MaxValue3 = MOTOR4_CurrentValue(3) ;  //喷水电机
    MotorControl[4].Current.MaxValue4 = MOTOR4_CurrentValue(4) ;
    MotorControl[4].Current.OFCnt1_T = 2000;
    MotorControl[4].Current.OFCnt2_T = 50000;
    MotorControl[4].Current.OFCnt3_T = 10000;
    MotorControl[4].Current.OFCnt4_T = 2000;

    /*************************单向有刷3_4 end***************************/

    /*************************无刷5，6***************************/
    MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(12) ;
    MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(14) ;
    MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(15) ;
    MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(16) ;
    MotorControl[5].Current.CurOffset = 0.2*MOTOR5CURCOEFFICIENT*100;		//默认0.3A
    MotorControl[5].Current.OFCnt1_T = 60000; //没用
    MotorControl[5].Current.OFCnt2_T = 100000;//10秒
    MotorControl[5].Current.OFCnt3_T = 50000; //5秒
    MotorControl[5].Current.OFCnt4_T = 10000; //1秒
    MotorControl[5].Pole_Paires = 4;
    MotorControl[5].Acceleration = 5;
    MotorControl[5].Deceleration = 5;

    MotorControl[6].Current.MaxValue1 = MOTOR6_CurrentValue(12) ;
    MotorControl[6].Current.MaxValue2 = MOTOR6_CurrentValue(14) ;
    MotorControl[6].Current.MaxValue3 = MOTOR6_CurrentValue(15) ;
    MotorControl[6].Current.MaxValue4 = MOTOR6_CurrentValue(16) ;
    MotorControl[6].Current.CurOffset = 0.3*MOTOR6CURCOEFFICIENT*100;		//默认0.3A
    MotorControl[6].Current.OFCnt1_T = 60000; //没用
    MotorControl[6].Current.OFCnt2_T = 100000;//10秒
    MotorControl[6].Current.OFCnt3_T = 50000; //5秒
    MotorControl[6].Current.OFCnt4_T = 10000; //1秒
    MotorControl[6].Pole_Paires = 4;
    MotorControl[6].Acceleration = 5;
    MotorControl[6].Deceleration = 5;
    /*************************无刷5，6end***************************/
    PID_Init (&PID_Speed_InitStruct[0],&PID_Current_InitStructure[0]);		//电机5的PID
    PID_Init (&PID_Speed_InitStruct[1],&PID_Current_InitStructure[1]);		//电机6的PID
    PID_PWM_Init(&PID_PWM);																								//推杆0的PID
    /*	加减速度初始化	*/
    MotorControl[0].Acceleration = 10;
    MotorControl[0].Deceleration = 10;
    MotorControl[0].LastMotorDirection = 2;
    MotorControl[0].Hall.HALL_CaptureValue = 0; //用于上电标定
    MotorControl[1].Acceleration = 10;
    MotorControl[1].Deceleration = 10;
    MotorControl[1].LastMotorDirection = 2;
    MotorControl[1].Hall.HALL_CaptureValue = 10000; //由于推杆1需要位置控制，不标定不会打开Push_Location_model开关，所以在直接给定标定命令的时候当前值为0
    MotorControl[2].Acceleration = 10;
    MotorControl[2].Deceleration = 10;
    MotorControl[2].LastMotorDirection = 2;
    MotorControl[2].Hall.HALL_CaptureValue = 0; //用于上电标定

    MotorControl[3].Acceleration = 10;
    MotorControl[3].Deceleration = 10;

    MotorControl[4].Acceleration = 10;
    MotorControl[4].Deceleration = 10;

    MotorControl[7].Acceleration = 10;
    MotorControl[7].Deceleration = 10;

    MotorControl[8].Acceleration = 10;
    MotorControl[8].Deceleration = 10;

    MotorControl[9].Acceleration = 10;
    MotorControl[9].Deceleration = 10;
    MotorControl[9].LastMotorDirection = 2;
    MotorControl[9].Hall.HALL_CaptureValue = 0; //用于上电标定

    MotorControl[10].Current.OFCnt1_T = 5000;
    MotorControl[10].Current.OFCnt2_T = 200;
    MotorControl[10].Current.OFCnt3_T = 20;
    MotorControl[10].Current.OFCnt4_T = 5;
    MotorControl[10].Acceleration = 10;
    MotorControl[10].Deceleration = 10;

    MotorControl[11].Current.OFCnt1_T = 5000;
    MotorControl[11].Current.OFCnt2_T = 200;
    MotorControl[11].Current.OFCnt3_T = 20;
    MotorControl[11].Current.OFCnt4_T = 5;
    MotorControl[11].Acceleration = 10;
    MotorControl[11].Deceleration = 10;

    MotorControl[12].Current.OFCnt1_T = 5000;
    MotorControl[12].Current.OFCnt2_T = 200;
    MotorControl[12].Current.OFCnt3_T = 20;
    MotorControl[12].Current.OFCnt4_T = 5;
    MotorControl[12].Acceleration = 10;
    MotorControl[12].Deceleration = 10;

    MotorControl[13].Current.OFCnt1_T = 5000;
    MotorControl[13].Current.OFCnt2_T = 200;
    MotorControl[13].Current.OFCnt3_T = 20;
    MotorControl[13].Current.OFCnt4_T = 5;
    MotorControl[13].Acceleration = 10;
    MotorControl[13].Deceleration = 10;

    MotorControl[5].Hall.HallState = (GPIOD->IDR>>5)&(0x7);
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

//    HALL_Study[0].HallTab[0] = 5;
//    HALL_Study[0].HallTab[1] = 1;
//    HALL_Study[0].HallTab[2] = 3;
//    HALL_Study[0].HallTab[3] = 2;
//    HALL_Study[0].HallTab[4] = 6;
//    HALL_Study[0].HallTab[5] = 4;

//    HALL_Study[1].HallTab[0] = 3;// 546231
//    HALL_Study[1].HallTab[1] = 2;
//    HALL_Study[1].HallTab[2] = 6;
//    HALL_Study[1].HallTab[3] = 4;
//    HALL_Study[1].HallTab[4] = 5;
//    HALL_Study[1].HallTab[5] = 1;
	HALL_Study[0].HallTab[0] = 6;
	HALL_Study[0].HallTab[1] = 4;
	HALL_Study[0].HallTab[2] = 5;
	HALL_Study[0].HallTab[3] = 1;
	HALL_Study[0].HallTab[4] = 3;
	HALL_Study[0].HallTab[5] = 2;

	HALL_Study[1].HallTab[0] = 6;// 546231
	HALL_Study[1].HallTab[1] = 4;
	HALL_Study[1].HallTab[2] = 5;
	HALL_Study[1].HallTab[3] = 1;
	HALL_Study[1].HallTab[4] = 3;
	HALL_Study[1].HallTab[5] = 2;

    MotorControl[0].Push_Location_model=1;
    MotorControl[1].Push_Location_model=0; //该推杆电机需要标定，标定成功才可以位置控制，其余电机不需要标定默认开启位置控制，只是伸出和收缩的控制没有位置控制
    MotorControl[2].Push_Location_model=1;
    MotorControl[9].Push_Location_model=1;
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
        RED_LED1_OFF;
    else if(led_mode)	//慢闪
    {
        if(DisplayCounter<2*led_speed)
            RED_LED1_ON;
        else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
            RED_LED1_TOGGLE;
    }
    else if(DisplayCounter<=(2*led_time)*led_speed)	//快闪
    {
        if((DisplayCounter-1)%led_speed==0)
        {
            if((DisplayCounter/led_speed)%2==0)
            {
                RED_LED1_ON;
            }
            else
                RED_LED1_OFF;
        }
    }
    else if(DisplayCounter<2*(led_time+1)*led_speed)
    {
        RED_LED1_OFF;
    }

    //led2
    if(led2==0)
        RED_LED2_OFF;
    else if(led_mode)
    {
        if(DisplayCounter<2*led_speed)
            RED_LED2_ON;
        else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
            RED_LED2_TOGGLE;
    }
    else if(DisplayCounter<=(2*led_time)*led_speed)
    {
        if((DisplayCounter-1)%led_speed==0)
        {
            if((DisplayCounter/led_speed)%2==0)
            {
                RED_LED2_ON;
            }
            else
                RED_LED2_OFF;
        }
    }
    else if(DisplayCounter<2*(led_time+1)*led_speed)
    {
        RED_LED2_OFF;
    }

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
    static u8 DisplayGreenCnt;
    DisplayGreenCnt++;
    if(DisplayGreenCnt>100)
    {
        GREEN_LED_TOGGLE;
        DisplayGreenCnt=0;
    }
    DisplayCounter++;
    if(wGlobal_Flags==NO_ERROR)
    {
        RED_LED1_OFF;
        RED_LED2_OFF;
        wGlobal_Flags1 = 0;
        FaultOccurred = 0;
        FaultOccurred1 = 0;
    }
    else
    {
        switch (FaultOccurred)
        {
        case PUSH_MOTOR1_OVER_CUR: //推杆1闪烁1次
            LEDSet(LED_OFF,LED_ON,1,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR2_OVER_CUR: //推杆2闪烁2次
            LEDSet(LED_OFF,LED_ON,2,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR3_OVER_CUR: //推杆3闪烁3次
            LEDSet(LED_OFF,LED_ON,3,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR4_OVER_CUR: //推杆4闪烁4次
            LEDSet(LED_OFF,LED_ON,4,0,LED_HIGH_SPEED);


            break;
        case BLDC1_OVER_CUR: //BLDC过流5次
            LEDSet(LED_OFF,LED_ON,5,0,LED_HIGH_SPEED);


            break;
        case BLDC2_OVER_CUR: //BLDC2过流上面6次
            LEDSet(LED_OFF,LED_ON,6,0,LED_HIGH_SPEED);


            break;
        case ONEWYA_MOTOR1_OVER_CUR:	//单向有刷1过流上面7次
            LEDSet(LED_OFF,LED_ON,7,0,LED_HIGH_SPEED);


            break;
        case ONEWYA_MOTOR2_OVER_CUR:		//单向有刷2过流上面8次
            LEDSet(LED_OFF,LED_ON,8,0,LED_HIGH_SPEED);
            break;

        case HALL5_SENSOR_ERR:    //无刷1霍尔错误下面红灯
            LEDSet(LED_ON,LED_OFF,1,1,LED_LOW_SPEED);		//慢闪

            break;

        case HALL6_SENSOR_ERR:		//无刷2霍尔错误上面红灯 慢闪1次
            LEDSet(LED_OFF,LED_ON,1,1,LED_LOW_SPEED);

            break;

        case OVER_VOLTAGE:  //过压两个灯常量
            RED_LED1_ON;
            RED_LED2_ON;
            DisplayCounter=0;
            break;
        case UNDER_VOLTAGE:   //欠压下面红灯亮
            RED_LED2_OFF;
            RED_LED1_ON;
            DisplayCounter=0;
            break;
        case MOTOR5_BREAK:		//无刷1brake下面快闪一次
            LEDSet(LED_ON,LED_OFF,1,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_BREAK:		//无刷2brake下面快闪2次
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case BRAKE_3_4:				//单向有刷3,4brake上面快闪3次
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case BRAKE_0_1_2_9:		//推杆brake上面快闪4次
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;
        case BRAKE10_11_12_13:	//有刷brake上面快闪5次
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;

            //6次

        case MOTOR5_PHASE_ERROR:			//无刷1相序错误下面7次
            LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_PHASE_ERROR:					//无刷2相序错误下面8次
            LEDSet(LED_ON,LED_OFF,8,0,LED_HIGH_SPEED);
            break;

        case CAN_COMMUNICATION_ERR:	//CAN通信错误两个灯快闪1次
            LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
            break;

        case MOTOR5_MISSING_PHASE: //无刷1缺相两个红灯快速闪烁2次
            LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_MISSING_PHASE: //无刷2缺相两个红灯快速闪烁3次
            LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);

            break;
        case MOTOR3_BRAKE_LINE:  //电机3断线两个红灯快速闪烁4次
            LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);

            break;
        case MOTOR4_BRAKE_LINE: //电机四断线两个红灯快速闪烁5次
            LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);

            break;
        case SIDE_BRUSH_ERROR:  //边刷错误两个红灯快速闪烁6次
            LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);

            break;
        case MOTOR5_OVER_SPEED:  //无刷1超速两个红灯快速闪烁7次
            LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);

            break;
        case MOTOR6_OVER_SPEED:  //无刷2超速两个红灯快速闪烁8次
            LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);

            break;
        case PUSH_BRAKE_LINE:  //推杆断线		下面常亮，上面快速闪烁1-4次
            switch (FaultOccurred1)
            {
            case PUSH0_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
                break;
            case PUSH1_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
                break;
            case PUSH2_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);
                break;
            case PUSH9_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);
                break;
            }
            RED_LED1_ON;

            break;
        case PUSH_LOST_HALL:  //霍尔断线  	下面常亮，上面快速闪烁5-8次
            switch (FaultOccurred1)
            {
            case PUSH0_LOST_HALL:
                LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);
                break;
            case PUSH1_LOST_HALL:
                LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);
                break;
            case PUSH2_LOST_HALL:
                LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);
                break;
            case PUSH9_LOST_HALL:
                LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);
                break;
            }
            RED_LED1_ON;

            break;
        case PUSHMOTOR_INT_ERROR:			//风机错误上面常亮，下面1-5次
            LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
            RED_LED2_ON;
            break;
        case PUSH_OVER_TIME:		//推杆标定失败上面常亮，下面6-9次
            switch(FaultOccurred1)
            {
            case PUSH0_OVERTIME:
                LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);
                break;
            case PUSH1_OVERTIME:
                LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);
                break;
            case PUSH2_OVERTIME:
                LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);
                break;
            case PUSH9_OVERTIME:
                LEDSet(LED_ON,LED_ON,9,0,LED_HIGH_SPEED);
                break;
            }
            RED_LED2_ON;
            break;
        default:
            RED_LED1_OFF;
            RED_LED2_OFF;
            break;
        }
    }
}
/*------------------------------------------------
Function:推杆电机位置环
Input   :No
Output  :No
Explain :if当中执行的是需要进行自适应的推杆，如果没有推杆自适应，那就位置控制
------------------------------------------------*/
void Push_Motor_Location_Control(u8 motornum)
{
    if(motornum == 1) //50机器滚刷推杆，后期预留滚刷自适应
    {
        if(MotorControl[motornum].Push_motor_calibrationFLAG == 2)//标定正确，开始执行位置控制
        {
            if(MotorControl[5].Current.SetValue ==0)	//没有做电流自适应的情况，进行位置控制
            {
                if(MotorControl[motornum].Hall.HALL_CaptureValue > MotorControl[motornum].Location_Set)//收回，推杆1位置环
                {
                    MotorControl[motornum].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
                }
                if(MotorControl[motornum].Hall.HALL_CaptureValue < MotorControl[motornum].Location_Set)
                {
                    MotorControl[motornum].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
                }
                if(f_abs(MotorControl[motornum].Hall.HALL_CaptureValue- MotorControl[motornum].Location_Set) < 5)
                {
                    MotorControl[motornum].PWM_DutySet = 0;
                }
            }
        }
        else
        {

        }
    }
    else
    {
        if(MotorControl[motornum].Hall.HALL_CaptureValue > MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
        {
            MotorControl[motornum].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
        }
        else if(MotorControl[motornum].Hall.HALL_CaptureValue < MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
        {
            MotorControl[motornum].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
        }
        else if(MotorControl[motornum].Hall.HALL_CaptureValue == MotorControl[motornum].Location_Set)//||fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) < 5)
        {
            MotorControl[motornum].PWM_DutySet = 0;
        }
    }
}
/*------------------------------------------------

Function:4路推杆电机霍尔检测和断联检测
Input   :No
Output  :No
Explain : 100us
------------------------------------------------*/
u8 NeedToCheekPush[MOTOR_NUM] = {3,4,0,2,0,0,0,0,0,0,0,0,0,0}; //如果该推杆电机需要霍尔错误或者推杆断联检测 0 1 2 9号电机默认都打开检测
u8 NeedToMissingHallPush[MOTOR_NUM] = {0,1,0,0,0,0,0,0,0,0,0,0,0,0}; //该数组来确保推杆是否需要霍尔保护
uint16_t PushMaxPulse[MOTOR_NUM] = {780,560,4000,0,0,0,0,0,0,4000,0,0,0,0}; //这个数组需要确定整机推杆的最大值，必须写入正确值，否则在推杆推到尽头可能误报
u32 Push_Error_Cheek_Cnt[MOTOR_NUM] = {0},Push_Error_Cheek_Cnt_1[MOTOR_NUM] = {0};
int Push_curr_temp[MOTOR_NUM] = {0},Push_curr_temp_1[MOTOR_NUM] = {0};
#define PUSH_PROTECT_MIN_VAL 2999
#define PUSH_PROTECT_MIN_CUR  50
#define MOTOR34_PROTECT_MIN_CUR 50
#define MOTOR34_PROTECT_MIN_VAL 2999
u8 CurChkArray[2]= {0};
u8 CurChkArray1[2]= {0};
int test_value=0;
void Push_Motor_Cheek(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        if(NeedToCheekPush[i]==1) //如果该推杆电机需要霍尔错误或者推杆断联检测
        {
            /*占空比大于2999才保护，电机使能、未达到实际位置、PWM大于一定值、霍尔值在推杆最大霍尔值范围之内*/
            if(MotorControl[i].Motor_Start_Stop == 1 && (l_abs(MotorControl[i].Location_Set-MotorControl[i].Hall.HALL_CaptureValue))>20 && \
                    l_abs(MotorControl[i].PWM_Duty) > PUSH_PROTECT_MIN_VAL && MotorControl[i].Hall.HALL_CaptureValue < PushMaxPulse[i])
            {
                /*计数值和电流值累计应该放在电机运行时*/
                Push_Error_Cheek_Cnt[i]++;
                Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
                /*10012000最小公倍数时，会同时执行Push_Error_Cheek_Cnt[i]%51 和 Push_Error_Cheek_Cnt[i]%100==0这里剔除这种情况，因为是想在1000时赋值，在2000时判断，如果同时赋值并判断，霍尔值还没有改变会存在问题*/
                if(Push_Error_Cheek_Cnt[i]%2002000==0)
                {
                    Push_Error_Cheek_Cnt[i]++;
                }
                /*100.1ms将捕获值更新一次，并进行一次断线检测*/
                if(Push_Error_Cheek_Cnt[i]%1001==0)
                {
                    /*每100ms进行一次霍尔值的更新*/
                    MotorControl[i].Hall.HALL_CaptureValueDelta = MotorControl[i].Hall.HALL_CaptureValue;
                    /*如果一段时间的累计电流值小于设定值，就认定为推杆断线*/
                    if(Push_curr_temp[i] < PUSH_PROTECT_MIN_CUR)
                    {
                        MotorControl[i].Motor_Start_Stop = 0;
                        Push_Error_Cheek_Cnt[i] = 0;
                        Push_curr_temp[i] = 0;
                        /*断线错误可以不置错误标志位，可以继续使能*/
//											  MotorControl[i].Fault_Flag = 1;
                        if(i == 0)
                        {
                            /*指示灯做了处理先判断PUSH_BRAKE_LINE，在PUSH_BRAKE_LINE中在进行判断PUSH0_BRAKE_LINE具体是哪个电机存在问题*/
                            MC_SetFault1(PUSH0_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);
                        }
                        if(i == 1)
                        {
                            MC_SetFault1(PUSH1_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);
                        }
                        if(i == 2)
                        {
                            MC_SetFault1(PUSH2_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);
                        }
                        if(i == 9)
                        {
                            MC_SetFault1(PUSH9_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);
                        }
                    }
                    /*这里100ms必须清一次否则不能报警断线保护*/
                    Push_curr_temp[i] = 0;
                }
                /*以上部分先判断有没有断线，以下再判断推杆电机霍尔是否正常*/
                if(NeedToMissingHallPush[i])
                {
                    /*如果200ms内推杆的霍尔值没有改变，那就说明推杆霍尔异常*/
                    if((Push_Error_Cheek_Cnt[i]%2000==0)&&(MotorControl[i].Hall.HALL_CaptureValueDelta == \
                                                           MotorControl[i].Hall.HALL_CaptureValue)&&Push_curr_temp[i] >= PUSH_PROTECT_MIN_CUR)
                    {
                        Push_Error_Cheek_Cnt[i] = 0;
                        Push_curr_temp[i] = 0;
                        MotorControl[i].Motor_Start_Stop = 0;   //霍尔错误
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
                        if(i == 2)
                        {
                            MC_SetFault1(PUSH2_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
                        }
                        if(i == 9)
                        {
                            MC_SetFault1(PUSH9_LOST_HALL);
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
            if((MotorControl[0].Motor_Start_Stop == 1)&&(MotorControl[0].Push_motor_calibrationFLAG==2))
            {
                if(MotorControl[0].PWM_Duty>PUSH_PROTECT_MIN_VAL)
                {
                    Push_Error_Cheek_Cnt[0]++;
                    Push_curr_temp[0] += MotorControl[0].Current.FilterValue;
					test_value=Push_curr_temp[0];
                    Push_Error_Cheek_Cnt_1[0] =0;
                    Push_curr_temp_1[0] = 0;
                    if(Push_Error_Cheek_Cnt[0]%30000==0)
                    {
                        if(Push_curr_temp[0] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
                        {
                            CurChkArray[0] =1;
                        }
                        else
                        {
                            CurChkArray[0] =0;
                            CurChkArray[1] =0;
                        }
                        Push_Error_Cheek_Cnt[0]=0;
                        Push_curr_temp[0] =0;
                    }
                }
                else if((MotorControl[0].PWM_Duty<-PUSH_PROTECT_MIN_VAL)&&(MotorControl[0].Push_motor_calibrationFLAG==2))
                {
                    Push_Error_Cheek_Cnt_1[0]++;
                    Push_curr_temp_1[0] += MotorControl[0].Current.FilterValue;
					test_value=Push_curr_temp_1[0];
                    Push_Error_Cheek_Cnt[0] =0;
                    Push_curr_temp[0] = 0;
                    if(Push_Error_Cheek_Cnt_1[0]%30000==0)
                    {
                        if(Push_curr_temp_1[0] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
                        {
                            CurChkArray[1] =1;
                        }
                        else
                        {
                            CurChkArray[1] =0;
                            CurChkArray[0] =0;
                        }
                        Push_Error_Cheek_Cnt_1[0]=0;
                        Push_curr_temp_1[0] =0;
                    }
                }
                if(CurChkArray[0]==1&&CurChkArray[1]==1)
                {
                    MotorControl[0].Motor_Start_Stop = DISABLE;   //断线错误
                    MotorControl[0].Fault_Flag = 1;  //断线错误
                    MC_SetFault1(PUSH0_BRAKE_LINE);
                    MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
                }
            }
        }
		
		        else if(NeedToCheekPush[i]==4)
        {
            if(MotorControl[1].Motor_Start_Stop == 1)
            {
                if(MotorControl[1].PWM_Duty>PUSH_PROTECT_MIN_VAL)
                {
                    Push_Error_Cheek_Cnt[1]++;
                    Push_curr_temp[1] += MotorControl[1].Current.FilterValue;
                    Push_Error_Cheek_Cnt_1[1] =0;
                    Push_curr_temp_1[1] = 0;
                    if(Push_Error_Cheek_Cnt[1]%10000==0)
                    {
                        if(Push_curr_temp[1] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
                        {
                            CurChkArray1[0] =1;
                        }
                        else
                        {
                            CurChkArray1[0] =0;
                            CurChkArray1[1] =0;
                        }
                        Push_Error_Cheek_Cnt[1]=0;
                        Push_curr_temp[1] =0;
                    }
                }
                else if(MotorControl[1].PWM_Duty<-PUSH_PROTECT_MIN_VAL)
                {
                    Push_Error_Cheek_Cnt_1[1]++;
                    Push_curr_temp_1[1] += MotorControl[1].Current.FilterValue;
                    Push_Error_Cheek_Cnt[1] =0;
                    Push_curr_temp[1] = 0;
                    if(Push_Error_Cheek_Cnt_1[1]%10000==0)
                    {
                        if(Push_curr_temp_1[1] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
                        {
                            CurChkArray1[1] =1;
                        }
                        else
                        {
                            CurChkArray1[1] =0;
                            CurChkArray1[0] =0;
                        }
                        Push_Error_Cheek_Cnt_1[1]=0;
                        Push_curr_temp_1[1] =0;
                    }
                }
                if(CurChkArray1[0]==1&&CurChkArray1[1]==1)
                {
                    MotorControl[1].Motor_Start_Stop = DISABLE;   //断线错误
                    MotorControl[1].Fault_Flag = 1;  //断线错误
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
Explain :需要上位机发送标定标志位，需要位置控制的推杆需要上电标定否则不能使用
------------------------------------------------*/
uint32_t Sys_Ticktemp[10] = {0};
extern u8 First_PowerOn;
void Push_Motor_Calibrate(u8 motornum)
{
    u8 caputernumsetfinsh = 0;
    if(MotorControl[motornum].Push_Location_model ==1)      //每次标定完毕会将该标志位置1，标定过程中Push_Location_model为0
    {
        MotorControl[motornum].Fault_Flag =0;//标定前把错误清零在标定中堵转或者全部缩回都判断为标定成功
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PWM_DutySet = 0;
        MotorControl[motornum].Push_Location_model = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 10000;  //用于标定，在不同的位置标定开始之后都需要自减
    }
    if(MotorControl[motornum].Fault_Flag == 0)
    {
        MotorControl[motornum].PWM_DutySet=-PERCENT_95_OF_PWM_PERIOD;//不能删除否则不能进捕获中断
        MotorControl[motornum].PWM_Duty-=10;
    }
    else
    {
        MotorControl[motornum].PWM_Duty=0;
    }
    if(MotorControl[motornum].PWM_Duty > PERCENT_95_OF_PWM_PERIOD)
    {
        MotorControl[motornum].PWM_Duty = PERCENT_95_OF_PWM_PERIOD;
    }
    if(MotorControl[motornum].PWM_Duty < -PERCENT_95_OF_PWM_PERIOD)
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
    else if(motornum == 2)
    {
        MOTOR2CW();
    }
    else if(motornum == 9)
    {
        MOTOR9CW();
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
        MotorControl[motornum].Location_Set = 0;
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PWM_DutySet = 0; //必须清除，因为没有位置环还可以按照PWM控制方式来控制，这种情况不允许
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
        else if(motornum == 2)
        {
            MOTOR2STOP();
            MC_ClearFault(PUSH_MOTOR3_OVER_CUR);
        }
        else if(motornum == 9)
        {
            MOTOR9STOP();
            MC_ClearFault(PUSH_MOTOR4_OVER_CUR);
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
        else if(motornum == 2)
        {
            MOTOR2STOP();
            MC_SetFault1(PUSH2_CALIBRATE);
        }
        else if(motornum == 9)
        {
            MOTOR9STOP();
            MC_SetFault1(PUSH9_CALIBRATE);
        }
        MotorControl[motornum].Motor_Start_Stop = 0;
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PWM_DutySet = 0;
        MotorControl[motornum].Hall.HALL_PreCaptureValue = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 0;
        MotorControl[motornum].Push_Location_model = 1;
        MotorControl[motornum].Push_motor_calibrationFLAG = 3; //关闭推杆1位置环，并且退出本次标定
        Sys_Ticktemp[motornum] = 0;
        MC_SetFault(PUSHMOTOR_INT_ERROR);
        MotorState = START;
    }
}

/*------------------------------------------------
Function:交换排序
Input   :No
Output  :No
Explain :为了缺相检测当中的电流进行排序排序之后进行运算
------------------------------------------------*/
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
Explain :其中的0.9的数值需要根据不同的电机进行测试
------------------------------------------------*/
#define ac 0
#define ab 1
#define bc 2
#define ac_ab_bc_index 3
float M5_100us_ac_ab_bc_cur[ac_ab_bc_index] = {0};
float M6_100us_ac_ab_bc_cur[ac_ab_bc_index] = {0};
float testM5_100us_ac_ab_bc_cur = 0;
float testM6_100us_ac_ab_bc_cur = 0;
int16_t test_Lack_Count=0;
int16_t test_Lack_Count1=0;
int virtual=20000;
int virtual_time2=0;
int virtual_time_1=0;
int Miss_Phase_Count=0;
int Miss6_Phase_Count=0;
int Moto5_Speed_Check=0;
int Moto6_Speed_Check=0;
void Motor5_Default_Phase_Cheek(void)
{
   if(Motor_Type<2)
  {
	  if(Motor_Type==0)
	  {
		  Moto5_Speed_Check=600;
	  }
	  else if(Motor_Type==1)
	  {
		  Moto5_Speed_Check=1500;
	  }
    if(ABS(MotorControl[5].Speed_Real) > Moto5_Speed_Check && MotorControl[5].Motor_Start_Stop == 1 && \
            MotorControl[5].Speed_Set != 0  )
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
					MotorControl[5].Max_Phase_Lack=testM5_100us_ac_ab_bc_cur*1000;  //记录发生的时候的缺相检测最大值
					Miss_Phase_Count++;
					if(Miss_Phase_Count>2)
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
		if(MotorControl[5].Direction==1||MotorControl[5].Direction==-1)             //加入方向判断
		{
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
}
/*------------------------------------------------
Function:电机6缺相检测
Input   :No
Output  :No
Explain :其中的0.9的数值需要根据不同的电机进行测试
------------------------------------------------*/
void Motor6_Default_Phase_Cheek(void)
{
  if(Motor_Type<2)
  {
	  if(Motor_Type==0)
	  {
		  Moto6_Speed_Check=600;
	  }
	  else if(Motor_Type==1)
	  {
		  Moto6_Speed_Check=1500;
	  }
    if(ABS(MotorControl[6].Speed_Real) > Moto6_Speed_Check && MotorControl[6].Motor_Start_Stop == 1 && \
            MotorControl[6].Speed_Set != 0  )
    {
        Motor6_Current_temp.cnt++;
        if(Motor6_Current_temp.cnt>30000)    //6.15修改
        {
            Motor6_Current_temp.cnt = 0;
            BubbleSort(M6_100us_ac_ab_bc_cur,ac_ab_bc_index);
			if(M6_100us_ac_ab_bc_cur[2]>100)
			{
				testM6_100us_ac_ab_bc_cur =	(M6_100us_ac_ab_bc_cur[2]-M6_100us_ac_ab_bc_cur[0])/M6_100us_ac_ab_bc_cur[2];//测试观察值
				if(testM6_100us_ac_ab_bc_cur>0.9) //从小到大排序之后最大值-最小值，0.5为测试估计值
				{
					Miss6_Phase_Count++;
					if(Miss6_Phase_Count>2)
					{
						MotorControl[6].Max_Phase_Lack=testM6_100us_ac_ab_bc_cur*1000;  //记录发生的时候的缺相检测最大值
						MC_SetFault(MOTOR6_MISSING_PHASE); //单纯为了指示灯
						MotorControl[6].Fault_Flag =1;
						MotorControl[6].Motor_Start_Stop = 0;
					}
				}

				else
				{
					if(Miss6_Phase_Count>0)
					{
						Miss6_Phase_Count--;
					}
				}
			}
            M6_100us_ac_ab_bc_cur[ab] = 0;
            M6_100us_ac_ab_bc_cur[ac] = 0;
            M6_100us_ac_ab_bc_cur[bc] = 0;
        }
		if(MotorControl[6].Direction==1||MotorControl[6].Direction==-1)        //加入方向判断
		{
			if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[4]||\
					MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[1])  //BC  CB 项电流采样
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
    }
    else
    {
        M6_100us_ac_ab_bc_cur[ab] = 0;
        M6_100us_ac_ab_bc_cur[ac] = 0;
        M6_100us_ac_ab_bc_cur[bc] = 0;
        if(Motor6_Current_temp.cnt > 0)
        {
            Motor6_Current_temp.cnt --;
        }
    }
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
    if(OverFlow_Cnt[2]>=BREAK_CNT_MAX)	/* 单向有刷3——4 */
    {
        MC_SetFault(BRAKE_3_4);
        MotorControl[3].Motor_Start_Stop = DISABLE;
        MotorControl[3].Fault_Flag = 1;
        MotorControl[4].Motor_Start_Stop = DISABLE;
        MotorControl[4].Fault_Flag = 1;
    }
    if(OverFlow_Cnt[3]>=BREAK_CNT_MAX)	/* 推杆0_1_2_9 */
    {
        MC_SetFault(BRAKE_0_1_2_9);
        MotorControl[0].Motor_Start_Stop = DISABLE;
        MotorControl[1].Motor_Start_Stop = DISABLE;
        MotorControl[2].Motor_Start_Stop = DISABLE;
        MotorControl[9].Motor_Start_Stop = DISABLE;
        MotorControl[0].Fault_Flag = 1;
        MotorControl[1].Fault_Flag = 1;
        MotorControl[2].Fault_Flag = 1;
        MotorControl[9].Fault_Flag = 1;
    }
    if(OverFlow_Cnt[4]>=BREAK_CNT_MAX)	/* 有刷10_11_12_13 */
    {
        MC_SetFault(BRAKE10_11_12_13);
        MotorControl[10].Motor_Start_Stop = DISABLE;
        MotorControl[11].Motor_Start_Stop = DISABLE;
        MotorControl[12].Motor_Start_Stop = DISABLE;
        MotorControl[13].Motor_Start_Stop = DISABLE;
        MotorControl[10].Fault_Flag = 1;
        MotorControl[11].Fault_Flag = 1;
        MotorControl[12].Fault_Flag = 1;
        MotorControl[13].Fault_Flag = 1;
    }
}
/*------------------------------------------------
Function:偏置电压校准
Input   :No
Output  :No
Explain :以下两个数组操作，为了对应电流采样通道对应电机电流
------------------------------------------------*/
u8 Voltage_offset_calibration[8] = {0,1,2,3,4,5,6,9};//上下对起来看
u8 ADC_ConvertedValueindex[8] =    {6,7,8,0,1,2,4,9};//上下对起来看
#define OFFSET_NUM 100
u8 offset_cnt = 0;
void Voltage_offset_cali(void)
{
    offset_cnt++;
    if(offset_cnt<=OFFSET_NUM)
    {
        for(int i = 0; i < 8; i++)
        {
            MotorControl[Voltage_offset_calibration[i]].Current.offset += ADC_ConvertedValue[ADC_ConvertedValueindex[i]];
        }
        if(offset_cnt == OFFSET_NUM)		/* 取100次的均值 */
        {
            for(int k = 0; k < 8; k++)
            {
                MotorControl[Voltage_offset_calibration[k]].Current.offset = (MotorControl[Voltage_offset_calibration[k]].Current.offset/OFFSET_NUM);
            }
            MotorState = INIT;
        }
    }
    else if(offset_cnt>OFFSET_NUM)
    {
        offset_cnt = OFFSET_NUM+1;
        MotorState = INIT;
    }
}
/*************************
*Function Name 		: BLDC5_Phase_Check
*Description   		: 相序错误检查
* Input           : None
* Output          : None
* Return          : None		2021.11.30	by diamond
*************************/
u8 motor_err_cnt5 = 0;
void BLDC5_Phase_Check(void)
{
  if(Motor_Type<2)
   {
     if(ABS(MotorControl[5].PWM_Duty)>4000&&MotorControl[5].Motor_Start_Stop==1)
    {
        if(MotorControl[5].Direction!=-1&&MotorControl[5].Direction!=1)
        {
            motor_err_cnt5++;
            if(motor_err_cnt5>60)		//累计60次报警
            {
                motor_err_cnt5=0;
                MotorControl[5].Motor_Start_Stop = 0;
                MC_SetFault(MOTOR5_PHASE_ERROR);
                MotorControl[5].Fault_Flag =1;
            }
        }
        else if(ABS(MotorControl[5].Speed_Set-MotorControl[5].Speed_Real)<50)
        {
            if(motor_err_cnt5>0)
                motor_err_cnt5--;
        }
    }
  }
}
/*************************
*Function Name 		: BLDC6_Phase_Check
*Description   		: 相序错误检查
* Input           : None
* Output          : None
* Return          : None		2021.11.30	by diamond
*************************/
u8 motor_err_cnt6 = 0;
void BLDC6_Phase_Check(void)
{
 if(Motor_Type<2)
  {
    if(ABS(MotorControl[6].PWM_Duty)>4000&&MotorControl[6].Motor_Start_Stop==1)
    {
        if(MotorControl[6].Direction!=-1&&MotorControl[6].Direction!=1)
        {
            motor_err_cnt6++;
            if(motor_err_cnt6>60)//20次报警
            {
                MC_SetFault(MOTOR6_PHASE_ERROR);
                motor_err_cnt6=0;
                MotorControl[6].Motor_Start_Stop = 0;
                MotorControl[6].Fault_Flag =1;
            }
        }
        else if(ABS(MotorControl[6].Speed_Set-MotorControl[6].Speed_Real)<50)
        {
            if(motor_err_cnt6>0)
			{
                motor_err_cnt6--;
			}
        }
    }
  }
}
/*************************
*Function Name 		:Push_Motor_CurSelfAdapt
*Description   		:推杆电流根据无刷电机实现高度自适应
* Input           : bldc_num ： 无刷电机号
										push_num ： 推杆电机号
										hall_max ： 霍尔下放最大值
										hall_min ： 霍尔最小回收值
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
float cur_ratio[7] = {0,0,0,0,0,MOTOR5CURCOEFFICIENT,MOTOR6CURCOEFFICIENT};
int16_t limitpwm = 4000;
uint8_t hight_err_cnt = 0;
int CurSelfAdapt_Value=20;  //偏差值0.2A
int CurSelfAdapt_Time=2000; //滤波时间1s
int CurSelfAdapt_Convert_Time;
void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min)             //15ms一次
{
    CurSelfAdapt_Convert_Time=CurSelfAdapt_Time/15;
    if(MotorControl[bldc_num].Current.SetValue!=0)
    {
        if(MotorControl[bldc_num].Speed_Real>200) //:2022.4.28 readme解释
        {
            if(MotorControl[push_num].Motor_Start_Stop ==DISABLE)
                MotorControl[push_num].Motor_Start_Stop = ENABLE;
            if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])<MotorControl[bldc_num].Current.CurOffset*cur_ratio[bldc_num] )
            {
                MotorControl[push_num].PWM_DutySet = 0;
                if(hight_err_cnt>0)
                    hight_err_cnt--;
            }
            else
            {
                if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])>(MotorControl[bldc_num].Current.CurOffset+CurSelfAdapt_Value)*cur_ratio[bldc_num])
                    hight_err_cnt++;
                if(hight_err_cnt>CurSelfAdapt_Convert_Time)          //200对应3s
                {
                    hight_err_cnt=CurSelfAdapt_Convert_Time;
                    MotorControl[push_num].PWM_DutySet = PID_Regulator(MotorControl[bldc_num].Current.SetValue,MotorControl[bldc_num].Current.DeepFilterVAL,&PID_PWM);
                    if(f_abs(MotorControl[push_num].PWM_DutySet)<1000)
                    {
                        MotorControl[push_num].PWM_DutySet = 0;
                    }
                }
            }
            if(MotorControl[push_num].Hall.HALL_CaptureValue>hall_max&&MotorControl[push_num].PWM_DutySet>0)    MotorControl[push_num].PWM_DutySet = 0;
            if(MotorControl[push_num].Hall.HALL_CaptureValue<hall_min&&MotorControl[push_num].PWM_DutySet<0)    MotorControl[push_num].PWM_DutySet = 0;
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
*Function Name 		:BLDC_Stuck_Chk
*Description   		:BLDC堵转保护  500ms无动作报警
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
int16_t Bldc_lack_flag=0;
u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
void BLDC_Stuck_Chk(void)			/* 1ms循环里 */
{
//    static u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
    if(GetMotorSpeed(Motor5)==0&&ABS(MotorControl[5].PWM_Duty)>4000&&MotorControl[5].Motor_Start_Stop==1)	//BLDC1带载时缺相，电流为0
    {
        if(MotorControl[5].Current.DeepFilterVAL < 10) //基本可以判断为带载时缺相，因为那一相是断开的没有电流
        {
            M5phase_check_cnt++;//带载缺相计数
        }
        else if(MotorControl[5].Current.FilterValue > 700)
        {
            M5stuck_check_cnt++;//堵转计数
        }

        if(M5phase_check_cnt>PHASE_ERR_MAX)
        {
			MotorControl[5].Start_Phase_lack_cnt=M5phase_check_cnt;
            Bldc_lack_flag=1;
            MC_SetFault(MOTOR5_MISSING_PHASE);//M5缺相
            M5phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[5].Fault_Flag =1;
            MotorControl[5].Motor_Start_Stop = 0;

        }
        if(M5stuck_check_cnt>SUTCK_ERR_MAX)
        {
			MotorControl[5].Stuck_time=M5stuck_check_cnt;
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

    if(GetMotorSpeed(Motor6)==0&&ABS(MotorControl[6].PWM_Duty)>4000&&MotorControl[6].Motor_Start_Stop==1)	//BLDC1带载时缺相，电流为0
    {
        if(MotorControl[6].Current.DeepFilterVAL < 10) //基本可以判断为带载时缺相，因为那一相是断开的没有电流
        {
            M6phase_check_cnt++;//带载缺相计数
        }
        else if(MotorControl[6].Current.FilterValue > 700)
        {
            M6stuck_check_cnt++;//堵转计数
        }

        if(M6phase_check_cnt>PHASE_ERR_MAX)
        {
			MotorControl[6].Start_Phase_lack_cnt=M6phase_check_cnt;
            MC_SetFault(MOTOR6_MISSING_PHASE);//M6缺相
            M6phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[6].Fault_Flag =1;
            MotorControl[6].Motor_Start_Stop = 0;

        }
        if(M6stuck_check_cnt>SUTCK_ERR_MAX)
        {
			MotorControl[6].Stuck_time=M6stuck_check_cnt;
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
   if(Motor_Type<2)
  {
    static u16 speed_err_cnt = 0;
    if((MotorControl[5].Speed_Ref>0&&MotorControl[5].Speed_Real<0)||(MotorControl[5].Speed_Ref<0&&MotorControl[5].Speed_Real>0))		/* 反向转 */
        speed_err_cnt++;
    else if(MotorControl[5].Speed_Ref==0&&ABS(MotorControl[5].Speed_Set)<500)
        speed_err_cnt=0;
    else if(ABS(MotorControl[5].Speed_Real)-ABS(MotorControl[5].Speed_Ref)>500)																								/* 实际>设定 差的绝对值->绝对值的差，修复重载导致降速问题*/
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
}
void BLDC2_OverSpdChk(void)
{
 if(Motor_Type<2)
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
void Push_OverRunChk(u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3)
{
   
    if(push1)
    {
        if(MotorControl[0].Motor_Start_Stop==ENABLE&&ABS(MotorControl[0].PWM_Duty)>500)
        {
            timer_cnt[0]++;
            if(timer_cnt[0]>timer0*2)
            {
                MotorControl[0].Motor_Start_Stop=DISABLE;
//                MC_SetFault(PUSH_OVER_TIME);
                MC_SetFault1(PUSH0_OVERTIME);
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
//                MC_SetFault(PUSH_OVER_TIME);
                MC_SetFault1(PUSH1_OVERTIME);
                timer_cnt[1]=0;
            }
        }
        else timer_cnt[1]=0;
    }
    if(push3)
    {
        if(MotorControl[2].Motor_Start_Stop==ENABLE&&ABS(MotorControl[2].PWM_Duty)>500)
        {
            timer_cnt[2]++;
            if(timer_cnt[2]>timer2*2)
            {
                MotorControl[2].Motor_Start_Stop=DISABLE;
//                MC_SetFault(PUSH_OVER_TIME);
                MC_SetFault1(PUSH2_OVERTIME);
                timer_cnt[2]=0;
            }
        }
        else timer_cnt[2]=0;
    }
    if(push4)
    {
        if(MotorControl[9].Motor_Start_Stop==ENABLE&&ABS(MotorControl[9].PWM_Duty)>500)
        {
            timer_cnt[3]++;
            if(timer_cnt[3]>timer3*2)
            {
                MotorControl[9].Motor_Start_Stop=DISABLE;
//                MC_SetFault(PUSH_OVER_TIME);
                MC_SetFault1(PUSH9_OVERTIME);
                timer_cnt[3]=0;
            }
        }
        else timer_cnt[3]=0;
    }
}



void Bldc5_Hall_Check(void)   //hall错误函数
{
	 if(MotorControl[5].Hall.HallState == 0||MotorControl[5].Hall.HallState == 7 )
		{
			if(Motor_Type<2)
			{
				MotorControl[5].Hall_Error_Syscnt++;
				if(MotorControl[5].Hall_Error_Syscnt>3)   //3次hall不低则上报霍尔错误
				{
				   MC_SetFault(HALL5_SENSOR_ERR);
				   MotorControl[5].Hall_Error_Syscnt=0;
				}
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
void Bldc6_Hall_Check(void)   //hall错误函数
   {
		if(MotorControl[6].Hall.HallState == 0||MotorControl[6].Hall.HallState == 7 )
		{
			if(Motor_Type<2)
			{
				MotorControl[6].Hall_Error_Syscnt++;
				if(MotorControl[6].Hall_Error_Syscnt>3)   //3次hall不低则上报霍尔错误
				{
					 MC_SetFault(HALL6_SENSOR_ERR);
					 MotorControl[6].Motor_Start_Stop = DISABLE;
				}
			}
		}
		else
		{
			if(MotorControl[6].Hall_Error_Syscnt>0)
			{
				MotorControl[6].Hall_Error_Syscnt--;
			}
		}
    }
