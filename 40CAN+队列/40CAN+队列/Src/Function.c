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
DefaultPhaseCheekCurTemp Motor5_Current_temp;
DefaultPhaseCheekCurTemp Motor6_Current_temp;
extern PID_Struct_t	 PID_PWM;
void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min);
void Push_OverRunChk(u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2);
void Motor7_Err_Chk(void);
extern void Motor_Fault_Clear(void);
extern int8_t OverFlow_Cnt[5];
/*------------------------------------------------
Function:MCU版本号
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver)
{
	pVersion->uVersionPartOfRobotType = robotype;				//项目类型	40线
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
	pVersion->HardwarePartOfMotorType = funtype;				//电机类型：混合H,伺服S,步进B
	pVersion->HardwarePartOfVotage = vol;								//工作电压	A：24V
	pVersion->HardwarePartOfCurrent = cur_max;					//最大电流
	pVersion->HardwarePartOfVersion = update_ver;				//更新的版本号	
	pVersion->HardwareFullVersion = (funtype<<24)+(vol<<16)+(cur_max<<8)+update_ver;
}
/*------------------------------------------------
Function:电流保护限制
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void CurrentLimit(u8 Motor_NUM)              //过流保护
{
    if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue1)
    {
        MotorControl[Motor_NUM].Current.OFCnt1++;
        if(MotorControl[Motor_NUM].Current.OFCnt1 > MotorControl[Motor_NUM].Current.OFCnt1_T)
        {
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

u8 Need_To_Stuck_Function[MOTOR_NUM] = {0,0,0,0,0,1,1,0,0,0,0,0,0,0};  //哪个电机需要堵转保护就置1
//uint16_t Stuck_Current = 0;
int32_t  Stuck_Speed = 50;
uint32_t Stuck_Time = 100000;
uint16_t Stuck_TimeCnt[MOTOR_NUM] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//uint16_t Unit_Of_Current[MOTOR_NUM] = {637,637,637,316,316,253,253,0,0,637,0,0,0,0};
float Percentage_Of_Current[MOTOR_NUM] = {0.5,0.5,0.5,0.5,0.5,0.8,0.8,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
void MotorStuckCheck(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        CurrentLimit(i);
    }

}
void Over_VoltageCheck(void)  //10ms  过压和欠压保护
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
/*************************
*Function Name 		:Push_Motor_CurSelfAdapt
*Description   		:推杆电流根据无刷电机实现高度自适应  
* Input           : u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min 
										无刷电机号，有刷电机号，自适应最大高度，自适应最小高度
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
float cur_ratio[7] = {0,0,0,0,0,1,1.08f};
u16 hight_err_cnt = 0;
int Pushchangflag;
int CurSelfAdapt_Value=20;  //偏差值0.2A
int CurSelfAdapt_Time=3000; //滤波时间3s
int CurSelfAdapt_Convert_Time;

void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min)             //15ms一次，滚刷自适应
{
	CurSelfAdapt_Convert_Time=CurSelfAdapt_Time/15;
	if(MotorControl[bldc_num].Current.SetValue!=0)   //设置电流值不为0的时候
	{
		if(MotorControl[push_num].Motor_Start_Stop ==DISABLE)
		 MotorControl[push_num].Motor_Start_Stop = ENABLE;
		if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])<MotorControl[bldc_num].Current.CurOffset*cur_ratio[bldc_num] )  //电流偏差小于0.2A
		{
			MotorControl[push_num].PWM_DutySet = 0;
			if(hight_err_cnt>0)
			hight_err_cnt--;
		}

		else 
		{
			if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])>(MotorControl[bldc_num].Current.CurOffset+CurSelfAdapt_Value)*cur_ratio[bldc_num] )     //电流偏差大于0.2A
			hight_err_cnt++;
			if(hight_err_cnt>CurSelfAdapt_Convert_Time)          //200对应3s
			{
				hight_err_cnt=CurSelfAdapt_Convert_Time;
				MotorControl[push_num].PWM_DutySet = PID_Regulator(MotorControl[bldc_num].Current.DeepFilterVAL,MotorControl[bldc_num].Current.SetValue,&PID_PWM);  //滚刷自适应PID控制
				if(f_abs(MotorControl[push_num].PWM_DutySet)<1000)    
				 {	
					 MotorControl[push_num].PWM_DutySet = 0;
				 }
			}
		}
		if(MotorControl[push_num].Hall.HALL_CaptureValue>hall_max&&MotorControl[push_num].PWM_DutySet<0)		MotorControl[push_num].PWM_DutySet = 0;//推杆到顶了
		if(MotorControl[push_num].Hall.HALL_CaptureValue<hall_min&&MotorControl[push_num].PWM_DutySet>0)		MotorControl[push_num].PWM_DutySet = 0;
	}
}

//void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min)
//{
//    if(MotorControl[bldc_num].Current.SetValue!=0)
//    {
//        if(MotorControl[push_num].Motor_Start_Stop ==DISABLE)
//        {
//            MotorControl[push_num].Motor_Start_Stop = ENABLE;
//        }
//        if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])<MotorControl[bldc_num].Current.CurOffset*cur_ratio[bldc_num])
//        {
//            MotorControl[push_num].PWM_DutySet = 0;
//        }
//        else
//        {
//            MotorControl[push_num].PWM_DutySet = PID_Regulator(MotorControl[bldc_num].Current.DeepFilterVAL,MotorControl[bldc_num].Current.SetValue,&PID_PWM);
//            if(MotorControl[push_num].PWM_DutySet<1000&&MotorControl[push_num].PWM_DutySet>-1000)
//            {
//                MotorControl[push_num].PWM_DutySet = 0;
//            }
//        }
//        if(MotorControl[push_num].Hall.HALL_CaptureValue>hall_max&&MotorControl[push_num].PWM_DutySet>0)
//        {
//            MotorControl[push_num].PWM_DutySet = 0;
//        }
//        if(MotorControl[push_num].Hall.HALL_CaptureValue<hall_min&&MotorControl[push_num].PWM_DutySet<0)
//        {
//            MotorControl[push_num].PWM_DutySet = 0;
//        }
//    }
//}
/*------------------------------------------------
Function:参数初始化
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Init_Drive_Para(void)
{
    

    VoltVar.HighGuardTime  = 200;   //4月9号修改
    VoltVar.LowGuardTime  = 200;
    VoltVar.BusHigh = VOLT_360V ;
    VoltVar.BusLow =  VOLT_160V ;
//    TIM3->CCR1 = 0;
//    TIM8->CCR4 = 8400;
//	  MotorControl[10].PWM_DutySet = 8400;
//    MotorControl[12].PWM_DutySet = 8400;
//    MotorControl[10].Motor_Start_Stop = 1;
//    MotorControl[12].Motor_Start_Stop = 1;

    MotorControl[0].Current.MaxValue1 = MOTOR0_CurrentValue(1) ;
    MotorControl[0].Current.MaxValue2 = MOTOR0_CurrentValue(2) ;
    MotorControl[0].Current.MaxValue3 = MOTOR0_CurrentValue(3) ;
    MotorControl[0].Current.MaxValue4 = MOTOR0_CurrentValue(4) ;
    MotorControl[0].Current.OFCnt1_T = 200;
    MotorControl[0].Current.OFCnt2_T = 30000;
    MotorControl[0].Current.OFCnt3_T = 10000;
    MotorControl[0].Current.OFCnt4_T = 200;

    MotorControl[1].Current.MaxValue1 = MOTOR1_CurrentValue(1) ;
    MotorControl[1].Current.MaxValue2 = MOTOR1_CurrentValue(2) ;
    MotorControl[1].Current.MaxValue3 = MOTOR1_CurrentValue(3) ;
    MotorControl[1].Current.MaxValue4 = MOTOR1_CurrentValue(4) ;
    MotorControl[1].Current.OFCnt1_T = 200;
    MotorControl[1].Current.OFCnt2_T = 30000;
    MotorControl[1].Current.OFCnt3_T = 10000;
    MotorControl[1].Current.OFCnt4_T = 200;

    MotorControl[2].Current.MaxValue1 = MOTOR2_CurrentValue(1) ;   //921修改
    MotorControl[2].Current.MaxValue2 = MOTOR2_CurrentValue(1) ;
    MotorControl[2].Current.MaxValue3 = MOTOR2_CurrentValue(2) ;
    MotorControl[2].Current.MaxValue4 = MOTOR2_CurrentValue(3) ;
    MotorControl[2].Current.OFCnt1_T = 200;
    MotorControl[2].Current.OFCnt2_T = 20000;
    MotorControl[2].Current.OFCnt3_T = 8000;
    MotorControl[2].Current.OFCnt4_T = 500;		

    MotorControl[9].Current.MaxValue1 = MOTOR9_CurrentValue(3) ;
    MotorControl[9].Current.MaxValue2 = MOTOR9_CurrentValue(4) ;
    MotorControl[9].Current.MaxValue3 = MOTOR9_CurrentValue(5) ;
    MotorControl[9].Current.MaxValue4 = MOTOR9_CurrentValue(6) ;
    MotorControl[9].Current.OFCnt1_T = 200;
    MotorControl[9].Current.OFCnt2_T = 20;
    MotorControl[9].Current.OFCnt3_T = 8;
    MotorControl[9].Current.OFCnt4_T = 5;

    MotorControl[3].Current.MaxValue1 = MOTOR3_CurrentValue(7) ;  //过滤电机
    MotorControl[3].Current.MaxValue2 = MOTOR3_CurrentValue(8) ;
    MotorControl[3].Current.MaxValue3 = MOTOR3_CurrentValue(9) ;  //过滤电机
    MotorControl[3].Current.MaxValue4 = MOTOR3_CurrentValue(10) ;
    MotorControl[3].Current.OFCnt1_T = 200;
    MotorControl[3].Current.OFCnt2_T = 20;
    MotorControl[3].Current.OFCnt3_T = 8;
    MotorControl[3].Current.OFCnt4_T = 5;

    MotorControl[4].Current.MaxValue1 = MOTOR4_CurrentValue(7) ;  //喷水电机（S线）
    MotorControl[4].Current.MaxValue2 = MOTOR4_CurrentValue(8) ;
    MotorControl[4].Current.MaxValue3 = MOTOR4_CurrentValue(9) ;  //喷水电机（S线）
    MotorControl[4].Current.MaxValue4 = MOTOR4_CurrentValue(10) ;
    MotorControl[4].Current.OFCnt1_T = 200;
    MotorControl[4].Current.OFCnt2_T = 20;
    MotorControl[4].Current.OFCnt3_T = 8;
    MotorControl[4].Current.OFCnt4_T = 5;

    MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(10) ;  //滚刷电机（40）
    MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(14) ;
    MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(14) ;
    MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(14) ;
	MotorControl[5].Current.CurOffset = 30;		//默认0.4A
    MotorControl[5].Current.OFCnt1_T = 5000;
    MotorControl[5].Current.OFCnt2_T = 200;
    MotorControl[5].Current.OFCnt3_T = 40;
    MotorControl[5].Current.OFCnt4_T = 20;
    MotorControl[5].Pole_Paires = 4;
    MotorControl[5].Acceleration = 5;
    MotorControl[5].Deceleration = 5;

    MotorControl[6].Current.MaxValue1 = MOTOR6_CurrentValue(7) ;
    MotorControl[6].Current.MaxValue2 = MOTOR6_CurrentValue(9) ;
    MotorControl[6].Current.MaxValue3 = MOTOR6_CurrentValue(10) ;
    MotorControl[6].Current.MaxValue4 = MOTOR6_CurrentValue(11) ;
	MotorControl[6].Current.CurOffset = 20;		//默认0.2A
    MotorControl[6].Current.OFCnt1_T = 5000;
    MotorControl[6].Current.OFCnt2_T = 30000;
    MotorControl[6].Current.OFCnt3_T = 10000;
    MotorControl[6].Current.OFCnt4_T = 2000;
    MotorControl[6].Pole_Paires = 4;          //修改成4对极了
    MotorControl[6].Acceleration = 5;
    MotorControl[6].Deceleration = 5;
	/*************************无刷5，6end***************************/
	PID_Init (&PID_Speed_InitStruct[0],&PID_Current_InitStructure[0]);		//电机5的PID
	PID_Init (&PID_Speed_InitStruct[1],&PID_Current_InitStructure[1]);		//电机6的PID
	PID_PWM_Init(&PID_PWM);	

    MotorControl[0].Acceleration = 20;
    MotorControl[0].Deceleration = 20;
    MotorControl[0].LastMotorDirection = 2;
    MotorControl[0].Hall.HALL_CaptureValue = 10000; //用于上电标定
    MotorControl[1].Acceleration = 60;
    MotorControl[1].Deceleration = 60;
    MotorControl[1].LastMotorDirection = 2;
    MotorControl[1].Hall.HALL_CaptureValue = 10000; //用于上电标定
    MotorControl[2].Acceleration = 100;
    MotorControl[2].Deceleration = 100;
    MotorControl[2].LastMotorDirection = 2;
    MotorControl[2].Hall.HALL_CaptureValue = 10000; //用于上电标定

    MotorControl[3].Acceleration = 3;
    MotorControl[3].Deceleration = 3;

    MotorControl[4].Acceleration = 3;
    MotorControl[4].Deceleration = 3;

    MotorControl[7].Acceleration = 10;
    MotorControl[7].Deceleration = 10;

    MotorControl[8].Acceleration = 10;
    MotorControl[8].Deceleration = 10;

    MotorControl[9].Acceleration = 10;
    MotorControl[9].Deceleration = 10;
    MotorControl[9].LastMotorDirection = 2;
    MotorControl[9].Hall.HALL_CaptureValue = 10000; //用于上电标定

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
    MotorControl[5].Hall.ChangeFlag = 1; //电机停止之后需要换向启动
    MotorControl[6].Hall.HallState = HALL_GetPhase2();
    MotorControl[6].Hall.ChangeFlag = 1;


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

//    HALL_Study[1].HallTab[0] = 1;// 139测试车特用
//    HALL_Study[1].HallTab[1] = 3;
//    HALL_Study[1].HallTab[2] = 2;
//    HALL_Study[1].HallTab[3] = 6;
//    HALL_Study[1].HallTab[4] = 4;
//    HALL_Study[1].HallTab[5] = 5;
	
	HALL_Study[1].HallTab[0] = 6;// 除了139测试车其余车都用这个
    HALL_Study[1].HallTab[1] = 4;
    HALL_Study[1].HallTab[2] = 5;
    HALL_Study[1].HallTab[3] = 1;
    HALL_Study[1].HallTab[4] = 3;
    HALL_Study[1].HallTab[5] = 2;
	
//	HALL_Study[1].HallTab[0] = 6;// 测试架
//    HALL_Study[1].HallTab[1] = 2;
//    HALL_Study[1].HallTab[2] = 3;
//    HALL_Study[1].HallTab[3] = 1;
//    HALL_Study[1].HallTab[4] = 5;
//    HALL_Study[1].HallTab[5] = 4;
}
/*------------------------------------------------
Function:led灯设置
Input   :led1: 1/下面红灯亮  0/下面红灯不亮
				 led2: 1/上面红灯亮  0/上面红灯不亮
				 led_time:闪烁的次数
				 time_cnt:时间计数	都为DisplayCounter
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
		switch (FaultOccurred)
		{
		case PUSH_MOTOR1_OVER_CUR: //上面红灯闪烁1次（快闪）
				LEDSet(LED_OFF,LED_ON,1,0,LED_HIGH_SPEED);

				break;

		case PUSH_MOTOR2_OVER_CUR: //上面红灯闪烁2次
				LEDSet(LED_OFF,LED_ON,2,0,LED_HIGH_SPEED);

				break;

		case PUSH_MOTOR3_OVER_CUR: //上面红灯闪烁3次
				LEDSet(LED_OFF,LED_ON,3,0,LED_HIGH_SPEED);

				break;

		case PUSH_MOTOR4_OVER_CUR: //上面红灯闪烁4次
				LEDSet(LED_OFF,LED_ON,4,0,LED_HIGH_SPEED);


				break;
		case BLDC1_OVER_CUR: //上面红灯闪烁5次
				LEDSet(LED_OFF,LED_ON,5,0,LED_HIGH_SPEED);


				break;
		case BLDC2_OVER_CUR: //上面红灯闪烁6次
				LEDSet(LED_OFF,LED_ON,6,0,LED_HIGH_SPEED);


				break;
		case ONEWYA_MOTOR1_OVER_CUR:	//上面红灯闪烁7次
				LEDSet(LED_OFF,LED_ON,7,0,LED_HIGH_SPEED);


				break;
		case ONEWYA_MOTOR2_OVER_CUR:		//上面红灯闪烁8次
				LEDSet(LED_OFF,LED_ON,8,0,LED_HIGH_SPEED);
				break;

		case HALL5_SENSOR_ERR:    //下面红灯闪烁1次
				LEDSet(LED_ON,LED_OFF,1,1,LED_LOW_SPEED);		//慢闪

				break;

		case HALL6_SENSOR_ERR:		//上面红灯 慢闪1次
				LEDSet(LED_OFF,LED_ON,1,1,LED_LOW_SPEED);
				
				break;

		case OVER_VOLTAGE:  //两个灯常量
				RED_LED1_ON;
				RED_LED2_ON;
				DisplayCounter=0;
				break;
		case UNDER_VOLTAGE:   //欠压下面红灯亮
				RED_LED1_ON;
				RED_LED2_OFF;
				DisplayCounter=0;
				break;
		case MOTOR5_BREAK:		//下面快闪一次
				LEDSet(LED_ON,LED_OFF,1,0,LED_HIGH_SPEED);
				break;

		case MOTOR6_BREAK:		//下面快闪2次
				LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
				break;

		case BRAKE_3_4:				//下面快闪3次
				LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
				break;

		case BRAKE_0_1_2_9:		//下面快闪4次
				LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
				break;
		case BRAKE10_11_12_13:	//下面快闪5次
				LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
				break;
				
		//6次
				
		case MOTOR5_PHASE_ERROR:			//下面快闪7次		
				LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
				break;
				
		case MOTOR6_PHASE_ERROR:					//下面快闪8次	
				LEDSet(LED_ON,LED_OFF,8,0,LED_HIGH_SPEED);
				break;
		
		case CAN_COMMUNICATION_ERR:	//两个灯快闪1次
				LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
				break;
				
		case MOTOR5_MISSING_PHASE: //两个红灯快速闪烁2次
				LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
				break;

		case MOTOR6_MISSING_PHASE: //两个红灯快速闪烁3次
				LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);

				break;
		case MOTOR3_BRAKE_LINE:  //两个红灯快速闪烁4次
				LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);

				break;
		case MOTOR4_BRAKE_LINE: //两个红灯快速闪烁5次
				LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);

				break;		
		case SIDE_BRUSH_ERROR:  //两个红灯快速闪烁6次
				LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);

				break;
		case MOTOR5_OVER_SPEED:  //两个红灯快速闪烁7次
				LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);

				break;
		case MOTOR6_OVER_SPEED:  //两个红灯快速闪烁8次
				LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);

				break;
		case PUSH_BRAKE_LINE:  //推杆断线		下面常亮，上面快速闪烁1-4次
//				switch (FaultOccurred1)
//				{
//					case PUSH0_BRAKE_LINE:
//						LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
//						break;
//					case PUSH1_BRAKE_LINE:
//						LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
//						break;
//					case PUSH2_BRAKE_LINE:
//						LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);
//						break;
//					case PUSH9_BRAKE_LINE:
//						LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);
//						break;
//				}
//				RED_LED1_ON;

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
		case FAN_ERROR:			//风机错误上面常亮，下面1次
				LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
				RED_LED2_ON;
				break;
		case PUSHMOTOR_INT_ERROR:		//推杆标定失败上面常亮，下面2次
				LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
				RED_LED2_ON;
						break;
		 case PUSH_MOTOR_OVERTIME:		//推杆标定失败上面常亮，下面6-9次
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
/*------------------------------------------------
Function:推杆电机位置环
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Push_Motor_Location_Control(void)       //推杆位置环
{
    //			 Ramp_PPWM(1);
//			 MotorControl[1].PWM_DutySet = -PID_Controller( MotorControl[1].Location_Set, MotorControl[1].Hall.HALL_CaptureValue,&PID_Location_InitStruct[1]);
//			 SetMotorSpeed(1, MotorControl[1].PWM_DutySet);
    if(MotorControl[0].Hall.HALL_CaptureValue > MotorControl[0].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
    {
        MotorControl[0].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;   //3.30修改——捷昌电机
    }
    else if(MotorControl[0].Hall.HALL_CaptureValue < MotorControl[0].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
    {
        MotorControl[0].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
    }
    else if(MotorControl[0].Hall.HALL_CaptureValue == MotorControl[0].Location_Set)//||fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) < 5)
    {
        MotorControl[0].PWM_DutySet = 0;
    }
	if(MotorControl[6].Current.SetValue ==0)	//没有做电流自适应的情况
	{
		if(MotorControl[1].Hall.HALL_CaptureValue > MotorControl[1].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
		{
			MotorControl[1].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
		}
		else if(MotorControl[1].Hall.HALL_CaptureValue < MotorControl[1].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
		{
			MotorControl[1].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
		}
		else if(MotorControl[1].Hall.HALL_CaptureValue == MotorControl[1].Location_Set)//||fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) < 5)
		{
			MotorControl[1].PWM_DutySet = 0;
		}
	}

    if(MotorControl[2].Hall.HALL_CaptureValue > MotorControl[2].Location_Set)// && fabs(MotorControl[2].Hall.HALL_CaptureValue- MotorControl[2].Location_Set) > 5)
    {
        MotorControl[2].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;  //40边刷推杆升起
    }
    else if(MotorControl[2].Hall.HALL_CaptureValue < MotorControl[2].Location_Set)// && fabs(MotorControl[2].Hall.HALL_CaptureValue- MotorControl[2].Location_Set) > 5)
    {
        MotorControl[2].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;   //40边刷推杆下放
    }
    else if(MotorControl[2].Hall.HALL_CaptureValue == MotorControl[2].Location_Set)//||fabs(MotorControl[2].Hall.HALL_CaptureValue- MotorControl[2].Location_Set) < 5)
    {
        MotorControl[2].PWM_DutySet = 0;
    }

    if(MotorControl[9].Hall.HALL_CaptureValue > MotorControl[9].Location_Set)// && fabs(MotorControl[2].Hall.HALL_CaptureValue- MotorControl[2].Location_Set) > 5)
    {
        MotorControl[9].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
    }
    else if(MotorControl[9].Hall.HALL_CaptureValue < MotorControl[9].Location_Set)// && fabs(MotorControl[2].Hall.HALL_CaptureValue- MotorControl[2].Location_Set) > 5)
    {
        MotorControl[9].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
    }
    else if(MotorControl[9].Hall.HALL_CaptureValue == MotorControl[9].Location_Set)//||fabs(MotorControl[2].Hall.HALL_CaptureValue- MotorControl[2].Location_Set) < 5)
    {
        MotorControl[9].PWM_DutySet = 0;
    }
}
/*------------------------------------------------

Function:4路推杆电机霍尔检测和断联检测

Input   :No

Output  :No

Explain :No

------------------------------------------------*/
u8 NeedToCheekPush[MOTOR_NUM] = {0,1,0,2,2,0,0,0,0,0,0,0,0,0}; //如果该推杆电机需要霍尔错误或者推杆断联检测 0 1 2 9号电机默认都打开检测
u8 NeedToCheekPush1[MOTOR_NUM] = {3,4,5,2,2,0,0,0,0,0,0,0,0,0}; //如果该推杆电机需要霍尔错误或者推杆断联检测 0 1 2 9号电机默认都打开检测
u8 NeedToMissingHallPush[MOTOR_NUM] = {0,1,0,0,0,0,0,0,0,0,0,0,0,0}; //该数组来确保推杆是否需要霍尔保护
uint16_t PushMaxPulse[MOTOR_NUM] = {500,600,4000,0,0,0,0,0,0,4000,0,0,0,0}; //这个数组需要确定整机推杆的最大值
u32 Push_Error_Cheek_Cnt[MOTOR_NUM] = {0},Push_Error_Cheek_Cnt_1[MOTOR_NUM] = {0};
int Push_curr_temp[MOTOR_NUM] = {0},Push_curr_temp_1[MOTOR_NUM] = {0};
#define PUSH_PROTECT_MIN_VAL 3000
#define PUSH_PROTECT_MIN_CUR  50
#define MOTOR34_PROTECT_MIN_CUR 30
#define MOTOR34_PROTECT_MIN_VAL 399-1
u8 CurChkArray[2]= {0};
u8 CurChkArray1[2]= {0};
u8 CurChkArray2[2]= {0};
void Push_Motor_Cheek(void)        
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        if(NeedToCheekPush[i]==1) //如果该推杆电机需要霍尔错误或者推杆断联检测 0 1 2 9号电机默
        {
            Push_Error_Cheek_Cnt[i]++;
            if(Push_Error_Cheek_Cnt[i]%2002000==0) //51和100的最小公倍数时，会同时执行Push_Error_Cheek_Cnt[i]%51 和 Push_Error_Cheek_Cnt[i]%100==0这里剔除这种情况
            {
                Push_Error_Cheek_Cnt[i]++;
            }
            Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
            /*占空比大于999才保护*/
            if(MotorControl[i].Motor_Start_Stop == 1 && (l_abs(MotorControl[i].Location_Set-MotorControl[i].Hall.HALL_CaptureValue))>20 && \
							l_abs(MotorControl[i].PWM_Duty) > PUSH_PROTECT_MIN_VAL && MotorControl[i].Hall.HALL_CaptureValue < PushMaxPulse[i])
            {
                if(Push_Error_Cheek_Cnt[i]%1001==0) //100.1ms将捕获值更新一次，并进行一次断线检测
                {
                    MotorControl[i].Hall.HALL_CaptureValueDelta = MotorControl[i].Hall.HALL_CaptureValue;
//                    if(Push_curr_temp[i] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
//                    {
//                        MotorControl[i].Motor_Start_Stop = 0;   //断线错误
////											  MotorControl[i].Fault_Flag = 1;  //断线错误可以不置错误标志位，可以继续使能
//                        if(i == 0)
//                        {
//                            MC_SetFault1(PUSH0_BRAKE_LINE);//具体哪一个电机错误查看 第二个32位全局错误 wGlobal_Flags1
//                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
//                        }
//                        if(i == 1)
//                        {
//                            MC_SetFault1(PUSH1_BRAKE_LINE);
//                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
//                        }
//                        if(i == 2)
//                        {
//                            MC_SetFault1(PUSH2_BRAKE_LINE);
//                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
//                        }
//                        if(i == 9)
//                        {
//                            MC_SetFault1(PUSH9_BRAKE_LINE);
//                            MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
//                        }
//                    }
                }
                if(NeedToMissingHallPush[i]) //需要霍尔保护的推杆，默认都需要
                {
                    if((Push_Error_Cheek_Cnt[i]%2000==0)&&(MotorControl[i].Hall.HALL_CaptureValueDelta == \
											MotorControl[i].Hall.HALL_CaptureValue)&&Push_curr_temp[i] >= PUSH_PROTECT_MIN_CUR) //如果200ms内推杆的霍尔值没有改变，那就说明推杆霍尔异常
                    {
                        MotorControl[i].Motor_Start_Stop = 0;   //霍尔错误
                        MotorControl[i].Fault_Flag = 1; //之后需要重新进行推杆标定，所以需要手动清除
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
                        }
                    }
                }
            }
            else
            {
                Push_Error_Cheek_Cnt[i] = 0;
            }
            if(Push_Error_Cheek_Cnt[i]%1001==0)  //100.1ms清一次
            {
                Push_curr_temp[i] = 0;
            }
        }
//        if(NeedToCheekPush[i]==2)  //电机3电机4断线检测
//        {
//            Push_Error_Cheek_Cnt[i]++;
//            Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
//            if(MotorControl[i].Motor_Start_Stop == 1 && l_abs(MotorControl[i].PWM_Duty) > MOTOR34_PROTECT_MIN_VAL) //占空比大于2999才保护
//            {
//                if(Push_Error_Cheek_Cnt[i]%(100*(TIM10->PSC+1)/10)==0) //10个水泵周期进行一次断线检测
//                {
//                    if(Push_curr_temp[i] < MOTOR34_PROTECT_MIN_CUR)
//                    {
//                        MotorControl[i].Motor_Start_Stop = 0;
//                        if(i == 3)
//                        {
//                            MC_SetFault(MOTOR3_BRAKE_LINE);
//                        }
//                        if(i == 4)
//                        {
//                            MC_SetFault(MOTOR4_BRAKE_LINE);
//                        }
//                    }
//                    Push_curr_temp[i] = 0;
//                }
//            }
//            else
//            {
//                Push_Error_Cheek_Cnt[i] = 0;
//            }
//            if(Push_Error_Cheek_Cnt[i]%(100*(TIM10->PSC+1)/10)==0)  //1000ms清一次
//            {
//                Push_curr_temp[i] = 0;
//            }
//        }
   
            if(NeedToCheekPush1[i]==3)
        {
            if((MotorControl[0].Motor_Start_Stop == 1)&&(MotorControl[0].Push_motor_calibrationFLAG==2))
            {
                if(MotorControl[0].PWM_Duty>PUSH_PROTECT_MIN_VAL)
                {
                    Push_Error_Cheek_Cnt[0]++;
                    Push_curr_temp[0] += MotorControl[0].Current.FilterValue;
                    Push_Error_Cheek_Cnt_1[0] =0;
                    Push_curr_temp_1[0] = 0;
                    if(Push_Error_Cheek_Cnt[0]%10000==0)
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
                    Push_Error_Cheek_Cnt[0] =0;
                    Push_curr_temp[0] = 0;
                    if(Push_Error_Cheek_Cnt_1[0]%10000==0)
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
		
		   if(NeedToCheekPush1[i]==4)
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
		   if(NeedToCheekPush1[i]==5)
        {
            if(MotorControl[2].Motor_Start_Stop == 1)
            {
                if(MotorControl[2].PWM_Duty>PUSH_PROTECT_MIN_VAL)
                {
                    Push_Error_Cheek_Cnt[2]++;
                    Push_curr_temp[2] += MotorControl[2].Current.FilterValue;
                    Push_Error_Cheek_Cnt_1[2] =0;
                    Push_curr_temp_1[2] = 0;
                    if(Push_Error_Cheek_Cnt[2]%10000==0)
                    {
                        if(Push_curr_temp[2] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
                        {
                            CurChkArray2[0] =1;
                        }
                        else
                        {
                            CurChkArray2[0] =0;
                            CurChkArray2[1] =0;
                        }
                        Push_Error_Cheek_Cnt[2]=0;
                        Push_curr_temp[2] =0;
                    }
                }
                else if(MotorControl[2].PWM_Duty<-PUSH_PROTECT_MIN_VAL)
                {
                    Push_Error_Cheek_Cnt_1[2]++;
                    Push_curr_temp_1[2] += MotorControl[2].Current.FilterValue;
                    Push_Error_Cheek_Cnt[2] =0;
                    Push_curr_temp[2] = 0;
                    if(Push_Error_Cheek_Cnt_1[2]%10000==0)
                    {
                        if(Push_curr_temp_1[2] < PUSH_PROTECT_MIN_CUR)  //断线检测判断
                        {
                            CurChkArray2[1] =1;
                        }
                        else
                        {
                            CurChkArray2[1] =0;
                            CurChkArray2[0] =0;
                        }
                        Push_Error_Cheek_Cnt_1[2]=0;
                        Push_curr_temp_1[2] =0;
                    }
                }
                if(CurChkArray2[0]==1&&CurChkArray2[1]==1)
                {
                    MotorControl[2].Motor_Start_Stop = DISABLE;   //断线错误
                    MotorControl[2].Fault_Flag = 1;  //断线错误
                    MC_SetFault1(PUSH2_BRAKE_LINE);
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
uint32_t Sys_Ticktemp = 0;
void Push_Motor_Calibrate(void)
{
		u8 caputernumsetfinsh = 0;
    if(Push_motor_calibrationFLAG == 1)    //每次标定完毕会将该标志位置0
    {
        if(Push_Location_model ==1)      //每次标定完毕会将该标志位置1
        {
            MotorControl[0].Motor_Start_Stop = 0; //确保推杆在标定时为使能和PWM为0
            MotorControl[1].Motor_Start_Stop = 0;
            MotorControl[2].Motor_Start_Stop = 0;
            MotorControl[9].Motor_Start_Stop = 0;
            MotorControl[0].PWM_Duty = 0;
            MotorControl[1].PWM_Duty = 0;
            MotorControl[2].PWM_Duty = 0;
            MotorControl[9].PWM_Duty = 0;
//            Sys_Ticktemp  = HAL_GetTick(); //确保相对时间
            Push_Location_model = 0;
        }
        MotorControl[0].PWM_Duty+=10;
        MotorControl[1].PWM_Duty+=10;
        MotorControl[2].PWM_Duty+=10;
        MotorControl[9].PWM_Duty+=10;
        if(MotorControl[0].PWM_Duty > PERCENT_95_OF_PWM_PERIOD)  
				{
					MotorControl[0].PWM_Duty = PERCENT_95_OF_PWM_PERIOD;
					MotorControl[0].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD; 
				}
        if(MotorControl[1].PWM_Duty > PERCENT_95_OF_PWM_PERIOD)
				{
					MotorControl[1].PWM_Duty = PERCENT_95_OF_PWM_PERIOD;
					MotorControl[1].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD; 
				}
        if(MotorControl[2].PWM_Duty > PERCENT_95_OF_PWM_PERIOD)  
				{
					MotorControl[2].PWM_Duty = PERCENT_95_OF_PWM_PERIOD;
					MotorControl[2].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD; 
				}
        if(MotorControl[9].PWM_Duty > PERCENT_95_OF_PWM_PERIOD)
				{
					MotorControl[9].PWM_Duty = PERCENT_95_OF_PWM_PERIOD;
					MotorControl[9].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD; 
				}
//        MOTOR0CW();
        MOTOR1CW();
        MOTOR2CW();
        MOTOR9CW();
				Sys_Ticktemp++;
        if(Sys_Ticktemp%501 == 0)
        {
            MotorControl[0].Hall.HALL_PreCaptureValue = MotorControl[0].Hall.HALL_CaptureValue ;
            MotorControl[1].Hall.HALL_PreCaptureValue = MotorControl[1].Hall.HALL_CaptureValue ;
            MotorControl[2].Hall.HALL_PreCaptureValue = MotorControl[2].Hall.HALL_CaptureValue ;
            MotorControl[9].Hall.HALL_PreCaptureValue = MotorControl[9].Hall.HALL_CaptureValue ;
        }
				else if(Sys_Ticktemp%1000 == 0)
        {
            caputernumsetfinsh = 1;//局部变量来保证霍尔值更新之后进行到位判断
        }
        else
        {
            caputernumsetfinsh = 0;
        }

        if(((Sys_Ticktemp > 4000 )&& caputernumsetfinsh == 1 )&&
                MotorControl[0].Hall.HALL_PreCaptureValue == MotorControl[0].Hall.HALL_CaptureValue &&
                MotorControl[1].Hall.HALL_PreCaptureValue == MotorControl[1].Hall.HALL_CaptureValue &&
                MotorControl[2].Hall.HALL_PreCaptureValue == MotorControl[2].Hall.HALL_CaptureValue &&
                MotorControl[9].Hall.HALL_PreCaptureValue == MotorControl[9].Hall.HALL_CaptureValue &&
                MotorControl[0].Fault_Flag ==0&&
                MotorControl[1].Fault_Flag ==0&&
                MotorControl[2].Fault_Flag ==0&&
                MotorControl[9].Fault_Flag ==0) //如果时间到，计数值没有改变，电机没有过流，
        {

            MOTOR0STOP();
            MOTOR1STOP();
            MOTOR2STOP();
            MOTOR9STOP();
            MotorControl[0].PWM_Duty = 0;
            MotorControl[0].Hall.HALL_PreCaptureValue = 0;
            MotorControl[0].Hall.HALL_CaptureValue = 0;
            MotorControl[1].PWM_Duty = 0;
            MotorControl[1].Hall.HALL_PreCaptureValue = 0;
            MotorControl[1].Hall.HALL_CaptureValue = 0;
            MotorControl[2].PWM_Duty = 0;
            MotorControl[2].Hall.HALL_PreCaptureValue = 0;
            MotorControl[2].Hall.HALL_CaptureValue = 0;
            MotorControl[9].PWM_Duty = 0;
            MotorControl[9].Hall.HALL_PreCaptureValue = 0;
            MotorControl[9].Hall.HALL_CaptureValue = 0;

            Push_motor_calibrationFLAG = 0;
            Push_Location_model = 1; //开启推杆位置环
						Sys_Ticktemp = 0;//计数清零
        }
				else if(Sys_Ticktemp > 16000) //32秒
        {
            Sys_Ticktemp = 0;
//            MC_SetFault(PUSHMOTOR_INT_ERROR);
        }
    }
}

/*------------------------------------------------
Function:交换排序
Input   :No
Output  :No
Explain :No
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
void Motor5_Default_Phase_Cheek(void)  //无刷1缺项保护
{
    if(MotorControl[5].Speed_Real != 0 && MotorControl[5].Motor_Start_Stop == 1 && \
            MotorControl[5].Speed_Set != 0  )
    {
        Motor5_Current_temp.cnt++;
        if(Motor5_Current_temp.cnt>10000)
        {
            Motor5_Current_temp.cnt = 0;
            BubbleSort(M5_100us_ac_ab_bc_cur,ac_ab_bc_index);//从小到大进行排序
            testM5_100us_ac_ab_bc_cur =(M5_100us_ac_ab_bc_cur[2]-M5_100us_ac_ab_bc_cur[0])/M5_100us_ac_ab_bc_cur[2];//测试观察值
            if(testM5_100us_ac_ab_bc_cur>0.9) //从小到大排序之后最大值-最小值，0.6为测试估计值
            {
//                if(MotorControl[5].Current.DeepFilterVAL>50)
                {
                    MC_SetFault(MOTOR5_MISSING_PHASE); //单纯为了指示灯
                    MotorControl[5].Fault_Flag =1;
                    MotorControl[5].Motor_Start_Stop = 0;

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
		Motor5_Current_temp.cnt = 0;
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
            BubbleSort(M6_100us_ac_ab_bc_cur,ac_ab_bc_index);//从小到大进行排序
		 if(M6_100us_ac_ab_bc_cur[2]>100)
		  {
            testM6_100us_ac_ab_bc_cur =	(M6_100us_ac_ab_bc_cur[2]-M6_100us_ac_ab_bc_cur[0])/M6_100us_ac_ab_bc_cur[2];//测试观察值
            if(testM6_100us_ac_ab_bc_cur>0.9) //从小到大排序之后最大值-最小值，0.5为测试估计值
            {
//                if(MotorControl[6].Current.DeepFilterVAL>50)
                {   
									  MC_SetFault(MOTOR6_MISSING_PHASE); //单纯为了指示灯
                    MotorControl[6].Fault_Flag =1;
                    MotorControl[6].Motor_Start_Stop = 0;

                }
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
		Motor6_Current_temp.cnt = 0;
    }
}

/*************************
*Function Name 		:Hardware_flowCheck
*Description   		:Check Hardware flow cnt if >=BREAK_CNT_MAX setfault the motor
* Input           : None
* Output          : None
* Return          : None		2021.11.25	by diamond
*************************/
void Hardware_flowCheck(void) //硬件过流保护次数
{
	if(OverFlow_Cnt[0]>=BREAK_CNT_MAX)
	{
		MC_SetFault(MOTOR5_BREAK);
		MotorControl[5].Fault_Flag = 1;
		MotorControl[5].Motor_Start_Stop = DISABLE;
	}
	if(OverFlow_Cnt[1]>=BREAK_CNT_MAX)
	{
		MC_SetFault(MOTOR6_BREAK);
		MotorControl[6].Fault_Flag = 1;
		MotorControl[6].Motor_Start_Stop = DISABLE;
	}
	if(OverFlow_Cnt[2]>=BREAK_CNT_MAX)
	{
		MC_SetFault(BRAKE_3_4);
		MotorControl[3].Motor_Start_Stop = DISABLE;
		MotorControl[3].Fault_Flag = 1;
		MotorControl[4].Motor_Start_Stop = DISABLE;
		MotorControl[4].Fault_Flag = 1;
	}
	if(OverFlow_Cnt[3]>=BREAK_CNT_MAX)
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
	if(OverFlow_Cnt[4]>=BREAK_CNT_MAX)
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
Explain :No
------------------------------------------------*/
u8 Voltage_offset_calibration[8] = {0,1,2,3,4,5,6,9};//上下对起来看，电机
u8 ADC_ConvertedValueindex[8] =    {6,7,8,0,1,2,4,9};//上下对起来看，电机对应的ADC通道
#define OFFSET_NUM 100
u8 offset_cnt = 0;
void Voltage_offset_cali(void)      //偏差电压校准函数
{
    offset_cnt++;
    for(int i = 0; i < 8; i++)
    {
        MotorControl[Voltage_offset_calibration[i]].Current.offset += ADC_ConvertedValue[ADC_ConvertedValueindex[i]];
    }
    if(offset_cnt == OFFSET_NUM)
    {
        for(int k = 0; k < 8; k++)
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
    if(MotorControl[5].PWM_Duty!=0)
    {
        if(MotorControl[5].Direction!=-1&&MotorControl[5].Direction!=1)
        {
            motor_err_cnt5++;
            if(motor_err_cnt5>10)		//累计10次报警
            {
                motor_err_cnt5=10;
                MotorControl[5].Motor_Start_Stop = 0;
                MC_SetFault(MOTOR5_PHASE_ERROR);
                MotorControl[5].Fault_Flag =1;

            }
        }
        else if(ABS(MotorControl[5].Speed_Set-MotorControl[5].Speed_Real)<20)
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
                MC_SetFault(MOTOR6_PHASE_ERROR);
                motor_err_cnt6=10;
                MotorControl[6].Motor_Start_Stop = 0;
                MotorControl[6].Fault_Flag =1;

            }
        }
        else if(ABS(MotorControl[6].Speed_Set-MotorControl[6].Speed_Real)<20)
        {
            if(motor_err_cnt6>0)
                motor_err_cnt6--;
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
void BLDC1_OverSpdChk(void)         //超速保护
{
	static u16 speed_err_cnt = 0;
	if((MotorControl[5].Speed_Ref>0&&MotorControl[5].Speed_Real<0)||(MotorControl[5].Speed_Ref<0&&MotorControl[5].Speed_Real>0))		/* 反向转 */
		speed_err_cnt++;
	else if(MotorControl[5].Speed_Ref==0&&ABS(MotorControl[5].Speed_Set)<500)
		speed_err_cnt=0;			
	else if(ABS(MotorControl[5].Speed_Real)-ABS(MotorControl[5].Speed_Ref)>500)																								/* 实际>设定 */
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
*Function Name     :Motor7_Err_Chk
*Description       :边刷电机错误检测  2S没速度报警 
* Input           : None
* Output          : None  
* Return          : None    2021.12.16  by diamond
*************************/
void Motor7_Err_Chk(void)    /* 200ms循环里 边刷电机异常*/
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

void Motor8_Err_Chk(void)    /* 200ms循环里 边刷电机异常*/
{
  static u8 err_cur_cnt = 0;
  if(MotorControl[8].PWM_Duty>500&&MotorControl[8].Speed_Real==0)
  {
    err_cur_cnt++;
    if(err_cur_cnt==20)    
    {
      MotorControl[8].Fault_Flag =1;
      MotorControl[8].Motor_Start_Stop = DISABLE;
      MC_SetFault(FAN_ERROR);
    }
  }
  else 
    err_cur_cnt=0;
}


/*************************
*Function Name 		:Push_OverRunChk
*Description   		:推杆动作时间限制
* Input           : u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3
										push 1-4:	0关闭 1开启	timer:0-256 多少秒后停止
* Output          : None
* Return          : None		2022.3.7	by diamond
*************************/

//u8 timer_cnt[5];
void Push_OverRunChk(u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2)    //推杆动作时间限制保护
{
	static u8 timer_cnt[5];
	if(push1)
	{
		if(MotorControl[0].Motor_Start_Stop==ENABLE&&ABS(MotorControl[0].PWM_Duty)>500)
		{
			if(MotorControl[0].Current.FilterValue>5)
			{				
			    timer_cnt[0]++;
			   if(timer_cnt[0]>timer0*2)
			  {
				MotorControl[0].Motor_Start_Stop=DISABLE;
//				  MotorControl[0].Fault_Flag = 1;
				MC_SetFault(PUSH_MOTOR_OVERTIME);
				MC_SetFault1(PUSH0_OVERTIME);
				timer_cnt[0]=0;
			  }
		   }
			else timer_cnt[0]=0;
		}
		
	}
	if(push2)
	{
		if(MotorControl[1].Motor_Start_Stop==ENABLE&&ABS(MotorControl[1].PWM_Duty)>500&&MotorControl[1].Current.SetValue==0)   //滚刷自适应的时候不开启
		{
			if(MotorControl[1].Current.FilterValue>35)
			{
				timer_cnt[1]++;
				if(timer_cnt[1]>timer1*2)
				{
					MotorControl[1].Motor_Start_Stop=DISABLE;
//					MotorControl[1].Fault_Flag = 1;
					MC_SetFault(PUSH_MOTOR_OVERTIME);
					MC_SetFault1(PUSH1_OVERTIME);
					timer_cnt[1]=0;
				}
			}
			else timer_cnt[1]=0;
		}
		
	}
	if(push3)
	{
		if(MotorControl[2].Motor_Start_Stop==ENABLE&&ABS(MotorControl[2].PWM_Duty)>500)
		{
			if(MotorControl[2].Current.FilterValue>5)         //921修改
			{
				timer_cnt[2]++;
				if(timer_cnt[2]>timer2*2)
				{
					MotorControl[2].Motor_Start_Stop=DISABLE;
					MC_SetFault(PUSH_MOTOR_OVERTIME);
//					MotorControl[2].Fault_Flag = 1;
					MC_SetFault1(PUSH2_OVERTIME);
					timer_cnt[2]=0;
				}
		    }
			else timer_cnt[2]=0;
		}
	}
}

void Motor_Choose(void)
{
	if(Motor_Type==0)                  //滚刷电机选择，0：联谊滚刷电机   1：和泰滚刷电机
	{
		MotorControl[6].Pole_Paires = 4;
	}
	else if(Motor_Type==1)
	{
		MotorControl[6].Pole_Paires = 2;
		PID_Speed_InitStruct[1].hKp_Gain=600;
		PID_Current_InitStructure[1].hKp_Gain=600;
		PID_Speed_InitStruct[1].hKi_Gain=1;
	}
}
