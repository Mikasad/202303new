#include "function.h"
#include "stm32f4xx_hal.h"
#include "bsp_BDCMotor.h"
#include "main.h"
#include "canopen_pdo.h"
#include "Agreement.h"
#include "flash.h"
#include <stdlib.h>
VoltVar_TypeDef VoltVar;
u32 wGlobal_Flags;
u32 prewGlobal_Flags;
u32 wGlobal_Flags1;
u32 wGlobal_Flags2;
u32 FaultOccurred = 0;
u32 FaultOccurred_last = 0;
u32 FaultOccurred1 = 0;
u32 FaultOccurred2 = 0;
u32 FaultOccurred2_last = 0;
u8  FaultOccurred_flag=0;
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
void CurrentLimit(u8 Motor_NUM) //100us
{
	  
	  if( MotorState == IDLE) //电流校准期间不判断（因为如果有偏置电压开始时电流会很大）
		{
		 return;
		}
	  if(Motor_NUM==7 ||Motor_NUM==8 ||Motor_NUM==9) //边刷风机不进入
		{
		 return;
		}
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
void MC_SetFault(u32 hFault_type) //第一个全局错误
{
    u8 i;
	  if(hFault_type ==HALL5_SENSOR_ERR)//暂时没有电机5 40机器
		return;
    wGlobal_Flags |= hFault_type;
    if(FaultOccurred !=0)
    {
        FaultOccurred |=hFault_type;
        FaultOccurred &=hFault_type;
    }
    else
        FaultOccurred  |= hFault_type; //只报警当前错误类型
}
void MC_SetFault1(u32 hFault_type) //第二个全局错误
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
void MC_SetFault2(u32 hFault_type) //第3个全局错误
{
    wGlobal_Flags2 |= hFault_type;
    if(FaultOccurred2 !=0)
    {
        FaultOccurred2 |=hFault_type;
        FaultOccurred2 &=hFault_type;
    }
    else
        FaultOccurred2  |= hFault_type; //只报警当前错误类型
}
/*------------------------------------------------
Function:清除错误标志
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_ClearFault(u32 hFault_type) //清除第一个全局错误
{
    wGlobal_Flags &= ~hFault_type;
    FaultOccurred &= ~hFault_type; //清除当前错误类型

}

void MC_ClearFault2(u32 hFault_type) //清除第3个全局错误
{
    wGlobal_Flags2 &= ~hFault_type;
    FaultOccurred2 &= ~hFault_type; //清除当前错误类型

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
        if(((wGlobal_Flags&(PUSH_MOTOR1_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2|PUSH0_OVERTIME))==0)&&MotorControl[0].Fault_Flag == 1)//推杆0错误清除
        {
            MotorControl[0].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR2_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2))==0)&&MotorControl[1].Fault_Flag == 1)//推杆1错误清除
        {
            MotorControl[1].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR3_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2|PUSH2_OVERTIME))==0)&&MotorControl[2].Fault_Flag == 1)//推杆2错误清除
        {
            MotorControl[2].Fault_Flag = 0;
        }
//        if(((wGlobal_Flags&(PUSH_MOTOR4_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2|PUSH9_OVERTIME))==0)&&MotorControl[9].Fault_Flag == 1)//推杆9错误清除
//        {
//            MotorControl[9].Fault_Flag = 0;
//        }
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
    if(((wGlobal_Flags&FAN_ERROR)==0)&&MotorControl[8].Fault_Flag == 1)//电机8错误清除 边刷2
    {
        MotorControl[8].Fault_Flag = 0;
    }
    if(((wGlobal_Flags&SIDE2_BRUSH_ERROR)==0)&&MotorControl[9].Fault_Flag == 1)//电机9错误清除 风机
    {
        MotorControl[9].Fault_Flag = 0;
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
Explain :低压恢复：20V  高压恢复：30V  过压：31V 欠压：19V
------------------------------------------------*/
void Over_VoltageCheck(void)  //10ms
{
    if(VoltVar.BUS>VOLT_360V)
    {
        VoltVar.BusHighCnt++;
        if(VoltVar.BusHighCnt>VoltVar.HighGuardTime)
        {
            VoltVar.BusHighCnt   = VoltVar.HighGuardTime+50;
            for(u8 i=0; i < MOTOR_NUM; i++)
            {
                MotorControl[i].Motor_Start_Stop = DISABLE;
            }
            MC_SetFault(OVER_VOLTAGE); //过压
        }
    }
    else if(VoltVar.BUS < VOLT_300V )   //OVR
    {
        if(VoltVar.BusHighCnt>0)
        {
            VoltVar.BusHighCnt--;
            if(VoltVar.BusHighCnt < VoltVar.HighGuardTime)
            {
                MC_ClearFault(OVER_VOLTAGE); //过压恢复
            }
        }
    }

    if(VoltVar.BUS<VOLT_160V)
    {
        if(++VoltVar.BusLowCnt>VoltVar.LowGuardTime)
        {
            VoltVar.BusLowCnt = VoltVar.LowGuardTime+50;
            for(u8 i=0; i < MOTOR_NUM; i++)
            {
                MotorControl[i].Motor_Start_Stop = DISABLE;
            }
            MC_SetFault(UNDER_VOLTAGE); //欠压
        }
    }
    else if(VoltVar.BUS > VOLT_200V)
    {
        if(VoltVar.BusLowCnt>0) //UVR
        {
            VoltVar.BusLowCnt--;
            if(VoltVar.BusLowCnt < VoltVar.LowGuardTime)
            {
                MC_ClearFault(UNDER_VOLTAGE); //欠压恢复
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
    VoltVar.HighGuardTime  = 50; 	//过压超时时间 50*10==500ms
    VoltVar.LowGuardTime  = 50;   //欠压超时时间 50*10==500ms
//    VoltVar.BusHigh = VOLT_360V ; 
//    VoltVar.BusLow =  VOLT_160V ;
//    TIM3->CCR1 = 0;
//    TIM8->CCR4 = 8400;
//	   MotorControl[10].PWM_DutySet = 8400;		/* S线单向有刷，供电24V */   50不需要
//    MotorControl[12].PWM_DutySet = 8400;		/* S线单向有刷，供电24V */
//    MotorControl[10].Motor_Start_Stop = 1;
//    MotorControl[12].Motor_Start_Stop = 1;
    /************************推杆初始化***************************/
    MotorControl[0].Current.MaxValue1 = MOTOR0_CurrentValue(1) ;  //  额定1.5A保护电流5A
    MotorControl[0].Current.MaxValue2 = MOTOR0_CurrentValue(1.5) ;
    MotorControl[0].Current.MaxValue3 = MOTOR0_CurrentValue(2) ;
    MotorControl[0].Current.MaxValue4 = MOTOR0_CurrentValue(3) ;
    MotorControl[0].Current.OFCnt1_T = 200;
    MotorControl[0].Current.OFCnt2_T = 30000;  //3S
    MotorControl[0].Current.OFCnt3_T = 10000;  //1S
    MotorControl[0].Current.OFCnt4_T = 2000;   //200MS

    MotorControl[1].Current.MaxValue1 = MOTOR1_CurrentValue(1) ; //  额定1.5A保护电流5A
    MotorControl[1].Current.MaxValue2 = MOTOR1_CurrentValue(1.5) ;
    MotorControl[1].Current.MaxValue3 = MOTOR1_CurrentValue(2) ;
    MotorControl[1].Current.MaxValue4 = MOTOR1_CurrentValue(3) ;
    MotorControl[1].Current.OFCnt1_T = 200;
    MotorControl[1].Current.OFCnt2_T = 30000;
    MotorControl[1].Current.OFCnt3_T = 10000;
    MotorControl[1].Current.OFCnt4_T = 2000;

    MotorControl[2].Current.MaxValue1 = MOTOR2_CurrentValue(1) ; //  额定1.5A保护电流3A
    MotorControl[2].Current.MaxValue2 = MOTOR2_CurrentValue(1.5) ;
    MotorControl[2].Current.MaxValue3 = MOTOR2_CurrentValue(2) ;
    MotorControl[2].Current.MaxValue4 = MOTOR2_CurrentValue(3) ;
    MotorControl[2].Current.OFCnt1_T = 200;
    MotorControl[2].Current.OFCnt2_T = 30000;
    MotorControl[2].Current.OFCnt3_T = 10000;
    MotorControl[2].Current.OFCnt4_T = 2000;

    MotorControl[9].Current.MaxValue1 = MOTOR9_CurrentValue(2) ;//（没用）
    MotorControl[9].Current.MaxValue2 = MOTOR9_CurrentValue(3) ;
    MotorControl[9].Current.MaxValue3 = MOTOR9_CurrentValue(4) ;
    MotorControl[9].Current.MaxValue4 = MOTOR9_CurrentValue(5) ;
    MotorControl[9].Current.OFCnt1_T = 200;
    MotorControl[9].Current.OFCnt2_T = 30000;
    MotorControl[9].Current.OFCnt3_T = 10000;
    MotorControl[9].Current.OFCnt4_T = 2000;
    /*********************推杆初始化end***************************/

    /*************************单向有刷3_4***************************/
    MotorControl[3].Current.MaxValue1 = MOTOR3_CurrentValue(3) ;  //过滤电机 保护电流8A
    MotorControl[3].Current.MaxValue2 = MOTOR3_CurrentValue(4.5) ;
    MotorControl[3].Current.MaxValue3 = MOTOR3_CurrentValue(5) ;  //过滤电机
    MotorControl[3].Current.MaxValue4 = MOTOR3_CurrentValue(6) ;
    MotorControl[3].Current.OFCnt1_T = 2000;
    MotorControl[3].Current.OFCnt2_T = 30000;
    MotorControl[3].Current.OFCnt3_T = 10000;
    MotorControl[3].Current.OFCnt4_T = 2000;

    MotorControl[4].Current.MaxValue1 = MOTOR4_CurrentValue(2) ;  //喷水电机 保护电流点按3.6A
    MotorControl[4].Current.MaxValue2 = MOTOR4_CurrentValue(2) ;
    MotorControl[4].Current.MaxValue3 = MOTOR4_CurrentValue(2.5) ;  //喷水电机
    MotorControl[4].Current.MaxValue4 = MOTOR4_CurrentValue(3) ;
    MotorControl[4].Current.OFCnt1_T = 2000;
    MotorControl[4].Current.OFCnt2_T = 50000;
    MotorControl[4].Current.OFCnt3_T = 10000;
    MotorControl[4].Current.OFCnt4_T = 2000;

    /*************************单向有刷3_4 end***************************/

    /*************************无刷5，6***************************/
//    MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(7) ;
//    MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(9) ;
//    MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(10) ;
//    MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(11) ;
    MotorControl[6].Current.MaxValue1 = MOTOR5_CurrentValue(1) ; //大功率无刷 额定25A保护电流点88A
    MotorControl[6].Current.MaxValue2 = MOTOR5_CurrentValue(20) ;
    MotorControl[6].Current.MaxValue3 = MOTOR5_CurrentValue(22) ;
    MotorControl[6].Current.MaxValue4 = MOTOR5_CurrentValue(23) ;
    MotorControl[6].Current.CurOffset = 0.8*MOTOR5CURCOEFFICIENT*100;		//默认0.2A
    MotorControl[6].Current.OFCnt1_T = 30000;
    MotorControl[6].Current.OFCnt2_T = 30000;
    MotorControl[6].Current.OFCnt3_T = 10000;
    MotorControl[6].Current.OFCnt4_T = 1000;
    MotorControl[6].Pole_Paires = 2;              //极对数
    MotorControl[6].Acceleration = 5;             //加速度
    MotorControl[6].Deceleration = 5;             //减速度

    MotorControl[5].Current.MaxValue1 = MOTOR6_CurrentValue(1) ; //小功率无刷 额定6.25A保护电流点49A
    MotorControl[5].Current.MaxValue2 = MOTOR6_CurrentValue(6.25) ;
    MotorControl[5].Current.MaxValue3 = MOTOR6_CurrentValue(8) ;
    MotorControl[5].Current.MaxValue4 = MOTOR6_CurrentValue(10) ;
    MotorControl[5].Current.CurOffset = 0.3*MOTOR6CURCOEFFICIENT*100;		//默认0.3A
    MotorControl[5].Current.OFCnt1_T = 30000;
    MotorControl[5].Current.OFCnt2_T = 30000;
    MotorControl[5].Current.OFCnt3_T = 10000;
    MotorControl[5].Current.OFCnt4_T = 1000;
    MotorControl[5].Pole_Paires = 2;
    MotorControl[5].Acceleration = 5;
    MotorControl[5].Deceleration = 5;
    /*************************无刷5，6end***************************/
    PID_Init (&PID_Speed_InitStruct[0],&PID_Current_InitStructure[0]);		//电机5的PID
    PID_Init (&PID_Speed_InitStruct[1],&PID_Current_InitStructure[1]);		//电机6的PID
    PID_PWM_Init(&PID_PWM);																								//推杆0的PID
    /*	加减速度初始化	*/
    MotorControl[0].Acceleration = 5;  //10->5 防止到达目标位置后来回动作时间过长
    MotorControl[0].Deceleration = 5;
    MotorControl[0].LastMotorDirection = 2;
    MotorControl[0].Hall.HALL_CaptureValue = 0; //用于上电标定
    MotorControl[1].Acceleration = 5;
    MotorControl[1].Deceleration = 5;
    MotorControl[1].LastMotorDirection = 2;
    MotorControl[1].Hall.HALL_CaptureValue = 10000; //由于推杆1需要位置控制，不标定不会打开Push_Location_model开关，所以在直接给定标定命令的时候当前值为0
    MotorControl[2].Acceleration = 5;
    MotorControl[2].Deceleration = 5;
    MotorControl[2].LastMotorDirection = 2;
    MotorControl[2].Hall.HALL_CaptureValue = 0; //用于上电标定

    MotorControl[3].Acceleration = 10;
    MotorControl[3].Deceleration = 10;

    MotorControl[4].Acceleration = 10;
    MotorControl[4].Deceleration = 10;

//    MotorControl[7].Acceleration = 60; //60 rpm/S
//    MotorControl[7].Deceleration = 60;

//    MotorControl[8].Acceleration = 60;
//    MotorControl[8].Deceleration = 60;

//    MotorControl[9].Acceleration = 60;
//    MotorControl[9].Deceleration = 60;

	  MotorControl[7].Acceleration = 10; //与端口控制对其 对应485控制60rpm/S
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

//    MotorControl[5].Hall.HallState = (GPIOD->IDR>>5)&(0x7);
//    MotorControl[6].Hall.HallState = HALL_GetPhase2();

    MotorControl[5].Hall.HallState = HALL_GetPhase2();
    MotorControl[6].Hall.HallState = (GPIOD->IDR>>5)&(0x7);
    /*           霍尔学习参数                */
    HALL_Study[1].StudySectorCnt3 = 200;/*换向一次时间1代表2ms*/
    HALL_Study[1].HallSector = 1;
    HALL_Study[1].StudySectorCnt = 0;
    HALL_Study[1].HallCommPWM = 4500;/*霍尔换向PWM值*/

    HALL_Study[0].StudySectorCnt3 = 200;/*换向一次时间1代表2ms*/
    HALL_Study[0].HallSector = 1;
    HALL_Study[0].StudySectorCnt = 0;
    HALL_Study[0].HallCommPWM = 4500;/*霍尔换向PWM值*/

//    HALL_Study[0].HallTab[0] = 6;
//    HALL_Study[0].HallTab[1] = 4;
//    HALL_Study[0].HallTab[2] = 5;
//    HALL_Study[0].HallTab[3] = 1;
//    HALL_Study[0].HallTab[4] = 3;
//    HALL_Study[0].HallTab[5] = 2;
    HALL_Study[0].HallTab[0] = 6; //反转 转化成 正转 的hall顺序
    HALL_Study[0].HallTab[1] = 4;
    HALL_Study[0].HallTab[2] = 5;
    HALL_Study[0].HallTab[3] = 1;
    HALL_Study[0].HallTab[4] = 3;
    HALL_Study[0].HallTab[5] = 2;

//    HALL_Study[1].HallTab[0] = 6;// 546231
//    HALL_Study[1].HallTab[1] = 4;
//    HALL_Study[1].HallTab[2] = 5;
//    HALL_Study[1].HallTab[3] = 1;
//    HALL_Study[1].HallTab[4] = 3;
//    HALL_Study[1].HallTab[5] = 2;

    HALL_Study[1].HallTab[0] = 1;//  机器实测
    HALL_Study[1].HallTab[1] = 5;
    HALL_Study[1].HallTab[2] = 4;
    HALL_Study[1].HallTab[3] = 6;
    HALL_Study[1].HallTab[4] = 2;
    HALL_Study[1].HallTab[5] = 3;

    MotorControl[0].Push_Location_model=1;
    MotorControl[1].Push_Location_model=1; //该推杆电机需要标定，标定成功才可以位置控制，其余电机不需要标定默认开启位置控制，只是伸出和收缩的控制没有位置控制
    MotorControl[2].Push_Location_model=1;
//    MotorControl[9].Push_Location_model=1;

    MotorControl[7].Motor_mode=Control_485; //边刷控制模式 485控制或者IO控制
		MotorControl[8].Motor_mode=Control_485; //边刷控制模式 485控制或者IO控制
		MotorControl[9].Motor_mode=Control_485; //风机控制模式 485控制或者IO控制
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
    if(wGlobal_Flags==NO_ERROR && wGlobal_Flags2==NO_ERROR)
    {
        RED_LED1_OFF;
        RED_LED2_OFF;
        wGlobal_Flags1 = 0;
        FaultOccurred = 0;
        FaultOccurred1 = 0;
			  FaultOccurred2 = 0;
    }
    else
    {
      if(FaultOccurred!=FaultOccurred_last) //两种错误码 处理方法
			{
        FaultOccurred_last=FaultOccurred;	
				FaultOccurred_flag=1;
			}else if(FaultOccurred2!=FaultOccurred2_last)
			{
			 FaultOccurred2_last=FaultOccurred2;
				FaultOccurred_flag=2;
			}
			if(FaultOccurred_flag==1)
			{
        switch (FaultOccurred)
        {
        case PUSH_MOTOR1_OVER_CUR: //下面红灯快闪1次
            LEDSet(LED_OFF,LED_ON,1,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR2_OVER_CUR: //下面红灯快闪2次
            LEDSet(LED_OFF,LED_ON,2,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR3_OVER_CUR: //下面红灯快闪3次
            LEDSet(LED_OFF,LED_ON,3,0,LED_HIGH_SPEED);

            break;

        case ONEWYA_MOTOR1_OVER_CUR: //下面红灯快闪4次
            LEDSet(LED_OFF,LED_ON,4,0,LED_HIGH_SPEED);


            break;
        case ONEWYA_MOTOR2_OVER_CUR: //下面红灯快闪5次
            LEDSet(LED_OFF,LED_ON,5,0,LED_HIGH_SPEED);


            break;
        case BLDC1_OVER_CUR: //下面红灯快闪6次
            LEDSet(LED_OFF,LED_ON,6,0,LED_HIGH_SPEED);


            break;
        case BLDC2_OVER_CUR:	//下面红灯快闪7次
            LEDSet(LED_OFF,LED_ON,7,0,LED_HIGH_SPEED);


            break;
        case HALL5_SENSOR_ERR:		//下面红灯快闪8次
            LEDSet(LED_OFF,LED_ON,8,0,LED_HIGH_SPEED);
            break;

        case MOTOR4_BRAKE_LINE:    //下面红灯快闪9次
            LEDSet(LED_OFF,LED_ON,9,0,LED_HIGH_SPEED);		//慢闪

            break;

        case MOTOR5_PHASE_ERROR:		//下面红灯快闪10次
            LEDSet(LED_OFF,LED_ON,10,0,LED_HIGH_SPEED);
            break;
        case MOTOR6_PHASE_ERROR:		//下面红灯快闪11次
            LEDSet(LED_OFF,LED_ON,11,0,LED_HIGH_SPEED);
            break;
        case HALL6_SENSOR_ERR:		//下面红灯快闪12次
            LEDSet(LED_OFF,LED_ON,12,0,LED_HIGH_SPEED);
            break;
				
        case OVER_VOLTAGE:  //两个红灯常量
            RED_LED1_ON;
            RED_LED2_ON;
            DisplayCounter=0;
            break;
        case UNDER_VOLTAGE:   //欠压上面红灯亮
            RED_LED2_OFF;
            RED_LED1_ON;
            DisplayCounter=0;
            break;
        case MOTOR5_OVER_SPEED:		//上面红灯快闪1次
            LEDSet(LED_ON,LED_OFF,1,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_OVER_SPEED:		//上面红灯快闪2次
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case CAN_COMMUNICATION_ERR:				//上面红灯快闪3次
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case MOTOR5_MISSING_PHASE:		//上面红灯快闪4次
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;
        case MOTOR6_MISSING_PHASE:	//上面红灯快闪5次
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;
        case PUSH_LOST_HALL:			//上面红灯快闪6次
            LEDSet(LED_ON,LED_OFF,6,0,LED_HIGH_SPEED);
            break;

        case MOTOR3_BRAKE_LINE:					//上面红灯快闪7次
            LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
            break;

        case BRAKE_3_4:	//上面红灯快闪8次
            LEDSet(LED_ON,LED_OFF,8,0,LED_HIGH_SPEED);
            break;

        case BRAKE_0_1_2: //上面红灯快闪9次
            LEDSet(LED_ON,LED_OFF,9,0,LED_HIGH_SPEED);
            break;

        case BRAKE10_11_12_13: //上面红灯快闪10次
            LEDSet(LED_ON,LED_OFF,10,0,LED_HIGH_SPEED);

            break;
        case MOTOR5_BREAK:  //上面红灯快闪11次
            LEDSet(LED_ON,LED_OFF,11,0,LED_HIGH_SPEED);

            break;
        case MOTOR6_BREAK: //上面红灯快闪12次
            LEDSet(LED_ON,LED_OFF,12,0,LED_HIGH_SPEED);
            break;
        case SIDE2_BRUSH_ERROR:  //右边刷错误		上面常亮，下面快速闪烁1-5次
            switch (FaultOccurred1)
            {
            case SB2_SENSOR_ERR:
                LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁1次
                break;
            case SB2_CUR_ERR:
                LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁2次
                break;
            case SB2_PHASECUR_ERR:
                LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁3次
                break;
            case SB2_LOCKED_ERR:
                LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁4次
                break;
            case SB2_TEMP_ERR:
                LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁5次
                break;
            }
            RED_LED1_ON;
            break;
        case FAN_ERROR:		//风机错误  上面常亮，下面快速闪烁6-10次
            switch(FaultOccurred1)
            {
            case MOTOR8_VOLTAGE_ERROR:
                LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁6次
                break;
            case MOTOR8_OVER_CURRENT:
                LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁7次
                break;
            case MOTOR8_OVER_SPEED:
                LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁8次
                break;
            case MOTOR8_OVER_TEMP:
                LEDSet(LED_ON,LED_ON,9,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁9次
                break;
            case MOTOR8_STUCK:
                LEDSet(LED_ON,LED_ON,10,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁10次
                break;
            }
            RED_LED1_ON;
            break;
        case PUSHMOTOR_INT_ERROR:			//推杆标定失败下上面常亮，下面快速闪烁11次
            LEDSet(LED_ON,LED_ON,11,0,LED_HIGH_SPEED);//上面常亮，下面快速闪烁11次
            RED_LED1_ON;
            break;
        case SIDE_BRUSH_ERROR:  //左边刷错误		下面常亮，上面快速闪烁1-5次
            switch (FaultOccurred1)
            {
            case SB2_SENSOR_ERR:
                LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);//下面常亮，上面快速闪烁1次
                break;
            case SB2_CUR_ERR:
                LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);//下面常亮，上面快速闪烁2次
                break;
            case SB2_PHASECUR_ERR:
                LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);//下面常亮，上面快速闪烁3次
                break;
            case SB2_LOCKED_ERR:
                LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);//下面常亮，上面快速闪烁4次
                break;
            case SB2_TEMP_ERR:
                LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);//下面常亮，上面快速闪烁5次
                break;
            }
            RED_LED2_ON;
            break;
        case PUSH_BRAKE_LINE:  //推杆断线		下面常亮，上面快速闪烁6-8次
            switch (FaultOccurred1)
            {
            case PUSH0_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);//下面常亮，上面快速闪烁6次
                break;
            case PUSH1_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);//下面常亮，上面快速闪烁7次
                break;
            case PUSH2_BRAKE_LINE:
                LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);//下面常亮，上面快速闪烁8次
                break;
            }
            RED_LED2_ON;
            break;
        case PUSH_OVER_TIME:		//推杆动作超时下面常亮，上面9-10次
            switch(FaultOccurred1)
            {
            case PUSH0_OVERTIME:
                LEDSet(LED_ON,LED_ON,9,0,LED_HIGH_SPEED);//推杆动作超时下面常亮，上面9次
                break;
            case PUSH2_OVERTIME:
                LEDSet(LED_ON,LED_ON,10,0,LED_HIGH_SPEED);//推杆动作超时下面常亮，上面10次
                break;
            }
            RED_LED2_ON;
            break;
//        default:
//            RED_LED1_OFF;
//            RED_LED2_OFF;
//            break;
        }
			}
			else if(FaultOccurred_flag==2)
			{
				switch (FaultOccurred2)
        {
	           case MOTOR7_DISCONNECT:
                 LEDSet(LED_ON,LED_OFF,13,0,LED_HIGH_SPEED); //上面红灯的13次
                break;
             case MOTOR8_DISCONNECT:
                LEDSet(LED_ON,LED_OFF,14,0,LED_HIGH_SPEED); //上面红灯的14次
                break;
             case MOTOR9_DISCONNECT:
                 LEDSet(LED_ON,LED_OFF,15,0,LED_HIGH_SPEED); //上面红灯的15次
                break;				
				}	
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
    if(motornum == 1) //40机器滚刷推杆，后期预留滚刷自适应
    {
        if(MotorControl[motornum].Push_motor_calibrationFLAG == 2)//标定正确，开始执行位置控制
        {
            if(MotorControl[6].Current.SetValue ==0)	//没有做电流自适应的情况，进行位置控制
            {
                if(MotorControl[motornum].Hall.HALL_CaptureValue > MotorControl[motornum].Location_Set)//收回
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
			if(motornum == 0xff) //为标定位置反向做预留
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
			else  //为标定位置正向位置控制
			{
        if(MotorControl[motornum].Hall.HALL_CaptureValue > MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
        {
            MotorControl[motornum].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
        }
        else if(MotorControl[motornum].Hall.HALL_CaptureValue < MotorControl[motornum].Location_Set)// && fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) > 5)
        {
            MotorControl[motornum].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
        }
        else if(MotorControl[motornum].Hall.HALL_CaptureValue == MotorControl[motornum].Location_Set)//||fabs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set) < 5)
        {
            MotorControl[motornum].PWM_DutySet = 0;
        }
			}
    }
}
/*------------------------------------------------

Function:4路推杆电机霍尔检测和断联检测
Input   :No
Output  :No
Explain : 100us
------------------------------------------------*/
u8 NeedToCheekPush[MOTOR_NUM] = {3,3,3,2,2,0,0,0,0,0,0,0,0,0}; // 单向电机断线：2 推杆断线：3 
u8 NeedToMissingHallPush[MOTOR_NUM] = {0,1,0,0,0,0,0,0,0,0,0,0,0,0}; //该数组来确保推杆是否需要霍尔检测
uint16_t PushMaxPulse[MOTOR_NUM] = {0,1500,0,0,0,0,0,0,0,0,0,0,0,0}; //这个数组需要确定整机推杆的最大值，必须写入正确值，否则在推杆推到尽头可能误报
u32 Push_Error_Cheek_Cnt[MOTOR_NUM] = {0},Push_Error_Cheek_Cnt_1[MOTOR_NUM] = {0};
int Push_curr_temp[MOTOR_NUM] = {0},Push_curr_temp_1[MOTOR_NUM] = {0};
#define PUSH_PROTECT_MIN_VAL 2999
#define PUSH_PROTECT_MIN_CUR  100
#define MOTOR34_PROTECT_MIN_CUR 50
#define MOTOR34_PROTECT_MIN_VAL 2999
u8 CurChkArray[2]= {0};
u8 CurChk_Forward[MOTOR_NUM]= {0};
u8 CurChk_Reverse[MOTOR_NUM]= {0};
void Push_Motor_Cheek(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        if(NeedToCheekPush[i]==1) //如果该推杆电机需要霍尔错误或者推杆断联检测
        {
            /*占空比大于2999才保护，电机使能、未达到实际位置、PWM大于一定值、霍尔值在推杆最大霍尔值范围之内*/
            if(MotorControl[i].Motor_Start_Stop == 1 && (l_abs(MotorControl[i].Location_Set-MotorControl[i].Hall.HALL_CaptureValue))>20 && \
                    l_abs(MotorControl[i].PWM_Duty) > PUSH_PROTECT_MIN_VAL && \
						        (MotorControl[i].Hall.HALL_CaptureValue < PushMaxPulse[i] )) //|| MotorControl[i].Push_motor_calibrationFLAG==1
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
                if(Push_Error_Cheek_Cnt[i]%1001==0) //100MS
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
//                        if(i == 9)
//                        {
//                            MC_SetFault1(SIDE2_BRUSH_ERROR);
//                            MC_SetFault(PUSH_BRAKE_LINE);
//                        }
                    }
                    /*这里100ms必须清一次否则不能报警断线保护*/
                    Push_curr_temp[i] = 0;
                }
                /*以上部分先判断有没有断线，以下再判断推杆电机霍尔是否正常*/
                if(NeedToMissingHallPush[i])
                {
                    /*如果200ms内推杆的霍尔值没有改变，那就说明推杆霍尔异常*/
                    if((Push_Error_Cheek_Cnt[i]%2000==0)&&(MotorControl[i].Hall.HALL_CaptureValueDelta == \
                                                           MotorControl[i].Hall.HALL_CaptureValue)&&Push_curr_temp[i] >= PUSH_PROTECT_MIN_CUR) //200MS
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
//                        if(i == 9)
//                        {
//                            MC_SetFault1(PUSH9_LOST_HALL);
//                            MC_SetFault(PUSH_LOST_HALL);//单纯为了指示灯
//                        }

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
													  CurChk_Forward[i]=1;
                        }
                        else
                        {
													  CurChk_Forward[i]=0;
													  CurChk_Reverse[i]=0;
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
                            CurChk_Reverse[i]=1;
                        }
                        else
                        {
 													  CurChk_Forward[i]=0;
													  CurChk_Reverse[i]=0;
                        }
                        Push_Error_Cheek_Cnt_1[i]=0;
                        Push_curr_temp_1[i] =0;
                    }
                }
                if(CurChk_Forward[i]==1&&CurChk_Reverse[i]==1)
                {
                    MotorControl[i].Motor_Start_Stop = DISABLE;   //断线错误
                    MotorControl[i].Fault_Flag = 1;  //断线错误
									  if(i==0)
										{
                     MC_SetFault1(PUSH0_BRAKE_LINE);
										}
										else if(i==1)
										{
										 MC_SetFault1(PUSH1_BRAKE_LINE);
										}
										else if(i==2)
										{
										 MC_SetFault1(PUSH2_BRAKE_LINE);
										}
                    MC_SetFault(PUSH_BRAKE_LINE);//单纯为了指示灯
                }
            }
        }
    }
}
/*------------------------------------------------
Function:边刷自检函数
Input   :No
Output  :No
Explain :Side_brush_report：边刷在位自检设置 0:null 1:开启自检 2：自检完成
Side_brush_flag7：左边刷是否在位：0：null 1：不在位 2：在位
Side_brush_live8：右边刷是否在位：0：null 1：不在位 2：在位
------------------------------------------------*/
uint8_t Side_brush_step=0; //自检步骤
uint32_t Side_brush_time=0;//自检时间
int32_t  MOTOR7_addCurrent,MOTOR8_addCurrent;//自检电流之和
uint16_t Side_brush_num7=0,Side_brush_num8=0;//自检电流之和次数
uint16_t Side_brush_flag7=0,Side_brush_flag8=0;//自检电流标志
int32_t MOTOR7_avgCurrent,MOTOR8_avgCurrent;
void Side_brush_exe(void)
{
	 if(Side_brush_step==0)
	 {
		MotorControl[7].Motor_Start_Stop=DISABLE; //左边刷停
		MotorControl[8].Motor_Start_Stop=DISABLE; //右边刷停
		Side_brush_step=1;
		Side_brush_time=HAL_GetTick();	 //1代表0.5ms 
	 }
	 else if(Side_brush_step==1 && (HAL_GetTick()-Side_brush_time>1000)) //500ms
	 {
		MotorControl[7].Motor_Start_Stop=ENABLE; //左边刷使能
		MotorControl[8].Motor_Start_Stop=ENABLE; //右边刷使能  
		MotorControl[7].PWM_DutySet=2500; //左边刷速度 100rpm 正转
		MotorControl[8].PWM_DutySet=-2500; //右边刷速度 100rpm	反转
		Side_brush_step=2;
		Side_brush_time=HAL_GetTick();	 //1代表0.5ms 
	 }
	 else if(Side_brush_step==2 && (HAL_GetTick()-Side_brush_time>6000)) //3s 等待转速稳定
	 {
		 Side_brush_flag7=1; //开始累加电流
		 Side_brush_flag8=1; //开始累加电流	  
	//	 MOTOR7_addCurrent+=MotorControl[7].Current.FilterValue;
	//	 MOTOR8_addCurrent+=MotorControl[9].Current.FilterValue;
	//	 Side_brush_num++;
		 Side_brush_step=3;
		 Side_brush_time=HAL_GetTick();	 //1代表0.5ms 
	 }	
	 else if(Side_brush_step==3) //测量数量需要实际确定
	 {
		 if(Side_brush_num7>brush_sample_num) //50次
		 {
		 Side_brush_flag7=0; //停止电流相加
		 }
		 if(Side_brush_num8>brush_sample_num)
		 {
		 Side_brush_flag8=0; //停止电流相加
		 }	
		 if((Side_brush_num7>brush_sample_num && Side_brush_num8>brush_sample_num) || (HAL_GetTick()-Side_brush_time>10000)) //采样够了或者超时5s	
		 {
//			Side_brush_flag7=0; //停止电流相加
//			Side_brush_flag8=0; //停止电流相加
			Side_brush_step=4;
		 }		 
	 }	 
	 else if(Side_brush_step==4) //计算平均电流值 测量数量需要实际确定
	 {
		 MOTOR7_avgCurrent=MOTOR7_addCurrent/Side_brush_num7;
		 MOTOR8_avgCurrent=MOTOR8_addCurrent/Side_brush_num8;
		 Side_brush_step=5;
	 }
	 else if(Side_brush_step==5) //测量数量需要实际确定
	 {
			if(abs(MOTOR7_avgCurrent)>brush_ave_cur) //阈值需要实际测量
			{
				Side_brush_live7=2;           //有毛刷
			}
			else
			{
				Side_brush_live7=1;           //无毛刷
			}
			if(abs(MOTOR8_avgCurrent)>brush_ave_cur)//阈值需要实际测量
			{
				Side_brush_live8=2;            //有毛刷
			}
			else
			{
				Side_brush_live8=1;           //无毛刷
			}
			MotorControl[7].Motor_Start_Stop=DISABLE; //左边刷失能
			MotorControl[8].Motor_Start_Stop=DISABLE; //右边刷失能  
			MotorControl[7].PWM_DutySet=0; //左边刷速度 0
			MotorControl[8].PWM_DutySet=0; //右边刷速度 0	
			Side_brush_report=2;//自检完成
			Side_brush_step=0;
			Side_brush_flag7=0;
			Side_brush_num7=0;
			Side_brush_flag8=0;
			Side_brush_num8=0;
			MOTOR7_addCurrent=0;
			MOTOR8_addCurrent=0; 
	 }
}

/*------------------------------------------------
Function:滚刷自检函数
Input   :No
Output  :No
Explain :Roll6_brush_report：滚刷在位自检设置 0:null 1:开启自检 2：自检完成
Roll_Side_brush_live6：滚刷是否在位：0：null 1：不在位 2：在位
------------------------------------------------*/
uint8_t Roll6_brush_step=0; //自检步骤
uint32_t Roll6_brush_time=0;//自检时间
int32_t  MOTOR6_addCurrent;//自检电流之和
uint16_t Roll6_brush_num=0;//自检电流之和次数
uint16_t Roll6_brush_flag=0;//自检电流标志
int32_t MOTOR6_avgCurrent;
void Roll_brush_exe(void)
{
	 if(Roll6_brush_step==0)
	 {
		MotorControl[6].Motor_Start_Stop=DISABLE; //停
		Roll6_brush_step=1;
		Roll6_brush_time=HAL_GetTick();	 //1代表0.5ms 
	 }
	 else if(Roll6_brush_step==1 && (HAL_GetTick()-Roll6_brush_time>1000)) //500ms
	 {
		MotorControl[6].Motor_Start_Stop=ENABLE; //使能 
		MotorControl[6].Speed_Set=1000; //速度 600rpm 正转
		Roll6_brush_step=2;
		Roll6_brush_time=HAL_GetTick();	 //1代表0.5ms 
	 }
	 else if(Roll6_brush_step==2 && (HAL_GetTick()-Roll6_brush_time>10000)) //5s 等待转速稳定
	 {
		 Roll6_brush_flag=1; //开始累加电流  
		 Roll6_brush_step=3;
		 Roll6_brush_time=HAL_GetTick();	 //1代表0.5ms 
	 }	
	 else if(Roll6_brush_step==3) //测量数量需要实际确定
	 {
		 if(Roll6_brush_num>Roll_brush_sample_num) //50次
		 {
		 Roll6_brush_flag=0; //停止电流相加
		 }	
		 if((Roll6_brush_num>Roll_brush_sample_num) || (HAL_GetTick()-Roll6_brush_time>10000)) //采样够了或者超时5s	
		 {
			Roll6_brush_step=4;
		 }		 
	 }	 
	 else if(Roll6_brush_step==4) //计算平均电流值 测量数量需要实际确定
	 {
		 MOTOR6_avgCurrent=MOTOR6_addCurrent/Roll6_brush_num;
		 Roll6_brush_step=5;
	 }
	 else if(Roll6_brush_step==5) //测量数量需要实际确定
	 {
			if(abs(MOTOR6_avgCurrent)>Roll_brush_ave_cur) //阈值需要实际测量
			{
				Roll_Side_brush_live6=2;           //有毛刷
			}
			else
			{
				Roll_Side_brush_live6=1;           //无毛刷
			}
			MotorControl[6].Motor_Start_Stop=DISABLE; //左边刷失能
			MotorControl[6].Speed_Set=0; //左边刷速度 0
			Roll6_brush_report=2;//自检完成
			Roll6_brush_step=0;
			Roll6_brush_flag=0;
			Roll6_brush_num=0;
			MOTOR6_addCurrent=0;
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
			if(motornum==0xff)  //两个方向标定 按需求来确定
			{
	      MotorControl[motornum].PWM_DutySet=PERCENT_95_OF_PWM_PERIOD;//不能删除否则不能进捕获中断
        MotorControl[motornum].PWM_Duty+=10;		
			}
			else
			{
        MotorControl[motornum].PWM_DutySet=-PERCENT_95_OF_PWM_PERIOD;//不能删除否则不能进捕获中断
        MotorControl[motornum].PWM_Duty-=10;
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
    if(MotorControl[motornum].PWM_Duty < -PERCENT_95_OF_PWM_PERIOD)
    {
        MotorControl[motornum].PWM_Duty = -PERCENT_95_OF_PWM_PERIOD;
    }
    if(motornum == 0)
    {
         MOTOR0CCW();
    }
    else if(motornum == 1)
    {
        MOTOR1CCW();
    }
    else if(motornum == 2)
    {
        MOTOR2CCW();
    }
//    else if(motornum == 9)
//    {
//        MOTOR9CW();
//    }
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
//        else if(motornum == 9)
//        {
//            MOTOR9STOP();
//            MC_ClearFault(PUSH_MOTOR4_OVER_CUR);
//        }

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
//            MC_SetFault1(PUSH1_CALIBRATE);
        }
        else if(motornum == 2)
        {
            MOTOR2STOP();
//            MC_SetFault1(PUSH2_CALIBRATE);
        }
//        else if(motornum == 9)
//        {
//            MOTOR9STOP();
//            MC_SetFault1(PUSH9_CALIBRATE);
//        }
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
void Motor5_Default_Phase_Cheek(void)
{
    if(MotorControl[5].Speed_Real != 0 && MotorControl[5].Motor_Start_Stop == 1 && \
            MotorControl[5].Speed_Set != 0  )
    {
		if(MotorControl[5].Speed_Real<=500)
		{
			Motor5_Current_temp.maxcnt=30000;
		}
		if(MotorControl[5].Speed_Real>500)
		{
			Motor5_Current_temp.maxcnt=30000;
		}
        Motor5_Current_temp.cnt++;
        if(Motor5_Current_temp.cnt>Motor5_Current_temp.maxcnt)   //6.10修改缺相计数由2秒变成3s
        {
            Motor5_Current_temp.cnt = 0;
            BubbleSort(M5_100us_ac_ab_bc_cur,ac_ab_bc_index);
            testM5_100us_ac_ab_bc_cur =(M5_100us_ac_ab_bc_cur[2]-M5_100us_ac_ab_bc_cur[0])/M5_100us_ac_ab_bc_cur[2];//测试观察值
            if(testM5_100us_ac_ab_bc_cur>0.93) //从小到大排序之后最大值-最小值，0.6为测试估计值
            {
									virtual_time2++;
									if(virtual_time2==1) //代码没用
									{ 
										 test_Lack_Count=testM5_100us_ac_ab_bc_cur*1000;
									}
//                if(MotorControl[5].Current.DeepFilterVAL>50)
//                {
                    MC_SetFault(MOTOR5_MISSING_PHASE); //单纯为了指示灯
                    MotorControl[5].Fault_Flag =1;
                    MotorControl[5].Motor_Start_Stop = 0;

//                }
            }
            M5_100us_ac_ab_bc_cur[ab] = 0;
            M5_100us_ac_ab_bc_cur[ac] = 0;
            M5_100us_ac_ab_bc_cur[bc] = 0;
        }
        if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[4]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[1])      //BA  AB电流 
        {
            M5_100us_ac_ab_bc_cur[bc] += MotorControl[5].Current.FilterValue;
        }
        else if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[5]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[2]) //BC  CB电流
        {
            M5_100us_ac_ab_bc_cur[ab] += MotorControl[5].Current.FilterValue;
        }
        else if(MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[0]||\
                MotorControl[5].Hall.HallState == HALL_Study[0].HallTab[3]) //CA  AC电流
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
Explain :其中的0.9的数值需要根据不同的电机进行测试
------------------------------------------------*/
void Motor6_Default_Phase_Cheek(void)
{
    if(MotorControl[6].Speed_Real != 0 && MotorControl[6].Motor_Start_Stop == 1 && \
            MotorControl[6].Speed_Set != 0  )
    {
        Motor6_Current_temp.cnt++;
        if(Motor6_Current_temp.cnt>30000)    //6.15修改
        {
            Motor6_Current_temp.cnt = 0;
            BubbleSort(M6_100us_ac_ab_bc_cur,ac_ab_bc_index);
            testM6_100us_ac_ab_bc_cur =	(M6_100us_ac_ab_bc_cur[2]-M6_100us_ac_ab_bc_cur[0])/M6_100us_ac_ab_bc_cur[2];//测试观察值
            if(testM6_100us_ac_ab_bc_cur>0.9) //从小到大排序之后最大值-最小值，0.5为测试估计值
            {
							virtual_time_1++;
							if(virtual_time_1==1)
								 test_Lack_Count1=testM6_100us_ac_ab_bc_cur*1000; //代码没用
//                if(MotorControl[6].Current.DeepFilterVAL>50)
                {
                    MC_SetFault(MOTOR6_MISSING_PHASE); //单纯为了指示灯
                    MotorControl[6].Fault_Flag =1;
                    MotorControl[6].Motor_Start_Stop = 0;
                }
            }
            M6_100us_ac_ab_bc_cur[ab] = 0;
            M6_100us_ac_ab_bc_cur[ac] = 0;
            M6_100us_ac_ab_bc_cur[bc] = 0;
        }
        if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[4]||\
                MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[1])  //BA  AB 项电流采样
        {
            M6_100us_ac_ab_bc_cur[bc] += MotorControl[6].Current.FilterValue;
        }
        else if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[5]||\
                MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[2]) //BC  CB
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
void Hardware_flowCheck(void)
{
    if(OverFlow_Cnt[0]>=BREAK_CNT_MAX)	/* 无刷5 超出最大错误限制*/
    {
        MC_SetFault(MOTOR5_BREAK);
        MotorControl[5].Fault_Flag = 1;
        MotorControl[5].Motor_Start_Stop = DISABLE;
    }
    if(OverFlow_Cnt[1]>=BREAK_CNT_MAX)	/* 无刷6 超出最大错误限制*/
    {
        MC_SetFault(MOTOR6_BREAK);
        MotorControl[6].Fault_Flag = 1;
        MotorControl[6].Motor_Start_Stop = DISABLE;
    }
    if(OverFlow_Cnt[2]>=BREAK_CNT_MAX)	/* 单向有刷3——4 超出最大错误限制*/
    {
        MC_SetFault(BRAKE_3_4);
        MotorControl[3].Motor_Start_Stop = DISABLE;
        MotorControl[3].Fault_Flag = 1;
        MotorControl[4].Motor_Start_Stop = DISABLE;
        MotorControl[4].Fault_Flag = 1;
    }
    if(OverFlow_Cnt[3]>=BREAK_CNT_MAX)	/* 推杆0_1_2_9 */
    {
        MC_SetFault(BRAKE_0_1_2);
        MotorControl[0].Motor_Start_Stop = DISABLE;
        MotorControl[1].Motor_Start_Stop = DISABLE;
        MotorControl[2].Motor_Start_Stop = DISABLE;
//        MotorControl[9].Motor_Start_Stop = DISABLE;
        MotorControl[0].Fault_Flag = 1;
        MotorControl[1].Fault_Flag = 1;
        MotorControl[2].Fault_Flag = 1;
//        MotorControl[9].Fault_Flag = 1;
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
Function:偏置电压校准 dma采集
Input   :No
Output  :No
Explain :以下两个数组操作，为了对应电流采样通道对应电机电流
------------------------------------------------*/
//u8 Voltage_offset_calibration[8] = {0,1,2,3,4,5,6,9};//上下对起来看 电机号
//u8 ADC_ConvertedValueindex[8] =    {6,7,8,0,1,2,4,9};//上下对起来看 adc通道
u8 Voltage_offset_calibration[8] = {0,2,1,3,4};//上下对起来看 电机号
u8 ADC_ConvertedValueindex[8] =    {6,7,8,0,1};//上下对起来看 adc通道
#define OFFSET_NUM 100
u8 offset_cnt = 0;
u8 cur_calib=0;
u8 cur_Inject_calib=0;
void Voltage_offset_cali(void)
{
    offset_cnt++;
    if(offset_cnt<=OFFSET_NUM)
    {
        for(int i = 0; i < 5; i++)
        {
            MotorControl[Voltage_offset_calibration[i]].Current.offset += ADC_ConvertedValue[ADC_ConvertedValueindex[i]];
        }
        if(offset_cnt == OFFSET_NUM)		/* 取100次的均值 */
        {
            for(int k = 0; k < 5; k++)
            {
                MotorControl[Voltage_offset_calibration[k]].Current.offset = (MotorControl[Voltage_offset_calibration[k]].Current.offset/OFFSET_NUM);
            }
//            MotorState = INIT;
						cur_calib=1;
        }
    }
    else if(offset_cnt>OFFSET_NUM)
    {
        offset_cnt = OFFSET_NUM+1;
//        MotorState = INIT;
			    cur_calib=1;
    }
		if(cur_calib==1 && cur_Inject_calib==1)
		{
		 MotorState = INIT;
		}
}
/*------------------------------------------------
Function:偏置电压校准 注入通道采集
Input   :No
Output  :No
Explain :以下两个数组操作，为了对应电流采样通道对应电机电流
------------------------------------------------*/
u8 Voltage_offset_calibration_Inject[2] = {6,5};//上下对起来看 电机号
u8 ADC_ConvertedValueindex_Inject[2] =    {0,1};//上下对起来看  adc通道
#define OFFSET_NUM_Inject 100
u8 offset_cnt_Inject = 0;
extern uint32_t cur_Inject[2];

void Voltage_offset_cali_Inject(void)
{
    offset_cnt_Inject++;
    if(offset_cnt_Inject<=OFFSET_NUM_Inject)
    {
        for(int i = 0; i < 2; i++)
        {
            MotorControl[Voltage_offset_calibration_Inject[i]].Current.offset += cur_Inject[ADC_ConvertedValueindex_Inject[i]];
        }
        if(offset_cnt_Inject == OFFSET_NUM_Inject)		/* 取100次的均值 */
        {
            for(int k = 0; k < 2; k++)
            {
                MotorControl[Voltage_offset_calibration_Inject[k]].Current.offset = (MotorControl[Voltage_offset_calibration_Inject[k]].Current.offset/OFFSET_NUM_Inject);
            }
 //           MotorState = INIT;
						  cur_Inject_calib=1;
        }
    }
    else if(offset_cnt_Inject>OFFSET_NUM_Inject)
    {
        offset_cnt_Inject = OFFSET_NUM_Inject+1;
//        MotorState = INIT;
			    cur_Inject_calib=1;
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
    if(abs(MotorControl[6].PWM_Duty)>3000 && MotorControl[6].Motor_Start_Stop==1)
    {
        if(MotorControl[5].Direction!=-1&&MotorControl[5].Direction!=1)
        {
            motor_err_cnt5++;
            if(motor_err_cnt5>60)		//累计10次报警
            {
                motor_err_cnt5=60;
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
    if(abs(MotorControl[6].PWM_Duty)>3000 && MotorControl[6].Motor_Start_Stop==1)
    {
        if(MotorControl[6].Direction!=-1&&MotorControl[6].Direction!=1)
        {
            motor_err_cnt6++;
            if(motor_err_cnt6>60)//20次报警
            {
                MC_SetFault(MOTOR6_PHASE_ERROR);
                motor_err_cnt6=60;
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
*Function Name 		:Push_Motor_CurSelfAdapt
*Description   		:推杆电流根据无刷电机实现高度自适应
* Input           : bldc_num ： 无刷电机号
										push_num ： 推杆电机号
										hall_max ： 霍尔下放最大值
										hall_min ： 霍尔最小回收值
* Output          : None
* Return          : None		2021.12.16	by diamond
// Current.SetValue：单位 10ma
*************************/
float cur_ratio[7] = {0,0,0,0,0,MOTOR5CURCOEFFICIENT,MOTOR6CURCOEFFICIENT};
int16_t limitpwm = 4000;
uint8_t hight_err_cnt = 0;
//int CurSelfAdapt_Value=20;  //偏差值0.2A
int CurSelfAdapt_Value=0.2*MOTOR5CURCOEFFICIENT*100;  //偏差值0.2A
int CurSelfAdapt_Time=1000; //滤波时间1s
int CurSelfAdapt_Convert_Time;
void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min)             //15ms一次
{
    CurSelfAdapt_Convert_Time=CurSelfAdapt_Time/15;
	
	  if(MotorControl[bldc_num].Current.SetValue >CurSelfAdapt_cur) //滚刷自适应电流现在 /21单位 A
		{
		 MotorControl[bldc_num].Current.SetValue =CurSelfAdapt_cur;
		}
	  if(MotorControl[bldc_num].Current.SetValue <-CurSelfAdapt_cur)
		{
		 MotorControl[bldc_num].Current.SetValue =-CurSelfAdapt_cur;
		}
    if(MotorControl[bldc_num].Current.SetValue!=0)
    {
        if(abs(MotorControl[bldc_num].Speed_Real)>200) //:2022.4.28 readme解释
        {
            if(MotorControl[push_num].Motor_Start_Stop ==DISABLE)
                MotorControl[push_num].Motor_Start_Stop = ENABLE;
            if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue)<MotorControl[bldc_num].Current.CurOffset )//目标-测试值小于0.8A关闭推杆
            {
                MotorControl[push_num].PWM_DutySet = 0;
							  PID_PWM.wIntegral=0; //积分清零
                if(hight_err_cnt>0)
                    hight_err_cnt--;
            }
            else
            {
                if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue)>(MotorControl[bldc_num].Current.CurOffset+CurSelfAdapt_Value))//目标-测试值大于1A 启动推杆
                    hight_err_cnt++;
                if(hight_err_cnt>CurSelfAdapt_Convert_Time)          //200对应3s
                {
                    hight_err_cnt=CurSelfAdapt_Convert_Time;
                    MotorControl[push_num].PWM_DutySet = PID_Regulator(abs(MotorControl[bldc_num].Current.SetValue),f_abs(MotorControl[bldc_num].Current.DeepFilterVAL),&PID_PWM);
                    if(f_abs(MotorControl[push_num].PWM_DutySet)<800)
                    {
                        MotorControl[push_num].PWM_DutySet = 0;
											 PID_PWM.wIntegral=0;
                    }
//                    if(MotorControl[push_num].PWM_DutySet>3000) //让推杆下降慢点
//                    {
//                        MotorControl[push_num].PWM_DutySet = 3000;
//                    }
                }
            }
            if(MotorControl[push_num].Hall.HALL_CaptureValue>hall_max&&MotorControl[push_num].PWM_DutySet>0)   
						{
						  MotorControl[push_num].PWM_DutySet = 0;
							PID_PWM.wIntegral=0;
						}							
							
            if(MotorControl[push_num].Hall.HALL_CaptureValue<hall_min&&MotorControl[push_num].PWM_DutySet<0)  
						{
						  MotorControl[push_num].PWM_DutySet = 0;
							PID_PWM.wIntegral=0;
						}							
							
        }
    }
		else
		{
		 hight_err_cnt=0;
		 PID_PWM.hOutput=0;
		 PID_PWM.OutPreSat=0;
		}
}
/*************************
*Function Name 		:Motor7_Err_Chk
*Description   		:边刷电机错误检测  2S没速度报警
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
/*
485控制边刷时的错误
1、母线过流保护阈值：3A，过流发生后触发停机保护，5S 后解除
2、电机过温保护阈值：80℃，温度低于阈值 5℃后解除保护
3、相电流保护阈值：4.6A，过流发生后触发停机保护，5S 后解除
4、堵转保护：堵转持续时间超过 5S 触发保护，5S 后解除
*/
void Motor7_Err_Chk(void)		/* 200ms循环里 */
{
    static u8 err_cur_cnt = 0;
		 if(MotorControl[7].Motor_mode == Control_IO) //
		 {			 
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
		 if(MotorControl[7].Motor_mode == Control_485) //
		 {
				if(MotorControl[7].Bm_fault_code !=0)//右边刷报错或者断联
				{
					  MotorControl[7].PWM_Duty = 0;
						MotorControl[7].Motor_Start_Stop = DISABLE;  
						MC_SetFault(SIDE_BRUSH_ERROR);	
						if(MotorControl[7].Bm_fault_code & 0x01)//传感器故障
						{
						MC_SetFault1(SB1_SENSOR_ERR);
						}
					 if(MotorControl[7].Bm_fault_code & 0x02)//过流故障
						{
						MC_SetFault1(SB1_CUR_ERR);
						}
						 if(MotorControl[7].Bm_fault_code & 0x04)//相电流过流
						{
						MC_SetFault1(SB1_PHASECUR_ERR);	
						}
						 if(MotorControl[7].Bm_fault_code & 0x08)//堵转故障
						{
						MC_SetFault1(SB1_LOCKED_ERR);	
						}
						if(MotorControl[7].Bm_fault_code & 0x10)//过温故障
						{
						MC_SetFault1(SB1_TEMP_ERR);	
						}						
				}
				else
				{
						MC_ClearFault(SIDE_BRUSH_ERROR);
				}
		}
}
/*************************
*Function Name 		:Motor9_Err_Chk
*Description   		:边刷电机错误检测  2S没速度报警
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
void Motor9_Err_Chk(void)		/* 200ms循环里 */
{		
    static u8 err_cur_cnt1 = 0;
		 if(MotorControl[8].Motor_mode == Control_IO) //IO控制
		 {		
				if(MotorControl[8].PWM_Duty>500&&MotorControl[8].Speed_Real==0)
				{
						err_cur_cnt1++;
						if(err_cur_cnt1==20)
						{
								MotorControl[8].Fault_Flag =1;
								MotorControl[8].Motor_Start_Stop = DISABLE;
								MC_SetFault(SIDE2_BRUSH_ERROR);
						}
				}
				else
						err_cur_cnt1=0;
	   }
		 if(MotorControl[8].Motor_mode == Control_485) //
		 {			 
				if(MotorControl[8].Bm_fault_code !=0)//右边刷报错或者断联
				{
					  MotorControl[8].PWM_Duty = 0;
						MotorControl[8].Motor_Start_Stop = DISABLE;  
						MC_SetFault(SIDE2_BRUSH_ERROR);	
						if(MotorControl[8].Bm_fault_code & 0x01)//传感器故障
						{
						MC_SetFault1(SB2_SENSOR_ERR);
						}
					 if(MotorControl[8].Bm_fault_code & 0x02)//过流故障
						{
						MC_SetFault1(SB2_CUR_ERR);
						}
						 if(MotorControl[8].Bm_fault_code & 0x04)//相电流过流
						{
						MC_SetFault1(SB2_PHASECUR_ERR);	
						}
						 if(MotorControl[8].Bm_fault_code & 0x08)//堵转故障
						{
						MC_SetFault1(SB2_LOCKED_ERR);	
						}
						if(MotorControl[8].Bm_fault_code & 0x10)//过温故障
						{
						MC_SetFault1(SB2_TEMP_ERR);	
						}						
				}
				else
				{
						MC_ClearFault(SIDE2_BRUSH_ERROR);
				}
		}
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
    if(GetMotorSpeed(Motor5)==0&&MotorControl[5].PWM_Duty!=0 && MotorControl[5].Motor_Start_Stop == 1)	//BLDC1带载时缺相，电流为0
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
			      Bldc_lack_flag=1;
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

    if(GetMotorSpeed(Motor6)==0&&MotorControl[6].PWM_Duty!=0 && MotorControl[6].Motor_Start_Stop == 1)	//BLDC1带载时缺相，电流为0
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
            MC_SetFault(MOTOR6_MISSING_PHASE);//M6缺相
            M6phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[6].Fault_Flag =1;
            MotorControl[6].Motor_Start_Stop = 0;

        }
        if(M6stuck_check_cnt>SUTCK_ERR_MAX)
        {
//            MC_SetFault(MOTOR6_PHASE_ERROR); //M6相序错误
//            M6stuck_check_cnt = SUTCK_ERR_MAX;
//            MotorControl[6].Fault_Flag =1;
//            MotorControl[6].Motor_Start_Stop = 0;

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
/*************************
*Function Name 		:BLDC2_OverSpdChk
*Description   		:BLDC2失速报警  4s连续速度异常报警
* Input           : None
* Output          : None
* Return          : None		2021.12.17	by diamond
*************************/
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
u8 timer_cnt[4]={0};
void Push_OverRunChk(u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3)
{
//    static u8 timer_cnt[4];
    if(push1)
    {
        if(MotorControl[0].Motor_Start_Stop==ENABLE&&ABS(MotorControl[0].PWM_Duty)>500)
        {
					if(MotorControl[0].Current.FilterValue>30)
					{				
						timer_cnt[0]++;
						if(timer_cnt[0]>timer0*2)
						{
							MotorControl[0].Motor_Start_Stop=DISABLE;
							MotorControl[0].Fault_Flag = 1;
//							MC_SetFault(PUSH_OVER_TIME);
//							MC_SetFault1(PUSH0_OVERTIME);
							timer_cnt[0]=0;
					  }
			   }
			     else timer_cnt[0]=0;
		   }
      else timer_cnt[0]=0;       
    }
    if(push2)
    {
        if(MotorControl[1].Motor_Start_Stop==ENABLE&&ABS(MotorControl[1].PWM_Duty)>500)
        {
					if(MotorControl[1].Current.FilterValue>30)
					{	
            timer_cnt[1]++;
            if(timer_cnt[1]>timer1*2)
            {
                MotorControl[1].Motor_Start_Stop=DISABLE;
//			        	MotorControl[1].Fault_Flag = 1;
//                MC_SetFault(PUSH_OVER_TIME);
                timer_cnt[1]=0;
            }
					}
					else timer_cnt[1]=0;
        }
        else timer_cnt[1]=0;
    }
    if(push3)
    {
        if(MotorControl[2].Motor_Start_Stop==ENABLE&&ABS(MotorControl[2].PWM_Duty)>500)
        {
					if(MotorControl[2].Current.FilterValue>30)
					{
            timer_cnt[2]++;
            if(timer_cnt[2]>timer2*2)
            {
                MotorControl[2].Motor_Start_Stop=DISABLE;
                MC_SetFault(PUSH_OVER_TIME);
                MC_SetFault1(PUSH2_OVERTIME);
                timer_cnt[2]=0;
            }
					}
					else timer_cnt[2]=0;
        }
        else timer_cnt[2]=0;
    }
//    if(push4)
//    {
//        if(MotorControl[9].Motor_Start_Stop==ENABLE&&ABS(MotorControl[9].PWM_Duty)>500)
//        {
//            timer_cnt[3]++;
//            if(timer_cnt[3]>timer3*2)
//            {
//                MotorControl[9].Motor_Start_Stop=DISABLE;
////                MC_SetFault(PUSH_OVER_TIME);
//                MC_SetFault1(PUSH9_OVERTIME);
//                timer_cnt[3]=0;
//            }
//        }
//        else timer_cnt[3]=0;
//    }
}
