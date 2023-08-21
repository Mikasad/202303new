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
//��ת�ṹ��
typedef struct OLCKED_ROTOR_Ttag
{
    long pStuckCurr;
    long pStuckTime;
    long pStuckVel;
} OLCKED_ROTOR_T;
/*------------------------------------------------
Function:MCU�汾��
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver)
{
    pVersion->uVersionPartOfRobotType = robotype;				//��Ŀ����
    pVersion->uVersionPartOfMainVersion = main_ver;			//��汾��
    pVersion->uVersionFullVersion = fun_ver;						//���ܰ汾��
    pVersion->uVersionPartOfSmallVersion = small_ver;		//С�汾�ţ�����bug�Լ�����΢��
    pVersion->uVersionFullVersion = (robotype<<24)+(main_ver<<16)+(fun_ver<<8)+small_ver;
}
/*------------------------------------------------
Function:Ӳ���汾��
Input   :&MCU_Version ,funtype��vol��cur_max��update_ver
Output  :No
Explain :No
------------------------------------------------*/
void HardVersion_Init(MCU_Version *pVersion,u8 funtype,u8 vol,u8 cur_max,u8 update_ver)
{
    pVersion->HardwarePartOfMotorType = funtype;        //������ͣ����H,�ŷ�S,����B
    pVersion->HardwarePartOfVotage = vol;                //������ѹ  A��24V
    pVersion->HardwarePartOfCurrent = cur_max;          //������
    pVersion->HardwarePartOfVersion = update_ver;        //���µİ汾��
    pVersion->HardwareFullVersion = (funtype<<24)+(vol<<16)+(cur_max<<8)+update_ver;
}
/*------------------------------------------------
Function:������������
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void CurrentLimit(u8 Motor_NUM)
{
    if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue1)		/* 1�׶ζ�����ˢ��ʱû�ã��ᱻPID���ڻ��� */
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
Function:���ô����־
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
        FaultOccurred  |= hFault_type; //ֻ������ǰ��������
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
        FaultOccurred1  |= hFault_type; //ֻ������ǰ��������
}
/*------------------------------------------------
Function:��������־
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_ClearFault(u32 hFault_type)
{
    wGlobal_Flags &= ~hFault_type;
    FaultOccurred &= ~hFault_type; //�����ǰ��������

}
/*------------------------------------------------
Function:�������
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Motor_Fault_Clear(void)
{
    if(OverFlow_Cnt[3]<BREAK_CNT_MAX) //���brake����5��֮��Ͳ��������
    {
        if(((wGlobal_Flags&(PUSH_MOTOR1_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2_9))==0)&&MotorControl[0].Fault_Flag == 1)//�Ƹ�0�������
        {
            MotorControl[0].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR2_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2_9))==0)&&MotorControl[1].Fault_Flag == 1)//�Ƹ�1�������
        {
            MotorControl[1].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR3_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2_9))==0)&&MotorControl[2].Fault_Flag == 1)//�Ƹ�2�������
        {
            MotorControl[2].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(PUSH_MOTOR4_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL|BRAKE_0_1_2_9))==0)&&MotorControl[9].Fault_Flag == 1)//�Ƹ�9�������
        {
            MotorControl[9].Fault_Flag = 0;
        }
		
    }
    if(OverFlow_Cnt[2]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(ONEWYA_MOTOR1_OVER_CUR|MOTOR3_BRAKE_LINE|BRAKE_3_4))==0)&&MotorControl[3].Fault_Flag == 1)//���3�������
        {
            MotorControl[3].Fault_Flag = 0;
        }
        if(((wGlobal_Flags&(ONEWYA_MOTOR2_OVER_CUR|MOTOR4_BRAKE_LINE|BRAKE_3_4))==0)&&MotorControl[4].Fault_Flag == 1)//���4�������
        {
            MotorControl[4].Fault_Flag = 0;
        }
    }
    if(OverFlow_Cnt[0]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(BLDC1_OVER_CUR|HALL5_SENSOR_ERR|MOTOR5_PHASE_ERROR|MOTOR5_OVER_SPEED|MOTOR5_MISSING_PHASE|MOTOR5_BREAK))==0)&&MotorControl[5].Fault_Flag == 1)//���5������� ��ˢ���
        {
            MotorControl[5].Fault_Flag = 0;
        }
    }

    if(OverFlow_Cnt[1]<BREAK_CNT_MAX)
    {
        if(((wGlobal_Flags&(BLDC2_OVER_CUR|HALL6_SENSOR_ERR|MOTOR6_PHASE_ERROR|MOTOR6_OVER_SPEED|MOTOR6_MISSING_PHASE|MOTOR6_BREAK))==0)&&MotorControl[6].Fault_Flag == 1)//���6������� ��ˢ���
        {
            MotorControl[6].Fault_Flag = 0;
        }
    }
    if(((wGlobal_Flags&SIDE_BRUSH_ERROR)==0)&&MotorControl[7].Fault_Flag == 1)//���7������� ��ˢ
    {
        MotorControl[7].Fault_Flag = 0;
    }
    if(((wGlobal_Flags&FAN_ERROR)==0)&&MotorControl[8].Fault_Flag == 1)//���8������� ���
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
Function:����ֵ������float���ͣ�
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
Function:����ֵ������long���ͣ�
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
Function:��������
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
Function:��ѹ���
Input   :No
Output  :No
Explain :��ѹ�ָ���20V  ��ѹ�ָ���30V  ��ѹ��36V Ƿѹ��16V 500ms�澯
------------------------------------------------*/
void Over_VoltageCheck(void)  //10ms
{
    if(VoltVar.BUS>VoltVar.BusHigh)//��ѹ
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
Function:5,6�ŵ��ѡ��
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
int kp_Value=500;
int ki_Value=10;
void Motor_Choose(void)
{
	if(Motor_Type==0)                  //��ˢ���ѡ��0����ˢ���   1����ˢ���
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
		PID_Speed_InitStruct[0].hLower_Limit_Output= -MotorControl[5].Current.MaxValue3*2;   //Lower Limit for Output limitation
        PID_Speed_InitStruct[0].hUpper_Limit_Output= MotorControl[5].Current.MaxValue3*2;   //Upper Limit for Output limitation
        PID_Speed_InitStruct[0].wLower_Limit_Integral = -MotorControl[5].Current.MaxValue3 * SP_KIDIV;
        PID_Speed_InitStruct[0].wUpper_Limit_Integral = MotorControl[5].Current.MaxValue3 * SP_KIDIV;
		
		PID_Speed_InitStruct[1].hLower_Limit_Output= -MotorControl[5].Current.MaxValue3*2;   //Lower Limit for Output limitation
        PID_Speed_InitStruct[1].hUpper_Limit_Output= MotorControl[5].Current.MaxValue3*2;   //Upper Limit for Output limitation
        PID_Speed_InitStruct[1].wLower_Limit_Integral = -MotorControl[5].Current.MaxValue3 * SP_KIDIV;
        PID_Speed_InitStruct[1].wUpper_Limit_Integral = MotorControl[5].Current.MaxValue3 * SP_KIDIV;
	}
	else if(Motor_Type==1)
	{
		MotorControl[5].Pole_Paires=3;
		MotorControl[6].Pole_Paires=3;
		if(MotorControl[5].Speed_PWN_Select>=0)
		{
			MotorControl[5].Speed_Set=-MotorControl[5].Speed_PWN_Select; //��ˢ���5����Ҫ��ת
		}
		else
		{
			MotorControl[5].Speed_Set=0;//��λ���·�����ת����Ϊ0
		}
		MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(12) ;
		MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(14) ;
		MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(15) ;
		MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(16) ;
	
        MotorControl[6].Speed_Set=MotorControl[6].Speed_PWN_Select;		//��ˢ�����ֱ�Ӹ�ֵ���ٶ�
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
			if(MotorControl[5].Speed_Real<-1500)   //5�ŵ������ת��
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
		MotorControl[5].PWM_DutySet=MotorControl[5].Speed_PWN_Select*14/5;  //1��3���޸�
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
Function:������ʼ��
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
//	   MotorControl[10].PWM_DutySet = 8400;		/* S�ߵ�����ˢ������24V */   50����Ҫ
//    MotorControl[12].PWM_DutySet = 8400;		/* S�ߵ�����ˢ������24V */
//    MotorControl[10].Motor_Start_Stop = 1;
//    MotorControl[12].Motor_Start_Stop = 1;
    /************************�Ƹ˳�ʼ��***************************/
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
    /*********************�Ƹ˳�ʼ��end***************************/

    /*************************������ˢ3_4***************************/
    MotorControl[3].Current.MaxValue1 = MOTOR3_CurrentValue(3) ;  //���˵��
    MotorControl[3].Current.MaxValue2 = MOTOR3_CurrentValue(4) ;
    MotorControl[3].Current.MaxValue3 = MOTOR3_CurrentValue(5) ;  //���˵��
    MotorControl[3].Current.MaxValue4 = MOTOR3_CurrentValue(6) ;
    MotorControl[3].Current.OFCnt1_T = 2000;
    MotorControl[3].Current.OFCnt2_T = 30000;
    MotorControl[3].Current.OFCnt3_T = 10000;
    MotorControl[3].Current.OFCnt4_T = 2000;

    MotorControl[4].Current.MaxValue1 = MOTOR4_CurrentValue(3) ;  //��ˮ���
    MotorControl[4].Current.MaxValue2 = MOTOR4_CurrentValue(2) ;
    MotorControl[4].Current.MaxValue3 = MOTOR4_CurrentValue(3) ;  //��ˮ���
    MotorControl[4].Current.MaxValue4 = MOTOR4_CurrentValue(4) ;
    MotorControl[4].Current.OFCnt1_T = 2000;
    MotorControl[4].Current.OFCnt2_T = 50000;
    MotorControl[4].Current.OFCnt3_T = 10000;
    MotorControl[4].Current.OFCnt4_T = 2000;

    /*************************������ˢ3_4 end***************************/

    /*************************��ˢ5��6***************************/
    MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(12) ;
    MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(14) ;
    MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(15) ;
    MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(16) ;
    MotorControl[5].Current.CurOffset = 0.2*MOTOR5CURCOEFFICIENT*100;		//Ĭ��0.3A
    MotorControl[5].Current.OFCnt1_T = 60000; //û��
    MotorControl[5].Current.OFCnt2_T = 100000;//10��
    MotorControl[5].Current.OFCnt3_T = 50000; //5��
    MotorControl[5].Current.OFCnt4_T = 10000; //1��
    MotorControl[5].Pole_Paires = 4;
    MotorControl[5].Acceleration = 5;
    MotorControl[5].Deceleration = 5;

    MotorControl[6].Current.MaxValue1 = MOTOR6_CurrentValue(12) ;
    MotorControl[6].Current.MaxValue2 = MOTOR6_CurrentValue(14) ;
    MotorControl[6].Current.MaxValue3 = MOTOR6_CurrentValue(15) ;
    MotorControl[6].Current.MaxValue4 = MOTOR6_CurrentValue(16) ;
    MotorControl[6].Current.CurOffset = 0.3*MOTOR6CURCOEFFICIENT*100;		//Ĭ��0.3A
    MotorControl[6].Current.OFCnt1_T = 60000; //û��
    MotorControl[6].Current.OFCnt2_T = 100000;//10��
    MotorControl[6].Current.OFCnt3_T = 50000; //5��
    MotorControl[6].Current.OFCnt4_T = 10000; //1��
    MotorControl[6].Pole_Paires = 4;
    MotorControl[6].Acceleration = 5;
    MotorControl[6].Deceleration = 5;
    /*************************��ˢ5��6end***************************/
    PID_Init (&PID_Speed_InitStruct[0],&PID_Current_InitStructure[0]);		//���5��PID
    PID_Init (&PID_Speed_InitStruct[1],&PID_Current_InitStructure[1]);		//���6��PID
    PID_PWM_Init(&PID_PWM);																								//�Ƹ�0��PID
    /*	�Ӽ��ٶȳ�ʼ��	*/
    MotorControl[0].Acceleration = 10;
    MotorControl[0].Deceleration = 10;
    MotorControl[0].LastMotorDirection = 2;
    MotorControl[0].Hall.HALL_CaptureValue = 0; //�����ϵ�궨
    MotorControl[1].Acceleration = 10;
    MotorControl[1].Deceleration = 10;
    MotorControl[1].LastMotorDirection = 2;
    MotorControl[1].Hall.HALL_CaptureValue = 10000; //�����Ƹ�1��Ҫλ�ÿ��ƣ����궨�����Push_Location_model���أ�������ֱ�Ӹ����궨�����ʱ��ǰֵΪ0
    MotorControl[2].Acceleration = 10;
    MotorControl[2].Deceleration = 10;
    MotorControl[2].LastMotorDirection = 2;
    MotorControl[2].Hall.HALL_CaptureValue = 0; //�����ϵ�궨

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
    MotorControl[9].Hall.HALL_CaptureValue = 0; //�����ϵ�궨

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

    /*           ����ѧϰ����                */
    HALL_Study[1].StudySectorCnt3 = 200;/*����һ��ʱ��1����2ms*/
    HALL_Study[1].HallSector = 1;
    HALL_Study[1].StudySectorCnt = 0;
    HALL_Study[1].HallCommPWM = 4500;/*��������PWMֵ*/

    HALL_Study[0].StudySectorCnt3 = 200;/*����һ��ʱ��1����2ms*/
    HALL_Study[0].HallSector = 1;
    HALL_Study[0].StudySectorCnt = 0;
    HALL_Study[0].HallCommPWM = 4500;/*��������PWMֵ*/

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
    MotorControl[1].Push_Location_model=0; //���Ƹ˵����Ҫ�궨���궨�ɹ��ſ���λ�ÿ��ƣ�����������Ҫ�궨Ĭ�Ͽ���λ�ÿ��ƣ�ֻ������������Ŀ���û��λ�ÿ���
    MotorControl[2].Push_Location_model=1;
    MotorControl[9].Push_Location_model=1;
}
/*------------------------------------------------
Function:led������
Input   :led1: 1/��������  0/�����Ʋ���
				 led2: 1/��������  0/�����Ʋ���
				 led_time:��˸�Ĵ���
				 led_mode:1/����һ�� 0/����
				 led_speed:��˸���ٶ�
Output  :No
Explain :No
------------------------------------------------*/
void LEDSet(u8 led1,u8 led2,u8 led_time,u8 led_mode,u8 led_speed)
{
    //led1
    if(led1==0)
        RED_LED1_OFF;
    else if(led_mode)	//����
    {
        if(DisplayCounter<2*led_speed)
            RED_LED1_ON;
        else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
            RED_LED1_TOGGLE;
    }
    else if(DisplayCounter<=(2*led_time)*led_speed)	//����
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
Function:����������ָʾ
Input   :No
Output  :No
Explain :10ms����һ��
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
        case PUSH_MOTOR1_OVER_CUR: //�Ƹ�1��˸1��
            LEDSet(LED_OFF,LED_ON,1,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR2_OVER_CUR: //�Ƹ�2��˸2��
            LEDSet(LED_OFF,LED_ON,2,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR3_OVER_CUR: //�Ƹ�3��˸3��
            LEDSet(LED_OFF,LED_ON,3,0,LED_HIGH_SPEED);

            break;

        case PUSH_MOTOR4_OVER_CUR: //�Ƹ�4��˸4��
            LEDSet(LED_OFF,LED_ON,4,0,LED_HIGH_SPEED);


            break;
        case BLDC1_OVER_CUR: //BLDC����5��
            LEDSet(LED_OFF,LED_ON,5,0,LED_HIGH_SPEED);


            break;
        case BLDC2_OVER_CUR: //BLDC2��������6��
            LEDSet(LED_OFF,LED_ON,6,0,LED_HIGH_SPEED);


            break;
        case ONEWYA_MOTOR1_OVER_CUR:	//������ˢ1��������7��
            LEDSet(LED_OFF,LED_ON,7,0,LED_HIGH_SPEED);


            break;
        case ONEWYA_MOTOR2_OVER_CUR:		//������ˢ2��������8��
            LEDSet(LED_OFF,LED_ON,8,0,LED_HIGH_SPEED);
            break;

        case HALL5_SENSOR_ERR:    //��ˢ1��������������
            LEDSet(LED_ON,LED_OFF,1,1,LED_LOW_SPEED);		//����

            break;

        case HALL6_SENSOR_ERR:		//��ˢ2�������������� ����1��
            LEDSet(LED_OFF,LED_ON,1,1,LED_LOW_SPEED);

            break;

        case OVER_VOLTAGE:  //��ѹ�����Ƴ���
            RED_LED1_ON;
            RED_LED2_ON;
            DisplayCounter=0;
            break;
        case UNDER_VOLTAGE:   //Ƿѹ��������
            RED_LED2_OFF;
            RED_LED1_ON;
            DisplayCounter=0;
            break;
        case MOTOR5_BREAK:		//��ˢ1brake�������һ��
            LEDSet(LED_ON,LED_OFF,1,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_BREAK:		//��ˢ2brake�������2��
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case BRAKE_3_4:				//������ˢ3,4brake�������3��
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case BRAKE_0_1_2_9:		//�Ƹ�brake�������4��
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;
        case BRAKE10_11_12_13:	//��ˢbrake�������5��
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;

            //6��

        case MOTOR5_PHASE_ERROR:			//��ˢ1�����������7��
            LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_PHASE_ERROR:					//��ˢ2�����������8��
            LEDSet(LED_ON,LED_OFF,8,0,LED_HIGH_SPEED);
            break;

        case CAN_COMMUNICATION_ERR:	//CANͨ�Ŵ��������ƿ���1��
            LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
            break;

        case MOTOR5_MISSING_PHASE: //��ˢ1ȱ��������ƿ�����˸2��
            LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
            break;

        case MOTOR6_MISSING_PHASE: //��ˢ2ȱ��������ƿ�����˸3��
            LEDSet(LED_ON,LED_ON,3,0,LED_HIGH_SPEED);

            break;
        case MOTOR3_BRAKE_LINE:  //���3����������ƿ�����˸4��
            LEDSet(LED_ON,LED_ON,4,0,LED_HIGH_SPEED);

            break;
        case MOTOR4_BRAKE_LINE: //����Ķ���������ƿ�����˸5��
            LEDSet(LED_ON,LED_ON,5,0,LED_HIGH_SPEED);

            break;
        case SIDE_BRUSH_ERROR:  //��ˢ����������ƿ�����˸6��
            LEDSet(LED_ON,LED_ON,6,0,LED_HIGH_SPEED);

            break;
        case MOTOR5_OVER_SPEED:  //��ˢ1����������ƿ�����˸7��
            LEDSet(LED_ON,LED_ON,7,0,LED_HIGH_SPEED);

            break;
        case MOTOR6_OVER_SPEED:  //��ˢ2����������ƿ�����˸8��
            LEDSet(LED_ON,LED_ON,8,0,LED_HIGH_SPEED);

            break;
        case PUSH_BRAKE_LINE:  //�Ƹ˶���		���泣�������������˸1-4��
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
        case PUSH_LOST_HALL:  //��������  	���泣�������������˸5-8��
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
        case PUSHMOTOR_INT_ERROR:			//����������泣��������1-5��
            LEDSet(LED_ON,LED_ON,2,0,LED_HIGH_SPEED);
            RED_LED2_ON;
            break;
        case PUSH_OVER_TIME:		//�Ƹ˱궨ʧ�����泣��������6-9��
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
Function:�Ƹ˵��λ�û�
Input   :No
Output  :No
Explain :if����ִ�е�����Ҫ��������Ӧ���Ƹˣ����û���Ƹ�����Ӧ���Ǿ�λ�ÿ���
------------------------------------------------*/
u16 Inplacecnt=0;
u16 MOTOR1_Inplacecnt=0;
int16_t MOTOR1_Adapt_Inplacecnt=0;
u8 Push_Move_Flag=0;
u8 Push_Motor_Location_cnt=0;
u8 First_Direction,Second_Direction=0;
u16 Record_PWMDuty=0;
u8 Push_Change_Flag=0;
u16 test_value9=500;
uint16_t Motor_HALL_CaptureValue; 
void Push_Motor_Location_Control(u8 motornum)
{
		if(MotorControl[1].Location_Set<=0)
	{
		MotorControl[1].Location_Set=-100;
		Inplacecnt++;
		if(Inplacecnt>3500)   //7s
		{
			MotorControl[1].Hall.HALL_CaptureValue=0;
			Inplacecnt=0;
//			Push_Move_Flag=2;
		}
		else if((MotorControl[1].Hall.HALL_CaptureValue<100)&&MotorControl[1].Current.FilterValue<=3)  //8s
		{
			MOTOR1_Inplacecnt++;
			if(MOTOR1_Inplacecnt>=50)
			{
				MOTOR1_Inplacecnt=0;
				MotorControl[1].Hall.HALL_CaptureValue=0;
				Inplacecnt=0;
//				Push_Move_Flag=2;
			}
		}
	}
	else
	{
		Inplacecnt=0;
		MOTOR1_Inplacecnt=0;
	}
    if(motornum == 1) //50������ˢ�Ƹˣ�����Ԥ����ˢ����Ӧ
    {
        if(MotorControl[motornum].Push_motor_calibrationFLAG == 2)//�궨��ȷ����ʼִ��λ�ÿ���
        {
			if(Push_Move_Flag==2)
			{
				if(MotorControl[5].Current.SetValue ==0)	//û������������Ӧ�����������λ�ÿ���
				{
					if(MotorControl[motornum].Hall.HALL_CaptureValue > MotorControl[motornum].Location_Set)//�ջأ��Ƹ�1λ�û�
					{
						MotorControl[motornum].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
					}
					if(MotorControl[motornum].Hall.HALL_CaptureValue < MotorControl[motornum].Location_Set)
					{
						MotorControl[motornum].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
					}
					if(f_abs(MotorControl[motornum].Hall.HALL_CaptureValue- MotorControl[motornum].Location_Set) < 3)
					{
						MotorControl[motornum].PWM_DutySet = 0;
					}
				}
			}
			     Push_Motor_Location_cnt++;
				if(Push_Motor_Location_cnt==1)
				{
					First_Direction=MotorControl[1].Direction;
				}
				else if(Push_Motor_Location_cnt==2)
				{
					Push_Motor_Location_cnt=0;
					Second_Direction=MotorControl[1].Direction;
				}
				
				if((First_Direction==CCW && Second_Direction==CW)||(First_Direction==CW && Second_Direction==CCW))
				{
					Push_Change_Flag=1;
				}
				if(Push_Change_Flag==1)
				{
					Inplacecnt=0;
					MOTOR1_Adapt_Inplacecnt++;
					if(MOTOR1_Adapt_Inplacecnt<=test_value9)
					{
						MotorControl[motornum].PWM_DutySet = 0;
						Push_Move_Flag=1;
					}
					else
					{
						Push_Change_Flag=0;
						Push_Move_Flag=2;
						MOTOR1_Adapt_Inplacecnt=0;
					}
				}
				else
				{
					Push_Move_Flag=2;
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

Function:4·�Ƹ˵���������Ͷ������
Input   :No
Output  :No
Explain : 100us
------------------------------------------------*/
u8 NeedToCheekPush[MOTOR_NUM] = {3,4,0,2,0,0,0,0,0,0,0,0,0,0}; //������Ƹ˵����Ҫ������������Ƹ˶������ 0 1 2 9�ŵ��Ĭ�϶��򿪼��
u8 NeedToMissingHallPush[MOTOR_NUM] = {0,1,0,0,0,0,0,0,0,0,0,0,0,0}; //��������ȷ���Ƹ��Ƿ���Ҫ��������
uint16_t PushMaxPulse[MOTOR_NUM] = {780,560,4000,0,0,0,0,0,0,4000,0,0,0,0}; //���������Ҫȷ�������Ƹ˵����ֵ������д����ȷֵ���������Ƹ��Ƶ���ͷ������
u32 Push_Error_Cheek_Cnt[MOTOR_NUM] = {0},Push_Error_Cheek_Cnt_1[MOTOR_NUM] = {0},Push_Error_Cheek_Cnt2[MOTOR_NUM] = {0};
int Push_curr_temp[MOTOR_NUM] = {0},Push_curr_temp_1[MOTOR_NUM] = {0};
#define PUSH_PROTECT_MIN_VAL 2999
#define PUSH_PROTECT_MIN_CUR  50
#define MOTOR34_PROTECT_MIN_CUR 50
#define MOTOR34_PROTECT_MIN_VAL 2999
u8 CurChkArray[2]= {0};
u8 CurChkArray1[2]= {0};
int test_value=0;
int test_valuex[10]={0};
u16 brake_flag_cnt=0;
u16 brake_flag_cnt1=0;
void Push_Motor_Cheek(void)
{
    for(u8 i=0; i<MOTOR_NUM; i++)
    {
        if(NeedToCheekPush[i]==1) //������Ƹ˵����Ҫ������������Ƹ˶������
        {
            /*ռ�ձȴ���2999�ű��������ʹ�ܡ�δ�ﵽʵ��λ�á�PWM����һ��ֵ������ֵ���Ƹ�������ֵ��Χ֮��*/
            if(MotorControl[i].Motor_Start_Stop == 1 && (l_abs(MotorControl[i].Location_Set-MotorControl[i].Hall.HALL_CaptureValue))>20 && \
                    l_abs(MotorControl[i].PWM_Duty) > PUSH_PROTECT_MIN_VAL && MotorControl[i].Hall.HALL_CaptureValue < PushMaxPulse[i])
            {
                /*����ֵ�͵���ֵ�ۼ�Ӧ�÷��ڵ������ʱ*/
                Push_Error_Cheek_Cnt[i]++;
                Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
                /*10012000��С������ʱ����ͬʱִ��Push_Error_Cheek_Cnt[i]%51 �� Push_Error_Cheek_Cnt[i]%100==0�����޳������������Ϊ������1000ʱ��ֵ����2000ʱ�жϣ����ͬʱ��ֵ���жϣ�����ֵ��û�иı���������*/
                if(Push_Error_Cheek_Cnt[i]%2002000==0)
                {
                    Push_Error_Cheek_Cnt[i]++;
                }
                /*100.1ms������ֵ����һ�Σ�������һ�ζ��߼��*/
                if(Push_Error_Cheek_Cnt[i]%1001==0)
                {
                    /*ÿ100ms����һ�λ���ֵ�ĸ���*/
                    MotorControl[i].Hall.HALL_CaptureValueDelta = MotorControl[i].Hall.HALL_CaptureValue;
                    /*���һ��ʱ����ۼƵ���ֵС���趨ֵ�����϶�Ϊ�Ƹ˶���*/
                    if(Push_curr_temp[i] < PUSH_PROTECT_MIN_CUR)
                    {
                        MotorControl[i].Motor_Start_Stop = 0;
                        Push_Error_Cheek_Cnt[i] = 0;
                        Push_curr_temp[i] = 0;
                        /*���ߴ�����Բ��ô����־λ�����Լ���ʹ��*/
//											  MotorControl[i].Fault_Flag = 1;
                        if(i == 0)
                        {
                            /*ָʾ�����˴������ж�PUSH_BRAKE_LINE����PUSH_BRAKE_LINE���ڽ����ж�PUSH0_BRAKE_LINE�������ĸ������������*/
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
                    /*����100ms������һ�η����ܱ������߱���*/
                    Push_curr_temp[i] = 0;
                }
                /*���ϲ������ж���û�ж��ߣ��������ж��Ƹ˵�������Ƿ�����*/
                if(NeedToMissingHallPush[i])
                {
                    /*���200ms���Ƹ˵Ļ���ֵû�иı䣬�Ǿ�˵���Ƹ˻����쳣*/
                    if((Push_Error_Cheek_Cnt[i]%2000==0)&&(MotorControl[i].Hall.HALL_CaptureValueDelta == \
                                                           MotorControl[i].Hall.HALL_CaptureValue)&&Push_curr_temp[i] >= PUSH_PROTECT_MIN_CUR)
                    {
                        Push_Error_Cheek_Cnt[i] = 0;
                        Push_curr_temp[i] = 0;
                        MotorControl[i].Motor_Start_Stop = 0;   //��������
                        MotorControl[i].Fault_Flag = 1; //֮����Ҫ���½����Ƹ˱궨��������Ҫ�ֶ����
                        MotorControl[i].Push_motor_calibrationFLAG = 3; //��������֮����Ҫ���±궨�����ܿ���  Ĭ��Ϊ0��1Ϊ��ʼ�궨 2 Ϊ�궨�ɹ���3Ϊ�궨ʧ��
                        if(i == 0)
                        {
                            MC_SetFault1(PUSH0_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//����Ϊ��ָʾ��
                        }
                        if(i == 1)
                        {
                            MC_SetFault1(PUSH1_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//����Ϊ��ָʾ��
                        }
                        if(i == 2)
                        {
                            MC_SetFault1(PUSH2_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//����Ϊ��ָʾ��
                        }
                        if(i == 9)
                        {
                            MC_SetFault1(PUSH9_LOST_HALL);
                            MC_SetFault(PUSH_LOST_HALL);//����Ϊ��ָʾ��
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
        else if(NeedToCheekPush[i]==2)  //���3���4���߼��
        {
            if(MotorControl[i].Motor_Start_Stop == 1 && l_abs(MotorControl[i].PWM_Duty) > MOTOR34_PROTECT_MIN_VAL) //ռ�ձȴ���2999�ű���
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
                        if(Push_curr_temp[0] < PUSH_PROTECT_MIN_CUR)  //���߼���ж�
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
                        if(Push_curr_temp_1[0] < PUSH_PROTECT_MIN_CUR)  //���߼���ж�
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
                    MotorControl[0].Motor_Start_Stop = DISABLE;   //���ߴ���
                    MotorControl[0].Fault_Flag = 1;  //���ߴ���
                    MC_SetFault1(PUSH0_BRAKE_LINE);
                    MC_SetFault(PUSH_BRAKE_LINE);//����Ϊ��ָʾ��
                }
            }
        }
		
		        else if(NeedToCheekPush[i]==4)
        {
            if(MotorControl[1].Motor_Start_Stop == 1)
            {
				if(MotorControl[1].Direction==STOPSTOP)
				{
				   Push_Error_Cheek_Cnt2[1]=0;
					Push_Error_Cheek_Cnt_1[1]=0;
					CurChkArray1[0]=0;
					CurChkArray1[1]=0;
				}
                if(MotorControl[1].PWM_Duty>PUSH_PROTECT_MIN_VAL)
                {
					brake_flag_cnt1=0;
                    Push_Error_Cheek_Cnt2[1]++;
                    Push_curr_temp[1] += MotorControl[1].Current.FilterValue;
                    Push_Error_Cheek_Cnt_1[1] =0;
                    Push_curr_temp_1[1] = 0;		
                    if(Push_Error_Cheek_Cnt2[1]%20000==0)
                    {
						brake_flag_cnt++;
						if(brake_flag_cnt==1)
						{
							test_valuex[0]=Push_curr_temp[1];
							if(Push_curr_temp[1] < PUSH_PROTECT_MIN_CUR)  //���߼���ж�
							{
								test_valuex[7]=Push_curr_temp[1];
								CurChkArray1[0] =1;
							}
							else
							{
								CurChkArray1[0] =0;
								CurChkArray1[1] =0;
							}
						}
						Push_Error_Cheek_Cnt2[1]=0;
						Push_curr_temp[1] =0;
						
                    }
                }
                else if(MotorControl[1].PWM_Duty<-PUSH_PROTECT_MIN_VAL)
                {
					brake_flag_cnt=0;
                    Push_Error_Cheek_Cnt_1[1]++;
                    Push_curr_temp_1[1] += MotorControl[1].Current.FilterValue;
                    Push_Error_Cheek_Cnt2[1] =0;
                    Push_curr_temp[1] = 0;				
                    if(Push_Error_Cheek_Cnt_1[1]%20000==0)
                    {
						brake_flag_cnt1++;
						if(brake_flag_cnt1==1)
						{
							test_valuex[1]=Push_curr_temp_1[1];
							if(Push_curr_temp_1[1] < PUSH_PROTECT_MIN_CUR)  //���߼���ж�
							{
								test_valuex[6]=Push_curr_temp_1[1];
								CurChkArray1[1] =1;
							}
							else
							{
								CurChkArray1[1] =0;
								CurChkArray1[0] =0;
							}
					   }
                        Push_Error_Cheek_Cnt_1[1]=0;
                        Push_curr_temp_1[1] =0;
                    }
                }			
                if(CurChkArray1[0]==1&&CurChkArray1[1]==1)
                {
                    MotorControl[1].Motor_Start_Stop = DISABLE;   //���ߴ���
                    MotorControl[1].Fault_Flag = 1;  //���ߴ���
                    MC_SetFault1(PUSH1_BRAKE_LINE);
                    MC_SetFault(PUSH_BRAKE_LINE);//����Ϊ��ָʾ��
                }
            }
        }

    }
}
/*------------------------------------------------
Function:�Ƹ˵���궨
Input   :No
Output  :No
Explain :��Ҫ��λ�����ͱ궨��־λ����Ҫλ�ÿ��Ƶ��Ƹ���Ҫ�ϵ�궨������ʹ��
------------------------------------------------*/
uint32_t Sys_Ticktemp[10] = {0};
extern u8 First_PowerOn;
void Push_Motor_Calibrate(u8 motornum)
{
    u8 caputernumsetfinsh = 0;
    if(MotorControl[motornum].Push_Location_model ==1)      //ÿ�α궨��ϻὫ�ñ�־λ��1���궨������Push_Location_modelΪ0
    {
        MotorControl[motornum].Fault_Flag =0;//�궨ǰ�Ѵ��������ڱ궨�ж�ת����ȫ�����ض��ж�Ϊ�궨�ɹ�
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PWM_DutySet = 0;
        MotorControl[motornum].Push_Location_model = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 10000;  //���ڱ궨���ڲ�ͬ��λ�ñ궨��ʼ֮����Ҫ�Լ�
    }
    if(MotorControl[motornum].Fault_Flag == 0)
    {
        MotorControl[motornum].PWM_DutySet=-PERCENT_95_OF_PWM_PERIOD;//����ɾ�������ܽ������ж�
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
    Sys_Ticktemp[motornum]++; //��ʱ
    if(Sys_Ticktemp[motornum]%501 == 0) // 501��ֵһ�� 1000���һ��  1002�θ�ֵһ�� 1503��ֵһ��  2000���һ�� 2004��ֵһ�� 2505��ֵһ�� 3000���һ�� 3006��ֵһ�� 3057��ֵһ�� 4000���һ��
    {
        MotorControl[motornum].Hall.HALL_PreCaptureValue = MotorControl[motornum].Hall.HALL_CaptureValue ;
    }
    else if(Sys_Ticktemp[motornum]%1000 == 0) //1000��ֵ���һ��
    {
        caputernumsetfinsh = 1;//�ֲ���������֤����ֵ����֮����е�λ�ж�
    }
    else
    {
        caputernumsetfinsh = 0;
    }

    if((caputernumsetfinsh == 1 )&&
            MotorControl[motornum].Hall.HALL_PreCaptureValue == MotorControl[motornum].Hall.HALL_CaptureValue
//            MotorControl[motornum].Fault_Flag ==0
      ) //���ʱ�䵽������ֵû�иı䣬���û�й�����
    {
        MotorControl[motornum].Fault_Flag = 0; //�궨������������������
        MotorControl[motornum].Motor_Start_Stop = 0;
        MotorControl[motornum].Location_Set = 0;
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PWM_DutySet = 0; //�����������Ϊû��λ�û������԰���PWM���Ʒ�ʽ�����ƣ��������������
        MotorControl[motornum].Hall.HALL_PreCaptureValue = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 0;
        MotorControl[motornum].Push_motor_calibrationFLAG = 2;//�궨��ȷ�˳��궨
        MotorControl[motornum].Push_Location_model=1; //�����Ƹ�λ�û����ò�����������λ�ÿ��ƣ���Ҫ��ֵĬ��ֵ��������Щ����Ҫλ�ÿ��Ƶ��Ƹ˵��
        Sys_Ticktemp[motornum] = 0;//��������
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
    else if(Sys_Ticktemp[motornum] > 7000) //14�� �궨ʧ��
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
        MotorControl[motornum].Push_motor_calibrationFLAG = 3; //�ر��Ƹ�1λ�û��������˳����α궨
        Sys_Ticktemp[motornum] = 0;
        MC_SetFault(PUSHMOTOR_INT_ERROR);
        MotorState = START;
    }
}

/*------------------------------------------------
Function:��������
Input   :No
Output  :No
Explain :Ϊ��ȱ���⵱�еĵ���������������֮���������
------------------------------------------------*/
void swap(float *x,float *y)
{
    uint32_t temp= 0;
    temp=*x;
    *x=*y;
    *y=temp;
}
void BubbleSort(float a[], float n)        // ���㷨��a[]�е�Ԫ�ش�С�������������
{
    uint32_t j= 0;
    for(uint32_t i = 0; i < n - 1; i++)
    {
        for(j = n - 1; j > i; j--)
        {
            if(a[j - 1]>a[j])
            {
                swap(&a[j - 1],&a[j]);      // Ϊ������������a[j] �� a[j - 1] ���н���
            }
            else
            {}
        }
    }
}
/*------------------------------------------------
Function:���5ȱ����
Input   :No
Output  :No
Explain :���е�0.9����ֵ��Ҫ���ݲ�ͬ�ĵ�����в���
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
        if(Motor5_Current_temp.cnt>30000)   //6.10�޸�ȱ�������2����3s
        {
            Motor5_Current_temp.cnt = 0;
            BubbleSort(M5_100us_ac_ab_bc_cur,ac_ab_bc_index);
			if(M5_100us_ac_ab_bc_cur[2]>100)
			{
					 testM5_100us_ac_ab_bc_cur =(M5_100us_ac_ab_bc_cur[2]-M5_100us_ac_ab_bc_cur[0])/M5_100us_ac_ab_bc_cur[2];//���Թ۲�ֵ
				
				
				if(testM5_100us_ac_ab_bc_cur>0.93) //��С��������֮�����ֵ-��Сֵ��0.6Ϊ���Թ���ֵ
				{
					MotorControl[5].Max_Phase_Lack=testM5_100us_ac_ab_bc_cur*1000;  //��¼������ʱ���ȱ�������ֵ
					Miss_Phase_Count++;
					if(Miss_Phase_Count>2)
					{
						MC_SetFault(MOTOR5_MISSING_PHASE); //����Ϊ��ָʾ��
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
		if(MotorControl[5].Direction==1||MotorControl[5].Direction==-1)             //���뷽���ж�
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
Function:���6ȱ����
Input   :No
Output  :No
Explain :���е�0.9����ֵ��Ҫ���ݲ�ͬ�ĵ�����в���
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
        if(Motor6_Current_temp.cnt>30000)    //6.15�޸�
        {
            Motor6_Current_temp.cnt = 0;
            BubbleSort(M6_100us_ac_ab_bc_cur,ac_ab_bc_index);
			if(M6_100us_ac_ab_bc_cur[2]>100)
			{
				testM6_100us_ac_ab_bc_cur =	(M6_100us_ac_ab_bc_cur[2]-M6_100us_ac_ab_bc_cur[0])/M6_100us_ac_ab_bc_cur[2];//���Թ۲�ֵ
				if(testM6_100us_ac_ab_bc_cur>0.9) //��С��������֮�����ֵ-��Сֵ��0.5Ϊ���Թ���ֵ
				{
					Miss6_Phase_Count++;
					if(Miss6_Phase_Count>2)
					{
						MotorControl[6].Max_Phase_Lack=testM6_100us_ac_ab_bc_cur*1000;  //��¼������ʱ���ȱ�������ֵ
						MC_SetFault(MOTOR6_MISSING_PHASE); //����Ϊ��ָʾ��
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
		if(MotorControl[6].Direction==1||MotorControl[6].Direction==-1)        //���뷽���ж�
		{
			if(MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[4]||\
					MotorControl[6].Hall.HallState == HALL_Study[1].HallTab[1])  //BC  CB ���������
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
    if(OverFlow_Cnt[0]>=BREAK_CNT_MAX)	/* ��ˢ5 */
    {
        MC_SetFault(MOTOR5_BREAK);
        MotorControl[5].Fault_Flag = 1;
        MotorControl[5].Motor_Start_Stop = DISABLE;
    }
    if(OverFlow_Cnt[1]>=BREAK_CNT_MAX)	/* ��ˢ6 */
    {
        MC_SetFault(MOTOR6_BREAK);
        MotorControl[6].Fault_Flag = 1;
        MotorControl[6].Motor_Start_Stop = DISABLE;
    }
    if(OverFlow_Cnt[2]>=BREAK_CNT_MAX)	/* ������ˢ3����4 */
    {
        MC_SetFault(BRAKE_3_4);
        MotorControl[3].Motor_Start_Stop = DISABLE;
        MotorControl[3].Fault_Flag = 1;
        MotorControl[4].Motor_Start_Stop = DISABLE;
        MotorControl[4].Fault_Flag = 1;
    }
    if(OverFlow_Cnt[3]>=BREAK_CNT_MAX)	/* �Ƹ�0_1_2_9 */
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
    if(OverFlow_Cnt[4]>=BREAK_CNT_MAX)	/* ��ˢ10_11_12_13 */
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
Function:ƫ�õ�ѹУ׼
Input   :No
Output  :No
Explain :�����������������Ϊ�˶�Ӧ��������ͨ����Ӧ�������
------------------------------------------------*/
u8 Voltage_offset_calibration[8] = {0,1,2,3,4,5,6,9};//���¶�������
u8 ADC_ConvertedValueindex[8] =    {6,7,8,0,1,2,4,9};//���¶�������
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
        if(offset_cnt == OFFSET_NUM)		/* ȡ100�εľ�ֵ */
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
*Description   		: ���������
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
            if(motor_err_cnt5>60)		//�ۼ�60�α���
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
*Description   		: ���������
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
            if(motor_err_cnt6>60)//20�α���
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
*Description   		:�Ƹ˵���������ˢ���ʵ�ָ߶�����Ӧ
* Input           : bldc_num �� ��ˢ�����
										push_num �� �Ƹ˵����
										hall_max �� �����·����ֵ
										hall_min �� ������С����ֵ
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
float cur_ratio[7] = {0,0,0,0,0,MOTOR5CURCOEFFICIENT,MOTOR6CURCOEFFICIENT};
int16_t limitpwm = 4000;
uint8_t hight_err_cnt = 0;
int CurSelfAdapt_Value=20;  //ƫ��ֵ0.2A
int CurSelfAdapt_Time=2000; //�˲�ʱ��1s
int CurSelfAdapt_Convert_Time;
void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min)             //15msһ��
{
    CurSelfAdapt_Convert_Time=CurSelfAdapt_Time/15;
    if(MotorControl[bldc_num].Current.SetValue!=0)
    {
        if(MotorControl[bldc_num].Speed_Real>200) //:2022.4.28 readme����
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
                if(hight_err_cnt>CurSelfAdapt_Convert_Time)          //200��Ӧ3s
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
*Description   		:��ˢ���������  2Sû�ٶȱ���
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
void Motor7_Err_Chk(void)		/* 200msѭ���� */
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
*Description   		:BLDC��ת����  500ms�޶�������
* Input           : None
* Output          : None
* Return          : None		2021.12.16	by diamond
*************************/
int16_t Bldc_lack_flag=0;
u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
void BLDC_Stuck_Chk(void)			/* 1msѭ���� */
{
//    static u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
    if(GetMotorSpeed(Motor5)==0&&ABS(MotorControl[5].PWM_Duty)>4000&&MotorControl[5].Motor_Start_Stop==1)	//BLDC1����ʱȱ�࣬����Ϊ0
    {
        if(MotorControl[5].Current.DeepFilterVAL < 10) //���������ж�Ϊ����ʱȱ�࣬��Ϊ��һ���ǶϿ���û�е���
        {
            M5phase_check_cnt++;//����ȱ�����
        }
        else if(MotorControl[5].Current.FilterValue > 700)
        {
            M5stuck_check_cnt++;//��ת����
        }

        if(M5phase_check_cnt>PHASE_ERR_MAX)
        {
			MotorControl[5].Start_Phase_lack_cnt=M5phase_check_cnt;
            Bldc_lack_flag=1;
            MC_SetFault(MOTOR5_MISSING_PHASE);//M5ȱ��
            M5phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[5].Fault_Flag =1;
            MotorControl[5].Motor_Start_Stop = 0;

        }
        if(M5stuck_check_cnt>SUTCK_ERR_MAX)
        {
			MotorControl[5].Stuck_time=M5stuck_check_cnt;
            M5stuck_check_cnt = SUTCK_ERR_MAX;
            MC_SetFault(MOTOR5_PHASE_ERROR); //M5�������
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

    if(GetMotorSpeed(Motor6)==0&&ABS(MotorControl[6].PWM_Duty)>4000&&MotorControl[6].Motor_Start_Stop==1)	//BLDC1����ʱȱ�࣬����Ϊ0
    {
        if(MotorControl[6].Current.DeepFilterVAL < 10) //���������ж�Ϊ����ʱȱ�࣬��Ϊ��һ���ǶϿ���û�е���
        {
            M6phase_check_cnt++;//����ȱ�����
        }
        else if(MotorControl[6].Current.FilterValue > 700)
        {
            M6stuck_check_cnt++;//��ת����
        }

        if(M6phase_check_cnt>PHASE_ERR_MAX)
        {
			MotorControl[6].Start_Phase_lack_cnt=M6phase_check_cnt;
            MC_SetFault(MOTOR6_MISSING_PHASE);//M6ȱ��
            M6phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[6].Fault_Flag =1;
            MotorControl[6].Motor_Start_Stop = 0;

        }
        if(M6stuck_check_cnt>SUTCK_ERR_MAX)
        {
			MotorControl[6].Stuck_time=M6stuck_check_cnt;
            MC_SetFault(MOTOR6_PHASE_ERROR); //M6�������
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
*Description   		:BLDC1ʧ�ٱ���  4s�����ٶ��쳣����
* Input           : None
* Output          : None
* Return          : None		2021.12.17	by diamond
*************************/
void BLDC1_OverSpdChk(void)
{
   if(Motor_Type<2)
  {
    static u16 speed_err_cnt = 0;
    if((MotorControl[5].Speed_Ref>0&&MotorControl[5].Speed_Real<0)||(MotorControl[5].Speed_Ref<0&&MotorControl[5].Speed_Real>0))		/* ����ת */
        speed_err_cnt++;
    else if(MotorControl[5].Speed_Ref==0&&ABS(MotorControl[5].Speed_Set)<500)
        speed_err_cnt=0;
//    else if(ABS(MotorControl[5].Speed_Real)-ABS(MotorControl[5].Speed_Ref)>500)																								/* ʵ��>�趨 ��ľ���ֵ->����ֵ�Ĳ�޸����ص��½�������*/
//        speed_err_cnt++;
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
    if((MotorControl[6].Speed_Ref>0&&MotorControl[6].Speed_Real<0)||(MotorControl[6].Speed_Ref<0&&MotorControl[6].Speed_Real>0))		/* ����ת */
        speed_err_cnt++;
    else if(MotorControl[6].Speed_Ref==0&&ABS(MotorControl[6].Speed_Set)<500)
        speed_err_cnt=0;
//    else if(ABS(MotorControl[6].Speed_Real)-ABS(MotorControl[6].Speed_Ref)>500)																										/* ʵ��>�趨 */
//        speed_err_cnt++;
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
*Description   		:�Ƹ˶���ʱ������
* Input           : u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3
										push 1-4:	0�ر� 1����	timer:0-256 �������ֹͣ
* Output          : None
* Return          : None		2022.1.27	by diamond
*************************/
u8 timer_cnt[4]=0;
u8 OverRunFlag=0;
void Push_OverRunChk(u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3)  //֮ǰ������Ϊ��DisplayErrLed()���ȫ�ִ���1����������û����
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
//                MC_SetFault1(PUSH0_OVERTIME);
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
				OverRunFlag=1;
				MotorControl[1].Inplace=1;
                MotorControl[1].Motor_Start_Stop=DISABLE;
//                MC_SetFault(PUSH_OVER_TIME);
//                MC_SetFault1(PUSH1_OVERTIME);
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
//                MC_SetFault1(PUSH2_OVERTIME);
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
//                MC_SetFault1(PUSH9_OVERTIME);
                timer_cnt[3]=0;
            }
        }
        else timer_cnt[3]=0;
    }
}



void Bldc5_Hall_Check(void)   //hall������
{
	 if(MotorControl[5].Hall.HallState == 0||MotorControl[5].Hall.HallState == 7 )
		{
			if((Motor_Type<2)&&(MotorControl[5].Speed_PWN_Select!=0))
			{
				MotorControl[5].Hall_Error_Syscnt++;
				if(MotorControl[5].Hall_Error_Syscnt>3)   //3��hall�������ϱ���������
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
void Bldc6_Hall_Check(void)   //hall������
{
	if(MotorControl[6].Hall.HallState == 0||MotorControl[6].Hall.HallState == 7 )
	{
		if((Motor_Type<2)&&(MotorControl[6].Speed_PWN_Select!=0))
		{
			MotorControl[6].Hall_Error_Syscnt++;
			if(MotorControl[6].Hall_Error_Syscnt>3)   //3��hall�������ϱ���������
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
   
	
u16 Push_Motor1_Inplacecnt=0;	
u8 Push_Motor_Location_cnt1=0;
int16_t Location_First_Direction=0;
int16_t Location_Second_Direction=0;
void Push_Motor_Inplace(void)   //����10ms
{
	Push_Motor_Location_cnt1++;
	if(Push_Motor_Location_cnt1==1)
	{
		Location_First_Direction=MotorControl[1].Hall.HALL_CaptureValue;
	}
	else if(Push_Motor_Location_cnt1==2)
	{
		Push_Motor_Location_cnt1=0;
		Location_Second_Direction=MotorControl[1].Hall.HALL_CaptureValue;
	}
	if(Location_First_Direction!=Location_Second_Direction)
	{
		Push_Motor1_Inplacecnt=0;
		if(MotorControl[1].Hall.HALL_CaptureValue>10)	
		{
			if(f_abs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set)<=5)
			{
				 MotorControl[1].Inplace=1;
			}
			else if(f_abs(MotorControl[1].Hall.HALL_CaptureValue- MotorControl[1].Location_Set)>5)
			{
				MotorControl[1].Inplace=0;
			}
		}
		else if(MotorControl[1].Hall.HALL_CaptureValue<=10)
		{
			MotorControl[1].Inplace=1;
		}
    }
	else if(Location_First_Direction==Location_Second_Direction)
	{
		Push_Motor1_Inplacecnt++;
		if(Push_Motor1_Inplacecnt>300)   //3s
		{
			if(MotorControl[1].Current.FilterValue<10)
			{
				Push_Motor1_Inplacecnt=0;
			    MotorControl[1].Inplace=1;
			}
		}				
	}
	if(OverRunFlag==1)
	{
		MotorControl[1].Inplace=1;
	}
	if(MotorControl[1].Direction==CW)  //����
	{
		MotorControl[1].Fang_xiang=1;
	}
	else if(MotorControl[1].Direction==CCW) //����
	{
		MotorControl[1].Fang_xiang=2;
	}
	else if(MotorControl[1].Direction==2) //ֹͣ
	{
		MotorControl[1].Fang_xiang=3;    
	}
}
