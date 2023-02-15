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
Function:MCU�汾��
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver)
{
	pVersion->uVersionPartOfRobotType = robotype;				//��Ŀ����	S��
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
Function:�ϵ��������µ����
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
Function:������ʵ��ֵת�� ת����1000-1A
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
Function:������������
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void CurrentLimit(u8 Motor_NUM)
{
    if(MotorControl[Motor_NUM].Current.FilterValue > MotorControl[Motor_NUM].Current.MaxValue1)		/* 1�׶ζ�����ˢ��ʱû�ã��ᱻPID���ڻ��� */
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
Function:���ô����־
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_SetFault(u32 hFault_type)
{
    wGlobal_Flags |= hFault_type;
	/*��һ����flash��ص������Ƶ�main������*/

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
    sendPDOevent(&CANopen_Drive);
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
		if(((wGlobal_Flags&(PUSH_MOTOR1_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL))==0)&&MotorControl[0].Fault_Flag == 1)//�Ƹ�0�������
		{
				MotorControl[0].Fault_Flag = 0;
		}
		if(((wGlobal_Flags&(PUSH_MOTOR2_OVER_CUR|PUSHMOTOR_INT_ERROR|PUSH_BRAKE_LINE|PUSH_LOST_HALL))==0)&&MotorControl[1].Fault_Flag == 1)//�Ƹ�1�������
		{
				MotorControl[1].Fault_Flag = 0;
		}
		if(((wGlobal_Flags&(ONEWYA_MOTOR1_OVER_CUR))==0)&&MotorControl[3].Fault_Flag == 1)//���3�������
		{
				MotorControl[3].Fault_Flag = 0;
		}
		if(((wGlobal_Flags&(ONEWYA_MOTOR2_OVER_CUR))==0)&&MotorControl[4].Fault_Flag == 1)//���4�������
		{
				MotorControl[4].Fault_Flag = 0;
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

}
/*----------------------------
*Fuction������ֵ����
*Explain�����ͣ�float
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

//��ת�ṹ��
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
		TIM4->CCR3= PWM_PERIOD+PWM_PeriodOFFSET;
		TIM4->CCR1 = PWM_PERIOD+PWM_PeriodOFFSET ;
		TIM4->CCR2 = PWM_PERIOD+PWM_PeriodOFFSET ;
		TIM11->CCR1 = 2*PWM_PERIOD+PWM_PeriodOFFSET ;
		TIM10->CCR1 = 2*PWM_PERIOD+PWM_PeriodOFFSET ;
/************************�Ƹ˳�ʼ��***************************/
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
    MotorControl[1].Current.MaxValue1 = MOTOR1_CurrentValue(1.8) ;
    MotorControl[1].Current.MaxValue2 = MOTOR1_CurrentValue(2) ;
    MotorControl[1].Current.MaxValue3 = MOTOR1_CurrentValue(2.5) ;
    MotorControl[1].Current.MaxValue4 = MOTOR1_CurrentValue(3)	;
    MotorControl[1].Current.OFCnt1_T = 20000;
    MotorControl[1].Current.OFCnt2_T = 2000;
    MotorControl[1].Current.OFCnt3_T = 800;
    MotorControl[1].Current.OFCnt4_T = 5;

    MotorControl[2].Current.MaxValue1 = MOTOR2_CurrentValue(3) ;
    MotorControl[2].Current.MaxValue2 = MOTOR2_CurrentValue(4) ;
    MotorControl[2].Current.MaxValue3 = MOTOR2_CurrentValue(5) ;
    MotorControl[2].Current.MaxValue4 = MOTOR2_CurrentValue(6) ;
    MotorControl[2].Current.OFCnt1_T = 200;
    MotorControl[2].Current.OFCnt2_T = 20;
    MotorControl[2].Current.OFCnt3_T = 8;
    MotorControl[2].Current.OFCnt4_T = 5;

/*********************�Ƹ˳�ʼ��end***************************/

/*************************������ˢ3_4***************************/
		MotorControl[3].Frequency = 10000/(TIM10->PSC+1);
    MotorControl[3].Current.MaxValue1 = MOTOR3_CurrentValue(2.5) ;  //���˵��
    MotorControl[3].Current.MaxValue2 = MOTOR3_CurrentValue(3) ;
    MotorControl[3].Current.MaxValue3 = MOTOR3_CurrentValue(3.5) ;  //���˵��
    MotorControl[3].Current.MaxValue4 = MOTOR3_CurrentValue(4) ;
    MotorControl[3].Current.OFCnt1_T = 20000;
    MotorControl[3].Current.OFCnt2_T = 2000;
    MotorControl[3].Current.OFCnt3_T = 800;
    MotorControl[3].Current.OFCnt4_T = 200;

		MotorControl[4].Frequency = 10000/(TIM11->PSC+1);
    MotorControl[4].Current.MaxValue1 = MOTOR4_CurrentValue(3) ;  //��ˮ���
    MotorControl[4].Current.MaxValue2 = MOTOR4_CurrentValue(3) ;
    MotorControl[4].Current.MaxValue3 = MOTOR4_CurrentValue(3.5) ;  //��ˮ���
    MotorControl[4].Current.MaxValue4 = MOTOR4_CurrentValue(4) ;
    MotorControl[4].Current.OFCnt1_T = 20000;
    MotorControl[4].Current.OFCnt2_T = 2000;
    MotorControl[4].Current.OFCnt3_T = 800;
    MotorControl[4].Current.OFCnt4_T = 200;

/*************************������ˢ3_4 end***************************/

/*************************��ˢ5��6***************************/

		MotorControl[5].Frequency = 10000/(TIM1->PSC+1);
    MotorControl[5].Current.MaxValue1 = MOTOR5_CurrentValue(2) ;
    MotorControl[5].Current.MaxValue2 = MOTOR5_CurrentValue(8) ;
    MotorControl[5].Current.MaxValue3 = MOTOR5_CurrentValue(9) ;
    MotorControl[5].Current.MaxValue4 = MOTOR5_CurrentValue(10.5) ;
		MotorControl[5].Current.CurOffset = 30;		//Ĭ��0.3A
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
		MotorControl[6].Current.CurOffset = 30;		//Ĭ��0.3A
    MotorControl[6].Current.OFCnt1_T = 30000;
    MotorControl[6].Current.OFCnt2_T = 30000;
    MotorControl[6].Current.OFCnt3_T = 10000;
    MotorControl[6].Current.OFCnt4_T = 200;
    MotorControl[6].Pole_Paires = 4;
    MotorControl[6].Acceleration = 5;
    MotorControl[6].Deceleration = 5;
		
		MotorControl[7].Frequency = 10000/(TIM8->PSC+1);
		MotorControl[7].Current.MaxValue1 = MOTOR7_CurrentValue(3) ;  //��ˢ���
    MotorControl[7].Current.MaxValue2 = MOTOR7_CurrentValue(3.5) ;
    MotorControl[7].Current.MaxValue3 = MOTOR7_CurrentValue(4) ;  //��ˢ���
    MotorControl[7].Current.MaxValue4 = MOTOR7_CurrentValue(4.5) ;
    MotorControl[7].Current.OFCnt1_T = 20000;
    MotorControl[7].Current.OFCnt2_T = 20000;
    MotorControl[7].Current.OFCnt3_T = 10000;
    MotorControl[7].Current.OFCnt4_T = 2000;
		
		MotorControl[8].Frequency = 10000/(TIM4->PSC+1);
		MotorControl[8].Current.MaxValue1 = MOTOR8_CurrentValue(13) ;  //������
    MotorControl[8].Current.MaxValue2 = MOTOR8_CurrentValue(20) ;
    MotorControl[8].Current.MaxValue3 = MOTOR8_CurrentValue(20) ;  //������
    MotorControl[8].Current.MaxValue4 = MOTOR8_CurrentValue(20) ;
    MotorControl[8].Current.OFCnt1_T = 20000;
    MotorControl[8].Current.OFCnt2_T = 10;
    MotorControl[8].Current.OFCnt3_T = 20000;
    MotorControl[8].Current.OFCnt4_T = 20000;
	TIM4->CCR4= 0;
/*************************��ˢ5��6end***************************/
		PID_Init (&PID_Speed_InitStruct[0],&PID_Current_InitStructure[0]);		//���5��PID
		PID_Init (&PID_Speed_InitStruct[1],&PID_Current_InitStructure[1]);		//���6��PID
		PID_PWM_Init(&PID_PWM);																								//�Ƹ�0��PID
/*	�Ӽ��ٶȳ�ʼ��	*/
    MotorControl[0].Acceleration = 10;
    MotorControl[0].Deceleration = 10;
    MotorControl[0].LastMotorDirection = 2;
    MotorControl[0].Hall.HALL_CaptureValue = 10000; //�����ϵ�궨
    MotorControl[1].Acceleration = 10;
    MotorControl[1].Deceleration = 10;
    MotorControl[1].LastMotorDirection = 2;
    MotorControl[1].Hall.HALL_CaptureValue = 10000; //�����ϵ�궨
    MotorControl[2].Acceleration = 10;
    MotorControl[2].Deceleration = 10;
    MotorControl[2].LastMotorDirection = 2;
    MotorControl[2].Hall.HALL_CaptureValue = 10000; //�����ϵ�궨
		MotorControl[0].Push_Location_model=0;
    MotorControl[1].Push_Location_model=1; //���Ƹ˵����Ҫ�궨���궨�ɹ��ſ���λ�ÿ��ƣ�����������Ҫ�궨Ĭ�Ͽ���λ�ÿ��ƣ�ֻ������������Ŀ���û��λ�ÿ���
		MotorControl[0].Hall.HALL_CaptureValue = 10000; //�����ϵ�궨    
		MotorControl[1].Hall.HALL_CaptureValue = 0; //�����Ƹ�1��Ҫλ�ÿ��ƣ����궨�����Push_Location_model���أ�������ֱ�Ӹ����궨�����ʱ��ǰֵΪ0
 
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

/*           ����ѧϰ����                */
    HALL_Study[1].StudySectorCnt3 = 200;/*����һ��ʱ��1����2ms*/
    HALL_Study[1].HallSector = 1;
    HALL_Study[1].StudySectorCnt = 0;
    HALL_Study[1].HallCommPWM = 4500;/*��������PWMֵ*/

    HALL_Study[0].StudySectorCnt3 = 200;/*����һ��ʱ��1����2ms*/
    HALL_Study[0].HallSector = 1;
    HALL_Study[0].StudySectorCnt = 0;
    HALL_Study[0].HallCommPWM = 4500;/*��������PWMֵ*/

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
		RED_LED_OFF;
		else if(led_mode)	//����
		{
			if(DisplayCounter<2*led_speed)
				RED_LED_ON;
			else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
				RED_LED_TOGGLE;
		}
		else if(DisplayCounter<=(2*led_time)*led_speed)	//����
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
Function:����������ָʾ
Input   :No
Output  :No
Explain :10ms����һ��
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
				case OVER_VOLTAGE:  //����
				case UNDER_VOLTAGE:   
						RED_LED_ON;
            DisplayCounter=0;
            break;
        case PUSH_MOTOR1_OVER_CUR: //�Ƹ�0��˸1��
				case PUSH_BRAKE_LINE:
				case PUSH_LOST_HALL:
				case PUSHMOTOR_INT_ERROR:
						LEDSet(LED_ON,LED_OFF,1,0,LED_HIGH_SPEED);
            break;
				
				case PUSH_OVER_TIME:
        case PUSH_MOTOR2_OVER_CUR: //�Ƹ�1��˸2��
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case ONEWYA_MOTOR1_OVER_CUR: //���3��˸3��
//				case MOTOR3_BRAKE_LINE:
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case ONEWYA_MOTOR2_OVER_CUR: //���4��˸4��
//				case MOTOR4_BRAKE_LINE:
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;
				
        case BLDC1_OVER_CUR: //BLDC1  5��
				case MOTOR5_PHASE_ERROR:
				case MOTOR5_BREAK:
				case MOTOR5_MISSING_PHASE:
				case MOTOR5_OVER_SPEED:
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;
				
//        case BLDC2_OVER_CUR: //BLDC2  6��
//				case MOTOR6_BREAK:
//				case MOTOR6_PHASE_ERROR:
//				case MOTOR6_MISSING_PHASE:
//				case MOTOR6_OVER_SPEED:
//            LEDSet(LED_ON,LED_OFF,6,0,LED_HIGH_SPEED);
//            break;
				
				case SIDE_BRUSH_ERROR:	//��ˢ7 ��
						LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
					break;
				case FAN_ERROR:					//��� 8��
						LEDSet(LED_ON,LED_OFF,8,0,LED_HIGH_SPEED);
					break;

				
				case CAN_COMMUNICATION_ERR:	//����1��
            LEDSet(LED_ON,LED_OFF,1,1,LED_LOW_SPEED);
            break;
        case HALL5_SENSOR_ERR:    //����2��
						LEDSet(LED_ON,LED_OFF,2,1,LED_LOW_SPEED);		//����

            break;

        case HALL6_SENSOR_ERR:		// ����3��
						LEDSet(LED_ON,LED_OFF,3,1,LED_LOW_SPEED);
            break;

        default:
						RED_LED_ON;
            break;
        }
    }

}
/*------------------------------------------------
Function:�Ƹ˵��λ�û�
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Push_Motor_Location_Control(u8 motornum)
{
    if(MotorControl[motornum].Push_Location_model==1)//�궨��ȷ����ʼִ��λ�ÿ���
    {
        if(motornum == 0) //50������ˢ�Ƹˣ�����Ԥ����ˢ����Ӧ
        {
            if(MotorControl[5].Current.SetValue ==0)	//û������������Ӧ�����������λ�ÿ���
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

Function:4·�Ƹ˵���������Ͷ������

Input   :No

Output  :No

Explain :No

------------------------------------------------*/
u8 NeedToCheekPush[MOTOR_NUM] = {0,3,0,0,0,0,0,0,0}; //������Ƹ˵����Ҫ������������Ƹ˶������ 0 1 2 9�ŵ��Ĭ�϶��򿪼��
u8 NeedToMissingHallPush[MOTOR_NUM] = {1,0,0,0,0,0,0,0}; //��������ȷ���Ƹ��Ƿ���Ҫ��������
uint16_t PushMaxPulse[MOTOR_NUM] = {260,4000,4000,0,0,0,0,0}; //���������Ҫȷ�������Ƹ˵����ֵ
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
        if(NeedToCheekPush[i]==1) //������Ƹ˵����Ҫ������������Ƹ˶������ 0 1 2 9�ŵ��Ĭ
        {
            /*ռ�ձȴ���5999�ű���*/
             if(MotorControl[i].Motor_Start_Stop == 1 && (l_abs(MotorControl[i].Location_Set-MotorControl[i].Hall.HALL_CaptureValue))>20 && \
                    l_abs(MotorControl[i].PWM_Duty) > PUSH_PROTECT_MIN_VAL && MotorControl[i].Hall.HALL_CaptureValue < PushMaxPulse[i])
            {
							/*����ֵ�͵���ֵ�ۼ�Ӧ�÷��ڵ������ʱ*/
                Push_Error_Cheek_Cnt[i]++;
                Push_curr_temp[i] += MotorControl[i].Current.FilterValue;
								if(Push_Error_Cheek_Cnt[i]%2002000==0)
                {
                    Push_Error_Cheek_Cnt[i]++;
                }
                if(Push_Error_Cheek_Cnt[i]%1001==0) //100.1ms������ֵ����һ�Σ�������һ�ζ��߼��
                {
                    MotorControl[i].Hall.HALL_CaptureValueDelta = MotorControl[i].Hall.HALL_CaptureValue;
                    if(Push_curr_temp[i] < PUSH_PROTECT_MIN_CUR)  //���߼���ж�
                    {
												Push_Error_Cheek_Cnt[i] = 0;
                        Push_curr_temp[i] = 0;
                        MotorControl[i].Motor_Start_Stop = DISABLE;   //���ߴ���
//											  MotorControl[i].Fault_Flag = 1;  //���ߴ�����Բ��ô����־λ�����Լ���ʹ��
                        if(i == 0)
                        {
                            MC_SetFault1(PUSH0_BRAKE_LINE);//������һ���������鿴 �ڶ���32λȫ�ִ��� wGlobal_Flags1
                            MC_SetFault(PUSH_BRAKE_LINE);//����Ϊ��ָʾ��
                        }
                        if(i == 1)
                        {
                            MC_SetFault1(PUSH1_BRAKE_LINE);
                            MC_SetFault(PUSH_BRAKE_LINE);//����Ϊ��ָʾ��
                        }
                    }
										Push_curr_temp[i] = 0;
                }
                if(NeedToMissingHallPush[i]) //��Ҫ�����������Ƹˣ�Ĭ�϶���Ҫ
                {
                    if((Push_Error_Cheek_Cnt[i]%2000==0)&&(MotorControl[i].Hall.HALL_CaptureValueDelta == \
											MotorControl[i].Hall.HALL_CaptureValue)&&Push_curr_temp[i] >= PUSH_PROTECT_MIN_CUR) //���200ms���Ƹ˵Ļ���ֵû�иı䣬�Ǿ�˵���Ƹ˻����쳣
                    {
												Push_Error_Cheek_Cnt[i] = 0;
                        Push_curr_temp[i] = 0;
                        MotorControl[i].Motor_Start_Stop = DISABLE;   //��������
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
								if(Push_curr_temp[i] < PUSH_PROTECT_MIN_CUR)  //���߼���ж�
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
								if(Push_curr_temp_1[i] < PUSH_PROTECT_MIN_CUR)  //���߼���ж�
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
							MotorControl[i].Motor_Start_Stop = DISABLE;   //���ߴ���
							MotorControl[i].Fault_Flag = 1;  //���ߴ���
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
Explain :No
------------------------------------------------*/
uint32_t Sys_Ticktemp[10] = {0};
extern u8 First_PowerOn;
int push_test_1=1000;
void Push_Motor_Calibrate(u8 motornum)
{
    u8 caputernumsetfinsh = 0;
    if(MotorControl[motornum].Push_Location_model ==1)      //ÿ�α궨��ϻὫ�ñ�־λ��1���궨������Push_Location_modelΪ0
    {
        MotorControl[motornum].Fault_Flag =0;//�궨ǰ�Ѵ��������ڱ궨�ж�ת����ȫ�����ض��ж�Ϊ�궨�ɹ�
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PercentOfPWM = 0;
        MotorControl[motornum].Push_Location_model = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 10000;  //���ڱ궨���ڲ�ͬ��λ�ñ궨��ʼ֮����Ҫ�Լ�
    }
    if(MotorControl[motornum].Fault_Flag == 0)
    {
        MotorControl[motornum].PercentOfPWM=-MAX_PERCENT;//����ɾ�������ܽ������ж�
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
        MotorControl[motornum].Location_Set = -100;     //929�޸�
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PercentOfPWM = 0; //�����������Ϊû��λ�û������԰���PWM���Ʒ�ʽ�����ƣ��������������
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
        MotorControl[motornum].Motor_Start_Stop = 0;
        MotorControl[motornum].PWM_Duty = 0;
        MotorControl[motornum].PercentOfPWM = 0;
        MotorControl[motornum].Hall.HALL_PreCaptureValue = 0;
        MotorControl[motornum].Hall.HALL_CaptureValue = 0;
				MotorControl[motornum].Push_Location_model = 1;
        MotorControl[motornum].Push_motor_calibrationFLAG = 3; //�ر��Ƹ�1λ�û��������˳����α궨
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
            MotorControl[5].Speed_Ref<-1000  )  //1010���޸�
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
					Miss_Phase_Count++;
					if(Miss_Phase_Count>1)
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
Function:���6ȱ����
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
            testM6_100us_ac_ab_bc_cur =  (M6_100us_ac_ab_bc_cur[2]-M6_100us_ac_ab_bc_cur[0])/M6_100us_ac_ab_bc_cur[2];//���Թ۲�ֵ
            if(testM6_100us_ac_ab_bc_cur>0.5) //��С��������֮�����ֵ-��Сֵ��0.5Ϊ���Թ���ֵ
            {
//                if(MotorControl[6].Current.DeepFilterVAL>50)
                {   
                    MC_SetFault(MOTOR6_MISSING_PHASE); //����Ϊ��ָʾ��
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
}
 /*------------------------------------------------
Function:ƫ�õ�ѹУ׼
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u8 Voltage_offset_calibration[8] = {0,1,3,4,5,6,7,8};//���¶�������
u8 ADC_ConvertedValueindex[8] =    {1,2,9,11,6,13,3,4};//���¶�������
#define OFFSET_NUM 100
u8 offset_cnt = 0;
void Voltage_offset_cali(void)		
{
    offset_cnt++;
    for(int i = 0; i < 7; i++)
    {
        MotorControl[Voltage_offset_calibration[i]].Current.offset += ADC_ConvertedValue[ADC_ConvertedValueindex[i]];
    }
    if(offset_cnt == OFFSET_NUM)		/* ȡ100�εľ�ֵ */
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
			if(motor_err_cnt5>60)		//�ۼ�10�α���
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
			if(motor_err_cnt6>10)//10�α���
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
*Description   		:�Ƹ˵���������ˢ���ʵ�ָ߶�����Ӧ  
* Input           : u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min 
										��ˢ����ţ���ˢ����ţ�����Ӧ���߶ȣ�����Ӧ��С�߶�
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
			if(MotorControl[push_num].Motor_Start_Stop ==DISABLE)MotorControl[push_num].Motor_Start_Stop = ENABLE;
			
			if(f_abs(MotorControl[bldc_num].Current.DeepFilterVAL-MotorControl[bldc_num].Current.SetValue*cur_ratio[bldc_num])<MotorControl[5].Current.CurOffset*cur_ratio[bldc_num])
			{
				MotorControl[push_num].PercentOfPWM = 0;
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
		if(MotorControl[bldc_num].Speed_Real<-100)         //ת���������
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
*Function Name     :BLDC_Stuck_Chk
*Description       :BLDC��ת����  500ms�޶�������
* Input           : None
* Output          : None
* Return          : None    2021.12.16  by diamond
*************************/
u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
void BLDC_Stuck_Chk(void)      /* 1msѭ���� */
{
//    static u16 M5phase_check_cnt=0,M6phase_check_cnt=0,M5stuck_check_cnt=0,M6stuck_check_cnt=0;
    if(GetMotorSpeed(Motor5)==0&&MotorControl[5].PWM_Duty!=0)  //BLDC1����ʱȱ�࣬����Ϊ0
    {
        if(MotorControl[5].Current.DeepFilterVAL < 20) //���������ж�Ϊ����ʱȱ�࣬��Ϊ��һ���ǶϿ���û�е���
        {
            M5phase_check_cnt++;//����ȱ�����
        }
        else
        {
            M5stuck_check_cnt++;//��ת����
        }

        if(M5phase_check_cnt>PHASE_ERR_MAX)
        {
            MC_SetFault(MOTOR5_MISSING_PHASE);//M5ȱ��
            M5phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[5].Fault_Flag =1;
            MotorControl[5].Motor_Start_Stop = 0;

        }
        if(M5stuck_check_cnt>SUTCK_ERR_MAX)
        {
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

    if(GetMotorSpeed(Motor6)==0&&MotorControl[6].PWM_Duty!=0)  //BLDC1����ʱȱ�࣬����Ϊ0
    {
        if(MotorControl[6].Current.DeepFilterVAL < 20) //���������ж�Ϊ����ʱȱ�࣬��Ϊ��һ���ǶϿ���û�е���
        {
            M6phase_check_cnt++;//����ȱ�����
        }
        else
        {
            M6stuck_check_cnt++;//��ת����
        }

        if(M6phase_check_cnt>PHASE_ERR_MAX)
        {
            MC_SetFault(MOTOR6_MISSING_PHASE);//M6ȱ��
            M6phase_check_cnt=PHASE_ERR_MAX;
            MotorControl[6].Fault_Flag =1;
            MotorControl[6].Motor_Start_Stop = 0;

        }
        if(M6stuck_check_cnt>SUTCK_ERR_MAX)
        {
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
	static u16 speed_err_cnt = 0;
	if((MotorControl[5].Speed_Ref>0&&MotorControl[5].Speed_Real<0)||(MotorControl[5].Speed_Ref<0&&MotorControl[5].Speed_Real>0))		/* ����ת */
		speed_err_cnt++;
	else if(MotorControl[5].Speed_Ref==0&&ABS(MotorControl[5].Speed_Set)<500)
		speed_err_cnt=0;			
	else if(ABS(MotorControl[5].Speed_Real)-ABS(MotorControl[5].Speed_Ref)>500&&MotorControl[5].Speed_Real<-3000)																								/* ʵ��>�趨 */
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
	if((MotorControl[6].Speed_Ref>0&&MotorControl[6].Speed_Real<0)||(MotorControl[6].Speed_Ref<0&&MotorControl[6].Speed_Real>0))		/* ����ת */
		speed_err_cnt++;			
	else if(MotorControl[6].Speed_Ref==0&&ABS(MotorControl[6].Speed_Set)<500)
		speed_err_cnt=0;		
	else if(ABS(MotorControl[6].Speed_Real)-ABS(MotorControl[6].Speed_Ref)>500)																										/* ʵ��>�趨 */
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
*Description   		:�Ƹ˶���ʱ������
* Input           : u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3
										push 1-4:	0�ر� 1����	timer:0-256 �������ֹͣ
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
	if(Motor_Type==0)                  //��ˢ���ѡ��0�������ˢ�������ǰ��12    1����̩��ˢ���  ��ǰ��6
	{
		ad_angel=12;
		MotorControl[5].Current.MaxValue1 = MOTOR6_CurrentValue(2) ;
		MotorControl[5].Current.MaxValue2 = MOTOR6_CurrentValue(5) ;
		MotorControl[5].Current.MaxValue3 = MOTOR6_CurrentValue(7) ;
		MotorControl[5].Current.MaxValue4 = MOTOR6_CurrentValue(8.5) ;
		BLDC_SPEED_COEEFFOCIENT=30;
		PID_Current_InitStructure[0].hLower_Limit_Output=-4000;    //��ʱ��1Ƶ�ʸĳ���20khz��PWM������4200
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
		PID_Current_InitStructure[0].hLower_Limit_Output=-4000;    //��ʱ��1Ƶ�ʸĳ���20khz��PWM������4200
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
		 Sys_Ticktemp[5]++; //��ʱ
		if(Sys_Ticktemp[5]%100 == 0) // 1s���һ��
		{
			MotorControl[0].Hall.HALL_PreCaptureValue = MotorControl[0].Hall.HALL_CaptureValue ;
		}
		if(Sys_Ticktemp[5]%800 == 0) // 8s���һ��
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
				MotorControl[0].Adjust_Flag=2;//У׼���
				MotorControl[0].Location_Set=-600;
			}
			else
			{
				Push_Type=0;
				MotorControl[0].Adjust_Flag=2;  //У׼���
				MotorControl[0].Location_Set=-600;
			}
		}	
	}
	if((MotorControl[0].Adjust_Flag==2)&&(Adjust_Flag1==1))
	{
		if((Push_Type==0&&MotorControl[0].Hall.HALL_CaptureValue_Max>150)||(Push_Type==1&&MotorControl[0].Hall.HALL_CaptureValue_Max>300))
		{
			Select_Flag=1;    //У׼�ɹ�
		}
		else
		{
			Select_Flag=2;    //У׼ʧ��
		}
			transbuf[80]=MotorControl[0].Adjust_Flag; // У׼���                          
			transbuf[81]=Push_Type; // �Ƹ�����                                          
			transbuf[82]=Select_Flag; // У׼�ɹ�                                  
			transbuf[83]=MotorControl[0].Hall.HALL_CaptureValue_Max; //���hall��¼ 
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


void Bldc_Hall_Check(void)   //hall������
{
	 if(MotorControl[5].Hall.HallState == 0||MotorControl[5].Hall.HallState == 7 )
		{
				MotorControl[5].Hall_Error_Syscnt++;
				if(MotorControl[5].Hall_Error_Syscnt>3)   //2.5s��hall�������ϱ���������
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