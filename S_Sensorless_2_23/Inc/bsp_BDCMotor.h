#ifndef __BDCMOTOR_TIM_H__
#define __BDCMOTOR_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "MC_PID_regulators.h"
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

/*motor 8 define begin*/
#define	VOLTAGE_ERROR					20
#define	M8_OVER_CURRENT				40
#define	OVER_SPEED_M8					60
#define	OVER_TMP_M8						80
#define	M8_STUCK							100
#define	PWM_OFFSET						3

/*Motor8 define end*/

/*timer define  build9*/
//#define BtoCA   0x0c9c
//#define CBtoA   0x099c
//#define CtoAB   0x09cc
//#define ACtoB   0x09c9
//#define AtoBC   0x0cc9  
//#define BAtoC   0x0c99  
//                        
//#define BtoC    0x0c98  
//#define B_complementary_to_C        0x0cd8
//#define BtoA    0x089c
//#define B_complementary_to_A        0x08dc
//#define AtoC    0x0c89  
//#define A_complementary_to_C        0x0c8d
//#define AtoB    0x08c9  
//#define A_complementary_to_B        0x08cd
//#define CtoA    0x098c  
//#define C_complementary_to_A        0x0d8c
//#define CtoB    0x09c8  
//#define C_complementary_to_B        0x0dc8
//                        
//#define OPEN_ALL_DOWN_TUBE   0x0ddd
//#define CLOSE_ALL_TUBE       0x0888



//build8
#define BtoCA   0x0414
#define CBtoA   0x0114
#define CtoAB   0x0144
#define ACtoB   0x0141
#define AtoBC   0x0441  
#define BAtoC   0x0411  
                        
#define BtoC    0x1410
#define B_complementary_to_C        0x1450
#define BtoA    0x1014
#define B_complementary_to_A        0x1054
#define AtoC    0x1401  
#define A_complementary_to_C        0x1405
#define AtoB    0x1041  
#define A_complementary_to_B        0x1045
#define CtoA    0x1104  
#define C_complementary_to_A        0x1504
#define CtoB    0x1140  
#define C_complementary_to_B        0x1540
                        
#define OPEN_ALL_DOWN_TUBE   0x1555
#define CLOSE_ALL_TUBE       0x1000

/*timer define end */

#define PWM_ADDCOUNT					6
#define POLE_PAIRES  2
#define POLE_PAIRES2  2
#define PWM_PERIOD 4200   //8400
#define	PWM_PeriodOFFSET	1
#define PERCENT_95_OF_PWM_PERIOD 4000   //4000
#define	PHASE_ANGLE		60	//3�������źţ�ÿ��60��
#define	MAX_PERCENT	95
#define	PWM_PECENT	100
#define	PWM_COEFFICIENT 84
//#define BLDC_SPEED_COEEFFOCIENT	30

#define NEGATIVE          (s8)-1
#define POSITIVE          (s8)1
#define NEGATIVE_SWAP     (s8)-2
#define POSITIVE_SWAP     (s8)2
#define ERROR1             (s8)3
#define MISSTEP           (s8)-3

#define MOTOR_NUM 9
#define CCW  1  //��ʱ��
#define CW  0   //˳ʱ��
#define STOPSTOP 2 //ֹͣ
#define MOTOR_COMMUTATION_DOWNTIME   1000   //�������ͣ��ʱ�䣬��ֵ��λΪ/500us    8000*500 = 4 000 000 us  = 4S

#define MOTOR0CCW()   { TIM2->CCR3 = 0;\
												TIM2->CCR4 =  -MotorControl[0].PWM_Duty;\
										    }

#define MOTOR0CW()    { TIM2->CCR4 = 0;\
												TIM2->CCR3 =  MotorControl[0].PWM_Duty;\
												}

#define MOTOR0STOP()	{	TIM2->CCR3 = 0;\
												TIM2->CCR4 = 0;\
												MotorControl[0].PWM_Duty = 0;\
												}

#define MOTOR1CW()		{	TIM9->CCR1 = MotorControl[1].PWM_Duty;\
												TIM9->CCR2 =  0;\
												}

#define MOTOR1CCW()		{	TIM9->CCR2 = -MotorControl[1].PWM_Duty;\
												TIM9->CCR1 = 0;\
												}

#define MOTOR1STOP()	{	TIM9->CCR1 = 0;\
												TIM9->CCR2 = 0;\
												MotorControl[1].PWM_Duty = 0;}


typedef struct
{
    int16_t GetADCValue;//��ȡ��ADCֵ
    int16_t FilterValue;//�˲����ADCֵ
		uint16_t Cur_Real;	//������ʵ��adc 1000-1A
		int16_t GetADC_Temp;
		int16_t FilterTemp;
		float PreFilterVal;	//�ϸ����ڵ��˲�ADCֵ
		float	DeepFilterVAL;	//����˲���adc
		int16_t ADCValue_Ref;
		uint16_t SetValue;		//��λ���趨�ĵ���ֵ
		u8 CurOffset;					//����ĵ���ƫ��
    uint16_t MaxValue1;   //���ֵ��һ�׶�
    uint16_t MaxValue2;   //���ֵ�ڶ��׶�
    uint16_t MaxValue3;   //���ֵ�����׶�
    uint16_t MaxValue4;   //���ֵ�ڽ׶�
    uint16_t MinValue;   //��Сֵ
    u16 OFCnt1;
    u16 OFCnt2;
    u16 OFCnt3;
    u16 OFCnt4;
    u16 OFCnt1_T;
    u16 OFCnt2_T;
    u16 OFCnt3_T;
    u16 OFCnt4_T;
    float ConversionFactor; //ת��ϵ��
    float   ActualUnitValue;  //ת��Ϊʵ�ʵ�λֵ
		int offset;
		int temp_offset;
		uint16_t SetValue_Min;		//��λ���趨��ˢ����Ӧ�����ж�
	int16_t    PhaseUCurrent;
	int16_t    PhaseVCurrent;
	int16_t    PhaseWCurrent;
	int16_t    PhaseUAD;
	int16_t    PhaseVAD;
	int16_t    PhaseWAD;

} ADC_ValueParameters_t;

typedef struct
{
    uint8_t HallState;               //��ǰHall״̬
    uint8_t PrevHallState;           //��һ��Hall״̬
    uint8_t HallState_Temp;          //����Hall״̬��ϣ������жϷ���
    uint8_t HallState_CCW;
    uint8_t HallState_CW;
    uint16_t HALL_CaptureValue;      //Hall����ֵ
    uint16_t HALL_PreCaptureValue;   //��һ��Hall����ֵ
    u32 PreHALL_OVF_Counter;         //��һ�μ��������ֵ
    s32 HALL_CaptureValueDelta;
		u16 chgperiod;
	  u8 Hall_Changecnt;		//���������PWM�Ĵ���
		u16 Advance_Angle;			//��ǰ��
		u8 HALL_CaptureValue_Direction_Judgment;

} HALL_Parameters_t;

typedef struct
{
    uint8_t  StudySectorCnt;
    uint16_t StudySectorCnt2;
    uint16_t StudySectorCnt3;
    uint8_t HallTab[6];
    uint8_t HallSector;
    int16_t HallCommPWM;
    uint8_t CommuntionState;
}
HALL_Study_t;

extern HALL_Study_t HALL_Study[2];
typedef struct
{
	int8_t  Inplace;
	int8_t  BrulessMode;
    int16_t Speed_Set;
    int16_t	Location_Set;
    int16_t Speed_Ref;
    int32_t Speed_Real;
    int16_t Acceleration;
    int16_t Deceleration;
    int16_t SpeedLimit;

		int8_t PercentOfPWM;		//PWMռ�ձ�
//		int8_t PercentOfSpeed;	
    int16_t PWM_Duty;
		
		uint16_t Frequency;			//Ƶ��
    uint16_t Pole_Paires;
    uint8_t Fault_Flag;
		uint8_t Fault_Cnt;

    int8_t  Direction;
    int8_t  LastMotorDirection;
		uint8_t	Push_Location_model;
		uint8_t Push_motor_calibrationFLAG;
    FunctionalState  Motor_Start_Stop;

    ADC_ValueParameters_t	Current;
    HALL_Parameters_t   Hall;

} MotorControlParameters_t;



typedef struct
{
    int16_t GetADCValue;//��ȡ��ADCֵ
    int16_t FilterValue;//�˲����ADCֵ
		int16_t ADCValue_Ref;
	
    uint16_t MaxValue1;   //���ֵ��һ�׶�
    uint16_t MaxValue2;   //���ֵ�ڶ��׶�
	  uint16_t MaxValue3;   //���ֵ��һ�׶�
    uint16_t MaxValue4;   //���ֵ�ڶ��׶�
    uint16_t MinValue;   //��Сֵ
		float PreFilterVal;	//�ϸ����ڵ��˲�ADCֵ
	float	DeepFilterVAL;	//����˲���adc
	
	  int16_t    PhaseACurrent;
	  int16_t    PhaseBCurrent;
	  int16_t    PhaseCCurrent;
	  uint16_t    PhaseAOffset;
	  uint16_t    PhaseBOffset;
	  uint16_t    PhaseCOffset;

    u16 OFCnt1;
    u16 OFCnt2;
	  u16 OFCnt3;
    u16 OFCnt4;
		   
		u16 OFCnt1_T;
    u16 OFCnt2_T;
	  u16 OFCnt3_T;
    u16 OFCnt4_T;
    float ConversionFactor; //ת��ϵ��
    float   ActualUnitValue;  //ת��Ϊʵ�ʵ�λֵ
    int offset;
	int16_t I1;
	int16_t I2;
	int16_t I3;
	int16_t I4;
	int16_t I5;
	int16_t I6;
	int16_t Iab;
	int16_t Iac;
	int16_t Ibc;
	int16_t Iba;
	int16_t Ica;
	int16_t Icb;
} ADC_Value_t;

typedef struct
{
	uint16_t IdleCountCnt1;
	uint16_t  CountSectorCnt;
	uint16_t CountSectorCnt2;
	uint16_t CountSectorCnt3;
	uint8_t  SenlessHallSector;
	int16_t  SenlessCommPWM;
	int16_t  PWM_DutySet;
	int16_t  PWM_Duty;
	int16_t    Speed_Real;
	int16_t    Speed_Set;
	int16_t    Speed_Ref;
	int16_t    PhaseUCurrent;
	int16_t    PhaseVCurrent;
	int16_t    PhaseWCurrent;
	uint8_t    State;
	int16_t    PWMTicks;
	int16_t    PWMTicksPre;
	int8_t    FLAGBEMF;
	uint8_t    Step;
	int16_t FlagSwitchStep;
	 int16_t Acceleration;
	int16_t Deceleration;
	u32 period;
	int16_t    PhaseUAD;
	int16_t    PhaseVAD;
	int16_t    PhaseWAD;
	int16_t    FirstFlag;
	uint8_t HallState;               //��ǰHall״̬
	HALL_Parameters_t   Hall;
	ADC_Value_t	Current;
	uint8_t DeepFilterFlag;    
    u32 Speed_Cnt; 
	u32 Speed_Total_Cnt; 
	int16_t Turn_Into_Closed_Cnt;
	int16_t Turn_Into_Flag;
    	
}
Sensorless_t;

extern Sensorless_t Sensorless[2];
extern MotorControlParameters_t MotorControl[MOTOR_NUM];


extern uint8_t PrevHallState1,PrevHallState2,HallState1,HallState2;
extern uint16_t  HALL_CaptureValue,HALL_PreCaptureValue,Speed_Hz;
extern s16 BLDC1_Speed_RPM ;

extern uint16_t  HALL_CaptureValue2,HALL_PreCaptureValue2;
extern s16 BLDC2_Speed_RPM ;


extern s32 HALL_CaptureValueDelta,HALL_CaptureValueDelta2;
extern u32 HALL_OVF_Counter ;

extern int8_t  Motor_Type;
extern int8_t  Push_Type;
//extern PID_Struct_t   PID_Speed_InitStruct[2];




typedef enum
{
    Motor0,
    Motor1,
    Motor2,
    Motor3,
    Motor4,
    Motor5,
    Motor6,
    Motor7,
    Motor8,
    Motor9
} Motor_NUM_t;


///* �������� ------------------------------------------------------------------*/

//void BDCMOTOR_TIMx_Init(void);
//void SetMotorDir(int16_t Dir);
void SetMotorSpeed(uint8_t Motor_NUM,int16_t PWM_Duty);
void SetMotorStop(uint8_t Motor_NUM);
void  OneFilter(u16 NewValue,u16 OldValue,u8 FilterFactor );
u8 HALL_GetPhase1(void);
u8 HALL_GetPhase2(void);
void BLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty);
void BLDC2_PhaseChange(u8 bHallState, s16 PWM_Duty);
void HALLSTUDY_PhaseChange0(u8 bHallState);
void HALLSTUDY_PhaseChange1(u8 bHallState);
s16 Ramp_Speed(u8 Mortor_NUM);
s16 Ramp_PPWM(u8 Mortor_NUM);

void HallStudyHandle0(void);
void HallStudyHandle1(void);
int16_t PWM_DutyControl(u8 addpwm_cnt,int16_t PWM_Dutytemp);
void Suction_motor_errhandle(u8 Duty_ratio);
void BLDC_Hall_Handle(u8 motor_num);


#endif	/* __BDCMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
