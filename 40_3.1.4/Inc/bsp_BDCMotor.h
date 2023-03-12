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

#define POLE_PAIRES  2
#define POLE_PAIRES2  2
#define PWM_PERIOD 8400
#define	PWM_PeriodOFFSET	1
#define PERCENT_95_OF_PWM_PERIOD 7500
#define PWM_Complementary_Switch_M2 4500
#define PWM_Complementary_Switch_M1 4500

#define NEGATIVE          (s8)-1
#define POSITIVE          (s8)1
#define NEGATIVE_SWAP     (s8)-2
#define POSITIVE_SWAP     (s8)2
#define ERROR1             (s8)3
#define MISSTEP           (s8)-3

#define MOTOR_NUM 14
#define CCW  1  //��ʱ��
#define CW  0   //˳ʱ��
#define STOPSTOP 2 //ֹͣ
#define MOTOR_COMMUTATION_DOWNTIME   1000   //�������ͣ��ʱ�䣬��ֵ��λΪ/500us    8000*500 = 4 000 000 us  = 4S

#define MOTOR0CCW()   { TIM2->CCR1 = 0;\
										    HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_RESET);\
												TIM2->CCR2 =  -MotorControl[0].PWM_Duty;\
										    }

#define MOTOR0CW()    { TIM2->CCR2 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_RESET);\
												TIM2->CCR1 = MotorControl[0].PWM_Duty;\
												}

#define MOTOR0STOP()	{	TIM2->CCR1 = 0;\
												TIM2->CCR2 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_SET);\
												MotorControl[0].PWM_Duty = 0;\
												}

#define MOTOR1CCW()		{	TIM2->CCR3 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_RESET);\
												TIM2->CCR4 =  -MotorControl[1].PWM_Duty;\
												}

#define MOTOR1CW()		{	TIM2->CCR4 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_RESET);\
												TIM2->CCR3 = MotorControl[1].PWM_Duty;\
												}

#define MOTOR1STOP()	{	TIM2->CCR3 = 0;\
												TIM2->CCR4 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);\
												MotorControl[1].PWM_Duty = 0;}

#define MOTOR2CCW()		{ TIM4->CCR1 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_RESET);\
												TIM4->CCR2 =  -MotorControl[2].PWM_Duty;\
												}

#define MOTOR2CW()		{ TIM4->CCR2 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_RESET);\
												TIM4->CCR1 = MotorControl[2].PWM_Duty;\
												}

#define MOTOR2STOP()  { TIM4->CCR1 = 0;\
												TIM4->CCR2 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_SET);\
												MotorControl[2].PWM_Duty = 0;}

#define MOTOR9CCW()		{	TIM4->CCR3 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_BL4_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_AL4_Pin,GPIO_PIN_RESET);\
												TIM4->CCR4 =  -MotorControl[9].PWM_Duty;\
												}

#define MOTOR9CW()		{	TIM4->CCR3 = MotorControl[9].PWM_Duty;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL4_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL4_Pin,GPIO_PIN_RESET);\
												TIM4->CCR4 = 0;\
												}

#define MOTOR9STOP()	{	TIM4->CCR3 = 0;\
												TIM4->CCR4 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL4_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL4_Pin,GPIO_PIN_SET);\
												MotorControl[9].PWM_Duty = 0;}
#define BtoCA   0x3c9c
#define CBtoA   0x399c
#define CtoAB   0x39cc
#define ACtoB   0x39c9
#define AtoBC   0x3cc9	
#define BAtoC   0x3c99	
												
#define BtoC    0x3c98	
#define B_complementary_to_C        0x3cd8
#define BtoA    0x389c
#define B_complementary_to_A        0x38dc
#define AtoC    0x3c89	
#define A_complementary_to_C        0x3c8d
#define AtoB    0x38c9	
#define A_complementary_to_B        0x38cd
#define CtoA    0x398c	
#define C_complementary_to_A        0x3d8c
#define CtoB    0x39c8	
#define C_complementary_to_B        0x3dc8
												
#define OPEN_ALL_DOWN_TUBE   0x3ddd
#define CLOSE_ALL_TUBE       0x3888
typedef struct
{
	int16_t GetADCValue;//��ȡ��ADCֵ
	int16_t Pressure_ADCValue;
    int16_t FilterValue;//�˲����ADCֵ
	float PreFilterVal;	//�ϸ����ڵ��˲�ADCֵ
	float	DeepFilterVAL;	//����˲���adc
	int16_t ADCValue_Ref;
	uint16_t SetValue;		//��λ���趨�ĵ���ֵ
	u8 CurOffset;
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

} ADC_ValueParameters_t;

typedef struct
{
    uint8_t ChangeFlag;     //��һ��������־
    uint8_t HallState;               //��ǰHall״̬
    uint8_t PrevHallState;           //��һ��Hall״̬
    uint8_t HallState_Temp;          //����Hall״̬��ϣ������жϷ���
    uint8_t HallState_CCW;
    uint8_t HallState_CW;
    uint16_t HALL_CaptureValue;      //Hall����ֵ
    uint16_t HALL_PreCaptureValue;   //��һ��Hall����ֵ
    u32 PreHALL_OVF_Counter;         //��һ�μ��������ֵ
    s32 HALL_CaptureValueDelta;
     uint8_t  HALL_CaptureValue_Direction_Judgment;
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
    int16_t Speed_Set;
    int16_t	Location_Set;
    int16_t Speed_Ref;
    int32_t Speed_Real;
    int16_t Acceleration;
    int16_t Deceleration;
    int16_t SpeedLimit;

    int16_t PWM_DutySet;
    int16_t PWM_Duty;
    uint16_t Pole_Paires;
    uint8_t Fault_Flag;
		uint8_t Fault_Cnt;
		int16_t Speed_PWN_Select;


    int8_t  Direction;
    int8_t  LastMotorDirection;
    FunctionalState  Motor_Start_Stop;

    ADC_ValueParameters_t	Current;
    HALL_Parameters_t   Hall;
	uint8_t Push_Location_model;
    uint8_t Push_motor_calibrationFLAG;
	
	int  Max_Phase_Lack;
	int  Start_Phase_lack_cnt;
    int  Stuck_time;
	int  Hall_Error_Cnt;
	int  Hall_Error_Syscnt;
} MotorControlParameters_t;





extern MotorControlParameters_t MotorControl[MOTOR_NUM];


extern uint8_t PrevHallState1,PrevHallState2,HallState1,HallState2;
extern uint16_t  HALL_CaptureValue,HALL_PreCaptureValue,Speed_Hz;
extern s16 BLDC1_Speed_RPM ;

extern uint16_t  HALL_CaptureValue2,HALL_PreCaptureValue2;
extern s16 BLDC2_Speed_RPM ;


extern s32 HALL_CaptureValueDelta,HALL_CaptureValueDelta2;
extern u32 HALL_OVF_Counter  ;

extern int8_t  Motor_Type;



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
u8 HALL_GetPhase2(void);
void BLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty);
void BLDC2_PhaseChange(u8 bHallState, s16 PWM_Duty);
void HALLSTUDY_PhaseChange0(u8 bHallState);
void HALLSTUDY_PhaseChange1(u8 bHallState);
s16 Ramp_Speed(u8 Mortor_NUM);
s16 Ramp_PPWM(u8 Mortor_NUM);

void HallStudyHandle0(void);
void HallStudyHandle1(void);
void Suction_motor_errhandle(u8 Duty_ratio);


#endif	/* __BDCMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/