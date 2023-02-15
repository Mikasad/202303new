#ifndef __BDCMOTOR_TIM_H__
#define __BDCMOTOR_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
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
                        
#define BtoC    0x0410
#define B_complementary_to_C        0x0450
#define BtoA    0x0014
#define B_complementary_to_A        0x0054
#define AtoC    0x0401  
#define A_complementary_to_C        0x0405
#define AtoB    0x0041  
#define A_complementary_to_B        0x0045
#define CtoA    0x0104  
#define C_complementary_to_A        0x0504
#define CtoB    0x0140  
#define C_complementary_to_B        0x0540
                        
#define OPEN_ALL_DOWN_TUBE   0x1555
#define CLOSE_ALL_TUBE       0x1000

/*timer define end */

#define PWM_ADDCOUNT					6
#define POLE_PAIRES  2
#define POLE_PAIRES2  2
#define PWM_PERIOD 8400
#define	PWM_PeriodOFFSET	1
#define PERCENT_95_OF_PWM_PERIOD 8000
#define	PHASE_ANGLE		60	//3个霍尔信号，每个60°
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
#define CCW  1  //逆时针
#define CW  0   //顺时针
#define STOPSTOP 2 //停止
#define MOTOR_COMMUTATION_DOWNTIME   1000   //电机换向停机时间，该值单位为/500us    8000*500 = 4 000 000 us  = 4S

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
    int16_t GetADCValue;//获取的ADC值
    int16_t FilterValue;//滤波后的ADC值
		uint16_t Cur_Real;	//反馈的实际adc 1000-1A
		int16_t GetADC_Temp;
		int16_t FilterTemp;
		float PreFilterVal;	//上个周期的滤波ADC值
		float	DeepFilterVAL;	//深度滤波的adc
		int16_t ADCValue_Ref;
		uint16_t SetValue;		//上位机设定的电流值
		u8 CurOffset;					//允许的电流偏差
    uint16_t MaxValue1;   //最大值第一阶段
    uint16_t MaxValue2;   //最大值第二阶段
    uint16_t MaxValue3;   //最大值第三阶段
    uint16_t MaxValue4;   //最大值第阶段
    uint16_t MinValue;   //最小值
    u16 OFCnt1;
    u16 OFCnt2;
    u16 OFCnt3;
    u16 OFCnt4;
    u16 OFCnt1_T;
    u16 OFCnt2_T;
    u16 OFCnt3_T;
    u16 OFCnt4_T;
    float ConversionFactor; //转化系数
    float   ActualUnitValue;  //转换为实际单位值
		int offset;
		int temp_offset;
		uint16_t SetValue_Min;		//到位的设定滚刷自适应电流判断

} ADC_ValueParameters_t;

typedef struct
{
    uint8_t HallState;               //当前Hall状态
    uint8_t PrevHallState;           //上一次Hall状态
    uint8_t HallState_Temp;          //两次Hall状态组合，用来判断方向
    uint8_t HallState_CCW;
    uint8_t HallState_CW;
    uint16_t HALL_CaptureValue;      //Hall捕获值
    uint16_t HALL_PreCaptureValue;   //上一次Hall捕获值
    u32 PreHALL_OVF_Counter;         //上一次计数器溢出值
    s32 HALL_CaptureValueDelta;
		u16 chgperiod;
	  u8 Hall_Changecnt;		//换向后增大PWM的次数
		u16 Advance_Angle;			//超前角
		u8 HALL_CaptureValue_Direction_Judgment;
	 uint16_t HALL_CaptureValue_Max;      //Hall捕获值

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
    int16_t Speed_Set;
    int16_t	Location_Set;
    int16_t Speed_Ref;
    int32_t Speed_Real;
    int16_t Acceleration;
    int16_t Deceleration;
    int16_t SpeedLimit;

		int8_t PercentOfPWM;		//PWM占空比
//		int8_t PercentOfSpeed;	
    int16_t PWM_Duty;
		
		uint16_t Frequency;			//频率
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
	uint8_t Adjust_Flag;
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


///* 函数声明 ------------------------------------------------------------------*/

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
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
