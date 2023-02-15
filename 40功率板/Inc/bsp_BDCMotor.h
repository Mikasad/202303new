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
#define PWM_Complementary_Switch_M2 4500
#define PWM_Complementary_Switch_M1 4500
#define POLE_PAIRES  2
#define POLE_PAIRES2  2
#define PWM_PERIOD 8400
#define	PWM_PeriodOFFSET	1
#define PERCENT_95_OF_PWM_PERIOD   6000
//#define PERCENT_95_OF_PWM_PERIOD   8400
#define NEGATIVE          (s8)-1
#define POSITIVE          (s8)1
#define NEGATIVE_SWAP     (s8)-2
#define POSITIVE_SWAP     (s8)2
#define ERROR1             (s8)3
#define MISSTEP           (s8)-3

#define MOTOR_NUM 14
#define CCW  1  //逆时针
#define CW  0   //顺时针
#define STOPSTOP 2 //停止
#define MOTOR_COMMUTATION_DOWNTIME   1000   //电机换向停机时间，该值单位为/500us    8000*500 = 4 000 000 us  = 4S

#define MOTOR0CW()   { TIM2->CCR1 = 0;\
										    HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_RESET);\
												TIM2->CCR2 =  MotorControl[0].PWM_Duty;\
										    }

#define MOTOR0CCW()    { TIM2->CCR2 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_RESET);\
												TIM2->CCR1 = -MotorControl[0].PWM_Duty;\
												}

#define MOTOR0STOP()	{	TIM2->CCR1 = 0;\
												TIM2->CCR2 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_SET);\
												MotorControl[0].PWM_Duty = 0;\
												}

#define MOTOR2CW()		{	TIM2->CCR3 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_RESET);\
												TIM2->CCR4 =  MotorControl[2].PWM_Duty;\
												}

#define MOTOR2CCW()		{	TIM2->CCR4 = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_RESET);\
												TIM2->CCR3 = -MotorControl[2].PWM_Duty;\
												}

#define MOTOR2STOP()	{	TIM2->CCR3 = 0;\
												TIM2->CCR4 = 0;\
	                      MotorControl[2].PWM_Duty = 0;\
												HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_RESET);\
												HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_RESET);\
												MotorControl[2].PWM_Duty = 0;}

												
#define MOTOR1CW()		{ TIM4->CCR1 = 0;\
												TIM4->CCR2 =  MotorControl[1].PWM_Duty;\
												}

#define MOTOR1CCW()		{ TIM4->CCR2 = 0;\
												TIM4->CCR1 = -MotorControl[1].PWM_Duty;\
												}

#define MOTOR1STOP()  {TIM4->CCR1 = 0;\
												TIM4->CCR2 = 0;\
												MotorControl[1].PWM_Duty = 0;}

#define MOTOR9CCW()		{}

#define MOTOR9CW()		{	}

#define MOTOR9STOP()	{	}

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

//#define BtoC    0x37d8	
//#define B_complementary_to_C        0x37d8
//#define BtoA    0x38d7
//#define B_complementary_to_A        0x38d7
//#define AtoC    0x378d	
//#define A_complementary_to_C        0x378d
//#define AtoB    0x387d	
//#define A_complementary_to_B        0x387d
//#define CtoA    0x3d87	
//#define C_complementary_to_A        0x3d87
//#define CtoB    0x3d78	
//#define C_complementary_to_B        0x3d78

#define OPEN_ALL_DOWN_TUBE   0x3ddd
#define CLOSE_ALL_TUBE       0x3888


												
												
												
typedef struct
{
    int16_t GetADCValue;//获取的ADC值
    int16_t FilterValue;//滤波后的ADC值
    float PreFilterVal;	//上个周期的滤波ADC值
    float	DeepFilterVAL;	//深度滤波的adc
    int16_t ADCValue_Ref;
    int16_t SetValue;		//上位机设定的电流值
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

} ADC_ValueParameters_t;

typedef struct
{
    uint8_t HallState;               //当前Hall状态
    uint8_t PrevHallState;           //上一次Hall状态
    uint8_t HallState_Temp;          //两次Hall状态组合，用来判断方向
    uint8_t HallState_CCW;
    uint8_t HallState_CW;
    uint8_t  HALL_CaptureValue_Direction_Judgment;
    uint16_t HALL_CaptureValue;      //Hall捕获值
    uint16_t HALL_PreCaptureValue;   //上一次Hall捕获值
    u32 PreHALL_OVF_Counter;         //上一次计数器溢出值
    s32 HALL_CaptureValueDelta;

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
typedef enum 
{
  Bm_cur_mode     = 1, 
  Bm_vel_mode     = 2,
	Bm_pos_mode     = 3
} BmModeSet;
//typedef struct
//{
//		uint8_t   Bm_id;//BM电机id
//		BmModeSet Bm_mode;//BM电机模式
//	  int16_t Speed_Set;//BM电机模式
//	  
//}Bm_Motor_Set;
//typedef struct
//{
//		
//	  
//}Bm_Motor_Read;
//typedef struct
//{ 
//		Bm_Motor_Set  Motor_Set;
//	  Bm_Motor_Read Motor_Read;
//	  
//}Bm_Motor;
typedef enum 
{ 
 Control_485=1,
 Control_IO=2
}Cont_Mode;
typedef struct
{
    int16_t Speed_Set;
    int16_t	Location_Set;
    int16_t Last_Location_Set;
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


    int8_t  Direction;
    int8_t  LastMotorDirection;
    FunctionalState  Motor_Start_Stop;

    ADC_ValueParameters_t	Current;
    HALL_Parameters_t   Hall;
    uint8_t Push_Location_model;
    uint8_t Push_motor_calibrationFLAG;
		BmModeSet Bm_mode; //0x01 ：设定为电流环 0x02 ：设定为速度环 0x03 ：设定为位置环
		uint8_t   Bm_fault_code;
		int8_t   temperature;
		Cont_Mode  Motor_mode; //边刷与风机控制模式 1:485控制  2：端口控制
//    Bm_Motor bm_motor;

} MotorControlParameters_t;

extern uint8_t Side_brush_report;
extern uint8_t Side_brush_live7;
extern uint8_t Side_brush_live8;

extern uint8_t Roll6_brush_report;
extern uint8_t Roll_Side_brush_live6;

extern MotorControlParameters_t MotorControl[MOTOR_NUM];


extern uint8_t PrevHallState1,PrevHallState2,HallState1,HallState2;
extern s16 BLDC1_Speed_RPM ;
extern s16 BLDC2_Speed_RPM ;


extern s32 HALL_CaptureValueDelta,HALL_CaptureValueDelta2;
extern u32 HALL_OVF_Counter  ;

//extern PID_Struct_t   PID_Speed_InitStruct[2];

#define SB_ID    4
#define SB2_ID   5
#define FAN_ID   0xA1
extern uint16_t RS_485_TIME;
#define SB_TIME   100
#define SB2_TIME  200
#define FAN_TIME  300

#define Total_TIME  330
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
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
