/************************************************************
*	CopyRight (c) 2012-2012 ZL
*	All rights reserved.
*	FileName:		DAYE.c
*	Dependencies:
*	Revision:		1.0
*	Author:			solo
*	Date:			2015-01-01
*	Modified:
************************************************************/

#ifndef	_FUNCTION_H
#define	_FUNCTION_H

/*
***********************************************
* Include headfile
***********************************************
*/

//#include "include.h"
#include "stm32f4xx_hal.h"

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
/*
***********************************************
* Private define
***********************************************
*/
#define CurSelfAdapt_cur 250
#define U8_MAX     (255)
#define S8_MAX     (127)
#define S8_MIN     (-128)
#define U16_MAX    (65535u)
#define S16_MAX    (32767)
#define S16_MIN    (-32768)
#define U32_MAX    (4294967295uL)
#define S32_MAX    (2147483647)
#define S32_MIN    (-2147483648uL)


#define A1mSec		0x01
#define A2mSec		0x02
#define A10mSec		0x04
#define A20mSec		0x08
#define A250mSec	0x10
#define A500mSec	0x20
#define A1Sec		0x40


#define BREAK_CNT_MAX		5		//过流最大次数限制
#define PWMMAX	499
#define PWMMin        0
#define Cur_Adapt_Hall_MAX		38
#define Cur_Adapt_Hall_MIN		5

#define ABS(i)  ((i)>=0?(i):~(i))

#define MOTOR5CURCOEFFICIENT 0.38
#define MOTOR6CURCOEFFICIENT 0.21
//#define CurrentValue(A)   (A*14.74)//Current value=(14.74*A)A  3.6mR
#define CurrentValue(A)   (A*15.48)//Current value=(15.48*A)A  3.6mR

#define BusVoltValue(B)   (B*62.6)// BusVoltValue/ V
//#define MOTOR0_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo
//#define MOTOR1_CurrentValue(A)   (A*194) 	//A  电流检测范围为 21.15A 4mo
//#define MOTOR2_CurrentValue(A)   (A*194) 	//A  电流检测范围为 21.15A 4mo
//#define MOTOR3_CurrentValue(A)   (A*145)  //A  电流检测范围为28.2A 3mo
//#define MOTOR4_CurrentValue(A)   (A*145) 	//A  电流检测范围为28.2A 3mo
//#define MOTOR5_CurrentValue(A)   (A*MOTOR5CURCOEFFICIENT*100)	  //A  电流检测范围为35.25A 2mo  108 和114为实测修正后的1A的值
//#define MOTOR6_CurrentValue(A)   (A*MOTOR6CURCOEFFICIENT*100)	  //A  电流检测范围为35.25A 2mo
//#define MOTOR7_CurrentValue(A)   (A*15.48)
//#define MOTOR8_CurrentValue(A)   (A*15.48)
#define MOTOR9_CurrentValue(A)    (A*194)	 //A  电流检测范围为 21.15A 4mo

//#define MOTOR10_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo
//#define MOTOR11_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo
//#define MOTOR12_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo
//#define MOTOR13_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo


//#define BusVoltValue(B)   (B*62.6)// BusVoltValue/ V
#define MOTOR0_CurrentValue(A)   (A*373)  //A  20 15电流检测范围为 21.15A 4mo
#define MOTOR1_CurrentValue(A)   (A*373) 	//A  20 15电流检测范围为 21.15A 4mo
#define MOTOR2_CurrentValue(A)   (A*621) 	//A  20 30电流检测范围为 21.15A 4mo
#define MOTOR3_CurrentValue(A)   (A*467)  //A  电流检测范围为28.2A 3mo
#define MOTOR4_CurrentValue(A)   (A*1036) 	//A  电流检测范围为28.2A 3mo
//#define MOTOR5_CurrentValue(A)   (A*21)	  //A  电流检测范围为88A 3mo 5.67倍 21.1
//#define MOTOR6_CurrentValue(A)   (A*38)	  //A  电流检测范围为49A 5mo 6.12倍 37.98
#define MOTOR5_CurrentValue(A)   (A*MOTOR5CURCOEFFICIENT*100)	  //A 
#define MOTOR6_CurrentValue(A)   (A*MOTOR6CURCOEFFICIENT*100)	  //A  

#define MOTOR_5_6_ENABLE  HAL_GPIO_WritePin(GPIOE, BLDC_ENABLE_Pin, GPIO_PIN_SET);

#define MOTOR_0_1_ENABLE  HAL_GPIO_WritePin(GPIOA, RESET_1_Pin, GPIO_PIN_SET);

#define MOTOR_2_3_4_ENABLE  HAL_GPIO_WritePin(RESET_2_GPIO_Port, RESET_2_Pin, GPIO_PIN_SET);


#if 1
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
#define NEED_TO_CURRPROJECT_MOTORNUM  8   //需要过流保护电机个数
#define OVER_CURRENT(NUM) 	1<<NUM

/*第一个主错误*/
#define  NO_ERROR                        0X00000000

#define  PUSH_MOTOR1_OVER_CUR            0X00000001 //尘推推杆过流
#define  PUSH_MOTOR2_OVER_CUR            0X00000002 //滚刷推杆过流
#define  PUSH_MOTOR3_OVER_CUR            0X00000004 
#define  ONEWYA_MOTOR1_OVER_CUR          0X00000008 

#define  ONEWYA_MOTOR2_OVER_CUR          0X00000010 
#define  BLDC1_OVER_CUR           			 0X00000020 //
#define  BLDC2_OVER_CUR            			 0X00000040 //滚刷电机过流
#define  OVER_VOLTAGE                    0X00000080 //电源过压

#define  UNDER_VOLTAGE                   0X00000100 //电源欠压
#define  SIDE2_BRUSH_ERROR          	   0X00000200 //左边刷错误。。。
#define  HALL5_SENSOR_ERR         			 0X00000400 
#define  MOTOR4_BRAKE_LINE         			 0X00000800    

#define  FAN_ERROR					    				 0X00001000  //风机错误。。。
#define  MOTOR5_PHASE_ERROR		    			 0X00002000  //
#define  MOTOR6_PHASE_ERROR         		 0X00004000  //滚刷电机相序错误(hall或者动力线)
#define  HALL6_SENSOR_ERR        			   0X00008000  //滚刷电机hall错误

#define  SIDE_BRUSH_ERROR		        		 0X00010000//右边刷错误。。。
#define  PUSHMOTOR_INT_ERROR					   0X00020000//推杆标定失败。。。
#define  MOTOR5_OVER_SPEED 		           0X00040000//
#define  MOTOR6_OVER_SPEED               0X00080000//滚刷电机失速

#define  CAN_COMMUNICATION_ERR           0X00100000 //can通讯错误
#define  MOTOR5_MISSING_PHASE            0X00200000 
#define  MOTOR6_MISSING_PHASE            0X00400000 //滚刷电机缺相
#define  PUSH_BRAKE_LINE    					 	 0X00800000 //推杆断线。。。

#define  PUSH_LOST_HALL    					 		 0X01000000 //推杆hall错误
#define  MOTOR3_BRAKE_LINE               0X02000000 
#define  BRAKE_3_4    					       	 0X04000000 
#define  BRAKE_0_1_2    				 			   0X08000000 //刷盘升降或水趴升降推杆硬件保护

#define  BRAKE10_11_12_13    				 		 0X10000000 
#define  MOTOR5_BREAK										 0X20000000 
#define  MOTOR6_BREAK										 0X40000000 //滚刷电机硬件保护
#define  PUSH_OVER_TIME                  0X80000000 //推杆动作超时。。。

/*第一个主错误的子错误*/
#define  PUSH0_BRAKE_LINE            0X00000001 //尘推推杆断线
#define  PUSH1_BRAKE_LINE            0X00000002 //滚刷推杆断线
#define  PUSH2_BRAKE_LINE            0X00000004 //
#define  PUSH9_BRAKE_LINE            0X00000008 //

#define  PUSH0_LOST_HALL             0X00000010 //
#define  PUSH1_LOST_HALL             0X00000020 //滚刷推杆霍尔断线
#define  PUSH2_LOST_HALL             0X00000040 //
#define  PUSH9_LOST_HALL             0X00000080 //

#define  MOTOR5_MISSING_PHASE_A      0X00000100 //
#define  MOTOR5_MISSING_PHASE_B      0X00000200 //
#define  MOTOR5_MISSING_PHASE_C      0X00000400 //
#define  MOTOR6_MISSING_PHASE_A      0X00000800 //滚刷电机缺A  


#define  MOTOR6_MISSING_PHASE_B       0X00001000 //滚刷电机缺B
#define  MOTOR6_MISSING_PHASE_C       0X00002000 //滚刷电机缺C
#define  MOTOR8_VOLTAGE_ERROR					0X00004000	//风机电压异常
#define  MOTOR8_OVER_CURRENT					0X00008000	//风机过流

#define  MOTOR8_OVER_SPEED						0X00010000	//风机超速
#define  MOTOR8_OVER_TEMP							0X00020000	//风机过温
#define  MOTOR8_STUCK									0X00040000	//风机卡死
#define  PUSH0_OVERTIME               0X00080000  //尘推推杆动作超时

#define  SB1_SENSOR_ERR               0X00100000  //右边刷传感器故障
#define  PUSH2_OVERTIME               0X00200000  //
#define  SB2_SENSOR_ERR               0X00400000  //左边刷传感器故障
#define	 PUSH0_CALIBRATE							0X00800000  //

#define	 SB1_CUR_ERR					        0X01000000  //右边刷过流
#define	 SB1_PHASECUR_ERR		    	    0X02000000  //右边刷相过流
#define	 SB1_LOCKED_ERR						    0X04000000  //右边刷堵转
#define	 SB1_TEMP_ERR							    0X08000000  //右边刷过温

#define	 SB2_CUR_ERR					        0X10000000  //左边刷过流
#define	 SB2_PHASECUR_ERR		    	    0X20000000  //左边刷相过流
#define	 SB2_LOCKED_ERR						    0X40000000  //左边刷堵转
#define	 SB2_TEMP_ERR							    0X80000000  //左边刷过温
//第二个主错误
#define	 MOTOR7_DISCONNECT						0X00000001 //右边刷断连
#define	 MOTOR8_DISCONNECT						0X00000002 //左边刷断连
#define	 MOTOR9_DISCONNECT						0X00000004 //风机断连
//AD转换V的系数=0.015797
//#define Volt180V (1127)
//#define Volt200V (1252)
//#define Volt240V (1519)
//#define Volt330V (2066)
//#define Volt360V (2254)

//24V 1525  0.015737
#define VOLT_160V (1016)
//#define VOLT_190V (1210)
#define VOLT_200V (1252)  //欠压恢复点
//#define VOLT_240V (1519)
//#define VOLT_330V (2066)
#define VOLT_300V (1910)  //过压恢复点
#define VOLT_360V (2287)
//#define VOLT_310V (1974)


/*故障灯*/
#define LED_ON		1
#define	LED_OFF		0
#define LED_LOW_SPEED 		40
#define LED_HIGH_SPEED		20
#define GREEN_LED_ON     		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)
#define GREEN_LED_OFF 			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)
#define GREEN_LED_TOGGLE		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12)

#define	RED_LED2_ON         HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET) //下面灯
#define	RED_LED2_OFF				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET)
#define	RED_LED2_TOGGLE			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12)
#define	RED_LED1_ON 				HAL_GPIO_WritePin(FAULT1_GPIO_Port,FAULT1_Pin,GPIO_PIN_RESET) //上面灯
#define	RED_LED1_OFF				HAL_GPIO_WritePin(FAULT1_GPIO_Port,FAULT1_Pin,GPIO_PIN_SET)
#define	RED_LED1_TOGGLE			HAL_GPIO_TogglePin(FAULT1_GPIO_Port,FAULT1_Pin)
#endif

//***************************************************************
/*
***********************************************
* global variable
***********************************************
*/
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
typedef struct
{
    u8 SysTimFlag;	//system flag
    u8 MicroSec; /* 微秒计数 */
    u8 MilliSec; /* 毫秒计数 */
    u8 SecondCntr;	/*	1秒计数 */
    u8 test;
} SysCLC_TypeDef;
extern SysCLC_TypeDef SysCLC;//system clock variable



/*
***********************************************
*PWMVar Struct
***********************************************
*/
typedef struct
{
    u16 test;
    u16 StartDutyCycle;
    u16 CutOFFDutyCycle;
    u16 UpDateDutyCycle;
    u16 IntCnt;
    u16 DutyCycleMax;
    u16 DutyCycleMin;
} PWMVAR_TypeDef;
extern PWMVAR_TypeDef PWMVAR;//TIM1 variable

typedef struct
{
    u32 ab;
    u32 ac;
    u32 bc;
    u32 cnt;
	u32 maxcnt;
} DefaultPhaseCheekCurTemp;
extern DefaultPhaseCheekCurTemp Motor5_Current_temp;
extern DefaultPhaseCheekCurTemp Motor6_Current_temp;


/*
***********************************************
*VoltVar Struct
***********************************************
*/

typedef struct
{
    u16 AdBuf;
    u16 SVADBuf;
    u16 SVADAverage;
    u16 BUS;
    u16 BusLow;
    int16_t BusLowCnt;
    u16 HighGuardTime;
    u16 BusHigh;
    int16_t BusHighCnt;
    u16 LowGuardTime;
    u8 SvModelCnt;
    u16 servo;
    u16 ADTTT;
} VoltVar_TypeDef;
extern VoltVar_TypeDef VoltVar;

/*
******************************
MCU程序的版本号
******************************
*/
typedef struct
{
  u8 uVersionPartOfRobotType;        //车型对应的版本号
  u8 uVersionPartOfMainVersion;      //大版本号
  u8 uVersionPartOfFunVersion;      //功能版本
  u8 uVersionPartOfSmallVersion;    //bug修改，功能完善与优化
  u32 uVersionFullVersion;          //完整版本号
  u8 HardwarePartOfMotorType;        //驱动板电机类型
  u8 HardwarePartOfVotage;          //驱动板工作电压
  u8 HardwarePartOfCurrent;          //驱动板最大电流
  u8 HardwarePartOfVersion;          //驱动板更新版本
  u32 HardwareFullVersion;          //硬件完整版本号
} MCU_Version;
extern MCU_Version ProgramVersion;

extern  u32 wGlobal_Flags;
extern  u32 prewGlobal_Flags;
extern  u32 FaultOccurred;
extern  u32 wGlobal_Flags1;
extern  u32 FaultOccurred1;
extern  u32 wGlobal_Flags2;
extern  u32 FaultOccurred2;
extern uint32_t ADC_ConvertedValue[12];
#define brush_sample_num  50
#define brush_ave_cur     850 //无毛刷电流在600-700 ； 有毛刷电流在950以上（推杆越靠地面电流越大）

#define Roll_brush_sample_num  50 //测量次数
#define Roll_brush_ave_cur     20 //电流阈值
/*
***********************************************
* User Functions
***********************************************
*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver);
void SysTime(void);
void CurrentLimit(u8 Motor_NUM);
void MotorStuckCheck(void);
void Over_VoltageCheck(void);
void MotorClearFault(void);
void MC_SetFault(u32 hFault_type);
void MC_SetFault1(u32 hFault_type);
void MC_SetFault2(u32 hFault_type);
void MC_ClearFault(u32 hFault_type);
void MC_ClearFault2(u32 hFault_type);
void Motor_Fault_Clear(void);
void Init_Drive_Para(void);
void DisplayErrLed(void);
void Push_Motor_Cheek(void);
void Push_Motor_Location_Control(u8 motornum);
void Side_brush_exe(void);
void Roll_brush_exe(void);
void Push_Motor_Calibrate(u8 motornum);
void Motor5_Default_Phase_Cheek(void);
void Motor5_Default_Phase_Cheek_IT(void);
void Motor6_Default_Phase_Cheek(void);
void Motor6_Default_Phase_Cheek_IT(void);
void Voltage_offset_cali(void);
void Voltage_offset_cali_Inject(void);
void Hardware_flowCheck(void);
void BLDC5_Phase_Check(void);
void BLDC6_Phase_Check(void);
void Push_Motor_CurSelfAdapt(u8 bldc_num,u8 push_num,u16 hall_max,u16 hall_min);
void Motor7_Err_Chk(void);
void Motor9_Err_Chk(void);
void BLDC_Stuck_Chk(void);
void BLDC1_OverSpdChk(void);
void BLDC2_OverSpdChk(void);
void HardVersion_Init(MCU_Version *pVersion,u8 funtype,u8 vol,u8 cur_max,u8 update_ver);
void Push_OverRunChk(u8 push1,u8 timer0,u8 push2,u8 timer1,u8 push3,u8 timer2,u8 push4,u8 timer3);
#endif
