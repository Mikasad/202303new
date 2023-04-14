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


#define BREAK_CNT_MAX		1		//过流最大次数限制
#define PWMMAX	499
#define PWMMin        0

#define ABS(i)  ((i)>=0?(i):~(i))


//#define CurrentValue(A)   (A*14.74)//Current value=(14.74*A)A  3.6mR
#define CurrentValue(A)   (A*15.48)//Current value=(15.48*A)A  3.6mR

#define BusVoltValue(B)   (B*62.6)// BusVoltValue/ V
#define MOTOR0_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo
#define MOTOR1_CurrentValue(A)   (A*194) 	//A  电流检测范围为 21.15A 4mo
#define MOTOR2_CurrentValue(A)   (A*194) 	//A  电流检测范围为 21.15A 4mo
#define MOTOR3_CurrentValue(A)   (A*145)  //A  电流检测范围为28.2A 3mo
#define MOTOR4_CurrentValue(A)   (A*145) 	//A  电流检测范围为28.2A 3mo
#define MOTOR5_CurrentValue(A)   (A*116)	  //A  电流检测范围为35.25A 2mo
#define MOTOR6_CurrentValue(A)   (A*116)	  //A  电流检测范围为35.25A 2mo
//#define MOTOR7_CurrentValue(A)   (A*15.48)
//#define MOTOR8_CurrentValue(A)   (A*15.48)
#define MOTOR9_CurrentValue(A)    (A*194)	 //A  电流检测范围为 21.15A 4mo

//#define MOTOR10_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo
//#define MOTOR11_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo
//#define MOTOR12_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo
//#define MOTOR13_CurrentValue(A)   (A*194)  //A  电流检测范围为 21.15A 4mo

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


#define  NO_ERROR                        0X00000000

#define  PUSH_MOTOR1_OVER_CUR            0X00000001
#define  PUSH_MOTOR2_OVER_CUR            0X00000002
#define  PUSH_MOTOR3_OVER_CUR            0X00000004
#define  ONEWYA_MOTOR1_OVER_CUR          0X00000008

#define  ONEWYA_MOTOR2_OVER_CUR          0X00000010
#define  BLDC1_OVER_CUR           	 0X00000020
#define  BLDC2_OVER_CUR            	 0X00000040
#define  OVER_VOLTAGE                0X00000080

#define  UNDER_VOLTAGE               0X00000100
#define  PUSH_MOTOR4_OVER_CUR        0X00000200
#define  HALL5_SENSOR_ERR         	 0X00000400
#define  MOTOR4_BRAKE_LINE         	 0X00000800//   

#define  FAN_ERROR					 0X00001000//八号电机出错
#define  MOTOR5_PHASE_ERROR		     0X00002000//5相序错误
#define  MOTOR6_PHASE_ERROR          0X00004000//6相序错误
#define  HALL6_SENSOR_ERR        	 0X00008000

//#define  PUSHMOTOR_INT_ERROR		 0X00010000//
//#define  MOTOR6_NTC_BREAK_LINE       0X00020000//暂时未使用
#define  MOTOR5_OVER_SPEED 			 0X00040000//5号电机失速
#define  MOTOR6_OVER_SPEED           0X00080000//6号电机失速

#define  CAN_COMMUNICATION_ERR       0X00100000
#define  MOTOR5_MISSING_PHASE        0X00200000 //电机5启动时就缺相，无法检测是哪一相
#define  MOTOR6_MISSING_PHASE        0X00400000 //电机6启动时就缺相，无法检测是哪一相
#define  PUSH_BRAKE_LINE    		 0X00800000

#define  PUSH_LOST_HALL    			 0X01000000
#define  MOTOR3_BRAKE_LINE           0X02000000
#define  BRAKE_3_4    				 0X04000000
#define  BRAKE_0_1_2_9    			 0X08000000

#define  BRAKE10_11_12_13    		 0X10000000

#define  MOTOR5_BREAK		         0X20000000
#define  MOTOR6_BREAK                0X40000000
#define	 FLASH_WRITE_FAILED			 0X80000000
#define  SIDE_BRUSH_ERROR            0X00010000//七号电机出错

#define PUSHMOTOR_INT_ERROR          0X00020000
#define  PUSH_MOTOR_OVERTIME         0X80000000   //推杆过时



/*开始第二个32位全局错误变量*/
#define  PUSH0_BRAKE_LINE            0X00000001 //推杆0断线
#define  PUSH1_BRAKE_LINE            0X00000002 //推杆1断线
#define  PUSH2_BRAKE_LINE            0X00000004 //推杆2断线
#define  PUSH9_BRAKE_LINE            0X00000008 //推杆9断线

#define  PUSH0_LOST_HALL             0X00000010 //推杆0霍尔断线
#define  PUSH1_LOST_HALL             0X00000020 //推杆1霍尔断线
#define  PUSH2_LOST_HALL             0X00000040 //推杆2霍尔断线
#define  PUSH9_LOST_HALL             0X00000080 //推杆9霍尔断线

#define  MOTOR5_MISSING_PHASE_A      0X00000100 //电机5缺A
#define  MOTOR5_MISSING_PHASE_B      0X00000200 //电机5缺B
#define  MOTOR5_MISSING_PHASE_C      0X00000400 //电机5缺C
#define  MOTOR6_MISSING_PHASE_A      0X00000800 //电机6缺A   
#define  MOTOR6_MISSING_PHASE_B      0X00001000 //电机6缺B
#define  MOTOR6_MISSING_PHASE_C      0X00002000 //电机6缺C

#define	 PUSH0_OVERTIME				 0X00080000	//推杆0动作超时
#define	 PUSH1_OVERTIME				 0X00100000	//推杆1动作超时
#define	 PUSH2_OVERTIME				 0X00200000	//推杆2动作超时
#define	 PUSH9_OVERTIME				 0X00400000	//推杆9动作超时
#define	 PUSH5_OVERTIME	             0X00800000  //推杆5动作超时

//AD转换V的系数=0.015797
//#define Volt180V (1127)
//#define Volt200V (1252)
//#define Volt240V (1519)
//#define Volt330V (2066)
//#define Volt360V (2254)

#define VOLT_160V (1001)
#define VOLT_200V (1252)  //欠压恢复点
//#define VOLT_240V (1519)
//#define VOLT_330V (2066)
#define VOLT_300V (1878)  //过压恢复点
#define VOLT_360V (2254)



/*故障灯*/
#define LED_ON		1
#define	LED_OFF		0
#define LED_LOW_SPEED 		40
#define LED_HIGH_SPEED		20
#define GREEN_LED_ON     		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)
#define GREEN_LED_OFF 			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)
#define GREEN_LED_TOGGLE        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12)
#define	RED_LED2_ON             HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET) //上面灯
#define RED_LED2_TOGGLE         HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12)
#define	RED_LED2_OFF			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET)
#define	RED_LED1_ON 			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET) //下面灯
#define	RED_LED1_OFF			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define RED_LED1_TOGGLE         HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6)
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
******************************
MCU程序的版本号
******************************
*/
typedef struct
{
	u8 uVersionPartOfRobotType;				//车型对应的版本号
	u8 uVersionPartOfMainVersion;			//大版本号
	u8 uVersionPartOfFunVersion;			//功能版本
	u8 uVersionPartOfSmallVersion;		//bug修改，功能完善与优化
	u32 uVersionFullVersion;					//完整版本号
	u8 HardwarePartOfMotorType;				//驱动板电机类型
	u8 HardwarePartOfVotage;					//驱动板工作电压
	u8 HardwarePartOfCurrent;					//驱动板最大电流
	u8 HardwarePartOfVersion;					//驱动板更新版本
	u32 HardwareFullVersion;					//硬件完整版本号
} MCU_Version;
extern MCU_Version ProgramVersion;

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


extern  u32 wGlobal_Flags;
extern  u32 FaultOccurred;
extern u8 Push_motor_calibrationFLAG;
extern u8 Push_Location_model;
extern uint32_t ADC_ConvertedValue[12];
extern u32 prewGlobal_Flags;
extern u32 wGlobal_Flags1;
extern u32 FaultOccurred1 ;
extern uint32_t DisplayCounter;
/*
***********************************************
* User Functions
***********************************************
*/

void SysTime(void);
void CurrentLimit(u8 Motor_NUM);
void MotorStuckCheck(void);
void Over_VoltageCheck(void);
void MotorClearFault(void);
void MC_SetFault(u32 hFault_type);
void MC_ClearFault(u32 hFault_type);
void Init_Drive_Para(void);
void DisplayErrLed(void);
void Push_Motor_Cheek(void);
void Push_Motor_Location_Control(void);
void Push_Motor_Calibrate(void);
void Motor5_Default_Phase_Cheek(void);
void Motor5_Default_Phase_Cheek_IT(void);
void Motor6_Default_Phase_Cheek(void);
void Motor6_Default_Phase_Cheek_IT(void);
void Voltage_offset_cali(void);
void Hardware_flowCheck(void);
void BLDC5_Phase_Check(void);
void BLDC6_Phase_Check(void);
void BLDC1_OverSpdChk(void);
void BLDC2_OverSpdChk(void);
void Motor_Choose(void);
#endif
