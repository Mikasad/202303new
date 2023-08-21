//标准头文件
#include <stdlib.h>
//stm32头文件
#include "stm32f4xx_hal.h"

//FREERTOS
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"
//#include "croutine.h"
//#include "semphr.h"
//#include "event_groups.h"
//APP头文件
//#include "motor_driver.h"
#include "ctr_move_task.h"
#include "buf2.h"
#include "tim.h"
#include "iwdg_task.h"
/*
*********************************************************************************************************
*
*	模块名称 : BLDC驱动器控制模块
*	文件名称 : BLDCdriver.c
*	版    本 : 
*	说    明 : 
*			1，负责BLDC驱动器的控制、报警信息采集等功能
*			2，外部脉冲计数功能，该功能的函数使用了__weak属性，当有其他编码器被接入时这里的会被覆盖
*			
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.0    2018-11-05  杨臣   	正式发布
*		V1.0.1	  2018-12-13  杨臣	新接口，对外提供发送到驱动器的速度。
*	外设资源占用：
*		BLDC驱动器控制+报警信息采集：
*			TIM：TIM2
*			IO口：PA0,PA1,PA6,PA7,PE7,PF13,PF14,PF15,PG0,PG1
*		脉冲计数:
*			TIM：TIM3，TIM4
*			IO口：PB1,PE0
*********************************************************************************************************
*/

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/
//pwm计数周期
#define PWMTIMER_PERIOD		1000
//电机的最大转速
#define MOTOR_MAX_SPEED_RPM 3000 //需要与上位机保持一致

//控制
#define ENABLE_DRIVER_LEFT()		HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin,GPIO_PIN_SET)
#define DISABLE_DRIVER_LEFT()		HAL_GPIO_WritePin(EN0_GPIO_Port, EN0_Pin,GPIO_PIN_RESET)
#define ENABLE_BRK_LEFT()			HAL_GPIO_WritePin(BRK0_GPIO_Port, BRK0_Pin, GPIO_PIN_SET)
#define DISABLE_BRK_LEFT()			HAL_GPIO_WritePin(BRK0_GPIO_Port, BRK0_Pin, GPIO_PIN_RESET)
#define SET_FRONTWARD_LEFT()		HAL_GPIO_WritePin(DIR0_GPIO_Port, DIR0_Pin, GPIO_PIN_RESET)
#define SET_BACKWARD_LEFT()		    HAL_GPIO_WritePin(DIR0_GPIO_Port, DIR0_Pin, GPIO_PIN_SET)

#define ENABLE_DRIVER_RIGHT()		HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin,GPIO_PIN_SET)
#define DISABLE_DRIVER_RIGHT()	    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin,GPIO_PIN_RESET)
#define ENABLE_BRK_RIGHT()			HAL_GPIO_WritePin(BRK1_GPIO_Port, BRK1_Pin, GPIO_PIN_SET)
#define DISABLE_BRK_RIGHT()		    HAL_GPIO_WritePin(BRK1_GPIO_Port, BRK1_Pin, GPIO_PIN_RESET)
#define SET_FRONTWARD_RIGHT()		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET)
#define SET_BACKWARD_RIGHT()		HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET)

//出现报警时的外部电平，0=低，1=高
#define L_ALM_SET		  1
#define R_ALM_SET		  1
//报警状态，0=无报警；1=有报警
#define L_ALM_IN()	((HAL_GPIO_ReadPin(ALM_0_GPIO_Port,ALM_0_Pin)&0x01) == L_ALM_SET)
#define R_ALM_IN()	((HAL_GPIO_ReadPin(ALM_1_GPIO_Port,ALM_1_Pin)&0x01) == R_ALM_SET)

#define ALM_IN()		( R_ALM_IN()  | L_ALM_IN() )	

//使能时的外部电平，0=低，1=高
#define L_ENABLE_SET		  1
#define R_ENABLE_SET		  1
//使能状态，0=失能；1=使能
#define L_ENABLE_IN()	((HAL_GPIO_ReadPin(EN0_GPIO_Port,EN0_Pin)&0x01) == L_ENABLE_SET)
#define R_ENABLE_IN()	((HAL_GPIO_ReadPin(EN1_GPIO_Port,EN1_Pin)&0x01) == R_ENABLE_SET)

//刹车的外部电平，0=低，1=高
#define L_BRK_SET		  1
#define R_BRK_SET		  1
//刹车状态，0=向后；1=向前
#define L_BRK_IN()	((HAL_GPIO_ReadPin(BRK0_GPIO_Port, BRK0_Pin)&0x01) == L_BRK_SET)
#define R_BRK_IN()	((HAL_GPIO_ReadPin(BRK1_GPIO_Port, BRK1_Pin)&0x01) == R_BRK_SET)

//向前的外部电平，0=低，1=高
#define L_FRONT_SET		  1
#define R_FRONT_SET		  1
//方向状态，0=向后；1=向前
#define L_DIR_IN()	((HAL_GPIO_ReadPin(DIR0_GPIO_Port,DIR0_Pin)&0x01) == L_FRONT_SET)
#define R_DIR_IN()	((HAL_GPIO_ReadPin(DIR1_GPIO_Port,DIR1_Pin)&0x01) == R_FRONT_SET)
	


#define LIMIT32_P ((int32_t)0x7FFFFFFF)
#define LIMIT32_N ((int32_t)0x80000000)
#define LIMIT16_P ((int16_t)0x7FFF)
#define LIMIT16_N ((int16_t)0x8000) 


/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/

typedef enum{
	WHEEL_ROTATE_DIR_FORWARD = 1,
	WHEEL_ROTATE_DIR_STOP = 0,
	WHEEL_ROTATE_DIR_BACKWARD = -1
}wheelRotateDir_t;
typedef struct
{
	int32_t unitCnt;//单元计数，单次采样的脉冲计数
	int32_t relativeCnt;//相对计数
	int32_t lastCnt;//上次的总数
	int32_t cnt;//当前的总数
}pulseCnt_t;
typedef struct{
	uint8_t enable;	//使能
	uint8_t brk; 		//刹车
	uint8_t alarm;	//报警
}motorDriverState_t;

/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/
TaskHandle_t taskHandleMotorDriver = NULL;

//驱动器的控制
static FunctionalState gs_cfgLeftBrk = DISABLE;
static FunctionalState gs_cfgRightBrk = DISABLE;
static FunctionalState gs_cfgEnable= ENABLE;
//左右的方向
static wheelRotateDir_t gs_leftCurDir,gs_rightCurDir;
//电机驱动器状态
static motorDriverState_t gs_leftMotorDriverState,gs_rightMotorDriverState;
//左右的速度
static int32_t gs_leftSpeed,gs_rightSpeed;
static SemaphoreHandle_t gs_speedMutex;
//最终写入到驱动器的速度值
static int32_t gs_leftSpeedToDriver,gs_rightSpeedToDriver;
//脉冲计数
//static uint16_t gs_rightPulseCntInIRQ;//中断中计数
static pulseCnt_t gs_leftPulseCnt,gs_rightPulseCnt;
SemaphoreHandle_t gs_leftPulseCntMutex,gs_rightPulseCntMutex;
//计数溢出标记
uint8_t g_leftCountOutFlag = 0;//1=正溢出，2=负溢出
uint8_t g_RightCountOutFlag = 0;//1=正溢出，2=负溢出



/************************************************************************************************************************************
																静态函数声明
*************************************************************************************************************************************/
static wheelRotateDir_t GetLeftCurDir(void);
static wheelRotateDir_t GetRightCurDir(void);
static void SetSpeed(int32_t f_leftSpeed, int32_t f_rightSpeed);
static FunctionalState GetLeftDriverEnableState(void);
static FunctionalState GetRightDriverEnableState(void);
static FunctionalState GetLeftDriverBrkState(void);
static FunctionalState GetRightDriverBrkState(void);
static void EnableDriver(FunctionalState f_newState);
static void EnableLeftDriverBrk(FunctionalState f_newState);
static void EnableRightDriverBrk(FunctionalState f_newState);
static void SetDutyRatio(uint16_t f_leftRatio,uint16_t f_rightRatio);
static void SetMotorDir(wheelRotateDir_t f_leftDir, wheelRotateDir_t f_rightDir);
static void ReadSpeedVal(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed);
static void GetIoState(void);//获取管脚的状态信息

static uint16_t ReadRightPulseCnt(void);
static uint16_t ReadLeftPulseCnt(void);
static int InitDecoderMutex(void);


/************************************************************************************************************************************
																软硬件初始化
*************************************************************************************************************************************/


//0=成功，1=失败，-1=错误
static int InitSpeedMutex(void)
{
	gs_speedMutex = xSemaphoreCreateMutex();
	if(gs_speedMutex == NULL)
		return 1;
	return 0;
}
//0=成功，1=失败，-1=错误
static int InitDecoderMutex(void)
{
	gs_leftPulseCntMutex = xSemaphoreCreateMutex();
	if(gs_leftPulseCntMutex == NULL)
		return 1;
	
	gs_rightPulseCntMutex = xSemaphoreCreateMutex();
	if(gs_rightPulseCntMutex == NULL)
		return 1;

	return 0;
}


/************************************************************************************************************************************
																中断处理
*************************************************************************************************************************************/

/************************************************************************************************************************************
																内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/
static void ReadSpeedVal(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed)
{
	xSemaphoreTake(gs_speedMutex, portMAX_DELAY);
	*fp_leftSpeed = gs_leftSpeed; 
	*fp_rightSpeed = gs_rightSpeed; 
	xSemaphoreGive(gs_speedMutex);
}
static FunctionalState GetLeftDriverEnableState(void)
{
	return (FunctionalState)gs_leftMotorDriverState.enable;
}
static FunctionalState GetRightDriverEnableState(void)
{
	return (FunctionalState)gs_rightMotorDriverState.enable;
}
static FunctionalState GetLeftDriverBrkState(void)
{
	return (FunctionalState)gs_leftMotorDriverState.brk;
}
static FunctionalState GetRightDriverBrkState(void)
{
	return (FunctionalState)gs_rightMotorDriverState.brk;
}

//使能/失能驱动器
static void EnableDriver(FunctionalState f_newState)
{
	if(f_newState == ENABLE)
	{
		ENABLE_DRIVER_LEFT();
		ENABLE_DRIVER_RIGHT();
	}
	else
	{
		DISABLE_DRIVER_LEFT();
		DISABLE_DRIVER_RIGHT();
	}
}
static void GetIoState(void)//获取管脚的状态信息
{
	gs_leftMotorDriverState.enable 	= L_ENABLE_IN();
	gs_leftMotorDriverState.brk 	= L_BRK_IN();
	gs_leftMotorDriverState.alarm 	= L_ALM_IN();
	
	gs_rightMotorDriverState.enable = R_ENABLE_IN();
	gs_rightMotorDriverState.brk 	= R_BRK_IN();
	gs_rightMotorDriverState.alarm 	= R_ALM_IN();
}
//使能/失能刹车
static void EnableLeftDriverBrk(FunctionalState f_newState)
{
	if(f_newState == ENABLE)
	{
		ENABLE_BRK_LEFT();
	}
	else
	{
		DISABLE_BRK_LEFT();
	}	
}
static void EnableRightDriverBrk(FunctionalState f_newState)
{
	if(f_newState == ENABLE)
	{
		ENABLE_BRK_RIGHT();
	}
	else
	{
		DISABLE_BRK_RIGHT();
	}	
}


//设置PWM占空比
static void SetDutyRatio(uint16_t f_leftRatio,uint16_t f_rightRatio)
{
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, f_leftRatio);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, f_rightRatio);
}
//设置方向，
static void SetMotorDir(wheelRotateDir_t f_leftDir, wheelRotateDir_t f_rightDir)
{
	if(f_leftDir == WHEEL_ROTATE_DIR_BACKWARD)
		SET_BACKWARD_LEFT();
	else
		SET_FRONTWARD_LEFT();
	
		
	if(f_rightDir == WHEEL_ROTATE_DIR_BACKWARD)
		SET_BACKWARD_RIGHT();
	else
		SET_FRONTWARD_RIGHT();
	
	gs_leftCurDir = f_leftDir;
	gs_rightCurDir = f_rightDir; 
	
}
//获取方向
static wheelRotateDir_t GetLeftCurDir(void)
{
	return gs_leftCurDir;
}
static wheelRotateDir_t GetRightCurDir(void)
{
	return gs_rightCurDir;
}
//设置速度，包括转向和转速
static void SetSpeed(int32_t f_leftSpeed, int32_t f_rightSpeed)
{
	//占空比
	uint16_t leftDutyRatio = 0;
	uint16_t rightDutyRatio = 0;
	
	//把0-1000的千分比数值转为实际的转速，MOTOR_MAX_SPEED_RPM为电机的最大转速
	f_leftSpeed = MOTOR_MAX_SPEED_RPM * f_leftSpeed /1000;
	f_rightSpeed = MOTOR_MAX_SPEED_RPM * f_rightSpeed /1000;
	
	leftDutyRatio = abs(f_leftSpeed);
	rightDutyRatio = abs(f_rightSpeed);
	//方向
	wheelRotateDir_t leftDir = WHEEL_ROTATE_DIR_STOP;
	wheelRotateDir_t rightDir = WHEEL_ROTATE_DIR_STOP;
	//作乱反相
	if(f_leftSpeed > 0)
		leftDir = WHEEL_ROTATE_DIR_FORWARD;
	else if(f_leftSpeed < 0)
		leftDir = WHEEL_ROTATE_DIR_BACKWARD;
	else
		leftDir = WHEEL_ROTATE_DIR_STOP;
	
	if(f_rightSpeed > 0)
		rightDir = WHEEL_ROTATE_DIR_FORWARD;
	else if(f_rightSpeed < 0)
		rightDir = WHEEL_ROTATE_DIR_BACKWARD;
	else
		rightDir = WHEEL_ROTATE_DIR_STOP;
	
	//1,限制设置占空比的最大值
	leftDutyRatio = leftDutyRatio > PWMTIMER_PERIOD?PWMTIMER_PERIOD:leftDutyRatio;
	rightDutyRatio = rightDutyRatio > PWMTIMER_PERIOD?PWMTIMER_PERIOD:rightDutyRatio;
	//1，设置转向
	SetMotorDir(leftDir, rightDir);
	//2，设置PWM占空比
	SetDutyRatio(leftDutyRatio,rightDutyRatio);
	gs_leftSpeedToDriver = (leftDir == WHEEL_ROTATE_DIR_BACKWARD)?(-leftDutyRatio):leftDutyRatio;
	gs_rightSpeedToDriver = (rightDir == WHEEL_ROTATE_DIR_BACKWARD)?(-rightDutyRatio):rightDutyRatio;
}


static uint16_t ReadRightPulseCnt(void)
{	
	uint16_t countRet = 0;
	countRet = __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER( &htim4, 0 );
	return countRet;
}
static uint16_t ReadLeftPulseCnt(void)
{
	uint16_t countRet = 0;
    
    countRet = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER( &htim2, 0 );
	
	return countRet;
}



/*
*单元计数是为了以较快的频率读取编码器计数并清零，以防止高速时较低频率读取导致计数器溢出
*该函数允许短时间阻塞，在命令读取编码计数的时候会出现阻塞的情况。
*/
static void DecoderUnitCnt(void)//处理编码器的单元计数
{
	int32_t val;
	
	//1,读取脉冲计数
	gs_rightPulseCnt.unitCnt = ReadRightPulseCnt();
	gs_leftPulseCnt.unitCnt = ReadLeftPulseCnt();
	//2，BLDC驱动器反馈的脉冲没有方向，根据当前方向设置正负，有误差
	if(GetLeftCurDir() == WHEEL_ROTATE_DIR_BACKWARD)
		gs_leftPulseCnt.unitCnt = -gs_leftPulseCnt.unitCnt;
	else if(GetLeftCurDir() == WHEEL_ROTATE_DIR_STOP)
		gs_leftPulseCnt.unitCnt = 0;
	
	if(GetRightCurDir() == WHEEL_ROTATE_DIR_BACKWARD)
		gs_rightPulseCnt.unitCnt = -gs_rightPulseCnt.unitCnt;
	else if(GetLeftCurDir() == WHEEL_ROTATE_DIR_STOP)
		gs_rightPulseCnt.unitCnt = 0;
	//3,
	val = gs_rightPulseCnt.cnt + gs_rightPulseCnt.unitCnt;
	xSemaphoreTake(gs_rightPulseCntMutex, portMAX_DELAY);
	if(gs_rightPulseCnt.cnt > 0 && val < (LIMIT32_N/2))//正数加正数越界
	{
		gs_rightPulseCnt.cnt = val + LIMIT32_N;
		g_RightCountOutFlag = 1;
	}
	else if(gs_rightPulseCnt.cnt < 0 && val > (LIMIT32_P/2))//负数加负数越界
	{
		gs_rightPulseCnt.cnt = val - LIMIT32_P;
		g_RightCountOutFlag = 2;
	}
	else
		gs_rightPulseCnt.cnt = val;
	xSemaphoreGive(gs_rightPulseCntMutex);


	val = gs_leftPulseCnt.cnt + gs_leftPulseCnt.unitCnt;
	xSemaphoreTake(gs_leftPulseCntMutex, portMAX_DELAY);
	if(gs_leftPulseCnt.cnt > 0 && val < (LIMIT32_N/2))//正数加正数越界
	{
		gs_leftPulseCnt.cnt = val + LIMIT32_N;
		g_leftCountOutFlag = 1;
	}
	else if(gs_leftPulseCnt.cnt < 0 && val > (LIMIT32_P/2))//负数加负数越界
	{
		gs_leftPulseCnt.cnt = val - LIMIT32_P;
		g_leftCountOutFlag = 2;
	}
	else
		gs_leftPulseCnt.cnt = val;
	xSemaphoreGive(gs_leftPulseCntMutex);
}
static void DecoderRelativeCnt(void)//处理编码器的相对计数,改函数应该在DecoderUnitCnt之后调用
{
	xSemaphoreTake(gs_leftPulseCntMutex, portMAX_DELAY);
	if(g_leftCountOutFlag == 1)
	{
		gs_leftPulseCnt.relativeCnt= LIMIT32_N - gs_leftPulseCnt.lastCnt + gs_leftPulseCnt.cnt;
	}
	else if(g_leftCountOutFlag == 2)
	{
		gs_leftPulseCnt.relativeCnt= LIMIT32_P - gs_leftPulseCnt.lastCnt + gs_leftPulseCnt.cnt;
	}
	else
	{
		gs_leftPulseCnt.relativeCnt=gs_leftPulseCnt.cnt-gs_leftPulseCnt.lastCnt;
	}
	g_leftCountOutFlag = 0;
	gs_leftPulseCnt.lastCnt  = gs_leftPulseCnt.cnt;	
	xSemaphoreGive(gs_leftPulseCntMutex);
	
	xSemaphoreTake(gs_rightPulseCntMutex, portMAX_DELAY);
	if(g_RightCountOutFlag == 1)
	{
		gs_rightPulseCnt.relativeCnt= LIMIT32_N - gs_rightPulseCnt.lastCnt + gs_rightPulseCnt.cnt;
	}
	else if(g_RightCountOutFlag == 2)
	{
		gs_rightPulseCnt.relativeCnt= LIMIT32_P - gs_rightPulseCnt.lastCnt + gs_rightPulseCnt.cnt;
	}
	else
	{
		gs_rightPulseCnt.relativeCnt=gs_rightPulseCnt.cnt-gs_rightPulseCnt.lastCnt;
	}
	g_RightCountOutFlag = 0;
	gs_rightPulseCnt.lastCnt  = gs_rightPulseCnt.cnt;
	xSemaphoreGive(gs_rightPulseCntMutex);
}



/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/

void WriteSpeedVal(int32_t f_leftSpeed,int32_t f_rightSpeed)
{
	xSemaphoreTake(gs_speedMutex, portMAX_DELAY);
    gs_leftSpeed = f_leftSpeed; 
	gs_rightSpeed = f_rightSpeed; 
    xSemaphoreGive(gs_speedMutex);
	
}
void GetSpeedValToMotorDriver(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed)
{
	*fp_leftSpeed = gs_leftSpeedToDriver;
	*fp_rightSpeed = gs_rightSpeedToDriver;
}
//6代盒子BLDC驱动器刹车，左右侧都用的同一个引脚，所以一个刹车另一个也会刹车
void CfgDriverBrk(brkStatus_t f_brkStatus)
{
	switch(f_brkStatus)
	{
		case BRK_STATUS_NO_BRK:
			gs_cfgLeftBrk = DISABLE;
			gs_cfgRightBrk = DISABLE;
		break;
		case BRK_STATUS_ALL_BRK:
            gs_cfgLeftBrk = ENABLE;
			gs_cfgRightBrk = ENABLE;
            break;
		case BRK_STATUS_LEFT_BRK:
            gs_cfgLeftBrk = ENABLE;
            gs_cfgRightBrk = DISABLE;
            break;
		case BRK_STATUS_RIGHT_BRK:
			gs_cfgRightBrk = ENABLE;
            gs_cfgLeftBrk = DISABLE;
            break;
		default:
		break;
	}
}
void CfgDriverEnable(FunctionalState f_newState)
{
	gs_cfgEnable = f_newState;
}
//获取报警信息,SET=有报警，RESET=无报警
FlagStatus GetLeftDriverAlarmState(void)
{
	return (FlagStatus)gs_leftMotorDriverState.alarm;
}
FlagStatus GetRightDriverAlarmState(void)
{
	return (FlagStatus)gs_rightMotorDriverState.alarm;
}

//设置报警信息,SET=有报警，RESET=无报警
void SetLeftDriverAlarm(FlagStatus f_cmd)
{
	gs_leftMotorDriverState.alarm = f_cmd;
}
void SetRightDriverAlarm(FlagStatus f_cmd)
{
	gs_rightMotorDriverState.alarm = f_cmd;
}
uint8_t GetMotorDriverHealthState(void)
{
	uint8_t ret = 0;//1=健康
	ret = (GetLeftDriverAlarmState() == RESET)?1:0;
	ret &= (GetRightDriverAlarmState() == RESET)?1:0;
	return ret;//两者都健康认为健康
}
uint8_t GetLeftMotorHealthState(void)
{
	uint8_t ret = 0;//1=健康
	ret = (GetLeftDriverAlarmState() == RESET)?1:0;
	return ret;
}
uint8_t GetRightMotorHealthState(void)
{
	uint8_t ret = 0;//1=健康
	ret = (GetRightDriverAlarmState() == RESET)?1:0;
	return ret;
}

uint8_t GetMotorBrakeDownState(void)
{
	uint8_t ret = 0;//1=刹车
	ret = (gs_leftMotorDriverState.brk == SET)?1:0;
	ret |= (gs_rightMotorDriverState.brk == SET)?1:0;
	return ret;//两者有一个刹车认为刹车
}
//获取总数和相对计数值
__weak void GetLeftDecoderInfo(int32_t *fp_cnt, int32_t *fp_relativeCnt)
{
	xSemaphoreTake(gs_leftPulseCntMutex, portMAX_DELAY);
    *fp_cnt = gs_leftPulseCnt.lastCnt; //这里返回上次的值是因为相对计数是与上次的值对应的
	*fp_relativeCnt = gs_leftPulseCnt.relativeCnt; 
    xSemaphoreGive(gs_leftPulseCntMutex);
}
__weak void GetRightDecoderInfo(int32_t *fp_cnt, int32_t *fp_relativeCnt)
{
	xSemaphoreTake(gs_rightPulseCntMutex, portMAX_DELAY);
    *fp_cnt = gs_rightPulseCnt.lastCnt; //这里返回上次的值是因为相对计数是与上次的值对应的
	*fp_relativeCnt = gs_rightPulseCnt.relativeCnt; 
    xSemaphoreGive(gs_rightPulseCntMutex);
}


/************************************************************************************************************************************
												任务+对外的硬件初始化+对外的软件初始化
*************************************************************************************************************************************/

void MotorDriverTask(void const *pvParameters)
{
	uint8_t cnt = 0;
	TickType_t lastWakeTime;
	int32_t leftSpeed,rightSpeed;
	const TickType_t frequency = pdMS_TO_TICKS(1);
	uint32_t bitPos = (uint32_t)pvParameters;
	TickType_t readCntTime = 0;
	const TickType_t readCntFrequency = pdMS_TO_TICKS(50);//xms进行一次编码器单元计数
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
	
    while(1)
    {
		/* 喂狗 */
		FeedDog(bitPos);
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelayUntil(&lastWakeTime, frequency);
		if(readCntTime + readCntFrequency < xTaskGetTickCount())
		{
			DecoderUnitCnt();//单元计数
			cnt++;
			if(cnt >= 2)//n次单元计数之后(100ms)进行一次相对计算
			{
				DecoderRelativeCnt();//相对计数
				cnt = 0;
			}
			readCntTime = xTaskGetTickCount();
		}
		GetIoState();
		//1,获取速度
		ReadSpeedVal(&leftSpeed,&rightSpeed);
		//2,判断使能状态
		if(gs_cfgEnable != GetLeftDriverEnableState() || gs_cfgEnable != GetRightDriverEnableState())
		{
			EnableDriver(gs_cfgEnable);
		}
		//3，判断刹车状态,注意6代盒子BLDC驱动器刹车，左右侧都用的同一个引脚，所以一个刹车另一个也会刹车
		if(gs_cfgLeftBrk != GetLeftDriverBrkState())
			EnableLeftDriverBrk(gs_cfgLeftBrk);
		if(gs_cfgRightBrk != GetRightDriverBrkState())
			EnableRightDriverBrk(gs_cfgRightBrk);

		if(GetLeftDriverBrkState() == ENABLE || GetLeftDriverEnableState() == DISABLE)
			leftSpeed = 0;
		if(GetRightDriverBrkState() == ENABLE || GetRightDriverEnableState() == DISABLE)
			rightSpeed = 0;
		if(GetMotorDriverHealthState() == 0 )
			leftSpeed = rightSpeed = 0;
		//4,控制运动
		SetSpeed(leftSpeed, rightSpeed);
//		SetSpeed(100, 100);
        
        g_thread_call_count[thread_num]++;
    }
	
	
}


//0=成功
int InitSoftwareMotorDriver(void)
{
	return InitSpeedMutex();
}

//0=成功
__weak int InitHardwareDecoder(void)
{
	return 0;
}

__weak int InitSoftwareDecoder(void)
{
	return InitDecoderMutex();
}
