//标准头文件
#include <stdlib.h>
//stm32头文件
#include "stm32f4xx_hal.h"
//APP头文件
#include "motor_driver.h"
#include "can_task.h"
#include "security_state.h"
#include "objdict.h"
#include "main.h"
/*
*********************************************************************************************************
*
*	模块名称 :大能驱动器控制模块
*	文件名称 : daneng_driver.c
*	版    本 : 	V1.1.0
*	说    明 : 
*			1，使用CAN通信
*			
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.0    2018-11-05  杨臣   	正式发布
*		V1.0.1    2018-11-30  杨臣    向外输出写入驱动器的速度值
*		V1.0.2    2018-11-30  杨臣	修改了速度设置周期的逻辑，删除了对delta的限制。
*									电机是3000转的，所以速度设置时乘以3
*		V1.0.3	  2018-12-01  杨臣	速度的设置改回20ms一次的逻辑
*		V1.0.4	  2018-12-03  杨臣	编码计算的时机做了修改
*       V1.1.0    2019-11-26  ypc
*	外设资源占用：
*		CAN:CAN2
*********************************************************************************************************
*/


/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/
#define LEFT_ID	0x01
#define RIGHT_ID 0X02

//驱动器速度控制，单位RPM
#define INDEX_SET_SPEED 0x3020
#define SUBINDEX_SET_SPEED 0x00
//驱动器扭矩控制，单位0.068A
#define INDEX_SET_TORQUE 0x3021
#define SUBINDEX_SET_TORQUE 0x00
//设置控制模式,0=速度模式，1=力矩模式
#define INDEX_SET_MODE 0x3022
#define SUBINDEX_SET_MODE 0x00
//获取电池电压，单位V
#define INDEX_GET_BAT_VOLTAGE 0x3023
#define SUBINDEX_GET_BAT_VOLTAGE 0x00
//获取Q轴电流，单位A
#define INDEX_GET_Q_AXIS_CURRENT 0x3024
#define SUBINDEX_GET_Q_AXIS_CURRENT 0x00
//获取速度反馈，单位RPM
#define INDEX_GET_FEEDBACK_SPEED 0x3025
#define SUBINDEX_GET_FEEDBACK_SPEED 0x00
//获取当前控制状态
#define INDEX_GET_CTR_STATUS 0x3026
#define SUBINDEX_GET_CTR_STATUS 0x00
//获取故障码1
#define INDEX_GET_CTR_ERROR_MASK1 0x3027
#define SUBINDEX_GET_CTR_ERROR_MASK1 0x00
//获取故障码2
#define INDEX_GET_CTR_ERROR_MASK2 0x3028
#define SUBINDEX_GET_CTR_ERROR_MASK2 0x00
//驱动器失能/使能
#define INDEX_ENABLE 0x3029
#define SUBINDEX_ENABLE 0x00
//获取编码器计数
#define INDEX_GET_ENCODER_CNT 0x3030
#define SUBINDEX_GET_ENCODER_CNT 0x00
//超过清零
#define LIMIT_CLEAN_P ((int32_t)0x7FFFA3A0)
#define LIMIT_CLEAN_N ((int32_t)0x80005C60)
//32位有符号是的最大值和最小值
#define LIMIT32_P ((int32_t)0x7FFFFFFF)
#define LIMIT32_N ((int32_t)0x80000000)
//相对计数绝对值超过了下面这个值，说明计数出现了溢出
#define THRESHOLD_OF_CNT_OVERFLOW  0x20000000    //这个值应该相对大，但是不能太大，不要太接近 LIMIT_CLEAN_P

/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/


typedef enum{
	ERROR_MOS_U = 0X01,
	ERROR_MOS_V = 0X02,
	ERROR_MOS_W = 0X04,
	ERROR_OVER_VOLTAGE = 0X10,
	ERROR_UNDER_VOLTAGE = 0X20,
	ERROR_PRECHARGE = 0X40,
	ERROR_KEY_VOLTAGE = 0X80,
	ERROR_OPEN = 0X10000,
	ERROR_RELAY = 0X20000,
	ERROR_OVER_TEMPERATURE = 0X80000,
	ERROR_OVER_CURRENT = 0X100000,
}errorCodeMask1_t;

typedef struct
{
	int32_t relativeCnt;//相对计数
	int32_t lastCnt;//上次的总数
	int32_t cnt;//当前的总数
}encoderCnt_t;

typedef struct{
	uint8_t enable;	//使能
	uint8_t brk; 		//刹车
	uint8_t alarm;	//报警
}motorDriverState_t;
typedef enum{
	UNHEALTHY = 0, 
	HEALTHY = !UNHEALTHY,
}healthState_t;

/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/
//任务句柄
TaskHandle_t taskHandleMotorDriver = NULL;
//驱动器的控制
static FunctionalState gs_cfgLeftBrk = DISABLE;
static FunctionalState gs_cfgRightBrk = DISABLE;
static FunctionalState gs_cfgEnable= ENABLE;
//电机驱动器状态
static motorDriverState_t gs_leftMotorDriverState,gs_rightMotorDriverState;
//左右的速度
static int32_t gs_leftSpeedToDriver,gs_rightSpeedToDriver;
static SemaphoreHandle_t gs_speedToDriverMutex;
static int32_t gs_leftSpeed,gs_rightSpeed;
static SemaphoreHandle_t gs_speedMutex;
//与驱动器的的连接状态，规定时间内能够收到心跳或者其他信息则认为连接正常
static linkStatus_t gs_leftDriverLinkFlag,gs_rightDriverLinkFlag;//与驱动器的连接状态
static connectionStatus_t gs_leftDriverconnStatus = {LINK_STATUS_INIT,0,0,0,1000}; //设置超时时间为1000ms
static connectionStatus_t gs_rightDriverconnStatus = {LINK_STATUS_INIT,0,0,0,1000}; //设置超时时间为1000ms
//驱动器的健康状态：是否失联/是否有故障
static healthState_t gs_leftDriverHealthState,gs_rightDriverHealthState;
//驱动器故障码
static uint32_t gs_leftDriverErrorCode,gs_rightDriverErrorCode;
//标示着获取到驱动器返回信息
static uint8_t gs_getLeftDriverRet,gs_getRightDriverRet;


//编码计数
static encoderCnt_t gs_leftEncoderCnt,gs_rightEncoderCnt;
SemaphoreHandle_t gs_leftEncoderCntMutex,gs_rightEncoderCntMutex;
static uint32_t gs_getPosFlag = 0;//bit0 = 1表示获取到左轮位置，bit1 = 1表示获取到右轮位置

//驱动器电压电流
static float m_left_bat_voltage = 0, m_right_bat_voltage = 0;
static float m_left_axis_current = 0, m_right_axis_current = 0;

//编码器值读取成功回调函数
static void LeftEncodeCallBack(void* f_parm );
static void RightEncodeCallBack(void* f_parm );

static const Objdict_t m_objdict[] =
{
	{LEFT_ID,  INDEX_GET_BAT_VOLTAGE, SUBINDEX_GET_BAT_VOLTAGE, uint32, sizeof(uint32_t), RO, &m_left_bat_voltage, NULL },
	{RIGHT_ID, INDEX_GET_BAT_VOLTAGE, SUBINDEX_GET_BAT_VOLTAGE, uint32, sizeof(uint32_t), RO, &m_right_bat_voltage, NULL },
	{LEFT_ID,  INDEX_GET_Q_AXIS_CURRENT, SUBINDEX_GET_Q_AXIS_CURRENT, uint32, sizeof(uint32_t), RO, &m_left_axis_current, NULL },
	{RIGHT_ID, INDEX_GET_Q_AXIS_CURRENT, SUBINDEX_GET_Q_AXIS_CURRENT, uint32, sizeof(uint32_t), RO, &m_right_axis_current, NULL },
	{LEFT_ID,  INDEX_GET_CTR_ERROR_MASK1, SUBINDEX_GET_CTR_ERROR_MASK1, uint32, sizeof(uint32_t), RO, &gs_leftDriverErrorCode, NULL },
	{RIGHT_ID, INDEX_GET_CTR_ERROR_MASK1, SUBINDEX_GET_CTR_ERROR_MASK1, uint32, sizeof(uint32_t), RO, &gs_rightDriverErrorCode, NULL },
	{LEFT_ID,  INDEX_ENABLE, SUBINDEX_ENABLE, uint8, sizeof(uint8_t), RW, &gs_leftMotorDriverState.enable, NULL },
	{RIGHT_ID, INDEX_ENABLE, SUBINDEX_ENABLE, uint8, sizeof(uint8_t), RW, &gs_rightMotorDriverState.enable, NULL },
	{LEFT_ID,  INDEX_GET_ENCODER_CNT, SUBINDEX_GET_ENCODER_CNT, int32, sizeof(int32_t), RO, &gs_leftEncoderCnt.cnt, LeftEncodeCallBack },
	{RIGHT_ID, INDEX_GET_ENCODER_CNT, SUBINDEX_GET_ENCODER_CNT, int32, sizeof(int32_t), RO, &gs_rightEncoderCnt.cnt, RightEncodeCallBack },
};
static const uint8_t m_objdict_size = sizeof(m_objdict)/sizeof(Objdict_t);

/************************************************************************************************************************************
																静态函数声明
*************************************************************************************************************************************/

//static void InitDaNengDriverHW(void);
static int InitSpeedMutex(void);
static int InitSpeedToDriverMutex(void);

static void SetSpeed(int32_t f_leftSpeed, int32_t f_rightSpeed);
static FunctionalState GetLeftDriverEnableState(void);
static FunctionalState GetRightDriverEnableState(void);
static FunctionalState GetLeftDriverBrkState(void);
static FunctionalState GetRightDriverBrkState(void);
static void EnableDriver(uint8_t f_driverID,FunctionalState f_newState);
static void EnableLeftDriverBrk(FunctionalState f_newState);
static void EnableRightDriverBrk(FunctionalState f_newState);
static void ReadSpeedVal(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed);
static void SetSpeedToDaNeng(uint8_t f_driverID,int16_t f_speed);
static void CheckMotorDriverHealthState(void);
static void CheckMotorDriverLinkState(void);
static healthState_t GetRightDriverHealthState(void);
static healthState_t GetLeftDriverHealthState(void);
static void ReadErrorCode(uint8_t f_driverID);
static void ReadDriverEnableState(uint8_t f_driverID);

static int InitDecoderMutex(void);
static void DecoderRelativeCnt(void);
static void ReadEncoderCnt(uint8_t f_driverID);




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
static int InitSpeedToDriverMutex(void)
{
	gs_speedToDriverMutex = xSemaphoreCreateMutex();
	if(gs_speedToDriverMutex == NULL)
		return 1;
	return 0;
}


/************************************************************************************************************************************
																内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/
static void LeftEncodeCallBack(void* f_parm )
{
	gs_getPosFlag |= (0x01 << 0);
	gs_getLeftDriverRet = 1;
}
static void RightEncodeCallBack(void* f_parm )
{
	gs_getPosFlag |= (0x01 << 1);
	gs_getRightDriverRet = 1;
}
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
static void EnableDriver(uint8_t f_driverID,FunctionalState f_newState)
{
	uint8_t data = 0;
	
	data = (f_newState==ENABLE)?0x01:0x00;
	
	CanWriteSDO( f_driverID, INDEX_ENABLE, SUBINDEX_ENABLE, uint8, &data );
}

//读取驱动器使能状态
static void ReadDriverEnableState(uint8_t f_driverID)
{
	if( f_driverID == LEFT_ID )
	{
		CanReadSDO( LEFT_ID, INDEX_ENABLE, SUBINDEX_ENABLE, uint8, &gs_leftMotorDriverState.enable );
	}
	else if( f_driverID == RIGHT_ID )
	{
		CanReadSDO( RIGHT_ID, INDEX_ENABLE, SUBINDEX_ENABLE, uint8, &gs_rightMotorDriverState.enable );
	}
}

//刹车
static void EnableLeftDriverBrk(FunctionalState f_newState)
{
	gs_leftMotorDriverState.brk = f_newState;
}
static void EnableRightDriverBrk(FunctionalState f_newState)
{
	gs_rightMotorDriverState.brk = f_newState;
}


//读取驱动器告警信息
static void ReadErrorCode(uint8_t f_driverID)
{
	if( f_driverID == LEFT_ID )
	{
		CanReadSDO( LEFT_ID, INDEX_GET_CTR_ERROR_MASK1, SUBINDEX_GET_CTR_ERROR_MASK1, uint32, &gs_leftDriverErrorCode );
	}
	else if( f_driverID == RIGHT_ID )
	{
		CanReadSDO( RIGHT_ID, INDEX_GET_CTR_ERROR_MASK1, SUBINDEX_GET_CTR_ERROR_MASK1, uint32, &gs_rightDriverErrorCode );
	}
}

//将速度写入驱动器中
static void SetSpeedToDaNeng(uint8_t f_driverID,int16_t f_speed)
{
	f_speed *= 3;//额定转速3000r/min
	
//因为大能驱动器的问题，速度绝对值低于10的时候是不转的
	if(f_speed > 0 && f_speed < 10)
		f_speed = 10;
	else if(f_speed < 0 && f_speed > -10)
		f_speed = -10;
	
	CanWriteSDO( f_driverID, INDEX_SET_SPEED, SUBINDEX_SET_SPEED, int16, &f_speed );
}
//设置速度
static void SetSpeed(int32_t f_leftSpeed, int32_t f_rightSpeed)
{
	//1,设置速度到驱动器
	SetSpeedToDaNeng(LEFT_ID,(int16_t)f_leftSpeed);
	SetSpeedToDaNeng(RIGHT_ID,(int16_t)f_rightSpeed);
}


static void ReadEncoderCnt(uint8_t f_driverID)
{
	if( f_driverID == LEFT_ID )
	{
		CanReadSDO( LEFT_ID, INDEX_GET_ENCODER_CNT, SUBINDEX_GET_ENCODER_CNT, int32, &gs_leftEncoderCnt.cnt );
	}
	else if( f_driverID == RIGHT_ID )
	{
		CanReadSDO( RIGHT_ID, INDEX_GET_ENCODER_CNT, SUBINDEX_GET_ENCODER_CNT, int32, &gs_rightEncoderCnt.cnt );
	}
}


//0=成功，1=失败，-1=错误
static int InitDecoderMutex(void)
{
	gs_leftEncoderCntMutex = xSemaphoreCreateMutex();
	if(gs_leftEncoderCntMutex == NULL)
		return 1;
	
	gs_rightEncoderCntMutex = xSemaphoreCreateMutex();
	if(gs_rightEncoderCntMutex == NULL)
		return 1;

	return 0;
}
static void DecoderRelativeCnt(void)//处理编码器的相对计数,改函数应该在DecoderUnitCnt之后调用
{

	xSemaphoreTake(gs_leftEncoderCntMutex, portMAX_DELAY);
	gs_leftEncoderCnt.relativeCnt = gs_leftEncoderCnt.cnt - gs_leftEncoderCnt.lastCnt;
	//查看是否出现溢出清零
	if(gs_leftEncoderCnt.relativeCnt <= -THRESHOLD_OF_CNT_OVERFLOW)//正向溢出
		gs_leftEncoderCnt.relativeCnt = LIMIT_CLEAN_P - gs_leftEncoderCnt.lastCnt + gs_leftEncoderCnt.cnt;
	else if(gs_leftEncoderCnt.relativeCnt >= THRESHOLD_OF_CNT_OVERFLOW)
		gs_leftEncoderCnt.relativeCnt = LIMIT_CLEAN_N - gs_leftEncoderCnt.lastCnt + gs_leftEncoderCnt.cnt;


	gs_leftEncoderCnt.lastCnt  = gs_leftEncoderCnt.cnt;	
	
	xSemaphoreGive(gs_leftEncoderCntMutex);
	
	xSemaphoreTake(gs_rightEncoderCntMutex, portMAX_DELAY);
	gs_rightEncoderCnt.relativeCnt = gs_rightEncoderCnt.cnt - gs_rightEncoderCnt.lastCnt;
	
	//查看是否出现溢出清零
	if(gs_rightEncoderCnt.relativeCnt <= -THRESHOLD_OF_CNT_OVERFLOW)//正向溢出
		gs_rightEncoderCnt.relativeCnt = LIMIT_CLEAN_P - gs_rightEncoderCnt.lastCnt + gs_rightEncoderCnt.cnt;
	else if(gs_rightEncoderCnt.relativeCnt >= THRESHOLD_OF_CNT_OVERFLOW)
		gs_rightEncoderCnt.relativeCnt = LIMIT_CLEAN_N - gs_rightEncoderCnt.lastCnt + gs_rightEncoderCnt.cnt;

		
	gs_rightEncoderCnt.lastCnt  = gs_rightEncoderCnt.cnt;	
	xSemaphoreGive(gs_rightEncoderCntMutex);
}


static healthState_t GetLeftDriverHealthState(void)
{
	return gs_leftDriverHealthState;
}
static healthState_t GetRightDriverHealthState(void)
{
	return gs_rightDriverHealthState;
}

static void CheckMotorDriverHealthState(void)
{
	//未失联且无报警则认为健康
	if(gs_leftDriverLinkFlag == LINK_STATUS_LINKED && gs_leftMotorDriverState.alarm == 0)
		gs_leftDriverHealthState = HEALTHY;
	else
		gs_leftDriverHealthState = UNHEALTHY;

	if(gs_rightDriverLinkFlag == LINK_STATUS_LINKED && gs_rightMotorDriverState.alarm == 0)
		gs_rightDriverHealthState = HEALTHY;
	else
		gs_rightDriverHealthState = UNHEALTHY;
}
static void CheckMotorDriverLinkState(void)
{
	if(gs_getLeftDriverRet == 1)
	{
		ConnectedDev(&gs_leftDriverconnStatus);
		gs_getLeftDriverRet = 0;
		gs_leftDriverLinkFlag = LINK_STATUS_LINKED;
	}
	else
	{
		gs_leftDriverLinkFlag = CheckLinkState(&gs_leftDriverconnStatus);
	}
	
	if(gs_getRightDriverRet == 1)
	{
		ConnectedDev(&gs_rightDriverconnStatus);
		gs_getRightDriverRet = 0;
		gs_rightDriverLinkFlag = LINK_STATUS_LINKED;
	}
	else
	{
		gs_rightDriverLinkFlag = CheckLinkState(&gs_rightDriverconnStatus);
	}
	
	
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
//获取PC设置的速度值
void GetSpeedToDriver(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed)
{
	xSemaphoreTake(gs_speedToDriverMutex, portMAX_DELAY);
	*fp_leftSpeed = gs_leftSpeedToDriver;
	*fp_rightSpeed = gs_rightSpeedToDriver;
	xSemaphoreGive(gs_speedToDriverMutex);
}

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
			gs_cfgLeftBrk = DISABLE;
			gs_cfgRightBrk = ENABLE;
		break;
		default:
		break;
	}
}

void CfgDriverEnable(FunctionalState f_newState)
{
	gs_cfgEnable = f_newState;
}
uint8_t GetMotorDriverHealthState(void)
{
	uint8_t ret = 0;//1=健康
	if(GetRightDriverHealthState() == UNHEALTHY || GetLeftDriverHealthState == UNHEALTHY)
		ret = 0;
	else
		ret = 1;
	return ret;//两者都健康认为健康
}
uint8_t GetLeftMotorHealthState(void)
{
	uint8_t ret = 0;//1=健康
	ret = (GetLeftDriverHealthState() == HEALTHY)?1:0;
	return ret;
}
uint8_t GetRightMotorHealthState(void)
{
	uint8_t ret = 0;//1=健康
	ret = (GetRightDriverHealthState() == HEALTHY)?1:0;
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
	xSemaphoreTake(gs_leftEncoderCntMutex, portMAX_DELAY);
    *fp_cnt = gs_leftEncoderCnt.lastCnt; //这里返回上次的值是因为相对计数是与上次的值对应的
	*fp_relativeCnt = gs_leftEncoderCnt.relativeCnt; 
    xSemaphoreGive(gs_leftEncoderCntMutex);
}
__weak void GetRightDecoderInfo(int32_t *fp_cnt, int32_t *fp_relativeCnt)
{
	xSemaphoreTake(gs_rightEncoderCntMutex, portMAX_DELAY);
    *fp_cnt = -gs_rightEncoderCnt.lastCnt; //这里返回上次的值是因为相对计数是与上次的值对应的
	*fp_relativeCnt = -gs_rightEncoderCnt.relativeCnt; 
    xSemaphoreGive(gs_rightEncoderCntMutex);
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


void ReadMotorStatus( int32_t speed[], int32_t count[], int32_t dcount[])
{
    speed[0] = gs_leftSpeed; 
	speed[1] = gs_rightSpeed; 
    
    count[0] = gs_leftEncoderCnt.lastCnt;
    count[1] = -gs_rightEncoderCnt.lastCnt;
    
    dcount[0] = gs_leftEncoderCnt.relativeCnt;
    dcount[1] = -gs_rightEncoderCnt.relativeCnt;
}

/************************************************************************************************************************************
												任务+对外的硬件初始化+对外的软件初始化
*************************************************************************************************************************************/

/*
*********************************************************************************************************
*	函 数 名: MotorDriverTask
*	功能说明: 	电机驱动任务，处理具体的控制，对应BLDC驱动器来说可以不做任何事情
*				这个任务即使不做任何事情也需要创建,或者使用宏定义屏蔽
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 高 
*********************************************************************************************************
*/
void MotorDriverTask(void const *pvParameters)
{
	uint8_t readEncoderCntTimes = 0;//记录发送读取编码计数的次数，
	TickType_t lastWakeTime;
	int32_t leftSpeed,rightSpeed;
	const TickType_t c_frequency = pdMS_TO_TICKS(2);
//	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
	//读编码器值
	TickType_t readCntTime = 0;
	const TickType_t c_readCntFrequency = pdMS_TO_TICKS(20);
	//设置速度
	TickType_t setSpeedTime = 0;
	const TickType_t c_setSpeedFrequency = pdMS_TO_TICKS(20);
	//判断并设置使能
	TickType_t setEnableTime = 0;
	const TickType_t c_setEnableFrequency = pdMS_TO_TICKS(500);
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
	
    while(1)
    {
		/* 喂狗 */
//		FeedDog(bitPos);
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelayUntil(&lastWakeTime, c_frequency);
      	//获取编码计数
		if(readCntTime + c_readCntFrequency < xTaskGetTickCount())
		{
			ReadEncoderCnt(LEFT_ID);
			ReadEncoderCnt(RIGHT_ID);
			readEncoderCntTimes++;
			gs_getPosFlag = 0;
			readCntTime = xTaskGetTickCount();
		}
		//发送n此后收到了左右位置或者发送了n+1次
		if(((readEncoderCntTimes == 1) && (gs_getPosFlag == 0x03)) || (readEncoderCntTimes > 1))
		{
			DecoderRelativeCnt();//根据总值计算相对值
			readEncoderCntTimes = 0;
		}
		//1,获取速度
		ReadSpeedVal(&leftSpeed,&rightSpeed);
		//2,检查连接状态和健康状态
		CheckMotorDriverLinkState();
		CheckMotorDriverHealthState();
		//3,判断使能状态和错误状态,频率不能太高
		if(setEnableTime + c_setEnableFrequency < xTaskGetTickCount())
		{
			ReadErrorCode(LEFT_ID);
			ReadErrorCode(RIGHT_ID);
			ReadDriverEnableState(LEFT_ID);
			ReadDriverEnableState(RIGHT_ID);
			if(gs_cfgEnable != GetLeftDriverEnableState())
			{
				EnableDriver(LEFT_ID,gs_cfgEnable);
			}
			if(gs_cfgEnable != GetRightDriverEnableState())
			{
				EnableDriver(RIGHT_ID,gs_cfgEnable);
			}
			setEnableTime = xTaskGetTickCount();
		}
		//4,设置刹车状态
		if(gs_cfgLeftBrk != GetLeftDriverBrkState())
			EnableLeftDriverBrk(gs_cfgLeftBrk);
		if(gs_cfgRightBrk != GetRightDriverBrkState())
			EnableRightDriverBrk(gs_cfgRightBrk);
			
		//5,根据驱动器的刹车状态、使能状态、健康状态判断是否需要对速度清零
		if(GetLeftDriverBrkState() == ENABLE || GetLeftDriverEnableState() == DISABLE)
			leftSpeed = 0;
		if(GetRightDriverBrkState() == ENABLE || GetRightDriverEnableState() == DISABLE)
			rightSpeed = 0;
		if(GetMotorDriverHealthState() == 0 )
			leftSpeed = rightSpeed = 0;
        
		//6,控制运动,因为是使用can通信，所以速度的设置频率不能太高
		if(setSpeedTime + c_setSpeedFrequency < xTaskGetTickCount())
		{
			xSemaphoreTake(gs_speedToDriverMutex, portMAX_DELAY);
			gs_leftSpeedToDriver = leftSpeed;
			gs_rightSpeedToDriver = -rightSpeed;
			xSemaphoreGive(gs_speedToDriverMutex);
			
			SetSpeed(gs_leftSpeedToDriver, gs_rightSpeedToDriver);

			setSpeedTime = xTaskGetTickCount();
		}
		g_thread_call_count[thread_num]++;
    }
}



int InitSoftwareMotorDriver(void)
{
	if( InitSpeedMutex() != 0)
		return 1;
	if( InitSpeedToDriverMutex() != 0)
		return 1;
	ObjdictLoad( m_objdict, m_objdict_size );
	return 0;
}


int InitSoftwareDecoder(void)
{
	return InitDecoderMutex();
}
