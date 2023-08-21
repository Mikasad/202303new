//标准头文件
#include <stdlib.h>
//stm32头文件
#include "main.h"
//APP头文件
#include "motor_driver.h"
//#include "can_task.h"
#include "security_state.h"
#include "objdict.h"
#include "normal_io.h"
#include "power_ctr.h"
#include "main.h"
#include "iwdg_task.h"
#include "hc_can.h"

/*
*********************************************************************************************************
*
*	模块名称 :和利时驱动器控制模块
*	文件名称 : HollySys.c
*	版    本 : 	V1.0.0
*	说    明 : 
*			1，使用CAN通信
*			
*	修改记录 :
*		版本号  		日期        作者     说明
*       V1.0.0    2019-12-11  ypc
*	外设资源占用：
*		CAN:CAN1
*********************************************************************************************************
*/

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/
#define ENCODER_MAX_ERROR_NUM  4
#define HELISHI_DS102 (1) //和利时单轴驱动器
#define HELISHI_DS202 (2) //和利时双轴驱动器

//#define WHEEL_MOTOR_DRIVE    HELISHI_DS102	//左右轮驱动器

#define LEFT_ID	0x03
#define RIGHT_ID 0X04


#define INDEX_102_LEFT_ERROR_MSG    0x3012
#define INDEX_102_RIGHT_ERROR_MSG    0x3012
#define SUBINDEX_102_ERROR_MSG    0x00

#define INDEX_102_LEFT_LOAD    0x3002
#define INDEX_102_RIGHT_LOAD    0x3002
#define SUBINDEX_102_LOAD    0x00

#define INDEX_102_LEFT_TEMPERATURE    0x3016
#define INDEX_102_RIGHT_TEMPERATURE    0x3016
#define SUBINDEX_102_TEMPERATURE    0x00

#define INDEX_102_LEFT_MAX_CURRENT    0x202B
#define INDEX_102_RIGHT_MAX_CURRENT    0x202B
#define SUBINDEX_102_MAX_CURRENT    0x00

#define INDEX_102_VERSION    0x3008
#define SUBINDEX_102_VERSION  0x00

#define INDEX_102_CURLIMIT  0x202A //电流限制方式
#define SUBINDEX_102_CURLIMIT  0x00



#define INDEX_202_LEFT_ERROR_MSG  0x5012
#define INDEX_202_RIGHT_ERROR_MSG  0x5112
#define SUBINDEX_202_ERROR_MSG 0x00

#define INDEX_202_RIGHT_LOAD    0x5102
#define INDEX_202_LEFT_LOAD    0x5002
#define SUBINDEX_202_LOAD    0x00

#define INDEX_202_LEFT_TEMPERATURE    0x501B
#define INDEX_202_RIGHT_TEMPERATURE    0x511B
#define SUBINDEX_202_TEMPERATURE    0x00

#define INDEX_202_LEFT_MAX_CURRENT    0x242A
#define INDEX_202_RIGHT_MAX_CURRENT    0x342A
#define SUBINDEX_202_MAX_CURRENT    0x00

#define INDEX_202_VERSION    0x5018
#define SUBINDEX_202_VERSION  0x00

#define INDEX_202_LEFT_VELOCITY_1P  0x2300 //左驱动器速度环第一比例增益
#define INDEX_202_RIGHT_VELOCITY_1P 0x3300 //右驱动器速度环第一比例增益
#define SUBINDEX_202_VELOCITY_1P    0x00

#define INDEX_202_LEFT_VELOCITY_1I  0x2301 //读取左速度环第一积分增益
#define INDEX_202_RIGHT_VELOCITY_1I 0x3301 //读取右速度环第一积分增益
#define SUBINDEX_202_VELOCITY_1I  0x00

#define INDEX_202_LEFTCURLIMIT  0x2422
#define INDEX_202_RIGHTCURLIMIT  0x3422
#define SUBINDEX_202_CURLIMIT  0x00

#define INDEX_ENCODER_CNT  0x6063
#define SUBINDEX_ENCODER_CNT 0x00

#define INDEX_STATUS  0x6041
#define SUBINDEX_STATUS 0x00

#define INDEX_CTRLMODE  0x6060
#define SUBINDEX_CTRLMODE  0x00






/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/

typedef struct
{
	int32_t relativeCnt;//相对计数
	int32_t lastCnt;//上次的总数
	int32_t cnt;//当前的总数
    int32_t short_cnt;//上报给上位机的count值
    uint32_t tick; //读取到编码器数据时的系统tick
    uint8_t error_count;
}encoderCnt_t;

typedef struct{
    uint32_t status; //驱动器状态位
    uint32_t error_code;//驱动器告警信息
    uint32_t mcu_error_code;//根据error_code提取出的具体告警类型，通过mcu_error_code上传
    uint16_t load_factor;//负载率
    int16_t current;//根据负载率算出的当前电机电流，瞬时值
    int16_t temperature;//电机当前温度
    uint16_t max_current;//驱动器最大输出电流
    uint16_t velocity_1P;//动器速度环第一比例增益
    uint16_t velocity_1I;//速度环第一积分增益
    uint8_t enable;	//使能
	uint8_t brk; 		//刹车
	uint8_t alarm;	//电机是否有告警
    uint8_t alarm_current;//过流告警标志
    uint8_t alarm_temperature;//过温告警标志
	uint16_t cur_limit;
    int8_t ctrl_mode;
    int8_t driver_mode;
    uint8_t control_mode;
}motorDriverState_t;

typedef struct
{
    int16_t temperature;//摄氏度
    int16_t current;//0.1A
    uint16_t current_incharge;//0.1A
}MotorProtect_t;

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
static en_functional_state_t gs_cfgLeftBrk = DISABLE;
static en_functional_state_t gs_cfgRightBrk = DISABLE;
static en_functional_state_t gs_cfgEnable= DISABLE;
static uint8_t m_control_mode = MODE_SPEED;//MODE_TORQUE; //0速度模式，1转矩模式
//电机驱动器状态
static motorDriverState_t gs_leftMotorDriverState,gs_rightMotorDriverState;
//左右的速度
static int32_t gs_leftSpeedToDriver,gs_rightSpeedToDriver;
static SemaphoreHandle_t gs_speedToDriverMutex;
static int32_t gs_leftSpeed,gs_rightSpeed;
static SemaphoreHandle_t gs_speedMutex;
//与驱动器的的连接状态，规定时间内能够收到心跳或者其他信息则认为连接正常
static ConnectionStatus_t gs_leftDriverconnStatus = {5000,1000}; //设置超时时间为1000ms
static ConnectionStatus_t gs_rightDriverconnStatus = {5000,1000}; //设置超时时间为1000ms
//驱动器的健康状态：是否失联/是否有故障
static healthState_t gs_leftDriverHealthState,gs_rightDriverHealthState;
//标示着获取到驱动器返回信息
static uint8_t gs_getLeftDriverRet,gs_getRightDriverRet;


//编码计数
static encoderCnt_t gs_leftEncoderCnt,gs_rightEncoderCnt;
SemaphoreHandle_t gs_leftEncoderCntMutex,gs_rightEncoderCntMutex;
static uint32_t gs_getPosFlag = 0;//bit0 = 1表示获取到左轮位置，bit1 = 1表示获取到右轮位置

//左右轮电流和温度保护值
static MotorProtect_t m_left_protect = {75, 250, 50};
static MotorProtect_t m_right_protect = {75, 250, 50};

static uint8_t m_motor_alarm = 0; //驱动器告警标志

static uint32_t m_102_version = 0;//ds102版本号
static uint32_t m_202_version = 0;//ds202版本号
static uint8_t m_driver_type = 0; // 0=未确定；1=一拖一；2=一拖二
static uint8_t m_driver_type_update_flag = 0;
static uint16_t m_velocity_1P_incharge = 50;//第一速度环p参数，充电时用，方式车出桩
static uint16_t m_velocity_1I_incharge = 1;//第一速度环i参数

static uint8_t m_clear_ds_alarm = 0;//记录上位机下发的清告警命令
static uint32_t m_temperature_check_delay = 1;//用于开机延时检测驱动器过温，

//编码器值读取成功回调函数
static void LeftEncodeCallBack(void* f_parm );
static void RightEncodeCallBack(void* f_parm );
static void CalculateLeftCurrent(void* f_parm );
static void CalculateRightCurrent(void* f_parm );

static Objdict_t m_objdict[] =
{
	{LEFT_ID,  INDEX_202_LEFT_ERROR_MSG, SUBINDEX_202_ERROR_MSG, uint32, sizeof(uint32_t), RO, &gs_leftMotorDriverState.error_code, NULL },
	{RIGHT_ID, INDEX_202_RIGHT_ERROR_MSG, SUBINDEX_202_ERROR_MSG, uint32, sizeof(uint32_t), RO, &gs_rightMotorDriverState.error_code, NULL },
	{LEFT_ID,  INDEX_STATUS, SUBINDEX_STATUS, uint32, sizeof(uint32_t), RW, &gs_leftMotorDriverState.status, NULL },
	{RIGHT_ID, INDEX_STATUS, SUBINDEX_STATUS, uint32, sizeof(uint32_t), RW, &gs_rightMotorDriverState.status, NULL },
	{LEFT_ID,  INDEX_ENCODER_CNT, SUBINDEX_ENCODER_CNT, int32, sizeof(int32_t), RO, &gs_leftEncoderCnt.cnt, LeftEncodeCallBack },
	{RIGHT_ID, INDEX_ENCODER_CNT, SUBINDEX_ENCODER_CNT, int32, sizeof(int32_t), RO, &gs_rightEncoderCnt.cnt, RightEncodeCallBack },
    {LEFT_ID,  INDEX_202_LEFT_LOAD, SUBINDEX_202_LOAD, uint16, sizeof(uint16_t), RO, &gs_leftMotorDriverState.load_factor, CalculateLeftCurrent },
    {RIGHT_ID, INDEX_202_RIGHT_LOAD, SUBINDEX_202_LOAD, uint16, sizeof(uint16_t), RO, &gs_rightMotorDriverState.load_factor, CalculateRightCurrent },
    {LEFT_ID,  INDEX_202_LEFT_TEMPERATURE, SUBINDEX_202_TEMPERATURE, int16, sizeof(int16_t), RO, &gs_leftMotorDriverState.temperature, NULL },
    {RIGHT_ID, INDEX_202_RIGHT_TEMPERATURE, SUBINDEX_202_TEMPERATURE, int16, sizeof(int16_t), RO, &gs_rightMotorDriverState.temperature, NULL },
    {LEFT_ID,  INDEX_202_LEFT_MAX_CURRENT, SUBINDEX_202_MAX_CURRENT, int16, sizeof(int16_t), RO, &gs_leftMotorDriverState.max_current, NULL },
    {RIGHT_ID, INDEX_202_RIGHT_MAX_CURRENT, SUBINDEX_202_MAX_CURRENT, int16, sizeof(int16_t), RO, &gs_rightMotorDriverState.max_current, NULL },
    {LEFT_ID,  INDEX_102_VERSION, SUBINDEX_102_VERSION, uint32, sizeof(uint32_t), RW, &m_102_version, NULL },
    {LEFT_ID,  INDEX_202_VERSION, SUBINDEX_202_VERSION, uint32, sizeof(uint32_t), RW, &m_202_version, NULL },
    {LEFT_ID,  INDEX_202_LEFT_VELOCITY_1P, SUBINDEX_202_VELOCITY_1P, uint16, sizeof(uint16_t), RW, &gs_leftMotorDriverState.velocity_1P, NULL },
    {RIGHT_ID, INDEX_202_RIGHT_VELOCITY_1P, SUBINDEX_202_VELOCITY_1P, uint16, sizeof(uint16_t), RW, &gs_rightMotorDriverState.velocity_1P, NULL },
    {LEFT_ID,  INDEX_202_LEFT_VELOCITY_1I, SUBINDEX_202_VELOCITY_1I, uint16, sizeof(uint16_t), RW, &gs_leftMotorDriverState.velocity_1I, NULL },
    {RIGHT_ID, INDEX_202_RIGHT_VELOCITY_1I, SUBINDEX_202_VELOCITY_1I, uint16, sizeof(uint16_t), RW, &gs_rightMotorDriverState.velocity_1I, NULL },
    {LEFT_ID,  INDEX_202_LEFTCURLIMIT, SUBINDEX_202_CURLIMIT, uint16, sizeof(uint16_t), RW, &gs_leftMotorDriverState.cur_limit, NULL },
    {RIGHT_ID, INDEX_202_RIGHTCURLIMIT, SUBINDEX_202_CURLIMIT, uint16, sizeof(uint16_t), RW, &gs_rightMotorDriverState.cur_limit, NULL },
    {LEFT_ID,  INDEX_CTRLMODE, SUBINDEX_CTRLMODE, int8, sizeof(int8_t), RW, &gs_leftMotorDriverState.ctrl_mode, NULL },
    {RIGHT_ID, INDEX_CTRLMODE, SUBINDEX_CTRLMODE, int8, sizeof(int8_t), RW, &gs_rightMotorDriverState.ctrl_mode, NULL },
};
static const Objdict_t m_objdict_102[] =
{
	{LEFT_ID,  INDEX_102_LEFT_ERROR_MSG, SUBINDEX_102_ERROR_MSG, uint32, sizeof(uint32_t), RO, &gs_leftMotorDriverState.error_code, NULL },
	{RIGHT_ID, INDEX_102_RIGHT_ERROR_MSG, SUBINDEX_102_ERROR_MSG, uint32, sizeof(uint32_t), RO, &gs_rightMotorDriverState.error_code, NULL },
	{LEFT_ID,  INDEX_STATUS, SUBINDEX_STATUS, uint32, sizeof(uint32_t), RW, &gs_leftMotorDriverState.status, NULL },
	{RIGHT_ID, INDEX_STATUS, SUBINDEX_STATUS, uint32, sizeof(uint32_t), RW, &gs_rightMotorDriverState.status, NULL },
	{LEFT_ID,  INDEX_ENCODER_CNT, SUBINDEX_ENCODER_CNT, int32, sizeof(int32_t), RO, &gs_leftEncoderCnt.cnt, LeftEncodeCallBack },
	{RIGHT_ID, INDEX_ENCODER_CNT, SUBINDEX_ENCODER_CNT, int32, sizeof(int32_t), RO, &gs_rightEncoderCnt.cnt, RightEncodeCallBack },
    {LEFT_ID,  INDEX_102_LEFT_LOAD, SUBINDEX_102_LOAD, uint16, sizeof(uint16_t), RO, &gs_leftMotorDriverState.load_factor, CalculateLeftCurrent },
    {RIGHT_ID, INDEX_102_RIGHT_LOAD, SUBINDEX_102_LOAD, uint16, sizeof(uint16_t), RO, &gs_rightMotorDriverState.load_factor, CalculateRightCurrent },
    {LEFT_ID,  INDEX_102_LEFT_TEMPERATURE, SUBINDEX_102_TEMPERATURE, int16, sizeof(int16_t), RO, &gs_leftMotorDriverState.temperature, NULL },
    {RIGHT_ID, INDEX_102_RIGHT_TEMPERATURE, SUBINDEX_102_TEMPERATURE, int16, sizeof(int16_t), RO, &gs_rightMotorDriverState.temperature, NULL },
    {LEFT_ID,  INDEX_102_LEFT_MAX_CURRENT, SUBINDEX_102_MAX_CURRENT, int16, sizeof(int16_t), RO, &gs_leftMotorDriverState.max_current, NULL },
    {RIGHT_ID, INDEX_102_RIGHT_MAX_CURRENT, SUBINDEX_102_MAX_CURRENT, int16, sizeof(int16_t), RO, &gs_rightMotorDriverState.max_current, NULL },
    {LEFT_ID,  INDEX_102_VERSION, SUBINDEX_102_VERSION, uint32, sizeof(uint32_t), RW, &m_102_version, NULL },
    {LEFT_ID,  INDEX_202_VERSION, SUBINDEX_202_VERSION, uint32, sizeof(uint32_t), RW, &m_202_version, NULL },
    {LEFT_ID,  INDEX_102_CURLIMIT, SUBINDEX_102_CURLIMIT, uint16, sizeof(uint16_t), RW, &gs_leftMotorDriverState.cur_limit, NULL },
    {RIGHT_ID, INDEX_102_CURLIMIT, SUBINDEX_102_CURLIMIT, uint16, sizeof(uint16_t), RW, &gs_rightMotorDriverState.cur_limit, NULL },
    {LEFT_ID,  INDEX_CTRLMODE, SUBINDEX_CTRLMODE, int8, sizeof(int8_t), RW, &gs_leftMotorDriverState.ctrl_mode, NULL },
    {RIGHT_ID, INDEX_CTRLMODE, SUBINDEX_CTRLMODE, int8, sizeof(int8_t), RW, &gs_rightMotorDriverState.ctrl_mode, NULL },
};
static const uint8_t m_objdict_size = sizeof(m_objdict)/sizeof(Objdict_t);

/************************************************************************************************************************************
																静态函数声明
*************************************************************************************************************************************/

//static void InitDaNengDriverHW(void);
static int InitSpeedMutex(void);
static int InitSpeedToDriverMutex(void);

static void SetSpeed(int32_t f_leftSpeed, int32_t f_rightSpeed);
static en_functional_state_t GetLeftDriverEnableState(void);
static en_functional_state_t GetRightDriverEnableState(void);
static en_functional_state_t GetLeftDriverBrkState(void);
static en_functional_state_t GetRightDriverBrkState(void);
static void EnableDriver(uint8_t f_driverID,en_functional_state_t f_newState);
static void EnableLeftDriverBrk(en_functional_state_t f_newState);
static void EnableRightDriverBrk(en_functional_state_t f_newState);
static void ReadSpeedVal(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed);
static void SetSpeedToMotor(uint8_t f_driverID,int32_t f_speed);
static void CheckMotorDriverHealthState(void);
static void CheckMotorDriverLinkState(void);
static healthState_t GetRightDriverHealthState(void);
static healthState_t GetLeftDriverHealthState(void);

static int InitDecoderMutex(void);
static void DecoderRelativeCnt(void);
static void ReadEncoderCnt(uint8_t f_driverID);

static void DS20SetSpeedMode(uint8_t f_id);
static void DS20SetTorqueProfileMode( uint8_t f_id);
static void DS102MotorError(uint32_t src, uint32_t *prt);
static void DS202MotorError(uint32_t src, uint32_t *prt);


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

 
static int Can1Send( uint32_t f_id, uint8_t f_value[], uint8_t f_len )
{
	can_standard_send(CAN1_UNIT, f_id, f_value, f_len);
	return 0;
}

static void set_motor_stall_protection(void)
{
    uint16_t data = 0;
    if (m_driver_type == HELISHI_DS202)
    {
        data = 1;
        CanWriteSDO( LEFT_ID, 0x450A, 0x00, uint16, &data );
        data = 50; /* 失速报警等级 */
        CanWriteSDO( LEFT_ID, 0x2327, 0x00, uint16, &data );
        CanWriteSDO( LEFT_ID, 0x3327, 0x00, uint16, &data );
        data = 0;
        CanWriteSDO( LEFT_ID, 0x450A, 0x00, uint16, &data );
    }
}

/************************************************************************************************************************************
                                                     内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/
/**************************************************************************************
* 作者/公司  :杨鹏程/高仙
* 函数介绍   :和利时电机初始化函数
* 输入参数   :bit0=1表示左电机，bit1=1表示右电机
* 输出参数   :
* 返回值     :
****************************************************************************************/
static void MotorInit(uint8_t f_motor)
{
    uint8_t temp[8] = {0,0,0,0,0,0,0,0};
    temp[0] = 0x01;
    Can1Send( 0x00, temp, 8 );

    vTaskDelay(50);
    
    if(m_driver_type == HELISHI_DS102)
    {
        uint32_t data;
        uint16_t obj_102_size = sizeof(m_objdict_102)/sizeof(Objdict_t);
        for( int j=0;j<obj_102_size;j++)
        {
            m_objdict[j] = m_objdict_102[j];
        }
        //下面三个写操作设置最大输出电流
        if( f_motor & 0x01 )
        {
            data = 1;
            CanWriteSDO( LEFT_ID, 0x202A, 0x00, uint16, &data );
            data = 100;//额定电流的100% * 1.414
            CanWriteSDO( LEFT_ID, 0x202B, 0x00, uint16, &data );
            CanWriteSDO( LEFT_ID, 0x202C, 0x00, uint16, &data );
            data = 1800;
            CanWriteSDO( LEFT_ID, 0x2005, 0x00, int16, &data );//设置当前电机的额定电流
            
            gs_leftMotorDriverState.cur_limit = 0;
            gs_leftMotorDriverState.ctrl_mode = 0;
            
            vTaskDelay(2);
        }
        if( f_motor & 0x02 )
        {
            data = 1;
            CanWriteSDO( RIGHT_ID, 0x202A, 0x00, uint16, &data );
            data = 100;//额定电流的100% * 1.414
            CanWriteSDO( RIGHT_ID, 0x202B, 0x00, uint16, &data );
            CanWriteSDO( RIGHT_ID, 0x202C, 0x00, uint16, &data );
            data = 1800;
            CanWriteSDO( RIGHT_ID, 0x2005, 0x00, int16, &data );
            
            gs_rightMotorDriverState.cur_limit = 0;
            gs_rightMotorDriverState.ctrl_mode = 0;
            
            vTaskDelay(2);
        }
    }
    else if (m_driver_type == HELISHI_DS202)
    {
        uint16_t data = 0;
        if( f_motor & 0x01 )
        {
            temp[1] = LEFT_ID;
            Can1Send( 0x00, temp, 8 );
            vTaskDelay(2);
            data = 0;
            CanWriteSDO( LEFT_ID, 0x2422, 0x00, uint16, &data );//转矩限制使能，0电流限流方式，1转矩限流方式
            gs_leftMotorDriverState.cur_limit = 0;
            gs_leftMotorDriverState.ctrl_mode = 0;
        }
        if( f_motor & 0x02 )
        {
            temp[1] = RIGHT_ID;
            Can1Send( 0x00, temp, 8 );
            vTaskDelay(2);
            data = 0;
            CanWriteSDO( RIGHT_ID, 0x3422, 0x00, uint16, &data );
            gs_rightMotorDriverState.cur_limit = 0;
            gs_rightMotorDriverState.ctrl_mode = 0;
        }
        /* 修改失速保护参数 */
        set_motor_stall_protection();
    }
    if( m_driver_type )
    {
        if( f_motor & 0x01 )
        {
            if( m_control_mode==MODE_SPEED)
                DS20SetSpeedMode(LEFT_ID); //设置为速度模式
            else if( m_control_mode==MODE_TORQUE)
                DS20SetTorqueProfileMode(LEFT_ID); //设置为转矩模式
            vTaskDelay(2);
            EnableDriver( LEFT_ID, gs_cfgEnable ); //使能电机
            vTaskDelay(2);
        }
        if( f_motor & 0x02 )
        {
            if( m_control_mode==MODE_SPEED)
                DS20SetSpeedMode(RIGHT_ID);
            else if( m_control_mode==MODE_TORQUE)
                DS20SetTorqueProfileMode(RIGHT_ID); //设置为转矩模式
            vTaskDelay(2);
            EnableDriver( RIGHT_ID, gs_cfgEnable ); //使能电机
            vTaskDelay(2);
        }
    }
}
//将驱动器配置为速度模式
static void DS20SetSpeedMode(uint8_t f_id)
{
    static uint8_t s_first_flag = 0x11;
    uint32_t data = 0;
	
    data = 0x03;
    CanWriteSDO( f_id, 0x6060, 0x00, int8, &data );

    if(s_first_flag)
    {
        data = 80;
        CanWriteSDO( f_id, 0x6084, 0x00, int32, &data );
        
        data = 80;
        CanWriteSDO( f_id, 0x6083, 0x00, int32, &data );
    }

    data = 0;
    CanWriteSDO( f_id, 0x60FF, 0x00, int32, &data );
	
    if( f_id == LEFT_ID )
    {
        s_first_flag = s_first_flag&0x10;
        gs_leftMotorDriverState.control_mode = MODE_SPEED;
    }
    else if( f_id == RIGHT_ID )
    {
        s_first_flag = s_first_flag&0x01;
        gs_rightMotorDriverState.control_mode = MODE_SPEED;
    }
}
//将驱动器配置为转矩模式
static void DS20SetTorqueProfileMode( uint8_t f_id)
{
    static uint8_t s_first_flag = 0x11;
    uint32_t data = 0;
    
    data = 0;
    CanWriteSDO( f_id, 0x6071, 0x00, int16, &data );
    
    data = 0x04;
    CanWriteSDO( f_id, 0x6060, 0x00, int8, &data );

    if(s_first_flag)
    {
        data = 1000;
        CanWriteSDO( f_id, 0x6087, 0x00, int32, &data );
    }
    
    if( f_id == LEFT_ID )
    {
        s_first_flag = s_first_flag&0x10;
        gs_leftMotorDriverState.control_mode = MODE_TORQUE;
    }
    else if( f_id == RIGHT_ID )
    {
        s_first_flag = s_first_flag&0x01;
        gs_rightMotorDriverState.control_mode = MODE_TORQUE;
    }
}

static void WriteControlMode(uint8_t f_id, uint8_t f_mode)
{
    if( f_mode==MODE_SPEED)
        DS20SetSpeedMode(f_id );
    else if( f_mode==MODE_TORQUE)
        DS20SetTorqueProfileMode(f_id);
}
uint32_t gs_left_encoder_tick = 0;
uint32_t gs_left_encoder_cnt = 0;
uint32_t gs_right_encoder_tick = 0;
uint32_t gs_right_encoder_cnt = 0;
//左右编码器读取回调函数
static void LeftEncodeCallBack(void* f_parm )
{
	gs_getPosFlag |= (0x01 << 0);
	gs_getLeftDriverRet = 1;
    gs_leftEncoderCnt.tick = xTaskGetTickCount();
	gs_left_encoder_tick = gs_leftEncoderCnt.tick;
	gs_left_encoder_cnt = gs_leftEncoderCnt.cnt;
    if( gs_leftEncoderCnt.error_count<ENCODER_MAX_ERROR_NUM)
        gs_leftEncoderCnt.error_count=0;
    else
        gs_leftEncoderCnt.error_count=ENCODER_MAX_ERROR_NUM*2;
}
static void RightEncodeCallBack(void* f_parm )
{
	gs_getPosFlag |= (0x01 << 1);
	gs_getRightDriverRet = 1;
    gs_rightEncoderCnt.tick = xTaskGetTickCount();
	gs_right_encoder_tick = gs_rightEncoderCnt.tick;
	gs_right_encoder_cnt = gs_rightEncoderCnt.cnt;
    if( gs_rightEncoderCnt.error_count<ENCODER_MAX_ERROR_NUM )
        gs_rightEncoderCnt.error_count = 0;
    else
        gs_rightEncoderCnt.error_count = ENCODER_MAX_ERROR_NUM*2;
}
//负载读取回调函数
static void CalculateLeftCurrent(void* f_parm )
{
    gs_leftMotorDriverState.current = (short int)((float)gs_leftMotorDriverState.load_factor * 0.25452f );//180.0f * 1.414f / 1000);//单位0.1A
}
static void CalculateRightCurrent(void* f_parm )
{
    gs_rightMotorDriverState.current = (short int)((float)gs_rightMotorDriverState.load_factor * 0.25452f );//180.0f * 1.414f / 1000);//单位0.1A
}
//读取上位机下发的速度值
static void ReadSpeedVal(int32_t *fp_leftSpeed,int32_t *fp_rightSpeed)
{
	xSemaphoreTake(gs_speedMutex, portMAX_DELAY);
	*fp_leftSpeed = gs_leftSpeed; 
	*fp_rightSpeed = gs_rightSpeed; 
	xSemaphoreGive(gs_speedMutex);
}

static en_functional_state_t GetLeftDriverEnableState(void)
{
	return (en_functional_state_t)gs_leftMotorDriverState.enable;
}
static en_functional_state_t GetRightDriverEnableState(void)
{
	return (en_functional_state_t)gs_rightMotorDriverState.enable;
}
static en_functional_state_t GetLeftDriverBrkState(void)
{
	return (en_functional_state_t)gs_leftMotorDriverState.brk;
}
static en_functional_state_t GetRightDriverBrkState(void)
{
	return (en_functional_state_t)gs_rightMotorDriverState.brk;
}

//使能/失能驱动器
static void EnableDriver(uint8_t f_driverID,en_functional_state_t f_newState)
{
	uint32_t data = 0;
	
	if(f_newState==ENABLE)
    {
        data = 0x06;
        CanWriteSDO( f_driverID, 0x6040, 0x00, int16, &data );
    
        data = 0x07;
        CanWriteSDO( f_driverID, 0x6040, 0x00, int16, &data );
    
        data = 0x0F;
        CanWriteSDO( f_driverID, 0x6040, 0x00, int16, &data );
        
        if( f_driverID == LEFT_ID )
        {
            gs_leftMotorDriverState.enable = ENABLE;
        }
        else if( f_driverID == RIGHT_ID )
        {
            gs_rightMotorDriverState.enable = ENABLE;
        }
    }
    else
    {
		data = 0x06;
        CanWriteSDO( f_driverID, 0x6040, 0x00, int16, &data );
		
        data = 0x07;
        CanWriteSDO( f_driverID, 0x6040, 0x00, int16, &data );
        
        if( f_driverID == LEFT_ID )
        {
            gs_leftMotorDriverState.enable = DISABLE;
        }
        else if( f_driverID == RIGHT_ID )
        {
            gs_rightMotorDriverState.enable = DISABLE;
        }
    }
}
//驱动器复位 23 03 46 00 01 00 00 00
static void CleanMotorError(uint8_t f_driverID)
{
	uint32_t data = 0x01;
	CanWriteSDO( f_driverID, 0x4603, 0x00, uint32, &data );
	MotorInit(0x03);
}
//刹车
static void EnableLeftDriverBrk(en_functional_state_t f_newState)
{
	gs_leftMotorDriverState.brk = f_newState;
}
static void EnableRightDriverBrk(en_functional_state_t f_newState)
{
	gs_rightMotorDriverState.brk = f_newState;
}

static void ReadDriverStatus(void)
{
    static uint8_t s_idx_read = 0;
    
    s_idx_read++;
    if( m_driver_type == HELISHI_DS102 )
    {
        switch( s_idx_read )
        {
            case 1: //读取左轮告警信息
                CanReadSDO( LEFT_ID, INDEX_102_LEFT_ERROR_MSG, SUBINDEX_102_ERROR_MSG, uint32, &gs_leftMotorDriverState.error_code );
                break;
            
            case 2: //读取右轮告警信息
                CanReadSDO( RIGHT_ID, INDEX_102_RIGHT_ERROR_MSG, SUBINDEX_102_ERROR_MSG, uint32, &gs_rightMotorDriverState.error_code );
                break;
            
            case 3: //读取左轮状态，提取驱动器告警信息
                DS102MotorError( gs_leftMotorDriverState.error_code, &gs_leftMotorDriverState.mcu_error_code );
                CanReadSDO( LEFT_ID, INDEX_STATUS, SUBINDEX_STATUS, uint16, &gs_leftMotorDriverState.status );
                break;
            
            case 4: //读取右轮状态
                DS102MotorError( gs_rightMotorDriverState.error_code, &gs_rightMotorDriverState.mcu_error_code );
                CanReadSDO( RIGHT_ID, INDEX_STATUS, SUBINDEX_STATUS, uint16, &gs_rightMotorDriverState.status );
                break;
            
            case 5: //读取左轮电机负载率
                CanReadSDO( LEFT_ID, INDEX_102_LEFT_LOAD, SUBINDEX_102_LOAD, uint16, &gs_leftMotorDriverState.load_factor );
                break;
            
            case 6: //读取右轮电机负载率
                CanReadSDO( RIGHT_ID, INDEX_102_RIGHT_LOAD, SUBINDEX_102_LOAD, uint16, &gs_rightMotorDriverState.load_factor );
                break;
            
            case 7: //读取左轮温度
                CanReadSDO( LEFT_ID, INDEX_102_LEFT_TEMPERATURE, SUBINDEX_102_TEMPERATURE, int16, &gs_leftMotorDriverState.temperature );
                break;
            
            case 8: //读取右轮温度
                CanReadSDO( RIGHT_ID, INDEX_102_RIGHT_TEMPERATURE, SUBINDEX_102_TEMPERATURE, int16, &gs_rightMotorDriverState.temperature );
                break;
            
            case 9: //左轮允许的最大电流值
                CanReadSDO( LEFT_ID, INDEX_102_LEFT_MAX_CURRENT, SUBINDEX_102_MAX_CURRENT, int16, &gs_leftMotorDriverState.max_current );
                break;
            
            case 10: //右轮允许的最大电流值
                CanReadSDO( RIGHT_ID, INDEX_102_RIGHT_MAX_CURRENT, SUBINDEX_102_MAX_CURRENT, int16, &gs_rightMotorDriverState.max_current );
                break;
            
            case 11: //左轮转矩限制模式
                if( !gs_leftMotorDriverState.cur_limit )
                {
                    CanReadSDO( LEFT_ID, INDEX_102_CURLIMIT, SUBINDEX_102_CURLIMIT, uint16, &gs_leftMotorDriverState.cur_limit );
                    break;
                }
                else
                    s_idx_read++;
            
            case 12: //右轮转矩限制模式
                if( !gs_rightMotorDriverState.cur_limit )
                {
                    CanReadSDO( RIGHT_ID, INDEX_102_CURLIMIT, SUBINDEX_102_CURLIMIT, uint16, &gs_rightMotorDriverState.cur_limit );
                    break;
                }
                else
                    s_idx_read++;
            
            case 13: //左轮控制模式
                CanReadSDO( LEFT_ID, INDEX_CTRLMODE, SUBINDEX_CTRLMODE, int8, &gs_leftMotorDriverState.ctrl_mode );
                break;
            
            case 14: //右轮控制模式
                CanReadSDO( RIGHT_ID, INDEX_CTRLMODE, SUBINDEX_CTRLMODE, int8, &gs_rightMotorDriverState.ctrl_mode );
                break;
            
            default:
                s_idx_read = 0;
                break;
        }
    }
    else if( m_driver_type == HELISHI_DS202 )
    {
        switch( s_idx_read )
        {
            case 1: //读取左轮告警信息
                CanReadSDO( LEFT_ID, INDEX_202_LEFT_ERROR_MSG, SUBINDEX_202_ERROR_MSG, uint32, &gs_leftMotorDriverState.error_code );
                break;
            
            case 2: //读取右轮告警信息
                CanReadSDO( RIGHT_ID, INDEX_202_RIGHT_ERROR_MSG, SUBINDEX_202_ERROR_MSG, uint32, &gs_rightMotorDriverState.error_code );
                break;
            
            case 3: //读取左轮状态，提取驱动器告警信息
                DS202MotorError( gs_leftMotorDriverState.error_code, &gs_leftMotorDriverState.mcu_error_code );
                CanReadSDO( LEFT_ID, INDEX_STATUS, SUBINDEX_STATUS, uint16, &gs_leftMotorDriverState.status );
                break;
            
            case 4: //读取右轮状态
                DS202MotorError( gs_rightMotorDriverState.error_code, &gs_rightMotorDriverState.mcu_error_code );
                CanReadSDO( RIGHT_ID, INDEX_STATUS, SUBINDEX_STATUS, uint16, &gs_rightMotorDriverState.status );
                break;
            
            case 5: //读取左轮电机负载率
                CanReadSDO( LEFT_ID, INDEX_202_LEFT_LOAD, SUBINDEX_202_LOAD, uint16, &gs_leftMotorDriverState.load_factor );
                break;
            
            case 6: //读取右轮电机负载率
                CanReadSDO( RIGHT_ID, INDEX_202_RIGHT_LOAD, SUBINDEX_202_LOAD, uint16, &gs_rightMotorDriverState.load_factor );
                break;
            
            case 7: //读取左轮温度
                CanReadSDO( LEFT_ID, INDEX_202_LEFT_TEMPERATURE, SUBINDEX_202_TEMPERATURE, int16, &gs_leftMotorDriverState.temperature );
                break;
            
            case 8: //读取右轮温度
                CanReadSDO( RIGHT_ID, INDEX_202_RIGHT_TEMPERATURE, SUBINDEX_202_TEMPERATURE, int16, &gs_rightMotorDriverState.temperature );
                break;
            
            case 9: //左轮允许的最大电流值
                CanReadSDO( LEFT_ID, INDEX_202_LEFT_MAX_CURRENT, SUBINDEX_202_MAX_CURRENT, int16, &gs_leftMotorDriverState.max_current );
                break;
            
            case 10: //右轮允许的最大电流值
                CanReadSDO( RIGHT_ID, INDEX_202_RIGHT_MAX_CURRENT, SUBINDEX_202_MAX_CURRENT, int16, &gs_rightMotorDriverState.max_current );
                break;
            
            case 11: //驱动器速度环第一比例增益
                CanReadSDO( LEFT_ID, INDEX_202_LEFT_VELOCITY_1P, SUBINDEX_202_VELOCITY_1P, uint16, &gs_leftMotorDriverState.velocity_1P );
                break;
            
            case 12: //驱动器速度环第一比例增益
                CanReadSDO( RIGHT_ID, INDEX_202_RIGHT_VELOCITY_1P, SUBINDEX_202_VELOCITY_1P, uint16, &gs_rightMotorDriverState.velocity_1P );
                break;
            
            case 13: //驱动器速度环第一积分增益
                CanReadSDO( LEFT_ID, INDEX_202_LEFT_VELOCITY_1I, SUBINDEX_202_VELOCITY_1I, uint16, &gs_leftMotorDriverState.velocity_1I );
                break;
            
            case 14: //驱动器速度环第一积分增益
                CanReadSDO( RIGHT_ID, INDEX_202_RIGHT_VELOCITY_1I, SUBINDEX_202_VELOCITY_1I, uint16, &gs_rightMotorDriverState.velocity_1I );
                break;
            
            case 15: //左轮转矩限制模式
                CanReadSDO( LEFT_ID, INDEX_202_LEFTCURLIMIT, SUBINDEX_202_CURLIMIT, uint16, &gs_leftMotorDriverState.cur_limit );
                break;
            
            case 16: //右轮转矩限制模式
                CanReadSDO( RIGHT_ID, INDEX_202_RIGHTCURLIMIT, SUBINDEX_202_CURLIMIT, uint16, &gs_rightMotorDriverState.cur_limit );
                break;
            
            case 17: //左轮控制模式
                CanReadSDO( LEFT_ID, INDEX_CTRLMODE, SUBINDEX_CTRLMODE, int8, &gs_leftMotorDriverState.ctrl_mode );
                break;
            
            case 18: //右轮控制模式
                CanReadSDO( RIGHT_ID, INDEX_CTRLMODE, SUBINDEX_CTRLMODE, int8, &gs_rightMotorDriverState.ctrl_mode );
                s_idx_read = 0;
                break;
            
            default:
                s_idx_read = 0;
                break;
        }
    }
}

//将速度写入驱动器中
static void SetSpeedToMotor(uint8_t f_driverID,int32_t f_speed)
{
    int16_t torque = 0;
	if(f_speed > 6000)
		f_speed = 6000;
	else if(f_speed < -6000 )
		f_speed = -6000;
	
	if( gs_rightMotorDriverState.control_mode==MODE_SPEED)
        CanWriteSDO( f_driverID, 0x60FF, 0x00, int32, &f_speed );
    else
    {
        torque = (int16_t)f_speed;
        CanWriteSDO( f_driverID, 0x6071, 0x00, int16, &torque );
    }
}
//设置速度
static void SetSpeed(int32_t f_leftSpeed, int32_t f_rightSpeed)
{
	//1,设置速度到驱动器
	SetSpeedToMotor(LEFT_ID,f_leftSpeed);
	SetSpeedToMotor(RIGHT_ID,f_rightSpeed);
}
uint32_t gs_left_encoder_read_cnt = 0;
uint32_t gs_right_encoder_read_cnt = 0;
//读取编码器数据
static void ReadEncoderCnt(uint8_t f_driverID)
{
	if( f_driverID == LEFT_ID )
	{
        //读取编码器计数是计算一次上传给上位机的count值，现在频率为40hz
        if(m_driver_type == HELISHI_DS202)
            gs_leftEncoderCnt.short_cnt = gs_leftEncoderCnt.cnt;
        else if (m_driver_type == HELISHI_DS102)
            gs_leftEncoderCnt.short_cnt = 0 - gs_leftEncoderCnt.cnt;

		CanReadSDO( LEFT_ID, INDEX_ENCODER_CNT, SUBINDEX_ENCODER_CNT, int32, &gs_leftEncoderCnt.cnt );
        gs_left_encoder_read_cnt++;
        if( gs_leftEncoderCnt.error_count<ENCODER_MAX_ERROR_NUM )
            gs_leftEncoderCnt.error_count++;
	}
	else if( f_driverID == RIGHT_ID )
	{
        if (m_driver_type == HELISHI_DS202)
            gs_rightEncoderCnt.short_cnt = 4294967295 - (gs_rightEncoderCnt.cnt);
        else if (m_driver_type == HELISHI_DS102)
            gs_rightEncoderCnt.short_cnt = (gs_rightEncoderCnt.cnt);

		CanReadSDO( RIGHT_ID, INDEX_ENCODER_CNT, SUBINDEX_ENCODER_CNT, int32, &gs_rightEncoderCnt.cnt );
        gs_right_encoder_read_cnt++;
        if(gs_rightEncoderCnt.error_count<ENCODER_MAX_ERROR_NUM)
            gs_rightEncoderCnt.error_count++;
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
    static uint32_t left_count_tick = 0;
    static uint32_t right_count_tick = 0;
    int32_t count = 0;
    //计算左轮dcount
	xSemaphoreTake(gs_leftEncoderCntMutex, portMAX_DELAY);
    if (m_driver_type == HELISHI_DS202)
        count = gs_leftEncoderCnt.cnt;
    else if (m_driver_type == HELISHI_DS102)
        count = 0 - gs_leftEncoderCnt.cnt;

    if( gs_leftEncoderCnt.error_count < ENCODER_MAX_ERROR_NUM )
        gs_leftEncoderCnt.relativeCnt = count - gs_leftEncoderCnt.lastCnt;
    else
    {
        gs_leftEncoderCnt.relativeCnt = 0;
        if( gs_leftEncoderCnt.error_count >= ENCODER_MAX_ERROR_NUM*2 ) //从新读取成功后
            gs_leftEncoderCnt.error_count = 0;
    }
    //单次计算时间内不可能走这么多，由此判断发生了数据超限过65536从0开始
	//22.7.28 由于快速推动车时会超过30000(约1.7m/s)，导致定位异常故提升到45000(大于2.5m/s) odom计数由int16改成int32
    if( gs_leftEncoderCnt.relativeCnt > 45000 )
    {
        gs_leftEncoderCnt.relativeCnt -= 4294967295;
    }
    else if(  gs_leftEncoderCnt.relativeCnt < -45000 )
    {
        gs_leftEncoderCnt.relativeCnt += 4294967295;
    }
    //使用两次获取到的count值时的时间来优化delta
    if( gs_leftEncoderCnt.tick-left_count_tick > 0)
    {
        gs_leftEncoderCnt.relativeCnt = gs_leftEncoderCnt.relativeCnt*50/(int32_t)(gs_leftEncoderCnt.tick-left_count_tick);
    }
    gs_leftEncoderCnt.lastCnt = count;
    left_count_tick = gs_leftEncoderCnt.tick;
	xSemaphoreGive(gs_leftEncoderCntMutex);
    
	//计算右轮dcount
	xSemaphoreTake(gs_rightEncoderCntMutex, portMAX_DELAY);
    if (m_driver_type == HELISHI_DS202)
        count = 4294967295 - (gs_rightEncoderCnt.cnt);
    else if (m_driver_type == HELISHI_DS102)
        count = (gs_rightEncoderCnt.cnt);

    if( gs_rightEncoderCnt.error_count < ENCODER_MAX_ERROR_NUM )
        gs_rightEncoderCnt.relativeCnt = count - gs_rightEncoderCnt.lastCnt;
    else
    {
        gs_rightEncoderCnt.relativeCnt = 0;
        if( gs_rightEncoderCnt.error_count>=ENCODER_MAX_ERROR_NUM*2 ) //从新读取成功后
            gs_rightEncoderCnt.error_count = 0;
    }
    
    if( gs_rightEncoderCnt.relativeCnt > 45000 )
    {
        gs_rightEncoderCnt.relativeCnt -= 4294967295;
    }
    else if(  gs_rightEncoderCnt.relativeCnt < -45000 )
    {
        gs_rightEncoderCnt.relativeCnt += 4294967295;
    }
    
    if( gs_rightEncoderCnt.tick-right_count_tick > 0)
    {
        gs_rightEncoderCnt.relativeCnt = gs_rightEncoderCnt.relativeCnt*50/(int32_t)(gs_rightEncoderCnt.tick-right_count_tick);
    }
    gs_rightEncoderCnt.lastCnt  = count;
    right_count_tick = gs_rightEncoderCnt.tick;
	xSemaphoreGive(gs_rightEncoderCntMutex);
}

//同驱动器通讯连接情况，1表示正常，0表示异常
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
	if( GetLinkStatus(&gs_leftDriverconnStatus) )
		gs_leftDriverHealthState = HEALTHY;
	else
		gs_leftDriverHealthState = UNHEALTHY;

	if( GetLinkStatus(&gs_rightDriverconnStatus) )
		gs_rightDriverHealthState = HEALTHY;
	else
		gs_rightDriverHealthState = UNHEALTHY;
}
static void CheckMotorDriverLinkState(void)
{
	if(gs_getLeftDriverRet == 1)
	{
		ResetConnectedTime(&gs_leftDriverconnStatus);
		gs_getLeftDriverRet = 0;
	}
	
	if(gs_getRightDriverRet == 1)
	{
		ResetConnectedTime(&gs_rightDriverconnStatus);
		gs_getRightDriverRet = 0;
	}
}

//电机运行状态检测
static int MotorLogic(void)
{
    static uint8_t s_old_alarm = 0;
    static TickType_t s_left_cur_over_start = 0;
    static TickType_t s_right_cur_over_start = 0;
    static TickType_t s_left_temp_over_start = 0;
    static TickType_t s_right_temp_over_start = 0;
    static uint8_t s_clear_ds_alarm_delay = 0;
    
    uint16_t data = 0;
	uint32_t clear_err = 0;
    
    
    if( s_old_alarm != m_motor_alarm )
    {
        s_old_alarm = m_motor_alarm;
        if( s_old_alarm == 0 ) //报警信息 1 --> 0 清除报警信息
        {
            gs_leftMotorDriverState.alarm = 0;
            gs_leftMotorDriverState.alarm_current = 0;
            gs_leftMotorDriverState.alarm_temperature = 0;
            gs_rightMotorDriverState.alarm = 0;
            gs_rightMotorDriverState.alarm_current = 0;
            gs_rightMotorDriverState.alarm_temperature = 0;
			//驱动器复位
			if( gs_leftMotorDriverState.mcu_error_code || gs_rightMotorDriverState.mcu_error_code )
			{
				CleanMotorError( LEFT_ID);
				gs_leftMotorDriverState.mcu_error_code = 0;
				gs_rightMotorDriverState.mcu_error_code = 0;
			}
        }
    }
    
    if( gs_leftMotorDriverState.current >= m_left_protect.current )//过流检测
    {
        if( xTaskGetTickCount() - s_left_cur_over_start >= 5000 )
        {
            gs_leftMotorDriverState.alarm = 1;
            gs_leftMotorDriverState.alarm_current = 1;
        }
    }
    else
    {
        s_left_cur_over_start = xTaskGetTickCount();
    }
    
    if( gs_rightMotorDriverState.current >= m_right_protect.current )
    {
        if( xTaskGetTickCount() - s_right_cur_over_start >= 5000 )
        {
            gs_rightMotorDriverState.alarm = 1;
            gs_rightMotorDriverState.alarm_current = 1;
        }
    }
    else
    {
        s_right_cur_over_start = xTaskGetTickCount();
    }
    if( !m_temperature_check_delay )
    {
        if( gs_leftMotorDriverState.temperature >= m_left_protect.temperature )//过温检测
        {
            if( xTaskGetTickCount() - s_left_temp_over_start >= 5000 )
            {
                gs_leftMotorDriverState.alarm = 1;
                gs_leftMotorDriverState.alarm_temperature = 1;
            }
        }
        else
        {
            s_left_temp_over_start = xTaskGetTickCount();
        }
        
        if( gs_rightMotorDriverState.temperature >= m_right_protect.temperature )
        {
            if( xTaskGetTickCount() - s_right_temp_over_start >= 5000 )
            {
                gs_rightMotorDriverState.alarm = 1;
                gs_rightMotorDriverState.alarm_temperature = 1;
            }
        }
        else
        {
            s_right_temp_over_start = xTaskGetTickCount();
        }
        
        if( gs_leftMotorDriverState.temperature > m_left_protect.temperature+5 )//过温检测
        {
            gs_leftMotorDriverState.alarm = 1;
            gs_leftMotorDriverState.alarm_temperature = 1;
        }
        
        if( gs_rightMotorDriverState.temperature > m_right_protect.temperature+5 )
        {
            gs_rightMotorDriverState.alarm = 1;
            gs_rightMotorDriverState.alarm_temperature = 1;
        }
    }
    
    if( gs_leftMotorDriverState.alarm ==1 || gs_rightMotorDriverState.alarm == 1 )
    {
        m_motor_alarm = 1;
    }
    
    s_clear_ds_alarm_delay++;
    if( s_clear_ds_alarm_delay >= 100 ) //清除驱动器告警
    {
        if (m_driver_type == HELISHI_DS202)
        {
            if( (gs_leftMotorDriverState.mcu_error_code || gs_rightMotorDriverState.mcu_error_code)&&m_clear_ds_alarm)
            {
                CleanMotorError( LEFT_ID);
                gs_leftMotorDriverState.mcu_error_code = 0;
                gs_rightMotorDriverState.mcu_error_code = 0;
            }
        }
        else if( m_driver_type == HELISHI_DS102)
        {
            data = 1;
            if( gs_leftMotorDriverState.mcu_error_code&&m_clear_ds_alarm )
            {
                CanWriteSDO( LEFT_ID, 0x2053, 0x00, uint16, &data );
            }
            if( gs_rightMotorDriverState.mcu_error_code&&m_clear_ds_alarm )
            {
                CanWriteSDO( RIGHT_ID, 0x2053, 0x00, uint16, &data );
            }
        }
        s_clear_ds_alarm_delay = 0;
        m_clear_ds_alarm = 0;
    }
    
    return m_motor_alarm;
}

static void DS102MotorError(uint32_t src, uint32_t *prt)
{
	uint32_t temp = 0;

	temp = 0;
	switch(src)
	{
		case 0x1516: temp |= (0x01 << 0); break; //不可识别故障
		case 0x160C: temp |= (0x01 << 1); break; //过流
		case 0x1614: temp |= (0x01 << 2); break; //过载
		case 0x1611: temp |= (0x01 << 3); break; //放电回路频繁动作平均功率大
		case 0x1411: temp |= (0x01 << 4); break; //放电报警瞬时功率大
		case 0x161C: temp |= (0x01 << 5); break; //过压
		case 0x170A: temp |= (0x01 << 6); break; //参数故障
		case 0x120C: temp |= (0x01 << 7); break; //电流采样（中点）故障
		case 0x0E0C: temp |= (0x01 << 8); break; //编码器故障 ABZ 报警
		case 0x170E: temp |= (0x01 << 9); break; //位置超差
		case 0x1A1B: temp |= (0x01 << 10); break; //失速
		case 0x141C: temp |= (0x01 << 11); break; //欠压
		case 0x0E11: temp |= (0x01 << 12); break; //编码器故障 UVW 报警
		case 0x0A16: temp |= (0x01 << 13); break; //缺相保护
		case 0x0F0E: temp |= (0x01 << 14); break; //FPGA 配置失败
		case 0x120D: temp |= (0x01 << 15); break; //输入口功能定义重复
		case 0x0D0E: temp |= (0x01 << 16); break; //功率母线未准备好
		case 0x1A14: temp |= (0x01 << 17); break; //速度指令太小
		case 0x0F0D: temp |= (0x01 << 18); break; //FPGA 死机
		case 0x0C0E: temp |= (0x01 << 19); break; //通讯错误
		case 0x0B0E: temp |= (0x01 << 20); break; //协同模式报警
		case 0x1416: temp |= (0x01 << 21); break; //CAN 总线断线报警
        case 0x161B: temp |= (0x01 << 24); break;  //温度报警
		default: break;
	}

	*prt = temp;
}
/**************************************************************************************
* 作者/公司  :杨磊/高仙
* 函数介绍   :双轴和利时读取的错误参数转换为上位机所需要数据
* 输入参数   :
* 输出参数   :
* 返回值     :
****************************************************************************************/
static void DS202MotorError(uint32_t src, uint32_t *prt)
{
  uint32_t temp = 0;
  
    temp = 0;
    switch(src)
    {
      case 0x01: temp |= (0x01 << 8);  break;  //编码器故障 ABZ 报警
      case 0x02: temp |= (0x01 << 12); break;  //编码器故障 UVW 报警
      case 0x03: temp |= (0x01 << 9);  break;  //位置超差
      case 0x04: temp |= (0x01 << 10); break;  //失速
      case 0x05: temp |= (0x01 << 7);  break;  //电流采样（中点）故障
      case 0x06: temp |= (0x01 << 2);  break;  //过载
      case 0x07: temp |= (0x01 << 22); break;  //功率电源欠压
      case 0x08: temp |= (0x01 << 23); break;  //功率电源过压
      case 0x09: temp |= (0x01 << 1);  break;  //过流
      case 0x0A: temp |= (0x01 << 4);  break;  //瞬时放电报警
      case 0x0B: temp |= (0x01 << 3);  break;  //平均放电报警
      case 0x0C: temp |= (0x01 << 6);  break;  //参数读写异常
      case 0x0D: temp |= (0x01 << 15); break;  //输入端口重复定义
      case 0x0E: temp |= (0x01 << 21); break;  //断线保护
      case 0x0F: temp |= (0x01 << 24); break;  //温度报警
      default: break;
    }
    
    if( m_driver_type == HELISHI_DS202 )//一拖二检测到断链时需要清除告警后驱动器才能正常工作
    {
        uint16_t data = 0;
        if( src == 0x0E )
        {
            CanWriteSDO( LEFT_ID, 0x4602, 0x00, uint16, &data );
        }
    }
  
    *prt = temp;
}


static uint16_t LeftMotorMaxCurrent(void)
{
    if( m_driver_type==HELISHI_DS202 )
        return gs_leftMotorDriverState.max_current/10;//0.1A
    else
        return (uint16_t)(gs_leftMotorDriverState.max_current*2.5); //0.1A
}

static uint16_t RightMotorMaxCurrent(void)
{
    if( m_driver_type==HELISHI_DS202 )
        return gs_rightMotorDriverState.max_current/10;//0.1A
    else
        return (uint16_t)(gs_rightMotorDriverState.max_current*2.5);//0.1A
}
//充电时更改电机保护电流，防止过温
static void SetMotorMaxCurrent(void)
{
    static uint16_t s_left_velocity_1P = 0;
    static uint16_t s_right_velocity_1P = 0;
    static uint16_t s_left_velocity_1I = 0;
    static uint16_t s_right_velocity_1I = 0;
    uint16_t left_target_value = 0;
    uint16_t right_target_value = 0;
    uint16_t left_velocity_1P = 0;
    uint16_t right_velocity_1P = 0;
    uint16_t left_velocity_1I = 0;
    uint16_t right_velocity_1I = 0;
    uint8_t count = 0;
    
	//记录上电后驱动器内部的pid参数，以便不充电时再改回原来参数
    if( !s_left_velocity_1P && gs_leftMotorDriverState.velocity_1P )
        s_left_velocity_1P = gs_leftMotorDriverState.velocity_1P;
    
    if( !s_right_velocity_1P && gs_rightMotorDriverState.velocity_1P )
        s_right_velocity_1P = gs_rightMotorDriverState.velocity_1P;
    
    if( !s_left_velocity_1I && gs_leftMotorDriverState.velocity_1I )
        s_left_velocity_1I = gs_leftMotorDriverState.velocity_1I;
    
    if( !s_right_velocity_1I && gs_rightMotorDriverState.velocity_1I )
        s_right_velocity_1I = gs_rightMotorDriverState.velocity_1I;
    
    if( GetRelayStatus() & 0x01 )//自动充电继电器闭合
    {
		//计算充电时下发给驱动器的电流保护值以及pid参数
        if( m_driver_type == HELISHI_DS202)
        {
            left_target_value = m_left_protect.current_incharge*10;
            right_target_value = m_right_protect.current_incharge*10;
        }
        else
        {
            left_target_value = m_left_protect.current_incharge*10/25;
            right_target_value = m_right_protect.current_incharge*10/25;
        }
        left_velocity_1P = m_velocity_1P_incharge;
        right_velocity_1P = m_velocity_1P_incharge;
        left_velocity_1I = m_velocity_1I_incharge;
        right_velocity_1I = m_velocity_1I_incharge;
    }
    else
    {//充电完成后将保护参数改回
        if( m_driver_type == HELISHI_DS202)
        {
            left_target_value = m_left_protect.current*10;
            right_target_value = m_right_protect.current*10;
        }
        else
        {
            left_target_value = m_left_protect.current*10/25;
            right_target_value = m_right_protect.current*10/25;
        }
        left_velocity_1P = s_left_velocity_1P;
        right_velocity_1P = s_right_velocity_1P;
        left_velocity_1I = s_left_velocity_1I;
        right_velocity_1I = s_right_velocity_1I;
    }
    
    if (m_driver_type == HELISHI_DS202)
    {
        if( s_left_velocity_1P && left_velocity_1P && gs_leftMotorDriverState.velocity_1P!=left_velocity_1P )
        {
            CanWriteSDO( LEFT_ID, 0x2300, 0x00, uint16, &left_velocity_1P);
            count++;
        }
        if( s_left_velocity_1I && left_velocity_1I && gs_leftMotorDriverState.velocity_1I!=left_velocity_1I )
        {
            CanWriteSDO( LEFT_ID, 0x2301, 0x00, uint16, &left_velocity_1I);
            count++;
        }
        if( s_right_velocity_1P && right_velocity_1P && gs_rightMotorDriverState.velocity_1P!=right_velocity_1P )
        {
            CanWriteSDO( RIGHT_ID, 0x3300, 0x00, uint16, &right_velocity_1P);
            count++;
        }
        if( s_right_velocity_1I && right_velocity_1I && gs_rightMotorDriverState.velocity_1I!=right_velocity_1I )
        {
            CanWriteSDO( RIGHT_ID, 0x3301, 0x00, uint16, &right_velocity_1I);
            count++;
        }
        if( count )//pid修改完成
        {;}
        else if( LeftMotorMaxCurrent() != 0 && gs_leftMotorDriverState.max_current != left_target_value )
        {
            CanWriteSDO( LEFT_ID, INDEX_202_LEFT_MAX_CURRENT, SUBINDEX_202_MAX_CURRENT, uint16, &left_target_value );
        }
        else if( RightMotorMaxCurrent() != 0 && gs_rightMotorDriverState.max_current != right_target_value )
        {
            CanWriteSDO(  RIGHT_ID, INDEX_202_RIGHT_MAX_CURRENT, SUBINDEX_202_MAX_CURRENT, uint16, &right_target_value);
        }
    }
    else if (m_driver_type == HELISHI_DS102)
    {
        if( LeftMotorMaxCurrent() != 0 && gs_leftMotorDriverState.max_current != left_target_value )
        {
            CanWriteSDO( LEFT_ID, 0x202B, 0x00, uint16, &left_target_value );
            CanWriteSDO( LEFT_ID, 0x202C, 0x00, uint16, &left_target_value );
        }
        else if( RightMotorMaxCurrent() != 0 && gs_rightMotorDriverState.max_current != right_target_value )
        {
            CanWriteSDO( RIGHT_ID, 0x202B, 0x00, uint16, &right_target_value );
            CanWriteSDO( RIGHT_ID, 0x202C, 0x00, uint16, &right_target_value );
        }
    }
}

//检查驱动器状态与命令是否相符
static int CheckDriverStatus(void)
{
    static TickType_t s_left_tick = 10000;
    static TickType_t s_right_tick = 10000;
    static uint16_t s_check_curlimit_delay = 0;
    static uint16_t s_check_ctrlmode_delay = 0;
    int ret = 0;
    
    if( (gs_leftMotorDriverState.status&0x0050) == 0x0050 )
    {
        if( xTaskGetTickCount() - s_left_tick >= 5000 ) //防止频繁初始化
        {
            MotorInit(0x01);
            s_left_tick = xTaskGetTickCount();
            ret = 1;
        }
    }
    if( (gs_rightMotorDriverState.status&0x0050) == 0x0050 )
    {
        if( xTaskGetTickCount() - s_right_tick >= 5000 ) //防止频繁初始化
        {
            MotorInit(0x02);
            s_right_tick = xTaskGetTickCount();
            ret = 1;
        }
    }
    if (m_driver_type == HELISHI_DS102)//确保一拖一使用转矩限制模式
    {
        if( 1!=gs_leftMotorDriverState.cur_limit || 1!=gs_rightMotorDriverState.cur_limit )
        {
            s_check_curlimit_delay++;
            if( s_check_curlimit_delay >= 400 )
            {
                s_check_curlimit_delay = 0;
                MotorInit(0x03);
                ret = 1;
            }
        }
        else
            s_check_curlimit_delay = 0;
    }
    else if (m_driver_type == HELISHI_DS202)//确保一拖二过流保护使用电流限制模式
    {
        if( 1==gs_leftMotorDriverState.cur_limit || 1==gs_rightMotorDriverState.cur_limit )
        {
            s_check_curlimit_delay++;
            if( s_check_curlimit_delay >= 400 )
            {
                s_check_curlimit_delay = 0;
                MotorInit(0x03);
                ret = 1;
            }
        }
        else
            s_check_curlimit_delay = 0;
    }
	//防止左右轮速度模式和转矩模式切换时没有切换成功，
    if( gs_leftMotorDriverState.ctrl_mode!=((gs_leftMotorDriverState.control_mode==MODE_TORQUE)?4:3 )\
        || gs_rightMotorDriverState.ctrl_mode!=((gs_rightMotorDriverState.control_mode==MODE_TORQUE)?4:3 ) )
    {
        s_check_ctrlmode_delay++;
        if( s_check_ctrlmode_delay>=400)
        {
            s_check_ctrlmode_delay = 0;
            if( gs_leftMotorDriverState.ctrl_mode!=gs_leftMotorDriverState.control_mode )
                WriteControlMode( LEFT_ID, gs_leftMotorDriverState.control_mode );
            if( gs_rightMotorDriverState.ctrl_mode!=gs_rightMotorDriverState.control_mode )
                WriteControlMode( RIGHT_ID, gs_rightMotorDriverState.control_mode );
        }
    }
    else
        s_check_ctrlmode_delay = 0;
    
    return ret;
}
/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/
//设置电机保护温度
int16_t GetLeftMotorTemperature(void)
{
    return gs_leftMotorDriverState.temperature;
}

int16_t GetRightMotorTemperature(void)
{
    return gs_rightMotorDriverState.temperature;
}
//设置电机保护电流
int16_t GetLeftMotorCurrent(void)
{
    return gs_leftMotorDriverState.current;
}

int16_t GetRightMotorCurrent(void)
{
    return gs_rightMotorDriverState.current;
}
//获取驱动器对电机负载率
uint16_t GetLeftMotorLoadFactor(void)
{
    return gs_leftMotorDriverState.load_factor;
}

uint16_t GetRightMotorLoadFactor(void)
{
    return gs_rightMotorDriverState.load_factor;
}
//获取驱动器错误码
uint32_t GetLeftMotorErrorCode(void)
{
    return gs_leftMotorDriverState.mcu_error_code;
}

uint32_t GetRightMotorErrorCode(void)
{
    return gs_rightMotorDriverState.mcu_error_code;
}
//获取过温告警
uint8_t GetLRTemperatureAlarm(void)
{
    uint8_t alarm = 0;
    alarm |= gs_leftMotorDriverState.alarm_temperature&0x01;
    alarm |= (gs_rightMotorDriverState.alarm_temperature&0x01)<<1;
    return alarm;
}
//获取过流告警
uint8_t GetLRCurrentAlarm(void)
{
    uint8_t alarm = 0;
    alarm |= gs_leftMotorDriverState.alarm_current&0x01;
    alarm |= (gs_rightMotorDriverState.alarm_current&0x01)<<1;
    return alarm;
}
//清除驱动器告警
void ClearHollySysMotorAlarm(void)
{
    m_motor_alarm = 0;
    m_clear_ds_alarm = 1;
}
//写左右轮速度
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
//设置刹车
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
//使能驱动器（手自动）
void CfgDriverEnable(en_functional_state_t f_newState)
{
	gs_cfgEnable = f_newState;
}
uint8_t GetMotorDriverHealthState(void)
{
	uint8_t ret = 0;//1=健康
	if(GetRightDriverHealthState() == UNHEALTHY || GetLeftDriverHealthState() == UNHEALTHY)
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
void GetLeftDecoderInfo(int32_t *fp_cnt, int32_t *fp_relativeCnt)
{
	xSemaphoreTake(gs_leftEncoderCntMutex, portMAX_DELAY);
    *fp_cnt = gs_leftEncoderCnt.short_cnt; //这里返回上次的值是因为相对计数是与上次的值对应的
	*fp_relativeCnt = gs_leftEncoderCnt.relativeCnt; 
    xSemaphoreGive(gs_leftEncoderCntMutex);
}
void GetRightDecoderInfo(int32_t *fp_cnt, int32_t *fp_relativeCnt)
{
	xSemaphoreTake(gs_rightEncoderCntMutex, portMAX_DELAY);
    *fp_cnt = gs_rightEncoderCnt.short_cnt; //这里返回上次的值是因为相对计数是与上次的值对应的
	*fp_relativeCnt = gs_rightEncoderCnt.relativeCnt; 
    xSemaphoreGive(gs_rightEncoderCntMutex);
}
//获取报警信息,SET=有报警，RESET=无报警
en_functional_state_t GetLeftDriverAlarmState(void)
{
	return (en_functional_state_t)gs_leftMotorDriverState.alarm;
}
en_functional_state_t GetRightDriverAlarmState(void)
{
	return (en_functional_state_t)gs_rightMotorDriverState.alarm;
}


void ReadMotorStatus( uint8_t* fp_enable, int32_t speed[], int32_t count[], int32_t dcount[], uint16_t fp_max_cur[])
{
    *fp_enable =  gs_leftMotorDriverState.enable|gs_rightMotorDriverState.enable;
    speed[0] = gs_leftSpeed; 
	speed[1] = gs_rightSpeed; 
    speed[2] = gs_leftSpeedToDriver;
    speed[3] = gs_rightSpeedToDriver;
    
    count[0] = gs_leftEncoderCnt.lastCnt;
    count[1] = gs_rightEncoderCnt.lastCnt;
    
    dcount[0] = gs_leftEncoderCnt.relativeCnt;
    dcount[1] = gs_rightEncoderCnt.relativeCnt;
    
    fp_max_cur[0] = gs_leftMotorDriverState.cur_limit;
    fp_max_cur[1] = gs_rightMotorDriverState.cur_limit;
    fp_max_cur[2] = gs_leftMotorDriverState.max_current;
    fp_max_cur[3] = gs_rightMotorDriverState.max_current;
}

//获取车是否在运动，非零表示在运动
int GetCarMoveStatus(void)
{
    if( abs(gs_leftEncoderCnt.relativeCnt) < 8 && abs(gs_rightEncoderCnt.relativeCnt) < 8 )
    {
        return 0;
    }
    return 1;
}

//获取控制模式，0速度模式，1扭矩模式
uint8_t GetHollySysControlMode(void)
{
    uint8_t mode = 0;
    
    if( gs_rightMotorDriverState.control_mode == MODE_TORQUE )
        mode = 3;
    
    return mode;
}

void GetHollySysDriverType( uint32_t fp_type[] )
{
    if( NULL != fp_type )
    {
        fp_type[0] = m_driver_type;
        fp_type[1] = m_102_version;
        fp_type[2] = m_202_version;
    }
}
//设置自动充电时轮毂电机最大电流
void SetMotorMaxCurrentInCharge( uint32_t f_current )
{
    if( f_current>=100 )
    {
        m_left_protect.current_incharge = f_current/100;
        m_right_protect.current_incharge = f_current/100;
    }
}

void SetHollySysDriverType( uint8_t f_type )
{
    if( !m_driver_type ) //尚未获取到驱动器类型
    {
        if( f_type == HELISHI_DS102 )
        {
            m_driver_type = f_type;
            m_driver_type_update_flag = 1;
        }
        else if( f_type == HELISHI_DS202 )
        {
            m_driver_type = f_type;
            m_driver_type_update_flag = 1;
        }
    }
    else
    {
        if( m_driver_type != f_type )
        {
            //上报告警给上位机
        }
    }
}

void SetHollySysPidIncharge(uint32_t f_p, uint32_t f_i )
{
    if( f_p>0 && f_i>0 )
    {
        m_velocity_1P_incharge = f_p;
        m_velocity_1I_incharge = f_i;
    }
}

void SetHollySysMotorTemperature(uint32_t f_left, uint32_t f_right )
{
    if( f_left >= 75 )
        m_left_protect.temperature = f_left;
    if( f_right>=75 )
        m_right_protect.temperature = f_right;
}

void SetHollySysMotorCurrent(uint32_t f_left, uint32_t f_right )
{
    if( f_left>=5000)
        m_left_protect.current = f_left/100;
    if( f_right>=5000)
        m_right_protect.current = f_right/100;
}

void SetHollySysControlMode(uint8_t f_mode )
{
    m_control_mode = f_mode;
    if( m_control_mode > MODE_TORQUE)
        m_control_mode = MODE_SPEED;
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
#include "nanopb_tcpip.h"
void MotorDriverTask(void const *pvParameters)
{
	uint8_t readEncoderCntTimes = 0;//记录发送读取编码计数的次数，
	TickType_t lastWakeTime;
	int32_t leftSpeed,rightSpeed;
	const TickType_t c_frequency = pdMS_TO_TICKS(5);
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
	InitSoftwareMotorDriver();
	InitSoftwareDecoder( );
	//读编码器值
	TickType_t readCntTime = 0;
	const TickType_t c_readCntFrequency = pdMS_TO_TICKS(25);
	//设置速度
	TickType_t setSpeedTime = 0;
	const TickType_t c_setSpeedFrequency = pdMS_TO_TICKS(20);
	//判断并设置使能
	TickType_t last_read_status_tick = 0;
	const TickType_t c_read_status_frequency = pdMS_TO_TICKS(30);
    
    TickType_t set_max_current_tick = 0;
    const TickType_t c_set_current_frequency = pdMS_TO_TICKS(500);
    
    uint8_t read_count = 0;
    
    uint8_t control_mode = 0;
    
    vTaskDelay(5000);
	while((ConfigCommandCMD.motor_driver_type != 1)&&(ConfigCommandCMD.motor_driver_type != 2))
	{
		vTaskDelay(5000);
	}
	ObjdictLoad( m_objdict, m_objdict_size );


    lastWakeTime = xTaskGetTickCount();
    //确认驱动器类型
    while( !m_driver_type )
    {
//        for(int i=0;i<4;i++)
//            FeedDog(bitPos);
        CanReadSDO( LEFT_ID, INDEX_102_VERSION, SUBINDEX_102_VERSION, uint32, &m_102_version );
        CanReadSDO( LEFT_ID, INDEX_202_VERSION, SUBINDEX_202_VERSION, uint32, &m_202_version );
        vTaskDelay( 20);
        if( m_102_version && m_202_version )
        {
            m_driver_type = HELISHI_DS202;
        }
        else if( m_102_version )
        {
            read_count++;
            if( read_count > 10 )
            {
                m_driver_type = HELISHI_DS102;
            }
        }
//        g_thread_call_count[thread_num]++;
    }
    //和利时电机初始化
    MotorInit(0x03);
    
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
    readCntTime = lastWakeTime;
    setSpeedTime = lastWakeTime;
    last_read_status_tick = lastWakeTime;

    while(1)
    {
		/* 喂狗 */
//		FeedDog(bitPos);
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelay(c_frequency);
      	//获取编码计数
		if( xTaskGetTickCount() - readCntTime >= c_readCntFrequency )
		{
			ReadEncoderCnt(LEFT_ID);
			ReadEncoderCnt(RIGHT_ID);
			readEncoderCntTimes++;
			gs_getPosFlag = 0;
			readCntTime = xTaskGetTickCount();
		}
		//发送n此后收到了左右位置或者发送了n+1次
		if(((readEncoderCntTimes == 2) && (gs_getPosFlag == 0x03)) || (readEncoderCntTimes > 2))
		{
			DecoderRelativeCnt();//根据总值计算相对值
			readEncoderCntTimes = 0;
		}
		//1,获取速度
		ReadSpeedVal(&leftSpeed,&rightSpeed);
		//2,检查连接状态和健康状态
		CheckMotorDriverLinkState();
		CheckMotorDriverHealthState();
		//3,读取驱动器状态信息，检查驱动器异常
		if( xTaskGetTickCount() - last_read_status_tick >= c_read_status_frequency )
		{
            ReadDriverStatus();
			last_read_status_tick = xTaskGetTickCount();
		}
        if(MotorLogic())
        {
            if( GetEmergencyStopState() != 0x01 )
            {
                gs_cfgEnable = DISABLE;
                leftSpeed = 0;
                rightSpeed = 0;
            }
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
        
        //6，判断使能状态，
        if(gs_cfgEnable != GetLeftDriverEnableState())
        {
            EnableDriver(LEFT_ID,gs_cfgEnable);
        }
        if(gs_cfgEnable != GetRightDriverEnableState())
        {
            EnableDriver(RIGHT_ID,gs_cfgEnable);
        }
        
		//7,控制运动,因为是使用can通信，所以速度的设置频率不能太高
		if( xTaskGetTickCount() - setSpeedTime >= c_setSpeedFrequency )
		{
            setSpeedTime = xTaskGetTickCount();
			xSemaphoreTake(gs_speedToDriverMutex, portMAX_DELAY);
			gs_leftSpeedToDriver = leftSpeed*10;
			gs_rightSpeedToDriver = -rightSpeed*10;
			xSemaphoreGive(gs_speedToDriverMutex);
			
			SetSpeed(gs_leftSpeedToDriver, gs_rightSpeedToDriver);
		}
        
        //转矩模式切换
        if( GetCurCtrMode() )
            m_control_mode = MODE_SPEED;
        control_mode = m_control_mode;
        if( GetEmergencyStopState() )
            control_mode = MODE_SPEED; //急停时切到速度模式以刹车
        if( !leftSpeed && !rightSpeed )
        {
            if( control_mode != gs_leftMotorDriverState.control_mode )
                WriteControlMode(LEFT_ID, control_mode);
            if( control_mode != gs_rightMotorDriverState.control_mode )
                WriteControlMode(RIGHT_ID, control_mode);
        }
        
        //驱动器是否重新上电
        if( CheckDriverStatus())
            lastWakeTime = xTaskGetTickCount();
        
        if( xTaskGetTickCount() - set_max_current_tick >= c_set_current_frequency )
        {
            set_max_current_tick = xTaskGetTickCount();
            SetMotorMaxCurrent();
        }
		
        //驱动器类型更新
		if(m_driver_type_update_flag)
		{
			m_driver_type_update_flag = 0;
			MotorInit(0x03);
            lastWakeTime = xTaskGetTickCount();
		}
        
        //识别驱动器后两分钟内不检测过温告警
        if( m_temperature_check_delay>0 )
        {
            m_temperature_check_delay += c_frequency;
            if( m_temperature_check_delay >= 120000 )
                m_temperature_check_delay = 0;
        }
        
//		g_thread_call_count[thread_num]++;
    }
}

int InitSoftwareMotorDriver(void)
{
	if( InitSpeedMutex() != 0)
		return 1;
	if( InitSpeedToDriverMutex() != 0)
		return 1;
	return 0;
}

int InitSoftwareDecoder(void)
{
	return InitDecoderMutex();
}

static uint32_t s_hub_motor_damping = 0;
void set_hub_motor_damping(uint32_t enable)
{
   uint16_t data = 0;

   if (s_hub_motor_damping == enable)
       return;

   if (enable)
       data = 0;
   else
       data = 1;
   CanWriteSDO( LEFT_ID, 0x2117, 0x00, uint16, &data );
   CanWriteSDO( LEFT_ID, 0x3117, 0x00, uint16, &data );
   s_hub_motor_damping = enable;
}








