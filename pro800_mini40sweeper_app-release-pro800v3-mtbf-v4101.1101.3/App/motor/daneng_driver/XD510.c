#include "XD510.h"
#include "motor_driver.h"
//#include "can_task.h"
#include "objdict.h"
#include "normal_io.h"
#include "pc_correspond.h"
//#include "info_store.h"
#include "main.h"
//#include "tim.h"
#include "iwdg_task.h"
#include "nanopb_tcpip.h"
#include "hc_can.h"
#include "hc_extint.h"

/*
*********************************************************************************************************
*
*	模块名称 :XD510控制器
*	文件名称 : XD510.c
*	版    本 : 	V1.0.0
*	说    明 : 
*			1，使用CAN通信
*			
*	修改记录 :
*		版本号  		日期        作者     说明
*       V1.0.0    2019-12-16  ypc
*	外设资源占用：
*		CAN:CAN1
*********************************************************************************************************
*/

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/

#define NODE_ID 0x02

#define USE_MOTOR_1    1
#define USE_MOTOR_2    1
#define USE_MOTOR_3    0
#define USE_MOTOR_4    1
#define USE_MOTOR_5    1
#define USE_MOTOR_6    1

#define MOTOR_1_ACCEL    10

#define BRUSH_LIFT_UP_STOP_LEAD  7  //推杆电机到位检测提前量，单位霍尔脉冲值
#define BRUSH_LIFT_DOWN_STOP_LEAD  8
#define HULL_COUNT_PER_MM  7

#define BURSH_LIFT_HULL_COUNT  280



#define INDEX_MOTOR_PWM          0x3080 //设置电机pwm
#define INDEX_MOTOR_ACCEL        0x3081 //设置电机加速度
#define INDEX_MOTOR_DECEL        0x3082 //设置电机减速度
#define INDEX_MOTOR_CURRENT_PWM  0x3083 //读取电机当前pwm
#define INDEX_MOTOR_CURRENT      0x3084 //读取电机电流值
#define SUBINDEX_MOTOR_1    0x01
#define SUBINDEX_MOTOR_2    0x02
#define SUBINDEX_MOTOR_3    0x03
#define SUBINDEX_MOTOR_4    0x04
#define SUBINDEX_MOTOR_5    0x05
#define SUBINDEX_MOTOR_6    0x06



#define INDEX_MOTOR_STOP_BITS    0x3085 //电机运行控制位
#define SUBINDEX_MOTOR_STOP_BIT    0x00

#define INDEX_MOTOR_RUN_STATUS    0x3086 //读取电机运行状态位
#define SUBINDEX_MOTOR_RUN_STATUS    0x00

#define INDEX_SYS_ERROR_BITS1    0x3088 //系统故障码1
#define SUBINDEX_SYS_ERROR_BITS1    0x00

#define INDEX_SYS_ERROR_BITS2    0x3089 //系统故障码2
#define SUBINDEX_SYS_ERROR_BITS2    0x00

#define INDEX_PCB_TEMPERATURE    0x308B //驱动器温度
#define SUBINDEX_PCB_TEMPERATURE    0x00

#define INDEX_OUTPUT_PWM    0x308C //设置pwm输出占空比值
#define SUBINDEX_OUTPUT_PWM_1    0x01
#define SUBINDEX_OUTPUT_PWM_2    0x02
#define SUBINDEX_OUTPUT_PWM_3    0x03
#define SUBINDEX_OUTPUT_PWM_4    0x04
#define SUBINDEX_OUTPUT_PWM_5    0x05
#define SUBINDEX_OUTPUT_PWM_6    0x06
#define SUBINDEX_OUTPUT_PWM_7    0x07
#define SUBINDEX_OUTPUT_PWM_8    0x04
#define SUBINDEX_OUTPUT_PWM_9    0x05
#define SUBINDEX_OUTPUT_PWM_10   0x06

#define INDEX_BAT_VOLTAGE    0x3004 //驱动器电压
#define SUBINDEX_BAT_VOLTAGE    0x00

#define INDEX_XD510_VERSION  0x3206
#define SUBINDEX_XD510_VERSION  0x09



/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/
typedef enum
{
    SPARY_OFF = 0,
    SPARY_ONING,
    SPARY_ON,
    SPARY_OFFING,
}eSparyStatus_t;

typedef struct
{
    int16_t target_pwm;
    int16_t max_current;
    uint8_t enable;
}MotorCmd_t;

typedef struct
{
    int16_t current;
    uint16_t run_status;
    uint8_t overcurrent;
    uint16_t accel;
}MotorStatus_t;

/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/
static MotorCmd_t m_motor1_cmd = {0, 300, 0};
static MotorCmd_t m_motor2_cmd = {0, 300, 0};
static MotorCmd_t m_motor3_cmd = {0, 30, 0};
static MotorCmd_t m_motor4_cmd = {0, 30, 0};
static MotorCmd_t m_motor5_cmd = {0, 30, 0};
static MotorCmd_t m_motor6_cmd = {0, 30, 0};
static MotorCmd_t m_side_lift_motor_cmd = {0,70, 0}; //边刷升降控制命令，电器接口同吸水条
static MotorStatus_t m_motor1_status = {0, 0, 0};
static MotorStatus_t m_motor2_status = {0, 0, 0};
static MotorStatus_t m_motor3_status = {0, 0, 0};
static MotorStatus_t m_motor4_status = {0, 0, 0};
static MotorStatus_t m_motor5_status = {0, 0, 0};
static MotorStatus_t m_motor6_status = {0, 0, 0};
static MotorStatus_t m_side_lift_status = {0, 0, 0}; //边刷升降状态

//系统状态和故障码
static uint32_t m_motor_run_status = 0;
static uint32_t m_motor_stop_bits = 0;
static uint32_t m_sys_error_code1 = 0;
static uint32_t m_sys_error_code2 = 0;
//驱动器温度
static int16_t m_temperature = 0;
//设置pwm输出
static uint8_t m_output_pwm_1 = 0;
static uint8_t m_output_pwm_2 = 0;
static uint8_t m_output_pwm_3 = 0;
static uint8_t m_output_pwm_4 = 0;
static uint8_t m_output_pwm_5 = 0;
//static uint8_t m_output_pwm_6 = 0;
//驱动器供电电压
static int16_t m_battery_voltage = 0;

uint8_t m_vacuum_close_delay_s = 10; //急停和钥匙开关响应后关吸风延时时间

static uint32_t  m_brush_lift_cmd = 0; //刷盘升降电机命令，非零表示下放
static int16_t m_brush_lift_position_table[4] = {0,28*HULL_COUNT_PER_MM,33*HULL_COUNT_PER_MM,40*HULL_COUNT_PER_MM}; //推杆电机行程绝对脉冲数
static int16_t m_brush_lift_travel = 40*HULL_COUNT_PER_MM; //刷盘升降电机相对抬起位置的行程（霍尔脉冲数）
static int16_t m_real_brush_lift_travel = 0;//刷盘升降实时位置
static uint16_t m_brush_lift_target_position = 0;//刷盘升降目标位置，霍尔值

static uint8_t  m_side_lift_cmd = 0; //边刷升降命令，非零表示放下
static int16_t  m_side_lift_position_table[4] = {0,6*HULL_COUNT_PER_MM,8*HULL_COUNT_PER_MM,10*HULL_COUNT_PER_MM}; //边刷推杆电机行程绝对脉冲数
static int16_t  m_side_lift_travel = 10*HULL_COUNT_PER_MM; //边刷推杆电机相对抬起位置的行程（霍尔脉冲数）

static uint8_t  m_squeegee_lift_cmd = 0; //吸水条升降命令，非零表示下放
//static uint16_t m_brush_down_time_ms = 5000; //刷盘下放时间
static uint16_t m_squeegee_down_time_ms = 5000; //吸水条下放时间

static uint8_t m_robot_work_mode = 0; //机器人工作模式，0正常，1执行扫地任务，2执行充电任务，3执行加水任务

static uint8_t m_outlet_sewage_cmd = 0; //排污命令

static uint8_t m_motor_overcurrent = 0; //电机过流

static uint8_t m_xd510_callback_flag = 1;
static ConnectionStatus_t m_xd510_connect_status = {12000,2000}; //设置超时时间，初始化时不检测

static uint8_t m_motor6_port_occupy = 0;//0表示未被使用；1表示吸水条升降在使用该驱动器接口，2表示边刷升降在使用该驱动器接口

static uint32_t m_xd510_version = 0;

static void ReadCallback(void* fp_param );

static const Objdict_t m_objdict[] =
{
	{NODE_ID,  INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_1, int16, sizeof(int16_t), RO, &m_motor1_status.current, NULL },
	{NODE_ID,  INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_2, int16, sizeof(int16_t), RO, &m_motor2_status.current, NULL },
	{NODE_ID,  INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_3, int16, sizeof(int16_t), RO, &m_motor3_status.current, NULL },
	{NODE_ID,  INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_4, int16, sizeof(int16_t), RO, &m_motor4_status.current, NULL },
	{NODE_ID,  INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_5, int16, sizeof(int16_t), RO, &m_motor5_status.current, NULL },
	{NODE_ID,  INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_6, int16, sizeof(int16_t), RO, &m_motor6_status.current, NULL },
	{NODE_ID,  INDEX_MOTOR_RUN_STATUS, SUBINDEX_MOTOR_RUN_STATUS, uint32, sizeof(uint32_t), RO, &m_motor_run_status, NULL },
    {NODE_ID,  INDEX_MOTOR_STOP_BITS, SUBINDEX_MOTOR_STOP_BIT, uint32, sizeof(uint32_t), RO, &m_motor_stop_bits, NULL },
	{NODE_ID,  INDEX_SYS_ERROR_BITS1, SUBINDEX_SYS_ERROR_BITS1, uint32, sizeof(uint32_t), RO, &m_sys_error_code1, NULL },
	{NODE_ID,  INDEX_SYS_ERROR_BITS2, SUBINDEX_SYS_ERROR_BITS2, uint32, sizeof(uint32_t), RO, &m_sys_error_code2, NULL },
	{NODE_ID,  INDEX_PCB_TEMPERATURE, SUBINDEX_PCB_TEMPERATURE, int16, sizeof(int16_t), RO, &m_temperature, NULL },
	{NODE_ID,  INDEX_BAT_VOLTAGE, SUBINDEX_BAT_VOLTAGE, int16, sizeof(int16_t), RO, &m_battery_voltage, ReadCallback },
    {NODE_ID,  INDEX_MOTOR_ACCEL, SUBINDEX_MOTOR_1, uint16, sizeof(uint16_t), RW, &m_motor1_status.accel, NULL },
    {NODE_ID,  INDEX_XD510_VERSION, SUBINDEX_XD510_VERSION, uint32, sizeof(uint32_t), RW, &m_xd510_version, NULL },
};
static const uint8_t m_objdict_size = sizeof(m_objdict)/sizeof(Objdict_t);

/************************************************************************************************************************************
															静态函数声明
*************************************************************************************************************************************/
static void DeviceInit(void);

/************************************************************************************************************************************
												内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/
//电机初始化
static void DeviceInit(void)
{
    uint16_t data = 0;
#if USE_MOTOR_4
    //设置刷盘电机pwm、加速度、减速度
	CanWriteSDO( NODE_ID, INDEX_MOTOR_PWM, SUBINDEX_MOTOR_4, int16, &data );
    data = 1;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_ACCEL, SUBINDEX_MOTOR_4, uint16, &data );
    data = 5;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_DECEL, SUBINDEX_MOTOR_4, uint16, &data );
    vTaskDelay(3);
#endif
#if USE_MOTOR_5
    //设置刷盘升降电机pwm、加速度、减速度
    data = 0;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_PWM, SUBINDEX_MOTOR_5, int16, &data );
    data = 10;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_ACCEL, SUBINDEX_MOTOR_5, uint16, &data );
    data = 50; //拍下急停后能够快速响应
    CanWriteSDO( NODE_ID, INDEX_MOTOR_DECEL, SUBINDEX_MOTOR_5, uint16, &data );
    vTaskDelay(3);
#endif
#if USE_MOTOR_1
    //设置吸风电机pwm、加速度、减速度
    data = 0;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_PWM, SUBINDEX_MOTOR_1, int16, &data );
    data = MOTOR_1_ACCEL;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_ACCEL, SUBINDEX_MOTOR_1, uint16, &data );
    CanWriteSDO( NODE_ID, INDEX_MOTOR_DECEL, SUBINDEX_MOTOR_1, uint16, &data );
    vTaskDelay(3);
#endif
#if USE_MOTOR_6
    //设置吸水扒升降电机pwm、加速度、减速度
    data = 0;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_PWM, SUBINDEX_MOTOR_6, int16, &data );
    data = 5;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_ACCEL, SUBINDEX_MOTOR_6, uint16, &data );
    data = 50; //拍下急停后能够快速响应
    CanWriteSDO( NODE_ID, INDEX_MOTOR_DECEL, SUBINDEX_MOTOR_6, uint16, &data );
    vTaskDelay(3);
#endif
#if USE_MOTOR_3
    //喷水电机pwm、加速度、减速度
    data = 0;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_PWM, SUBINDEX_MOTOR_3, int16, &data );
    data = 1;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_ACCEL, SUBINDEX_MOTOR_3, uint16, &data );
    
    CanWriteSDO( NODE_ID, INDEX_MOTOR_DECEL, SUBINDEX_MOTOR_3, uint16, &data );
    vTaskDelay(3);
#endif
#if USE_MOTOR_2
    //过滤电机pwm、加速度、减速度
    data = 0;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_PWM, SUBINDEX_MOTOR_2, int16, &data );
    data = 1;
    CanWriteSDO( NODE_ID, INDEX_MOTOR_ACCEL, SUBINDEX_MOTOR_2, uint16, &data );
    CanWriteSDO( NODE_ID, INDEX_MOTOR_DECEL, SUBINDEX_MOTOR_2, uint16, &data );
    vTaskDelay(3);
#endif
}
//设置电机速度
static void SetMotorPwm( uint8_t f_subidx, int16_t f_pwm )
{
    switch( f_subidx )
    {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
            CanWriteSDO( NODE_ID, INDEX_MOTOR_PWM, f_subidx, int16, &f_pwm );
            break;
        
        default:
            break;
    }
}

static void SetOutputPwm( uint8_t f_subidx, uint8_t f_pwm )
{
    switch( f_subidx )
    {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            CanWriteSDO( NODE_ID, INDEX_OUTPUT_PWM, f_subidx, uint8, &f_pwm );
            break;
        
        default:
            break;
    }
}

#if USE_MOTOR_4
//设置尘推电机pwm
static void RunDustMotor( void )
{
	int16_t data = 0;
	
	if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
	{
		data = 0;
	}
	else if( m_motor4_cmd.enable )
	{
		data = m_motor4_cmd.target_pwm;
	}
	else
	{
		data = 0;
        m_motor4_status.run_status = 0;
	}
	
	if( data >10000 )
		data = 10000;
	if( data < -10000 )
		data = -10000;
	if( data )
    {
        m_motor4_status.run_status = data/95;
    }
    else
    {
        m_motor4_status.run_status = 0;
    }
	SetMotorPwm( SUBINDEX_MOTOR_4, data );
}
#endif

#if USE_MOTOR_1
//设置吸风电机pwm
static void RunVacuumMotor(void)
{
    static TickType_t s_last_run_tick = 0;
    
    static int16_t data = 0;
	
	if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
	{
        if( xTaskGetTickCount() - s_last_run_tick >= 1000*m_vacuum_close_delay_s )
            data = 0;
	}
	else if( m_motor1_cmd.enable )
	{
		data = m_motor1_cmd.target_pwm;
        s_last_run_tick = xTaskGetTickCount();
	}
	else
	{
		data = 0;
        m_motor1_status.run_status = 0;
	}
	
	if( data >10000 )
		data = 10000;
	if( data < -10000 )
		data = -10000;
	if( data )
    {
        m_motor1_status.run_status = 1;
    }
    else
    {
        m_motor1_status.run_status = 0;
    }
	SetMotorPwm( SUBINDEX_MOTOR_1, data );
}
#endif

#if USE_MOTOR_2
//设置尘推自洁pwm
static void RunDustClearMotor(void)
{
    int16_t data = 0;
	
	if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
	{
		data = 0;
	}
	else if( m_motor2_cmd.enable )
	{
		data = m_motor2_cmd.target_pwm;
	}
	else
	{
		data = 0;
        m_motor2_status.run_status = 0;
	}
	
	if( data >10000 )
		data = 10000;
	if( data < -10000 )
		data = -10000;
	
    if( data )
    {
        m_motor2_status.run_status = data/95;
    }
    else
    {
        m_motor2_status.run_status = 0;
    }
        
	SetMotorPwm( SUBINDEX_MOTOR_2, data );
}
#endif

#if USE_MOTOR_3
//设置喷水电机pwm
static void RunSprayMotor(void)
{
    static eSparyStatus_t s_status = SPARY_OFF;
    static TickType_t s_start_tick = 0;
    
    int16_t data = 0;
	
	if( GetEmergencyStopState() || GetKeyState() == 0x02 || !GetPcConnectStatus() )
	{
		data = 0;
	}
	else if( m_motor3_cmd.enable )
	{
		data = m_motor3_cmd.target_pwm;
	}
	else
	{
		data = 0;
        m_motor3_status.run_status = 0;
	}
	
	if( data >10000 )
		data = 10000;
	if( data < -10000 )
		data = -10000;
    
    switch( s_status )
    {
        case SPARY_OFF:
            if( data != 0 )
            {
                s_start_tick = xTaskGetTickCount();
                data = 0;
                s_status = SPARY_ONING;
                m_output_pwm_3 = 255;
                SetOutputPwm( SUBINDEX_OUTPUT_PWM_3, m_output_pwm_3 );//开电磁阀
                m_output_pwm_5 = 255;
                SetOutputPwm( SUBINDEX_OUTPUT_PWM_5, m_output_pwm_5 );//开电动阀
            }
            break;
        case SPARY_ONING:
            if( xTaskGetTickCount() - s_start_tick >= 3000 )//电动阀开启需要时间，延时等待电动阀开启
            {
                s_status = SPARY_ON;
            }
            else
            {
                data = 0;
            }
            break;
        case SPARY_ON:
            if( data == 0 )
            {
                s_start_tick = xTaskGetTickCount();
                s_status = SPARY_OFFING;
            }
            break;
        case SPARY_OFFING:
            if( m_output_pwm_3 && xTaskGetTickCount() - s_start_tick >= 500 )//延时500ms关电磁阀
            {
                m_output_pwm_3 = 0;
                SetOutputPwm( SUBINDEX_OUTPUT_PWM_3, m_output_pwm_3 );//关电磁阀
            }
            if( xTaskGetTickCount() - s_start_tick >= 10000 )//延时10s关电动阀
            {
                m_output_pwm_5 = 0;
                SetOutputPwm( SUBINDEX_OUTPUT_PWM_5, m_output_pwm_5 );//开电动阀
                s_status = SPARY_OFF;
            }
            else if( data != 0 )//等待关闭电动阀期间重新启动了喷水
            {
                m_output_pwm_3 = 255;
                SetOutputPwm( SUBINDEX_OUTPUT_PWM_3, m_output_pwm_3 );//开电磁阀
                s_status = SPARY_ON;
            }
            break;
    }
    
    if( data )
    {
        m_motor3_status.run_status = 1;
    }
    else
    {
        m_motor3_status.run_status = 0;
    }
    
    SetMotorPwm( SUBINDEX_MOTOR_3, data );
}
#endif

 
static uint16_t GetHullCount(void)
{
	    return ENC0_Count;
}
static void ResetHullCount(void)
{
		ENC0_Count = 0;
}

//刷盘升降需要移动的霍尔脉冲数
static int16_t GetDeltaTravel( int16_t f_last_cmd, int16_t f_cur_cmd )
{
    int16_t count = 0;
    int16_t count1 = 0;
    
    count = f_cur_cmd-m_real_brush_lift_travel;//根据当前刷盘位置计算要动作的距离
    count1 = f_cur_cmd-f_last_cmd;//当前命令与上次命令之间的位置差
    
    if(!( (count>0&&count1>0) || (count<0&&count1<0) ))//若两种方式计算出的运动方向不一致，很有可能是因为霍尔出了问题
    {
        count = count1;
    }
    
    if(!f_cur_cmd)
        count = -BURSH_LIFT_HULL_COUNT;//抬升时用时间控制
    
    return count;
}
//计算当前运动的最长时间
static uint16_t GetProtectTime(uint16_t f_hull_count)
{
    uint16_t time = 0;
    time = f_hull_count*13 + 500;

    return time;
}
//检查本次动作是否达到了最小动作要求
static uint8_t CheckPositionChanged(int f_cmd1, int f_cmd2)
{
    uint8_t ret = 0;
    if( ABS( f_cmd1-f_cmd2) >= HULL_COUNT_PER_MM*2 )
        ret = 1;
    return ret;
}

//滚刷推杆位置控制
static void BrushLiftMotorPositionCtr(uint8_t f_flag)
{
    static TickType_t s_operate_delay_tick = 0;
    static TickType_t s_start_tick = 0;
    static uint16_t   s_operate_time_ms = 0;
    static uint16_t   s_last_target_position = BURSH_LIFT_HULL_COUNT;//上次目标位置
    static int16_t    s_delta_target = 0;//本次要动作的
    static int16_t    s_brush_lift_travel = BURSH_LIFT_HULL_COUNT;
    static int16_t    s_last_realtime_travel = 0;
    static uint8_t    s_last_operate_compelte = 1; //上次操作是否完成，非零表示完成
    static uint8_t    s_hull_stop_lead = 0;
    
    if( GetKeyState() == 0x02 && !GetEmergencyStopState() )//非急停同时钥匙关机
	{
		m_brush_lift_target_position = 0;
	}
    //非急停模式下且上次动作已完成，同时动作行程满足最小动作值时执行
    if( !GetEmergencyStopState() && s_last_operate_compelte && CheckPositionChanged(s_last_target_position,m_brush_lift_target_position) )
    {
		//获取本次要动作的行程
        s_delta_target = GetDeltaTravel( s_last_target_position,m_brush_lift_target_position);
        
        //移动距离为0同时非回零点，此时为异常状态，不执行
        if( !s_delta_target && m_brush_lift_target_position )
        {
            s_last_operate_compelte = 1;
            m_motor5_cmd.target_pwm = 0;
        }
        else
        {
            s_operate_time_ms = GetProtectTime( ABS(s_delta_target) );//计算动作保护时间
            s_last_target_position = m_brush_lift_target_position;
            s_last_operate_compelte = 0;//设置为动作未完成
            s_start_tick = xTaskGetTickCount();//更新升降电机启动时间
            
            if( s_delta_target>0)
                s_hull_stop_lead = BRUSH_LIFT_DOWN_STOP_LEAD;
            else
                s_hull_stop_lead = BRUSH_LIFT_UP_STOP_LEAD;
            
            if( ABS(s_delta_target) <=21 )//距离太短，提前量折半
            {
                s_hull_stop_lead /=3;
                s_hull_stop_lead = 7;
            }
            else if( ABS(s_delta_target) <=28 )//距离太短，提前量折半
                s_hull_stop_lead =2*s_hull_stop_lead/3;
            
            ResetHullCount();//清脉冲计数值
            s_last_realtime_travel = m_real_brush_lift_travel;
			m_motor5_status.run_status = 2;
        }
    }
    else//通过时间和霍尔行程判断是否执行到位
    if( !s_last_operate_compelte && ( xTaskGetTickCount()-s_start_tick <= s_operate_time_ms) && \
        (GetHullCount()<=ABS(s_delta_target)-s_hull_stop_lead) )
    {
        if( GetEmergencyStopState() )
        {
            if( m_motor5_cmd.target_pwm ) //之前在运动
            {
                s_operate_time_ms = s_operate_time_ms - (xTaskGetTickCount() - s_start_tick) + 100; //急停后重新计算急停回复后运行的时间
            }
            m_motor5_cmd.target_pwm = 0;
            s_start_tick = xTaskGetTickCount();
        }
        else
        {
            m_motor5_cmd.target_pwm = (s_delta_target>0) ? 6000 : -6000;
        }
        s_operate_delay_tick = xTaskGetTickCount();
		//实时更新霍尔脉冲数
        m_real_brush_lift_travel = s_last_realtime_travel+((s_delta_target>0)? GetHullCount(): -GetHullCount());
    }
    else if( !s_last_operate_compelte )//动作完成
    {
        if( xTaskGetTickCount() - s_operate_delay_tick >= 2000)//等待设备停止动作
        {
            s_last_operate_compelte = 1;
            s_brush_lift_travel += (s_delta_target>0)? GetHullCount(): -GetHullCount();//更新本次动作行程到累计行程里
            if( !s_last_target_position )//抬起后重置累计行程
                s_brush_lift_travel = 0;
            m_real_brush_lift_travel = s_brush_lift_travel;
        }
        m_motor5_cmd.target_pwm = 0;

		
		m_motor5_status.run_status = s_last_target_position?1:0;
        f_flag = 1;
    }
    if(f_flag)
        SetMotorPwm( SUBINDEX_MOTOR_5, m_motor5_cmd.target_pwm );
}
#include "nanopb_tcpip.h"
#include "zd_driver.h"
Xd510_Self_Adaption_t Xd510_Self_Adaption={0};

//滚刷推杆自适应控制
static void BrushLiftMotorSelfCtr(uint8_t f_flag)
{
   
	static uint32_t above_tick = 0; 
	static uint32_t below_tick = 0; 
	static uint32_t last_send_tick=0;   //避免can数据发送过于频繁
	static uint8_t first_run_flag=1;

	if(first_run_flag)       //初始化last_current的值
	{
		first_run_flag=0;
		Xd510_Self_Adaption.last_current=1;
	} 
	
	if( Xd510_Self_Adaption.last_current!=Xd510_Self_Adaption.target_current)
	{
		Xd510_Self_Adaption.last_current=Xd510_Self_Adaption.target_current;
		Xd510_Self_Adaption.new_cmd_flag = 1;
	}		
	if( GetKeyState() == 0x02 && !GetEmergencyStopState() )//非急停同时钥匙关机
	{
		Xd510_Self_Adaption.target_current = 0;
	}
	if(!GetEmergencyStopState())  //未触发急停 
	{

		if(Xd510_Self_Adaption.target_current ==0 )  //抬起滚刷
		{
			Xd510_Self_Adaption.response_status = 0;	
			m_motor5_cmd.target_pwm = -9000; //抬起
		} 
		else if(Xd510_Self_Adaption.new_cmd_flag)     //自适应完成接收到新命令
		{
			Xd510_Self_Adaption.new_cmd_flag=0;
			Xd510_Self_Adaption.response_status = 12;  //推杆运行中	
		}
		else if(DeviceCommandCMD.brush_spin_level ==0 && Xd510_Self_Adaption.response_status==1) //手动任务自适应完成
		{
			Xd510_Self_Adaption.beyond_status=0;
			above_tick= HAL_GetTick();
			below_tick= HAL_GetTick();
			m_motor5_cmd.target_pwm=0; 
			Xd510_Self_Adaption.response_status = 1;
		}
		else if(DeviceCommandCMD.brush_spin_level &&Xd510_Self_Adaption.real_current < Xd510_Self_Adaption.min_current )
		{
			if(Xd510_Self_Adaption.beyond_status==1&&HAL_GetTick()-below_tick>Xd510_Self_Adaption.debounce_time)
			{
				m_motor5_cmd.target_pwm=5000; //5000放下
			}
			else if(Xd510_Self_Adaption.beyond_status!=1)
			{
				m_motor5_cmd.target_pwm=0;
				Xd510_Self_Adaption.beyond_status=1;
				below_tick= HAL_GetTick();
			}
			Xd510_Self_Adaption.response_status = 12;  //推杆运行中	
		}          
		else if(DeviceCommandCMD.brush_spin_level &&Xd510_Self_Adaption.real_current > Xd510_Self_Adaption.max_current)
		{
			if(Xd510_Self_Adaption.beyond_status==2&&HAL_GetTick()-above_tick>Xd510_Self_Adaption.debounce_time)
			{
				m_motor5_cmd.target_pwm=-5000; //-5000抬起
			}
			else if(Xd510_Self_Adaption.beyond_status!=2)
			{
				m_motor5_cmd.target_pwm=0;
				Xd510_Self_Adaption.beyond_status=2;
				above_tick= HAL_GetTick();
			}
			Xd510_Self_Adaption.response_status = 12;  //推杆运行中	
		}	 
		else if(DeviceCommandCMD.brush_spin_level)
		{	
			Xd510_Self_Adaption.beyond_status=0;
			above_tick= HAL_GetTick();
			below_tick= HAL_GetTick();
			m_motor5_cmd.target_pwm=0;
			Xd510_Self_Adaption.response_status=1;
			
		}			
	}
	else    //急停触发
	{	
		m_motor5_cmd.target_pwm = 0;
	}
	
	if(HAL_GetTick()-last_send_tick>50)
	{
		SetMotorPwm( SUBINDEX_MOTOR_5, m_motor5_cmd.target_pwm );	 
		last_send_tick = HAL_GetTick();
	}
	
}
//滚刷推杆控制总接口
static void BrushLiftMotorCtr(uint8_t f_flag)
{
	static uint8_t last_ctr_mode=0;      
	static uint32_t star_self_ctr_tick=0;
	
	Xd510_Self_Adaption.enable=ConfigCommandCMD.self_adaption_enable;
	Xd510_Self_Adaption.max_current= Xd510_Self_Adaption.target_current*(100.0+Xd510_Self_Adaption.deviation)/100;
	Xd510_Self_Adaption.min_current= Xd510_Self_Adaption.target_current*(100.0-Xd510_Self_Adaption.deviation)/100;
	Xd510_Self_Adaption.real_current = GetRollerCurrent();
	
	if(Xd510_Self_Adaption.enable)                   //使用自适应控制
	{
		if(last_ctr_mode==0)                         //位置切自适应
		{	
			last_ctr_mode=Xd510_Self_Adaption.enable;
			star_self_ctr_tick=HAL_GetTick();
		}
				
		if(HAL_GetTick()-star_self_ctr_tick<4000)
		{
			SetMotorPwm( SUBINDEX_MOTOR_5, 6000 ); //模式切换后先抬起推杆
			Xd510_Self_Adaption.response_status=0;
		}
		else
		{	
			BrushLiftMotorSelfCtr(f_flag);
		}
	}
	else                    //使用位置控制
	{
		if(last_ctr_mode) 	//自适应切位置 
		{		
			last_ctr_mode=Xd510_Self_Adaption.enable;
		}
		BrushLiftMotorPositionCtr( f_flag);	
	}
}


#if USE_MOTOR_6
static void RunSqueegeeLiftMotor(void)
{
    static uint8_t s_last_operate_compelte = 1; //上次操作是否完成，非零表示完成
    static uint32_t s_last_cmd = 1; //上次执行的命令，非零表示抬起
    static TickType_t s_start_tick = 0;
    static uint16_t s_operate_time_ms = 0;
    static TickType_t s_operate_delay_tick = 0;
    
    if( GetKeyState() == 0x02 && !GetEmergencyStopState() )//非急停同时钥匙关机
	{
		m_squeegee_lift_cmd = 0;
	}
        
	if( !m_motor6_port_occupy || m_motor6_port_occupy==2 )
	{
		if( !GetEmergencyStopState() && s_last_operate_compelte && s_last_cmd != m_squeegee_lift_cmd )
		{
			s_last_operate_compelte = 0;
			s_last_cmd = m_squeegee_lift_cmd;
			s_start_tick = xTaskGetTickCount();//更新升降电机启动时间
			if( s_last_cmd ) //下放
			{
				s_operate_time_ms = m_squeegee_down_time_ms;
			}
			else //上抬
			{
				s_operate_time_ms = 8000;
			}
			m_motor6_port_occupy = 2;
			SetOutputPwm( SUBINDEX_OUTPUT_PWM_5, 0 );
			SetOutputPwm( SUBINDEX_OUTPUT_PWM_5, 0 );
		}
		
		if( !s_last_operate_compelte && (xTaskGetTickCount() - s_start_tick <= s_operate_time_ms) )
		{
			if( GetEmergencyStopState() )
			{
				if( m_motor6_cmd.target_pwm ) //之前在运动
				{
					//s_operate_time_ms = s_operate_time_ms - (xTaskGetTickCount() - s_start_tick) + 500; //急停后重新计算急停回复后运行的时间
					s_operate_time_ms = 8000;//急停后将设备抬起，然后根据上位机命令执行
					s_last_cmd = 0;
				}
				m_motor6_cmd.target_pwm = 0;
				s_start_tick = xTaskGetTickCount();
			}
			else
			{
				m_motor6_cmd.target_pwm = s_last_cmd ? 6000 : -6000;
			}
			s_operate_delay_tick = xTaskGetTickCount();
		}
		else //动作完成
		{
			if( xTaskGetTickCount() - s_operate_delay_tick >= 3000)
			{
				s_last_operate_compelte = 1;
				m_motor6_port_occupy = 0;
				m_motor6_status.run_status = s_last_cmd;
			}
			m_motor6_cmd.target_pwm = 0;
		}
		
		if(m_motor6_port_occupy)
			SetMotorPwm( SUBINDEX_MOTOR_6, m_motor6_cmd.target_pwm );
	}
}

static uint16_t GetSideHullCount(void)
{
	return ENC0_Count;
}

static void ResetSideHullCount(void)
{
    ENC0_Count = 0;
}

static int16_t GetSideDeltaTravel(uint8_t f_current, uint8_t f_target)
{
    int16_t count = 0;
    if( f_current<4 && f_target<4 )
    {
        //count = m_side_lift_position_table[f_target] - m_side_lift_position_table[f_current];//;m_side_lift_travel;
        count = m_side_lift_position_table[f_target] - m_side_lift_travel;
    }
    return count;
}

static void RunSideBrushLiftMotor(uint8_t f_flag)
{
    static uint8_t    s_last_operate_compelte = 1; //上次操作是否完成，非零表示完成
    static uint32_t   s_last_cmd = 3; //上次执行的命令，零表示抬起
    static TickType_t s_start_tick = 0;
    static uint16_t   s_operate_time_ms = 0;
    static TickType_t s_operate_delay_tick = 0;
    static int16_t    s_delta_target = 0;
    static uint8_t    s_hull_stop_lead = 0;
    static TickType_t s_hull_check_tick = 0;
    static uint16_t   s_last_hull_count = 0;
    static uint8_t    s_lift_working = 1; //霍尔推杆是否在运动
    
    if( GetKeyState() == 0x02 && !GetEmergencyStopState() )//非急停同时钥匙关机
	{
		m_side_lift_cmd = 0;
	}
	
	if( !m_motor6_port_occupy || m_motor6_port_occupy==1 )
	{
		if( !GetEmergencyStopState() && s_last_operate_compelte && s_last_cmd != m_side_lift_cmd )
		{
			s_delta_target = GetSideDeltaTravel( s_last_cmd, m_side_lift_cmd);

			if( !s_delta_target && m_side_lift_cmd ) //异常情况，不执行（回零时执行）
			{
				s_last_operate_compelte = 1;
				m_side_lift_motor_cmd.target_pwm = 0;
				m_side_lift_status.run_status = s_last_cmd;
			}
			else
			{
				m_side_lift_status.run_status = s_last_cmd*100+m_side_lift_cmd*10;
				s_last_cmd = m_side_lift_cmd;
				s_last_operate_compelte = 0;
				s_start_tick = xTaskGetTickCount();//更新升降电机启动时间
				s_hull_check_tick = xTaskGetTickCount();
				s_last_hull_count = 0;
				s_lift_working = 1;
				s_operate_time_ms = ABS(s_delta_target)*30 + 4000;//刷盘升降控制保护时间

				if( s_delta_target>0)
					s_hull_stop_lead = BRUSH_LIFT_DOWN_STOP_LEAD;
				else
					s_hull_stop_lead = BRUSH_LIFT_UP_STOP_LEAD;
				
				ResetSideHullCount();//清脉冲计数值
				m_motor6_port_occupy = 1;
				SetOutputPwm( SUBINDEX_OUTPUT_PWM_5, 100 );
				SetOutputPwm( SUBINDEX_OUTPUT_PWM_5, 100 );
			}
		}
		
		//连续一秒霍尔计数未改变则认为电机执行到位
		if( !s_last_operate_compelte && s_lift_working && xTaskGetTickCount()-s_hull_check_tick >= 1500)
		{
			s_hull_check_tick = xTaskGetTickCount();
			if( !GetEmergencyStopState() )
			{
				if( ABS(GetSideHullCount()-s_last_hull_count) < HULL_COUNT_PER_MM )
					s_lift_working = 0;
				else
					s_last_hull_count = GetSideHullCount();
			}
		}

		//执行未完成且未超时且霍尔计数未到
		if( !s_last_operate_compelte && ( xTaskGetTickCount()-s_start_tick <= s_operate_time_ms) && (GetSideHullCount()<=ABS(s_delta_target)-s_hull_stop_lead) && s_lift_working )
		{
			if( GetEmergencyStopState() )
			{
				if( m_side_lift_motor_cmd.target_pwm ) //之前在运动
				{
					s_operate_time_ms = s_operate_time_ms - (xTaskGetTickCount() - s_start_tick) + 500; //急停后重新计算急停回复后运行的时间
				}
				m_side_lift_motor_cmd.target_pwm = 0;
				s_start_tick = xTaskGetTickCount();
			}
			else
			{
				m_side_lift_motor_cmd.target_pwm = (s_delta_target>0) ? -6000 : 6000;
			}
			s_operate_delay_tick = xTaskGetTickCount();
		}
		else if( !s_last_operate_compelte )//动作完成
		{
			if( xTaskGetTickCount() - s_operate_delay_tick >= 3000)
			{
				s_last_operate_compelte = 1;
				m_side_lift_travel += (s_delta_target>0)? GetSideHullCount(): -GetSideHullCount();
				if( !s_last_cmd )
					m_side_lift_travel = 0; //零位置霍尔计数校准
	//            printf("%d,%d\r\n", m_side_lift_travel,GetSideHullCount());
				m_motor6_port_occupy = 0;
			}
			m_side_lift_status.run_status = s_last_cmd;
			m_side_lift_motor_cmd.target_pwm = 0;
			f_flag = 1;
			s_lift_working = 0;
		}
		
		if(f_flag && m_motor6_port_occupy)
			SetMotorPwm( SUBINDEX_MOTOR_6, m_side_lift_motor_cmd.target_pwm );
	}
}
    
#endif

//电机电流检测
static void CurrentCheck(void)
{
    static TickType_t s_overcurrent_start_1 = 0;
//    static TickType_t s_overcurrent_start_2 = 0;
//    static TickType_t s_overcurrent_start_3 = 0;
    static TickType_t s_overcurrent_start_4 = 0;
    static TickType_t s_overcurrent_start_5 = 0;
    static TickType_t s_overcurrent_start_6 = 0;
    
    uint8_t flag = 0;
    uint32_t stop = 0xFFFF;
    
    
    if( (m_motor1_status.current > m_motor1_cmd.max_current) && (xTaskGetTickCount() - s_overcurrent_start_1 >= 5000) )
    {
        m_motor1_status.overcurrent = 1;
    }
    else if( m_motor1_status.current <= m_motor1_cmd.max_current)
    {
        s_overcurrent_start_1 = xTaskGetTickCount();
    }
//    if( (m_motor2_status.current > m_motor2_cmd.max_current) && (xTaskGetTickCount() - s_overcurrent_start_2 >= 5000) )//电机2同电机1电流相同
//    {
//        m_motor2_status.overcurrent = 1;
//    }
//    else if( m_motor2_status.current <= m_motor2_cmd.max_current)
//    {
//        s_overcurrent_start_2 = xTaskGetTickCount();
//    }
//    if( (m_motor3_status.current > m_motor3_cmd.max_current) && (xTaskGetTickCount() - s_overcurrent_start_3 >= 5000) )
//    {
//        m_motor3_status.overcurrent = 1;
//    }
//    else if( m_motor3_status.current <= m_motor3_cmd.max_current)
//    {
//        s_overcurrent_start_3 = xTaskGetTickCount();
//    }
    if( (m_motor4_status.current > m_motor4_cmd.max_current) && (xTaskGetTickCount() - s_overcurrent_start_4 >= 5000) )
    {
        m_motor4_status.overcurrent = 1;
    }
    else if(m_motor4_status.current <= m_motor4_cmd.max_current)
    {
        s_overcurrent_start_4 = xTaskGetTickCount();
    }
    if( (m_motor5_status.current > m_motor5_cmd.max_current) && (xTaskGetTickCount() - s_overcurrent_start_5 >= 3000) )
    {
        m_motor5_status.overcurrent = 1;
    }
    else if( m_motor5_status.current <= m_motor5_cmd.max_current)
    {
        s_overcurrent_start_5 = xTaskGetTickCount();
    }
    if( (m_motor6_status.current > m_motor6_cmd.max_current) && (xTaskGetTickCount() - s_overcurrent_start_6 >= 3000) )
    {
        m_motor6_status.overcurrent = 1;
    }
    else if( m_motor6_status.current <= m_motor6_cmd.max_current)
    {
        s_overcurrent_start_6 = xTaskGetTickCount();
    }
    
    flag |= m_motor1_status.overcurrent;
//    flag |= m_motor2_status.overcurrent;
    flag |= m_motor3_status.overcurrent;
    flag |= m_motor4_status.overcurrent;
    flag |= m_motor5_status.overcurrent;
    flag |= m_motor6_status.overcurrent;
    
    if( flag && !m_motor_stop_bits)
    {
        m_motor_overcurrent = 1;
        CanWriteSDO( NODE_ID, INDEX_MOTOR_STOP_BITS, SUBINDEX_MOTOR_STOP_BIT, uint32, &stop );
    }
    else if( !m_motor_overcurrent && m_motor_stop_bits )
    {
        m_motor1_status.overcurrent = 0;
//        m_motor2_status.overcurrent = 0;
        m_motor3_status.overcurrent = 0;
        m_motor4_status.overcurrent = 0;
        m_motor5_status.overcurrent = 0;
        m_motor6_status.overcurrent = 0;
        stop = 0;
        CanWriteSDO( NODE_ID, INDEX_MOTOR_STOP_BITS, SUBINDEX_MOTOR_STOP_BIT, uint32, &stop );
    }
    
}

//运行电机任务
static void RunMotorTask(void)
{
    static uint8_t s_idx = 0;

    s_idx++;
    
    switch( s_idx )
    {
        case 1:
            #if USE_MOTOR_4
            RunDustMotor();
            s_idx = 1;
            break;
            #endif
        case 2:
            #if USE_MOTOR_1
            RunVacuumMotor();
            s_idx = 2;
            break;
            #endif
        case 3:
            #if USE_MOTOR_2
            RunDustClearMotor();
            s_idx = 3;
            break;
            #endif
        case 4:
            #if USE_MOTOR_3
            RunSprayMotor();
            s_idx = 4;
            break;
            #endif
        case 5:
            #if USE_MOTOR_5
            BrushLiftMotorCtr(1);
            s_idx = 5;
            break;
            #endif
        case 6:
            #if USE_MOTOR_6
            RunSqueegeeLiftMotor();
			RunSideBrushLiftMotor(1);
            s_idx = 6;
            break;
            #endif
        default:
            s_idx = 0;
            break;
    }
}

//设置LED灯条
static void SetLedPwm(void)
{
    static uint8_t s_breath = 1;
    if( GetEmergencyStopState() )//急停，红灯亮蓝灯不亮
    {
        m_output_pwm_1 = 100; //红色灯条
        m_output_pwm_2 = 0; //蓝色灯条
    }
    else if( !GetPcConnectStatus() ) //上下位机断链，红灯亮蓝灯不亮
    {
        m_output_pwm_1 = 100; //红色灯条
        m_output_pwm_2 = 0; //蓝色灯条
    }
    else if( m_robot_work_mode == 1 )//扫地任务
    {
        if( s_breath )
        {
            m_output_pwm_2 += 20; 
            if( m_output_pwm_2 >= 100 )
            {
                s_breath = 0;
            }
        }
        else
        {
            m_output_pwm_2 -= 20; 
            if( m_output_pwm_2 < 20 )
            {
                s_breath = 1;
            }
        }
        m_output_pwm_1 = 0; //红色灯条灭
        
    }
    else
    {
        m_output_pwm_1 = 0; //红色灯条灭
        m_output_pwm_2 = 100; //蓝色灯条
    }
}

static void SetOutletSewagePwm(void)
{
    if( GetEmergencyStopState() || GetKeyState() == 0x02 )
	{
		m_output_pwm_4 = 0;
	}
    else
    {
        if( m_outlet_sewage_cmd )
        {
            m_output_pwm_4 = 100;
        }
        else
        {
            m_output_pwm_4 = 0;
        }
    }
}

//运行驱动器outputpwm输出
static void RunOutputTask(void)
{
    static uint8_t s_idx = 0;

    s_idx++;
    switch( s_idx )
    {
        case 1:
            SetLedPwm();
            SetOutputPwm( SUBINDEX_OUTPUT_PWM_1, m_output_pwm_1 );
            break;
        
        case 2:
            SetOutputPwm( SUBINDEX_OUTPUT_PWM_2, m_output_pwm_2 );
            break;
        
        case 3:
            SetOutputPwm( SUBINDEX_OUTPUT_PWM_3, m_output_pwm_3 );
            break;
        
        case 4:
            SetOutletSewagePwm();
            SetOutputPwm( SUBINDEX_OUTPUT_PWM_4, m_output_pwm_4 );
            break;
        
        case 5:
//            SetOutputPwm( SUBINDEX_OUTPUT_PWM_5, m_output_pwm_5 ); //在另一处调用
            break;
        
        default:
            s_idx = 0;
            break;
    }
}
//读取电机电流和状态信息
static void ReadXd510StatusTask(void)
{
    static uint8_t s_idx = 0;
    
    CurrentCheck();
    
    s_idx++;
    
    switch( s_idx )
    {
        case 1:
            CanReadSDO( NODE_ID, INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_1, int16, &m_motor1_status.current );
            break;
        case 2:
            CanReadSDO( NODE_ID, INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_2, int16, &m_motor2_status.current );
            break;
        case 3:
            CanReadSDO( NODE_ID, INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_3, int16, &m_motor3_status.current );
            break;
        case 4:
            CanReadSDO( NODE_ID, INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_4, int16, &m_motor4_status.current );
            break;
        case 5:
            CanReadSDO( NODE_ID, INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_5, int16, &m_motor5_status.current );
            break;
        case 6:
            CanReadSDO( NODE_ID, INDEX_MOTOR_CURRENT, SUBINDEX_MOTOR_6, int16, &m_motor6_status.current );
            break;
        case 7:
            CanReadSDO( NODE_ID, INDEX_MOTOR_RUN_STATUS, SUBINDEX_MOTOR_RUN_STATUS, uint32, &m_motor_run_status );
            break;
        case 8:
            CanReadSDO( NODE_ID, INDEX_SYS_ERROR_BITS1, SUBINDEX_SYS_ERROR_BITS1, uint32, &m_sys_error_code1 );
            break;
        case 9:
            CanReadSDO( NODE_ID, INDEX_SYS_ERROR_BITS2, SUBINDEX_SYS_ERROR_BITS2, uint32, &m_sys_error_code2 );
            break;
        case 10:
            CanReadSDO( NODE_ID, INDEX_PCB_TEMPERATURE, SUBINDEX_PCB_TEMPERATURE, int16, &m_temperature );
            break;
        case 11:
            CanReadSDO( NODE_ID, INDEX_BAT_VOLTAGE, SUBINDEX_BAT_VOLTAGE, int16, &m_battery_voltage );
            break;
        case 12:
            CanReadSDO( NODE_ID, INDEX_MOTOR_STOP_BITS, SUBINDEX_MOTOR_STOP_BIT, uint32, &m_motor_stop_bits );
            break;
        case 13:
            CanReadSDO( NODE_ID, INDEX_MOTOR_ACCEL, SUBINDEX_MOTOR_1, uint16, &m_motor1_status.accel );
            break;
        case 14:
            if( !m_xd510_version )
                CanReadSDO( NODE_ID, INDEX_XD510_VERSION, SUBINDEX_XD510_VERSION, uint32, &m_xd510_version );
            break;
        default:
            s_idx = 0;
            break;
    }
}

static void ReadCallback(void* fp_param )
{
    m_xd510_callback_flag = 1;
}

static void CheckDriverInitStatus(void)
{
    if( GetLinkStatus( &m_xd510_connect_status ) && m_motor1_status.accel != MOTOR_1_ACCEL )
    {
        DeviceInit();
    }
}
/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/
void SetDustMotor( uint32_t f_pwm )
{
    m_motor4_cmd.enable = f_pwm ? 1 : 0;
    m_motor4_cmd.target_pwm = f_pwm*95;
}

void SetVacuumMotor( uint32_t f_enable, uint32_t f_pwm )
{
    m_motor1_cmd.enable = f_enable ? 1 : 0;
    m_motor1_cmd.target_pwm = f_pwm*95;
}

void SetSprayMotor( uint32_t f_enable, uint32_t f_pwm )
{
    m_motor3_cmd.enable = f_enable ? 1 : 0;
    m_motor3_cmd.target_pwm = f_pwm*95;
}

void SetDustClearMotor( uint32_t f_pwm )
{
    m_motor2_cmd.enable = f_pwm ? 1 : 0;
    m_motor2_cmd.target_pwm = f_pwm*95;
}

void SetBrushLift( uint32_t f_cmd )
{
    m_brush_lift_cmd = f_cmd;
//    m_brush_down_time_ms = f_level*80;
}

void SetSideBrushLift( uint32_t f_cmd )
{
	m_side_lift_cmd = f_cmd;
}

void SetSqueegeeLift( uint32_t f_cmd )
{
    m_squeegee_lift_cmd = f_cmd?1:0;
    m_squeegee_down_time_ms = 7000;
}

void SetVacuumCloseDelay( uint32_t f_time )
{
    m_vacuum_close_delay_s = (uint8_t)f_time;
}

void SetRobotWorkMode( uint32_t f_mode )
{
    m_robot_work_mode = f_mode;
}

void SetOutletSewageCmd( uint32_t f_cmd )
{
    m_outlet_sewage_cmd = f_cmd?1:0;
}

void SetHullLiftTravelTable(uint16_t f_low, uint16_t f_median, uint16_t f_high)
{
    if( f_high>f_median && f_median>f_low )
    {
        m_brush_lift_position_table[1] = f_low*HULL_COUNT_PER_MM;
        m_brush_lift_position_table[2] = f_median*HULL_COUNT_PER_MM;
        m_brush_lift_position_table[3] = f_high*HULL_COUNT_PER_MM;
    }
}

void SetSideLiftTravelTable(uint16_t f_low, uint16_t f_median, uint16_t f_high)
{
    if( f_high>f_median && f_median>f_low )
    {
        m_side_lift_position_table[1] = f_low*HULL_COUNT_PER_MM;
        m_side_lift_position_table[2] = f_median*HULL_COUNT_PER_MM;
        m_side_lift_position_table[3] = f_high*HULL_COUNT_PER_MM;
    }
}

void SetVacuumMaxCurrent(uint32_t f_max)
{
    if( f_max>=5000)
        m_motor1_cmd.max_current = f_max/100;
}

void SetBrushLiftMaxCurrent(uint32_t f_max)
{
    if( f_max>=1000)
        m_motor5_cmd.max_current = f_max/100;
}

void SetSqueegeeLiftMaxCurrent(uint32_t f_max)
{
    if( f_max>=1000)
        m_motor6_cmd.max_current = f_max/100;
}

uint8_t GetDustMotorWorkStatus( void )
{
    return m_motor4_status.run_status;
}

uint8_t GetVacuumMotorWorkStatus(void)
{
    return m_motor1_status.run_status;
}

uint8_t GetSprayMotorWorkStatus(void)
{
    return m_motor3_status.run_status;
}

uint8_t GetDustClearMotorWorkStatus(void)
{
    return m_motor2_status.run_status;
}

uint16_t GetBrushLiftStatus(void)
{
	if(Xd510_Self_Adaption.enable == 1  )
	{	
		return Xd510_Self_Adaption.response_status;
	}
    return m_motor5_status.run_status;
}

uint8_t GetSqueegeeLiftStatus(void)
{
    return m_motor6_status.run_status;
}

uint16_t GetSideBrushLiftStatus(void)
{
	return m_side_lift_status.run_status;
}

uint32_t GetXd510SysErrBits1(void)
{
    return m_sys_error_code1&0xCE1E7FFF;
}

uint32_t GetXd510SysErrBits2(void)
{
    return m_sys_error_code2&0xFF1CFFFF;
}

uint32_t GetXd510SysErrSelf(void)
{
    uint32_t error = 0;
    
    error |= (m_sys_error_code1&0x00010000)?0x01:0;
    error |= (m_sys_error_code1&0x00200000)?0x02:0;
    error |= (m_sys_error_code1&0x00800000)?0x04:0;
    error |= (m_sys_error_code2&0x00020000)?0x08:0;
    error |= (m_sys_error_code2&0x00200000)?0x10:0;
    error |= (m_sys_error_code2&0x00800000)?0x20:0;
    return error;
}

uint16_t GetBatteryVoltage(void)
{
    return m_battery_voltage;
}

int16_t GetXd510Temperature(void)
{
    return m_temperature;
}

int16_t GetDustMotorCurrent(void)
{
    return m_motor4_status.current;
}

int16_t GetVacuumMotorCurrent(void)
{
    return m_motor1_status.current*10; //上报单位0.01A
}

int16_t GetSprayMotorCurrent(void)
{
    return m_motor3_status.run_status ? m_motor3_status.current:0;//喷水电机在稳定运行时电流小于1A，所以在此上次的电流单位为0.1A
}

int16_t GetDustClearMotorCurrent(void)
{
    return m_motor2_status.current;
}

int16_t GetBrushLiftMotorCurrent(void)
{
    return m_motor5_status.current/10;
}

int16_t GetSqueegeeLiftMotorCurrent(void)
{
    return m_motor6_status.current/10;
}
uint8_t GetValveStatus(void)
{
    return ( m_output_pwm_3 && m_output_pwm_5 )?1:0;
}
void GetXd510Version(uint32_t *fp_version )
{
    if( fp_version != NULL )
    {
        fp_version[0] = 1;
        fp_version[2] = m_xd510_version/100;
        fp_version[3] = m_xd510_version%100;
    }
}
int16_t GetBrushLiftHullCountReal(void)
{
    return m_real_brush_lift_travel;
}
uint16_t GetDeviceMotorCurrentAlarm(void)
{
    uint16_t alarm = 0;

    alarm |= (m_motor3_status.overcurrent&0x01)<<1;
    alarm |= (m_motor5_status.overcurrent&0x01)<<5;
    alarm |= (m_motor6_status.overcurrent&0x01)<<6;
    alarm |= (m_motor1_status.overcurrent&0x01)<<7;
    alarm |= (m_motor4_status.overcurrent&0x01)<<8;
    alarm |= (m_motor2_status.overcurrent&0x01)<<9;
    return alarm;
}
void GetXd510motorPwm( int16_t f_pwm[] )
{
    if( NULL != f_pwm )
    {
        f_pwm[0] = m_motor1_cmd.target_pwm;
        f_pwm[1] = m_motor2_cmd.target_pwm;
        f_pwm[2] = m_motor3_cmd.target_pwm;
        f_pwm[3] = m_motor4_cmd.target_pwm;
        f_pwm[4] = m_motor5_cmd.target_pwm;
        f_pwm[5] = m_motor6_cmd.target_pwm;
    }
}

void GetXd510OutputPwm( uint8_t f_pwm[] )
{
    if( NULL != f_pwm )
    {
        f_pwm[0] = m_output_pwm_1;
        f_pwm[1] = m_output_pwm_2;
        f_pwm[2] = m_output_pwm_3;
        f_pwm[3] = m_output_pwm_4;
        f_pwm[4] = m_output_pwm_5;
    }
}

void ClearXd510Overcurrent(void)
{
    m_motor_overcurrent = 0;
}

int GetXd510HealthStatus(void)
{
    return GetLinkStatus( &m_xd510_connect_status );
}

void GetHullTravelTable(uint16_t fp_travel[])
{
    if( NULL != fp_travel)
    {
        fp_travel[0] = m_brush_lift_position_table[1];
        fp_travel[1] = m_brush_lift_position_table[2];
        fp_travel[2] = m_brush_lift_position_table[3];
        fp_travel[3] = m_brush_lift_travel;
    }
}
//刷盘升降位置模式设置
void SetBrushLiftPosition( uint32_t f_position)
{
        m_brush_lift_target_position = f_position*HULL_COUNT_PER_MM;
}
void GetSideTravelTable(uint16_t fp_travel[])
{
    if( NULL != fp_travel)
    {
        fp_travel[0] = m_side_lift_position_table[1];
        fp_travel[1] = m_side_lift_position_table[2];
        fp_travel[2] = m_side_lift_position_table[3];
        fp_travel[3] = m_side_lift_travel;
    }
}

void Xd510Debug( uint8_t f_idx )
{
    switch( f_idx )
    {
        case 1:
            SetDustMotor(50);
            break;
        case 2:
            SetDustMotor( 0 );
            break;
        case 3:
            SetBrushLift( 0 );
            break;
        case 4:
            SetBrushLift( 1 );
            break;
        case 5:
            SetBrushLift( 2);
            break;
        case 6:
            SetBrushLift( 3 );
            break;
        case 7:
            SetSideBrushLift(0);
            break;
        case 8:
            SetSideBrushLift(1);
            break;
        case 9:
            SetSideBrushLift(2);
            break;
        case 10:
            SetSideBrushLift(3);
            break;
        case 11:
            SetDustClearMotor(50);
            break;
        case 12:
            SetDustClearMotor(0);
            break;
        case 13:
            SetSqueegeeLift( 1 );
            break;
        case 14:
            SetSqueegeeLift( 0 );
            break;
        default:
            break;
    }
}



/************************************************************************************************************************************
												任务+对外的硬件初始化+对外的软件初始化
*************************************************************************************************************************************/
/*
*********************************************************************************************************
*	函 数 名: Xd510Task
*	功能说明: 	处理xd510上所接的设备
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 
*********************************************************************************************************
*/
void Xd510Task(void const *pvParameters)
{
	TickType_t lastWakeTime;
    TickType_t output_start_tick;
    TickType_t read_status_tick;
    TickType_t check_driver_tick;
    TickType_t run_motor_tick;
	const TickType_t frequency = pdMS_TO_TICKS(10);
    const TickType_t output_frequency = pdMS_TO_TICKS(30);
    const TickType_t read_frequency = pdMS_TO_TICKS(50);
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    
    
    vTaskDelay(7000);
    //初始化配置
    DeviceInit();
	DeviceDATA.hull_disconnect_flag = 0;
	/*检测推杆电机霍尔健康状态*/
	SetMotorPwm( SUBINDEX_MOTOR_5, 8000 );
	vTaskDelay(500);
	SetMotorPwm( SUBINDEX_MOTOR_5, -8000 );
	vTaskDelay(500);
	if(ENC0_Count == 0)
	{
		DeviceDATA.hull_disconnect_flag = 1;
	}	
    vTaskDelay(3000);

	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
    output_start_tick = lastWakeTime;
    read_status_tick = lastWakeTime;
    check_driver_tick = lastWakeTime;
    run_motor_tick = lastWakeTime;
    (void)bitPos;
	
    while(1)
    {
		while(ConfigCommandCMD.motor_driver_type != 0)
		{
			vTaskDelay(500);
		}
		/* 喂狗 */
//		FeedDog(bitPos);
        //滚刷升降控制检测中，提高控制速度
        if( m_motor5_status.run_status >= 10 )
        {
            BrushLiftMotorCtr(0);
        }
		if( m_side_lift_status.run_status >= 10 )
			RunSideBrushLiftMotor(0);
        
        if( xTaskGetTickCount() - run_motor_tick >= 30)
        {
            run_motor_tick = xTaskGetTickCount();
            RunMotorTask();
        }
       
        if( xTaskGetTickCount() - output_start_tick >= output_frequency )
        {
            output_start_tick = xTaskGetTickCount();
            RunOutputTask();
        }
        
        if( xTaskGetTickCount() - read_status_tick >= read_frequency )
        {
            read_status_tick = xTaskGetTickCount();
            ReadXd510StatusTask();
        }
        
        if( m_xd510_callback_flag )
        {
            m_xd510_callback_flag = 0;
            ResetConnectedTime( &m_xd510_connect_status );
        }
        
        if( xTaskGetTickCount() - check_driver_tick >= 2000 )
        {
            check_driver_tick = xTaskGetTickCount();
            CheckDriverInitStatus();
        }
        
//        g_thread_call_count[thread_num]++;
        
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelay( frequency);
    }
}

int InitSoftwareXd510(void)
{
    ObjdictLoad( m_objdict, m_objdict_size );
    
    return 0;
}







