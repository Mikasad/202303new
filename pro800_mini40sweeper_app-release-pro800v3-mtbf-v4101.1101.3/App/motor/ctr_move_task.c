
/*
*********************************************************************************************************
*
*	模块名称 : 控制运动模块
*	文件名称 : ctr_move_task.c
*	版    本 : 	
*	说    明 : 
*			1，该模块不涉及具体的硬件，主要是综合各个传感器信息、上位机信息、遥控器信息、电机驱动信息等然后对运动进行控制
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.0    2018-11-05  杨臣   	正式发布
*		V1.0.1    2018-11-26  杨臣	添加了工作模式错误码的设置和清除
*		V1.2.0	  2018-12-13  杨臣  	1，添加了手推模式；2，添加了对sensorhub刹车释放的处理逻辑；3，添加了新的控制模式，命令行模式
*		V1.2.1	  2018-12-14  杨臣	1，添加了一个命令行模式的刹车控制指令，用来在命令行模式时控制刹车的失能/使能。
*		V1.2.2	  2018-12-18  杨臣	1，命令行控制模式中，关掉了防跌落对运动的限制
*       V1.3.0    2019-12-23  ypc
*	外设资源占用：
*		无
*	待优化：（需要优化但是未优化之前请保留该提示）
*		1,暂无
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*刹车状态有两种：
*	1，电机的刹车状态
*		电机刹车指电机的刹车管脚或者刹车命令，辅助速度=0时制动状态不好的情况
*	2，机器人的刹车状态
*		机器人刹车状态值接收到了刹车命令而执行的电机和刹车结构的刹车
*********************************************************************************************************
*/

#include <stdlib.h>
#include "main.h"
#include "nanopb_tcpip.h"
#include "motor_driver.h"
#include "security_state.h"
#include "ctr_move_task.h"
#include "sensor_hub.h"
#include "sensor.h"
#include "pc_correspond.h"
#include "normal_io.h"
#include "iwdg_task.h"
#include "anti_drop_collision.h"
#include "anti_collision.h"
#include "rfid_etag.h"


static en_functional_state_t gs_robotBrkState = DISABLE;//机器人的刹车状态
static uint8_t m_robot_run_mode = 0;
static uint8_t m_mode_check_flag = 1;

static void SetRobotBrkState(brkStatus_t f_brkStatus);
static en_functional_state_t GetRobotBrkState(void);
static uint32_t CheckRobotMoveDir(int32_t f_leftSpeed, int32_t f_rightSpeed);
static void SetRobotBrkInTrolleyMode(brkStatus_t f_brkStatus);
static uint32_t CheckForbidMoveDirRobotBrk(void);
static void SetRobotBrk(controlMode_t f_ctrMode, brkStatus_t f_brkStatus);


static void SetRobotBrkState(brkStatus_t f_brkStatus)
{
	switch(f_brkStatus)
	{
		case BRK_STATUS_NO_BRK:
			gs_robotBrkState = DISABLE;
		break;
		case BRK_STATUS_ALL_BRK:
			gs_robotBrkState = ENABLE;
		break;
		case BRK_STATUS_LEFT_BRK:
			gs_robotBrkState = DISABLE;
		break;
		case BRK_STATUS_RIGHT_BRK:
			gs_robotBrkState = DISABLE;
		break;
		default:
		break;
	}
}

static en_functional_state_t GetRobotBrkState(void)
{
	return gs_robotBrkState;
}

/*
*********************************************************************************************************
*	函 数 名: SetRobotBrk
*	功能说明: 设置刹车状态，
*	形    参: 
*		f_ctrMode：设置刹车状态的生效模式，并非当前机器人的工作模式，当生效与当前模式相同时才生效
*		f_brkStatus：刹车状态
*	返 回 值:将要移动的方向
*********************************************************************************************************
*/
static void SetRobotBrk(controlMode_t f_ctrMode, brkStatus_t f_brkStatus)
{
	if(GetCurCtrMode() != f_ctrMode)
		return;
	//1,执行刹车命令:1,驱动器刹车；2，物理刹车片刹车
	CfgDriverBrk(f_brkStatus);
	//2,设置刹车标记
	SetRobotBrkState(f_brkStatus);
}

static uint32_t CheckForbidMoveDirRobotBrk(void)
{
	uint32_t ret = 0;
	if(GetRobotBrkState() == ENABLE)
		ret = 0xFFFF;
	else
		ret = 0;
	return ret;
}

static void SetRobotBrkInTrolleyMode(brkStatus_t f_brkStatus)
{
	SetRobotBrk(CTR_MODE_TROLLEY, f_brkStatus);
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 通过轮子速度检查机器人的运行方向，目前只有前后和原地打转、停止
*	形    参: 
*	返 回 值:将要移动的方向
*********************************************************************************************************
*/
static uint32_t CheckRobotMoveDir(int32_t f_leftSpeed, int32_t f_rightSpeed)
{
	if(f_leftSpeed > 0 && f_rightSpeed > 0)//总体向前认为向前
		return ROBOT_MOVE_DIR_FRONT;
	else if(f_leftSpeed < 0 && f_rightSpeed < 0)//总体向后认为向后
		return ROBOT_MOVE_DIR_BACK;
	else if(f_leftSpeed == 0 && f_rightSpeed == 0)
		return ROBOT_MOVE_DIR_STOP;
	else if((abs(f_leftSpeed) == abs(f_rightSpeed)) && (f_leftSpeed) != (f_rightSpeed))
		return ROBOT_MOVE_DIR_SPOTTURN;
	else if(abs(f_leftSpeed) > abs(f_rightSpeed))
	{
		 if(f_leftSpeed > 0)
			 return ROBOT_MOVE_DIR_RIGHTFRONT;
		 else
			 return ROBOT_MOVE_DIR_RIGHTBACK;
	}
	else //abs(f_leftSpeed) < abs(f_rightSpeed),不会相等
	{
		 if(f_rightSpeed > 0)
			 return ROBOT_MOVE_DIR_LEFTFRONT;
		 else
			 return ROBOT_MOVE_DIR_LEFTBACK;
	}
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 	手推模式下的操作，失能驱动器，失能所有刹车机构，让机器人可以被推动
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
static void CtrMoveInTrolleyMode(void)
{
    static uint8_t s_had_speed_flag = 0;
    int32_t leftSpeed,rightSpeed;
    int car_move_status = 0;
    
    car_move_status = GetCarMoveStatus();

    if(GetEmergencyStopState() )
    {
        CfgDriverEnable(ENABLE);
        WriteSpeedVal(0, 0);
    }
    else if( m_mode_check_flag && car_move_status ) //自动切换手动时需要将车刹停
    {
        CfgDriverEnable(ENABLE);
        WriteSpeedVal(0, 0);
    }
    else
    {
		GetPcSpeed(&leftSpeed,&rightSpeed);
        if( leftSpeed || rightSpeed )
        {
            s_had_speed_flag = 1;
            CfgDriverEnable(ENABLE);
            WriteSpeedVal(leftSpeed, rightSpeed);
        }
        else
        {
            WriteSpeedVal(0, 0);
            if( s_had_speed_flag && car_move_status )//手动模式下响应上位机速度结束后需要将车刹停
            {
                CfgDriverEnable(ENABLE);
            }
            else
            {
                CfgDriverEnable(DISABLE);
            }
        }
    }
    if(m_mode_check_flag && !car_move_status ) //将车刹停
    {
        m_mode_check_flag = 0;
    }
    
    if( s_had_speed_flag && !car_move_status ) //将车刹停
    {
        s_had_speed_flag = 0;
    }
	//失能所有的刹车
	SetRobotBrkInTrolleyMode(BRK_STATUS_NO_BRK);
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 自动模式下的操作，需要考虑到所有的情况
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
static void CtrMoveInAutoMode(void)
{
	int32_t leftSpeed,rightSpeed;
	uint32_t moveDir = 0;
	uint32_t forbidMoveDir = 0;

    m_mode_check_flag = 1;

	//使能驱动器
	CfgDriverEnable(ENABLE);
	//1,获取PC下发的速度
	GetPcSpeed(&leftSpeed,&rightSpeed);
	//2,判断期望的运动方向
	moveDir = CheckRobotMoveDir(leftSpeed, rightSpeed);
	
	forbidMoveDir |= CheckForbidMoveDirByKey();//钥匙开关对运动方向的限制
	forbidMoveDir |= CheckForbidMoveDirRobotBrk();//机器人刹车状态
	forbidMoveDir |= CheckForbidMoveDirByEmergencyStop();//急停开关
	//4,比较期望运动的方向和禁止运动的方向是否冲突
    if( GetAntiCollisionStop() || GetAntiDropStop() || GetRfidProtectFlag())
    {
        forbidMoveDir = 0xFFFF;
    }
	forbidMoveDir = ~forbidMoveDir;
	if((forbidMoveDir & moveDir) != moveDir)//判断电机的运行方向是不是当前禁止的
	{
		leftSpeed = 0;
		rightSpeed = 0;
	}

	//5,写入速度到变量，具体的速度到驱动器由驱动器任务控制
	WriteSpeedVal(leftSpeed, rightSpeed);
}

void SetRobotBrkInAutoMode(brkStatus_t f_brkStatus)
{
	SetRobotBrk(CTR_MODE_AUTO, f_brkStatus);
}

//设置手自动模式
void SetRobotTrolleyMode( uint8_t f_mode )
{
    if( f_mode )
    {
        m_robot_run_mode = 1;
    }
    else
    {
        m_robot_run_mode = 0;
    }
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 获取控制模式
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
controlMode_t GetCurCtrMode(void)
{
	if( m_robot_run_mode )
		return CTR_MODE_AUTO;
	else
		return CTR_MODE_TROLLEY;
}

/*
*********************************************************************************************************
*	函 数 名: CtrMoveTask
*	功能说明: 控制移动的任务	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 高 
*********************************************************************************************************
*/
void CtrMoveTask(void const *pvParameters)
{
	const TickType_t frequency = pdMS_TO_TICKS(1);
	controlMode_t curCtrMode = CTR_MODE_AUTO;
    vTaskDelay(1000);

    while((ConfigCommandCMD.motor_driver_type != 1)&&(ConfigCommandCMD.motor_driver_type != 2))
	{
		vTaskDelay(5000);
	}
    while(1)
    {
        vTaskDelay(frequency);
		//不同模式下，对移动的控制方式和安全检测不一样，故先判断当前的控制模式
		//1，判断工作模式
		curCtrMode = GetCurCtrMode();
		//2，根据不同的工作模式采取不同的控制
		switch(curCtrMode)
		{
			case CTR_MODE_AUTO:
				CtrMoveInAutoMode();
			break;
			case CTR_MODE_TROLLEY:
				CtrMoveInTrolleyMode();
			break;
			default://未知的工作模式错误，输出错误码
			break;
		}
    }
}
