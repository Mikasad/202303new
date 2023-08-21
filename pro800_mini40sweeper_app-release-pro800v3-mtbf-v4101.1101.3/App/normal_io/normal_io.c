//stm32头文件
//APP头文件
#include "normal_io.h"
#include "main.h"
#include "iwdg_task.h"
#include "nanopb_tcpip.h"


/*
*********************************************************************************************************
*
*	模块名称 : 普通IO口模块
*	文件名称 : normal_io.c
*	版    本 : 
*	说    明 : 
*			1，普通IO模块是操作或者读取单独的引脚，该引脚不与其他的模块有牵连
（如BLDC控制器的刹车管脚其实也是普通管脚，但其是BLDC控制的一部分，所以不算是单独的引脚)
*			
*	修改记录 :
*		版本号  		日期   	作者     说明
*		V1.0.0    2018-11-05  	杨臣   	正式发布
*		V1.0.1	  2018-12-20  	杨臣	1，对于两个relay的控制，当钥匙开关关闭时，输出0.否则输出受上位机控制；
*										2，钥匙开关关闭时，两个led关闭，修复了LED1关闭时的一个代码书写错误
*	外设资源占用：
*		IO口：
*********************************************************************************************************
*/
/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/






/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/

TaskHandle_t taskHandleNormalIo;
static uint8_t gs_emergencyStopState;//急停的状态，0=放开，1=拍下
#ifdef USE_MODE_BUTTON
static uint8_t gs_trolleyModeStatusByButton = 0;
#endif
static uint8_t m_buoy_status = 0;
static uint8_t m_manual_draw_water = 0;

allKeysSt_t m_allKeysSt = {
	.din = 	{
						{0,1,DIN1_Pin,DIN1_GPIO_Port,0,10},
						{0,1,DIN2_Pin,DIN2_GPIO_Port,0,10},
						{0,1,DIN3_Pin,DIN3_GPIO_Port,0,10},
						{0,1,DIN4_Pin,DIN4_GPIO_Port,0,10},
					},
	.in = 	{
						{0,0,IN0_Pin,IN0_GPIO_Port,0,500},
						{0,0,IN1_Pin,IN1_GPIO_Port,0,500},
						{0,1,IN2_Pin,IN2_GPIO_Port,0,300},
						{0,0,IN3_Pin,IN3_GPIO_Port,0,1000},
						{0,0,IN4_Pin,IN4_GPIO_Port,0,50},
						{0,0,IN5_Pin,IN5_GPIO_Port,0,100},
						{0,0,IN6_Pin,IN6_GPIO_Port,0,15},
						{0,0,IN7_Pin,IN7_GPIO_Port,0,15},
                        {0,1,IN8_Pin,IN8_GPIO_Port,0,15},
                        {0,1,IN9_Pin,IN9_GPIO_Port,0,15},
                        {0,0,IN10_Pin,IN10_GPIO_Port,0,100},
                        {0,0,IN11_Pin,IN11_GPIO_Port,0,100},
                        {0,1,IN12_Pin,IN12_GPIO_Port,0,100},
						{0,0,IN13_Pin,IN13_GPIO_Port,0,100},
                        {0,1,IN14_Pin,IN14_GPIO_Port,0,100},
						{0,1,IN14_Pin,IN14_GPIO_Port,0,100},

					},
	.alm = 	{
						{0,0,ALM_0_Pin,ALM_0_GPIO_Port,0,15},
						{0,0,ALM_1_Pin,ALM_1_GPIO_Port,0,15},
					},
	.hardVer = 	{
						{0,0,HARD_VER1_Pin,HARD_VER1_GPIO_Port,0,15},
						{0,0,HARD_VER2_Pin,HARD_VER2_GPIO_Port,0,15},
					},
	.mcuKey = {1,0,MCU_KEY_Pin,MCU_KEY_GPIO_Port,0,100},
	.stop = {0,0,STOP_Pin,STOP_GPIO_Port,0,30},
};
#define KEY_NUMS sizeof(allKeysSt_t)/sizeof(keySt_t)
    
static uint8_t m_keyState;//钥匙开关状态,低两位有效，00，11 无效，01开启，10关闭
static uint8_t m_mcu_key_off_enable = 1;
static uint8_t m_din1_data_buff[100] = {0};
static uint8_t m_din1_sum = 0;
static uint8_t m_emergemcy_type = 0; //0自锁型，1非自锁型
static uint32_t m_emergency_status_min_time = 500; //非自锁型急停按钮时急停持续的最短时间
static SemaphoreHandle_t m_mutex_emergency = NULL;
static uint8_t m_emergency_cmd = 0;

extern struct CONFIGCOMMAND_CMD  ConfigCommandCMD;
/************************************************************************************************************************************
																静态函数声明
*************************************************************************************************************************************/
static void UpdateEmergency(uint8_t f_cmd);

/************************************************************************************************************************************
																硬件初始化
*************************************************************************************************************************************/


/************************************************************************************************************************************
																中断处理
*************************************************************************************************************************************/



/************************************************************************************************************************************
																内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/
static void IoInit(void)
{
    m_mutex_emergency = xSemaphoreCreateMutex();
}
static void UpdateKeyState(keySt_t *fp_keySt)
{
	if(fp_keySt->reverse == 1)
	{
		if(fp_keySt->state != HAL_GPIO_ReadPin(fp_keySt->gpio, fp_keySt->gpioPin))
		{
			fp_keySt->startTime = HAL_GetTick();
		}
		else if(fp_keySt->holdTime <= HAL_GetTick() - fp_keySt->startTime)//状态变化超过holdTime，则切换状态
		{
			fp_keySt->state = (~HAL_GPIO_ReadPin(fp_keySt->gpio,fp_keySt->gpioPin)) & 0x01;
		}
	}
	else
	{
		if(fp_keySt->state == HAL_GPIO_ReadPin(fp_keySt->gpio,fp_keySt->gpioPin))
		{
			fp_keySt->startTime = HAL_GetTick();
		}
		else if(fp_keySt->holdTime  <= HAL_GetTick() - fp_keySt->startTime)//状态变化超过holdTime，则切换状态
		{
			fp_keySt->state = HAL_GPIO_ReadPin(fp_keySt->gpio,fp_keySt->gpioPin) & 0x01;
		}
		
	}
}
//uint8_t GetKeyStatus( keySt_t *fp_keySt )
//{
//    return fp_keySt->state & 0x01;
//}

static void UpdateAllKeysSt(void)
{
	keySt_t *p_keySt = (keySt_t *)&m_allKeysSt;
	int i = 0;
	
	for(i=0;i<KEY_NUMS;i++)
		UpdateKeyState(p_keySt+i);
    
    //急停
    xSemaphoreTake(m_mutex_emergency, portMAX_DELAY);
    UpdateEmergency(0);
    xSemaphoreGive(m_mutex_emergency);
    
    //模式切换
    
//    m_keyState = m_allKeysSt.mcuKey.state ? 1 : 2;
    if( m_mcu_key_off_enable && m_allKeysSt.mcuKey.state )
    {
        m_keyState = 1;
    }
    else
    {
        m_keyState = 2;
        m_mcu_key_off_enable = 0;
    }
    
//    m_buoy_status = m_allKeysSt.in[0].state | (m_allKeysSt.in[0].state<<1) | (m_allKeysSt.in[0].state<<2) | (m_allKeysSt.in[1].state<<3)\
//                            |(m_allKeysSt.in[2].state<<4) | (m_allKeysSt.in[2].state<<5) | (m_allKeysSt.in[2].state<<6) | (m_allKeysSt.in[3].state<<7);
#ifdef USE_MODE_BUTTON
    gs_trolleyModeStatusByButton = m_allKeysSt.in[4].state;
#endif
    
    m_manual_draw_water = m_allKeysSt.in[5].state;
}

//0表示更新，1表示清除
static void UpdateEmergency(uint8_t f_cmd)
{
    static uint8_t s_emer_flag = 0; //标记急停按钮状态
    static TickType_t s_button_down_tick = 0; //记录急停按钮按下时刻
    
    if( f_cmd && m_emergemcy_type && !m_allKeysSt.stop.state)
    {
        s_emer_flag = 0;
        s_button_down_tick = 0;
        gs_emergencyStopState = 0;
    }
    else
    {
        if(!m_emergemcy_type)
        {
            gs_emergencyStopState = m_allKeysSt.stop.state;
            s_emer_flag = 0;
            s_button_down_tick = 0;
        }
        else
        {
            if( m_allKeysSt.stop.state || (m_emergency_cmd && !gs_emergencyStopState) )
            {
                if( !s_emer_flag ) //非急停模式下第一次按下急停
                {
                    s_emer_flag |= 0x01;
                    s_button_down_tick = xTaskGetTickCount();
                    gs_emergencyStopState = 1;
                }
                if( (s_emer_flag&0x11)==0x11 ) //急停触发时再次按下急停
                {
                    if( xTaskGetTickCount()-s_button_down_tick >= m_emergency_status_min_time )
                        s_emer_flag |= 0x02;
                }
            }
            else
            {
                if( gs_emergencyStopState && (s_emer_flag&0x01) )
                {
                    s_emer_flag |= 0x10;
                }
                if( gs_emergencyStopState && (s_emer_flag&0x02) )
                {
                    s_emer_flag |= 0x20;
                }
                if( (s_emer_flag&0x33)==0x33 ) //急停按压两个周期结束
                {
                    s_emer_flag = 0;
                    s_button_down_tick = 0;
                    gs_emergencyStopState = 0;
                }
            }
			 m_emergency_cmd = 0;//命令是否使用均清掉
        }
    }
}

/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/

uint8_t GetEmergencyStopState(void)
{
	return gs_emergencyStopState;
}

uint8_t GetEmergencyButtonStatus(void)
{
    return m_allKeysSt.stop.state;
}

//本函数不可在本文件内调用
void ClearEmergencyStop(void)
{
    if( m_emergemcy_type && !m_allKeysSt.stop.state )
    {
        xSemaphoreTake(m_mutex_emergency, portMAX_DELAY);
        UpdateEmergency(1);
        xSemaphoreGive(m_mutex_emergency);
    }
}

void SetEmergency(uint8_t f_cmd)
{
    xSemaphoreTake(m_mutex_emergency, portMAX_DELAY);
    if( m_emergemcy_type && !gs_emergencyStopState )
        m_emergency_cmd = f_cmd;
    xSemaphoreGive(m_mutex_emergency);
}

uint32_t CheckForbidMoveDirByEmergencyStop(void) 
{
	uint32_t ret = 0;

	if(GetEmergencyStopState() == 0x01)
		ret = 0xFFFF;
	return ret;
}
#ifdef USE_MODE_BUTTON
uint8_t GetTrolleyModeStatusByButton(void)
{
	return gs_trolleyModeStatusByButton;
}
#endif
uint8_t GetKeyState(void)
{
	return m_keyState;
}

uint32_t CheckForbidMoveDirByKey(void)//钥匙开关对运动方向的限制
{
	uint32_t ret = 0;
	if(GetKeyState() == 0x02)//钥匙开关关闭
		ret = 0xFFFF;
	else
		ret = 0x00;
	return ret;
}

//io输入，扩展io暂未上报
uint16_t GetNormalIoInputInfo(void)
{
    uint16_t input = 0;
    
    input |= m_allKeysSt.in[0].state << 0;
    input |= m_allKeysSt.in[1].state << 1;
    input |= m_allKeysSt.in[2].state << 2;
    input |= m_allKeysSt.in[3].state << 3;
    input |= m_allKeysSt.in[4].state << 4;
    input |= m_allKeysSt.in[5].state << 5;
    input |= m_allKeysSt.in[6].state << 6;
    input |= m_allKeysSt.in[7].state << 7;
    input |= m_allKeysSt.in[8].state << 8;
    input |= m_allKeysSt.in[9].state << 9;
	input |= m_allKeysSt.in[10].state << 10;
	input |= m_allKeysSt.in[11].state << 11;
	input |= m_allKeysSt.in[12].state << 12;
	input |= m_allKeysSt.in[13].state << 13;
	input |= m_allKeysSt.in[14].state << 14;
    return input;
}
//手动充电在位检测 0未插充电器 1充电器已插上
uint16_t GetChargeDetectionInputInfo(void)
{
    return m_allKeysSt.in[2].state ;
}

uint8_t GetExInputStatus(void)
{
    uint8_t input = 0;
    input |= m_allKeysSt.din[0].state;
    input |= m_allKeysSt.din[1].state<<1;
    
    return input;
}
// 侧门微动开关检测 V3.7只有一个 3.7以上整机有两个开关
uint8_t GetSideDoorOpenStatus(void)  
{
    uint8_t input = 0;
	
	if(ConfigCommandCMD.side_door_sensor_type == 1)
	{
		input |= m_allKeysSt.in[8].state?0:1;
		input |= (m_allKeysSt.in[9].state?0:1)<<1;
	}
	else if(ConfigCommandCMD.side_door_sensor_type == 2)
	{
		input |= GetSideDoorAdcStatus();       
	}

	return input;  // 上报  1侧门开 0侧门关
}
//获取清水箱污水箱浮标状态
uint8_t GetBuoyStatus(void)
{
    return m_buoy_status;
}
//获取清水箱水位
uint8_t GetCleanWaterLevel(void)
{
    uint8_t level = 0;
    if( (m_buoy_status & 0x08) )//&& (( m_buoy_status & 0x04 || ( m_buoy_status & 0x02 )) ) )//判断满时需要下面两级中至少一级触发
    {
        level = 100;
    }
    else if( m_buoy_status & 0x04 )
    {
        level = 50;
    }
//    else if( m_buoy_status & 0x02 )
//    {
//        level = 30;
//    }
    else
    {
        level = 0;
    }
    return level;
}
//获取污水箱水位
uint8_t GetSewageLevel(void)
{
    uint8_t level = 0;
    
    if( (m_buoy_status & 0x80) )//&& ( ( m_buoy_status & 0x40 ) || ( m_buoy_status & 0x20 ) ) )//判断满时需要下面两级中至少一级触发
    {
        level = 100;
    }
    else if( m_buoy_status & 0x40 )
    {
        level = 50;
    }
//    else if( m_buoy_status & 0x20 )
//    {
//        level = 30;
//    }
    else
    {
        level = 0;
    }
    return level;
}

uint8_t GetManualDrawStatus(void)
{
    return m_manual_draw_water;
}

uint8_t GetDin1Sum(void)
{
    return m_din1_sum;
}

uint8_t GetHepaDustBagAlarm(void)
{
    uint8_t alarm = 0;
    alarm |= m_allKeysSt.in[0].state << 0;
    alarm |= m_allKeysSt.in[1].state << 1;
    
    return alarm;
}

void ConfigEmergencyParam(uint8_t f_type, uint32_t f_time )
{
    m_emergemcy_type = f_type;
    m_emergency_status_min_time = (f_time>=100)?f_time:100;
}

uint8_t GetEmergencyType(void)
{
    return m_emergemcy_type;
}

uint32_t GetEmergencyMinTime(void)
{
    return m_emergency_status_min_time;
}

uint8_t GetDisinfectLevel(void)
{
    uint8_t level = 0;
    
    level = m_allKeysSt.in[9].state<<1;
//    level |= m_allKeysSt.in[8].state<<1;
    return level;
}

/************************************************************************************************************************************
												任务+对外的硬件初始化+对外的软件初始化
*************************************************************************************************************************************/
#include "power_ctr.h"
/**********************************************************************************************
*函数名	: Set_RGB_Led
*介	 绍	：设置三色灯带状态
*形  参 : status 2红色 1蓝色 3绿色
*返回值 : 无
***********************************************************************************************/
void SetLedRgb(uint8_t status)
{
	if(status==1)
	{
		CO24V_5_ON;  //红色灯条
		CO24V_6_OFF; //蓝色灯条
		CO24V_7_OFF; //绿色灯条
	}
	else if(status==2)
	{
		CO24V_5_OFF;  //红色灯条
		CO24V_6_ON; //蓝色灯条
		CO24V_7_OFF; //绿色灯条
	}
	else if(status==3)
	{
		CO24V_5_OFF;  //红色灯条
		CO24V_6_OFF; //蓝色灯条
		CO24V_7_ON; //绿色灯条
	}
}

//设置LED灯条
static void SetRGBLed(void)
{
    static uint8_t s_breath = 1;
    if( GetEmergencyStopState() )//急停，红灯亮蓝灯不亮
    {
		SetLedRgb(1);
    }
    else if( !GetPcConnectStatus() ) //上下位机断链，红灯亮蓝灯不亮
    {
       SetLedRgb(1);
    }
//    else if( m_robot_work_mode == 1 )//扫地任务
//    {
//        if( s_breath )
//        {
//            m_output_pwm_2 += 20; 
//            if( m_output_pwm_2 >= 100 )
//            {
//                s_breath = 0;
//            }
//        }
//        else
//        {
//            m_output_pwm_2 -= 20; 
//            if( m_output_pwm_2 < 20 )
//            {
//                s_breath = 1;
//            }
//        }
//        m_output_pwm_1 = 0; //红色灯条灭
//        
//    }
    else
    {
     SetLedRgb(2);
    }
}

/*
*********************************************************************************************************
*	函 数 名: NormalIoTask
*	功能说明: 	处理普通IO的输入输出任务
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 
*********************************************************************************************************
*/
void NormalIoTask(void const *pvParameters)
{
	TickType_t lastWakeTime;
	const TickType_t frequency = pdMS_TO_TICKS(5);
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    uint8_t din1_idx = 0;
    uint8_t delay_count = 0;

    IoInit();
    
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
    (void)bitPos;
	
    while(1)
    {
		/* 喂狗 */
//		FeedDog(bitPos);
        
        delay_count++;
        if( delay_count>=2)
        {
            delay_count = 0;
            if(din1_idx <100)
                m_din1_data_buff[din1_idx] = m_allKeysSt.din[0].state;
            else
            {
                m_din1_sum = 0;
                for(int i=0;i<100;i++)
                    m_din1_sum += m_din1_data_buff[i];
                din1_idx = 0;
                m_din1_data_buff[din1_idx] = m_allKeysSt.din[0].state;
            }
            din1_idx++;
        }
        
        UpdateAllKeysSt();
		SetRGBLed();
//        g_thread_call_count[thread_num]++;
        
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelay(5);
    }
}




