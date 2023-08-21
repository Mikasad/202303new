#include "power_ctr.h"
#include "main.h"
#include "normal_io.h"
#include "ctr_move_task.h"
//#include "info_store.h"
//#include "iwdg_task.h"
#include "app_bms.h"


/*
*********************************************************************************************************
*
*	模块名称 : 电源输出控制模块
*	文件名称 : power_ctr.c
*	版    本 : 
*	说    明 : 
*			   控制单板所有对外输出电压
*			
*	修改记录 :
*		版本号  		日期   	 作 者       说明
*		V1.0.0    2019-11-08  	 杨鹏程   	正式发布
*	外设资源占用：
*		IO口：
*********************************************************************************************************
*/

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/



/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/
typedef enum{
    RELAY1 = 0,
    RELAY2,
    RELAY3,
    RELAY4,
	RELAY5,
    RELAY6,
    RELAY7,
}eRelay_t;

typedef enum
{
    CO24V1 = 0,
    CO24V2,
    CO24V3,
    CO24V4,
	CO24V5,
    CO24V6,
    CO24V7,

}eCo24V_t;

typedef enum
{
    DOUT1 = 0,
    DOUT2,
}eDout_t;

typedef enum
{
    OUT24V1 = 0,
    OUT24V2,
}eOut24V_t;

typedef enum
{
    DAC_1 = 0,
    DAC_2,
}eDac_t;

typedef struct
{
    keySt_t relay[4];//控制继电器
    keySt_t co24v[7];//其输出与24v电源组合使用，其控制的引脚输出gnd时对应组合输出24V
    keySt_t dout[2];//扩展io输出
    keySt_t output24v[2];//24V输出，与GND组合使用
    keySt_t charge_relay;
    keySt_t motor_relay;
}AllOutputStatus_t;

/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/
static AllOutputStatus_t m_output_status =
{
    .relay = 
    {
        {0, 0, ON_RELAY1_Pin, ON_RELAY1_GPIO_Port, 0, 15},
        {0, 0, ON_RELAY2_Pin, ON_RELAY2_GPIO_Port, 0, 15},
        {0, 0, ON_RELAY3_Pin, ON_RELAY3_GPIO_Port, 0, 15},
        {0, 0, ON_RELAY4_Pin, ON_RELAY4_GPIO_Port, 0, 15},
    },
    .co24v =
    {
        {0, 0, CO24V_1_Pin, CO24V_1_GPIO_Port, 0, 15},
        {0, 0, CO24V_2_Pin, CO24V_2_GPIO_Port, 0, 15},
        {0, 0, CO24V_3_Pin, CO24V_3_GPIO_Port, 0, 15},
        {0, 0, CO24V_4_Pin, CO24V_4_GPIO_Port, 0, 15},
		{0, 0, CO24V_5_Pin, CO24V_2_GPIO_Port, 0, 15},
        {0, 0, CO24V_6_Pin, CO24V_3_GPIO_Port, 0, 15},
        {0, 0, CO24V_7_Pin, CO24V_4_GPIO_Port, 0, 15},
    },
    .dout =
    {
        {0, 0, DOUT1_Pin, DOUT1_GPIO_Port, 0, 15},
        {0, 0, DOUT2_Pin, DOUT2_GPIO_Port, 0, 15},
    },
    .output24v =
    {
        {0, 0, OUTPUT1_Pin, OUTPUT1_GPIO_Port, 0, 15},
        {0, 0, OUTPUT2_Pin, OUTPUT2_GPIO_Port, 0, 15},
    },
    .charge_relay = {0, 0, CHARGE_SWITCH_Pin, CHARGE_SWITCH_GPIO_Port, 0, 15},
    .motor_relay = {0, 0, MOTOR_CTR_Pin, MOTOR_CTR_GPIO_Port, 0, 15},
};
#define OUTPUT_NUMS sizeof(AllOutputStatus_t)/sizeof(keySt_t)
    
static uint8_t m_shutdown_enable = 0;
static uint8_t m_restart_host_enable = 0;

/************************************************************************************************************************************
														   静态函数声明
*************************************************************************************************************************************/
static void SetRelayOn( eRelay_t f_relay );
static void SetRelayOff( eRelay_t f_relay );
static void SetCo24VOn( eCo24V_t f_co24v );
static void SetCo24VOff( eCo24V_t f_co24v );
static void SetDoutOn( eDout_t f_fout );
static void SetDoutOff( eDout_t f_fout );
static void SetOut24VOn( eOut24V_t f_out24 );
static void SetOut24VOff( eOut24V_t f_out24 );


/************************************************************************************************************************************
											   内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/
static void UpdateOutputState(keySt_t *fp_output)
{
	if(fp_output->reverse == 1)
	{
		if(fp_output->state != HAL_GPIO_ReadPin(fp_output->gpio, fp_output->gpioPin))
		{
			fp_output->startTime = HAL_GetTick();
		}
		else if(fp_output->holdTime <= HAL_GetTick() - fp_output->startTime)//状态变化超过holdTime，则切换状态
		{
			fp_output->state = (~HAL_GPIO_ReadPin(fp_output->gpio,fp_output->gpioPin)) & 0x01;
		}
	}
	else
	{
		if(fp_output->state == HAL_GPIO_ReadPin(fp_output->gpio,fp_output->gpioPin))
		{
			fp_output->startTime = HAL_GetTick();
		}
		else if(fp_output->holdTime  <= HAL_GetTick() - fp_output->startTime)//状态变化超过holdTime，则切换状态
		{
			fp_output->state = HAL_GPIO_ReadPin(fp_output->gpio,fp_output->gpioPin) & 0x01;
		}
		
	}
}

//static uint8_t GetOutputStatus( keySt_t *fp_output )
//{
//    return fp_output->state & 0x01;
//}

static void UpdateOutputSt(void)
{
	keySt_t *p_outputSt = (keySt_t *)&m_output_status;
	int i = 0;
	
	for( i=0; i < OUTPUT_NUMS; i++ )
		UpdateOutputState( p_outputSt+i );
    
}


//继电器闭合
static void SetRelayOn( eRelay_t f_relay )
{
	switch( f_relay )
	{
		case RELAY1:
			RELAY1_ON;
			break;
		
		case RELAY2:
			RELAY2_ON;
			break;
		
		case RELAY3:
			RELAY3_ON;
			break;
		
		case RELAY4:
			RELAY4_ON;
			break;
		
		default:
			break;
	}
}

//继电器断开
static void SetRelayOff( eRelay_t f_relay )
{
	switch( f_relay )
	{
		case RELAY1:
			RELAY1_OFF;
			break;
		
		case RELAY2:
			RELAY2_OFF;
			break;
		
		case RELAY3:
			RELAY3_OFF;
			break;
		
		case RELAY4:
			RELAY4_OFF;
			break;
		
		case CO24V5:
			CO24V_5_ON;
			break;
		
		case CO24V6:
			CO24V_6_ON;
			break;
		
		case CO24V7:
			CO24V_7_ON;
			break;
		
		default:
			break;
	}
}

//设置24V输出，
static void SetCo24VOn( eCo24V_t f_co24v )
{
	switch( f_co24v )
	{
		case CO24V1:
			CO24V_1_ON;
			break;
		
		case CO24V2:
			CO24V_2_ON;
			break;
		
		case CO24V3:
			CO24V_3_ON;
			break;
		
		case CO24V4:
			CO24V_4_ON;
			break;
		
		default:
			break;
	}
}

//关闭24V输出
static void SetCo24VOff( eCo24V_t f_co24v )
{
	switch( f_co24v )
	{
		case CO24V1:
			CO24V_1_OFF;
			break;
		
		case CO24V2:
			CO24V_2_OFF;
			break;
		
		case CO24V3:
			CO24V_3_OFF;
			break;
		
		case CO24V4:
			CO24V_4_OFF;
			break;
		case CO24V5:
			CO24V_5_OFF;
			break;
		
		case CO24V6:
			CO24V_6_OFF;
			break;
		
		case CO24V7:
			CO24V_7_OFF;
			break;	
		
		default:
			break;
	}
}

//设置扩展IO输出口输出
static void SetDoutOn( eDout_t f_fout )
{
	if( DOUT1 == f_fout )
	{
		DOUT1_ON;
	}
	else if( DOUT2 == f_fout )
	{
		DOUT2_ON;
	}
}

//关闭扩展IO输出
static void SetDoutOff( eDout_t f_fout )
{
	if( DOUT1 == f_fout )
	{
		DOUT1_OFF;
	}
	else if( DOUT2 == f_fout )
	{
		DOUT2_OFF;
	}
}

//设置24V输出
static void SetOut24VOn( eOut24V_t f_out24 )
{
	if( OUT24V1 == f_out24 )
	{
		OUTPUT24V_1_ON;
	}
	else if( OUT24V2 == f_out24 )
	{
		OUTPUT24V_2_ON;
	}
}

//关闭24V输出
static void SetOut24VOff( eOut24V_t f_out24 )
{
	if( OUT24V1 == f_out24 )
	{
		OUTPUT24V_1_OFF;
	}
	else if( OUT24V2 == f_out24 )
	{
		OUTPUT24V_2_OFF;
	}
}



//上位机断电
static void HostPowerOff(void)
{
    SetRelayOff( RELAY3 );
}

//上位机上电
static void HostPowerOn(void)
{
    SetRelayOn( RELAY3 );
}

//驱动器下电
static void DrivePowerOff(void)
{
    SetRelayOff( RELAY2 );
    SetRelayOff( RELAY1 );
}

//驱动器上电
static void DrivePowerOn(void)
{
    SetRelayOn( RELAY1 );
    SetRelayOn( RELAY2 );
}

static void UserPowerOn(void)
{
    
}

static void UserPowerOff(void)
{
    
    SetRelayOff( RELAY4 );
}

static void CtrLed(void)
{
//    //模式灯
//    if( GetCurCtrMode() )
//    {
//        SetCo24VOn(CO24V3);
//    }
//    else
//    {
//        SetCo24VOff(CO24V3);
//    }
    
    //钥匙灯
    if( GetKeyState() == 0x02 )
    {
        SetCo24VOff(CO24V4);
    }
    else
    {
        SetCo24VOn(CO24V4);
    }
    
//    //清水空灯
//    if( GetCleanWaterLevel() == 0 )
//    {
//        SetCo24VOn(CO24V1);
//    }
//    else
//    {
//        SetCo24VOff(CO24V1);
//    }
//    //污水满灯
//    if( GetSewageLevel() == 100 )
//    {
//        SetCo24VOn(CO24V2);
//    }
//    else
//    {
//        SetCo24VOff(CO24V2);
//    }
}

/************************************************************************************************************************************
										           	       对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/

void CtrShutDown(uint32_t f_shutDown)
{
	if( f_shutDown )
    {
        m_shutdown_enable = 1;
    }
}

void ResartHost(void)
{
    m_restart_host_enable = 1;
}

void SetRelay( uint32_t f_relay )
{
    //上位机未发下电指令且钥匙开关闭合
    if( !m_shutdown_enable && GetKeyState() == 0x01 )
    {
        if( f_relay == 0x01 )//闭合充电继电器
        {
            SetRelayOn( RELAY4 );
        }
        else if( f_relay == 0x02 )//断开充电继电器
        {
            SetRelayOff( RELAY4 );
        }
    }
}

//获取继电器状态
uint8_t GetRelayStatus(void)
{
    uint8_t relay = 0;
    
    relay |= m_output_status.relay[3].state ? 0x01 : 0x02;
    relay |= m_output_status.relay[2].state ? 0x04 : 0x08;
    relay |= m_output_status.relay[1].state ? 0x10 : 0x20;
    relay |= m_output_status.relay[0].state ? 0x40 : 0x80;
    
    return relay;
}

//获取24V输出状态
uint8_t GetCo24vStatus(void)
{
    uint8_t status = 0;
    
    status |= m_output_status.co24v[0].state;
    status |= m_output_status.co24v[1].state<<1;
    status |= m_output_status.co24v[2].state<<2;
    status |= m_output_status.co24v[3].state<<3;
    
    return status;
}

//获取扩展io输出状态
uint8_t GetExOutputStatus(void)
{
    uint8_t status = 0;
    
    status |= m_output_status.dout[0].state;
    status |= m_output_status.dout[1].state<<1;
    
    return status;
}

//获取24V输出状态
uint8_t Get24vOutStatus(void)
{
    uint8_t status = 0;
    
    status |= m_output_status.output24v[0].state;
    status |= m_output_status.output24v[1].state<<1;
    
    return status;
}

//获取板载充电继电器状态
uint8_t GetChargeRelayStatus(void)
{
    return m_output_status.charge_relay.state;
}

//获取板载motor控制继电器状态
uint8_t GetMotorRelayStatus(void)
{
    return m_output_status.motor_relay.state;
}

void SetDisinfectRelay(uint8_t f_cmd)
{
    if( f_cmd)
        SetOut24VOn(OUT24V1);
    else
        SetOut24VOff(OUT24V1);
}

uint8_t GetDisinfectRelayStatus(void)
{
    return m_output_status.output24v[0].state;
}

/*
*********************************************************************************************************
*	函 数 名: PowerCtrTask
*	功能说明: 	处理对外输出控制
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 
*********************************************************************************************************
*/
void PowerCtrTask(void const *pvParameters)
{
    TickType_t restart_host_tick = 0;
	const TickType_t frequency = pdMS_TO_TICKS(5);
	uint32_t bitPos = (uint32_t)pvParameters;
    int32_t hostshutdown_delay = 10000; //ms
    int32_t coreshutdown_delay = CORE_SHUTDOWN_DELAY_MS;//ms
    uint8_t shutdown_relay = 1;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    uint8_t bms_reset_count = 0;    
    vTaskDelay(2000);
    HostPowerOn();
    vTaskDelay(2000);
    UserPowerOn();

    while(1)
    {

        UpdateOutputSt();
        
        if( m_shutdown_enable )//上位机下发了关机命令
        {
            //驱动器断电，用户继电器断电，陀螺仪断电
            if( shutdown_relay )
            {
                shutdown_relay =0;
                HAL_GPIO_WritePin(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin, GPIO_PIN_RESET);//陀螺仪
                DrivePowerOff();
                UserPowerOff();
            }
            
            hostshutdown_delay -= frequency;
            
            //断上位机电源
            if( hostshutdown_delay <= 0 )
            {
                HostPowerOff();
                McuDrivePowerDown();
            }
            //下位机断电，上位机断电后延时一段时间再给下位机下电
            if( hostshutdown_delay <= -2000  )
            {
                app_bms_task_suspend_resume(APP_OTA_TASK_SUSPEND);
                app_anti_collision_task_suspend_resume(APP_OTA_TASK_SUSPEND);
                do{
                    app_bms_reset();
                    printf("Bms Reset.\r\n");
                    vTaskDelay(30);
                }while(!app_bms_get_reset_status() && (bms_reset_count++ < 30));
                bms_reset_count = 0;                
                McuPowerDown();
            }
        }
        
        if( GetKeyState() == 0x02 || coreshutdown_delay <= 2000 )//检测到钥匙断开后下位机延时一段时间然后关机
        {
            coreshutdown_delay -= frequency;
            
            if( coreshutdown_delay >7500 && coreshutdown_delay <= 8000 )
            {
                DrivePowerOff();
            }
            if( coreshutdown_delay >5500 && coreshutdown_delay <= 6000 )
            {
                UserPowerOff();
            }
            if( coreshutdown_delay >3500 && coreshutdown_delay <= 4000 )
            {
                HostPowerOff();
            }
            
            //驱动器断电，用户继电器断电，断上位机电源，陀螺仪断电
            if( shutdown_relay && coreshutdown_delay <= 2000 )
            {
                shutdown_relay =0;
                HAL_GPIO_WritePin(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin, GPIO_PIN_RESET);//陀螺仪
                McuDrivePowerDown();
            }
            if( coreshutdown_delay <= 0  )
            {
                McuPowerDown();
            }
        }
        if( m_restart_host_enable && GetKeyState() != 0x02 && !m_shutdown_enable )
        {
            HostPowerOff();
            m_restart_host_enable = 0;
            restart_host_tick = xTaskGetTickCount();
        }
        else if( restart_host_tick && xTaskGetTickCount()-restart_host_tick>=5000 )
        {
            restart_host_tick = 0;
            HostPowerOn();
        }
        CtrLed();
        vTaskDelay( frequency);
    }
}

int DevicePowerOn(void)
{
    //驱动器进行供电
    DrivePowerOn();

    //不会调用if里的内容，在这只是为了消除编译告警
    if( m_shutdown_enable )
    {
//        SetRelayOn( RELAY4 );
//        SetRelayOff( RELAY4 );
//        SetCo24VOn( CO24V4 );
//        SetCo24VOff( CO24V4 );
        SetDoutOn( DOUT2 );
        SetDoutOff( DOUT2 );
//        SetOut24VOn( OUT24V2 );
//        SetOut24VOff( OUT24V2 );
    }
    return 0;
}

int DevicePowerOff(void)
{
    //驱动器进行供电
    DrivePowerOff();
    return 0;
}
#define RELAY2_IS_ON 0x10
uint8_t get_hub_motor_power_state(void)
{
    return ((GetRelayStatus() & RELAY2_IS_ON) == RELAY2_IS_ON) ? 1 : 0;
}

