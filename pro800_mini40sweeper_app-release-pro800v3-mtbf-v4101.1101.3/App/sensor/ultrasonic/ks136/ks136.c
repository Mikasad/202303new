//标准头文件
#include <string.h>
//stm32头文件
#include "main.h"
//APP头文件
#include "sensor.h"
#include "ultrasonic.h"
#include "debug.h"
//#include "usart.h"
#include "main.h"
#include "iwdg_task.h"
#include "hc_gpio.h"
#include "hc_usart.h"

/*
*********************************************************************************************************
*
*	模块名称 : KS136超声模块
*	文件名称 : ks136.c
*	版    本 : 	V1.0.0
*	说    明 : 
*			1，KS136支持 IIC（板子不支持）和TTL、485。
*			2，对于串口模式，应用层代码相同的，但是串口模式下只能对0x02寄存器读写，读写他寄存器不会响应
*			3，KS136的使用是问答式的，最多可以接12个超声探头。
*			4，可以成对使用，一个发一个收，最多6对，如果使用两个模块可以最多12对
*			5，可以单独使用，自发自收，最多12个
*			6，KS136支持即单又双，但是当前模块不支持这个功能，如果需要再支持
*	修改记录 :
*		版本号  		日期        作者     说明

*	外设资源占用：
*		UART：UART5
*		GPIO：PD2,PC12,PA3
*		NVIC：UART5_IRQn，优先级（8，0）
*********************************************************************************************************
*/





/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/
//版本：KS136
#define ULTRASONIC_VERSION_TYPE 	0x1//字母K
#define ULTRASONIC_VERSION_MAJOR 	0x1//字母S
#define ULTRASONIC_VERSION_MINOR 	0x03//1
#define ULTRASONIC_VERSION_BUILD 	0x6//36


#define KS136_NUM    12
#define KS136_IDLE   1
#define KS136_BUSY   2

/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/


/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/
//第四个字节定义采样顺序
uint8_t m_ks136_cmd[KS136_NUM][4] =
{
    0xE8, 0x02, 0x01, 0x01,
    0xE8, 0x02, 0x02, 0x02,
    0xE8, 0x02, 0x03, 0x03,
    0xE8, 0x02, 0x04, 0x04,
    0xE8, 0x02, 0x05, 0x05,
    0xE8, 0x02, 0x06, 0x06,
    0xEA, 0x02, 0x02, 0x07,
    0xEA, 0x02, 0x06, 0x08,
    0xEA, 0x02, 0x01, 0x09,
    0xEA, 0x02, 0x03, 0x0A,
    0xEA, 0x02, 0x04, 0x0B,
    0xEA, 0x02, 0x05, 0x0C,
    
};

static uint16_t m_ks136recv_data[KS136_NUM] = {0};

static uint8_t m_ks136_status = KS136_IDLE;

static uint8_t m_idx = 0;

static uint32_t m_ultrasonic_enable = 0;

static uint32_t m_last_recv_tick = 0;

/************************************************************************************************************************************
																静态函数声明
*************************************************************************************************************************************/
static void Ks136UartSend( uint8_t* data, uint8_t len )
{
	hc_usart5_transmit(data, len);
}
//循环读取使能的超声数据
static void Ks136Send(void)
{
    uint8_t i = 0;
    
    if( m_ultrasonic_enable & 0x00000FFF )//使能判断
    {
        if( m_idx >= KS136_NUM )
        {
            m_idx = 0;
        }
        
        if( m_ks136_status == KS136_BUSY )//上次发送未收到回复数据
        {
            m_ks136recv_data[m_idx] = 0xFFFF;
            m_ks136_status = KS136_IDLE;
            m_idx++;
            if( m_idx >= KS136_NUM )
            {
                m_idx = 0;
            }
        }
        
        while( m_ks136_status == KS136_IDLE )
        {
            for( i=0; i< KS136_NUM; i++ )
            {
                if( m_idx == m_ks136_cmd[i][3] -1 )
                {
                    break;
                }
            }
            if( i==KS136_NUM )//命令配置表有误，从头开始
            {
                i = 0;
                m_idx = 0;
            }
            if( m_ultrasonic_enable & (1<<i) )//该位使能
            {
                Ks136UartSend( &m_ks136_cmd[i][0], 3 );
                m_ks136_status = KS136_BUSY;
                break;
            }
            else//未使能，发送下个
            {
                m_idx++;
                if( m_idx >= KS136_NUM )
                {
                    m_idx = 0;
                }
            }
        }
    }
}
//接收中断中调用，接收超声盒返回的读数
void Ks136Recv( uint8_t* data, uint32_t len )
{
    if( 2 == len && m_ks136_status == KS136_BUSY )
    {
        m_ks136recv_data[m_idx] = ((uint16_t)data[0]<<8) | data[1];
        m_idx++;
        m_ks136_status = KS136_IDLE;
        m_last_recv_tick = xTaskGetTickCount();
    }
}



/************************************************************************************************************************************
																硬件初始化
*************************************************************************************************************************************/


/************************************************************************************************************************************
																中断处理
*************************************************************************************************************************************/
void KS136_IRQHandler(void)
{
//    uint32_t recv_len = 0;
//    uint32_t flag = 0;
//	uint32_t temp;
//	flag =__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE); //获取IDLE标志位
//	if((flag != RESET))//idle标志被置位
//	{ 
//		__HAL_UART_CLEAR_IDLEFLAG(&huart5);//清除标志位
//		temp = huart5.Instance->SR;  //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
//		temp = huart5.Instance->DR; //读取数据寄存器中的数据
//		HAL_UART_DMAStop(&huart5); //
//		recv_len =  UART5_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx); //总计数减去未传输的数据个数，得到已经接收的数据个数
//        Ks136Recv( &g_uart5RxBuf[0], recv_len );
//        HAL_UART_Receive_DMA(&huart5,g_uart5RxBuf,UART5_RX_BUF_SIZE);
//    }
//    (void)temp;
}



/************************************************************************************************************************************
																内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/


/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/
//获取超声版本，此处非版本，而是超声盒类型号
void GetUltrasonicVersion(version_t *fp_version)
{
	fp_version->type = ULTRASONIC_VERSION_TYPE;
	fp_version->major = ULTRASONIC_VERSION_MAJOR;
	fp_version->minor = ULTRASONIC_VERSION_MINOR;
	fp_version->build = ULTRASONIC_VERSION_BUILD;
}
uint8_t GetUltrasonicHealthState(void)
{
	uint8_t ret = 0;//1=健康
    if( m_ultrasonic_enable )//只在使能时检测断链
        ret = (xTaskGetTickCount()-m_last_recv_tick > 2000)?0:1;
	else
    {
        ret = 1;
        m_last_recv_tick = xTaskGetTickCount();
    }
	return ret;
}

void GetUltrasonicDistance(uint32_t * fp_dest)
{
	int i = 0;
	for(i=0;i<KS136_NUM;i++)
		fp_dest[i] = m_ks136recv_data[i];
}

void GetUltrasonicCfgPara(sensorDirCfg_t f_ultrasonicDirCfg, uint32_t f_enableCfg)
{
//	int i = 0;
//	uint32_t numbersOfUsedCom = 0;
//	
//	gs_ultrasonicCfgPara = f_ultrasonicDirCfg;
//    
    m_ultrasonic_enable = f_enableCfg;

//	if(f_enableCfg == 0)
//		gs_cfgUltrasonicEnable = 0;
//	else
//		gs_cfgUltrasonicEnable =   gs_ultrasonicCfgPara.front \
//								| gs_ultrasonicCfgPara.back \
//								| gs_ultrasonicCfgPara.left \
//								| gs_ultrasonicCfgPara.right \
//								| gs_ultrasonicCfgPara.leftFront \
//								| gs_ultrasonicCfgPara.rightFront \
//								| gs_ultrasonicCfgPara.leftBack \
//								| gs_ultrasonicCfgPara.rightBack;

//	//通过bit=1的个数计算需要使用的超声波端口个数
//	for(i=0;i<12;i++)
//	{
//		if(((gs_cfgUltrasonicEnable >> i) & 0x01) == 0x01)
//			 numbersOfUsedCom++;
//	}
//	gs_ks136UserInfo.numbersOfUsedCom = numbersOfUsedCom;
//	if((KS136_DETECT_MODE == KS136_DETECT_MODE_DOUBLE) && (NUMBERS_OF_KS136_USED == 1) \
//		&& (gs_ks136UserInfo.numbersOfUsedCom > 6)) //最多只能有6个
//		gs_ks136UserInfo.numbersOfUsedCom = 6;
}



/*
*********************************************************************************************************
*	函 数 名: UltrasonicTask
*	功能说明: 
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 高 
*********************************************************************************************************
*/
#include "nanopb_tcpip.h"
#include "hc_usart.h"

void UltrasonicTask(void const *pvParameters)
{
//	uint32_t initFlag = 0;
	TickType_t lastWakeTime;
    TickType_t last_check_time;
	const TickType_t c_frequency = pdMS_TO_TICKS(20);
	uint32_t bitPos = (uint32_t)pvParameters;
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    uint8_t disconnect_count = 0;
    
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();
    m_last_recv_tick = xTaskGetTickCount();
    last_check_time = m_last_recv_tick;

    while(1)
    {
		uint16_t len = BUF_UsedSize(&m_stcRingBuf_RS485_2);
		BUF_Read(&m_stcRingBuf_RS485_2, g_uart5RxBuf, len);        
		Ks136Recv( &g_uart5RxBuf[0], len );

        Ks136Send();	
        
        if( xTaskGetTickCount() - last_check_time >= 1000 )//当一直未接收到回复数据时重新初始化串口，防止串口异常导致
        {
            last_check_time = xTaskGetTickCount();
            if( !GetUltrasonicHealthState() )
            {
                disconnect_count++;
                if( disconnect_count == 50 )
                {
                    ;
                }
                else if( disconnect_count >= 100 )
                {
                    disconnect_count = 0;
                }
            }
            else
            {
                disconnect_count = 0;
            }
        }
        vTaskDelay(c_frequency);
	}
}
