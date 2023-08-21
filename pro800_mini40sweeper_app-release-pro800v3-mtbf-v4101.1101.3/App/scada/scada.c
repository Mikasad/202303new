#include "scada.h"
//#include "adc.h"
//#include "i2c.h"
//#include "ads1115.h"
#include <stdbool.h>
#include "freertos.h"
#include "hc_adc.h"
#include "task.h"

/*
*********************************************************************************************************
*
*	模块名称 : 电源输出控制模块
*	文件名称 : scada.c
*	版    本 : 
*	说    明 : 电压电流采集、充电监控
*			  
*			
*	修改记录 :
*		版本号  		日期   	 作 者       说明
*		V1.0.0    2019-11-12  	    	
*	外设资源占用：
*		IO口：
*********************************************************************************************************
*/

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/
#define USE_SIMULATE_IIC
#define ADC_NEW 1

/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/
typedef struct{
	uint16_t chargeCurrentADC;
	uint16_t autoVoltageADC;
	uint16_t handleVoltageADC;
	uint16_t batVoltageADC; 
	
	float chargeCurrent;
	float autoVoltage;
	float handleVoltage;
	float batVoltage;
	
	float adcInfo[6][2];
}ADCInfo_t;

/************************************************************************************************************************************
														    变量定义
*************************************************************************************************************************************/


static ADCInfo_t m_adc_info = {0};//ADC采样数据存储
static chargeConfig_t m_charge_config = {0};//充电监控相关的参数，由上位机配置
static bool m_is_charge = false;

/************************************************************************************************************************************
														  静态函数声明
*************************************************************************************************************************************/
static void HandleADC(void);

/************************************************************************************************************************************
												内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/



static void HandleADC(void)
{
	float adcV[6] = {0};
	int j=0;
	//计算mcu电源电压
//    HAL_ADC_Stop_DMA(&hadc1);
	ADC_Stop(CM_ADC1);

//    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc1RawData, 10);
    ADC_Start(CM_ADC1); 


//    HAL_ADC_Stop_DMA(&hadc3);
	ADC_Stop(CM_ADC3);
    
    //计数6路AD检测电压 
	adcV[0] = g_Adc3DmaData[1];   
	adcV[1] = g_Adc3DmaData[3];
	adcV[2] = g_Adc3DmaData[0];
	adcV[3] = g_Adc3DmaData[5];
	adcV[4] = g_Adc1DmaData[0];
	adcV[5] = g_Adc1DmaData[1];
	
    for(int i=0;i<6;i++)
    {
		if (i == 0)
		{
			m_adc_info.adcInfo[i][0] = adcV[i];
			m_adc_info.adcInfo[i][1] = adcV[i]* 3.371/210;
		}
		else
		{
			m_adc_info.adcInfo[i][0] = adcV[i];
			m_adc_info.adcInfo[i][1] = adcV[i]* 3.371/2510;		
		}
		
    }

	ADC_Start(CM_ADC3);
}


/************************************************************************************************************************************
										           	       对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/
__weak uint16_t GetBatteryVoltage(void)
{
    return (uint16_t)(m_adc_info.adcInfo[1][1]*10);
}

__weak uint16_t GetChargeVoltage(void)
{
    return (m_adc_info.adcInfo[0][1]>1) ? (uint16_t)(m_adc_info.adcInfo[0][1]*1000) : 0;
}

__weak uint16_t GetChargeCurrent(void)
{
    return (m_adc_info.chargeCurrent >= 0.5f) ? (uint16_t)(m_adc_info.chargeCurrent*1000) : 0;
}

void CfgChargeData( chargeConfig_t charge )
{
    m_charge_config = charge;
}

void GetAdcValue( float value[] )
{
    if( value != NULL )
    {
        value[0] = m_adc_info.adcInfo[0][1];
        value[1] = m_adc_info.adcInfo[1][1];
        value[2] = m_adc_info.adcInfo[2][1];
        value[3] = m_adc_info.adcInfo[3][1];
        value[4] = m_adc_info.batVoltage;
        value[5] = m_adc_info.chargeCurrent;
        value[6] = m_adc_info.autoVoltage;
        value[7] = m_adc_info.handleVoltage;
    }
    
}

uint16_t GetTorqueVoltage1(void)
{
    return (uint16_t)(m_adc_info.adcInfo[1][1]*100);
}

uint16_t GetTorqueVoltage2(void)
{
    return (uint16_t)(m_adc_info.adcInfo[2][1]*100);
}

uint16_t GetDustBag1Voltage(void)
{
    return (uint16_t)(m_adc_info.adcInfo[3][1]*100);
}

//距离

float GetDustBagDistance(float vol)
{
	float x;
	float distance=0.0f;
	x=vol;
	distance = -0.9451 +6.3511/x;
    return distance;
}
 
uint16_t GetDustBag1Distance(void)
{
	float distance=0.0f;
	distance=GetDustBagDistance(m_adc_info.adcInfo[3][1]);
    return (uint16_t)(distance*10);
}

/**********************************************************************************************
*函数名	: GetDustBagStatus
*介	 绍	：获取尘袋状态
*形  参 : 无
*返回值 : 0：尘袋空 1：尘袋满
***********************************************************************************************/
float dust_distanc;
#include "nanopb_tcpip.h"
extern struct CONFIGCOMMAND_CMD  ConfigCommandCMD;
float dust_distanc;
uint16_t GetDustBagStatus(void)
{
	static uint32_t  start_error_tick=0;
	static uint32_t  start_ok_tick=0;

	static uint16_t status=0;
	
	dust_distanc=GetDustBag1Distance();
	if(dust_distanc<ConfigCommandCMD.dust_bag_error_distance)
	{
		if(xTaskGetTickCount()-start_error_tick>ConfigCommandCMD.dust_full_threshold)
		{
			status=1;
		}
		start_ok_tick = xTaskGetTickCount();
	}
	else
	{
		
		if(xTaskGetTickCount()-start_ok_tick>ConfigCommandCMD.dust_full_threshold)
		{
			status=0;
		}
		start_error_tick= xTaskGetTickCount();
	}

	return status;
}

/**********************************************************************************************
*函数名	: GetDustBagErrorStatus
*介	 绍	：获取尘袋告警状态
*形  参 : 无
*返回值 : bit0 尘袋1
***********************************************************************************************/
uint16_t GetDustBagErrorStatus(void)
{
	uint16_t bag1vol;
	static uint32_t  start_error_tick=0;
	static uint32_t  start_ok_tick=0;
	static uint16_t status=0;
	bag1vol=GetDustBag1Voltage()/10; //转变为单位0.1V  

	if(bag1vol<ConfigCommandCMD.dust_bag_error_vol||bag1vol>50)
	{
		if(xTaskGetTickCount()-start_error_tick>ConfigCommandCMD.dust_full_threshold)
		{
			status=1;
		}
		start_ok_tick = xTaskGetTickCount();
	}
	else
	{
		if(xTaskGetTickCount()-start_ok_tick>ConfigCommandCMD.dust_full_threshold)
		{
			status=0;
		}
		start_error_tick= xTaskGetTickCount();
	}

	return status;
}

//5V霍尔模块  开门5V，关门0V
uint16_t GetSideDoorAdcStatus(void) //4.1整机侧门检测使用ADC5、6 
{
	uint8_t left_door = 0;
	uint8_t right_door = 0;
	left_door = (m_adc_info.adcInfo[4][1]*100)>4?01:0;
	right_door = (m_adc_info.adcInfo[5][1]*100)>4?1:0;
	
    return (left_door|(right_door<<1)) ;  //上报  1侧门开 0侧门关
}

/*
*********************************************************************************************************
*	函 数 名: ScadaTask
*	功能说明: 	处理对外输出控制
*	形    参: void
*	返 回 值: 无
*   优 先 级: 最低，放在空闲任务中执行
*********************************************************************************************************
*/
void ScadaTask(void)
{
    static TickType_t s_last_scada_tick = 0;
    
    if( xTaskGetTickCount() - s_last_scada_tick >= 100 )
    {
        s_last_scada_tick = xTaskGetTickCount();
        HandleADC();
}
}

