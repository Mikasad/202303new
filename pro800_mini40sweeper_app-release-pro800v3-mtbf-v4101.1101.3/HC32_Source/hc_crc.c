/********************************************************************************
*文件名			: hc_crc.c
*简  介			: crc配置，由官方测试例程修改获得。
*日	 期		: 2022/7/10
*作  者			: 井小飞  
********************************************************************************
*备注： 
*外部晶振为25MHz
*
********************************************************************************/
 
/******************************头文件*******************************************/
#include "cmsis_os.h"
#include "hc_crc.h"


SemaphoreHandle_t  m_crc_mutex_handle = NULL;

extern uint8_t g_scheduler_start;
/*****************************私有变量***************************************/

/******************************宏定义***************************************/

/***************************私有函数原形*************************************/


/****************************************************************************
*函数名	: sw_crc32_calculate
*介	 绍	：软件CRC32计算，保持和上位机CRC计算一致
*形  参 : buf：待计算的数据，len：数据长度
*返回值 : CRC32值
******************************************************************************/
static uint32_t sw_crc32_calculate(uint32_t *ptr, int len)
{
	uint32_t xbit;
	uint32_t data;
	uint32_t crc = 0xFFFFFFFF;    // init
	uint32_t dwPolynomial = 0x04c11db7;
	while (len--) {
		xbit = 1UL << 31;

		data = *ptr++;
		for (int bits = 0; bits < 32; bits++) {
			if (crc & 0x80000000) {
				crc <<= 1;
				crc ^= dwPolynomial;
			} else
				crc <<= 1;
			if (data & xbit)
				crc ^= dwPolynomial;

			xbit >>= 1;
		}
	}
	return crc;
}

/****************************************************************************
*函数名	: hc_crc32_init
*介	 绍	：CRC32初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_crc32_init(void)
{
	stc_crc_init_t stcCrcInit;

    m_crc_mutex_handle = xSemaphoreCreateMutex();
    if( m_crc_mutex_handle == NULL )
        return;
	
	/* Enable CRC module clock. */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_CRC, ENABLE);
	
	stcCrcInit.u32Protocol = CRC_CRC32;
    stcCrcInit.u32InitValue = CRC32_INIT_VALUE;
    (void)CRC_Init(&stcCrcInit);
}

/****************************************************************************
*函数名	: hc_sw_crc32_calculate
*介	 绍	：软件CRC32计算，此计算并非使用HC官方的，为了兼容ST，使用ST的软件计算方法
*形  参 : buf：待计算的数据，len：数据长度
*返回值 : CRC32值
******************************************************************************/
uint32_t hc_sw_crc32_calculate(uint32_t* buf, uint32_t len)
{
    uint32_t crc = 0;
	
    if( g_scheduler_start && m_crc_mutex_handle!=NULL )
        xSemaphoreTake(m_crc_mutex_handle, portMAX_DELAY);

		crc = sw_crc32_calculate(buf, len);

    if( g_scheduler_start && m_crc_mutex_handle!=NULL )
        xSemaphoreGive( m_crc_mutex_handle);
	
    return crc;
}

/****************************************************************************
*函数名	: hc_hw_crc32_calculate
*介	 绍	：硬件CRC32计算
*形  参 : buf：待计算的数据，len：数据长度
*返回值 : CRC32值
******************************************************************************/
uint32_t hc_hw_crc32_calculate(uint32_t* buf, uint32_t len)
{
    uint32_t crc = 0;

    if( g_scheduler_start && m_crc_mutex_handle!=NULL )
        xSemaphoreTake(m_crc_mutex_handle, portMAX_DELAY);

		crc = CRC_CalculateData32(CRC32_INIT_VALUE, buf, len);

    if( g_scheduler_start && m_crc_mutex_handle!=NULL )
        xSemaphoreGive( m_crc_mutex_handle);
	
    return crc;
}

/****************************************************************************
*函数名	: hc_crc32_deinit
*介	 绍	：CRC32取消初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_crc32_deinit(void)
{
	CRC_DeInit();
    /* Disable CRC Clock */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_CRC, DISABLE);
}

