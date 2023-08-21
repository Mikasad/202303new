/********************************************************************************
*文件名			: adc.c
*简  介			: adc配置，由官方测试例程修改获得。
*日	 期		: 2022/6/11
*作  者			: 井小飞  
********************************************************************************
*备注： 
*外部晶振为25MHz
*
********************************************************************************/
 
/******************************头文件*******************************************/
#include "hc_adc.h"

/*****************************私有变量***************************************/
uint16_t g_Adc1DmaData[ADC1_DMA_BLOCK_SIZE] = {0};
uint16_t g_Adc3DmaData[ADC3_DMA_BLOCK_SIZE] = {0};
__IO static uint8_t m_u8AdcValUpdated = 0U;

/******************************宏定义***************************************/
 

/***************************私有函数原形*************************************/
static void Adc1SetPinAnalogMode(void);
static void Adc3SetPinAnalogMode(void);
#if 0
static void Adc1hc_dma_init(void);
static void Adc3hc_dma_init(void);
#endif
static void hc_aos_init(void);
static void adc1_dma_irq_config(void);
static void adc3_dma_irq_config(void);
static void ADC1_DMA_IrqCallback(void);
static void ADC3_DMA_IrqCallback(void);

/****************************************************************************
*函数名	: hc_adc1_init
*介	 绍	：ADC1配置
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_adc1_init(void)
{
	stc_adc_init_t stcAdcInit;

	/* 1. Enable ADC peripheral clock. */
	FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);

	/* 2. Modify the default value depends on the application. */
	(void)ADC_StructInit(&stcAdcInit);
	stcAdcInit.u16ScanMode = ADC_MD_SEQA_CONT;

	/* 3. Initializes ADC. */
	(void)ADC_Init(CM_ADC1, &stcAdcInit);
	ADC_SetSampleTime(CM_ADC1, ADC_IN5, ADC_SAMPLE_TIME);
	ADC_SetSampleTime(CM_ADC1, ADC_IN6, ADC_SAMPLE_TIME);
	
	/* 4. ADC channel configuration. */
	/* 4.1 Set the ADC pin to analog input mode. */
	Adc1SetPinAnalogMode();
	/* 4.2 Enable ADC channels. */
	ADC_ChCmd(CM_ADC1, ADC_SEQ, ADC_IN5, ENABLE);
	ADC_ChCmd(CM_ADC1, ADC_SEQ, ADC_IN6, ENABLE);

#if 1
//	Adc1hc_dma_init();
#else
//	hc_dma_init(CM_ADC1);
#endif
	/* 5. Start ADC */
//	ADC_Start(CM_ADC1);
}

/****************************************************************************
*函数名	: hc_adc1_init
*介	 绍	：ADC1配置
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_adc3_init(void)
{
	stc_adc_init_t stcAdcInit;

	/* 1. Enable ADC peripheral clock. */
	FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC3, ENABLE);

	/* 2. Modify the default value depends on the application. */
	(void)ADC_StructInit(&stcAdcInit);
	stcAdcInit.u16ScanMode = ADC_MD_SEQA_CONT;

	/* 3. Initializes ADC. */
	(void)ADC_Init(CM_ADC3, &stcAdcInit);
	ADC_SetSampleTime(CM_ADC3, ADC_IN1, ADC_SAMPLE_TIME);
	ADC_SetSampleTime(CM_ADC3, ADC_IN2, ADC_SAMPLE_TIME);
	ADC_SetSampleTime(CM_ADC3, ADC_IN3, ADC_SAMPLE_TIME);
	ADC_SetSampleTime(CM_ADC3, ADC_IN4, ADC_SAMPLE_TIME);

	/* 4. ADC channel configuration. */
	/* 4.1 Set the ADC pin to analog input mode. */
	Adc3SetPinAnalogMode();
	/* 4.2 Enable ADC channels. */
	ADC_ChCmd(CM_ADC3, ADC_SEQ, ADC_IN1, ENABLE);
	ADC_ChCmd(CM_ADC3, ADC_SEQ, ADC_IN2, ENABLE);
	ADC_ChCmd(CM_ADC3, ADC_SEQ, ADC_IN3, ENABLE);
	ADC_ChCmd(CM_ADC3, ADC_SEQ, ADC_IN4, ENABLE);

#if 1
//	Adc3hc_dma_init();
#else
//	hc_dma_init(CM_ADC3);
#endif
	/* 5. Start ADC */
//	ADC_Start(CM_ADC3);
}

/****************************************************************************
*函数名	: Adc1SetPinAnalogMode
*介	 绍	：ADC管脚配置模拟输入
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void Adc1SetPinAnalogMode(void)
{
	stc_gpio_init_t stcGpioInit;

	(void)GPIO_StructInit(&stcGpioInit);
	stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;

	(void)GPIO_Init(ADC_IN5_GPIO_Port, ADC_IN5_Pin, &stcGpioInit);
	(void)GPIO_Init(ADC_IN6_GPIO_Port, ADC_IN6_Pin, &stcGpioInit);
}

/****************************************************************************
*函数名	: Adc1SetPinAnalogMode
*介	 绍	：ADC管脚配置模拟输入
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void Adc3SetPinAnalogMode(void)
{
	stc_gpio_init_t stcGpioInit;

	(void)GPIO_StructInit(&stcGpioInit);
	stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;

	(void)GPIO_Init(ADC_IN1_GPIO_Port, ADC_IN1_Pin, &stcGpioInit);
	(void)GPIO_Init(ADC_IN2_GPIO_Port, ADC_IN2_Pin, &stcGpioInit);
	(void)GPIO_Init(ADC_IN3_GPIO_Port, ADC_IN3_Pin, &stcGpioInit);
	(void)GPIO_Init(ADC_IN4_GPIO_Port, ADC_IN4_Pin, &stcGpioInit);
}
#if 0
/****************************************************************************
*函数名	: hc_dma_init
*介	 绍	：DMA配置
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void Adc1hc_dma_init(void)
{
	stc_dma_init_t stcDmaInit;
	stc_dma_repeat_init_t stcDmaRptInit;

	(void)DMA_StructInit(&stcDmaInit);
	stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
	stcDmaInit.u32SrcAddr     = ADC1_DMA_SRC_ADDR;
	stcDmaInit.u32DestAddr    = ADC1_DMA_DEST_ADDR;
	stcDmaInit.u32DataWidth   = DMA_DATA_WIDTH;
	stcDmaInit.u32BlockSize   = ADC1_DMA_BLOCK_SIZE;
	stcDmaInit.u32TransCount  = DMA_TRANS_CNT;
	stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
	stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;

	/* Enable DMA peripheral clock and AOS function. */
	FCG_Fcg0PeriphClockCmd(DMA_PERIPH_CLK, ENABLE);
	(void)DMA_Init(DMA_UNIT, ADC1_DMA_CH, &stcDmaInit);

	stcDmaRptInit.u32Mode      = DMA_RPT_BOTH;
	stcDmaRptInit.u32SrcCount  = ADC1_DMA_BLOCK_SIZE;
	stcDmaRptInit.u32DestCount = ADC1_DMA_BLOCK_SIZE;
	(void)DMA_RepeatInit(DMA_UNIT, ADC1_DMA_CH, &stcDmaRptInit);
#if 0
	/* Enable AOS clock */
	FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
	/* Set DMA trigger source */
	AOS_SetTriggerEventSrc(ADC1_DMA_AOS_TRIG_SEL, ADC1_DMA_TRIG_EVT);
#endif
	/* DMA IRQ configuration. */
	adc1_dma_irq_config();

	DMA_Cmd(DMA_UNIT, ENABLE);
	DMA_ChCmd(DMA_UNIT, ADC1_DMA_CH, ENABLE);
}
#endif
/****************************************************************************
*函数名	: hc_aos_init
*介	 绍	：AOS配置
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void hc_aos_init(void)
{
	/* Enable AOS clock */
	FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
	/* Set DMA trigger source */
	AOS_SetTriggerEventSrc(ADC1_DMA_AOS_TRIG_SEL, ADC1_DMA_TRIG_EVT);
	AOS_SetTriggerEventSrc(ADC3_DMA_AOS_TRIG_SEL, ADC3_DMA_TRIG_EVT);
}

/****************************************************************************
*函数名	: hc_dma_init
*介	 绍	：DMA配置
*形  参 : 无
*返回值 : 无
******************************************************************************/
#if 0
static void adc_dma_init(CM_ADC_TypeDef* adc)
{
	uint8_t ch;
	stc_dma_init_t stcDmaInit;
	stc_dma_repeat_init_t stcDmaRptInit;
	
	(void)DMA_StructInit(&stcDmaInit);
	stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
	stcDmaInit.u32DataWidth   = DMA_DATA_WIDTH;
	stcDmaInit.u32TransCount  = DMA_TRANS_CNT;
	stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
	stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
	if (CM_ADC1 == adc)
	{
		stcDmaInit.u32SrcAddr     = ADC1_DMA_SRC_ADDR;
		stcDmaInit.u32DestAddr    = ADC1_DMA_DEST_ADDR;
		stcDmaInit.u32BlockSize   = ADC1_DMA_BLOCK_SIZE;
	}
	else if (CM_ADC3 == adc)
	{
		stcDmaInit.u32SrcAddr     = ADC3_DMA_SRC_ADDR;
		stcDmaInit.u32DestAddr    = ADC3_DMA_DEST_ADDR;
		stcDmaInit.u32BlockSize   = ADC3_DMA_BLOCK_SIZE;
	}
	if (CM_ADC1 == adc)
		ch = ADC1_DMA_CH;
	else if (CM_ADC3 == adc)
		ch = ADC3_DMA_CH;
	(void)DMA_Init(DMA_UNIT, ch, &stcDmaInit);

	stcDmaRptInit.u32Mode      = DMA_RPT_BOTH;
	if (CM_ADC1 == adc)
	{
		stcDmaRptInit.u32SrcCount  = ADC1_DMA_BLOCK_SIZE;
		stcDmaRptInit.u32DestCount = ADC1_DMA_BLOCK_SIZE;
	}
	else if (CM_ADC3 == adc)
	{
		stcDmaRptInit.u32SrcCount  = ADC3_DMA_BLOCK_SIZE;
		stcDmaRptInit.u32DestCount = ADC3_DMA_BLOCK_SIZE;
	}
	(void)DMA_RepeatInit(DMA_UNIT, ch, &stcDmaRptInit);
}
#endif
#if 0
void hc_dma_init(void)
{
#if 0
    uint8_t ch;
	stc_dma_init_t stcDmaInit;
	stc_dma_repeat_init_t stcDmaRptInit;

	(void)DMA_StructInit(&stcDmaInit);
	stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
	stcDmaInit.u32DataWidth   = DMA_DATA_WIDTH;
	stcDmaInit.u32TransCount  = DMA_TRANS_CNT;
	stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
	stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
	if (CM_ADC1 == adc)
	{
		stcDmaInit.u32SrcAddr     = ADC1_DMA_SRC_ADDR;
		stcDmaInit.u32DestAddr    = ADC1_DMA_DEST_ADDR;
		stcDmaInit.u32BlockSize   = ADC1_DMA_BLOCK_SIZE;
	}
	else if (CM_ADC3 == adc)
	{
		stcDmaInit.u32SrcAddr     = ADC3_DMA_SRC_ADDR;
		stcDmaInit.u32DestAddr    = ADC3_DMA_DEST_ADDR;
		stcDmaInit.u32BlockSize   = ADC3_DMA_BLOCK_SIZE;
	}
	
	/* Enable DMA peripheral clock and AOS function. */
	FCG_Fcg0PeriphClockCmd(DMA_PERIPH_CLK, ENABLE);
	if (CM_ADC1 == adc)
		ch = ADC1_DMA_CH;
	else if (CM_ADC3 == adc)
		ch = ADC3_DMA_CH;
	(void)DMA_Init(DMA_UNIT, ch, &stcDmaInit);

	stcDmaRptInit.u32Mode      = DMA_RPT_BOTH;
	if (CM_ADC1 == adc)
	{
		stcDmaRptInit.u32SrcCount  = ADC1_DMA_BLOCK_SIZE;
		stcDmaRptInit.u32DestCount = ADC1_DMA_BLOCK_SIZE;
	}
	else if (CM_ADC3 == adc)
	{
		stcDmaRptInit.u32SrcCount  = ADC3_DMA_BLOCK_SIZE;
		stcDmaRptInit.u32DestCount = ADC3_DMA_BLOCK_SIZE;
	}
	(void)DMA_RepeatInit(DMA_UNIT, ch, &stcDmaRptInit);

	/* Enable AOS clock */
	FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
	/* Set DMA trigger source */
	if (CM_ADC1 == adc)
		AOS_SetTriggerEventSrc(ADC1_DMA_AOS_TRIG_SEL, ADC1_DMA_TRIG_EVT);
	else if (CM_ADC3 == adc)
		AOS_SetTriggerEventSrc(ADC3_DMA_AOS_TRIG_SEL, ADC3_DMA_TRIG_EVT);

	/* DMA IRQ configuration. */
	if (CM_ADC1 == adc)
		adc1_dma_irq_config();
	else if (CM_ADC3 == adc)
		adc3_dma_irq_config();
#endif
	adc_dma_init(CM_ADC1);
	adc_dma_init(CM_ADC3);
	FCG_Fcg0PeriphClockCmd(DMA_PERIPH_CLK, ENABLE);
	adc1_dma_irq_config();
	adc3_dma_irq_config();
	hc_aos_init();
	DMA_Cmd(DMA_UNIT, ENABLE);
	DMA_ChCmd(DMA_UNIT, ADC1_DMA_CH, ENABLE);
	DMA_ChCmd(DMA_UNIT, ADC3_DMA_CH, ENABLE);
}
#else
void hc_dma_init(void)
{
	stc_dma_init_t stcDmaInit;
	stc_dma_repeat_init_t stcDmaRptInit;
#if 1
	(void)DMA_StructInit(&stcDmaInit);
	stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
	stcDmaInit.u32DataWidth   = DMA_DATA_WIDTH;
	stcDmaInit.u32TransCount  = DMA_TRANS_CNT;
	stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
	stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
	
	stcDmaInit.u32SrcAddr     = ADC1_DMA_SRC_ADDR;
	stcDmaInit.u32DestAddr    = ADC1_DMA_DEST_ADDR;
	stcDmaInit.u32BlockSize   = ADC1_DMA_BLOCK_SIZE;
	FCG_Fcg0PeriphClockCmd(DMA_PERIPH_CLK, ENABLE);
	(void)DMA_Init(DMA_UNIT, ADC1_DMA_CH, &stcDmaInit);
#endif
#if 1
	(void)DMA_StructInit(&stcDmaInit);
	stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
	stcDmaInit.u32DataWidth   = DMA_DATA_WIDTH;
	stcDmaInit.u32TransCount  = DMA_TRANS_CNT;
	stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
	stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
	
	stcDmaInit.u32SrcAddr     = ADC3_DMA_SRC_ADDR;
	stcDmaInit.u32DestAddr    = ADC3_DMA_DEST_ADDR;
	stcDmaInit.u32BlockSize   = ADC3_DMA_BLOCK_SIZE;
	(void)DMA_Init(DMA_UNIT, ADC3_DMA_CH, &stcDmaInit);
#endif
#if 1
	stcDmaRptInit.u32Mode      = DMA_RPT_BOTH;
	stcDmaRptInit.u32SrcCount  = ADC1_DMA_BLOCK_SIZE;
	stcDmaRptInit.u32DestCount = ADC1_DMA_BLOCK_SIZE;
	(void)DMA_RepeatInit(DMA_UNIT, ADC1_DMA_CH, &stcDmaRptInit);
#endif
#if 1
	stcDmaRptInit.u32Mode      = DMA_RPT_BOTH;
	stcDmaRptInit.u32SrcCount  = ADC3_DMA_BLOCK_SIZE;
	stcDmaRptInit.u32DestCount = ADC3_DMA_BLOCK_SIZE;
	(void)DMA_RepeatInit(DMA_UNIT, ADC3_DMA_CH, &stcDmaRptInit);
#endif
	hc_aos_init();
	
	adc1_dma_irq_config();
	adc3_dma_irq_config();

	DMA_Cmd(DMA_UNIT, ENABLE);
	DMA_ChCmd(DMA_UNIT, ADC1_DMA_CH, ENABLE);
	DMA_ChCmd(DMA_UNIT, ADC3_DMA_CH, ENABLE);
}
#endif
/****************************************************************************
*函数名	: hc_dma_init
*介	 绍	：DMA配置
*形  参 : 无
*返回值 : 无
******************************************************************************/
#if 0
static void Adc3hc_dma_init(void)
{
	stc_dma_init_t stcDmaInit;
	stc_dma_repeat_init_t stcDmaRptInit;

	(void)DMA_StructInit(&stcDmaInit);
	stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
	stcDmaInit.u32SrcAddr     = ADC3_DMA_SRC_ADDR;
	stcDmaInit.u32DestAddr    = ADC3_DMA_DEST_ADDR;
	stcDmaInit.u32DataWidth   = DMA_DATA_WIDTH;
	stcDmaInit.u32BlockSize   = ADC3_DMA_BLOCK_SIZE;
	stcDmaInit.u32TransCount  = DMA_TRANS_CNT;
	stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
	stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;

	/* Enable DMA peripheral clock and AOS function. */
	FCG_Fcg0PeriphClockCmd(DMA_PERIPH_CLK, ENABLE);
	(void)DMA_Init(DMA_UNIT, ADC3_DMA_CH, &stcDmaInit);

	stcDmaRptInit.u32Mode      = DMA_RPT_BOTH;
	stcDmaRptInit.u32SrcCount  = ADC3_DMA_BLOCK_SIZE;
	stcDmaRptInit.u32DestCount = ADC3_DMA_BLOCK_SIZE;
	(void)DMA_RepeatInit(DMA_UNIT, ADC3_DMA_CH, &stcDmaRptInit);
#if 0
	/* Enable AOS clock */
	FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
	/* Set DMA trigger source */
	AOS_SetTriggerEventSrc(ADC3_DMA_AOS_TRIG_SEL, ADC3_DMA_TRIG_EVT);
#endif
	/* DMA IRQ configuration. */
	adc3_dma_irq_config();

	DMA_Cmd(DMA_UNIT, ENABLE);
	DMA_ChCmd(DMA_UNIT, ADC3_DMA_CH, ENABLE);
}
#endif
#if 1
/****************************************************************************
*函数名	: DmaIrqConfig
*介	 绍	：DMA中断配置
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void adc1_dma_irq_config(void)
{
	stc_irq_signin_config_t stcIrqSignConfig;

	stcIrqSignConfig.enIntSrc    = ADC1_DMA_INT_SRC;
	stcIrqSignConfig.enIRQn      = ADC1_DMA_INT_IRQn;
	stcIrqSignConfig.pfnCallback = &ADC1_DMA_IrqCallback;

	(void)INTC_IrqSignIn(&stcIrqSignConfig);
	DMA_ClearTransCompleteStatus(DMA_UNIT, ADC1_DMA_INT_FLAG);

	/* NVIC setting */
	NVIC_ClearPendingIRQ(ADC1_DMA_INT_IRQn);
	NVIC_SetPriority(ADC1_DMA_INT_IRQn, ADC1_DMA_INT_PRIO);
	NVIC_EnableIRQ(ADC1_DMA_INT_IRQn);
}

/****************************************************************************
*函数名	: DmaIrqConfig
*介	 绍	：DMA中断配置
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void adc3_dma_irq_config(void)
{
	stc_irq_signin_config_t stcIrqSignConfig;
#if 0
	stcIrqSignConfig.enIntSrc    = ADC1_DMA_INT_SRC;
	stcIrqSignConfig.enIRQn      = ADC1_DMA_INT_IRQn;
	stcIrqSignConfig.pfnCallback = &ADC1_DMA_IrqCallback;
	
	(void)INTC_IrqSignIn(&stcIrqSignConfig);
	DMA_ClearTransCompleteStatus(DMA_UNIT, ADC1_DMA_INT_FLAG);
#endif	
	stcIrqSignConfig.enIntSrc    = ADC3_DMA_INT_SRC;
	stcIrqSignConfig.enIRQn      = ADC3_DMA_INT_IRQn;
	stcIrqSignConfig.pfnCallback = &ADC3_DMA_IrqCallback;

	(void)INTC_IrqSignIn(&stcIrqSignConfig);
	DMA_ClearTransCompleteStatus(DMA_UNIT, ADC3_DMA_INT_FLAG);

	/* NVIC setting */
//	NVIC_ClearPendingIRQ(ADC1_DMA_INT_IRQn);
	NVIC_ClearPendingIRQ(ADC3_DMA_INT_IRQn);
//	NVIC_SetPriority(ADC1_DMA_INT_IRQn, ADC1_DMA_INT_PRIO);
	NVIC_SetPriority(ADC3_DMA_INT_IRQn, ADC3_DMA_INT_PRIO);
//	NVIC_EnableIRQ(ADC1_DMA_INT_IRQn);
	NVIC_EnableIRQ(ADC3_DMA_INT_IRQn);
}

/****************************************************************************
*函数名	: DMA_IrqCallback
*介	 绍	：DMA中断回调
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void ADC1_DMA_IrqCallback(void)
{
    DMA_ClearTransCompleteStatus(DMA_UNIT, ADC1_DMA_INT_FLAG);
//    m_u8AdcValUpdated = 1U;
}

/****************************************************************************
*函数名	: DMA_IrqCallback
*介	 绍	：DMA中断回调
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void ADC3_DMA_IrqCallback(void)
{
    DMA_ClearTransCompleteStatus(DMA_UNIT, ADC3_DMA_INT_FLAG);
//    m_u8AdcValUpdated = 1U;
}
#endif

void hc_adc_test(void)
{
	uint8_t i = 0;
	
	ADC_Stop(CM_ADC1);
	ADC_Start(CM_ADC1);
	printf("ADC1: ");
	for (i = 0; i < ADC1_DMA_BLOCK_SIZE; i++)
		printf("%d ", g_Adc1DmaData[i]);
	printf("\r\n");
	
	printf("ADC3: ");
	for (i = 0; i < ADC3_DMA_BLOCK_SIZE; i++)
		printf("%d ", g_Adc3DmaData[i]);
	printf("\r\n");
}


