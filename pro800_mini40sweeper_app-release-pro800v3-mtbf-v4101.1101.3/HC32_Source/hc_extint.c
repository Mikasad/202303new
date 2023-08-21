/********************************************************************************
*文件名			: hc_exti.c
*简  介			: 外部IO中断
*日	 期			: 2022/7/5
*作  者			: 刘德明 
********************************************************************************
*使用范例： 
*        
* 		 
********************************************************************************/

/******************************头文件*******************************************/
#include "hc_extint.h"

__IO uint32_t ENC0_Count,ENC1_Count = 0;

/****************************************************************************
*函数名	: HC_ExtiInit
*介	 绍	：外部输入中断初始化
*形  参 : 无 
*返回值 : 无
******************************************************************************/
void HC_ExtiInit(void)
{
	PA15_ExtiInit();
	PE0_ExtiInit();
}

/****************************************************************************
*函数名	: EXTINT_ECN0_IrqCallback
*介	 绍	：定时器回调函数
*形  参 : 无 
*返回值 : 无
******************************************************************************/
 static void EXTINT_PA15_IrqCallback(void)
{
	ENC0_Count++;
	EXTINT_ClearExtIntStatus(ECN0_EXTINT_CH);
}

/****************************************************************************
*函数名	: EXTINT_ECN1_IrqCallback
*介	 绍	：定时器回调函数
*形  参 : 无 
*返回值 : 无
******************************************************************************/
 static void EXTINT_PE0_IrqCallback(void)
{
	ENC1_Count++;
	EXTINT_ClearExtIntStatus(ECN1_EXTINT_CH);
}
/****************************************************************************
*函数名	: PA15_ExtiInit
*介	 绍	：PA15外部输入中断初始化
*形  参 : 无 
*返回值 : 无
******************************************************************************/
void PA15_ExtiInit(void)
{
    stc_extint_init_t stcExtIntInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    stc_gpio_init_t stcGpioInit;
	
    //LL_PERIPH_WE(LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_GPIO | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM);
    /* GPIO config */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16ExtInt = PIN_EXTINT_ON;
    stcGpioInit.u16PullUp = PIN_PU_ON;
    (void)GPIO_Init(ECN0_PORT, ECN0_PIN, &stcGpioInit);

    /* ExtInt config */
    (void)EXTINT_StructInit(&stcExtIntInit);
    stcExtIntInit.u32Filter      = EXTINT_FILTER_OFF;
    stcExtIntInit.u32FilterClock = EXTINT_FCLK_DIV8;
    stcExtIntInit.u32Edge = EXTINT_TRIG_FALLING;
    (void)EXTINT_Init(ECN0_EXTINT_CH, &stcExtIntInit);

    /* IRQ sign-in */
    stcIrqSignConfig.enIntSrc = ECN0_INT_SRC;
    stcIrqSignConfig.enIRQn   = ECN0_INT_IRQn;
    stcIrqSignConfig.pfnCallback = &EXTINT_PA15_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);

    /* NVIC config */
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, ECN0_INT_PRIO);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
	
}

/****************************************************************************
*函数名	: PE0_ExtiInit
*介	 绍	：PA0外部输入中断初始化
*形  参 : 无 
*返回值 : 无
******************************************************************************/
void PE0_ExtiInit(void)
{
    stc_extint_init_t stcExtIntInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    stc_gpio_init_t stcGpioInit;
	
    //LL_PERIPH_WE(LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_GPIO | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM);
    /* GPIO config */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16ExtInt = PIN_EXTINT_ON;
    stcGpioInit.u16PullUp = PIN_PU_ON;
    (void)GPIO_Init(ECN1_PORT, ECN1_PIN, &stcGpioInit);

    /* ExtInt config */
    (void)EXTINT_StructInit(&stcExtIntInit);
    stcExtIntInit.u32Filter      = EXTINT_FILTER_OFF;
    stcExtIntInit.u32FilterClock = EXTINT_FCLK_DIV8;
    stcExtIntInit.u32Edge = EXTINT_TRIG_FALLING;
    (void)EXTINT_Init(ECN1_EXTINT_CH, &stcExtIntInit);

    /* IRQ sign-in */
    stcIrqSignConfig.enIntSrc = ECN1_INT_SRC;
    stcIrqSignConfig.enIRQn   = ECN1_INT_IRQn;
    stcIrqSignConfig.pfnCallback = &EXTINT_PE0_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);

    /* NVIC config */
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, ECN1_INT_PRIO);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

}

 