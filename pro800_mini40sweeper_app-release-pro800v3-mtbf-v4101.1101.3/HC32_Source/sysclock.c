/********************************************************************************
*文件名			: sysclock.c
*简  介			: 系统时钟配置，由官方测试例程修改获得。
*日	 期			: 2022/6/8
*作  者			: 刘德明  
********************************************************************************
*备注： 
*外部晶振为25MHz
*
********************************************************************************/
 
/******************************头文件*******************************************/
#include "main.h"
#include "sysclock.h"
/*****************************私有变量***************************************/
static uint8_t au8SysClockTbl[] = {
    CLK_SYSCLK_SRC_HRC,
    CLK_SYSCLK_SRC_MRC,
    CLK_SYSCLK_SRC_LRC,
    CLK_SYSCLK_SRC_XTAL,
    CLK_SYSCLK_SRC_XTAL32,
    CLK_SYSCLK_SRC_PLL,
};

/******************************宏定义***************************************/
#define MCO_PORT            GPIO_PORT_A
#define MCO_PIN             GPIO_PIN_08
#define MCO_GPIO_FUNC       GPIO_FUNC_1

 

/***************************私有函数原形*************************************/
static void MCO_init(void);
static void xtal_init(void);
static void xtal32_init(void);
static void pllh_init(void);
 

/****************************************************************************
*函数名	: MCO_init
*介	 绍	：MCO引脚初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void MCO_init(void)
{
    /* Configure clock output pin */
    GPIO_SetFunc(MCO_PORT, MCO_PIN, MCO_GPIO_FUNC);
    /* Configure clock output system clock */
    CLK_MCOConfig(CLK_MCO1, CLK_MCO_SRC_HCLK, CLK_MCO_DIV8);
    /* MCO1 output enable */
    CLK_MCOCmd(CLK_MCO1, ENABLE);
}

/****************************************************************************
*函数名	: xtal_init
*介	 绍	：xtal时钟初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void xtal_init(void)
{
    stc_clock_xtal_init_t stcXtalInit;

    /* XTAL config */
    (void)CLK_XtalStructInit(&stcXtalInit);
    /* Config Xtal and Enable Xtal */
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_ULOW;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);
}

/****************************************************************************
*函数名	: xtal32_init
*介	 绍	：xtal时钟初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void xtal32_init(void)
{
    stc_clock_xtal32_init_t stcXtal32Init;

    /* Xtal32 config */
    (void)CLK_Xtal32StructInit(&stcXtal32Init);
    stcXtal32Init.u8State = CLK_XTAL32_ON;
    stcXtal32Init.u8Drv = CLK_XTAL32_DRV_MID;
    stcXtal32Init.u8Filter = CLK_XTAL32_FILTER_ALL_MD;
    (void)CLK_Xtal32Init(&stcXtal32Init);
}

 
/****************************************************************************
*函数名	: pllh_init
*介	 绍	：pllh时钟初始化 
*         PCLK0: 240MHz
*         PCLK1: 120MHz
*         PCLK2: 60MHz
*         PCLK3: 60MHz
*         PCLK4: 120MHz
*         EXCLK: 120MHz
*         HCLK:  240MHz
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void pllh_init(void)
{
    stc_clock_pll_init_t stcpllh_init;

    (void)CLK_PLLStructInit(&stcpllh_init);

    /* PLLH config */
	/* 25MHz/M*N = 25/1*48/5 =240MHz */
	/* VCO = (25/1)*48 = 1200MHz*/
    stcpllh_init.PLLCFGR = 0UL;
    stcpllh_init.PLLCFGR_f.PLLM = (1UL  - 1UL);       //1~4 任意分频可选
    stcpllh_init.PLLCFGR_f.PLLN = (48L - 1UL);        //20~480倍频
    stcpllh_init.PLLCFGR_f.PLLR = (5UL  - 1UL);       //2~16 任意分频可选
    stcpllh_init.PLLCFGR_f.PLLQ = (5UL  - 1UL);
    stcpllh_init.PLLCFGR_f.PLLP = (5UL  - 1UL);
    stcpllh_init.u8PLLState = CLK_PLL_ON;
    stcpllh_init.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;     /* Xtal = 25MHz */
    (void)CLK_PLLInit(&stcpllh_init);
}

/****************************************************************************
*函数名	: sysclock_test
*介	 绍	：系统时钟源切换测试工程
*形  参 : 无
*返回值 : 无
******************************************************************************/
void sysclock_test(void)
{
    static uint8_t i = 0;
	static stc_clock_freq_t pstcClockFreq = {NULL} ;
	static int32_t vlaue = 0;
	/*避免编译出现告警*/
	(void)i;
	(void)pstcClockFreq;
	(void)vlaue;
    /* Register write unprotected for some required peripherals. */
    //LL_PERIPH_WE(LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_GPIO | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM);
    /* Set bus clock div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));
 
    /* sram init include read/write wait cycle setting */
    SRAM_SetWaitCycle(SRAM_SRAM_ALL, SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    /* flash read wait cycle setting */
    EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    /* output system clock */
    MCO_init();
    /* Xtal initialize */
    xtal_init();
    /* Reset the VBAT area */
//    PWC_VBAT_Reset();
    /* enable Xtal32 */
    xtal32_init();
    /* MPLL initialize */
    pllh_init();
    /* enable LRC */
    (void)CLK_LrcCmd(ENABLE);
    /* enable HRC */
    (void)CLK_HrcCmd(ENABLE);
    /* Register write protected for some required peripherals. */
    //LL_PERIPH_WP(LL_PERIPH_EFM | LL_PERIPH_GPIO | LL_PERIPH_SRAM);

	/* 用于debug时进行切换测试 */
	while(1)
	{
		if(i<6)
		{
			CLK_SetSysClockSrc(au8SysClockTbl[i]);
			i = 7;
		}	
		vlaue = CLK_GetClockFreq(&pstcClockFreq);
	}
 
}
/****************************************************************************
*函数名	: sysclock_init
*介	 绍	：系统时钟初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
void sysclock_init(void)
{

    /* Register write unprotected for some required peripherals. */
    //LL_PERIPH_WE(LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_GPIO | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM);
    /* Set bus clock div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));
 
    /* sram init include read/write wait cycle setting */
    SRAM_SetWaitCycle(SRAM_SRAM_ALL, SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    /* flash read wait cycle setting */
    EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    /* MPLL initialize */
    pllh_init();	
	/* 选择时钟源 */
	CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
	
	SysTick_Init(1000);
 
}

void BSP_CLK_Init(void)
{
    stc_clock_xtal_init_t stcXtalInit;
    stc_clock_pll_init_t stcPLLHInit;

	    /* Register write unprotected for some required peripherals. */
    //LL_PERIPH_WE(LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_GPIO | LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM);
    /* PCLK0, HCLK  Max 240MHz */
    /* PCLK1, PCLK4 Max 120MHz */
    /* PCLK2, PCLK3 Max 60MHz  */
    /* EX BUS Max 120MHz */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, \
                    (CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | CLK_PCLK2_DIV4 | \
                     CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2 | CLK_EXCLK_DIV2 | \
                     CLK_HCLK_DIV1));

    (void)CLK_XtalStructInit(&stcXtalInit);
    /* Config Xtal and enable Xtal */
    stcXtalInit.u8Mode   = CLK_XTAL_MD_OSC;
    stcXtalInit.u8Drv    = CLK_XTAL_DRV_ULOW;
    stcXtalInit.u8State  = CLK_XTAL_ON;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);

    (void)CLK_PLLStructInit(&stcPLLHInit);
    /* VCO = (8/1)*120 = 960MHz*/
    stcPLLHInit.u8PLLState = CLK_PLL_ON;
    stcPLLHInit.PLLCFGR = 0UL;
    stcPLLHInit.PLLCFGR_f.PLLM = (1UL  - 1UL);       //1~4 ɎӢؖƵࠉѡ
    stcPLLHInit.PLLCFGR_f.PLLN = (48L - 1UL);        //20~480ѶƵ
    stcPLLHInit.PLLCFGR_f.PLLR = (5UL  - 1UL);       //2~16 ɎӢؖƵࠉѡ
    stcPLLHInit.PLLCFGR_f.PLLQ = (5UL  - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLP = (5UL  - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcPLLHInit);

    /* Highspeed SRAM set to 0 Read/Write wait cycle */
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);

    /* SRAM1_2_3_4_backup set to 1 Read/Write wait cycle */
    SRAM_SetWaitCycle((SRAM_SRAM123 | SRAM_SRAM4 | SRAM_SRAMB), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

    /* 0-wait @ 40MHz */
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);

    /* 4 cycles for 200 ~ 250MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT4);

    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
	SysTick_Init(1000);

}
