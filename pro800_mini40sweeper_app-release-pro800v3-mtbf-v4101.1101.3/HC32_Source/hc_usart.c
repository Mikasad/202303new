/********************************************************************************
*文件名   : hc_usart.c
*简  介   : usart配置，由官方测试例程修改获得。
*日	 期   : 2022/6/23
*作  者   : 井小飞  
********************************************************************************
*备注： 
*外部晶振为25MHz
*
********************************************************************************/

#define  USART_DEBUG_ON   0

#if  USART_DEBUG_ON
#define usart_printf(...) do {  printf(__VA_ARGS__); } while(0)
#else
#define  usart_printf(...) do { } while(0)
#endif
 
/******************************头文件*******************************************/
#include "hc_usart.h"
#include "hc_gpio.h"
#include "string.h"
#include "FreeRTOS.h"
#include  "queue.h"
#include  "hc_uart2.h"
#include "bms.h"

#define PRO800_TEST


//-----------------------------------------------------------------------------//
__IO en_flag_status_t m_recv_flag;
__IO uint16_t m_recv_len;
uint8_t g_uart2RxBuf[UART2_RX_BUF_SIZE];

//-----------------------------------------------------------------------------//
uint8_t g_uart3RxBuf[UART3_RX_BUF_SIZE];
uint8_t m_au8DataBuf_RS485_3[UART3_RX_BUF_SIZE];
stc_ring_buf_t m_stcRingBuf_RS485_3;
static __IO en_flag_status_t m_enTxCompleteFlag_RS485_3 = SET;

//-----------------------------------------------------------------------------//
uint8_t g_uart4RxBuf[UART4_RX_BUF_SIZE];
uint8_t m_au8DataBuf_RS232_1[RS232_1_RING_BUF_SIZE];
stc_ring_buf_t m_stcRingBuf_RS232_1;
static __IO en_flag_status_t m_enTxCompleteFlag_RS232_1 = SET;

//-----------------------------------------------------------------------------//
uint8_t g_uart5RxBuf[UART5_RX_BUF_SIZE];
uint8_t m_au8DataBuf_RS485_2[RS485_2_RING_BUF_SIZE];
stc_ring_buf_t m_stcRingBuf_RS485_2;
static __IO en_flag_status_t m_enTxCompleteFlag_RS485_2 = SET;

//-----------------------------------------------------------------------------//
uint8_t g_uart6RxBuf[UART6_RX_BUF_SIZE];
uint8_t m_au8DataBuf_RS232_2[RS232_2_RING_BUF_SIZE];
stc_ring_buf_t m_stcRingBuf_RS232_2;
static __IO en_flag_status_t m_enTxCompleteFlag_RS232_2 = SET;

/**
 * @brief  USART RX timeout IRQ callback.
 * @param  None
 * @retval None
 */
#include "nanopb_tcpip.h"
extern struct DEVICE_DATA      DeviceDATA ;

void usartTxString(const __IO uint8_t *u16TxString, uint32_t len, CM_USART_TypeDef *USARTx)
{
    for (uint32_t i = 0; i < len; i++)
    {
        USART_WriteData(USARTx, u16TxString[i]);
        while (RESET == USART_GetStatus(USARTx, USART_FLAG_TX_CPLT)) {
        }
    }
}

/**
 * @brief  USART RX IRQ callback
 * @param  None
 * @retval None
 */
static void RS485_2_USART_RxFull_IrqCallback(void)
{
    uint8_t u8Data = (uint8_t)USART_ReadData(RS485_2_USART_UNIT);
//	usart_printf("485_2 RECV: %02X \r\n", u8Data);

    (void)BUF_Write(&m_stcRingBuf_RS485_2, &u8Data, 1UL);
}

/**
 * @brief  USART error IRQ callback.
 * @param  None
 * @retval None
 */
static void RS485_2_USART_RxError_IrqCallback(void)
{
    if (SET == USART_GetStatus(RS485_2_USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR))) {
        (void)USART_ReadData(RS485_2_USART_UNIT);
    }

    USART_ClearStatus(RS485_2_USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

/**
 * @brief  USART error IRQ callback.
 * @param  None
 * @retval None
 */
static void RS485_3_USART_RxError_IrqCallback(void)
{
    if (SET == USART_GetStatus(RS485_3_USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR))) {
        (void)USART_ReadData(RS485_3_USART_UNIT);
    }

    USART_ClearStatus(RS485_3_USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

/**
 * @brief  USART RX IRQ callback
 * @param  None
 * @retval None
 */
static void RS485_3_USART_RxFull_IrqCallback(void)
{
    uint8_t u8Data = (uint8_t)USART_ReadData(RS485_3_USART_UNIT);
//	usart_printf("485_3 RECV: %02X\r\n", u8Data);

    (void)BUF_Write(&m_stcRingBuf_RS485_3, &u8Data, 1UL);

}

/**
 * @brief  USART error IRQ callback.
 * @param  None
 * @retval None
 */
static void RS232_1_USART_RxError_IrqCallback(void)
{
    if (SET == USART_GetStatus(RS232_1_USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR))) {
        (void)USART_ReadData(RS232_1_USART_UNIT);
    }

    USART_ClearStatus(RS232_1_USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

/**
 * @brief  USART RX IRQ callback
 * @param  None
 * @retval None
 */
static void RS232_1_USART_RxFull_IrqCallback(void)
{
    uint8_t u8Data = (uint8_t)USART_ReadData(RS232_1_USART_UNIT);

    (void)BUF_Write(&m_stcRingBuf_RS232_1, &u8Data, 1UL);
}

/**
 * @brief  USART error IRQ callback.
 * @param  None
 * @retval None
 */
static void RS232_2_USART_RxError_IrqCallback(void)
{
    if (SET == USART_GetStatus(RS232_2_USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR))) {
        (void)USART_ReadData(RS232_2_USART_UNIT);
    }

    USART_ClearStatus(RS232_2_USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

/**
 * @brief  USART RX IRQ callback
 * @param  None
 * @retval None
 */
static void RS232_2_USART_RxFull_IrqCallback(void)
{
    uint8_t u8Data = (uint8_t)USART_ReadData(RS232_2_USART_UNIT);

    (void)BUF_Write(&m_stcRingBuf_RS232_2, &u8Data, 1UL);
}

/**
 * @brief  Instal IRQ handler.
 * @param  [in] pstcConfig      Pointer to struct @ref stc_irq_signin_config_t
 * @param  [in] u32Priority     Interrupt priority
 * @retval None
 */
static void INTC_IrqInstalHandler(const stc_irq_signin_config_t *pstcConfig, uint32_t u32Priority)
{
    if (NULL != pstcConfig) {
        (void)INTC_IrqSignIn(pstcConfig);
        NVIC_ClearPendingIRQ(pstcConfig->enIRQn);
        NVIC_SetPriority(pstcConfig->enIRQn, u32Priority);
        NVIC_EnableIRQ(pstcConfig->enIRQn);
    }
}

void initUsart3(uint32_t baud_rate)
{
    stc_usart_uart_init_t stcUartInit;

    /* Configure USART RX/TX pin. */
    GPIO_SetFunc(RS485_3_USART_RX_PORT, RS485_3_USART_RX_PIN, RS485_3_USART_RX_GPIO_FUNC);
    GPIO_SetFunc(RS485_3_USART_TX_PORT, RS485_3_USART_TX_PIN, RS485_3_USART_TX_GPIO_FUNC);

    /* Enable peripheral clock */
    RS485_3_USART_FCG_ENABLE();

    /* Initialize ring buffer function. */
    (void)BUF_Init(&m_stcRingBuf_RS485_3, m_au8DataBuf_RS485_3, sizeof(m_au8DataBuf_RS485_3));

    /* Initialize UART. */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV64;
    stcUartInit.u32Baudrate = baud_rate;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(RS485_3_USART_UNIT, &stcUartInit, NULL)) {
        for (;;) {
        }
    }
}

void usart3NVIC()
{
    stc_irq_signin_config_t stcIrqSigninConfig;

    /* Register RX error IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = RS485_3_USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc = RS485_3_USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &RS485_3_USART_RxError_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_04);

    /* Register RX full IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = RS485_3_USART_RX_FULL_IRQn;
    stcIrqSigninConfig.enIntSrc = RS485_3_USART_RX_FULL_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &RS485_3_USART_RxFull_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_04);

    /* Register TX empty IRQ handler && configure NVIC. */
//    stcIrqSigninConfig.enIRQn = RS485_3_USART_TX_EMPTY_IRQn;
//    stcIrqSigninConfig.enIntSrc = RS485_3_USART_TX_EMPTY_INT_SRC;
//    stcIrqSigninConfig.pfnCallback = &RS485_3_USART_TxEmpty_IrqCallback;
//    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register TX complete IRQ handler && configure NVIC. */
//    stcIrqSigninConfig.enIRQn = RS485_3_USART_TX_CPLT_IRQn;
//    stcIrqSigninConfig.enIntSrc = RS485_3_USART_TX_CPLT_INT_SRC;
//    stcIrqSigninConfig.pfnCallback = &RS485_3_USART_TxComplete_IrqCallback;
//    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);
}

void enableUsart3()
{
    /* Enable RX function */
    USART_FuncCmd(RS485_3_USART_UNIT, (USART_RX | USART_INT_RX), ENABLE);
    USART_FuncCmd(RS485_3_USART_UNIT, (USART_TX | USART_INT_TX_EMPTY), ENABLE);
}

void initUsart4()
{
    stc_usart_uart_init_t stcUartInit;

    /* Configure USART RX/TX pin. */
    GPIO_SetFunc(RS232_1_USART_RX_PORT, RS232_1_USART_RX_PIN, RS232_1_USART_RX_GPIO_FUNC);
    GPIO_SetFunc(RS232_1_USART_TX_PORT, RS232_1_USART_TX_PIN, RS232_1_USART_TX_GPIO_FUNC);

    /* Enable peripheral clock */
    RS232_1_USART_FCG_ENABLE();

    /* Initialize ring buffer function. */
    (void)BUF_Init(&m_stcRingBuf_RS232_1, m_au8DataBuf_RS232_1, sizeof(m_au8DataBuf_RS232_1));

    /* Initialize UART. */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV64;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(RS232_1_USART_UNIT, &stcUartInit, NULL)) {
        for (;;) {
        }
    }
}

void usart4NVIC()
{
    stc_irq_signin_config_t stcIrqSigninConfig;

    /* Register RX error IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = RS232_1_USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc = RS232_1_USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &RS232_1_USART_RxError_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register RX full IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = RS232_1_USART_RX_FULL_IRQn;
    stcIrqSigninConfig.enIntSrc = RS232_1_USART_RX_FULL_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &RS232_1_USART_RxFull_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register TX empty IRQ handler && configure NVIC. */
//    stcIrqSigninConfig.enIRQn = RS232_1_USART_TX_EMPTY_IRQn;
//    stcIrqSigninConfig.enIntSrc = RS232_1_USART_TX_EMPTY_INT_SRC;
//    stcIrqSigninConfig.pfnCallback = &RS232_1_USART_TxEmpty_IrqCallback;
//    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register TX complete IRQ handler && configure NVIC. */
//    stcIrqSigninConfig.enIRQn = RS232_1_USART_TX_CPLT_IRQn;
//    stcIrqSigninConfig.enIntSrc = RS232_1_USART_TX_CPLT_INT_SRC;
//    stcIrqSigninConfig.pfnCallback = &RS232_1_USART_TxComplete_IrqCallback;
//    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);
}

void enableUsart4()
{
    /* Enable RX function */
    USART_FuncCmd(RS232_1_USART_UNIT, (USART_RX | USART_INT_RX), ENABLE);
    USART_FuncCmd(RS232_1_USART_UNIT, (USART_TX | USART_INT_TX_EMPTY), ENABLE);
}

void initUsart5()
{
    stc_usart_uart_init_t stcUartInit;

    /* Configure USART RX/TX pin. */
    GPIO_SetFunc(RS485_2_USART_RX_PORT, RS485_2_USART_RX_PIN, RS485_2_USART_RX_GPIO_FUNC);
    GPIO_SetFunc(RS485_2_USART_TX_PORT, RS485_2_USART_TX_PIN, RS485_2_USART_TX_GPIO_FUNC);

    /* Enable peripheral clock */
    RS485_2_USART_FCG_ENABLE();

    /* Initialize ring buffer function. */
    (void)BUF_Init(&m_stcRingBuf_RS485_2, m_au8DataBuf_RS485_2, sizeof(m_au8DataBuf_RS485_2));

    /* Initialize UART. */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV64;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(RS485_2_USART_UNIT, &stcUartInit, NULL)) {
        for (;;) {
        }
    }
}

void usart5NVIC()
{
    stc_irq_signin_config_t stcIrqSigninConfig;

    /* Register RX error IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = RS485_2_USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc = RS485_2_USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &RS485_2_USART_RxError_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register RX full IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = RS485_2_USART_RX_FULL_IRQn;
    stcIrqSigninConfig.enIntSrc = RS485_2_USART_RX_FULL_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &RS485_2_USART_RxFull_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register TX empty IRQ handler && configure NVIC. */
//    stcIrqSigninConfig.enIRQn = RS485_3_USART_TX_EMPTY_IRQn;
//    stcIrqSigninConfig.enIntSrc = RS485_3_USART_TX_EMPTY_INT_SRC;
//    stcIrqSigninConfig.pfnCallback = &RS485_3_USART_TxEmpty_IrqCallback;
//    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register TX complete IRQ handler && configure NVIC. */
//    stcIrqSigninConfig.enIRQn = RS485_3_USART_TX_CPLT_IRQn;
//    stcIrqSigninConfig.enIntSrc = RS485_3_USART_TX_CPLT_INT_SRC;
//    stcIrqSigninConfig.pfnCallback = &RS485_3_USART_TxComplete_IrqCallback;
//    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);
}

void enableUsart5()
{
    /* Enable RX function */
    USART_FuncCmd(RS485_2_USART_UNIT, (USART_RX | USART_INT_RX), ENABLE);
    USART_FuncCmd(RS485_2_USART_UNIT, (USART_TX | USART_INT_TX_EMPTY), ENABLE);
}


void initUsart6()
{
    stc_usart_uart_init_t stcUartInit;

    /* Configure USART RX/TX pin. */
    GPIO_SetFunc(RS232_2_USART_RX_PORT, RS232_2_USART_RX_PIN, RS232_2_USART_RX_GPIO_FUNC);
    GPIO_SetFunc(RS232_2_USART_TX_PORT, RS232_2_USART_TX_PIN, RS232_2_USART_TX_GPIO_FUNC);

    /* Enable peripheral clock */
    RS232_2_USART_FCG_ENABLE();

    /* Initialize ring buffer function. */
    (void)BUF_Init(&m_stcRingBuf_RS232_2, m_au8DataBuf_RS232_2, sizeof(m_au8DataBuf_RS232_2));

    /* Initialize UART. */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV64;
    stcUartInit.u32Baudrate = 115200UL;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(RS232_2_USART_UNIT, &stcUartInit, NULL)) {
        for (;;) {
        }
    }
}

void usart6NVIC()
{
    stc_irq_signin_config_t stcIrqSigninConfig;

    /* Register RX error IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = RS232_2_USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc = RS232_2_USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &RS232_2_USART_RxError_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register RX full IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = RS232_2_USART_RX_FULL_IRQn;
    stcIrqSigninConfig.enIntSrc = RS232_2_USART_RX_FULL_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &RS232_2_USART_RxFull_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register TX empty IRQ handler && configure NVIC. */
//    stcIrqSigninConfig.enIRQn = RS232_1_USART_TX_EMPTY_IRQn;
//    stcIrqSigninConfig.enIntSrc = RS232_1_USART_TX_EMPTY_INT_SRC;
//    stcIrqSigninConfig.pfnCallback = &RS232_1_USART_TxEmpty_IrqCallback;
//    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);

    /* Register TX complete IRQ handler && configure NVIC. */
//    stcIrqSigninConfig.enIRQn = RS232_1_USART_TX_CPLT_IRQn;
//    stcIrqSigninConfig.enIntSrc = RS232_1_USART_TX_CPLT_INT_SRC;
//    stcIrqSigninConfig.pfnCallback = &RS232_1_USART_TxComplete_IrqCallback;
//    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_DEFAULT);
}

void enableUsart6()
{
    /* Enable RX function */
    USART_FuncCmd(RS232_2_USART_UNIT, (USART_RX | USART_INT_RX), ENABLE);
    USART_FuncCmd(RS232_2_USART_UNIT, (USART_TX | USART_INT_TX_EMPTY), ENABLE);
}

/****************************************************************************
*函数名	: hc_usart3_init
*介	 绍	：USART3端口初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_usart3_init(uint32_t baud_rate)
{
	initUsart3(baud_rate);
	usart3NVIC();
	enableUsart3();
}

/****************************************************************************
*函数名	: hc_usart4_init
*介	 绍	：USART4端口初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_usart4_init(void)
{
	initUsart4();
	usart4NVIC();
	enableUsart4();
}

/****************************************************************************
*函数名	: hc_usart5_init
*介	 绍	：USART5端口初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_usart5_init(void)
{
	initUsart5();
	usart5NVIC();
	enableUsart5();
}

/****************************************************************************
*函数名	: hc_usart6_init
*介	 绍	：USART6端口初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_usart6_init(void)
{
	initUsart6();
	usart6NVIC();
	enableUsart6();
}

void hc_usart3_transmit(const __IO uint8_t *u16TxString, const uint32_t len)
{
	GPIO_SetPins(RS485_CONTROL3_GPIO_Port, RS485_CONTROL3_Pin);
	
	DDL_DelayUS(12);
	USART_FuncCmd(RS485_3_USART_UNIT, (USART_RX | USART_INT_RX), DISABLE);
	//portENTER_CRITICAL();
	usartTxString(u16TxString, len, RS485_3_USART_UNIT);
	/*Configure GPIO pin Output Level */
	GPIO_ResetPins(RS485_CONTROL3_GPIO_Port, RS485_CONTROL3_Pin);
	USART_FuncCmd(RS485_3_USART_UNIT, (USART_RX | USART_INT_RX), ENABLE);
	//portEXIT_CRITICAL();

}

void hc_usart4_transmit(const __IO uint8_t *u16TxString, const uint32_t len)
{
	usartTxString(u16TxString, len, RS232_1_USART_UNIT);
}

void hc_usart5_transmit(const __IO uint8_t *u16TxString, const uint32_t len)
{
	GPIO_SetPins(RS485_CONTROL2_GPIO_Port, RS485_CONTROL2_Pin);
	//portENTER_CRITICAL();
	USART_FuncCmd(RS485_2_USART_UNIT, (USART_RX | USART_INT_RX), DISABLE);
 	usartTxString(u16TxString, len, RS485_2_USART_UNIT);
	GPIO_ResetPins(RS485_CONTROL2_GPIO_Port, RS485_CONTROL2_Pin);
	USART_FuncCmd(RS485_2_USART_UNIT, (USART_RX | USART_INT_RX), ENABLE);
	//portEXIT_CRITICAL();

}

void hc_usart6_transmit(const __IO uint8_t *u16TxString, const uint32_t len)
{
	usartTxString(u16TxString, len, RS232_2_USART_UNIT);
}

void hc_usart3_test(void)
{
	uint8_t i = 0;
	uint8_t len=0;
	uint8_t testStr[] = "UART3_TEST";

	GPIO_ResetPins(RS485_CONTROL3_GPIO_Port, RS485_CONTROL3_Pin);

	HAL_Delay(50);

	len = BUF_UsedSize(&m_stcRingBuf_RS485_3);
	BUF_Read(&m_stcRingBuf_RS485_3, g_uart3RxBuf, len);
}

void hc_usart4_test(void)
{
	uint8_t i = 0;
	uint8_t len = 0;
	uint8_t testStr[] = "UART4_TEST";

	usart_printf("UART4 send: %s\r\n", testStr);
	hc_usart4_transmit(testStr, strlen((char *)testStr));
	HAL_Delay(5);
	len = BUF_UsedSize(&m_stcRingBuf_RS232_1);
	len = BUF_Read(&m_stcRingBuf_RS232_1, g_uart4RxBuf, len);
	usart_printf("UART4 recv:");
	for (i = 0 ; i < len; i++)
		usart_printf(" %02X", g_uart4RxBuf[i]);
	usart_printf("\r\n");
}

void hc_usart5_test(void)
{
	uint8_t i = 0;
	uint8_t len = 0;
	uint8_t testStr[] = "UART5_TEST";

	usart_printf("UART5 send: %s\r\n", testStr);
	hc_usart5_transmit(testStr, strlen((char *)testStr));
	HAL_Delay(5);
	len = BUF_UsedSize(&m_stcRingBuf_RS485_2);
	BUF_Read(&m_stcRingBuf_RS485_2, g_uart5RxBuf, len);
	usart_printf("UART5 recv:");
	for (i = 0 ; i < len; i++)
		usart_printf(" %02X", g_uart5RxBuf[i]);
	usart_printf("\r\n");
}

void hc_usart6_test(void)
{
	uint8_t i = 0;
	uint8_t len = 0;
	uint8_t testStr[] = "UART6_TEST";

	usart_printf("UART6 send: %s\r\n", testStr);
	hc_usart6_transmit(testStr, strlen((char *)testStr));
	HAL_Delay(5);
	len = BUF_UsedSize(&m_stcRingBuf_RS232_2);
	len = BUF_Read(&m_stcRingBuf_RS232_2, g_uart6RxBuf, len);
	usart_printf("UART6 recv:");
	for (i = 0 ; i < len; i++)
		usart_printf(" %02X", g_uart6RxBuf[i]);
	usart_printf("\r\n");
}


