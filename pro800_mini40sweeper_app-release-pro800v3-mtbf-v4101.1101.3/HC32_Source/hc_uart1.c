///*
// * @Author: Jack Yi
// * @Date: 2022-08-31 15:20:26
// * @LastEditors: Jack Yi
// * @LastEditTime: 2022-08-31 16:45:26
// * @FilePath: \m_line\HC32\hardware\hc_uart.c
// * @Description: 
// * 
// * Copyright (c) 2022 by Jack Yi, All Rights Reserved. 
// */

#include "hc32_ll_usart.h"
#include "hc_uart1.h"
#include "stdio.h"
#include "string.h"
#include "hc_gpio.h"
#include "bms.h"
#include "hc32_ll_utility.h"

//#define USART1_USE_DMA

/**
 * gd32通信串口 ： UART4   PC10_TX PC11_RX 
 * imu通信串口 ： UART2   PD5_TX PD6_RX 
 * 功率版板通信串口 ： UART6   PG14_TX PG9_RX 
 * @Date: 2022-08-31 15:22:05
 */

QueueHandle_t bms_usart1_quene_handle = NULL;

uint8_t m_au8TxBuf1[APP_FRAME_LEN_MAX] ={0};
uint8_t m_au8RxBuf1[APP_FRAME_LEN_MAX] = {0};

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

/**
 * @brief  DMA transfer complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void RX_DMA_TC_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, USART_RX_TIMEOUT, DISABLE);
    DMA_ClearTransCompleteStatus(RX_DMA_UNIT, RX_DMA_TC_FLAG);
}

/**
 * @brief  DMA transfer complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void TX_DMA_TC_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, USART_INT_TX_CPLT, ENABLE);
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, TX_DMA_TC_FLAG);
}

/**
 * @brief  Initialize DMA.
 * @param  None
 * @retval int32_t:
 *           - LL_OK:                   Initialize successfully.
 *           - LL_ERR_INVD_PARAM:       Initialization paramters is invalid.
 */
static int32_t DMA_Config(void)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;
    stc_dma_llp_init_t stcDmaLlpInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    static stc_dma_llp_descriptor_t stcLlpDesc;

    /* DMA&AOS FCG enable */
    RX_DMA_FCG_ENABLE();
    TX_DMA_FCG_ENABLE();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* USART_RX_DMA */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8RxBuf1);
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr = (uint32_t)m_au8RxBuf1;
    stcDmaInit.u32SrcAddr = ((uint32_t)(&USART_UNIT->DR) + 2UL);
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    i32Ret = DMA_Init(RX_DMA_UNIT, RX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        (void)DMA_LlpStructInit(&stcDmaLlpInit);
        stcDmaLlpInit.u32State = DMA_LLP_ENABLE;
        stcDmaLlpInit.u32Mode  = DMA_LLP_WAIT;
        stcDmaLlpInit.u32Addr  = (uint32_t)&stcLlpDesc;
        (void)DMA_LlpInit(RX_DMA_UNIT, RX_DMA_CH, &stcDmaLlpInit);

        stcLlpDesc.SARx   = stcDmaInit.u32SrcAddr;
        stcLlpDesc.DARx   = stcDmaInit.u32DestAddr;
        stcLlpDesc.DTCTLx = (stcDmaInit.u32TransCount << DMA_DTCTL_CNT_POS) | (stcDmaInit.u32BlockSize << DMA_DTCTL_BLKSIZE_POS);;
        stcLlpDesc.LLPx   = (uint32_t)&stcLlpDesc;
        stcLlpDesc.CHCTLx = stcDmaInit.u32SrcAddrInc | stcDmaInit.u32DestAddrInc | stcDmaInit.u32DataWidth |  \
                            stcDmaInit.u32IntEn      | stcDmaLlpInit.u32State    | stcDmaLlpInit.u32Mode;

        DMA_ReconfigLlpCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
        DMA_ReconfigCmd(RX_DMA_UNIT, ENABLE);
        AOS_SetTriggerEventSrc(RX_DMA_RECONF_TRIG_SEL, RX_DMA_RECONF_TRIG_EVT_SRC);

        stcIrqSignConfig.enIntSrc = RX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn  = RX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &RX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_05);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(RX_DMA_TRIG_SEL, RX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(RX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(RX_DMA_UNIT, RX_DMA_TC_INT, ENABLE);
        (void)DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);
    }

    /* USART_TX_DMA */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8TxBuf1);
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr = (uint32_t)(&USART_UNIT->DR);
    stcDmaInit.u32SrcAddr = (uint32_t)m_au8TxBuf1;
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    i32Ret = DMA_Init(TX_DMA_UNIT, TX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        stcIrqSignConfig.enIntSrc = TX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn  = TX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &TX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_05);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(TX_DMA_TRIG_SEL, TX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(TX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(TX_DMA_UNIT, TX_DMA_TC_INT, ENABLE);
    }

    return i32Ret;
}

/**
 * @brief  Configure TMR0.
 * @param  [in] u16TimeoutBits:         Timeout bits
 * @retval None
 */
static void TMR0_Config(uint16_t u16TimeoutBits)
{
    uint16_t u16CompareValue;
    stc_tmr0_init_t stcTmr0Init;

    TMR0_FCG_ENABLE();

    /* Initialize TMR0 base function. */
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_XTAL32;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV2;
    stcTmr0Init.u32Func     = TMR0_FUNC_CMP;
    if (TMR0_CLK_DIV1 == stcTmr0Init.u32ClockDiv) {
        u16CompareValue = (u16TimeoutBits - 4U);
    } else if (TMR0_CLK_DIV2 == stcTmr0Init.u32ClockDiv) {
        u16CompareValue = (u16TimeoutBits / 2U - 2U);
    } else {
        u16CompareValue = (u16TimeoutBits / ((uint16_t)1U << (stcTmr0Init.u32ClockDiv >> TMR0_BCONR_CKDIVA_POS)) - 1U);
    }
    stcTmr0Init.u16CompareValue = u16CompareValue;
    (void)TMR0_Init(TMR0_UNIT, TMR0_CH, &stcTmr0Init);

    TMR0_HWStartCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
    TMR0_HWClearCondCmd(TMR0_UNIT, TMR0_CH, ENABLE);
		
}

/**
 * @brief  USART RX timeout IRQ callback.
 * @param  None
 * @retval None
 */
static void USART_RxTimeout_IrqCallback(void)
{
    bms_pack_t bms_pack;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    bms_pack.len = APP_FRAME_LEN_MAX - (uint16_t)DMA_GetTransCount(RX_DMA_UNIT, RX_DMA_CH);

    TMR0_Stop(TMR0_UNIT, TMR0_CH);
    USART_ClearStatus(USART_UNIT, USART_FLAG_RX_TIMEOUT);

    memcpy(bms_pack.rx_data, m_au8RxBuf1, bms_pack.len);

    DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, DISABLE);
    DMA_SetDestAddr(RX_DMA_UNIT,RX_DMA_CH , (uint32_t) m_au8RxBuf1);
    DMA_SetTransCount(RX_DMA_UNIT, RX_DMA_CH, APP_FRAME_LEN_MAX);
    DMA_ChCmd(RX_DMA_UNIT, RX_DMA_CH, ENABLE);

    if (bms_usart1_quene_handle != NULL)
    {
        xQueueSendFromISR(bms_usart1_quene_handle, &bms_pack, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }   
}

/**
 * @brief  USART RX IRQ callback
 * @param  None
 * @retval None
 */
#define BMS_PACK_LEN 75 /* 一帧数据长度 */
static bms_pack_t bms_pack;
static void USART_RxFull_IrqCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint8_t data = (uint8_t)USART_ReadData(USART_UNIT);

    bms_pack.rx_data[bms_pack.len] = data;
    bms_pack.len++;
    if( (bms_usart1_quene_handle != NULL) && (bms_pack.len >= BMS_PACK_LEN) )
    {
        xQueueSendFromISR(bms_usart1_quene_handle, &bms_pack, &xHigherPriorityTaskWoken);
        bms_pack.len = 0;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  USART TX complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void USART_TxComplete_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    TMR0_Stop(TMR0_UNIT, TMR0_CH);
    USART_ClearStatus(USART_UNIT, USART_FLAG_RX_TIMEOUT);
    USART_FuncCmd(USART_UNIT, USART_RX_TIMEOUT, ENABLE);
    USART_ClearStatus(USART_UNIT, USART_FLAG_TX_CPLT);
    GPIO_ResetPins(RS485_CONTROL1_GPIO_Port, RS485_CONTROL1_Pin);
}
static void USART_RxError_IrqCallback(void)
{
    if (SET == USART_GetStatus(USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR))) {
        (void)USART_ReadData(USART_UNIT);
    }
    USART_ClearStatus(USART_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

#ifdef USART1_USE_DMA
/**
 * @brief 串口1 带 dma接收初始化
 */
static void hc_usart1_init(void)
{
    float f32Error;
	stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;
  	
    stc_gpio_init_t stcGpioInit;
    (void)GPIO_StructInit(&stcGpioInit);

    //485ctr pin
    stcGpioInit.u16PinState = PIN_STAT_RST;
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinDrv = PIN_LOW_DRV ; //低速推挽输出
    (void)GPIO_Init(RS485_CONTROL1_GPIO_Port, RS485_CONTROL1_Pin, &stcGpioInit);
    GPIO_ResetPins(RS485_CONTROL1_GPIO_Port, RS485_CONTROL1_Pin);

    /* Initialize DMA. */
    (void)DMA_Config();

    /* Initialize TMR0. */
    TMR0_Config(USART_TIMEOUT_BITS);

    /* Configure USART RX/TX pin. */
    GPIO_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_GPIO_FUNC);
    GPIO_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_GPIO_FUNC);

    /* Enable peripheral clock */
    USART_FCG_ENABLE();

    /* Initialize UART. */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV64;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_ENABLE;
    stcUartInit.u32Baudrate = USART_BAUDRATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(USART_UNIT, &stcUartInit, NULL)) {

        for (;;) {
        }
    }

    for (uint32_t u32Div = 0UL; u32Div <= USART_CLK_DIV64; u32Div++) 
    {
        USART_SetClockDiv(USART_UNIT, u32Div);
        USART_SetBaudrate(USART_UNIT, USART_BAUDRATE, &f32Error);
        if ((-0.025F <= f32Error) && (f32Error <= 0.025F)) 
        {
            break;
        } 
    }
    /* Register RX error IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxError_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_05);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* Register TX complete IRQ handler. */
    stcIrqSigninConfig.enIRQn = USART_TX_CPLT_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_TX_CPLT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_TxComplete_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_05);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* Register RX timeout IRQ handler. */
    stcIrqSigninConfig.enIRQn = USART_RX_TIMEOUT_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_RX_TIMEOUT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxTimeout_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_05);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* Enable TX && RX && RX interrupt function */
    USART_FuncCmd(USART_UNIT, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT | \
                               USART_INT_RX_TIMEOUT), ENABLE);
}
#else
/**
 * @brief 串口1初始化
 * @param  baud_rate 波特率
 */
static void hc_usart1_init(void)
{
    stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;

    GPIO_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_GPIO_FUNC);
    GPIO_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_GPIO_FUNC);

    USART_FCG_ENABLE();

    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV64;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_ENABLE;
    stcUartInit.u32Baudrate = USART_BAUDRATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(USART_UNIT, &stcUartInit, NULL)) {

        for (;;) {
        }
    }

    /* Register RX error IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = USART_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxError_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_05);

    /* Register RX full IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = USART_RX_FULL_IRQn;
    stcIrqSigninConfig.enIntSrc = USART_RX_FULL_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART_RxFull_IrqCallback;
    INTC_IrqInstalHandler(&stcIrqSigninConfig, DDL_IRQ_PRIO_05);

    USART_FuncCmd(USART_UNIT, (USART_RX | USART_INT_RX), ENABLE);
    USART_FuncCmd(USART_UNIT, (USART_TX | USART_INT_TX_EMPTY), ENABLE);
}
#endif

/**
 * @brief 串口1 初始化
 * @return int16_t 
 */
int16_t bsp_usart1_init(void)
{
    bms_usart1_quene_handle = xQueueCreate(8, sizeof(bms_pack_t));
    if (bms_usart1_quene_handle == NULL) 
    {
        printf("uart1 Create queue error.\r\n");
        return -1;
    }

    hc_usart1_init();
    
    return 0;
}

#ifdef USART1_USE_DMA
/**
 * @brief 串口1 dma发送
 * @param  data              Param doc
 * @param  len               Param doc
 */
void usart1_485_send(uint8_t *data, uint8_t len)
{
    GPIO_SetPins(RS485_CONTROL1_GPIO_Port, RS485_CONTROL1_Pin);
    memcpy(m_au8TxBuf1, data, len);
    DMA_SetSrcAddr(TX_DMA_UNIT, TX_DMA_CH, (uint32_t)m_au8TxBuf1);
    DMA_SetTransCount(TX_DMA_UNIT, TX_DMA_CH, len);
    (void)DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, ENABLE);
    USART_FuncCmd(USART_UNIT, USART_TX, ENABLE);
}
#else
/**
 * @brief 
 */
void usart1_485_send(uint8_t *data, uint8_t len)
{
    GPIO_SetPins(RS485_CONTROL1_GPIO_Port, RS485_CONTROL1_Pin);
    USART_FuncCmd(USART_UNIT, (USART_RX | USART_INT_RX), DISABLE);
    vTaskDelay(1);
    for (uint32_t i = 0; i < len; i++)
    {
        USART_WriteData(USART_UNIT, data[i]);
        while (RESET == USART_GetStatus(USART_UNIT, USART_FLAG_TX_CPLT)) {
        }
    }
    DDL_DelayUS(1);
    GPIO_ResetPins(RS485_CONTROL1_GPIO_Port, RS485_CONTROL1_Pin);
    USART_FuncCmd(USART_UNIT, (USART_RX | USART_INT_RX), ENABLE); 
}
#endif
