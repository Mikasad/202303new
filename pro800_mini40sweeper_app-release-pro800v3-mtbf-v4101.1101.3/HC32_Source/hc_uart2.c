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

#include "main.h"
#include "hc_uart2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hc32_ll_tmr0.h"
#include "hc32_ll_usart.h"
#include "queue.h"
#include "stdint.h"
#include "stdio.h"
#include "message_buffer.h"
#include "string.h"
#include "imu_task.h"

/**
 * gd32通信串口 ： UART4   PC10_TX PC11_RX
 * imu通信串口 ： UART2   PD5_TX PD6_RX
 * 功率版板通信串口 ： UART6   PG14_TX PG9_RX
 * @Date: 2022-08-31 15:22:05
 */

QueueHandle_t imu_pack_quene_handle = NULL;
volatile uint8_t m_au8RxBuf[USART2_FRAME_LEN_MAX];
volatile uint8_t m_au8TxBuf[USART2_FRAME_LEN_MAX] = {0};

/**
 * @brief  DMA transfer complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void UART2_RX_DMA_TC_IrqCallback(void)
{
    USART_FuncCmd(USART2_UNIT, USART_RX_TIMEOUT, DISABLE);
    DMA_ClearTransCompleteStatus(UART2_RX_DMA_UNIT, UART2_RX_DMA_TC_FLAG);
}

/**
 * @brief  DMA transfer complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void UART2_TX_DMA_TC_IrqCallback(void)
{
    USART_FuncCmd(USART2_UNIT, USART_INT_TX_CPLT, ENABLE);
    USART_FuncCmd(USART2_UNIT, USART_TX, ENABLE);
    DMA_ClearTransCompleteStatus(UART2_TX_DMA_UNIT, UART2_TX_DMA_TC_FLAG);
}

/**
 * @brief  Initialize DMA.
 * @param  None
 * @retval int32_t:
 *           - LL_OK:                   Initialize successfully.
 *           - LL_ERR_INVD_PARAM:       Initialization paramters is invalid.
 */
static int32_t USART2_DMA_Config(void)
{
    int32_t i32Ret;
    stc_dma_init_t stcDmaInit;
    stc_dma_llp_init_t stcDmaLlpInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    static stc_dma_llp_descriptor_t stcLlpDesc;

    /* DMA&AOS FCG enable */
    UART2_RX_DMA_FCG_ENABLE();
    UART2_TX_DMA_FCG_ENABLE();
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* USART_RX_DMA */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8RxBuf);
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr = (uint32_t)m_au8RxBuf;
    stcDmaInit.u32SrcAddr = ((uint32_t)(&USART2_UNIT->DR) + 2UL);
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    i32Ret = DMA_Init(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        (void)DMA_LlpStructInit(&stcDmaLlpInit);
        stcDmaLlpInit.u32State = DMA_LLP_ENABLE;
        stcDmaLlpInit.u32Mode  = DMA_LLP_WAIT;
        stcDmaLlpInit.u32Addr  = (uint32_t)&stcLlpDesc;
        (void)DMA_LlpInit(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, &stcDmaLlpInit);

        stcLlpDesc.SARx   = stcDmaInit.u32SrcAddr;
        stcLlpDesc.DARx   = stcDmaInit.u32DestAddr;
        stcLlpDesc.DTCTLx = (stcDmaInit.u32TransCount << DMA_DTCTL_CNT_POS) | (stcDmaInit.u32BlockSize << DMA_DTCTL_BLKSIZE_POS);;
        stcLlpDesc.LLPx   = (uint32_t)&stcLlpDesc;
        stcLlpDesc.CHCTLx = stcDmaInit.u32SrcAddrInc | stcDmaInit.u32DestAddrInc | stcDmaInit.u32DataWidth |  \
                            stcDmaInit.u32IntEn      | stcDmaLlpInit.u32State    | stcDmaLlpInit.u32Mode;

        DMA_ReconfigLlpCmd(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, ENABLE);
        DMA_ReconfigCmd(UART2_RX_DMA_UNIT, ENABLE);
        //AOS_SetTriggerEventSrc(UART2_RX_DMA_RECONF_TRIG_SEL, UART2_RX_DMA_RECONF_TRIG_EVT_SRC);

        stcIrqSignConfig.enIntSrc = UART2_RX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn  = UART2_RX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &UART2_RX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_05);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(UART2_RX_DMA_TRIG_SEL, UART2_RX_DMA_TRIG_EVT_SRC);  //暂时不用aos系统

        DMA_Cmd(UART2_RX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(UART2_RX_DMA_UNIT, UART2_RX_DMA_TC_INT, ENABLE);
        (void)DMA_ChCmd(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, ENABLE);
    }

    /* USART_TX_DMA */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCount = ARRAY_SZ(m_au8TxBuf);
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr = (uint32_t)(&USART2_UNIT->DR);
    stcDmaInit.u32SrcAddr = (uint32_t)m_au8TxBuf;
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    i32Ret = DMA_Init(UART2_TX_DMA_UNIT, UART2_TX_DMA_CH, &stcDmaInit);
    if (LL_OK == i32Ret) {
        stcIrqSignConfig.enIntSrc = UART2_TX_DMA_TC_INT_SRC;
        stcIrqSignConfig.enIRQn  = UART2_TX_DMA_TC_IRQn;
        stcIrqSignConfig.pfnCallback = &UART2_TX_DMA_TC_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrqSignConfig);
        NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
        NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_06);
        NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);

        AOS_SetTriggerEventSrc(UART2_TX_DMA_TRIG_SEL, UART2_TX_DMA_TRIG_EVT_SRC);

        DMA_Cmd(UART2_TX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(UART2_TX_DMA_UNIT, UART2_TX_DMA_TC_INT, ENABLE);
    }

    return i32Ret;
}

/**
 * @brief  Configure TMR0.
 * @param  [in] u16TimeoutBits:         Timeout bits
 * @retval None
 */
static void USART2_TMR0_Config(uint16_t u16TimeoutBits)
{
    uint16_t u16CompareValue;
    stc_tmr0_init_t stcTmr0Init;

    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE);

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
    (void)TMR0_Init(UART2_TMR0_UNIT, UART2_TMR0_CH, &stcTmr0Init);

    TMR0_HWStartCondCmd(UART2_TMR0_UNIT, UART2_TMR0_CH, ENABLE);
    TMR0_HWClearCondCmd(UART2_TMR0_UNIT, UART2_TMR0_CH, ENABLE);
}

/**
 * @brief  USART RX timeout IRQ callback.
 * @param  None
 * @retval None
 */
static void USART2_RxTimeout_IrqCallback(void)
{
    imu_pack_t imu_pack;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    imu_pack.len = USART2_FRAME_LEN_MAX - (uint16_t)DMA_GetTransCount(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH);

	TMR0_Stop(UART2_TMR0_UNIT, UART2_TMR0_CH);
    USART_ClearStatus(USART2_UNIT, USART_FLAG_RX_TIMEOUT);
    DMA_ChCmd(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, DISABLE);

	if( imu_pack.len <= USART2_FRAME_LEN_MAX )
    	memcpy(imu_pack.rx_data, (uint8_t*)m_au8RxBuf, imu_pack.len);
    /*重新配置dma*/
	
    DMA_SetDestAddr(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, (uint32_t)m_au8RxBuf);
    DMA_SetTransCount(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, USART2_FRAME_LEN_MAX);
    DMA_SetBlockSize(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, 1);
    DMA_ChCmd(UART2_RX_DMA_UNIT, UART2_RX_DMA_CH, ENABLE);
    
    if (imu_pack_quene_handle != NULL)
    {
        xQueueSendFromISR(imu_pack_quene_handle, &imu_pack, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }   
}

/**
 * @brief  USART TX complete IRQ callback function.
 * @param  None
 * @retval None
 */
static void USART2_TxComplete_IrqCallback(void)
{
    USART_FuncCmd(USART2_UNIT, (USART_TX | USART_INT_TX_CPLT), DISABLE);
    TMR0_Stop(UART2_TMR0_UNIT, UART2_TMR0_CH);
    USART_ClearStatus(USART2_UNIT, USART_FLAG_RX_TIMEOUT);
    USART_FuncCmd(USART2_UNIT, USART_RX_TIMEOUT, ENABLE);
    USART_ClearStatus(USART2_UNIT, USART_FLAG_TX_CPLT);
}

/**
 * @brief  USART RX error IRQ callback.
 * @param  None
 * @retval None
 */
static void USART2_RxError_IrqCallback(void)
{
    if (SET == USART_GetStatus(USART2_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR))) {
        (void)USART_ReadData(USART2_UNIT);
    }
    USART_ClearStatus(USART2_UNIT, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

/**
 * @brief 初始化串口2
 */
static void hc_usart2_init(void)
{
    float f32Error = 0;
    
    (void)USART2_DMA_Config();

    /* Initialize TMR0. */
    USART2_TMR0_Config(USART2_TIMEOUT_BITS);
    stc_usart_uart_init_t stcUartInit;
    stc_irq_signin_config_t stcIrqSigninConfig;

    GPIO_SetFunc(USART2_RX_PORT, USART2_RX_PIN, USART2_RX_GPIO_FUNC);
    GPIO_SetFunc(USART2_TX_PORT, USART2_TX_PIN, USART2_TX_GPIO_FUNC);

    /* Enable peripheral clock */
    USART2_FCG_ENABLE();

    /* Initialize UART. */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32ClockDiv = USART_CLK_DIV64;
    stcUartInit.u32CKOutput = USART_CK_OUTPUT_ENABLE;
    stcUartInit.u32Baudrate = USART2_BAUDRATE;
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    if (LL_OK != USART_UART_Init(USART2_UNIT, &stcUartInit, NULL)) {
        for (;;) {
        }
    }

    for (uint32_t u32Div = 0; u32Div <= USART_CLK_DIV64; u32Div++)
    {
        USART_SetClockDiv(CM_USART2, u32Div);
        USART_SetBaudrate(CM_USART2, USART2_BAUDRATE, &f32Error);
        if ((-0.025F <= f32Error) && (f32Error <= 0.025F))
        {
            break;
        }
    }
 
    /* Register RX error IRQ handler && configure NVIC. */
    stcIrqSigninConfig.enIRQn = USART2_RX_ERR_IRQn;
    stcIrqSigninConfig.enIntSrc = USART2_RX_ERR_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART2_RxError_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* Register TX complete IRQ handler. */
    stcIrqSigninConfig.enIRQn = USART2_TX_CPLT_IRQn;
    stcIrqSigninConfig.enIntSrc = USART2_TX_CPLT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART2_TxComplete_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* Register RX timeout IRQ handler. */
    stcIrqSigninConfig.enIRQn = USART2_RX_TIMEOUT_IRQn;
    stcIrqSigninConfig.enIntSrc = USART2_RX_TIMEOUT_INT_SRC;
    stcIrqSigninConfig.pfnCallback = &USART2_RxTimeout_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSigninConfig);
    NVIC_ClearPendingIRQ(stcIrqSigninConfig.enIRQn);
    NVIC_SetPriority(stcIrqSigninConfig.enIRQn, DDL_IRQ_PRIO_05);
    NVIC_EnableIRQ(stcIrqSigninConfig.enIRQn);

    /* Enable TX && RX && RX interrupt function */
    USART_FuncCmd(USART2_UNIT, (USART_RX | USART_INT_RX | USART_RX_TIMEOUT | \
                               USART_INT_RX_TIMEOUT ), ENABLE);
}

/**
 * @brief 初始化 串口
 * @return int16_t 
 */
int16_t bsp_usart2_init(void) 
{
    imu_pack_quene_handle = xQueueCreate(8, sizeof(imu_pack_t));
    if (imu_pack_quene_handle == NULL) 
    {
        printf("uart2 Create queue error.\r\n");
        return -1;
    }
    hc_usart2_init();
}

/**
 * @brief 串口发送数据
 * @param  data              Param doc
 * @param  len               Param doc
 */
void usart2_dma_send(uint8_t *data, uint8_t len)
{
    memcpy(m_au8TxBuf, data, len);
    DMA_SetSrcAddr(UART2_TX_DMA_UNIT, UART2_TX_DMA_CH, (uint32_t)m_au8TxBuf);
    DMA_SetTransCount(UART2_TX_DMA_UNIT, UART2_TX_DMA_CH, len);
    (void)DMA_ChCmd(UART2_TX_DMA_UNIT, UART2_TX_DMA_CH, ENABLE);
    USART_FuncCmd(USART2_UNIT, USART_TX, ENABLE);
}
