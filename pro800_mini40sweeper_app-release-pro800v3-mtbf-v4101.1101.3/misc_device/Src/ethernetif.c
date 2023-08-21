/**
 *******************************************************************************
 * @file  lwip/lwip_http_server_socket_rtos/source/ethernetif.c
 * @brief This file implements Ethernet network interface drivers for lwIP.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "ethernetif.h"


/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Define those to better describe your network interface. */
#define IFNAME0                         'h'
#define IFNAME1                         'd'

/* Reset time of PHY hardware */
#define PHY_HW_RST_DELAY                (0x80U)
/* Stack size of ETH receive thread  */
#define ETH_RX_THREAD_STACK_SIZE        (512U)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
// __IO uint32_t u32EthRxFlag = 0UL;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
void ethernetif_input(void const *argument);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
/* Global Ethernet handle*/
static stc_eth_handle_t EthHandle;
/* Semaphore to signal incoming packets */
static osSemaphoreId EthRxSem = NULL;

/* Ethernet Tx DMA Descriptor */
__ALIGN_BEGIN static stc_eth_dma_desc_t EthDmaTxDscrTab[ETH_TX_BUF_NUM];
/* Ethernet Rx DMA Descriptor */
__ALIGN_BEGIN static stc_eth_dma_desc_t EthDmaRxDscrTab[ETH_RX_BUF_NUM];
/* Ethernet Transmit Buffer */
__ALIGN_BEGIN static uint8_t EthTxBuff[ETH_TX_BUF_NUM][ETH_TX_BUF_SIZE];
/* Ethernet Receive Buffer */
__ALIGN_BEGIN static uint8_t EthRxBuff[ETH_RX_BUF_NUM][ETH_RX_BUF_SIZE];

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @brief  Ethernet Rx completed callback
 * @param  None
 * @retval None
 */
void ETH_RxCpltCallback(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if( RESET != ETH_DMA_GetStatus( ETH_DMA_FLAG_RIS ) )
    {
        xSemaphoreGiveFromISR(EthRxSem, &xHigherPriorityTaskWoken );
    }

    ETH_DMA_ClearStatus( ETH_DMA_FLAG_RIS );
    ETH_DMA_ClearStatus( ETH_DMA_FLAG_NIS );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void ETH_int_config(void)
{
    stc_irq_signin_config_t stcIrqSignConfig;

    /* ETH interrupt configure */
    stcIrqSignConfig.enIntSrc    = INT_SRC_ETH_GLB_INT;
    stcIrqSignConfig.enIRQn      = INT008_IRQn;
    stcIrqSignConfig.pfnCallback = &ETH_RxCpltCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
//    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_05);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
 }
	
/**
 * @brief  Initializes the Ethernet GPIO.
 * @param  None
 * @retval None
 */
static void Ethernet_GpioInit(void)
{
	//LL_PERIPH_WE(LL_PERIPH_GPIO); /* Register write enable for some required peripherals. */

    stc_irq_signin_config_t stcIrqSignConfig;

    /* ETH_RST */
	stc_gpio_init_t stcGpioInit;
	(void)GPIO_StructInit(&stcGpioInit);

	stcGpioInit.u16PinState = PIN_STAT_RST;
	stcGpioInit.u16PinDir   = PIN_DIR_OUT;
	stcGpioInit.u16PullUp   = PIN_PU_OFF;
	(void)GPIO_Init(GPIO_PORT_G, GPIO_PIN_13, &stcGpioInit);

	GPIO_ResetPins(GPIO_PORT_G,GPIO_PIN_13 );
//	DDL_DelayMS(ETH_PHY_RST_DELAY);
    vTaskDelay(ETH_PHY_RST_DELAY);
	GPIO_SetPins(GPIO_PORT_G, GPIO_PIN_13);
//	DDL_DelayMS(ETH_PHY_RST_DELAY);
    vTaskDelay(ETH_PHY_RST_DELAY);
	
    /* Configure MII/RMII selection IO for ETH */
#ifdef ETH_INTERFACE_RMII
    /* Ethernet RMII pins configuration */
    /*
        ETH_SMI_MDIO ----------------> PA2
        ETH_SMI_MDC -----------------> PC1
        ETH_RMII_TX_EN --------------> PG11//PB11
        ETH_RMII_TXD0 ---------------> PG13//PB12
        ETH_RMII_TXD1 ---------------> PG14//PB13
        ETH_RMII_REF_CLK ------------> PA1
        ETH_RMII_CRS_DV -------------> PA7
        ETH_RMII_RXD0 ---------------> PC4
        ETH_RMII_RXD1 ---------------> PC5
        ETH_RMII_RX_ER --------------> PI10
    */
    /* Configure PA1, PA2 and PA7 */
    GPIO_SetFunc(GPIO_PORT_A, (GPIO_PIN_01 | GPIO_PIN_02 | GPIO_PIN_07), GPIO_FUNC_11);
    /* Configure PC1, PC4 and PC5 */
    GPIO_SetFunc(GPIO_PORT_C, (GPIO_PIN_01 | GPIO_PIN_04 | GPIO_PIN_05), GPIO_FUNC_11);
    /* Configure PG11, PG13 and PG14 */
    GPIO_SetFunc(GPIO_PORT_B, (GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13), GPIO_FUNC_11);
    /* Configure PI10 */
//    GPIO_SetFunc(GPIO_PORT_I, GPIO_PIN_10, GPIO_FUNC_11);


#else
    /* Ethernet MII pins configuration */
    /*
        ETH_SMI_MDIO ----------------> PA2
        ETH_SMI_MDC -----------------> PC1
        ETH_MII_TX_CLK --------------> PB6
        ETH_MII_TX_EN ---------------> PG11
        ETH_MII_TXD0 ----------------> PG13
        ETH_MII_TXD1 ----------------> PG14
        ETH_MII_TXD2 ----------------> PB9
        ETH_MII_TXD3 ----------------> PB8
        ETH_MII_RX_CLK --------------> PA1
        ETH_MII_RX_DV ---------------> PA7
        ETH_MII_RXD0 ----------------> PC4
        ETH_MII_RXD1 ----------------> PC5
        ETH_MII_RXD2 ----------------> PB0
        ETH_MII_RXD3 ----------------> PB1
        ETH_MII_RX_ER ---------------> PI10
        ETH_MII_CRS -----------------> PH2
        ETH_MII_COL -----------------> PH3
    */
    /* Configure PA1, PA2 and PA7 */
    GPIO_SetFunc(GPIO_PORT_A, (GPIO_PIN_01 | GPIO_PIN_02 | GPIO_PIN_07), GPIO_FUNC_11);
    /* Configure PB0, PB1, PB6, PB8 and PB9 */
    GPIO_SetFunc(GPIO_PORT_B, (GPIO_PIN_00 | GPIO_PIN_01 | GPIO_PIN_06 | GPIO_PIN_08 | GPIO_PIN_09), GPIO_FUNC_11);
    /* Configure PC1, PC4 and PC5 */
    GPIO_SetFunc(GPIO_PORT_C, (GPIO_PIN_01 | GPIO_PIN_04 | GPIO_PIN_05), GPIO_FUNC_11);
    /* Configure PG11, PG13 and PG14 */
    GPIO_SetFunc(GPIO_PORT_G, (GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14), GPIO_FUNC_11);
    /* Configure PH2, PH3 */
    GPIO_SetFunc(GPIO_PORT_H, (GPIO_PIN_02 | GPIO_PIN_03), GPIO_FUNC_11);
    /* Configure PI10 */
    GPIO_SetFunc(GPIO_PORT_I, GPIO_PIN_10, GPIO_FUNC_11);
#endif
	//LL_PERIPH_WP(LL_PERIPH_GPIO);	/* Register write protected for some required peripherals. */

}

/**
 * @brief  In this function, the hardware should be initialized.
 * @param  netif                         The already initialized network interface structure for this ethernetif.
 * @retval None
 */
static void low_level_init(struct netif *netif)
{
    stc_eth_init_t stcEthInit;
    uint16_t u16RegVal;

    /* Enable ETH clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_ETHMAC, ENABLE);
    /* Init Ethernet GPIO */
    Ethernet_GpioInit();
    /* Reset ETHERNET */
    (void)ETH_DeInit();
    /* Configure structure initialization */
    (void)ETH_CommStructInit(&EthHandle.stcCommInit);
    (void)ETH_StructInit(&stcEthInit);

    /* Configure ethernet peripheral */
    if (LL_OK == ETH_Init(&EthHandle, &stcEthInit)) {
        /* Set netif link flag */
        netif->flags |= NETIF_FLAG_LINK_UP;
    }
    /* Initialize Tx Descriptors list: Chain Mode */
    (void)ETH_DMA_TxDescListInit(&EthHandle, EthDmaTxDscrTab, &EthTxBuff[0][0], ETH_TX_BUF_NUM);
    /* Initialize Rx Descriptors list: Chain Mode  */
    (void)ETH_DMA_RxDescListInit(&EthHandle, EthDmaRxDscrTab, &EthRxBuff[0][0], ETH_RX_BUF_NUM);

    /* set MAC hardware address length */
    netif->hwaddr_len = (u8_t)ETH_HWADDR_LEN;
    /* set MAC hardware address */
    netif->hwaddr[0] = (EthHandle.stcCommInit).au8MacAddr[0];
    netif->hwaddr[1] = (EthHandle.stcCommInit).au8MacAddr[1];
    netif->hwaddr[2] = (EthHandle.stcCommInit).au8MacAddr[2];
    netif->hwaddr[3] = (EthHandle.stcCommInit).au8MacAddr[3];
    netif->hwaddr[4] = (EthHandle.stcCommInit).au8MacAddr[4];
    netif->hwaddr[5] = (EthHandle.stcCommInit).au8MacAddr[5];
    /* maximum transfer unit */
    netif->mtu = 1500U;
    /* device capabilities */
    /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

    taskENTER_CRITICAL();
    /* create a binary semaphore */
    osSemaphoreDef(RxSem);
    EthRxSem = osSemaphoreCreate(osSemaphore(RxSem), 1);
    /* create the task that handles the ETH_MAC */
//    osThreadDef(EthIF, ethernetif_input, osPriorityRealtime, 0, ETH_RX_THREAD_STACK_SIZE);
    osThreadDef(EthIF, ethernetif_input, 17, 0, ETH_RX_THREAD_STACK_SIZE);
    osThreadCreate(osThread(EthIF), netif);
    taskEXIT_CRITICAL();
    /* Enable MAC and DMA transmission and reception */
    (void)ETH_Start();

    ETH_int_config();

    /* Configure PHY LED mode */
    u16RegVal = PHY_PAGE_ADDR_7;
    (void)ETH_PHY_WriteReg(&EthHandle, PHY_PSR, u16RegVal);
    (void)ETH_PHY_ReadReg(&EthHandle, PHY_P7_IWLFR, &u16RegVal);
    MODIFY_REG16(u16RegVal, PHY_LED_SELECT, PHY_LED_SELECT_10);
    (void)ETH_PHY_WriteReg(&EthHandle, PHY_P7_IWLFR, u16RegVal);
    u16RegVal = PHY_PAGE_ADDR_0;
    (void)ETH_PHY_WriteReg(&EthHandle, PHY_PSR, u16RegVal);
#ifdef ETH_INTERFACE_RMII
    /* Disable Power Saving Mode */
    (void)ETH_PHY_ReadReg(&EthHandle, PHY_PSMR, &u16RegVal);
    CLR_REG16_BIT(u16RegVal, PHY_EN_PWR_SAVE);
    (void)ETH_PHY_WriteReg(&EthHandle, PHY_PSMR, u16RegVal);
    /* Configure PHY to generate an interrupt when Eth Link state changes */
    u16RegVal = PHY_PAGE_ADDR_7;
    (void)ETH_PHY_WriteReg(&EthHandle, PHY_PSR, u16RegVal);
    /* Enable Interrupt on change of link status */
    (void)ETH_PHY_ReadReg(&EthHandle, PHY_P7_IWLFR, &u16RegVal);
    SET_REG16_BIT(u16RegVal, PHY_INT_LINK_CHANGE);
    (void)ETH_PHY_WriteReg(&EthHandle, PHY_P7_IWLFR, u16RegVal);
    u16RegVal = PHY_PAGE_ADDR_0;
    (void)ETH_PHY_WriteReg(&EthHandle, PHY_PSR, u16RegVal);
#endif
}

/**
 * @brief  This function should do the actual transmission of the packet.
 * @param  netif                        The lwip network interface structure for this ethernetif.
 * @param  p                            The MAC packet to send.
 * @return err_t:
 *           - ERR_OK: if the packet could be sent
 *           - an err_t value: if the packet couldn't be sent
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
    err_t errval;
    struct pbuf *q;
    uint8_t *txBuffer;
    __IO stc_eth_dma_desc_t *DmaTxDesc;
    uint32_t byteCnt;
    uint32_t frameLength = 0UL;
    uint32_t bufferOffset;
    uint32_t payloadOffset;

    DmaTxDesc = EthHandle.stcTxDesc;
    txBuffer = (uint8_t *)((EthHandle.stcTxDesc)->u32Buf1Addr);
    bufferOffset = 0UL;
    /* Copy frame from pbufs to driver buffers */
    for (q = p; q != NULL; q = q->next) {
        /* If this buffer isn't available, goto error */
        if (0UL != (DmaTxDesc->u32ControlStatus & ETH_DMA_TXDESC_OWN)) {
            errval = (err_t)ERR_USE;
            goto error;
        }

        /* Get bytes in current lwIP buffer */
        byteCnt = q->len;
        payloadOffset = 0UL;
        /* Check if the length of data to copy is bigger than Tx buffer size */
        while ((byteCnt + bufferOffset) > ETH_TX_BUF_SIZE) {
            /* Copy data to Tx buffer*/
            (void)memcpy((uint8_t *) & (txBuffer[bufferOffset]), (uint8_t *) & (((uint8_t *)q->payload)[payloadOffset]), (ETH_TX_BUF_SIZE - bufferOffset));
            /* Point to next descriptor */
            DmaTxDesc = (stc_eth_dma_desc_t *)(DmaTxDesc->u32Buf2NextDescAddr);
            /* Check if the buffer is available */
            if (0UL != (DmaTxDesc->u32ControlStatus & ETH_DMA_TXDESC_OWN)) {
                errval = (err_t)ERR_USE;
                goto error;
            }

            txBuffer = (uint8_t *)(DmaTxDesc->u32Buf1Addr);
            byteCnt = byteCnt - (ETH_TX_BUF_SIZE - bufferOffset);
            payloadOffset = payloadOffset + (ETH_TX_BUF_SIZE - bufferOffset);
            frameLength = frameLength + (ETH_TX_BUF_SIZE - bufferOffset);
            bufferOffset = 0UL;
        }
        /* Copy the remaining bytes */
        (void)memcpy((uint8_t *) & (txBuffer[bufferOffset]), (uint8_t *) & (((uint8_t *)q->payload)[payloadOffset]), byteCnt);
        bufferOffset = bufferOffset + byteCnt;
        frameLength = frameLength + byteCnt;
    }
    /* Prepare transmit descriptors to give to DMA */
    (void)ETH_DMA_SetTransFrame(&EthHandle, frameLength);
    errval = (err_t)ERR_OK;

error:
    /* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
    if (RESET != ETH_DMA_GetStatus(ETH_DMA_FLAG_UNS)) {
        /* Clear DMA UNS flag */
        ETH_DMA_ClearStatus(ETH_DMA_FLAG_UNS);
        /* Resume DMA transmission */
        WRITE_REG32(CM_ETH->DMA_TXPOLLR, 0UL);
    }

    return errval;
}

/**
 * @brief  Should allocate a pbuf and transfer the bytes of the incoming packet from the interface into the pbuf.
 * @param  netif                        The network interface structure for this ethernetif.
 * @retval A pbuf filled with the received packet (including MAC header) or NULL on memory error.
 */
static struct pbuf *low_level_input(struct netif *netif)
{
    struct pbuf *p = NULL;
    struct pbuf *q;
    uint32_t len;
    uint8_t *rxBuffer;
    __IO stc_eth_dma_desc_t *DmaRxDesc;
    uint32_t byteCnt;
    uint32_t bufferOffset;
    uint32_t payloadOffset;
    uint32_t i;

    /* Get received frame */
    if (LL_OK != ETH_DMA_GetReceiveFrame_Int(&EthHandle)) {
        return NULL;
    }

    /* Obtain the size of the packet */
    len = (EthHandle.stcRxFrame).u32Len;
    rxBuffer = (uint8_t *)(EthHandle.stcRxFrame).u32Buf;
    if (len > 0UL) {
        /* Allocate a pbuf chain of pbufs from the Lwip buffer pool */
        p = pbuf_alloc(PBUF_RAW, (uint16_t)len, PBUF_POOL);
    }
    if (p != NULL) {
        DmaRxDesc = (EthHandle.stcRxFrame).pstcFSDesc;
        bufferOffset = 0UL;
        for (q = p; q != NULL; q = q->next) {
            byteCnt = q->len;
            payloadOffset = 0UL;
            /* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size */
            while ((byteCnt + bufferOffset) > ETH_RX_BUF_SIZE) {
                /* Copy data to pbuf */
                (void)memcpy((uint8_t *) & (((uint8_t *)q->payload)[payloadOffset]), (uint8_t *) & (rxBuffer[bufferOffset]), (ETH_RX_BUF_SIZE - bufferOffset));
                /* Point to next descriptor */
                DmaRxDesc = (stc_eth_dma_desc_t *)(DmaRxDesc->u32Buf2NextDescAddr);
                rxBuffer = (uint8_t *)(DmaRxDesc->u32Buf1Addr);
                byteCnt = byteCnt - (ETH_RX_BUF_SIZE - bufferOffset);
                payloadOffset = payloadOffset + (ETH_RX_BUF_SIZE - bufferOffset);
                bufferOffset = 0UL;
            }
            /* Copy remaining data in pbuf */
            (void)memcpy((uint8_t *) & (((uint8_t *)q->payload)[payloadOffset]), (uint8_t *) & (rxBuffer[bufferOffset]), byteCnt);
            bufferOffset = bufferOffset + byteCnt;
        }
    }
    /* Release descriptors to DMA */
    DmaRxDesc = (EthHandle.stcRxFrame).pstcFSDesc;
    for (i = 0UL; i < (EthHandle.stcRxFrame).u32SegCount; i++) {
        DmaRxDesc->u32ControlStatus |= ETH_DMA_RXDESC_OWN;
        DmaRxDesc = (stc_eth_dma_desc_t *)(DmaRxDesc->u32Buf2NextDescAddr);
    }
    /* Clear Segment_Count */
    (EthHandle.stcRxFrame).u32SegCount = 0UL;

    /* When Rx Buffer unavailable flag is set, clear it and resume reception */
    if (RESET != ETH_DMA_GetStatus(ETH_DMA_FLAG_RUS)) {
        /* Clear DMA RUS flag */
        ETH_DMA_ClearStatus(ETH_DMA_FLAG_RUS);
        /* Resume DMA reception */
        WRITE_REG32(CM_ETH->DMA_RXPOLLR, 0UL);
    }

    return p;
}

/**
 * @brief  This function should be called when a packet is ready to be read from the interface.
 * @param  argument                     Pointer that is passed to the thread function as start argument.
 * @retval None
 */
uint32_t rx_data_len = 0;
uint8_t buff[128];
uint8_t out_flag =0;
void ethernetif_input(void const *argument)
{
    struct pbuf *p;
    struct netif *netif = (struct netif *)argument;

    for (;;) {
        if (osSemaphoreWait(EthRxSem, osWaitForever) == osOK) {
            do {
                p = low_level_input(netif);
                if (p != NULL) {
					if(p->len<128)
					{
						memcpy(buff, p->payload, p->len);
					}
					else
					{
						memcpy(buff, p->payload, 128);
					}
					rx_data_len = p->len ;
					if(out_flag)
						low_level_output(netif,p);
					ip4_debug_print(p);
                    if (netif->input(p, netif) != ERR_OK) {
                        LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                        pbuf_free(p);
                    }
                }
            } while (p != NULL);
        }
    }
}

/**
 * @brief  Should be called at the beginning of the program to set up the network interface.
 * @param  netif                        The network interface structure for this ethernetif.
 * @return err_t:
 *           - ERR_OK: if the IF is initialized
 *           - ERR_MEM: if private data couldn't be allocated any other err_t on error.
 */
err_t ethernetif_init(struct netif *netif)
{
#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;
    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...) */
    netif->output = &etharp_output;
    netif->linkoutput = &low_level_output;
    /* initialize the hardware */
    low_level_init(netif);

    return (err_t)ERR_OK;
}

#ifndef ETH_INTERFACE_RMII
/**
 * @brief  Check the netif link status.
 * @param  netif                        The network interface.
 * @retval None
 */
static void EthernetIF_CheckLink(struct netif *netif)
{
    uint16_t u16RegVal = 0U;
    static uint8_t u8PreStatus = 0U;

    /* Read PHY_BSR */
    (void)ETH_PHY_ReadReg(&EthHandle, PHY_BSR, &u16RegVal);
    /* Check whether the link is up or down*/
    if ((0x0000U != u16RegVal) && (0xFFFFU != u16RegVal)) {
        if ((0U != (u16RegVal & PHY_LINK_STATUS)) && (0U == u8PreStatus)) {
            netif_set_link_up(netif);
            u8PreStatus = 1U;
        }
        if ((0U == (u16RegVal & PHY_LINK_STATUS)) && (1U == u8PreStatus)) {
            netif_set_link_down(netif);
            u8PreStatus = 0U;
        }
    }
}
#endif

#ifdef ETH_INTERFACE_RMII
/**
 * @brief  Update the netif link status.
 * @param  netif                        The network interface.
 * @retval None
 */
static void EthernetIF_UpdateLink(struct netif *netif)
{
    uint16_t u16RegVal;
    uint16_t u16Page;

    /* Switch page */
    (void)ETH_PHY_ReadReg(&EthHandle, PHY_PSR, &u16Page);
    if (u16Page != PHY_PAGE_ADDR_0) {
        u16RegVal = PHY_PAGE_ADDR_0;
        (void)ETH_PHY_WriteReg(&EthHandle, PHY_PSR, u16RegVal);
    }
    /* Read PHY_IISDR */
    (void)ETH_PHY_ReadReg(&EthHandle, PHY_IISDR, &u16RegVal);
    /* Check whether the link interrupt has occurred or not */
    if (0U != (u16RegVal & PHY_FLAG_LINK_STATUS_CHANGE)) {
        /* Read PHY_BSR */
        (void)ETH_PHY_ReadReg(&EthHandle, PHY_BSR, &u16RegVal);
        if ((0x0000U != u16RegVal) && (0xFFFFU != u16RegVal)) {
            if (!netif_is_link_up(netif)) {
                /* Wait until the auto-negotiation will be completed */
                SysTick_Delay(2U);
                (void)ETH_PHY_ReadReg(&EthHandle, PHY_BSR, &u16RegVal);
            }
            /* Check whether the link is up or down*/
            if (0U != (u16RegVal & PHY_LINK_STATUS)) {
                netif_set_link_up(netif);
            } else {
                netif_set_link_down(netif);
            }
        }
    }
    /* Restore page */
    if (u16Page != PHY_PAGE_ADDR_0) {
        (void)ETH_PHY_WriteReg(&EthHandle, PHY_PSR, u16Page);
    }
}
#endif

/**
 * @brief  Link callback function
 * @note   This function is called on change of link status to update low level driver configuration.
 * @param  netif                        The network interface
 * @retval None
 */
void EthernetIF_LinkCallback(struct netif *netif)
{
    uint16_t u16RegVal = 0U;
    __IO uint32_t u32TickStart = 0UL;
    __IO int32_t i32negoResult = LL_ERR;

    if (netif_is_link_up(netif)) {
        /* Restart the auto-negotiation */
        if (ETH_AUTO_NEGO_DISABLE != (EthHandle.stcCommInit).u16AutoNego) {
            /* Enable Auto-Negotiation */
            (void)ETH_PHY_ReadReg(&EthHandle, PHY_BCR, &u16RegVal);
            u16RegVal |= PHY_AUTONEGOTIATION;
            (void)ETH_PHY_WriteReg(&EthHandle, PHY_BCR, u16RegVal);

            /* Wait until the auto-negotiation will be completed */
            u32TickStart = SysTick_GetTick();
            do {
                (void)ETH_PHY_ReadReg(&EthHandle, PHY_BSR, &u16RegVal);
                if (PHY_AUTONEGO_COMPLETE == (u16RegVal & PHY_AUTONEGO_COMPLETE)) {
                    break;
                }
                /* Check for the Timeout (3s) */
            } while ((SysTick_GetTick() - u32TickStart) <= 3000U);
            if (PHY_AUTONEGO_COMPLETE == (u16RegVal & PHY_AUTONEGO_COMPLETE)) {
                i32negoResult = LL_OK;
                /* Configure ETH duplex mode according to the result of automatic negotiation */
                if (0U != (u16RegVal & (PHY_100BASE_TX_FD | PHY_10BASE_T_FD))) {
                    (EthHandle.stcCommInit).u32DuplexMode = ETH_MAC_DUPLEX_MD_FULL;
                } else {
                    (EthHandle.stcCommInit).u32DuplexMode = ETH_MAC_DUPLEX_MD_HALF;
                }
                /* Configure ETH speed according to the result of automatic negotiation */
                if (0U != (u16RegVal & (PHY_100BASE_TX_FD | PHY_100BASE_TX_HD))) {
                    (EthHandle.stcCommInit).u32Speed = ETH_MAC_SPEED_100M;
                } else {
                    (EthHandle.stcCommInit).u32Speed = ETH_MAC_SPEED_10M;
                }
            }
        }

        /* AutoNegotiation disable or failed*/
        if (LL_ERR == i32negoResult) {
            (void)ETH_PHY_ReadReg(&EthHandle, PHY_BCR, &u16RegVal);
            CLR_REG16_BIT(u16RegVal, PHY_FULLDUPLEX_100M);
            /* Set MAC Speed and Duplex Mode to PHY */
            (void)ETH_PHY_WriteReg(&EthHandle, PHY_BCR,
                                   ((uint16_t)((EthHandle.stcCommInit).u32DuplexMode >> 3U) |
                                    (uint16_t)((EthHandle.stcCommInit).u32Speed >> 1U) | u16RegVal));
        }
        /* ETH MAC Re-Configuration */
        ETH_MAC_SetDuplexSpeed((EthHandle.stcCommInit).u32DuplexMode, (EthHandle.stcCommInit).u32Speed);
        /* Restart MAC interface */
        (void)ETH_Start();
    } else {
        /* Stop MAC interface */
        (void)ETH_Stop();
    }
    /* Notify link status change */
    EthernetIF_NotifyLinkChange(netif);
}


/**
 * @brief  Lwip link thread.
 * @param  [in] argument                Pointer that is passed to the thread function as start argument.
 * @retval None
 */
void ETH_LinkThread(void const *argument)
{
    stc_eth_link_arg_t *LinkArg = (stc_eth_link_arg_t *)argument;

    for (;;) {
#ifdef ETH_INTERFACE_RMII
        if (osSemaphoreWait(LinkArg->sem, osWaitForever) == osOK) {
            EthernetIF_UpdateLink(LinkArg->netif);
        }
#else
        EthernetIF_CheckLink(LinkArg->netif);
        osDelay(LINK_TIMER_INTERVAL);
#endif /* ETH_INTERFACE_RMII */
    }
}


/**
 * @brief  Notify link status change.
 * @param  netif                        The network interface
 * @retval None
 */
__WEAKDEF void EthernetIF_NotifyLinkChange(struct netif *netif)
{
    /* This is function could be implemented in user file when the callback is needed */
}

uint32_t soft_reset_phy(void)
{
	return	ETH_PHY_WriteReg(&EthHandle, PHY_BCR, PHY_SOFT_RESET);
}

/**
 * @brief  Returns the current time in milliseconds when LWIP_TIMERS == 1 and NO_SYS == 1
 * @param  None
 * @retval Current Time value
 */
u32_t sys_now(void)
{
    return SysTick_GetTick();
}

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
