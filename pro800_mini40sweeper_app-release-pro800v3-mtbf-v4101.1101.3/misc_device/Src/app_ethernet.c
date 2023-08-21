/**
 *******************************************************************************
 * @file  lwip/lwip_http_server_socket_rtos/source/app_ethernet.c
 * @brief Ethernet DHCP and Connect status module.
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
#include "lwip/tcpip.h"
#include "hc32_ll.h"
#include "app_ethernet.h"
#include "tcp_server.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#ifdef USE_DHCP
#define DHCP_MAX_TRIES                  (4U)
#endif

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
#ifdef USE_DHCP
static __IO uint8_t u8DHCPState = DHCP_OFF;
#endif

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @brief  Notify network connect status of Ethernet.
 * @param  [in] netif                   Pointer to a struct netif structure
 * @retval None
 */
void EthernetIF_NotifyConnStatus(struct netif *netif)
{
    if (netif_is_up(netif)) {
#ifdef USE_DHCP
        u8DHCPState = DHCP_START;
#else
        /* Turn On LED BLUE to indicate ETH and LwIP init success*/
//        BSP_LED_On(LED_BLUE);
#endif /* USE_DHCP */
    } else {
#ifdef USE_DHCP
        u8DHCPState = DHCP_LINK_DOWN;
#endif  /* USE_DHCP */
        /* Turn On LED RED to indicate ETH and LwIP init error */
//        BSP_LED_On(LED_RED);
    }
}

/**
 * @brief  Notify link status change.
 * @param  [in] netif                   Pointer to a struct netif structure
 * @retval None
 */
void EthernetIF_NotifyLinkChange(struct netif *netif)
{
#ifndef USE_DHCP
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;
#endif

    if (netif_is_link_up(netif)) {
//        BSP_LED_Off(LED_RED);
//        BSP_LED_On(LED_BLUE);
#ifdef USE_DHCP
        /* Update DHCP state machine */
        u8DHCPState = DHCP_START;
#else
        IP_ADDR4(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
        IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
        IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
        netif_set_addr(netif, &ipaddr, &netmask, &gw);
#endif /* USE_DHCP */
        /* When the netif is fully configured this function must be called. */
        netif_set_up(netif);
    } else {
//        BSP_LED_Off(LED_BLUE);
//        BSP_LED_On(LED_RED);
#ifdef USE_DHCP
        u8DHCPState = DHCP_LINK_DOWN;
#endif /* USE_DHCP */
        /*  When the netif link is down this function must be called. */
        netif_set_down(netif);
    }
}

#ifdef USE_DHCP
/**
 * @brief  Lwip DHCP thread.
 * @param  [in] argument                Pointer that is passed to the thread function as start argument.
 * @retval None
 */
void LwIP_DhcpThread(void const *argument)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;
    struct dhcp *dhcp;
    struct netif *netif = (struct netif *)argument;

    for (;;) {
        switch (u8DHCPState) {
            case DHCP_START:
                ip_addr_set_zero_ip4(&netif->ip_addr);
                ip_addr_set_zero_ip4(&netif->netmask);
                ip_addr_set_zero_ip4(&netif->gw);
                u8DHCPState = DHCP_WAIT_ADDR;
                (void)dhcp_start(netif);
                break;
            case DHCP_WAIT_ADDR:
                if (0U != dhcp_supplied_address(netif)) {
                    u8DHCPState = DHCP_ADDR_ASSIGNED;
                    BSP_LED_On(LED_BLUE);
                } else {
                    dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);
                    /* DHCP timeout */
                    if (dhcp->tries > DHCP_MAX_TRIES) {
                        u8DHCPState = DHCP_TIMEOUT;
                        dhcp_stop(netif);
                        /* Static address used */
                        IP_ADDR4(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
                        IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
                        IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
                        netif_set_addr(netif, &ipaddr, &netmask, &gw);
                        BSP_LED_On(LED_BLUE);
                    }
                }
                break;
            case DHCP_LINK_DOWN:
                /* Stop DHCP */
                dhcp_stop(netif);
                u8DHCPState = DHCP_OFF;
                break;
            default:
                break;
        }
        osDelay(100);
    }
}
#endif






/* unlock/lock peripheral */
#define EXAMPLE_PERIPH_WE               (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
                                         LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
/* RMII_INTB */
#ifdef ETH_INTERFACE_RMII
#define ETH_RMII_INTB_PORT              (GPIO_PORT_B)
#define ETH_RMII_INTB_PIN               (GPIO_PIN_00)
#define ETH_RMII_INTB_EXTINT            (EXTINT_CH00)
#define ETH_RMII_INTB_INT_SRC           (INT_SRC_PORT_EIRQ0)
#define ETH_RMII_INTB_IRQn              (INT006_IRQn)
#endif

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
stc_eth_link_arg_t EthLinkArg;
#ifdef ETH_INTERFACE_RMII
osSemaphoreId EthLinkSem = NULL;
#endif

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static struct netif gnetif;



#ifdef ETH_INTERFACE_RMII
/**
 * @brief  ETH RMII interrupt callback function.
 * @param  None
 * @retval None
 */
static void ETH_LinkIrqCallback(void)
{
    if (SET == EXTINT_GetExtIntStatus(ETH_RMII_INTB_EXTINT)) {
        osSemaphoreRelease(EthLinkSem);
        EXTINT_ClearExtIntStatus(ETH_RMII_INTB_EXTINT);
    }
}

/**
 * @brief  ETH link interrupt configuration.
 * @param  None
 * @retval None
 */
static void ETH_LinkIntConfig(void)
{
    stc_extint_init_t stcExtIntInit;
    stc_irq_signin_config_t stcIrqSignConfig;
    stc_gpio_init_t stcGpioInit;

    /* Configure GPIO */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16ExtInt = PIN_EXTINT_ON;
    stcGpioInit.u16PullUp = PIN_PU_ON;
    (void)GPIO_Init(ETH_RMII_INTB_PORT, ETH_RMII_INTB_PIN, &stcGpioInit);
    /* Configure Extint */
    (void)EXTINT_StructInit(&stcExtIntInit);
    stcExtIntInit.u32Edge = EXTINT_TRIG_FALLING;
    (void)EXTINT_Init(ETH_RMII_INTB_EXTINT, &stcExtIntInit);
    /* IRQ sign-in */
    stcIrqSignConfig.enIntSrc    = ETH_RMII_INTB_INT_SRC;
    stcIrqSignConfig.enIRQn      = ETH_RMII_INTB_IRQn;
    stcIrqSignConfig.pfnCallback = &ETH_LinkIrqCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
}
#endif /* ETH_INTERFACE_RMII */



/**
 * @brief  Configurate the network interface
 * @param  None
 * @retval None
 */
static void Netif_Config(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

#ifdef USE_DHCP
    ip_addr_set_zero_ip4(&ipaddr);
    ip_addr_set_zero_ip4(&netmask);
    ip_addr_set_zero_ip4(&gw);
#else
    IP_ADDR4(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
    IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
    IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif /* USE_DHCP */
    /* Add the network interface */
    (void)netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
    /*  Registers the default network interface */
    netif_set_default(&gnetif);

    if (netif_is_link_up(&gnetif)) {
        /* When the netif is fully configured this function must be called */
        netif_set_up(&gnetif);
    } else {
        /* When the netif link is down this function must be called */
        netif_set_down(&gnetif);
    }
    /* Set the link callback function, this function is called on change of link status*/
//    netif_set_link_callback(&gnetif, EthernetIF_LinkCallback);

    EthLinkArg.netif = &gnetif;
#ifdef ETH_INTERFACE_RMII
    /* create a binary semaphore */
    osSemaphoreDef(LinkSem);
    EthLinkSem = osSemaphoreCreate(osSemaphore(LinkSem), 1);
    EthLinkArg.sem   = EthLinkSem;
    /* Configure link interrupt IO for ETH RMII */
//    ETH_LinkIntConfig();
#endif
    /* Create the Ethernet link thread */
//    osThreadDef(EthLink, ETH_LinkThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 5);
    osThreadDef(EthLink, ETH_LinkThread, 17, 0, configMINIMAL_STACK_SIZE * 5);
    osThreadCreate(osThread(EthLink), &EthLinkArg);
}

/**
 * @brief  Start Thread.
 * @param  [in] argument                Pointer that is passed to the thread function as start argument.
 * @retval None
 */
void StartThread(void const *argument)
{
    /* Create tcp_ip stack thread */
    tcpip_init(NULL, NULL);
    /* Configure the Network interface */
    Netif_Config();
	/* Tcp server init */
	tcp_server_init();
	tcp_debug_init();
    /* Notify user about the network interface config */
    EthernetIF_NotifyConnStatus(&gnetif);
#ifdef USE_DHCP
    /* Start DHCPClient */
    osThreadDef(DHCP, LwIP_DhcpThread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 5);
    osThreadCreate(osThread(DHCP), &gnetif);
#endif

    for (;;) {
        osDelay(500U);	
    }
}

/**
 * @brief  网络任务初始化
 * @param  None
 * @retval 
 */
int32_t ETH_TaskInit(void)
{
	DDL_DelayMS(100);
    /* Init thread */
    osThreadDef(Start, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 5);
    osThreadCreate(osThread(Start), NULL);
	return 0;
}



/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
