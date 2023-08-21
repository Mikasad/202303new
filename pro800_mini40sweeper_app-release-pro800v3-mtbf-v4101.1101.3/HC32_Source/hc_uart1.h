/*
 * @Author: Jack Yi
 * @Date: 2022-08-31 15:20:20
 * @LastEditors: Jack Yi
 * @LastEditTime: 2022-09-14 16:03:13
 * @FilePath: \m_line\HC32\hardware\hc_uart1.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Jack Yi, All Rights Reserved. 
 */

#ifndef _HC_UART1_H_
#define _HC_UART1_H_
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hc32_ll_tmr0.h"
#include "hc32_ll_usart.h"
#include "queue.h"

/*usart1 rx pa9;tx pa10;485 ctr PA8*/
#define RX_DMA_UNIT                     (CM_DMA1)
#define RX_DMA_CH                       (DMA_CH0)
#define RX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define RX_DMA_TRIG_SEL                 (AOS_DMA1_0)
#define RX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART1_RI)
#define RX_DMA_RECONF_TRIG_SEL          (AOS_DMA_RC)
#define RX_DMA_RECONF_TRIG_EVT_SRC      (EVT_SRC_AOS_STRG)
#define RX_DMA_TC_INT                   (DMA_INT_TC_CH0)
#define RX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH0)
#define RX_DMA_TC_IRQn                  (INT000_IRQn)
#define RX_DMA_TC_INT_SRC               (INT_SRC_DMA1_TC0)

#define TX_DMA_UNIT                     (CM_DMA2)
#define TX_DMA_CH                       (DMA_CH1)
#define TX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define TX_DMA_TRIG_SEL                 (AOS_DMA2_1)
#define TX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART1_TI)
#define TX_DMA_TC_INT                   (DMA_INT_TC_CH1)
#define TX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH1)
#define TX_DMA_TC_IRQn                  (INT001_IRQn)
#define TX_DMA_TC_INT_SRC               (INT_SRC_DMA2_TC1)

/* Timer0 unit & channel definition */
#define TMR0_UNIT                       (CM_TMR0_1)
#define TMR0_CH                         (TMR0_CH_A)
#define TMR0_FCG_ENABLE()               (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE))

/* USART RX/TX pin definition */
#define USART_RX_PORT                   (GPIO_PORT_A)   /* PH13: USART1_RX */
#define USART_RX_PIN                    (GPIO_PIN_09)
#define USART_RX_GPIO_FUNC              (GPIO_FUNC_20)

#define USART_TX_PORT                   (GPIO_PORT_A)   /* PH15: USART1_TX */
#define USART_TX_PIN                    (GPIO_PIN_10)
#define USART_TX_GPIO_FUNC              (GPIO_FUNC_20)

/* USART unit definition */
#define USART_UNIT                      (CM_USART1)
#define USART_FCG_ENABLE()              (FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART1, ENABLE))

/* USART baudrate definition */
#define USART_BAUDRATE                  (19200UL)

/* USART timeout bits definition */
#define USART_TIMEOUT_BITS              (50U)

/* USART interrupt definition */
#define USART_TX_CPLT_IRQn              (INT002_IRQn)
#define USART_TX_CPLT_INT_SRC           (INT_SRC_USART1_TCI)

#define USART_RX_ERR_IRQn               (INT003_IRQn)
#define USART_RX_ERR_INT_SRC            (INT_SRC_USART1_EI)

#define USART_RX_TIMEOUT_IRQn           (INT004_IRQn)
#define USART_RX_TIMEOUT_INT_SRC        (INT_SRC_USART1_RTO)

#define USART_RX_FULL_IRQn              (INT027_IRQn)
#define USART_RX_FULL_INT_SRC           (INT_SRC_USART1_RI)

/* Application frame length max definition */
#define APP_FRAME_LEN_MAX               (256U)

typedef struct
{
    uint8_t rx_data[128];
    uint16_t len;
} bms_pack_t;

extern QueueHandle_t bms_usart1_quene_handle;

int16_t bsp_usart1_init(void); 
void usart1_485_send(uint8_t *data, uint8_t len);

#endif

