/*
 * @Author: Jack Yi
 * @Date: 2022-08-31 15:20:20
 * @LastEditors: Jack Yi
 * @LastEditTime: 2022-09-14 16:03:56
 * @FilePath: \m_line\HC32\hardware\hc_uart2.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Jack Yi, All Rights Reserved. 
 */

#ifndef _HC_UART2_H_
#define _HC_UART2_H_

#include "stdint.h"
#include "queue.h"

/*usart1 rx pa9;tx pa10;485 ctr PA8*/

//usart2 define pd5 pd6  ->imu 
#define UART2_RX_DMA_UNIT                     (CM_DMA1)
#define UART2_RX_DMA_CH                       (DMA_CH5)
#define UART2_RX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA1, ENABLE))
#define UART2_RX_DMA_TRIG_SEL                 (AOS_DMA1_5)
#define UART2_RX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART2_RI)
#define UART2_RX_DMA_RECONF_TRIG_SEL          (AOS_DMA_RC)
#define UART2_RX_DMA_RECONF_TRIG_EVT_SRC      (EVT_SRC_AOS_STRG)
#define UART2_RX_DMA_TC_INT                   (DMA_INT_TC_CH5)
#define UART2_RX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH5)
#define UART2_RX_DMA_TC_IRQn                  (INT005_IRQn)
#define UART2_RX_DMA_TC_INT_SRC               (INT_SRC_DMA1_TC5)

#define UART2_TX_DMA_UNIT                     (CM_DMA2)
#define UART2_TX_DMA_CH                       (DMA_CH3)
#define UART2_TX_DMA_FCG_ENABLE()             (FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE))
#define UART2_TX_DMA_TRIG_SEL                 (AOS_DMA2_3)
#define UART2_TX_DMA_TRIG_EVT_SRC             (EVT_SRC_USART2_TI)
#define UART2_TX_DMA_TC_INT                   (DMA_INT_TC_CH3)
#define UART2_TX_DMA_TC_FLAG                  (DMA_FLAG_TC_CH3)
#define UART2_TX_DMA_TC_IRQn                  (INT024_IRQn)
#define UART2_TX_DMA_TC_INT_SRC               (INT_SRC_DMA2_TC3)

/* Timer0 unit & channel definition */
#define UART2_TMR0_UNIT                       (CM_TMR0_1)
#define UART2_TMR0_CH                         (TMR0_CH_B)
#define UART2_TMR0_FCG_ENABLE()               (FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR0_1, ENABLE))

/* USART RX/TX pin definition */
#define USART2_RX_PORT                   (GPIO_PORT_D)   /* PH13: USART1_RX */
#define USART2_RX_PIN                    (GPIO_PIN_06)
#define USART2_RX_GPIO_FUNC              (GPIO_FUNC_20)

#define USART2_TX_PORT                   (GPIO_PORT_D)   /* PH15: USART1_TX */
#define USART2_TX_PIN                    (GPIO_PIN_05)
#define USART2_TX_GPIO_FUNC              (GPIO_FUNC_20)

/* USART unit definition */
#define USART2_UNIT                      (CM_USART2)
#define USART2_FCG_ENABLE()              (FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART2, ENABLE))

/* USART baudrate definition */
#define USART2_BAUDRATE                  (115200UL)

/* USART timeout bits definition */
#define USART2_TIMEOUT_BITS              (50U)

/* USART interrupt definition */
#define USART2_TX_CPLT_IRQn              (INT007_IRQn)
#define USART2_TX_CPLT_INT_SRC           (INT_SRC_USART2_TCI)

#define USART2_RX_ERR_IRQn               (INT025_IRQn)
#define USART2_RX_ERR_INT_SRC            (INT_SRC_USART2_EI)

#define USART2_RX_TIMEOUT_IRQn           (INT026_IRQn)
#define USART2_RX_TIMEOUT_INT_SRC        (INT_SRC_USART2_RTO)

/* Application frame length max definition */
#define USART2_FRAME_LEN_MAX               (256)

typedef struct
{
    uint8_t rx_data[USART2_FRAME_LEN_MAX];
    uint16_t len;
} imu_pack_t;

extern QueueHandle_t imu_pack_quene_handle;

int16_t bsp_usart2_init(void); 
void usart2_dma_send(uint8_t *data,uint8_t len);

#endif
