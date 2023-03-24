#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

#include "stm32f4xx_hal.h"
#include <stdio.h>

#define DEBUG_USARTx                                 USART3
#define DEBUG_USARTx_BAUDRATE                        115200
#define DEBUG_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART3_CLK_ENABLE()
#define DEBUG_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART3_CLK_DISABLE()

#define DEBUG_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define DEBUG_USARTx_Tx_GPIO_PIN                     GPIO_PIN_10
#define DEBUG_USARTx_Tx_GPIO                         GPIOB
#define DEBUG_USARTx_Rx_GPIO_PIN                     GPIO_PIN_11
#define DEBUG_USARTx_Rx_GPIO                         GPIOB
#define DEBUG_USART_IRQn                             USART3_IRQn

extern UART_HandleTypeDef husart_debug;


void MX_DEBUG_USART_Init(void);
void MX_DMA_Init(void);

#endif 
