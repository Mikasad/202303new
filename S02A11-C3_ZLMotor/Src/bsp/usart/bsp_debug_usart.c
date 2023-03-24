#include "usart/bsp_debug_usart.h"
#include "Usart_Labview.h"
#include "main.h"
UART_HandleTypeDef husart_debug;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if(huart->Instance==DEBUG_USARTx)
    {
        DEBUG_USART_RCC_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = DEBUG_USARTx_Tx_GPIO_PIN|DEBUG_USARTx_Rx_GPIO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        hdma_usart3_rx.Instance = DMA1_Stream1;
        hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_NORMAL;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
        {
            Error_Handler();
        }
        __HAL_LINKDMA(huart,hdmarx,hdma_usart3_rx);
        hdma_usart3_tx.Instance = DMA1_Stream3;
        hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_tx.Init.Mode = DMA_NORMAL;
        hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
        {
            Error_Handler();
        }
        __HAL_LINKDMA(huart,hdmatx,hdma_usart3_tx);
    }
}
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

    if(huart->Instance==DEBUG_USARTx)
    {
        DEBUG_USART_RCC_CLK_DISABLE();
        HAL_GPIO_DeInit(DEBUG_USARTx_Tx_GPIO, DEBUG_USARTx_Tx_GPIO_PIN);
        HAL_GPIO_DeInit(DEBUG_USARTx_Rx_GPIO, DEBUG_USARTx_Rx_GPIO_PIN);
        HAL_DMA_DeInit(huart->hdmarx);
        HAL_DMA_DeInit(huart->hdmatx);
        HAL_NVIC_DisableIRQ(DEBUG_USART_IRQn);
    }
}
extern uint8_t aRxBufferUsart3 ;
#define LABVIEWRXBUFFERSIZE 100
extern u8 UsartRxBuffer[LABVIEWRXBUFFERSIZE];
void MX_DEBUG_USART_Init(void)
{
    DEBUG_USARTx_GPIO_ClK_ENABLE();

    husart_debug.Instance = DEBUG_USARTx;
    husart_debug.Init.BaudRate = DEBUG_USARTx_BAUDRATE;
    husart_debug.Init.WordLength = UART_WORDLENGTH_8B;
    husart_debug.Init.StopBits = UART_STOPBITS_1;
    husart_debug.Init.Parity = UART_PARITY_NONE;
    husart_debug.Init.Mode = UART_MODE_TX_RX;
    husart_debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    husart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&husart_debug);
    __HAL_UART_ENABLE_IT(&husart_debug,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&husart_debug,UsartRxBuffer,LABVIEWRXBUFFERSIZE);
}
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
}

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&husart_debug, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}
int fgetc(FILE * f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&husart_debug,&ch, 1, 0xffff);
    return ch;
}
