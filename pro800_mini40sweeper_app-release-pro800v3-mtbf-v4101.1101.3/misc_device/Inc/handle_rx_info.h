#ifndef __HANDLE_RX_INFO_H_
#define __HANDLE_RX_INFO_H_

#include <stdint.h>
#include "main.h"

#define SOFTWARE_VER_MAIN 0
#define SOFTWARE_VER_MIN 2
#define SOFTWARE_VER_BUILD 1











#define UDP_SEVER_DBUG 0
#define UART1_DBUG 0
#define UART2_DBUG 0
#define UART3_DBUG 0
#define UART4_DBUG 0
#define UART5_DBUG 0
#define UART6_DBUG 0
#define UART7_DBUG 0
#define ADS1115_DBUG 0
#define RTC_DBUG 0
#define GD25Q32C_DBUG 0
#define IO_INPUT_DBUG 0
#define CAN1_DBUG 0
#define CAN2_DBUG 0
#define ADC_DBUG 0
#define ENCODER_DBUG 0

#define SYSTEM_INFO_DBUG 1


extern uint8_t gs_uart1RxChar;
extern uint8_t gs_uart5RxChar;
extern uint16_t g_adc3RawData[10][4];
extern uint16_t g_adc1RawData[5][2];
extern uint32_t g_encoderZero3;
extern uint32_t g_encoderZero2;
void UART1_IRQHandler_IDLE_DMA(void);
void UART2_IRQHandler_IDLE_DMA(void);
void UART3_IRQHandler_IDLE_DMA(void);
void UART4_IRQHandler_IDLE_DMA(void);
void UART5_IRQHandler_IDLE_DMA(void);
void UART6_IRQHandler_IDLE_DMA(void);
void UART7_IRQHandler_IDLE_DMA(void);

void HandleUart1Data(void);
void HandleUart2Data(void);
void HandleUart3Data(void);
void HandleUart4Data(void);
void HandleUart5Data(void);
void HandleUart6Data(void);
void HandleUart7Data(void);


void HandleADS1115Data(void);

void HandleRTCInfo(void);

void HandleGD25Q32C(void);

void HandleInput(void);
void HandleOutput(void);

void HandleCanOp(void);
void HandleADC(void);
void HandleDAC(void);
void HandlePWM(void);
void HandleEncoder(void);
void ShowSystemInfo(void);
#endif

