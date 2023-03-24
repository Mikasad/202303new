#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

#include "stm32f4xx_hal.h"
#include <stdio.h>

//#define Usart_Labview  1
#define LabView_Uart_TxBuffersize 2010
#define Rec_Len                 10      //?����?????��DD�쨺y?Y��?�䨮��??��3��?��
#define Rec_Num                 1      //�̣�??1|?��??�����?�䨮15??��DD�쨺y?Y
#define USART_REC_LEN  			   1000  	   //?����?�̣���?��y?Y�㨹��?�䨮?��������??����y
#define LABVIEWRXBUFFERSIZE 100
extern UART_HandleTypeDef husart_debug;
extern u8 datalens;
extern u8 UsartRxBuffer[LABVIEWRXBUFFERSIZE];
extern u8 UsartTxBuffer[LABVIEWRXBUFFERSIZE];
extern u8 UserCmd_Type;
extern u8 RefreshCmd_Type;
extern u8 HallStudyFlag1,HallStudyFlag2;
extern u8 Labview_uart_Flag;
extern uint8_t Motor_Judgment;
extern long Usart_ReadRegValue(u8 RxBuffer[],int i);
extern long Usart_Labview_SendRegValue(long TxDate);
extern void DualDrv_Parameter_Upload(void);
extern void DualDrv_Parameter_Download(void);	
uint8_t Motor_EN_OFF(_Bool on ,uint8_t motornum);
void Switch_Motor_Operation_Mode(uint8_t mode , uint8_t motornum);
void Usart_Labview_Analyze(void);
/*��labview��ӡ���ݲ鿴����*/
void Labview_uart(void);

#endif  /* __BSP_DEBUG_USART_H__ */
