#ifndef __USART1_H__
#define __USART1_H__
#include "stm32f4xx_hal.h"

#define USARTx_BAUDRATE                        115200
#define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART1_CLK_ENABLE()
#define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART1_CLK_DISABLE()
#define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_Tx_GPIO_PIN                     GPIO_PIN_9
#define USARTx_Tx_GPIO                         GPIOA
#define USARTx_Rx_GPIO_PIN                     GPIO_PIN_10   
#define USARTx_Rx_GPIO                         GPIOA
#define USARTx_AFx                             GPIO_AF7_USART1
#define USARTx_IRQHANDLER                      USART1_IRQHandler
#define USARTx_IRQn                            USART1_IRQn

#define USART3_RX_TX_FRAMESIZE 11

#define TRUE  0x1
#define FALSE 0x0
//Machine define
#define WAIT_DLE 			0x00
#define WAIT_STX 			0x01
#define WAIT_OPCODE 	0x02
#define ESCAPE_OPCODE 0x03
#define WAIT_LEN 			0x04
#define ESCAPE_LEN 		0x05
#define WAIT_DATA 		0x06
#define ESCAPE_DATA 	0x07
#define WAIT_CRC 			0x08
#define ESCAPE_CRC 		0x09
#define ENDMARK       0x10

#define Maxon_DLE  0x5A
#define Maxon_STX  0x02
#define Eed_MARK   0xef

#define CMD_READ_OBJ  0x60
#define CMD_WRITE_OBJ 0x68

#define SCIMONITOR_EN						0x1
#define SCIMONITOR_DIS					0
#define START_RX								0x00
#define COMPLETE_RX             0x11
/*串口接收错误类型*/
#define USART_RECEIVE_TIMEOUT 0xFF
#define USART_CRC_CHECK_ERROR 0xEE

/*RX_STACK_DEFINITION*/
#define RXBUFFER_STACK_DEPTH    3
#define REC_LEN_MAX                20      //定义每个有效数据最大字节长度
//Stack monitor state
#define STACK_START     0x00
#define STACK_CHECK     0x01
#define STACK_HANDLE	  0x02
#define STACK_FULL 	0x01
#define STACK_NULL 	0x00
#define STACK_CONDITION 0x00
#define STACK_PRIORITY  0x01
#define STACK_ADDR			0x02
#define STACK_LENTH			0x03
#define MACHINE_BUSY  0x01
#define MACHINE_FREE  0x00
//STACK_TREE_TYPE
#define TYPE_PRE_BUFFER		0x00
#define TYPE_STACK        0x01

typedef struct
{
    long Sci_BaudRate[3];		//波特率3个挡位[9600 19200 115200]
    long Sci_Resp_Cmd;			//本机回复CMD寄存器
    u16 Sci_RsCheckDate;     //校验数据
    u16 Sci_TsCheckDate;
    long Sci_ErrClr_En;
    short Para_addr;
    long Sci_state;
    unsigned char Sci_SxBuffer[USART3_RX_TX_FRAMESIZE];
    unsigned char Sci_RxBuffer[USART3_RX_TX_FRAMESIZE];
	  unsigned char Sci_RxBufferDecode[USART3_RX_TX_FRAMESIZE];
    unsigned int datalen;
    long Obj_data;
    unsigned char rx_state;
    unsigned int crc_len;
    u8 receivecount;
    u8 Sci_Monitor_En;
    u8 Monitor_state;
    u32 Sci_Timing;

    u16 Sci_handelStep;
    u8 Sci_HandleStatus;
} SCICOM;
typedef struct
{
	unsigned char RxMail_Prep_buf[REC_LEN_MAX];
	unsigned char Rx_Stack_Buffer[RXBUFFER_STACK_DEPTH][REC_LEN_MAX];
	unsigned int Stack_Pointer[RXBUFFER_STACK_DEPTH][4];//Depth:3  {Null/Full,Priority,Stack_addr,Stack_len}
	unsigned int Stack_status;
	unsigned int rxcnt;
	unsigned int checkCnt;
	unsigned int frameSize;
	unsigned int State_monitor;
	unsigned int CurrStack_addr;
}RX_EMAIL;

extern uint8_t aRxBuffer;
extern uint8_t aRxBufferUsart3;
extern RX_EMAIL USART3_EMAIL;
extern SCICOM RsCOM;
extern UART_HandleTypeDef husartx;

void Maxon_Communicate_clr(void);
void MX_USARTx_Init(void);
u8 Maxon_Sci_COM_Send(unsigned char *buffer,u16 len);
void Sci_Monitor(void);
void Usart3_Error_Response(u8 errortype);
void Usart3_Data_T0_Canopen(u16 stdid,u16 extid,u16 ide,u16 rtr,u16 dlc );
void Sci_Stack_Monitor(unsigned int *process);
void Pop_Down_stack(unsigned int len,unsigned char *buff_server,unsigned char *buff_client);
void USART3_STACK_CLR(char treeType,char stack_addr);
#endif 

