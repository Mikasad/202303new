#ifndef __MUTEX_CAN_BUF_H_
#define __MUTEX_CAN_BUF_H_

//#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "hc_can.h"


#define MUTEX_CAN_TX_BUF_SIZE 20//缓冲区数据域的大小

typedef struct
{
  stc_can_tx_frame_t head;
  uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0 to 0xFF. */
} CanTxMsg_t;

typedef struct{
    uint16_t len;
    uint16_t posIn;
    uint16_t posOut;
    CanTxMsg_t canTxMsg[MUTEX_CAN_TX_BUF_SIZE];
	SemaphoreHandle_t mutex;//互斥量，方便同步，使用之前要进行初始化创建
}mutexCanTxBuf_t;

int InitMutexCanTxBuf(mutexCanTxBuf_t *fp_mutexCanTxBuf);
int WriteMutexCanTxBuf(mutexCanTxBuf_t *fp_mutexCanTxBuf,CanTxMsg_t f_canTxMsg);
int ReadMutexCanTxBuf(mutexCanTxBuf_t *fp_mutexCanTxBuf,CanTxMsg_t *fp_canTxMsg);

#endif
