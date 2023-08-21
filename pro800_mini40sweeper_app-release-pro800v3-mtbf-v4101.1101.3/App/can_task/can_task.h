#ifndef __CAN_TASK_H_
#define __CAN_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"
#include "mutex_can_buf.h"
#include "main.h"

typedef struct CanRxMsg_s
{
    stc_can_rx_frame_t head;
    uint8_t Data[8];
} CanRxMsg_t;

void ClearCan1TxBuf(void);
int WriteCan1TxBuf(CanTxMsg_t f_canTxMsg);
int ReadCan1TxBuf(CanTxMsg_t *fp_canTxMsg);
uint16_t GetLenCan1TxBuf(void);

void ClearCan2TxBuf(void);
int WriteCan2TxBuf(CanTxMsg_t f_canTxMsg);
int ReadCan2TxBuf(CanTxMsg_t *fp_canTxMsg);
uint16_t GetLenCan2TxBuf(void);
int CanReadSDO(uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void *f_data);
int CanWriteSDO(uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void *f_data);
int Can1Send(uint32_t f_id, uint8_t f_value[], uint8_t f_len);
int Can2SendExt(uint32_t f_ext_id, uint8_t f_value[], uint8_t f_len);

void CanTask(void const *pvParameters);
int InitSoftwareCanTask(void);
int InitHardwareCanTask(void);

extern mutexCanTxBuf_t can1TxBuf;
extern mutexCanTxBuf_t can2TxBuf;

#endif
