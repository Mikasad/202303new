#ifndef __DECODER_TASK_H_
#define __DECODER_TASK_H_


//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"

extern TaskHandle_t taskHandleDecoderCnt;
void DecoderCntTask(void *pvParameters);

#endif
