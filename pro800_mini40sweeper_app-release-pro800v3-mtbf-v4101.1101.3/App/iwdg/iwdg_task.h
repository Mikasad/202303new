#ifndef __IWDG_TASK_H
#define __IWDG_TASK_H


//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"

#define USE_HW_IWDG	1


void IWDGTask(void const *pvParameters);

void FeedDog( uint32_t bit );


#endif
