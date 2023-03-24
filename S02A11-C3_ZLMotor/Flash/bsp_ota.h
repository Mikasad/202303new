#ifndef __BSP_OTA_H__
#define __BSP_OTA_H__

#include "stm32f4xx_hal.h"
#include <stdio.h>

typedef  void (*pFunction)(void);
uint8_t Jump_to_APP(uint32_t APP_addr);
void SoftReset(void);
	
#endif
