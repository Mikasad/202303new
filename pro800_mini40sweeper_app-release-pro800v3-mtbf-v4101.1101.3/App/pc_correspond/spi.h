#ifndef __SPI_H
#define __SPI_H

#include "stm32f4xx.h"

void WIZ_SPI_Init(void);
void WIZ_CS(uint8_t val);
uint8_t SPI2_SendByte(uint8_t byte);
#endif

