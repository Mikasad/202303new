#ifndef _HC_CRC_H
#define _HC_CRC_H

#include "hc32_ll.h"
#include "main.h"


void hc_crc32_init(void);
uint32_t hc_sw_crc32_calculate(uint32_t* buf, uint32_t len);
uint32_t hc_hw_crc32_calculate(uint32_t* buf, uint32_t len);
void hc_crc32_deinit(void);

#endif  
