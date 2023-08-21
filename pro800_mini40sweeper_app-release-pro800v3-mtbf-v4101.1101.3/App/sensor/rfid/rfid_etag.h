#ifndef _RFID_ETAG_H
#define _RFID_ETAG_H

//#include "stm32f4xx_hal.h"
#include <stdint.h>


#define RFID_TAG_MAX_NUM  16

typedef struct
{
    uint32_t id;
    uint32_t tag_num;
}RfidTag_t;


void HandleRfidData(const uint8_t fp_data[], uint16_t f_len );

void RfidTask(void const *pvParameters);

uint8_t GetRfidProtectFlag(void);
void ClearRfidProtectFlag(void);
uint8_t GetRfidHealthStatus(void);
void GetRfidData( RfidTag_t* fp_tags, uint16_t f_cpy_size );




#endif
