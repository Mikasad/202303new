#ifndef __INTERFACE_CAN2_H__
#define __INTERFACE_CAN2_H__

#include "hc32_ll.h"
#include "objdict.h"

extern QueueHandle_t bms_can2_rx_quene_handle;

int16_t bsp_can2_init(void);
int16_t bsp_can2_write_data(void);
int16_t bsp_can2_read_data(void);


#endif /* __INTERFACE_CAN2_H__ */
