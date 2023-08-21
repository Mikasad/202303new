

#ifndef ADS1115_H_
#define ADS1115_H_
#include "stm32f4xx_hal.h"

void ads1115_init(uint8_t ch);
void point_register(void);
uint16_t ads1115_read(void);
int get_ads1115_data(uint8_t ch, uint16_t* fp_data);

uint16_t get_adc0_value(void);
uint16_t get_adc1_value(void);


#endif 

