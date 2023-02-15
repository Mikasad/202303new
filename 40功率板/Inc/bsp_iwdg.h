#ifndef __IWDG_H__
#define __IWDG_H__

#include "stm32f4xx_hal.h"

/*ADC模拟看门狗阈值设置值*/
#define ADC1AnalogWDGHighThreshold  4095

extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef hadc1;
void MX_IWDG_Init(uint8_t prv ,uint16_t rlv);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);

#endif 
