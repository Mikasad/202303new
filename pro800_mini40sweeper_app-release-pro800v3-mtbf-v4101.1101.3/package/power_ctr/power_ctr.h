#ifndef _POWER_CTR_H
#define _POWER_CTR_H
#include "main.h"
#include "hc_gpio.h"
#include "stdio.h"

#define CORE_SHUTDOWN_DELAY_MS    30000

//继电器控制
#define RELAY1_ON     HAL_GPIO_WritePin(ON_RELAY1_GPIO_Port, ON_RELAY1_Pin, GPIO_PIN_SET)
#define RELAY1_OFF    HAL_GPIO_WritePin(ON_RELAY1_GPIO_Port, ON_RELAY1_Pin, GPIO_PIN_RESET)
#define RELAY2_ON     HAL_GPIO_WritePin(ON_RELAY2_GPIO_Port, ON_RELAY2_Pin, GPIO_PIN_SET)
#define RELAY2_OFF    HAL_GPIO_WritePin(ON_RELAY2_GPIO_Port, ON_RELAY2_Pin, GPIO_PIN_RESET)
#define RELAY3_ON     HAL_GPIO_WritePin(ON_RELAY3_GPIO_Port, ON_RELAY3_Pin, GPIO_PIN_SET)
#define RELAY3_OFF    HAL_GPIO_WritePin(ON_RELAY3_GPIO_Port, ON_RELAY3_Pin, GPIO_PIN_RESET)
#define RELAY4_ON     HAL_GPIO_WritePin(ON_RELAY4_GPIO_Port, ON_RELAY4_Pin, GPIO_PIN_SET)
#define RELAY4_OFF    HAL_GPIO_WritePin(ON_RELAY4_GPIO_Port, ON_RELAY4_Pin, GPIO_PIN_RESET)

//24v输出控制
#define CO24V_1_ON    HAL_GPIO_WritePin(CO24V_1_GPIO_Port, CO24V_1_Pin, GPIO_PIN_RESET)
#define CO24V_1_OFF   HAL_GPIO_WritePin(CO24V_1_GPIO_Port, CO24V_1_Pin, GPIO_PIN_SET)
#define CO24V_2_ON    HAL_GPIO_WritePin(CO24V_2_GPIO_Port, CO24V_2_Pin, GPIO_PIN_RESET)
#define CO24V_2_OFF   HAL_GPIO_WritePin(CO24V_2_GPIO_Port, CO24V_2_Pin, GPIO_PIN_SET)
#define CO24V_3_ON    HAL_GPIO_WritePin(CO24V_3_GPIO_Port, CO24V_3_Pin, GPIO_PIN_RESET)
#define CO24V_3_OFF   HAL_GPIO_WritePin(CO24V_3_GPIO_Port, CO24V_3_Pin, GPIO_PIN_SET)
#define CO24V_4_ON    HAL_GPIO_WritePin(CO24V_4_GPIO_Port, CO24V_4_Pin, GPIO_PIN_RESET)
#define CO24V_4_OFF   HAL_GPIO_WritePin(CO24V_4_GPIO_Port, CO24V_4_Pin, GPIO_PIN_SET)
#define CO24V_5_ON    HAL_GPIO_WritePin(CO24V_5_GPIO_Port, CO24V_5_Pin, GPIO_PIN_RESET)
#define CO24V_5_OFF   HAL_GPIO_WritePin(CO24V_5_GPIO_Port, CO24V_5_Pin, GPIO_PIN_SET)
#define CO24V_6_ON    HAL_GPIO_WritePin(CO24V_6_GPIO_Port, CO24V_6_Pin, GPIO_PIN_RESET)
#define CO24V_6_OFF   HAL_GPIO_WritePin(CO24V_6_GPIO_Port, CO24V_6_Pin, GPIO_PIN_SET)
#define CO24V_7_ON    HAL_GPIO_WritePin(CO24V_7_GPIO_Port, CO24V_7_Pin, GPIO_PIN_RESET)
#define CO24V_7_OFF   HAL_GPIO_WritePin(CO24V_7_GPIO_Port, CO24V_7_Pin, GPIO_PIN_SET)

//扩展输出控制
#define DOUT1_ON    HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_SET )
#define DOUT1_OFF   HAL_GPIO_WritePin(DOUT1_GPIO_Port, DOUT1_Pin, GPIO_PIN_RESET )
#define DOUT2_ON    HAL_GPIO_WritePin(DOUT2_GPIO_Port, DOUT2_Pin, GPIO_PIN_SET )
#define DOUT2_OFF   HAL_GPIO_WritePin(DOUT2_GPIO_Port, DOUT2_Pin, GPIO_PIN_RESET )

//24v输出控制
#define OUTPUT24V_1_ON    HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, GPIO_PIN_SET)
#define OUTPUT24V_1_OFF   HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, GPIO_PIN_RESET)
#define OUTPUT24V_2_ON    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_SET)
#define OUTPUT24V_2_OFF   HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_RESET)

//设置DAC输出
#define SET_DAC1_VALUE(value)    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value)
#define SET_DAC2_VALUE(value)    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, value)



void PowerCtrTask(void const *pvParameters);

int DevicePowerOn(void);
void ResartHost(void);

void CtrShutDown(uint32_t f_shutDown);


void SetRelay( uint32_t relay );
void SetDisinfectRelay(uint8_t f_cmd);

uint8_t GetRelayStatus(void);
uint8_t GetCo24vStatus(void);
uint8_t GetExOutputStatus(void);
uint8_t Get24vOutStatus(void);
uint8_t GetChargeRelayStatus(void);
uint8_t GetMotorRelayStatus(void);
uint8_t GetDisinfectRelayStatus(void);

#endif
