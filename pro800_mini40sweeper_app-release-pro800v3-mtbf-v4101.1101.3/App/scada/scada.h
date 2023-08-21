#ifndef _SCADA_H
#define _SCADA_H

#include <stdint.h>


typedef struct{
	uint16_t batteryVoltageLow;//电池电压无输入时的最大空载电压
	uint16_t batteryVoltageEmpty;//复充电压
	uint16_t batteryVoltageFull;//充满时的电压
	uint16_t chargeCurrentLow;//无充电电流输入时的最大空载电流
	uint16_t chargeCurrentEmpty;//充满电时的电流
	uint16_t chargeCurrentFast;//正常充电时的电流，大电流充电
	uint16_t chargeCurrentShort;//短期电流，指充电刚开始阶段的电流？
	uint16_t chargeVoltageLow;//充电电压无输入的最大空载电压
	uint16_t chargeTouchVoltageLow;//充电接触电压的低值
	uint16_t chargeTouchVoltageHigh;//接触电压的高值
	uint16_t chargeMaxTime;//单次最长充电时间*2为实际S数
}chargeConfig_t;//充电信息



uint16_t GetBatteryVoltage(void);
uint16_t GetChargeVoltage(void);
//uint16_t GetManualChargeVoltage(void);
uint16_t GetChargeCurrent(void);
void CfgChargeData( chargeConfig_t charge );
void GetAdcValue( float value[] );
uint16_t GetTorqueVoltage1(void);
uint16_t GetTorqueVoltage2(void);

uint16_t GetDustBag1Voltage(void);
uint16_t GetDustBagStatus(void);
uint16_t GetDustBagErrorStatus(void);
void ScadaTask(void);



#endif

