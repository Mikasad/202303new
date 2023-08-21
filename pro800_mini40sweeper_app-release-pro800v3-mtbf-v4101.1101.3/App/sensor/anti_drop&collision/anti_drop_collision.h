#ifndef _ANTI_DROP_COLLISION_H
#define _ANTI_DROP_COLLISION_H

#include "main.h"
#include "app_ota.h"
enum
{
	_eAntiDropType_Unknown = 0,
	_eAntiDropType_MVR     = 1,
	_eAntiDropType_TF      = 2,
};

typedef struct
{
	//通道1~4对应串口1~4
	uint8_t uch_Rx1State;//channel1 接收状态
	uint8_t uch_Rx2State;//channel2 接收状态  
	uint8_t uch_Rx3State;//channel3 接收状态  
	uint8_t uch_Rx4State;//channel4 接收状态      

	uint16_t un_channel1_Distance;//防跌落实时距离
	uint16_t un_channel2_Distance;//防跌落实时距离
	uint16_t un_channel3_Distance;//防跌落实时距离
	uint16_t un_channel4_Distance;//防跌落实时距离

	uint16_t un_channel1_Strength;
	uint16_t un_channel2_Strength;
	uint16_t un_channel3_Strength;
	uint16_t un_channel4_Strength;

//	uint32_t un_FRONT_Enable;
//	uint32_t un_LEFT_Enable;
//	uint32_t un_RIGHT_Enable;
//	uint32_t un_REAR_Enable;
//	uint32_t un_LEFT_FRONT_Enable;
//	uint32_t un_RIGHT_FRONT_Enable;
//	uint32_t un_LEFT_REAR_Enable;
//	uint32_t un_RIGHT_REAR_Enable;
	uint32_t sum_Enable ; //防跌落使能

	uint16_t un_channel1_Threshold;//防跌落1触发距离
	uint16_t un_channel2_Threshold;//防跌落2触发距离
	uint16_t un_channel3_Threshold;//防跌落3触发距离
	uint16_t un_channel4_Threshold;//防跌落4触发距离

	uint32_t b1_channel1_Enable:1;//防跌落1使能标志
	uint32_t b1_channel2_Enable:1;//防跌落2使能标志
	uint32_t b1_channel3_Enable:1;//防跌落3使能标志
	uint32_t b1_channel4_Enable:1;//防跌落4使能标志

	uint32_t b1_channel1_Disconnection:1;//防跌落1断线标志
	uint32_t b1_channel2_Disconnection:1;//防跌落2断线标志
	uint32_t b1_channel3_Disconnection:1;//防跌落3断线标志
	uint32_t b1_channel4_Disconnection:1;//防跌落4断线标志

	uint32_t b1_channel1_Trigger:1;//防跌落1触发标志
	uint32_t b1_channel2_Trigger:1;//防跌落2触发标志
	uint32_t b1_channel3_Trigger:1;//防跌落3触发标志
	uint32_t b1_channel4_Trigger:1;//防跌落4触发标志	
    
    uint8_t  uch_channel1_receive;
    uint8_t  uch_channel2_receive;
    uint8_t  uch_channel3_receive;
    uint8_t  uch_channel4_receive;
    
    uint8_t output_switch;
}Anti_Drop_Typedef;   



void HandleCAN2RXData(stc_can_rx_frame_t f_rx_msg);
void ConfigAntiDrop( uint32_t f_enable, uint32_t fp_threshold[] );
uint32_t GetAntiDropDisconnectStatus(void);
uint8_t GetAntiDropStop(void);
uint8_t GetGcubDisconnectStatus(void);
void SetAntiDropSwitch(uint8_t f_switch);

extern Anti_Drop_Typedef gt_AntiDrop;
extern uint8_t g_antiDropStop;
extern uint8_t g_antiDropDisconnectFlag;

void app_anti_collision_task_suspend_resume(app_ota_manage_task_t task_sta);
void AntiDropCollisionTask(void const *pvParameters);
void app_anti_rx_process_Task(void *pvParameters);



#endif
