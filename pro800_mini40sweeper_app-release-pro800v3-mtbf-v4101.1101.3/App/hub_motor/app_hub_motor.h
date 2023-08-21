#ifndef __APP_HUB_MOTOR_H__
#define __APP_HUB_MOTOR_H__
#include "hc32_ll.h"
#include "cmsis_os.h"

#define  HAL_GetTick SysTick_GetTick


typedef enum 
{
    CTR_TROLLEY=0,//手推
	CTR_AUTO,//自动
}Cont_Mode_t;


typedef struct
{
	uint8_t rx_mode;		    //上位机下发的工作模式0 手动 1自动 
	uint8_t tx_mode;	       //下发的的工作模式0 手动 1自动 
	uint8_t rx_ctrl_mode;       //上位机下发的工作模式0速度模式 1转矩模式
	uint8_t real_mode;		    //根据驱动器控制字获取的工作模式0 手动 1自动 
	uint8_t clean_error_flag;	//收到上位机清除告警的标志
	int16_t left_rx_speed;	    //上位机下发的左轮速度
	int16_t right_rx_speed;	    //上位机下发的右轮速度
	int16_t left_speed;		    //实际下发给驱动器的左轮速度
	int16_t right_speed;	    //实际下发给驱动器的右轮速度
}Motor_Control_Info_t;


extern Motor_Control_Info_t Hub_Info;

/*创建轮毂电机任务*/
void Hub_Motor_Task_Create(void);
/*电机控制逻辑*/
void Hub_Motor_Logic(void);
/*轮毂电机任务*/
void ZLAC_Motor_Task(void *pvParameters );
/*检查模式切换状态*/
uint8_t Hub_Motor_Mode_Check(void);
/*读取轮毂电机相关信息*/
void Hub_Motor_Info_Read(void);
/*清除轮毂电机告警*/
void Hub_Motor_error_Clear(void);
/*更新轮毂驱动器参数*/
void Hub_Motor_Param_Update(void);
#endif

