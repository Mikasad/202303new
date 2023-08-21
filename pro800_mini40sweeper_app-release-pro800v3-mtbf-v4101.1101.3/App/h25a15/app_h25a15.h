#ifndef __APP_H25A15__H__
#define __APP_H25A15__H__

//#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"
#include "string.h"
#include "main.h"

typedef struct
{
	uint8_t run_status;                 //0关闭 1运行 2运行中
	uint8_t over_current_status;       //过流状态
	uint16_t rx_cmd;		            //上上位机下发的命令
	uint16_t tx_cmd;		            //下发给驱动器的命令
	uint16_t current;                   //当前电流单位ma
	uint16_t max_current;               //允许的最大电流
	uint16_t over_current_time;         //允许的过流时间
	uint16_t operate_time;              //电机动作时间，针对推杆电机位置控制
	
	
}Motor_Info_t;        //电机状态

typedef struct
{
	uint8_t self_adaption_enable;   //1:使能自适应控制
	uint8_t target_current;         //目标电流，默认2A，单位0.1A
	uint8_t tx_cmd ;                //上次发送的命令
	uint8_t real_position ;         //当前位置
	uint8_t deviation;              //允许偏差，单位百分之1，最小百分之5
	uint16_t debounce_time;         //自适应电流去抖时间
}Lift_Self_Adaption_t;              //推杆自适应控制

typedef struct 
{
	uint32_t link_time;			//上位机连接时间
	uint32_t link_status;		//上位机通讯状态 0断连1连接正常
}H25a_Info_t;



extern Motor_Info_t Squeegee_Lift_Info ;
extern Motor_Info_t Roller_Brush_Lift_Info ;
extern Motor_Info_t Side_Brush_Lift_Info  ;
extern Motor_Info_t Water_Pump_Info  ;
extern Motor_Info_t Fan_Motor_Info;
extern Motor_Info_t Roller_motor_Info  ;
extern Motor_Info_t Side_motor_Info[2] ;
extern Lift_Self_Adaption_t  Self_Adaption; 
/*创建自研驱动器电机任务*/
void H25A_Motor_Task_Create(void);
/*自研驱动任务*/
void H25A_Motor_Task(void *pvParameters );
/*自研驱动器电机控制任务*/
void Motor_Control_Task(void);
/*喷水控制*/
void Water_Motor_Logic(uint8_t speed) ;
/*风机控制*/
void Fan_Motor_Logic(uint8_t speed) ;
/*滚刷边刷控制*/
void Brush_Motor_Logic( void) ;
/*滚刷升降控制*/
//void Roller_Brush_Lift_Logic(uint8_t current_cmd) ;
void Roller_Brush_Lift_Logic(uint8_t current_cmd,uint8_t target_current);

/*边刷升降控制*/
void Side_Brush_Lift_Logic(uint8_t current_cmd) ;
/*吸水趴推杆控制逻辑*/
void Squeegee_Lift_Logic(uint8_t current_cmd) ;
/*驱动器断连处理逻辑*/
void H25_Disconnetion_Logic(void) ;
/*滚刷升降自适应控制*/
void Roller_Brush_Lift_Self_Adapt(void) ;










#endif

