#ifndef __SECURITY_STATE_H_
#define __SECURITY_STATE_H_

#include <stdint.h>



typedef struct{
	uint8_t front:1;
	uint8_t back:1;
	uint8_t left:1;
	uint8_t right:1;
	uint8_t leftFront:1;
	uint8_t rightFront:1;
	uint8_t leftBack:1;
	uint8_t rightBack:1;
}eightDir_t;



typedef enum 
{
	ROBOT_MOVE_DIR_STOP = 0,
	ROBOT_MOVE_DIR_FRONT = 0x0001,
	ROBOT_MOVE_DIR_LEFTFRONT = 0x0002,
	ROBOT_MOVE_DIR_RIGHTFRONT = 0x0004,
	ROBOT_MOVE_DIR_LEFT = 0x0008,
	ROBOT_MOVE_DIR_RIGHT = 0x0010,
	ROBOT_MOVE_DIR_BACK = 0x0020,
	ROBOT_MOVE_DIR_LEFTBACK = 0x0040,
	ROBOT_MOVE_DIR_RIGHTBACK = 0x0080,
	ROBOT_MOVE_DIR_SPOTTURN = 0x0100,//原地打转
}robotMoveDir_t;

typedef union{
	eightDir_t dir;	
	uint8_t allBits;
}safetySensorTriggerStatus_t;//安全传感器的各种状态，1表示触发

typedef union{
	eightDir_t dir;		
	uint8_t allBits;
}sensorCfgEnable_t;//配置传感器使能
typedef union{
	eightDir_t dir;		
	uint8_t allBits;
}sensorEnableStatus_t;//传感器当前的使能状态
typedef struct{
	uint32_t last_connect_time;//起始时间
	uint32_t timeout_ms;//超时时间间隔,单位ms
}ConnectionStatus_t;

uint32_t CheckForbidMoveDirBySafetySensor(safetySensorTriggerStatus_t f_safetySensorTriggerStatus);
int GetLinkStatus(ConnectionStatus_t *fp_connStatus);
void ResetConnectedTime(ConnectionStatus_t *fp_connStatus);
#endif
