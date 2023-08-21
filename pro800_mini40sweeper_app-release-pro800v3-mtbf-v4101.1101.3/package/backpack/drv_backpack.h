#ifndef __DRV_BACKUPPACK_H
#define __DRV_BACKUPPACK_H

 
/******************************头文件***************************************/
#include "main.h"
#include "main.h"
#include <string.h>

/******************************类型声明*************************************/
typedef enum
{
	USE_NO_BACK_PACK            = 0,    /*未使用背包*/
	USE_DISINFECTION_PACK_1     = 1,    /*消杀1.0背包*/
	USE_DISINFECTION_PACK_2     = 2,    /*消杀2.0背包*/
	USE_DISINFECTION_PACK_3     = 3,    /*消杀3.0背包*/
} BackpackType_t;


typedef struct
{
	uint8_t device_box_type;     /*控制盒类型 0和1PRO800V1.0 2：PRO800V2.0*/
	uint8_t disinfect_type;      /*消杀包类型*/
    uint8_t incense_type;        /*香薰包类型*/
	uint8_t disinfect_detect;    /*消杀背包安装状态检测*/
	uint8_t incense_detect;      /*熏香背包安装状态检测*/
	uint8_t hard_info;           /*硬件信息   bit0 1有消杀0无消杀 bit1 1有香薰0无香薰*/   
	uint8_t disfection_level;    /*消杀水箱   液位状态，1低液位触发0未触发*/
	uint8_t atomizer_level;      /*雾化器水箱 液位状态，1低液位触发0未触发*/
	uint8_t incense_level;       /*香薰水箱   液位状态，1低液位触发0未触发*/

}BackPackInfo_t;

typedef struct
{
	uint8_t disinfect_cmd_1;     /*消杀继电器1控制*/
    uint8_t disinfect_cmd_2;     /*消杀继电器2控制*/
    uint8_t incense_cmd;         /*香薰命令 0:关闭 1:打开*/
}BackPackCtrl_t;


/*****************************常量定义**************************************/
#define CAR_TYPE     40


/******************************宏定义***************************************/
#define CO24V_1_ON        HAL_GPIO_WritePin(CO24V_1_GPIO_Port, CO24V_1_Pin, GPIO_PIN_RESET)  
#define CO24V_1_OFF       HAL_GPIO_WritePin(CO24V_1_GPIO_Port, CO24V_1_Pin, GPIO_PIN_SET)
#define OUTPUT1_24V_ON    HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, GPIO_PIN_SET)
#define OUTPUT1_24V_OFF   HAL_GPIO_WritePin(OUTPUT1_GPIO_Port, OUTPUT1_Pin, GPIO_PIN_RESET)
#define OUTPUT2_24V_ON    HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_SET)
#define OUTPUT2_24V_OFF   HAL_GPIO_WritePin(OUTPUT2_GPIO_Port, OUTPUT2_Pin, GPIO_PIN_RESET)

/******************************导出函数**************************************/
/****************************************************************************
*函数名	: BackpackTaskInit
*介	 绍	：背包任务初始化
*****************************************************************************/
void BackpackTaskInit(void);
 


 
 


 

#endif 
