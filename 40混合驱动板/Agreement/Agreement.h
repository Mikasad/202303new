#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "stm32f4xx_hal_usart.h"
/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/

#define DEBUG_USARTx_IRQn                            USART3_IRQn
#define DEBUG_USARTx_IRQHandler                      USART3_IRQHandler



//#define FRAME_LENTH               6    // 指令长度
#define FRAME_START               0x5A  // 协议帧开始
//#define FRAME_END                 '/'   // 协议帧结束
//#define FRAME_CHECK_BEGIN          1    // 校验码开始的位置 RxBuf[1]
//#define FRAME_CHECKSUM            14    // 校验码的位置   RxBuf[14]
//#define FRAME_CHECK_NUM           13    // 需要校验的字节数
//#define FILL_VALUE                0x55  // 填充值
//#define CODE_SETPID               0x07  // 设置PID参数
//#define CODE_SETTGT               0x08  // 设置目标值
//#define CODE_RESET                0x09   // 复位重启
//#define CODE_STARTMOTOR           0x0A   // 启动电机

#define ParaNum 250
/* 扩展变量 ------------------------------------------------------------------*/

enum bReadEnum
{
	WR = 0,
	OR = 1,
};
enum bSaveToFlashEnum
{
	YSFLASH,
	NOFLASH,
};
extern UART_HandleTypeDef husart_debug;

extern uint8_t Flash_Writesign;

  

extern UART_HandleTypeDef huart3;

/* 函数声明 ------------------------------------------------------------------*/
void MX_DEBUG_USART_Init(void);
void Flash_WriteCheck(void);
void ReturnError1(void); 
void QuickRead(void);
void Startagreement(void);

extern void (*ptr_Fun_)(void) ;//函数指针
/* 函数声明 ------------------------------------------------------------------*/
void MX_USARTx_Init(void);
void MX_USART3_UART_Init(void);
typedef struct
{
  enum			bReadEnum  bReadOnly; 		//读写属性
	enum			bSaveToFlashEnum bSaveToFlash; 	//是否允许保存Flsash
}PARAMETER_ATTRIBUTES;
//enum bOnly{RW,RO};
//enum bOnly bReadOnly;
//enum ToFlash{YSFLASH,NOFLASH};
//enum ToFlash bSaveToFlash;
typedef struct 
{
	short								sParID;                                                                                  
	short								sAddress; 		//通讯参数地址	
	PARAMETER_ATTRIBUTES		        stAttributes;   //相关属性
//	short								sMaxArrayIndex; //数组索引最大值
//	long								lMinValue; 	    //最小取值 
//	long 								lMaxValue; 		//最大取值范围
//	long								lDefaultValue;  //默认值
    long*								lpParam; 		//对应变量映射 
}PARAMETER_TABLE;

extern const PARAMETER_TABLE PARAMETER[];
extern int32_t transbuf[250];
#endif  /* __BSP_DEBUG_USART_H__ */