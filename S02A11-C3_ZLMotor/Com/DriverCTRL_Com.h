#include "mc_config.h"
#ifndef __DriverCTRL_Com_H
#define __DriverCTRL_Com_H

//===================================================================
// 宏常量定义
//===================================================================
#define	COM_FRAME_Start_Byte1		(u8)0x52		//帧头1字节
#define	COM_FRAME_Start_Byte2		(u8)0x54		//帧头2字节
#define	COM_FRAME_DCU_Address	(u8)0x80		//工控板帧地址
#define	COM_FRAME_DriverCTRL_Address	(u8)0x03		//驱控模块帧地址
#define	COM_FRAME_End_Byte1		(u8)0x0A		//帧尾1字节
#define	COM_FRAME_End_Byte2		(u8)0x0D		//帧尾2字节

#define	COM_Buff_MaxDataSize			(u16)255		//存放数据信息字节数，不包括其他字节，最大0xff

#define Soft_Version   0x010000 
#define Hard_Version   0x010000
//===================================================================
//	枚举常量定义
//===================================================================
//处理数据状态
typedef  enum
{
	Com_RXStep_Idle		= (u8)0,			//空闲状态
	Com_RXStep_StartByte1_Receive,			//开始第一字节接收状态(默认状态)
	Com_RXStep_StartByte2_Receive,			//开始第二字节接收状态
	Com_RXStep_Address_Receive,			//地址字节接收	
	Com_RXStep_Length_Receive,				//指令数据字节长度接收
	Com_RXStep_Instruct_Receive,			//指令字节接收
	Com_RXStep_Data_Receive,				//数据字节接收
	Com_RXStep_CRC_Receive,				//CRC校验接收
	Com_RXStep_End						//接收结束
}Com_RXStep_TypeDef;

//通信指令集
typedef  enum
{
	Com_Instruct_IDLE			= (u8)0,				//空指令
	
	Com_Instruct_CheckData		= (u8)0X01,			//电机零点设定
	Com_Instruct_CheckVersion 	= (u8)0X02,			//版本信息查询
	Com_Instruct_MotorCTRL 	= (u8)0X04,			//升降电机控制	
	Com_Instruct_StutterStop	= (u8)0X08,			//旋转电机控制
	Com_Instruct_hDurationmsValue 	= (u8)0X10,			//紫外灯控制
//	Com_Instruct_StutterStop 	= (u8)0X20,			//急停命令
//	Com_Instruct_CheckState 	= (u8)0X40,			//查询状态命令
	
	Com_Instruct_ACK			= (u8)0XF0			//应答
}Com_Instruct_TypeDef;			//通信指令



//===================================================================
//	数据结构定义
//===================================================================
//通信结构体
typedef  struct
{
	Com_RXStep_TypeDef		RXStep;			//接收步骤
	Com_Instruct_TypeDef		Instruct;		//通信指令
	
	u8	DataLength;
	u8	DataCount;
	u8	DataLen_Com[100];
	u8	CrcStateFlag;

	u8 	LwipSendBuf[100];
	u8 	SendBufTemp[100];

	
}SD_Com_Struct_TypeDef;

//===================================================================
//	全局变量定义及声明
//===================================================================

#ifdef	__DriverCTRL_Com_C
			SD_Com_Struct_TypeDef	SDCom;			//通信模块结构体
#else
extern		SD_Com_Struct_TypeDef	SDCom;
#endif	

//===================================================================
//	全局函数声明
//===================================================================
void LWIP_Data_Process(void);

#endif   //__DriverCTRL_Com_H
