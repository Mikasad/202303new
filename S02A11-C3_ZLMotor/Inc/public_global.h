/*
*******************************************************************************
 *版    权： 2021-xxxx,  GaussianRobot
 *文 件 名： public_global.h函数
 *简    介： 用于建立通信数据表格
 *作    者： LMmotor\忘川
 *日    期： 2022.1.4
 *功能描述： 
*******************************************************************************
 *备注：         
*******************************************************************************
*/
#ifndef __PUBLIC_GLOBAL_H__
#define __PUBLIC_GLOBAL_H__

#include "mc_config.h"

#define ADC_AMPER_Ratio  0.397f  
#define ParaNum    160		  
	  
typedef struct
{
    unsigned			bReadOnly : 1; 		   /*读写属性*/
    unsigned			bOKInMotion: 1; 	   /*运动中是否可修改*/
    unsigned			bOKMotorOn : 1; 	   /*使能是否可修改*/
    unsigned			bIsArray : 1;   	   /*是否是数组*/
    unsigned			bSaveToFlash : 1; 	 /*是否可以保存FLASH*/
    unsigned			bAxisRelated: 1; 	   /*是否轴相关*/
    unsigned			bReserved1 : 2;      /*保留*/

} PARAMETER_ATTRIBUTES;

typedef struct
{
    short								sParID;
    short								sAddress; 		               /*通讯参数地址	*/
    PARAMETER_ATTRIBUTES		        stAttributes;    /*相关属性   占用4个字节*/
    short								sMaxArrayIndex;              /*数组索引最大值  由于结构体对齐此处浪费 2个字节*/
    long								lMinValue; 	                 /*最小取值 */
    long 								lMaxValue; 		               /*最大取值范围*/
    long								lDefaultValue;               /*默认值*/
    long*								lpParam; 		                 /*对应变量映射*/

} PARAMETER_TABLE;
typedef struct
{
    u8 uVersionPartOfRobotType;				/*车型对应的版本号*/
    u8 uVersionPartOfMainVersion;			/*大版本号*/
    u8 uVersionPartOfFunVersion;			/*功能版本*/
    u8 uVersionPartOfSmallVersion;		/*bug修改，功能完善与优化*/
    u32 uVersionFullVersion;					/*完整版本号*/
    u32 uHardwareVersion;             /*硬件版本号*/
} MCU_Version;



#ifdef UART_DEBUG
extern  int16_t LV_Uart_TxBuffer[4][4010];
#endif

#ifdef Usart_Labview
extern  long LabView_Uart_TxBuffer[4][2010];
#endif

extern MCU_Version ProgramVersion;
extern uint8_t HallState1,HallState2;
extern int16_t  HAL_CommutationAngle;
extern const PARAMETER_TABLE  PARAMETER[];
extern PID_Handle_t PID_PosParamsM1;
extern PID_Handle_t PID_PosParamsM2;
extern Trapezoidal_Handle_t pTrapezoidalM1;
extern Trapezoidal_Handle_t pTrapezoidalM2;
extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDSpeedHandle_M2;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIqHandle_M2;
extern FOCVars_t FOCVars[2];
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2;
extern ENCODER_Handle_t ENCODER_M1;
extern ENCODER_Handle_t ENCODER_M2;
extern EncAlign_Handle_t EncAlignCtrlM1;
extern EncAlign_Handle_t EncAlignCtrlM2;
extern MCI_Handle_t Mci[2];
extern pSpeedMesa pSpeed_Mesa[];
extern int32_t VelocityPLLSwitch ;
extern s32 VBS_AvBusVoltage_V;


#endif
/*------------------------------------------------------------------------------*/

