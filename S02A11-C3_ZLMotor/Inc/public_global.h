/*
*******************************************************************************
 *��    Ȩ�� 2021-xxxx,  GaussianRobot
 *�� �� ���� public_global.h����
 *��    �飺 ���ڽ���ͨ�����ݱ��
 *��    �ߣ� LMmotor\����
 *��    �ڣ� 2022.1.4
 *���������� 
*******************************************************************************
 *��ע��         
*******************************************************************************
*/
#ifndef __PUBLIC_GLOBAL_H__
#define __PUBLIC_GLOBAL_H__

#include "mc_config.h"

#define ADC_AMPER_Ratio  0.397f  
#define ParaNum    160		  
	  
typedef struct
{
    unsigned			bReadOnly : 1; 		   /*��д����*/
    unsigned			bOKInMotion: 1; 	   /*�˶����Ƿ���޸�*/
    unsigned			bOKMotorOn : 1; 	   /*ʹ���Ƿ���޸�*/
    unsigned			bIsArray : 1;   	   /*�Ƿ�������*/
    unsigned			bSaveToFlash : 1; 	 /*�Ƿ���Ա���FLASH*/
    unsigned			bAxisRelated: 1; 	   /*�Ƿ������*/
    unsigned			bReserved1 : 2;      /*����*/

} PARAMETER_ATTRIBUTES;

typedef struct
{
    short								sParID;
    short								sAddress; 		               /*ͨѶ������ַ	*/
    PARAMETER_ATTRIBUTES		        stAttributes;    /*�������   ռ��4���ֽ�*/
    short								sMaxArrayIndex;              /*�����������ֵ  ���ڽṹ�����˴��˷� 2���ֽ�*/
    long								lMinValue; 	                 /*��Сȡֵ */
    long 								lMaxValue; 		               /*���ȡֵ��Χ*/
    long								lDefaultValue;               /*Ĭ��ֵ*/
    long*								lpParam; 		                 /*��Ӧ����ӳ��*/

} PARAMETER_TABLE;
typedef struct
{
    u8 uVersionPartOfRobotType;				/*���Ͷ�Ӧ�İ汾��*/
    u8 uVersionPartOfMainVersion;			/*��汾��*/
    u8 uVersionPartOfFunVersion;			/*���ܰ汾*/
    u8 uVersionPartOfSmallVersion;		/*bug�޸ģ������������Ż�*/
    u32 uVersionFullVersion;					/*�����汾��*/
    u32 uHardwareVersion;             /*Ӳ���汾��*/
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

