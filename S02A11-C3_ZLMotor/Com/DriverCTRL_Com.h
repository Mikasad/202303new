#include "mc_config.h"
#ifndef __DriverCTRL_Com_H
#define __DriverCTRL_Com_H

//===================================================================
// �곣������
//===================================================================
#define	COM_FRAME_Start_Byte1		(u8)0x52		//֡ͷ1�ֽ�
#define	COM_FRAME_Start_Byte2		(u8)0x54		//֡ͷ2�ֽ�
#define	COM_FRAME_DCU_Address	(u8)0x80		//���ذ�֡��ַ
#define	COM_FRAME_DriverCTRL_Address	(u8)0x03		//����ģ��֡��ַ
#define	COM_FRAME_End_Byte1		(u8)0x0A		//֡β1�ֽ�
#define	COM_FRAME_End_Byte2		(u8)0x0D		//֡β2�ֽ�

#define	COM_Buff_MaxDataSize			(u16)255		//���������Ϣ�ֽ����������������ֽڣ����0xff

#define Soft_Version   0x010000 
#define Hard_Version   0x010000
//===================================================================
//	ö�ٳ�������
//===================================================================
//��������״̬
typedef  enum
{
	Com_RXStep_Idle		= (u8)0,			//����״̬
	Com_RXStep_StartByte1_Receive,			//��ʼ��һ�ֽڽ���״̬(Ĭ��״̬)
	Com_RXStep_StartByte2_Receive,			//��ʼ�ڶ��ֽڽ���״̬
	Com_RXStep_Address_Receive,			//��ַ�ֽڽ���	
	Com_RXStep_Length_Receive,				//ָ�������ֽڳ��Ƚ���
	Com_RXStep_Instruct_Receive,			//ָ���ֽڽ���
	Com_RXStep_Data_Receive,				//�����ֽڽ���
	Com_RXStep_CRC_Receive,				//CRCУ�����
	Com_RXStep_End						//���ս���
}Com_RXStep_TypeDef;

//ͨ��ָ�
typedef  enum
{
	Com_Instruct_IDLE			= (u8)0,				//��ָ��
	
	Com_Instruct_CheckData		= (u8)0X01,			//�������趨
	Com_Instruct_CheckVersion 	= (u8)0X02,			//�汾��Ϣ��ѯ
	Com_Instruct_MotorCTRL 	= (u8)0X04,			//�����������	
	Com_Instruct_StutterStop	= (u8)0X08,			//��ת�������
	Com_Instruct_hDurationmsValue 	= (u8)0X10,			//����ƿ���
//	Com_Instruct_StutterStop 	= (u8)0X20,			//��ͣ����
//	Com_Instruct_CheckState 	= (u8)0X40,			//��ѯ״̬����
	
	Com_Instruct_ACK			= (u8)0XF0			//Ӧ��
}Com_Instruct_TypeDef;			//ͨ��ָ��



//===================================================================
//	���ݽṹ����
//===================================================================
//ͨ�Žṹ��
typedef  struct
{
	Com_RXStep_TypeDef		RXStep;			//���ղ���
	Com_Instruct_TypeDef		Instruct;		//ͨ��ָ��
	
	u8	DataLength;
	u8	DataCount;
	u8	DataLen_Com[100];
	u8	CrcStateFlag;

	u8 	LwipSendBuf[100];
	u8 	SendBufTemp[100];

	
}SD_Com_Struct_TypeDef;

//===================================================================
//	ȫ�ֱ������弰����
//===================================================================

#ifdef	__DriverCTRL_Com_C
			SD_Com_Struct_TypeDef	SDCom;			//ͨ��ģ��ṹ��
#else
extern		SD_Com_Struct_TypeDef	SDCom;
#endif	

//===================================================================
//	ȫ�ֺ�������
//===================================================================
void LWIP_Data_Process(void);

#endif   //__DriverCTRL_Com_H
