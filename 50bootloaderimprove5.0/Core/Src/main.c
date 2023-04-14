#include "main.h"
#include <stdio.h>
#include "stm32f4xx_hal_flash.h"
CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
UART_HandleTypeDef huart3;
pFunction Jump_To_Application;
TIM_HandleTypeDef htim7;

uint32_t JumpAddress;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
SysStatus_t SysState;
OTA_TypeDef OTA;
#define GREEN_LED_TOGGLE        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12)
#define RED_LED2_TOGGLE         HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12)
#define RED_LED1_TOGGLE         HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6)
uint16_t Receive_cnt000=0;
uint8_t buf_send[8];                  /*CAN�ظ������ݰ�*/
uint16_t ii;                          /*�ȴ�ʱ��*/
uint8_t Can_Err=0;                    /*���Բ鿴CAN�Ƿ��з���ʧ�ܵ����*/
uint32_t OTA_Flag[2]= {2,0};          /*OTA�жϱ�־λ��APP��תǰдΪ1��APP��ʼ��ֱ�Ӳ���ΪFF,�������ֱ����ת�����жϣ�
                                        �����ʱAPP������ȷ����Ϊû�в���ADDR_FLASH_SECTOR_4��ַ��01,����һֱ��boot
                                        �еȴ��������������������жϵ�\bin�ļ�����ȷ����������ô�����ַ��ֵ����
                                        ������һֱ����Ϊ1�����ܽ�����ת*/
int OTA_Time=0;                       /*�����ж�OTA������ʱ*/
int Message_Flag=0;                   /*�յ�CAN�����ֵ��1*/
void Flash_Read(uint32_t addresss,uint32_t *pBuffer,uint32_t bufsize);   /*��ĳ����ַĳ�����ȵ����ݷ��� pbuffer��ָ��������*/
uint8_t flash_buf[PACK_SIZE];         /*1024��С���ܼ�1kb����*/
uint8_t pack_cnt = 0,byte_cnt =0;     /*pack cnt ��ʾ����kb��byte cnt ��ʾ���ٸ��ֽ�*/
uint32_t ApplicationAddress = 0x8020000,APP_data[2],BackUp_data[2]; /*APP��ʼ��ַ��appdataδʹ�ã�backupdata��ʾ��Ҫ���˵Ĵ����С*/
uint32_t  read_buf[1] = {0};         /*���ڴ�Ŵ�APP д��� 8020200 ��ַд��� S02A���� ��������ʾ��0x41323053*/
uint8_t Version_Flag=0;   //�汾�жϱ�־
/* CRC��λ�ֽ�ֵ��*/
const unsigned char auchCRCHi[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
/* CRC��λ�ֽ�ֵ��*/
const unsigned char auchCRCLo[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;

unsigned short crc16(unsigned char *puchMsg, unsigned short usDataLen)
{
    unsigned char uchCRCHi = 0xFF; /* ��CRC�ֽڳ�ʼ�� */
    unsigned char uchCRCLo = 0xFF; /* ��CRC �ֽڳ�ʼ�� */
    unsigned short uIndex; /* CRCѭ���е����� */

    while (usDataLen--)  /* ������Ϣ������ */
    {
        uIndex = uchCRCHi^ *puchMsg++; /* ����CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}


uint8_t CRC16_chk(unsigned char *puchMsg, unsigned short usDataLen)
{
    uint16_t CRC16_temp = 0;
    CRC16_temp = crc16(puchMsg, usDataLen);
    if(((uint8_t)(CRC16_temp >> 8) == *(puchMsg + usDataLen)) && ((uint8_t)(CRC16_temp) == *(puchMsg + usDataLen + 1)))
        return TRUE;
    else return FALSE;
}

uint8_t CRC16_creat(unsigned char *puchMsg, unsigned short usDataLen)
{
    uint16_t CRC16_temp = 0;
    CRC16_temp = crc16(puchMsg, usDataLen);
    *(puchMsg + usDataLen) = (uint8_t)(CRC16_temp >> 8);
    *(puchMsg + usDataLen + 1) = (uint8_t)CRC16_temp;
    return TRUE;
}


int main(void)
{
    uint8_t rollback_packnum =0;         /*�ֽ���*/
    uint32_t flash_buftemp[4];           /*δʹ��*/
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_CAN2_Init();
    MX_TIM7_Init();
    Flash_Read(ADDR_FLASH_SECTOR_4,OTA_Flag,1);        /*��ȡ 0x0800C00��ַ������*/
//  MX_USART3_UART_Init();
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
    HAL_TIM_Base_Start_IT(&htim7);

    APP_data[0] = *(long*)Data_Version_Sector;					//app�汾��
    APP_data[1] = *(long*)(Data_Version_Sector+4);			//app�����С
    BackUp_data[0] = *(long*)(Data_Version_Sector+8);		//���ݴ���汾��
    BackUp_data[1] = *(long*)(Data_Version_Sector+12);	//���ݴ����С

    buf_send[0] = 0xe2;
    buf_send[1] = 0x01;
    CANx_SendNormalData(&hcan2,0x20,buf_send,2);  /*�ϵ緢�� CANID: 0x20 data: e2 01*/

    while (1)
    {
        if(OTA_Time>10000 )                                                /*������ʱ���������г�ʱ��δ�յ����� 0x11��ͷ����*/
        {
			OTA_Time=0;                                                 //��ֹ������;�������ߣ�������볬ʱ�ȴ����������0�����һֱ��wait��
            SysState=Wait;                                            /*���ˣ��ڻ������жϴ����Ƿ���Խ��л��ˣ�������ܻ������ʹ��������ת��wait״̬*/
        }
        switch(SysState)
        {
        case Wait:
            if(HAL_GetTick()>5000)                                         /*�����wait״̬����2.5S����ת��app���ϵ粻������ת*/
            {
                SendErrorCode(0x04,0x01);                                  /*������Wait״̬*/
                Flash_Read(0x08020200,read_buf,1);                         /*�˴���̹�ȷ��*//*�õ�ַ������ΪAPP����д��� S02A*/
                Flash_Read(ADDR_FLASH_SECTOR_4,OTA_Flag,1);                /*���otaflag���Աջ���������*/
                if((OTA_Flag[0] == 0xFF ||OTA_Flag[0] == 0xFFFFFFFF) && (read_buf[0] == 0x41413448))  /*  ��ȡapp�����Ƿ�Ϊ�����е�app�� 0x41323053 Ϊ asc�� A20S*/
                {
                    ApplicationAddress = APP_Sector_1;                         /*app��������*/
                    if(Jump_to_APP(ApplicationAddress)==FALSE)
                    {
                        HAL_Delay(1000);                                   /*�˴���̹�ȷ��*/
                        SendErrorCode(0x05,0x01);                          /*�޷���ת��app��*/
                    }
                }
                else if(OTA_Flag[0] == 0x00000002 || OTA_Flag[0] == 0x00000001)      /*�������1��APP�����ã��������򲻿��ã�1�����ʱbootloader����û�н�������������ֱ����תapp*/
                {
                    ApplicationAddress = APP_Sector_1;
                    if(Jump_to_APP(ApplicationAddress)==FALSE)
                    {
                        HAL_Delay(1000);                                   /*�˴���̹�ȷ��*/
                        SendErrorCode(0x05,0x01);                          /*�޷���ת��app��*/
                    }
                }
                else if(OTA_Flag[0] == 0x00000003)                         /*�������2�����������ã�APP���򲻿���*/
                {
                    SysState = Rollback;                                   /*���л���*/
                }
                else
                {
                    SendErrorCode(0x02,0x04);                              /*���Ǹ��豸��bin�ļ���������ת*/
                }
            }
            break;
        case Updating:
            if(OTA.EraseFlag)                                              /*�յ� ��λ���ظ���CANID:0x20 data: 14 xx�Ժ󽫱�־λ��ֵ1����ʼ���� APP��������*/
            {
                FlashEraseSector(FLASH_SECTOR_4,1);
                OTA_Flag[0] = 0x00000003;
                WriteUint32_tData_toFlash(OTA_Flag,1,ADDR_FLASH_SECTOR_4);
                /*�˴��� otaflag дΪ3 ��ʾ �������Ĵ�����ã�����APP���Ĵ�����ܴ��ڷ��ղ���ʹ�� ȷ���ϵ����*/
                if(Erase_Sector(ApplicationAddress,OTA.ByteNum)==TRUE)     /*����APP���������*/
                {
                    pack_cnt++;
                    buf_send[0] = pack_cnt;
                    buf_send[1] = byte_cnt;
                    CANx_SendNormalData(&hcan2,0x21,buf_send,2);           /*���������ݰ���Ϣ���֣��ظ���λ��CANID: 0x21 data: 01 00�������*/
                    OTA.Erase_finished = 1;
                }
                else
                {
                    SendErrorCode(0x01,0x02);                              /*����ʧ��*/
                }
                OTA.EraseFlag=0;
            }
            /*��������в����������ܽ���д�����*/
            if(OTA.WriteFlag && OTA.Erase_finished == 1)                   /*CRCУ�鲢�Ҳ�������Ժ�ʼд��*/
            {
                if(Flash_Write((pack_cnt-1)*PACK_SIZE+ApplicationAddress,flash_buf,OTA.SizeOfArray)==TRUE)   /*д��FLASH*/
                {
                    byte_cnt =0;                                           /*֡��������*/
                    pack_cnt++;                                            /*KB����++*/
                    buf_send[0] = pack_cnt;
                    buf_send[1] = byte_cnt;
                    CANx_SendNormalData(&hcan2,0x21,buf_send,2);      /*�ظ� CANID:0x21 data: pack_cnt byte_cnt*/
                }
                else
                {
                    SendErrorCode(0x02,0x03);
                }
                OTA.WriteFlag=0;
            }
            if(OTA.Back_finishFlag == 1&&OTA.version_flag == 1)            /*������ɲ�����λ�����͹̼���Ϣ���ſ���Ϊ���������*/
            {
				if(Version_Flag==1)
			  {
                 Erase_Sector(ADDR_FLASH_SECTOR_4,1);                       /*�˴�������Ϊ�˼��ݾɰ汾�����磺�°汾��bootloaderˢ��1.4.16��ǰ�İ汾
							                                                               ��Ϊ֮ǰ�İ汾APP��û�в�������������������޷�������ɿ�����ת��������
			  			                                                               ������ת*/
			  }
//                ApplicationAddress = APP_Sector_1;
//                Flash_Read(0x08020200,read_buf,1);                         /*�õ�ַ������ΪAPP����д��� S02A*/
//                if(read_buf[0] == 0x41413448)                              /*��ȡapp�����Ƿ�Ϊ�����е�app�� 0x41323053 Ϊ asc�� A20S*/
//                {
//                    if(Jump_to_APP(ApplicationAddress)==FALSE)
//                    {
//                        HAL_Delay(1000);
//                        SendErrorCode(0x05,0x01);                          /*�޷���ת��app��*/
//                    }
//                }
//                else
//                {
//                    SendErrorCode(0x02,0x04);                              /*���Ǹ��豸��bin�ļ���������ת*/
                SysState = Wait;                                       /*״̬��ת��Wait,����ô�����������ٷ���*/
//                }
            }
            break;
        case Copying:
			if(OTA_Flag[0] != 0x00000003)
			{
				OTA.BackUp_ByteNum = 131072;		                                /*128K ��С102400 ��Ϊ 131072*/
	//            OTA.BackUp_PackNum = (OTA.BackUp_ByteNum%PACK_SIZE==0) ? OTA.BackUp_ByteNum/PACK_SIZE:OTA.BackUp_ByteNum/PACK_SIZE+1;       //���ٸ�pack��
				if(OTA.BackUp_ByteNum%PACK_SIZE == 0)                           /*��ֵ�̶�Ϊ 128*/
				{
					OTA.BackUp_PackNum = OTA.BackUp_ByteNum/PACK_SIZE;          /*��������128KB ����ȫ��ת�ƣ���Ҫ�����������ݣ�����������ڴ��볬��100KB��Ҫ�޸�bootloader*/
				}
				else
				{
					OTA.BackUp_PackNum = OTA.BackUp_ByteNum/PACK_SIZE+1;        /*�����ϣ������������������*/
				}
				FlashEraseSector(FLASH_SECTOR_4,1);
				OTA_Flag[0] = 0x00000002;
				WriteUint32_tData_toFlash(OTA_Flag,1,ADDR_FLASH_SECTOR_4);
				/*�˴��� otaflag дΪ2 ��ʾ APP�Ĵ�����ã����Ǳ������Ĵ�����ܴ��ڷ��ղ���ʹ�� ��ֹ�ϵ�������֪���û��˵�APP���߱�����*/
				if(APP_Backup(APP_Sector_1,APP_Sector_2,OTA.BackUp_ByteNum,OTA.BackUp_PackNum)==TRUE)       /*APP�������ﲻ�ж�S02A�������Ӱ���������̣��Ȳ���APP_Sector_2������룬
																															�ٽ�APP_Sector_1 TO APP_Sector_2���� */
				{
					OTA.Back_finishFlag =1;          /*ȷ�����ϵ�����»�������*/
					buf_send[0] = 0xe4;
					buf_send[1] = 0x01;
					CANx_SendNormalData(&hcan2,0x20,buf_send,2);
					SysState = Updating;                              /*������ɺ�������״̬*/
				}
		    }
			else                                   //��ֹ���ݵ�ʱ������ϵ磬֮����λ���������������Լ���������������������ƭ��λ��������ɡ�
			{
//				OTA.Back_finishFlag =1;          /*ȷ�����ϵ�����»�������*/
//				buf_send[0] = 0xe4;
//				buf_send[1] = 0x01;
//				CANx_SendNormalData(&hcan2,0x20,buf_send,2);
				SendErrorCode(0x08,0x08);
				SysState = Wait;                              /*������ɺ�������״̬*/
			}

            break;
        case Rollback:                                                  /*�汾����*/
            BackUp_data[1]=131072;                                      /*128K ��С102400 ��Ϊ 131072*/
//            rollback_packnum = (BackUp_data[1]%PACK_SIZE==0) ? BackUp_data[1]/PACK_SIZE:BackUp_data[1]/PACK_SIZE+1;    //pack������
            if(BackUp_data[1]%PACK_SIZE == 0)
            {
                rollback_packnum = BackUp_data[1]/PACK_SIZE;
            }
            else
            {
                rollback_packnum = BackUp_data[1]/PACK_SIZE+1;
            }
            Flash_Read(0x08040200,read_buf,1);                           /*�õ�ַ������ΪAPP����д��� S02A����Ϊ�ǻ���������Ҫ�жϱ�����������*/
            if(read_buf[0] == 0x41413448)   /*��ȡapp�����Ƿ�Ϊ�����е�app�� 0x41323053 Ϊ asc�� A20S*/
            {
                if(APP_Backup(APP_Sector_2,APP_Sector_1,BackUp_data[1],rollback_packnum)==TRUE)   /*APP�������*/
                {
					Erase_Sector(ADDR_FLASH_SECTOR_4,1);   
                    ApplicationAddress = APP_Sector_1;
                    if(Jump_to_APP(ApplicationAddress)==FALSE)  //��ת��app��
                    {
                        HAL_Delay(1000);
                        SendErrorCode(0x05,0x01);
                    }
                }
            }
            else
            {
                SendErrorCode(0x02,0x04);                                /*���Ǹ��豸��bin�ļ���������ת*/
                SysState = Wait;                                         /*״̬��ת��Wait,����ô�����������ٷ���*/
//                ApplicationAddress = APP_Sector_1;                     /*app��������*/
//                Flash_Read(0x08020200,read_buf,1);                     /*�˴���̹�ȷ��*//*�õ�ַ������ΪAPP����д��� S02A*/
//                if(read_buf[0] == 0x41323053)                          /*��ȡapp�����Ƿ�Ϊ�����е�app�� 0x41323053 Ϊ asc�� A20S*/
//                {
//                    if(Jump_to_APP(ApplicationAddress)==FALSE)
//                    {
//                        HAL_Delay(1000);                               /*�˴���̹�ȷ��*/
//                        SendErrorCode(0x05,0x01);                      /*�޷���ת��app��*/
//                    }
//                }
            }
            break;

        case Fault:
            break;
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    // HAL_RCC_GetHCLKFreq()/1000    1ms
    // HAL_RCC_GetHCLKFreq()/100000	 10us
    // HAL_RCC_GetHCLKFreq()/1000000 1us
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/2000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);


    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

}





static void MX_TIM7_Init(void)     //�ж�1ms
{

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    __HAL_RCC_TIM7_CLK_ENABLE();
    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 83;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 1000;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0); //��ˢ���� �͵��7��8����
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

}

void TIM7_IRQHandler(void)
{
    /* USER CODE BEGIN TIM7_IRQn 0 */

    /* USER CODE END TIM7_IRQn 0 */
    HAL_TIM_IRQHandler(&htim7);
    /* USER CODE BEGIN TIM7_IRQn 1 */

    /* USER CODE END TIM7_IRQn 1 */
}
#define frequency_of_led 50
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    int32_t TempValue;
    static uint32_t red_led_cnt = 0;
    static uint32_t green_led_cnt = 0;
    if(htim==(&htim7))                                  /*�������������ʱ*/
    {
        red_led_cnt++;
        green_led_cnt++;
        if(Message_Flag==1 )   /*�������̵���˸*/
        {
            OTA_Time++;      /*������ʱʱ�䣬����ʱ��δ�յ����ݣ�����л���*/
            if(RxMessage.StdId ==0x11)
            {
                if(green_led_cnt > frequency_of_led)
                {
                    green_led_cnt = 0;
                    GREEN_LED_TOGGLE;
                }
            }
        }
        if(red_led_cnt > frequency_of_led)              /*��boot�к����˸*/
        {
            red_led_cnt = 0;
            RED_LED1_TOGGLE;
        }

    }
}


/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{
    /* USER CODE BEGIN CAN2_Init 0 */
    CAN_FilterConfTypeDef  sFilterConfig;
    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    hcan2.Instance = CAN2;
    hcan2.pTxMsg = &TxMessage;
    hcan2.pRxMsg = &RxMessage;
    hcan2.Init.Prescaler = 7;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SJW = CAN_SJW_1TQ;
    hcan2.Init.BS1 = CAN_BS1_10TQ;
    hcan2.Init.BS2 = CAN_BS2_1TQ;
    hcan2.Init.TTCM = DISABLE;
    hcan2.Init.ABOM = ENABLE;
    hcan2.Init.AWUM = ENABLE;
    hcan2.Init.NART = DISABLE ;  //DISABLE
    hcan2.Init.RFLM = DISABLE;
    hcan2.Init.TXFP = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN2_Init 2 */
    sFilterConfig.FilterNumber = 14;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

//	sFilterConfig.FilterIdHigh =0X0000; //Ҫ���˵�ID��λ
//	sFilterConfig.FilterIdLow =0X0000;//Ҫ���˵�ID��λ
//	sFilterConfig.FilterMaskIdHigh = 0X0000;
//	sFilterConfig.FilterMaskIdLow = 0X0000;

    sFilterConfig.FilterIdHigh         = (((uint32_t)0x17<<21)&0xFFFF0000)>>16;        /* Ҫ���˵�ID��λ */
    sFilterConfig.FilterIdLow          = (((uint32_t)0x17<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF; /* Ҫ���˵�ID��λ */
    sFilterConfig.FilterMaskIdHigh     = 0xFF1F;      /* ��������16λÿλ����ƥ�� */
    sFilterConfig.FilterMaskIdLow      = 0xFFFF;      /* ��������16λÿλ����ƥ�� */
    sFilterConfig.FilterFIFOAssignment=CAN_FilterFIFO0;
    sFilterConfig.BankNumber = 14;
    sFilterConfig.FilterActivation=ENABLE;
    if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
    {
        Error_Handler();
    }
    sFilterConfig.FilterNumber = 15;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x602<<5;        /* Ҫ���˵�ID��λ */
    sFilterConfig.FilterIdLow          = 0x0000;         /* Ҫ���˵�ID��λ */
    sFilterConfig.FilterMaskIdHigh     = 0x602<<5;      /* ��������16λÿλ����ƥ�� */
    sFilterConfig.FilterMaskIdLow      = 0xFFFF;      /* ��������16λÿλ����ƥ�� */

    sFilterConfig.FilterFIFOAssignment=CAN_FilterFIFO0;
    sFilterConfig.BankNumber = 14;
    sFilterConfig.FilterActivation=ENABLE;
    if(HAL_CAN_ConfigFilter(&hcan2,&sFilterConfig)!=HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
//	__HAL_RCC_GPIOD_CLK_ENABLE();
//
    GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
  
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */
/*------------------------------------------------
Function:���뱸�ݣ�Ĭ�� 0x8020000Ϊ���д������򣬡�0x8040000Ϊ���ݴ�������
Input   :No
Output  :No
Explain :App_addr�����д�������  back_up_addr�����뱸������ byte_num�������ֽ��� pack_num�����ݰ����� ֻ�ܰ�����������
------------------------------------------------*/
uint8_t APP_Backup(uint32_t App_addr,uint32_t back_up_addr,uint32_t byte_num,uint8_t pack_num)		//���ݳ���
{
    uint16_t i,j;
    if(Erase_Sector(back_up_addr,byte_num)==TRUE) //��������
    {
        for(j=0; j<pack_num; j++)//������  ÿ��д1K������ ֱ��pack_num   128K ����
        {
            for(i=0; i<PACK_SIZE; i++)//ÿ�� 1024���ֽ�
            {
                flash_buf[i] = *(uint8_t*)App_addr; //��Ҫ���ݵ�������ʼ��ַ
                App_addr++;
            }
            if(Flash_Write(back_up_addr+j*PACK_SIZE,flash_buf,PACK_SIZE)==FALSE)//���ݵ�ַ��back_up_addr��ַ��ʼ����ÿ��д1K������
            {
                SendErrorCode(0x02,0x01);
                return FALSE;
            }
        }
    }
    else
    {
        SendErrorCode(0x01,0x01);
    }
    return TRUE;
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t return_data[8],i,tempsize;
    if(hcan->Instance ==CAN2)
    {
        Message_Flag=1;
        OTA_Time=0;                          /*�յ�CAN����OTAtime���㣬�˴���Ҫ����CAN��������ܶ����ݴ˴��ж��Ƿ���Ч*/
        if(RxMessage.StdId == 0x10)          /*CAN ID:0x10 data:e0 xx ������λ������ָ��*/
        {
            if(RxMessage.Data[0]==0xe0)
            {
                SoftReset();                                           /*����������ָ���Ѿ�����*/
            }
            else if(RxMessage.Data[0]==0xe1&&RxMessage.Data[1]==0x01)  /*���¹̼���ָ���Ѿ�����*/
            {
                return_data[0] = 0xe2;
                return_data[1] = 0x01;
                CANx_SendNormalData(&hcan2,0x20,return_data,2);        /*����bootloader���������Զ����� CANID 0x20 data: e2 01*/
            }
            else if(RxMessage.Data[0]==0xe3)                           /*��λ����Ҫ���ͱ�������*/
            {
                SysState = Copying;                                    /*��ʼ����*/
            }
            else if(RxMessage.Data[0]==0xe5)		                       /*����ָ�� CANID 0x10 data: e5 01*/
            {
                return_data[0] = 0xe6;
                return_data[1] = 0x01;
                CANx_SendNormalData(&hcan2,0x20,return_data,2);
//                SysState = Rollback;                                   /*������ˣ����ε���λ�����ͻ���ָ��*/
            }
        }
        else if(RxMessage.StdId ==0x11)                                 /*CANID : 0x11��ͷ����Ϊ�� ��λ�����͵Ĵ�������*/
        {
            if(OTA.SizeOfArray == 1024)                                 /*ÿ��1024���ֽڽ���*/
            {
                OTA.SizeOfArray = 0;
            }
            OTA.SizeOfArray+=RxMessage.DLC;                            /*�յ������ݳ���*/
//          tempsize = (byte_cnt*8<OTA.SizeOfArray)?8:(OTA.SizeOfArray-8*(byte_cnt-1));       // //ÿ��8���ֽ�����һ�Σ�10 20 30 40 50 60 70 80����һ��
            if(byte_cnt*8<OTA.SizeOfArray)                             /*���С��128֡���ݣ���˵��flash_bufû��д��1024������*/
            {
                tempsize = 8;
            }
            else
            {
                tempsize = OTA.SizeOfArray-8*(byte_cnt-1);            /*���1kb���ݷ���û�з���У���룬��ôִ�д˴����˴�ִ�б����յ����ݶ���������ʧ��*/
            }
            for(i=0; i<tempsize; i++)                                 /*��flash_buf��д��ղ��յ���8���ֽڵ�����*/
            {
                flash_buf[8*byte_cnt+i] = RxMessage.Data[i];
            }
			   if(pack_cnt==1&&byte_cnt==0x40)
			{
				if(RxMessage.Data[0]==0x48&&RxMessage.Data[1]==0x34&&RxMessage.Data[2]==0x41&&RxMessage.Data[3]==0x41&&RxMessage.Data[4]==0x33&&RxMessage.Data[5]==0x30)
				{
				   Version_Flag=1;
				}
			}

            for( ii=0,ii<10000; ii++;);                               /*iiȫ�ֱ���������ΪʲôҪ�ȴ���*/

            byte_cnt++;                                               /* �յ�һ֡���� byte_cnt++ */
            return_data[0] = pack_cnt;                                /* 1k���ݰ�������������128֡ pack_cnt++��byte_cnt���� */
            return_data[1] = byte_cnt;
            CANx_SendNormalData(&hcan2,0x21,return_data,2);           /*�������յ������ݰ���Ϣ����*/
            Receive_cnt000++;                                         /*���Ա���*/

        }
        else if(RxMessage.StdId == 0x12)                              /*CANID: 0x12 data: xx xx ����xxΪ�յ���У����*/
        {
            if(RxMessage.DLC==2&&crc16(flash_buf,OTA.SizeOfArray)==(RxMessage.Data[0]<<8)+RxMessage.Data[1])      /*���㱾��1k���ݵ�У���룬������յ���У����һ�£���ʼflashд����*/
            {   /*ע�⣺дflash������ɺ�Żظ���CANID:0x21 data: pack_cnt byte_cnt*/
                OTA.WriteFlag=1;
            }
            else
            {   /*У��ʧ��*/
                SendErrorCode(0x03,pack_cnt);
            }
        }
        else if(RxMessage.StdId == 0x13)          //��λ�����͵Ĺ̼��汾,û���жϹ̼������ݣ�ֻҪ��idΪ0x13�����ݾ����
        {
            OTA.Code_Ver = (RxMessage.Data[3]<<24)+(RxMessage.Data[2]<<16)+(RxMessage.Data[1]<<8)+RxMessage.Data[0]; //�汾��
            OTA.ByteNum = (RxMessage.Data[7]<<24)+(RxMessage.Data[6]<<16)+(RxMessage.Data[5]<<8)+RxMessage.Data[4];  //�̼��ֽ���
            OTA.PackNum = (OTA.ByteNum%PACK_SIZE==0) ? OTA.ByteNum/PACK_SIZE:OTA.ByteNum/PACK_SIZE+1;                //������ٰ�
            OTA.version_flag=1;                                                               /*������ɱ�־�����ڷ��͵İ汾���Լ��̼��ֽ��������ж�*/
//		      if(pack_cnt!=OTA.PackNum+1&&pack_cnt!=1)                  //У�����Ľ��յ������ݰ���ʵ����λ�����͵�pack���ݰ�������һ��
//				  {
//					  SendErrorCode(0x07,0x01);
//				  }
        }
        else if(RxMessage.StdId == 0x14)                   /*����ָ�CANID: 0x14 data: xx xx*/
        {
            OTA.EraseFlag=1;
        }
        else if(RxMessage.StdId == 0x602)                   /*Ϊ�˷�ֹboot���������жϵ磬boot��������ת�ȴ�����ָ��ظ� CANID: 0x20 data: */
        {
            if(RxMessage.Data[0]==0x2F&&RxMessage.Data[1]==0x0B&&RxMessage.Data[2]==0x20&&RxMessage.Data[3]==0x00&&RxMessage.Data[4]==0x01)
            {
                buf_send[0] = 0xe2;
                buf_send[1] = 0x01;
                CANx_SendNormalData(&hcan2,0x20,buf_send,2);
            }
        }
    }
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);

}


void CANx_SendNormalData(CAN_HandleTypeDef *hcanX,uint16_t ID,uint8_t *pData, uint16_t Len)
{
    uint8_t i=0;
    hcan2.pTxMsg->StdId = ID;
    hcan2.pTxMsg->IDE = CAN_ID_STD;
    hcan2.pTxMsg->RTR = CAN_RTR_DATA;
    hcan2.pTxMsg->ExtId=0xFFF;
    hcan2.pTxMsg->DLC=Len;
    for(i=0; i<Len; i++)
    {
        hcan2.pTxMsg->Data[i]=pData[i];
    }
    Can_Err = HAL_CAN_Transmit(&hcan2,100);
}
void SoftReset(void)
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}
/*------------------------------------------------
* @function :1S����һ�δ���֡
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/3/20
------------------------------------------------*/
void SendErrorCode(uint8_t ErrorCode,uint8_t CodeNum)
{
    static uint32_t last_tick;
    if(HAL_GetTick()-last_tick>2000)
    {
        uint8_t CodeArray[2];
        CodeArray[0] = ErrorCode;
        CodeArray[1] = CodeNum;
        CANx_SendNormalData(&hcan2,0xff,CodeArray,2);
        last_tick = HAL_GetTick();
    }
}
uint8_t Jump_to_APP(uint32_t APP_addr)
{
    SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;     //����7/23

    if(((*(__IO uint32_t*)APP_addr) & 0x2FFE0000 )== 0x20000000)
    {
        JumpAddress = *(__IO uint32_t*) (APP_addr + 4);
        Jump_To_Application = (pFunction) JumpAddress;
        __set_PRIMASK(1);
        __disable_irq();
        HAL_NVIC_DisableIRQ(SysTick_IRQn);
        HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);   //����

        __set_MSP(*(__IO uint32_t*) APP_addr);
        Jump_To_Application();
    }
    else
        return FALSE;
    return FALSE;
}
static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}
uint8_t WriteData_toFlash(uint8_t *data,uint32_t len,uint32_t address)
{
    uint32_t i=0;
    for(i=0; i<len; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)address, *data) == HAL_OK)
        {
            address = address+1;
            data = data+1;
        }
        else
        {
            return FALSE;
        }
    }
    return TRUE;
}
uint8_t WriteUint32_tData_toFlash(uint32_t *data,uint32_t len,uint32_t address)
{
	HAL_FLASH_Unlock();
    uint32_t i=0;
    for(i=0; i<len; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)address, *data) == HAL_OK)
        {
            address = address+1;
            data = data+1;
        }
        else
        {
            return FALSE;
        }
    }
	HAL_FLASH_Lock();
    return TRUE;
}
uint8_t Write_Int_Data(uint32_t *data,uint32_t len,uint32_t address)
{
    uint32_t i=0;
    for(i=0; i<len; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)address, *data) == HAL_OK)
        {
            address = address+4;
            data = data+1;
        }
        else
        {
            return FALSE;
        }
    }
    return TRUE;
}

uint32_t FirstSector = 0, NbOfSectors = 0,SectorError = 0,MemoryProgramStatus = 0;
uint8_t Flash_Write(uint32_t Address,uint8_t *data,uint32_t data_len)
{
    uint32_t i;
    uint8_t tempdata;
    HAL_FLASH_Unlock();
    if(WriteData_toFlash(data,data_len,Address)==FALSE)
    {

    }
    HAL_FLASH_Lock();
    MemoryProgramStatus = 0;
    for(i=0; i<data_len; i++)
    {
        tempdata = *(__IO uint8_t*)(Address+i);
        if(tempdata !=data[i])
        {
            MemoryProgramStatus++;
        }
    }
    if(MemoryProgramStatus)
        return FALSE;
    else
        return TRUE;
}

uint8_t Erase_Sector(uint32_t StartAddress, uint32_t data_len)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    HAL_FLASH_Unlock();
    FirstSector = GetSector(StartAddress);
    NbOfSectors = 1;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;//��������
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FirstSector;
    EraseInitStruct.NbSectors = NbOfSectors;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return FALSE;
    }
    else
    {
        HAL_FLASH_Lock();
        return TRUE;
    }
}
uint8_t Flash_Write_long(uint32_t Address,uint32_t *data,uint32_t data_len)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t i,temp32;

    HAL_FLASH_Unlock();
    FirstSector = GetSector(Address);
    NbOfSectors = 1;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;//��������
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FirstSector;
    EraseInitStruct.NbSectors = NbOfSectors;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
    {

    }
    if(Write_Int_Data(data,data_len,Address)==FALSE)
    {

    }
    HAL_FLASH_Lock();
    MemoryProgramStatus = 0;
    for(i=0; i<data_len; i++)
    {
        temp32 = *(__IO uint32_t*)(Address+i*4);
        if(temp32 !=data[i])
        {
            MemoryProgramStatus++;
        }
    }
    if(MemoryProgramStatus)
        return FALSE;
    else
        return TRUE;
}
void Flash_Read(uint32_t addresss,uint32_t *pBuffer,uint32_t bufsize)			//��ȡflash����
{
    uint32_t i;
    for(i=0; i<bufsize; i++)
    {
        pBuffer[i] = *(long*)(addresss);
        addresss +=4;
    }
}
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

int fgetc(FILE * f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart3,&ch, 1, 0xffff);
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
/*----------------------------
*Fuction������ĳ����ַ���ڵ� flash ����
*Explain��No
----------------------------*/
int FlashEraseSector( uint32_t f_sector, uint8_t f_num_sectors )
{
    int ret = 1;
    uint32_t SectorError = 0;
    FLASH_EraseInitTypeDef eraseInit = {0};


    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.NbSectors = f_num_sectors;
    eraseInit.Sector = f_sector;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    /*�����ٽ���*/
//    __disable_irq();

    /*1����*/
    HAL_FLASH_Unlock();

    /*2��������*/
    HAL_FLASHEx_Erase(&eraseInit, &SectorError);

    /*3����*/
    HAL_FLASH_Lock();

    /*�˳��ٽ���*/
//    __enable_irq();

    if( 0xFFFFFFFFU != SectorError )
    {
        ret = -1;
    }
    return ret;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
