#include "mc_config.h"
#include "stdio.h"
#include "string.h"
#define	DEBUG_TST_NUM			31
long FlashArray[600];
long glDebugTestD[DEBUG_TST_NUM] = {0};
typedef __IO uint32_t  vu32;
/*----------------------------
*Fuction��������˹����д��FLASH
*Explain��32 λ����
�����������ʹ����ʽ:ID+ADDR+bAxisRelated+sMaxArrayIndex+&date
----------------------------*/
u8 Atlas_Write_Flash(u32 WriteAddr,long *array)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    static u16 i = 0,j = 0;
    static long Lenth = 0,writeNum_cnt = 0;				//���ݳ��Ⱥ�����λ��ָ��

    u32 addrx=0,endaddr=0;
    u32 SectorError=0;

    Lenth = 0;
    writeNum_cnt = 0;
    for(i = 0; i<ParaNum; i++)							//��һ�α���:�ҳ����ݳ���
    {
        if(PARAMETER[i].stAttributes.bSaveToFlash==1)	//����������FLASH
        {
            Lenth += (PARAMETER[i].sMaxArrayIndex+4);	//ID+ADDR+���������+��������+DATE����
        }
    }
    Lenth++;								    		//Ƕ�����ݳ�����Ϣ
    *(array+0) = Lenth-1;								//��4Byte���ڴ�����ݳ���,������Ч���ݳ���
    glDebugTestD[5] = Lenth;
    writeNum_cnt++;										//����ָ������
    for(i = 0; i<ParaNum; i++)							//�ڶ��α���:��������Buffer
    {
        if(PARAMETER[i].stAttributes.bSaveToFlash==1)	//����������FLASH
        {
            *(array+writeNum_cnt) = PARAMETER[i].sParID;//��װ���ַ���Ե���Ϣ
            *(array+writeNum_cnt+1) = PARAMETER[i].sAddress;
            *(array+writeNum_cnt+2) = PARAMETER[i].stAttributes.bAxisRelated;
            *(array+writeNum_cnt+3) = PARAMETER[i].sMaxArrayIndex;
            writeNum_cnt+=4;
            for(j=0; j<PARAMETER[i].sMaxArrayIndex; j++)	//װ������[�Դ�װ������ݽ���У�飬���У��ʧ�ܣ���װ��Ĭ��ֵ]
            {
                if((*(PARAMETER[i].lpParam+j) >= PARAMETER[i].lMinValue)&&((*(PARAMETER[i].lpParam+j) <= PARAMETER[i].lMaxValue)))//У��ɹ�
                {
                    *(array+writeNum_cnt+j) = *(PARAMETER[i].lpParam+j);
                }
//				else//У�鲻�ɹ���װ��ȱʡֵ
//				{
//					*(array+writeNum_cnt+j) = PARAMETER[i].lDefaultValue;			//TODO: IF check error,then disrgard it . no concer it.
//				}
            }
            writeNum_cnt += PARAMETER[i].sMaxArrayIndex;//������������ͣ��������ȾͰ��������С����
        }
    }
    if(Lenth != writeNum_cnt) return FLASH_ERROR;       //У������װ��������
    /*-----FLASHд��-----*/
    if(WriteAddr<STM32F4x_Flash_Base||WriteAddr%4)  return FLASH_ERROR;		//�Ƿ���ַ����д��ĵ�ַ����4�ı���
    HAL_FLASH_Unlock(); 													//����
    addrx=WriteAddr;														//�׵�ַ
    endaddr=WriteAddr+Lenth*4;												//ĩ��ַ
    if(addrx<0X1FFF0000)													//�ж��׵�ַ�Ƿ�������������
    {
        while(addrx<endaddr) 												//�Է� FFFFFFFF �ĵط�,�Ȳ���
        {
            if(Flash_ReadWord(addrx)!=0XFFFFFFFF)							//�з� 0XFFFFFFFF �ĵط�,Ҫ�����������
            {
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS; 			//������������
                FlashEraseInit.Sector=Flash_GetSector(addrx); 				//Ҫ����������
                FlashEraseInit.NbSectors=1; 								//һ��ֻ����һ������
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;			//VCC=2.7~3.6V
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) //�Ը��������в��������ؽ��
                {
                    printf("Flash_Erase error\r\n");
                    break;													//����������
                }
            } else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); 					//�ȴ��ϴβ������(�ȴ�ʱ��)
        }
    }
    else
    {
        ;//TODO:If there is some error, do dealwith... WSQ 21091017
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME); 			    //�ȴ��ϴβ������
    if(FlashStatus==HAL_OK)
    {
        while(WriteAddr<endaddr)											//д����
        {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*array)!=HAL_OK)//д������
            {
                glDebugTestD[11] ++;
                break; 														//д���쳣				//TODO: when there is some error, then Give Warning or Error conflt... to GUI
            }
            WriteAddr+=4;													//32/8(bit)
            array++;														//д������ָ������
        }
    }
    HAL_FLASH_Lock(); 														//����

    memset(array,0,sizeof(*array));
    return FLASH_SUCCESS;
}
/*------------------------------------------------
Function:��ȡFLASH���ݸ�ֵ�� RW����
Input   :FLASH��ʼ��ַ���м仺���������
Output  :FLASH_SUCCESS
Explain :�ڳ�����Χд��Ĭ��ֵelse���ֿ��ܻ��������
------------------------------------------------*/
u8 Atlas_Read_Flash(u32 WriteAddr,long *array)
{
    static u16 i = 0,j = 0,Lencnt = 0;
    static long Length = 0,ReadNum = 0;

    Length = 0;
    ReadNum = 0;
    Length = Flash_ReadWord(WriteAddr);				//������ַ��4byte(�ϴ�д�����ݳ���)
    WriteAddr+=4;									//��ַ����

    for(i=0; i<Length; i++)							//��������
    {
        *(array+i) = Flash_ReadWord(WriteAddr);//�����е����ݶ���������arrayָ���Ӧ��һά�����У��������СҪ����������������С
        WriteAddr+=4;
    }
    for(i=0; i<ParaNum; i++)							// ÿһ��������Ҫ����һ�������
    {
        for(Lencnt=0; Lencnt<Length; Lencnt++)		//����arrayָ���Ӧ��һά����
        {
            if((*(array+Lencnt)==PARAMETER[i].sParID)&&(*(array+Lencnt+1)==PARAMETER[i].sAddress)) //���ID��sAddress��ƥ���ǾͿ��Խ��и�ֵ
            {
                for(j=0; j<*(array+Lencnt+3); j++)				//�������������ζ�����ӳ���ַ
                {
                    if((*(array+Lencnt+4+j)>=PARAMETER[i].lMinValue)&&((*(array+Lencnt+4+j)<=PARAMETER[i].lMaxValue)))//�жϵ�i�����ݵ�ȡֵ��Χ
                    {
                        *(PARAMETER[i].lpParam+j) = *(array+Lencnt+4+j);
                    }
                    else
                    {
                        *(PARAMETER[i].lpParam+j) = PARAMETER[i].lDefaultValue;
                    }
                }
                ReadNum++;
            }
        }
    }
    memset(array,0,sizeof(*array));

    return FLASH_SUCCESS;
}
/*------------------------------------------------
Function:��ȡ����������Ĭ�ϲ���
Input   :No
Output  :No
Explain :��ʼ��������
------------------------------------------------*/
void Atlas_LoadDefaultPar(void)
{
    static u16 i = 0,j = 0;

    for(i=0; i<ParaNum; i++)						// Look up table parameters ciclely
    {
        if (PARAMETER[i].sAddress > 0x2000 && PARAMETER[i].sAddress < 0x2FFF)
        {
//		if (PARAMETER[i].bAxisRelated == AXIS)  //TODO: We should consider AXIS num later.... WSQ 20191017
            if (PARAMETER[i].stAttributes.bIsArray == ARRAY)
            {
                for(j=0; j<PARAMETER[i].sMaxArrayIndex; j++)				//�������������ζ�����ӳ���ַ������ò�����������ô��Ĭ��ֵ��ֵ�������е�ÿ��ֵ
                {
                    *(PARAMETER[i].lpParam+j) = PARAMETER[i].lDefaultValue;
                }
            }
            else
            {
                *(PARAMETER[i].lpParam) = PARAMETER[i].lDefaultValue;
            }
        }
    }
}
/*----------------------------
*Fuction����ȡָ����ַ����
*Explain��32 λ����
*Input	��faddr����ַ
*Output	����Ӧ����
----------------------------*/
u32 Flash_ReadWord(u32 faddr)
{
    return *(vu32*)faddr;
}

/*----------------------------
*Fuction����ȡĳ����ַ���ڵ� flash ����
*Explain��No
*Input	��flash ��ַ
*Output	��0~11,�� addr ���ڵ�����
----------------------------*/
u8 Flash_GetSector(u32 addr)
{
    if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
    else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
    else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
    else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
    else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
    else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
    else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
    else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
    else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
    else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
    else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10;
    else if(addr<ADDR_FLASH_SECTOR_12)return FLASH_SECTOR_11;
    else  return 0;
}

/*----------------------------
*Fuction������ĳ����ַ���ڵ� flash ����
*Explain��No
----------------------------*/
u8 Flash_Erase(u32 WriteAddr,s32 len)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    u32 SectorError=0;
    u32 addrx=0,endaddr=0;

    if(WriteAddr<STM32F4x_Flash_Base||WriteAddr%4) return 1; 				//�Ƿ���ַ����д��ĵ�ַ����4�ı���
    HAL_FLASH_Unlock(); 												//����
    addrx = WriteAddr;
    endaddr = WriteAddr+4*len;
    if(addrx<0X1FFF0000)
    {
        while(addrx<endaddr) 											//ɨ��һ���ϰ�.(�Է� FFFFFFFF �ĵط�,�Ȳ���)
        {
            if(Flash_ReadWord(addrx)!=0XFFFFFFFF)						//�з� 0XFFFFFFFF �ĵط�,Ҫ�����������
            {
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS; 		//������������
                FlashEraseInit.Sector=Flash_GetSector(addrx); 			//Ҫ����������
                FlashEraseInit.NbSectors=1; 							//һ��ֻ����һ������
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;		//VCC=2.7~3.6V
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) //�Ը��������в��������ؽ��
                {
                    break;												//����������
                }
            } else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); 				//�ȴ��ϴβ������(�ȴ�ʱ��)
        }
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME); 			//�ȴ��ϴβ������
    HAL_FLASH_Lock(); //����
    return FlashStatus;
}
/*----------------------------
*Fuction����ָ����ַ��ʼд��ָ�����ȵ�����
*Explain��1.��Ϊ STM32F4 ������ʵ��̫��,û�취���ر�����������,���Ա�����
					д��ַ����� 0XFF,��ô���Ȳ������������Ҳ�������������.����д
					�� 0XFF �ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������û
					����Ҫ����,��������������Ȳ�����,Ȼ����������д��
					2.�ú����� OTP ����Ҳ��Ч����������д OTP ��OTP �����ַ��Χ:0X1FFF7800~0X1FFF7A0F(��� 16 �ֽ����� OTP ���ݿ�����)
*Input	��WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ 4 �ı���)   pBuffer:����ָ��  NumToWrite:��(32 λ)��(����Ҫд��� 32 λ���ݵĸ���)
*Output	��No
----------------------------*/
void Flash_WriteData(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    u32 SectorError=0;
    u32 addrx=0,endaddr=0;

    if(WriteAddr<STM32F4x_Flash_Base||WriteAddr%4) return; 				//�Ƿ���ַ����д��ĵ�ַ����4�ı���
    HAL_FLASH_Unlock(); 												//����
    addrx=WriteAddr; 													  //д����ʼ��ַ
    endaddr=WriteAddr+NumToWrite*4; 									//д�������ַ
    if(addrx<0X1FFF0000)
    {
        while(addrx<endaddr) 											//ɨ��һ���ϰ�.(�Է� FFFFFFFF �ĵط�,�Ȳ���)
        {
            if(Flash_ReadWord(addrx)!=0XFFFFFFFF)						//�з� 0XFFFFFFFF �ĵط�,Ҫ�����������
            {
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS; 		//������������
                FlashEraseInit.Sector=Flash_GetSector(addrx); 			//Ҫ����������
                FlashEraseInit.NbSectors=1; 							//һ��ֻ����һ������
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;		//VCC=2.7~3.6V
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) //�Ը��������в��������ؽ��
                {
                    break;												//����������
                }
            } else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); 				//�ȴ��ϴβ������(�ȴ�ʱ��)
        }
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME); 			//�ȴ��ϴβ������
    if(FlashStatus==HAL_OK)
    {
        while(WriteAddr<endaddr)										//д����
        {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*pBuffer)!=HAL_OK)//д������
            {
                break; 													//д���쳣
            }
            WriteAddr+=4;												//32bit/8bit
            pBuffer++;													//д������ָ������
        }
    }
    HAL_FLASH_Lock(); //����
}

/*----------------------------
*Fuction����ָ����ַ��ʼ����ָ�����ȵ�����
*Explain��32 λ
*Input	��ReadAddr:��ʼ��ַ   pBuffer:����ָ��  NumToRead:��(32 λ)��
*Output	��No
----------------------------*/
void Flash_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)
{
    u32 i;
    for(i=0; i<NumToRead; i++)
    {
        pBuffer[i]=Flash_ReadWord(ReadAddr);		//��ȡ 4 ���ֽ�
        ReadAddr+=4;								//ƫ�� 4 ���ֽ�
    }
}

void LoadFlashParams(void)
{
    // First , load parameters to give default value
    Atlas_Read_Flash(Atlas_Flash_Addr,FlashArray);
//	pGainPar.filterPar.calcFiltEn[A_AXIS] = 1;
}
