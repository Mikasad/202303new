#include "mc_config.h"
#include "stdio.h"
#include "string.h"
#define	DEBUG_TST_NUM			31
long FlashArray[600];
long glDebugTestD[DEBUG_TST_NUM] = {0};
typedef __IO uint32_t  vu32;
/*----------------------------
*Fuction：阿特拉斯参数写入FLASH
*Explain：32 位数据
单个数据类型存入格式:ID+ADDR+bAxisRelated+sMaxArrayIndex+&date
----------------------------*/
u8 Atlas_Write_Flash(u32 WriteAddr,long *array)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    static u16 i = 0,j = 0;
    static long Lenth = 0,writeNum_cnt = 0;				//数据长度和数据位置指针

    u32 addrx=0,endaddr=0;
    u32 SectorError=0;

    Lenth = 0;
    writeNum_cnt = 0;
    for(i = 0; i<ParaNum; i++)							//第一次遍历:找出数据长度
    {
        if(PARAMETER[i].stAttributes.bSaveToFlash==1)	//如果允许存入FLASH
        {
            Lenth += (PARAMETER[i].sMaxArrayIndex+4);	//ID+ADDR+轴相关属性+数组索引+DATE长度
        }
    }
    Lenth++;								    		//嵌入数据长度信息
    *(array+0) = Lenth-1;								//首4Byte用于存放数据长度,存入有效数据长度
    glDebugTestD[5] = Lenth;
    writeNum_cnt++;										//数据指针增加
    for(i = 0; i<ParaNum; i++)							//第二次遍历:填入数据Buffer
    {
        if(PARAMETER[i].stAttributes.bSaveToFlash==1)	//如果允许存入FLASH
        {
            *(array+writeNum_cnt) = PARAMETER[i].sParID;//先装入地址属性等信息
            *(array+writeNum_cnt+1) = PARAMETER[i].sAddress;
            *(array+writeNum_cnt+2) = PARAMETER[i].stAttributes.bAxisRelated;
            *(array+writeNum_cnt+3) = PARAMETER[i].sMaxArrayIndex;
            writeNum_cnt+=4;
            for(j=0; j<PARAMETER[i].sMaxArrayIndex; j++)	//装入数据[对待装入的数据进行校验，如果校验失败，则装入默认值]
            {
                if((*(PARAMETER[i].lpParam+j) >= PARAMETER[i].lMinValue)&&((*(PARAMETER[i].lpParam+j) <= PARAMETER[i].lMaxValue)))//校验成功
                {
                    *(array+writeNum_cnt+j) = *(PARAMETER[i].lpParam+j);
                }
//				else//校验不成功，装入缺省值
//				{
//					*(array+writeNum_cnt+j) = PARAMETER[i].lDefaultValue;			//TODO: IF check error,then disrgard it . no concer it.
//				}
            }
            writeNum_cnt += PARAMETER[i].sMaxArrayIndex;//如果是数组类型，计数长度就按照数组大小增加
        }
    }
    if(Lenth != writeNum_cnt) return FLASH_ERROR;       //校验数据装填完成情况
    /*-----FLASH写入-----*/
    if(WriteAddr<STM32F4x_Flash_Base||WriteAddr%4)  return FLASH_ERROR;		//非法地址或者写入的地址不是4的倍数
    HAL_FLASH_Unlock(); 													//解锁
    addrx=WriteAddr;														//首地址
    endaddr=WriteAddr+Lenth*4;												//末地址
    if(addrx<0X1FFF0000)													//判断首地址是否属于正常扇区
    {
        while(addrx<endaddr) 												//对非 FFFFFFFF 的地方,先擦除
        {
            if(Flash_ReadWord(addrx)!=0XFFFFFFFF)							//有非 0XFFFFFFFF 的地方,要擦除这个扇区
            {
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS; 			//类型扇区擦除
                FlashEraseInit.Sector=Flash_GetSector(addrx); 				//要擦除的扇区
                FlashEraseInit.NbSectors=1; 								//一次只擦除一个扇区
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;			//VCC=2.7~3.6V
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) //对该扇区进行擦除并返回结果
                {
                    printf("Flash_Erase error\r\n");
                    break;													//发生错误了
                }
            } else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); 					//等待上次操作完成(等待时间)
        }
    }
    else
    {
        ;//TODO:If there is some error, do dealwith... WSQ 21091017
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME); 			    //等待上次操作完成
    if(FlashStatus==HAL_OK)
    {
        while(WriteAddr<endaddr)											//写数据
        {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*array)!=HAL_OK)//写入数据
            {
                glDebugTestD[11] ++;
                break; 														//写入异常				//TODO: when there is some error, then Give Warning or Error conflt... to GUI
            }
            WriteAddr+=4;													//32/8(bit)
            array++;														//写入数据指针自增
        }
    }
    HAL_FLASH_Lock(); 														//上锁

    memset(array,0,sizeof(*array));
    return FLASH_SUCCESS;
}
/*------------------------------------------------
Function:读取FLASH数据赋值给 RW参数
Input   :FLASH起始地址，中间缓存数组变量
Output  :FLASH_SUCCESS
Explain :在超过范围写入默认值else部分可能会出现问题
------------------------------------------------*/
u8 Atlas_Read_Flash(u32 WriteAddr,long *array)
{
    static u16 i = 0,j = 0,Lencnt = 0;
    static long Length = 0,ReadNum = 0;

    Length = 0;
    ReadNum = 0;
    Length = Flash_ReadWord(WriteAddr);				//读出地址首4byte(上次写入数据长度)
    WriteAddr+=4;									//地址自增

    for(i=0; i<Length; i++)							//读出数据
    {
        *(array+i) = Flash_ReadWord(WriteAddr);//把所有的数据读出来放在array指针对应的一维数组中，该数组大小要满足存入的数据量大小
        WriteAddr+=4;
    }
    for(i=0; i<ParaNum; i++)							// 每一个参数都要遍历一遍参数表
    {
        for(Lencnt=0; Lencnt<Length; Lencnt++)		//遍历array指针对应的一维数组
        {
            if((*(array+Lencnt)==PARAMETER[i].sParID)&&(*(array+Lencnt+1)==PARAMETER[i].sAddress)) //如果ID与sAddress都匹配那就可以进行赋值
            {
                for(j=0; j<*(array+Lencnt+3); j++)				//按数组索引依次读出到映射地址
                {
                    if((*(array+Lencnt+4+j)>=PARAMETER[i].lMinValue)&&((*(array+Lencnt+4+j)<=PARAMETER[i].lMaxValue)))//判断第i个数据的取值范围
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
Function:读取驱动器所有默认参数
Input   :No
Output  :No
Explain :初始化最后调用
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
                for(j=0; j<PARAMETER[i].sMaxArrayIndex; j++)				//按数组索引依次读出到映射地址，如果该参数是数组那么将默认值赋值给数组中的每个值
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
*Fuction：读取指定地址的字
*Explain：32 位数据
*Input	：faddr读地址
*Output	：对应数据
----------------------------*/
u32 Flash_ReadWord(u32 faddr)
{
    return *(vu32*)faddr;
}

/*----------------------------
*Fuction：获取某个地址所在的 flash 扇区
*Explain：No
*Input	：flash 地址
*Output	：0~11,即 addr 所在的扇区
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
*Fuction：擦除某个地址所在的 flash 扇区
*Explain：No
----------------------------*/
u8 Flash_Erase(u32 WriteAddr,s32 len)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    u32 SectorError=0;
    u32 addrx=0,endaddr=0;

    if(WriteAddr<STM32F4x_Flash_Base||WriteAddr%4) return 1; 				//非法地址或者写入的地址不是4的倍数
    HAL_FLASH_Unlock(); 												//解锁
    addrx = WriteAddr;
    endaddr = WriteAddr+4*len;
    if(addrx<0X1FFF0000)
    {
        while(addrx<endaddr) 											//扫清一切障碍.(对非 FFFFFFFF 的地方,先擦除)
        {
            if(Flash_ReadWord(addrx)!=0XFFFFFFFF)						//有非 0XFFFFFFFF 的地方,要擦除这个扇区
            {
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS; 		//类型扇区擦除
                FlashEraseInit.Sector=Flash_GetSector(addrx); 			//要擦除的扇区
                FlashEraseInit.NbSectors=1; 							//一次只擦除一个扇区
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;		//VCC=2.7~3.6V
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) //对该扇区进行擦除并返回结果
                {
                    break;												//发生错误了
                }
            } else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); 				//等待上次操作完成(等待时间)
        }
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME); 			//等待上次操作完成
    HAL_FLASH_Lock(); //上锁
    return FlashStatus;
}
/*----------------------------
*Fuction：从指定地址开始写入指定长度的数据
*Explain：1.因为 STM32F4 的扇区实在太大,没办法本地保存扇区数据,所以本函数
					写地址如果非 0XFF,那么会先擦除整个扇区且不保存扇区数据.所以写
					非 0XFF 的地址,将导致整个扇区数据丢失.建议写之前确保扇区里没
					有重要数据,最好是整个扇区先擦除了,然后慢慢往后写。
					2.该函数对 OTP 区域也有效，可以用来写 OTP 区OTP 区域地址范围:0X1FFF7800~0X1FFF7A0F(最后 16 字节用于 OTP 数据块锁定)
*Input	：WriteAddr:起始地址(此地址必须为 4 的倍数)   pBuffer:数据指针  NumToWrite:字(32 位)数(就是要写入的 32 位数据的个数)
*Output	：No
----------------------------*/
void Flash_WriteData(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    u32 SectorError=0;
    u32 addrx=0,endaddr=0;

    if(WriteAddr<STM32F4x_Flash_Base||WriteAddr%4) return; 				//非法地址或者写入的地址不是4的倍数
    HAL_FLASH_Unlock(); 												//解锁
    addrx=WriteAddr; 													  //写入起始地址
    endaddr=WriteAddr+NumToWrite*4; 									//写入结束地址
    if(addrx<0X1FFF0000)
    {
        while(addrx<endaddr) 											//扫清一切障碍.(对非 FFFFFFFF 的地方,先擦除)
        {
            if(Flash_ReadWord(addrx)!=0XFFFFFFFF)						//有非 0XFFFFFFFF 的地方,要擦除这个扇区
            {
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS; 		//类型扇区擦除
                FlashEraseInit.Sector=Flash_GetSector(addrx); 			//要擦除的扇区
                FlashEraseInit.NbSectors=1; 							//一次只擦除一个扇区
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;		//VCC=2.7~3.6V
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) //对该扇区进行擦除并返回结果
                {
                    break;												//发生错误了
                }
            } else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); 				//等待上次操作完成(等待时间)
        }
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME); 			//等待上次操作完成
    if(FlashStatus==HAL_OK)
    {
        while(WriteAddr<endaddr)										//写数据
        {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*pBuffer)!=HAL_OK)//写入数据
            {
                break; 													//写入异常
            }
            WriteAddr+=4;												//32bit/8bit
            pBuffer++;													//写入数据指针自增
        }
    }
    HAL_FLASH_Lock(); //上锁
}

/*----------------------------
*Fuction：从指定地址开始读出指定长度的数据
*Explain：32 位
*Input	：ReadAddr:起始地址   pBuffer:数据指针  NumToRead:字(32 位)数
*Output	：No
----------------------------*/
void Flash_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)
{
    u32 i;
    for(i=0; i<NumToRead; i++)
    {
        pBuffer[i]=Flash_ReadWord(ReadAddr);		//读取 4 个字节
        ReadAddr+=4;								//偏移 4 个字节
    }
}

void LoadFlashParams(void)
{
    // First , load parameters to give default value
    Atlas_Read_Flash(Atlas_Flash_Addr,FlashArray);
//	pGainPar.filterPar.calcFiltEn[A_AXIS] = 1;
}
