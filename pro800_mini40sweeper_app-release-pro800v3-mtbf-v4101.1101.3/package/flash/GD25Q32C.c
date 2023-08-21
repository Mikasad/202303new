#include "GD25Q32C.h"
#include "hc_spi.h"


/*
GD25Q16C：
注意擦除用时较长，读写操作需要等待擦除完成
该芯片的擦写次数最少100000次。即100K cycle
对于扇区和块的擦除用时，文档中如下记录（手册46页）：

Note:
1. Max Value 4KB tSE with<50K cycles is 150ms and >50K & <100k cycles is 300ms.
2. Max Value 32KB tBE with<50K cycles is 0.3s and >50K & <100k cycles is 0.7s.
3. Max Value 64KB tBE with<50K cycles is 0.5s and >50K & <100k cycles is 0.8s.

擦除/写从接收到擦除指令并CS为高电平开始执行并计时。擦除/写过程中WIP=1，擦除/写完成WIP=0.注意，在擦除完成之前的某个不确定时间，WEL会被清零
也就是说和擦写次数有关

注意擦写操作都需要WEL=1时才能操作。所以在逻辑上，每次的编程操作都需要先读WEL，如果不为1则需要执行写使能并且确认WEL=1再操作。
注意读书节时，如果正在执行擦除、程序或写循环，则拒绝读数据，而且不会影响到正在进行的操作


状态寄存器的值可以在任何时候读取

GD25Q32C：参考上面的，不完全一样




*/

#define DEVICE_SIZE 	0x400000 	// 4M
#define BLOCK_SIZE		0x10000		//64K
#define SECT_SIZE		  0x1000 		//4K
#define PAGE_SIZE		  0x100 		//256


#ifndef HAL_Delay
#define HAL_Delay DDL_DelayMS
#endif

#define	SPI4_CS_ON			BSP_SPI_CS_INACTIVE()
#define	SPI4_CS_OFF			BSP_SPI_CS_ACTIVE()


typedef enum{
	
	COMMAND_PAGE_PROGRAM = 0x02,
	COMMAND_READ_DATA_BYTES = 0x03,
	COMMAND_READ_STATUS_REGISTER = 0x05,
	COMMAND_WRITE_ENABLE = 0x06,
	COMMAND_SECTOR_ERASE = 0x20,
	COMMAND_READ_STATUS_REGISTER_1 = 0x35,
}commandList_t;




//extern SPI_HandleTypeDef hspi4;

//static SemaphoreHandle_t gs_busyMutex;
GD25Q16CInfo_t g_GD25Q16CInfo;
//static uint8_t gs_flashTestCmd = 0;
static uint8_t gs_sectorBuf[SECT_SIZE] = {0};



//------------------================================--------------------------------------
static void EnableWrite(void);
static uint8_t GetStatusLow(void);
static void EraseSector(uint32_t f_addr);


void GD25Q32CCS(uint8_t val)
{
	if (val == 0)
	{
        SPI4_CS_OFF;
//		HAL_Delay(2);
	}
	else if (val == 1)
	{
        SPI4_CS_ON;
//		HAL_Delay(2);
	}
}


void GetID_90(uint8_t *fp_mID, uint8_t *fp_dID)
{
	HAL_StatusTypeDef result;
	uint8_t txData[4] = {0x90,0,0,0};
	uint8_t rxData[2] = {0};
	
	
	GD25Q32CCS(0);
	
	result = hc_spi_trans(txData,4);
	printf("hc_spi_trans:%d\r\n",result);
	memset(txData,0,sizeof(txData));
	
	result = hc_spi_trans_recv(txData, rxData, sizeof(rxData));
	printf("hc_spi_trans:%d\r\n",result);
	*fp_mID = rxData[0];
	*fp_dID = rxData[1];
	
	GD25Q32CCS(1);
    
    (void)result;
}

void GetID_9F(uint8_t *fp_mID, uint8_t *fp_type, uint8_t *fp_capacity)
{

	HAL_StatusTypeDef result;
	uint8_t txData[4] = {0x9F,0,0,0};
	uint8_t rxData[3] = {0};
	
	
	GD25Q32CCS(0);
	
	result = hc_spi_trans(txData,1);
	
	memset(txData,0,sizeof(txData));
	
	result = hc_spi_trans_recv(txData, rxData, sizeof(rxData));

	*fp_mID = rxData[0];
	*fp_type = rxData[1];
	*fp_capacity = rxData[2];
	
	GD25Q32CCS(1);
	
    (void)result;

}

void EnableWrite(void)
{
	HAL_StatusTypeDef result;
	uint8_t txData[4] = {COMMAND_WRITE_ENABLE,0,0,0};
	
	GD25Q32CCS(0);
	result = hc_spi_trans(txData,1);
	GD25Q32CCS(1);
    (void)result;
}





static void EraseSector(uint32_t f_addr)
{
	uint8_t txData[4] = {COMMAND_SECTOR_ERASE,(f_addr >> 16)&0xFF,(f_addr >> 8)&0xFF,(f_addr >> 0)&0xFF};
	HAL_StatusTypeDef result;
	
	GD25Q32CCS(0);
	result = hc_spi_trans(txData,4);
	GD25Q32CCS(1);
    
    (void)result;
}

int EraseFlashSector( uint32_t f_addr )
{
    int ret = 0;
    if( !PrepareToWriteOrErase(PREPARE_WRITE_OR_ERASE_TIMEOUT) )
    {
        EraseSector( f_addr );
        if( WriteOrEraseComplete(SECTOR_ERASE_COMPLETE_TIMEOUT) == 0 )//超时
        {
            ret = 1;
        }
    }
    else
    {
        ret = 2;
    }
    return ret;
}


void ReadDataBytes(uint32_t f_srcAddr, uint8_t *fp_dataBuf, uint32_t f_len)
{
	uint8_t txData[4] = {COMMAND_READ_DATA_BYTES,(f_srcAddr >> 16)&0xFF,(f_srcAddr >> 8)&0xFF,(f_srcAddr >> 0)&0xFF};
	HAL_StatusTypeDef result;
//	uint8_t txData2[100] = {0};

	GD25Q32CCS(0);
	
	result = hc_spi_trans(txData,4);
	
	//result = HAL_SPI_TransmitReceive(&hspi4, txData2, fp_dataBuf, f_len,100);
    hc_spi_recv(fp_dataBuf, f_len);
	
	GD25Q32CCS(1);
    
    (void)result;
}
uint8_t GetStatusLow(void)
{
	uint8_t ret = 0;
	HAL_StatusTypeDef result;
	uint8_t txData[4] = {COMMAND_READ_STATUS_REGISTER,0,0,0};

	GD25Q32CCS(0);

	result = hc_spi_trans(txData,1);
	
	memset(txData,0,sizeof(txData));
	
	result = hc_spi_trans_recv(txData, &ret, 1);

	GD25Q32CCS(1);
    
    (void)result;

	return ret;
}

//0=没有错误，可以开始写。1=超时，不能进行写
int PrepareToWriteOrErase(uint32_t f_timeOutMs)
{
	uint32_t cnt = 0;
	uint8_t statusLow = 0;
	
	while(1)
	{
		//1，写使能
		EnableWrite();
		//2，获取状态
		statusLow = GetStatusLow();
		//3，判断是否 WIP=0,WEL=1
		if((statusLow & 0x03) == 0x02)
			return 0;
		else
		{
			HAL_Delay(1);
			cnt += 1;
			if(cnt > f_timeOutMs)//超时
			{
				return 1;
			}	
		}
 
	}
}
//写的超时时间为5ms(手册中是2.5)，擦扇区超时时间为310ms。 返回1表示准备好了
int WriteOrEraseComplete(uint32_t f_timeOutMs)
{
	uint32_t cnt = 0;
	uint8_t statusWIP = 0;
	
	while(1)
	{
		statusWIP = GetStatusLow() & 0x01;
		
		if(statusWIP == 0)
		{
			return 1;
		}
		else
		{
			HAL_Delay(1);
			cnt += 1;
			if(cnt > f_timeOutMs)//超时
			{
				return 0;
			}	
		}
	}
}
int PrepareToReadData(uint32_t f_timeOutMs)
{
	uint32_t cnt = 0;
	uint8_t statusWIP = 0;
	
	while(1)
	{
		statusWIP = GetStatusLow() & 0x01;
		
		if(statusWIP == 0)
		{
			return 0;
		}
		else
		{
			HAL_Delay(1);
			cnt += 1;
			if(cnt > f_timeOutMs)//超时，正常一页最多写2.5ms
			{
				return 1;
			}	
		}
	}
}


//不进行任何的检测，直接写入.
static void WriteDataNoCheck(uint32_t f_destAddr, uint8_t *fp_data, uint32_t f_len)
{
	uint8_t txData[4] = {COMMAND_PAGE_PROGRAM,(f_destAddr >> 16)&0xFF,(f_destAddr >> 8)&0xFF,(f_destAddr >> 0)&0xFF};
	HAL_StatusTypeDef result;
	GD25Q32CCS(0);
	
	result = hc_spi_trans(txData,4);
	
	result = hc_spi_trans(fp_data,f_len);
	
	GD25Q32CCS(1);
    
    (void)result;
}
//不进行擦除的写入，但是要检测页
static int WriteDataNoErase(uint32_t f_destAddr, uint8_t *fp_data, uint32_t f_len)
{
	uint16_t remain=0;
	
	if(f_len == 0)
		return 0;
    
    remain = f_destAddr%PAGE_SIZE; //当前地址在页内的偏移量
    
    if( remain )
    {
        if(PrepareToWriteOrErase(PREPARE_WRITE_OR_ERASE_TIMEOUT) == 1)
			return -1;
        remain = PAGE_SIZE-remain;//当前页可填充字节数
        if( f_len <= remain ) //当前页内可填充要写入数据
        {
            WriteDataNoCheck(f_destAddr, fp_data, f_len);
            if(WriteOrEraseComplete(WRITE_COMPLETE_TIMEOUT) == 0)//超时
			{
				return -4;
			}
            return 0;
        }
        else
        {
            WriteDataNoCheck(f_destAddr, fp_data, remain);
            if(WriteOrEraseComplete(WRITE_COMPLETE_TIMEOUT) == 0)//超时
			{
				return -5;
			}
            f_destAddr += remain;
			fp_data += remain;
			f_len -= remain;
        }
    }
	
	for(;;)
	{
		if(PrepareToWriteOrErase(PREPARE_WRITE_OR_ERASE_TIMEOUT) == 1)
			return -1;
		if(f_len > PAGE_SIZE)
		{
			WriteDataNoCheck(f_destAddr, fp_data, PAGE_SIZE);
			f_destAddr += PAGE_SIZE;
			fp_data += PAGE_SIZE;
			f_len -= PAGE_SIZE;
			
			if(WriteOrEraseComplete(WRITE_COMPLETE_TIMEOUT) == 0)//超时
			{
				return -2;
			}
		}
		else
		{
			WriteDataNoCheck(f_destAddr, fp_data, f_len);
			if(WriteOrEraseComplete(WRITE_COMPLETE_TIMEOUT) == 0)//超时
			{
				return -3;
			}
			break;
		}
	}
	return 0;
}

int WriteData(uint32_t f_destAddr, uint8_t *fp_data, uint32_t f_len)
{
	uint32_t i = 0;

	uint32_t sectorNum = 0;
	uint32_t sectorOff = 0;


	uint32_t readFrom = 0;
	uint32_t diff = 0;
	int ret = 0;

	if(f_destAddr >= DEVICE_SIZE || f_destAddr + f_len >= DEVICE_SIZE)
		return 1;
	
	if(f_len == 0)
		return 0;
	memset(gs_sectorBuf,0,sizeof(gs_sectorBuf));

	sectorNum = f_destAddr/SECT_SIZE;
	sectorOff = f_destAddr%SECT_SIZE;
	
	readFrom = sectorNum * SECT_SIZE;
	
	for(;;)
	{
		if(f_len == 0)
		{
			return 0;
		}
		//注意 while an Erase,Program or Write cycle is in progress 读命令会被拒绝
		if(PrepareToReadData(PREPARE_READ_TIMEOUT) == 1)
		{
			return 2;
		}	
		ReadDataBytes(readFrom, gs_sectorBuf, SECT_SIZE);

		for(i=sectorOff;i<SECT_SIZE;i++)
		{
			if(gs_sectorBuf[i] != 0xFF)
				break;
		}
		if(i < SECT_SIZE)//遇到了非0xFF的数据，
		{
			if(PrepareToWriteOrErase(PREPARE_WRITE_OR_ERASE_TIMEOUT) == 1)
			{
				return 3;
			}
			EraseSector(readFrom);
			if(WriteOrEraseComplete(SECTOR_ERASE_COMPLETE_TIMEOUT) == 0)//擦除超时
			{
				return 4;
			}
			diff = SECT_SIZE - sectorOff;
			if(f_len > diff)
			{
				memcpy(gs_sectorBuf+sectorOff,fp_data,diff);
				ret = WriteDataNoErase(readFrom,gs_sectorBuf,SECT_SIZE);
				if(0 != ret)
					return ret;
				f_len -= diff;
				fp_data += diff;
				sectorOff = 0;
				readFrom += SECT_SIZE;
				f_destAddr += diff;
			}
			else
			{
				memcpy(gs_sectorBuf+sectorOff,fp_data,f_len);
				ret = WriteDataNoErase(readFrom,gs_sectorBuf,SECT_SIZE);
				if(0 != ret)
					return ret;
				f_len = 0;
			}
			
		}
		else//不需要擦除，直接写
		{
			diff = SECT_SIZE - sectorOff;
			if(f_len > diff)
			{
				memcpy(gs_sectorBuf+sectorOff,fp_data,f_len);
				ret = WriteDataNoErase(f_destAddr,gs_sectorBuf+sectorOff,diff);
				if(0 != ret)
					return ret;
				f_len -= diff;
				fp_data += diff;
				sectorOff = 0;
				readFrom += diff;
				f_destAddr += diff;
			}
			else
			{
				memcpy(gs_sectorBuf+sectorOff,fp_data,f_len);
				ret = WriteDataNoErase(f_destAddr,gs_sectorBuf+sectorOff,f_len);
				if(0 != ret)
					return ret;
				f_len = 0;
			}
		}
		
	}
}


