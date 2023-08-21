//标准头文件
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
//hc32头文件
#include "hc_crc.h"
//APP
#include "flash_op.h"




/*
*********************************************************************************************************
*
*	模块名称 : 
*	文件名称 :
*	版    本 : 
*	说    明 : 
*			1，flash在进行写和擦除操作的时候如果读flash会导致总线阻塞，所以在进行写和擦除操作时需要使用临界区保护
*			2，因为擦除占用时间较长，会使得上位机等待数据超时。所以第一升级时，先擦除整个扇区，不写。下一次不需要擦除，直接写。
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.1    		2019-02-14  杨臣   	正式发布
*	外设资源占用：
		flash
*	待优化：（需要优化但是未优化之前请保留该提示）
*		1,暂无
*********************************************************************************************************
*/








/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/

/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/
static version_t gs_anotherVer = {0};
//const uint8_t gc_test[0x23780] = {0};
////const uint8_t gc_test[0x10] = {0};
//static uint8_t gs_ta = 0;
/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/




static paramAreaInfo_t gs_paramAreaInfo;
static upgradeProcessInfo_t gs_upgradeProcessInfo;
static uint32_t gs_upgradeDataRecState = 0;//1=表示升级数据接收成功，仅仅用来反馈给上位机
static uint32_t gs_upgradeFlag = 0;//接收到非回滚命令则置一

static uint8_t m_otaing_flag = 0; //非零表示正在进行ota


/************************************************************************************************************************************
																静态函数声明
*************************************************************************************************************************************/
static int WriteDataToBackup1(uint32_t f_addr, uint8_t *fp_data, uint32_t f_dataLen);
static int WriteDataToBackup2(uint32_t f_addr, uint8_t *fp_data, uint32_t f_dataLen);
static int HandleReceivedBinData(uint32_t f_backupNum, uint32_t f_msgPkgNum,uint32_t f_dataLen,uint8_t *fp_dataBuf,uint32_t f_dataCRC);




/************************************************************************************************************************************
																硬件初始化
*************************************************************************************************************************************/


/************************************************************************************************************************************
																中断处理
*************************************************************************************************************************************/

/************************************************************************************************************************************
																内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/
//连续擦除n块扇区
static int FlashEraseSector( uint32_t f_sector, uint8_t f_num_sectors )
{
    int ret = 0;
    int32_t SectorError = 0;

	//进入临界区
	__disable_irq();
	
	//1，解锁
	EFM_FWMC_Cmd(ENABLE);
	EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, ENABLE);
    
	//2，擦除扇区
	for (uint8_t i = 0; i < f_num_sectors; i++)
		SectorError += EFM_SectorErase(EFM_SECTOR_ADDR(f_sector+i));

	//4，上锁
//	HAL_FLASH_Lock();
	EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, DISABLE);
	EFM_FWMC_Cmd(DISABLE);
	
	//退出临界区
	__enable_irq();
    
    if( LL_OK != SectorError )
    {
		ret = -1;
    }
	return ret;
}
/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明:		写数据到backup1区,backup区有两个连续的扇区，都是128K.
*	形    参: 	
*	返 回 值:		
*********************************************************************************************************
*/
static int WriteDataToBackup1(uint32_t f_addr, uint8_t *fp_data, uint32_t f_dataLen)
{
	int ret = 0;

	static uint32_t s_sector1UnusedAddr = BACKUP_AREA_1_SECTOR_1_ADDR;
	static uint32_t s_sector2UnusedAddr = BACKUP_AREA_1_SECTOR_2_ADDR;
	static uint32_t s_sector1HasErased = 0;
	static uint32_t s_sector2HasErased = 0;
	uint32_t eraseFlag = 0;
	uint16_t f_sector = f_addr/SECTOR_SIZE;
	uint16_t f_num_sectors = f_dataLen%SECTOR_SIZE == 0 ? f_dataLen/SECTOR_SIZE : f_dataLen/SECTOR_SIZE + 1;
	
	if(f_dataLen == 0)
		return 0;
	if(f_dataLen > BACKUP_AREA_1_SIZE)
		return 1;
	if(f_dataLen % 4 != 0)
		return 1;
	//一次性写入的数据不允许跨扇区
	if(f_addr < (BACKUP_AREA_1_SECTOR_1_ADDR + BACKUP_AREA_1_SECTOR_1_SIZE) && (f_addr + f_dataLen) > (BACKUP_AREA_1_SECTOR_1_ADDR + BACKUP_AREA_1_SECTOR_1_SIZE))
		return 2;
	//检查地址的合法性
	if(f_addr < BACKUP_AREA_1_SECTOR_1_ADDR)
		return 3;
	if(f_addr >= BACKUP_AREA_1_SECTOR_2_ADDR + BACKUP_AREA_1_SECTOR_2_SIZE)
		return 4;

	//即将修改备份区域1的内容，对备份区域1的信息清空,只需要设置为无代码即可
	if(gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag != 0)
	{
		gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag = 0;
		UpdateParamAreaData();

	}
	#if 1
	if (f_addr%SECTOR_SIZE == 0)
	{
		ret = FlashEraseSector( f_sector, 1 );
	
		if(ret != 0)
			return 5;
	}
	#else
	//第一次写入数据时，查看两个扇区是否都已经擦除，如果没有擦除则一次性全擦除
	if(f_addr == BACKUP_AREA_1_SECTOR_1_ADDR)
	{
		eraseFlag = 0;
		if(s_sector1HasErased == 0)
		{
			ret = FlashEraseSector( BACKUP_AREA_1_SECTOR_1_NUM, BACKUP_AREA_1_SECTOR_1_SIZE/SECTOR_SIZE );
			
			if(ret != 0)
				return 5;
			else
			{
				s_sector1UnusedAddr = BACKUP_AREA_1_SECTOR_1_ADDR;
				s_sector1HasErased = 1;
				eraseFlag++;
			}
		}
		if(s_sector2HasErased == 0)
		{
			ret = FlashEraseSector( BACKUP_AREA_1_SECTOR_2_NUM, BACKUP_AREA_1_SECTOR_2_SIZE/SECTOR_SIZE );
			
			if(ret != 0)
				return 5;
			else
			{
				s_sector2UnusedAddr = BACKUP_AREA_1_SECTOR_2_ADDR;
				s_sector2HasErased = 1;
				eraseFlag++;
			}
		}
		if(eraseFlag != 0)//进行了查擦，因为时间较长，所以这一次并不写入flash，保证下一次升级时能够不需要擦除扇区直接写。
			return 100;
	}
	#endif
#if 0
	/*对于目标地址小于未使用地址的，应该先擦除扇区，但是又不允许从中间开始写，
	所以此时只有目标地址等于首地址时才会擦除扇区，否则返回错误*/
	if(f_addr == BACKUP_AREA_1_SECTOR_1_ADDR && s_sector1HasErased == 0)
	{
		//进入临界区
		__disable_irq();
		//1，解锁
		HAL_FLASH_Unlock();
		//2，擦除扇区
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
						FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
		if(FLASH_COMPLETE != FLASH_EraseSector(BACKUP_AREA_1_SECTOR_1_NUM, VoltageRange_3))
			ret = 5;
		//3，上锁
		HAL_FLASH_Lock();
		//退出临界区
		__enable_irq();
		
		if(ret != 0)
			return ret;
		else
		{
			s_sector1UnusedAddr = BACKUP_AREA_1_SECTOR_1_ADDR;
			s_sector1HasErased = 1;
		}
	}
	else if(f_addr < s_sector1UnusedAddr)
		return 6;
	else if(f_addr == BACKUP_AREA_1_SECTOR_2_ADDR && s_sector2HasErased == 0)
	{
		//进入临界区
		__disable_irq();
		//1，解锁
		HAL_FLASH_Unlock();
		//2，擦除扇区
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
						FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
		if(FLASH_COMPLETE != FLASH_EraseSector(BACKUP_AREA_1_SECTOR_2_NUM, VoltageRange_3))
			ret = 7;
		//3，上锁
		HAL_FLASH_Lock();
		//退出临界区
		__enable_irq();
		
		if(ret != 0)
			return ret;
		else
		{
			s_sector2UnusedAddr = BACKUP_AREA_1_SECTOR_2_ADDR;
			s_sector2HasErased = 1;
		}
	}
	else if(f_addr >= BACKUP_AREA_1_SECTOR_2_ADDR && f_addr < s_sector2UnusedAddr)
		return 8;
#endif
    #if 0
	if((f_addr < BACKUP_AREA_1_SECTOR_2_ADDR && f_addr >= s_sector1UnusedAddr) || \
		(f_addr >= BACKUP_AREA_1_SECTOR_2_ADDR && f_addr >= s_sector2UnusedAddr))	//写入扇区
    #endif
	{
		//进入临界区
		__disable_irq();
		//解锁
		EFM_FWMC_Cmd(ENABLE);
		EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, ENABLE);
		ret = WriteWordsToFlash(f_addr,(uint32_t *)fp_data, f_dataLen/4);
		//上锁
		EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, DISABLE);
		EFM_FWMC_Cmd(DISABLE);
		//退出临界区
		__enable_irq();
        #if 0
		if(f_addr >= BACKUP_AREA_1_SECTOR_2_ADDR)
		{
			s_sector2UnusedAddr = f_addr + f_dataLen;
			s_sector2HasErased = 0;
		}
		else
		{
			s_sector1UnusedAddr = f_addr + f_dataLen;
			s_sector1HasErased = 0;
		}
        #endif
		gs_paramAreaInfo.upgradeInfo.backupArea1.state = 2;
	}
	
	return ret;

}
static int WriteDataToBackup2(uint32_t f_addr, uint8_t *fp_data, uint32_t f_dataLen)
{
	int ret = 0;

	static uint32_t s_sector1UnusedAddr = BACKUP_AREA_2_SECTOR_1_ADDR;
	static uint32_t s_sector2UnusedAddr = BACKUP_AREA_2_SECTOR_2_ADDR;
	static uint32_t s_sector1HasErased = 0;
	static uint32_t s_sector2HasErased = 0;
	uint32_t eraseFlag = 0;
	uint16_t f_sector = f_addr/SECTOR_SIZE;
	uint16_t f_num_sectors = f_dataLen%SECTOR_SIZE == 0 ? f_dataLen/SECTOR_SIZE : f_dataLen/SECTOR_SIZE + 1;
	
	if(f_dataLen == 0)
		return 0;
	if(f_dataLen > BACKUP_AREA_2_SIZE)
		return 1;
	if(f_dataLen % 4 != 0)
		return 1;
	//一次性写入的数据不允许跨扇区
	if(f_addr < (BACKUP_AREA_2_SECTOR_1_ADDR + BACKUP_AREA_2_SECTOR_1_SIZE) && (f_addr + f_dataLen) > (BACKUP_AREA_2_SECTOR_1_ADDR + BACKUP_AREA_2_SECTOR_1_SIZE))
		return 2;
	//检查地址的合法性
	if(f_addr < BACKUP_AREA_2_SECTOR_1_ADDR)
		return 3;
	if(f_addr >= BACKUP_AREA_2_SECTOR_2_ADDR + BACKUP_AREA_2_SECTOR_2_SIZE)
		return 4;
		
	if(gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag != 0)
	{
		gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag = 0;
		UpdateParamAreaData();
	}
	#if 1
	if (f_addr%SECTOR_SIZE == 0)
	{
		ret = FlashEraseSector( f_sector, 1 );
	
		if(ret != 0)
			return 5;
	}
	#else
		//第一次写入数据时，查看两个扇区是否都已经擦除，如果没有擦除则一次性全擦除
	if(f_addr == BACKUP_AREA_2_SECTOR_1_ADDR)
	{
		eraseFlag = 0;
		if(s_sector1HasErased == 0)
		{
			ret = FlashEraseSector( BACKUP_AREA_2_SECTOR_1_NUM, BACKUP_AREA_2_SECTOR_1_SIZE/SECTOR_SIZE );
			
			if(ret != 0)
				return 5;
			else
			{
				s_sector1UnusedAddr = BACKUP_AREA_2_SECTOR_1_ADDR;
				s_sector1HasErased = 1;
				eraseFlag++;
			}
		}
		if(s_sector2HasErased == 0)
		{
			ret = FlashEraseSector( BACKUP_AREA_2_SECTOR_2_NUM, BACKUP_AREA_2_SECTOR_2_SIZE/SECTOR_SIZE );
			
			if(ret != 0)
				return 5;
			else
			{
				s_sector2UnusedAddr = BACKUP_AREA_2_SECTOR_2_ADDR;
				s_sector2HasErased = 1;
				eraseFlag++;
			}
		}
		if(eraseFlag != 0)//进行了查擦，因为时间较长，所以这一次并不写入flash，保证下一次升级时能够不需要擦除扇区直接写。
			return 100;
	}
	#endif
#if 0
	/*对于目标地址小于未使用地址的，应该先擦除扇区，但是又不允许从中间开始写，
	所以此时只有目标地址等于首地址时才会擦除扇区，否则返回错误*/
	if(f_addr == BACKUP_AREA_2_SECTOR_1_ADDR && s_sector1HasErased == 0)
	{
		//进入临界区
		__disable_irq();
		//1，解锁
		HAL_FLASH_Unlock();
		//2，擦除扇区
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
						FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
		if(FLASH_COMPLETE != FLASH_EraseSector(BACKUP_AREA_2_SECTOR_1_NUM, VoltageRange_3))
			ret = 5;
		//3，上锁
		HAL_FLASH_Lock();
		//退出临界区
		__enable_irq();
		
		if(ret != 0)
			return ret;
		else
		{
			s_sector1UnusedAddr = BACKUP_AREA_2_SECTOR_1_ADDR;
			s_sector1HasErased = 1;
		}
	}
	else if(f_addr < s_sector1UnusedAddr)
		return 6;
	else if(f_addr == BACKUP_AREA_2_SECTOR_2_ADDR && s_sector2HasErased == 0)
	{
		//进入临界区
		__disable_irq();
		//1，解锁
		HAL_FLASH_Unlock();
		//2，擦除扇区
		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
						FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
		if(FLASH_COMPLETE != FLASH_EraseSector(BACKUP_AREA_2_SECTOR_2_NUM, VoltageRange_3))
			ret = 7;
		//3，上锁
		HAL_FLASH_Lock();
		//退出临界区
		__enable_irq();
		
		if(ret != 0)
			return ret;
		else
		{
			s_sector2UnusedAddr = BACKUP_AREA_2_SECTOR_2_ADDR;
			s_sector2HasErased = 1;
		}
	}
	else if(f_addr >= BACKUP_AREA_2_SECTOR_2_ADDR && f_addr < s_sector2UnusedAddr)
		return 8;
#endif
    #if 0
	if((f_addr < BACKUP_AREA_2_SECTOR_2_ADDR && f_addr >= s_sector1UnusedAddr) || \
		(f_addr >= BACKUP_AREA_2_SECTOR_2_ADDR && f_addr >= s_sector2UnusedAddr))	//写入扇区
	#endif
	{
		//进入临界区
		__disable_irq();
		//解锁
		EFM_FWMC_Cmd(ENABLE);
		EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, ENABLE);
		ret = WriteWordsToFlash(f_addr,(uint32_t *)fp_data, f_dataLen/4);
		//上锁
		EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, DISABLE);
		EFM_FWMC_Cmd(DISABLE);
		//退出临界区
		__enable_irq();
        #if 0
		if(f_addr >= BACKUP_AREA_2_SECTOR_2_ADDR)
		{
			s_sector2UnusedAddr = f_addr + f_dataLen;
			s_sector2HasErased = 0;
		}
		else
		{
			s_sector1UnusedAddr = f_addr + f_dataLen;
			s_sector1HasErased = 0;
		}
        #endif
		gs_paramAreaInfo.upgradeInfo.backupArea2.state = 2;
	}
	
	return ret;

}

/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/
/*
接收bin文件时，写入flash时，2k写一次，同时写之前计算这2K数据的crc，然后保存。以防一直占用crc资源。写完整个文件之后
同样每读2K进行校验，然后比较写入前后的crc是否相同，如果相同则进行一次所有数据的校验，并记录下这个校验值。
*/


/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 通过参数区第一部分的信息获取整个参数区的数据
*	形    参: f_bufLen 缓冲区的大小，单位时字节
*	返 回 值:
*********************************************************************************************************
*/
uint32_t GetAllDataFromParamAreaByLen(uint32_t *fp_buf, uint32_t f_bufLen)
{
	uint32_t crc = 0;
	uint32_t tempU32 = 0;
	uint32_t *p_tempU32 = 0;
	paramAreaLength_t paramAreaLength;
	//1，取数据
	memcpy((uint8_t *)&paramAreaLength,(uint8_t *)PARAM_LEN_AREA_ADDR,sizeof(paramAreaLength_t));
	//2，校验长度数据的可靠性
	crc = SWCalcCRC(paramAreaLength.len);
	
	if(crc != paramAreaLength.crcCheck)
		return 1;
	
	if(f_bufLen < paramAreaLength.len)
		return 2; 
		
	//检验所有数据的可靠性
	tempU32 = paramAreaLength.len / 4;
	crc = SWCalcBlockCRC((uint32_t *)PARAM_AREA_ADDR, tempU32 - 1);
	p_tempU32 = (uint32_t *)(PARAM_AREA_ADDR + paramAreaLength.len - 4);
	if(crc != *p_tempU32)
		return 3;

	memcpy((uint8_t *)fp_buf,(uint8_t *)PARAM_AREA_ADDR,paramAreaLength.len);
	
	return 0;
}
/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 通过参数区数据结构获取所有数据
*	形    参: 
*	返 回 值:
*********************************************************************************************************
*/
uint32_t GetAllDataFromParamArea(paramAreaInfo_t *fp_paramAreaInfo)
{
	uint32_t ret = 0;
	//1，取数据
	memcpy((uint8_t *)fp_paramAreaInfo,(uint8_t *)PARAM_AREA_ADDR,sizeof(paramAreaInfo_t));
	//2，校验
	if(fp_paramAreaInfo->paramAreaLength.crcCheck != SWCalcCRC(fp_paramAreaInfo->paramAreaLength.len))
		ret |= (0x01 << 0);
	if(fp_paramAreaInfo->crcCheck != SWCalcBlockCRC((uint32_t *)fp_paramAreaInfo,sizeof(paramAreaInfo_t)/4 - 1))
		ret |= (0x01 << 1);	
	
	
	return ret;
}
/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 		写flash，以字为单位
*	形    参: 	f_len单位也是字
*	返 回 值:		0=成功
*********************************************************************************************************
*/
int WriteWordsToFlash(uint32_t f_address,uint32_t *fp_wordData, uint32_t f_len)
{
	uint32_t i;
//	EFM_FWMC_Cmd(ENABLE);
//    EFM_SequenceSectorOperateCmd(64, 32, ENABLE);
	for(i=0;i<f_len;i++)
	{
		if (LL_OK != EFM_ProgramWord(f_address, fp_wordData[i]))
			return -1;
		f_address += 4;
	}
//	EFM_FWMC_Cmd(DISABLE);
//    EFM_SequenceSectorOperateCmd(64, 32, DISABLE);

	return 0;
}



/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 		把参数数据写入参数区域
*	形    参: 	f_len单位也是字
*	返 回 值:		0=成功
*********************************************************************************************************
*/
int WriteParamDataToParamArea(const paramAreaInfo_t *fp_paramAreaInfo)
{
	int ret = 0;
	uint16_t num = PARAM_AREA_SIZE%SECTOR_SIZE == 0 ? PARAM_AREA_SIZE/SECTOR_SIZE : PARAM_AREA_SIZE/SECTOR_SIZE + 1;

	//擦除扇区
	ret = FlashEraseSector(PARAM_AREA_SECTOR_NUM, num);

	//进入临界区
	__disable_irq();
	
	//1，解锁
	EFM_FWMC_Cmd(ENABLE);
	EFM_SequenceSectorOperateCmd(PARAM_AREA_SECTOR_NUM, num, ENABLE);

	if(ret == 0)//3，写入flash
    {
		ret = WriteWordsToFlash(PARAM_AREA_ADDR,(uint32_t *)fp_paramAreaInfo, PARAM_AREA_SIZE/4);
    }
	//4，上锁
	EFM_SequenceSectorOperateCmd(PARAM_AREA_SECTOR_NUM, num, DISABLE);
	EFM_FWMC_Cmd(DISABLE);
	
	//退出临界区
	__enable_irq();

	return ret;
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 		硬件计算块数据的crc，使用互斥量对crc硬件资源进行保护计算100K byte数据大概需要3ms
*	形    参: 	f_ticksToWait 等待获取互斥量的最大时间,f_bufLen单位是字
*	返 回 值:		0=成功
*********************************************************************************************************
*/
uint32_t SWCalcBlockCRC(uint32_t *fp_buf, uint32_t f_bufLen)
{
	uint32_t crc = 0;
	

	//1，复位数据寄存器
//	CRC_ResetDR();
//	//2，计算
//	crc = CRC_CalcBlockCRC(fp_buf, f_bufLen);
//	crc = CRC_Calculate( fp_buf, f_bufLen );
	crc = hc_sw_crc32_calculate(fp_buf, f_bufLen);

	return crc;
}
/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 		硬件计算单个字的crc，使用互斥量对crc硬件资源进行保护
*	形    参: 	f_ticksToWait 等待获取互斥量的最大时间
*	返 回 值:		0=成功
*********************************************************************************************************
*/
uint32_t SWCalcCRC(uint32_t f_data)
{
	uint32_t crc = 0;
	
	
	//1，复位数据寄存器
//	CRC_ResetDR();
//	//2，计算
//	crc = CRC_CalcCRC(f_data);
//	crc = CRC_Calculate( &f_data, 1 );
	crc = hc_sw_crc32_calculate(&f_data, 1);

	return crc;
}
/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 		1，校验数据
*	形    参: 	
*			f_msgPkgNum 	包序号
*			f_dataLen 		数据长度
*			fp_dataBuf 		数据缓冲区
*			f_dataCRC 		数据的CRC
*			
*	返 回 值:		0=成功，1=失败，<0=错误
*********************************************************************************************************
*/
int HandleUpgradeData(int32_t f_msgPkgNum,uint32_t f_dataLen,uint8_t *fp_dataBuf,uint32_t f_dataCRC)
{
	int ret = 0;
	static uint32_t s_backupNum = 0;
	static int s_waitingVersion = 0;//1=所有数据包已经接收，等待结束包，结束包中含有版本
	static backupAreaInfo_t *p_backupAreaInfo = NULL;
	version_t version;

//	printf("pkgNum = %d, dataLen = %d\n",f_msgPkgNum,f_dataLen);

	if(s_backupNum == 0)//为了实现回滚代码功能，APP正在运行的代码所在的备份区不能被覆盖
	{
		if(f_msgPkgNum == 0)
		{
			if(gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag != 1)
				s_backupNum = 1;
			else if(gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag != 1)
				s_backupNum = 2;
			else
			{
				switch(gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc)
				{
					case 1:
						s_backupNum = 2;
					break;
					case 2:
						s_backupNum = 1;
					break;
					default:
						s_backupNum = 1;				
					break;

				}
			}
			printf("s_backupNum:%d\r\n",s_backupNum);
		}
		else if(f_msgPkgNum != -3)
		{
			return -2;
		}
	}

	if(f_msgPkgNum != -3)//只要不是回滚命令，就设置标记。标记设置后如果接收到了回滚命令则不会滚并清除本次接收到的所有升级信息
		gs_upgradeFlag = 1;
	
	if(s_backupNum == 1)
		p_backupAreaInfo = &gs_paramAreaInfo.upgradeInfo.backupArea1;
	else
		p_backupAreaInfo = &gs_paramAreaInfo.upgradeInfo.backupArea2;


	if(f_msgPkgNum >= 0)//数据包
	{
        m_otaing_flag = 1;
		ret = HandleReceivedBinData(s_backupNum, f_msgPkgNum,f_dataLen,fp_dataBuf,f_dataCRC);
		printf("HandleReceivedBinData() = %d\n",ret);
		if(ret == 0)//升级成功，修改配置信息
		{
			s_waitingVersion = 1;
			ret = 0;
		}
		else if(ret == 1)//数据包没问题，还有后续数据包等待接收
		{
			ret = 0;
		}
		else if(ret < 0)//有错误，根据情况修改gs_upgradeProcessInfo数据和配置信息
		{
			s_backupNum = 0;
			gs_upgradeProcessInfo.msgPkgNum = 0;
			gs_upgradeProcessInfo.getAllMsgPkgFlag = 0;
			gs_upgradeProcessInfo.dataLen = 0;
			gs_upgradeProcessInfo.saveDataIndex = 0;
			gs_upgradeProcessInfo.saveCRCIndex = 0;
			if(p_backupAreaInfo->state == 2)
			{
				p_backupAreaInfo->hasCodeFlag = 0;
				p_backupAreaInfo->codeSize = 0;
				p_backupAreaInfo->state = 0;
				gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.UpgradeFlag = 0;
				//把参数信息写入flash
				UpdateParamAreaData();
			}
		}
		
	}
	else if(f_msgPkgNum == -1)//发送完成并包含版本信息，此时认为升级成功
	{
        m_otaing_flag = 0;
		gs_upgradeFlag = 1;
		if(s_waitingVersion == 1)
		{
			if(sizeof(p_backupAreaInfo->version) != f_dataLen)
			{
				return 1;
			}
			s_waitingVersion = 0;
			memset((uint8_t *)&p_backupAreaInfo->version,0,sizeof(p_backupAreaInfo->version));
			memcpy((uint8_t *)&p_backupAreaInfo->version,fp_dataBuf,f_dataLen);


			p_backupAreaInfo->codeSize = gs_upgradeProcessInfo.dataLen;
			p_backupAreaInfo->hasCodeFlag = 1;
			p_backupAreaInfo->state = 1;
			p_backupAreaInfo->codeHwCRC = gs_upgradeProcessInfo.hwCRC;

			gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.UpgradeFlag = 1;
			gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.backupAreaNum = s_backupNum;
			s_backupNum = 0;
			gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.state = 2;
			

			memset((uint8_t *)&gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.version,0,sizeof(p_backupAreaInfo->version));
			memcpy((uint8_t *)&gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.version,fp_dataBuf,f_dataLen);

			//printf("gs_paramAreaInfo:\t%d\t%d\n",gs_paramAreaInfo.upgradeInfo.backupArea1.state,gs_paramAreaInfo.upgradeInfo.backupArea2.state);

//				//测试
//			gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc = gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.backupAreaNum;
//			gs_paramAreaInfo.upgradeInfo.appAreaInfo.version = p_backupAreaInfo->version;

			//把参数信息写入flash
			ret = UpdateParamAreaData();
			printf("get version:%d.%d.%d.%d\n",p_backupAreaInfo->version.type,p_backupAreaInfo->version.major,p_backupAreaInfo->version.minor,p_backupAreaInfo->version.build);
			printf("UpdateParamAreaData() = %d\n",ret);
			if(ret != 0)
				return 5;
		}
		else
		{
			return 2;
		}

	}
	else if(f_msgPkgNum == -3)//版本回滚
	{
        m_otaing_flag = 0;
		if(gs_upgradeFlag == 1)
		{
			if(gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.UpgradeFlag == 1)//说明升级成功，清空升级的备份区
			{
				switch(gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.backupAreaNum)
				{
					case 1:
						memset((uint8_t *)&gs_paramAreaInfo.upgradeInfo.backupArea1,0,sizeof(backupAreaInfo_t));
//						FlashEraseSector(BACKUP_AREA_1_SECTOR_1_NUM, BACKUP_AREA_1_SIZE/SECTOR_SIZE);
					break;
					
					case 2:
						memset((uint8_t *)&gs_paramAreaInfo.upgradeInfo.backupArea2,0,sizeof(backupAreaInfo_t));
//						FlashEraseSector(BACKUP_AREA_2_SECTOR_1_NUM, BACKUP_AREA_2_SIZE/SECTOR_SIZE);
					break;
					
					default:
					break;
				}
			}
			memset((uint8_t *)&gs_paramAreaInfo.upgradeInfo.upgradeControlInfo,0,sizeof(upgradeControlInfo_t));
			
			//把参数信息写入flash
			UpdateParamAreaData();
			return 0;	
		}
		
		//1，查看需要回退的版本是不是在备份区，并找出正确的备份区
	
		memcpy((uint8_t *)&version,fp_dataBuf,f_dataLen);

		if((gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag == 1) && \
			(VerCmp(gs_paramAreaInfo.upgradeInfo.backupArea1.version,version) == 0))
			s_backupNum = 1;
		else if((gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag == 1) && \
			(VerCmp(gs_paramAreaInfo.upgradeInfo.backupArea2.version,version) == 0))
			s_backupNum = 2;
		else
		{
			printf("rollback version:%d.%d.%d.%d is unavailable\n",version.type,version.major,version.minor,version.build);
			return 3;
		}
		
		
			
		//2，修改配置信息
		gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.UpgradeFlag = 1;
		gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.backupAreaNum = s_backupNum;
		s_backupNum = 0;
		gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.state = 2;


		gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.version = version;

		//把参数信息写入flash
		UpdateParamAreaData();
		printf("rollback version:%d.%d.%d.%d is available\n",version.type,version.major,version.minor,version.build);
			
		if(0)
		{//重启
			__set_FAULTMASK(1); //关闭所有中断
			NVIC_SystemReset(); //复位
		}
		
		ret = 0;

	}
	else if(f_msgPkgNum == -2)//升级失败
	{
//		if (s_backupNum == 1)
//			FlashEraseSector(BACKUP_AREA_1_SECTOR_1_NUM, BACKUP_AREA_1_SIZE/SECTOR_SIZE);
//		else
//			FlashEraseSector(BACKUP_AREA_2_SECTOR_1_NUM, BACKUP_AREA_2_SIZE/SECTOR_SIZE);
        m_otaing_flag = 0;
		s_backupNum = 0;
		gs_upgradeProcessInfo.msgPkgNum = 0;
		gs_upgradeProcessInfo.getAllMsgPkgFlag = 0;
		gs_upgradeProcessInfo.dataLen = 0;
		gs_upgradeProcessInfo.saveDataIndex = 0;
		gs_upgradeProcessInfo.saveCRCIndex = 0;
		if(p_backupAreaInfo->state == 2)
		{
			p_backupAreaInfo->hasCodeFlag = 0;
			p_backupAreaInfo->codeSize = 0;
			p_backupAreaInfo->state = 0;
			gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.UpgradeFlag = 0;
			//把参数信息写入flash
			UpdateParamAreaData();
			ret = 0;
		}
		printf("failed 3 times!\n");
		ret = 0;
	}
	else//未知
	{
		ret = -1;
	}
	return ret;
}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 		1，校验数据
*	形    参: 	
*			f_msgPkgNum 	包序号
*			f_dataLen 		数据包中的数据长度
*			fp_dataBuf 		数据缓冲区
*			f_dataCRC 		数据的CRC
*			
*	返 回 值:		0=成功接收所有数据并校验通过，1=不是最后一包数据，<0=错误
*********************************************************************************************************
*/
#define CRC_CHECK_EACH_TIME
static int HandleReceivedBinData(uint32_t f_backupNum, uint32_t f_msgPkgNum,uint32_t f_dataLen,uint8_t *fp_dataBuf,uint32_t f_dataCRC)
{
	int ret = 0;
	uint32_t hwCRC = 0;
	uint32_t swCRC = 0;
	uint32_t tempU32 = 0;
	uint32_t addr = 0;
	int i=0,j=0;
	uint32_t topOfStack = 0;
	uint32_t resetHandlerAddr = 0;
//		uint32_t cnt1 = 0;
//	uint32_t cnt2 = 0;

	//1，校验数据：序号，长度，crc
	if((f_msgPkgNum != 0) && (f_msgPkgNum != gs_upgradeProcessInfo.msgPkgNum))
		return -1;
	if(f_dataLen > UPGRADE_PKG_SIZE)
		return -2;
	if(f_dataLen % 4 != 0)//stm32f4的bin文件大小一定是4的整数倍，满包是256，则每包也一定是4的整数倍
		return -3;
	//swCRC = CRC32Default(fp_dataBuf,f_dataLen);
	swCRC = SWCalcBlockCRC((uint32_t *)fp_dataBuf,f_dataLen/4);
	if(swCRC != f_dataCRC)
	{
//		for(i=0;i<f_dataLen;i+=4)
//			printf("%02x %02x %02x %02x ",fp_dataBuf[i+3],fp_dataBuf[i+2],fp_dataBuf[i+1],fp_dataBuf[i]);
//		printf("\r\n");
//		printf("%x\t%x\r\n",f_dataCRC,swCRC);
		return -4;
	}

	if(f_backupNum != 1 && f_backupNum != 2)
		return -5;
	if(gs_upgradeProcessInfo.saveDataIndex + f_dataLen > 1024)//这个错误比较严重，除非程序出错否则不会出现
		return -6;

	if(f_msgPkgNum == 0)//检查第一包数据的合法性
	{
		topOfStack = ((uint32_t *)fp_dataBuf)[0];
		resetHandlerAddr = ((uint32_t *)fp_dataBuf)[1];
		if((topOfStack < 0x1FFE0000) || (topOfStack > 0x200F0FFF))
			return -10;
		else if((resetHandlerAddr <= VECTOR_TABLE_OFFSET_ADDR) || ((resetHandlerAddr & 0xFFF00000) != 0x00000000/*0x08000000*/))
			return -11;
		memset((uint8_t *)&gs_upgradeProcessInfo,0,sizeof(upgradeProcessInfo_t));
		
	}
	if(f_dataLen < UPGRADE_PKG_SIZE)
	{
	#if 0
		printf("last pkg len = %d:\n",f_dataLen);
		for(i=0;i<f_dataLen;i++)
		{
			if(i % 16 == 0)
			{
				printf("%08xh: ",256*f_msgPkgNum+i);
			}
			printf("%02X ",fp_dataBuf[i]);
			if(i % 16 == 15)
			{
				printf("\n");
			}
		}
		printf("\n");
	#endif
	}
	memcpy(gs_upgradeProcessInfo.saveData+gs_upgradeProcessInfo.saveDataIndex,fp_dataBuf,f_dataLen);
	gs_upgradeProcessInfo.saveDataIndex += f_dataLen;
	gs_upgradeProcessInfo.msgPkgNum++;

	//暂存区满，或者已经是最后一包数据，则计算CRC并写入flash中
	if((gs_upgradeProcessInfo.saveDataIndex == 1024) || f_dataLen < UPGRADE_PKG_SIZE)
	{
#if 1
		//计算并保存硬件CRC
		gs_upgradeProcessInfo.saveCRC[gs_upgradeProcessInfo.saveCRCIndex] = \
			SWCalcBlockCRC((uint32_t *)gs_upgradeProcessInfo.saveData, gs_upgradeProcessInfo.saveDataIndex/4);
#ifndef CRC_CHECK_EACH_TIME
		gs_upgradeProcessInfo.saveCRCIndex++;
#endif
#endif
//		cnt1 = xTaskGetTickCount();
//printf("f_backupNum:%d\r\n",f_backupNum);
		//把数据写入flash
		if(f_backupNum == 1)
		{
			ret = WriteDataToBackup1(gs_upgradeProcessInfo.dataLen + BACKUP_AREA_1_SECTOR_1_ADDR, gs_upgradeProcessInfo.saveData, gs_upgradeProcessInfo.saveDataIndex);
		}
		else
		{
			ret = WriteDataToBackup2(gs_upgradeProcessInfo.dataLen + BACKUP_AREA_2_SECTOR_1_ADDR, gs_upgradeProcessInfo.saveData, gs_upgradeProcessInfo.saveDataIndex);
		}	
//		cnt2 = xTaskGetTickCount();
//printf("EFM_SectorErase time:%d\r\n",cnt2-cnt1);
		if(ret != 0)//写入错误
		{
//		printf("ret=%d\r\n",ret);
			memset((uint8_t *)&gs_upgradeProcessInfo,0,sizeof(upgradeProcessInfo_t));
			return -7;
		}
		else
		{
#if 1
#ifdef CRC_CHECK_EACH_TIME
			addr = (f_backupNum == 1) ? BACKUP_AREA_1_SECTOR_1_ADDR : BACKUP_AREA_2_SECTOR_1_ADDR;
			swCRC = SWCalcBlockCRC((uint32_t *)(gs_upgradeProcessInfo.dataLen + addr), gs_upgradeProcessInfo.saveDataIndex/4);
//			printf("%d %d %X %X\r\n",crcNum++,gs_upgradeProcessInfo.saveDataIndex,gs_upgradeProcessInfo.saveCRC[gs_upgradeProcessInfo.saveCRCIndex],hwCRC);
			if(gs_upgradeProcessInfo.saveCRC[gs_upgradeProcessInfo.saveCRCIndex] != swCRC)
			{
				printf("saveCRC[%d] = %x,swCRC=%x\n",gs_upgradeProcessInfo.saveCRCIndex,gs_upgradeProcessInfo.saveCRC[gs_upgradeProcessInfo.saveCRCIndex],swCRC);
				return -8;
			}
			gs_upgradeProcessInfo.saveCRCIndex++;
#endif	
#endif
			gs_upgradeProcessInfo.dataLen += gs_upgradeProcessInfo.saveDataIndex;
		}
		gs_upgradeProcessInfo.saveDataIndex = 0;
		
	}

	
	if(f_dataLen < UPGRADE_PKG_SIZE)//最后一包数据小于最大值
	{
		gs_upgradeProcessInfo.getAllMsgPkgFlag = 1;
		gs_upgradeProcessInfo.msgPkgNum = 0;
#ifndef CRC_CHECK_EACH_TIME
		//1，此时所有数据都已经写入flash，然后需要每次1k的读出所有的code，然后校验crc，看写入是否正确
		for(i=0,j=0;i<gs_upgradeProcessInfo.dataLen;)
		{
			tempU32 = ((i+1024) <= gs_upgradeProcessInfo.dataLen)?1024:(gs_upgradeProcessInfo.dataLen - i);

			if(tempU32 % 4 != 0)
				return 8;
			if(f_backupNum == 1)
			{
				swCRC = SWCalcBlockCRC((uint32_t *)(BACKUP_AREA_1_SECTOR_1_ADDR+i),tempU32/4);
			}
			else
			{
				swCRC = SWCalcBlockCRC((uint32_t *)(BACKUP_AREA_2_SECTOR_1_ADDR+i),tempU32/4);
			}
			if(gs_upgradeProcessInfo.saveCRC[j] != swCRC || gs_upgradeProcessInfo.saveCRCIndex < j)
			{
				printf("saveCRC[%d] = %x,swCRC=%x\n",j,gs_upgradeProcessInfo.saveCRC[j],swCRC);
				return -8;
			}

			i += tempU32;
			j++;
		}
		//2，如果写入正确则完整的对所有的code进行一次总的crc，然后记录该crc并更新参数区的升级信息
		if(f_backupNum == 1)
		{
			gs_upgradeProcessInfo.hwCRC = SWCalcBlockCRC((uint32_t *)BACKUP_AREA_1_SECTOR_1_ADDR,gs_upgradeProcessInfo.dataLen/4);
		}
		else
		{
			gs_upgradeProcessInfo.hwCRC = SWCalcBlockCRC((uint32_t *)BACKUP_AREA_2_SECTOR_1_ADDR,gs_upgradeProcessInfo.dataLen/4);
		}
#else
		/* 上位机没有下发完整数据的crc，下位机通过硬件crc计算，记录该crc并更新参数区的升级信息 */
		if(f_backupNum == 1)
		{
			gs_upgradeProcessInfo.hwCRC = hc_hw_crc32_calculate((uint32_t *)BACKUP_AREA_1_SECTOR_1_ADDR,gs_upgradeProcessInfo.dataLen/4);
		}
		else
		{
			gs_upgradeProcessInfo.hwCRC = hc_hw_crc32_calculate((uint32_t *)BACKUP_AREA_2_SECTOR_1_ADDR,gs_upgradeProcessInfo.dataLen/4);
		}
//		printf("gs_upgradeProcessInfo.hwCRC=%X dataLen=%d\r\n",gs_upgradeProcessInfo.hwCRC,gs_upgradeProcessInfo.dataLen);
#endif
		return 0;

	}
	else
	{
		gs_upgradeProcessInfo.getAllMsgPkgFlag = 0;
	}

	return 1;


}

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: 		1，更新参数区数据
*	形    参: 	
*			
*	返 回 值:		0=成功，1=失败，-1=错误
*********************************************************************************************************
*/
int UpdateParamAreaData(void)
{
	int ret = 0;
	uint32_t srcDataCRC = 0;

//1，计算新数据的所有crc
	//计算参数区数据长度
	gs_paramAreaInfo.paramAreaLength.len = sizeof(paramAreaInfo_t);
	gs_paramAreaInfo.paramAreaLength.crcCheck = SWCalcCRC(gs_paramAreaInfo.paramAreaLength.len);

	//计算参数区数据的总crc
	gs_paramAreaInfo.crcCheck = SWCalcBlockCRC((uint32_t *)&gs_paramAreaInfo, sizeof(paramAreaInfo_t)/4 - 1);

	


//2，比较数据是否有更新
	srcDataCRC = SWCalcBlockCRC((uint32_t *)PARAM_AREA_ADDR, sizeof(paramAreaInfo_t)/4 - 1);
	if(gs_paramAreaInfo.crcCheck == srcDataCRC)
		return 0;

//3，把数据写入flash
	ret = WriteParamDataToParamArea(&gs_paramAreaInfo);
	if(ret != 0)
		return ret;


//4，查看是否写入成
	srcDataCRC = SWCalcBlockCRC((uint32_t *)PARAM_AREA_ADDR, sizeof(paramAreaInfo_t)/4 - 1);
	if(gs_paramAreaInfo.crcCheck == srcDataCRC)//写入成功
		return 0;
	else
		return -1;


}

void SetUpgradeDataRecState(uint32_t f_state)
{
	gs_upgradeDataRecState = f_state;
}

uint32_t GetUpgradeDataRecState(void)
{
	return gs_upgradeDataRecState;
}

void GetBackup1Version(version_t *fp_version)
{
	if(gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag == 1)
		*fp_version = gs_paramAreaInfo.upgradeInfo.backupArea1.version;
	else
		memset((uint8_t *)fp_version,0,sizeof(version_t));
}
void GetBackup2Version(version_t *fp_version)
{
	if(gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag == 1)
		*fp_version = gs_paramAreaInfo.upgradeInfo.backupArea2.version;
	else
		memset((uint8_t *)fp_version,0,sizeof(version_t));
}

void GetCtrAreaVersion(version_t *fp_version)
{
	if(gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.backupAreaNum == 1 || gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.backupAreaNum == 2)
		*fp_version = gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.version;
	else
		memset((uint8_t *)fp_version,0,sizeof(version_t));
}

void GetAppAreaVersion(version_t *fp_version)
{
	if(gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc == 1 || gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc == 2)
		*fp_version = gs_paramAreaInfo.upgradeInfo.appAreaInfo.version;
	else
		memset((uint8_t *)fp_version,0,sizeof(version_t));
}



//比较两个版本是否相同，0=相同，1=ver1>ver2,-1=ver1<ver2
int VerCmp(version_t f_ver1,version_t f_ver2)
{

	if(f_ver1.type > f_ver2.type)
		return 1;
	if(f_ver1.type < f_ver2.type)
		return -1;
		
	if(f_ver1.major > f_ver2.major)
		return 1;
	if(f_ver1.major < f_ver2.major)
		return -1;

	if(f_ver1.minor > f_ver2.minor)
		return 1;
	if(f_ver1.minor < f_ver2.minor)
		return -1;

	if(f_ver1.build > f_ver2.build)
		return 1;
	if(f_ver1.build < f_ver2.build)
		return -1;

	return 0;

}


void SetAnotherVer(version_t f_version)
{
	gs_anotherVer = f_version;
}
void GetAnotherVer(version_t *fp_version)
{
    if( NULL != fp_version)
        *fp_version = gs_anotherVer;
}
void GetBootloadVer(version_t *fp_version)
{
    if( NULL != fp_version)
    {
        fp_version->type = (gs_paramAreaInfo.cfgInfo.cfgParam1>>24)&0xFF;
        fp_version->major = (gs_paramAreaInfo.cfgInfo.cfgParam1>>16)&0xFF;
        fp_version->minor = (gs_paramAreaInfo.cfgInfo.cfgParam1>>8)&0xFF;
        fp_version->build = (gs_paramAreaInfo.cfgInfo.cfgParam1>>0)&0xFF;
    }
}

uint8_t GetOtaingFlag(void)
{
    return m_otaing_flag;
}


//0=成功，
int InitSoftwareFlashOp(void)
{
	int ret = 0;
	version_t version;
	backupAreaInfo_t *p_backupAreaInfo = NULL;
	uint32_t crc;
	uint16_t f_sector = 0;
	uint16_t f_num_sectors = 0;

	ret = GetAllDataFromParamArea(&gs_paramAreaInfo);
//	printf("ret = %d\r\n",ret);
//	printf("codeSrc = %d\r\n",gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc);
//	printf("hasCodeFlag1 = %d\r\n",gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag);
//	printf("hasCodeFlag2 = %d\r\n",gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag);
	
//	if(ret == 0)
	{
		if(ret == 0 && gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc == 1)
		{
//		printf("hasCodeFlag2 = %d\r\n",gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag);
			if(gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag == 1)
				p_backupAreaInfo = &gs_paramAreaInfo.upgradeInfo.backupArea2;
		}
		else if(ret == 0 && gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc == 2)
		{
//		printf("hasCodeFlag1 = %d\r\n",gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag);
			if(gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag == 1)
				p_backupAreaInfo = &gs_paramAreaInfo.upgradeInfo.backupArea1;
		}
		else 
		{
			//当前版本不在两个备份区，则把两个备份区信息清除
			memset((uint8_t *)&gs_paramAreaInfo.upgradeInfo.backupArea1,0,sizeof(backupAreaInfo_t));
			memset((uint8_t *)&gs_paramAreaInfo.upgradeInfo.backupArea2,0,sizeof(backupAreaInfo_t));
			
			
			gs_paramAreaInfo.upgradeInfo.appAreaInfo.version.type  = MAIN_VERSION_TYPE;
			gs_paramAreaInfo.upgradeInfo.appAreaInfo.version.major = MAIN_VERSION_MAJOR;
			gs_paramAreaInfo.upgradeInfo.appAreaInfo.version.minor = MAIN_VERSION_MINOR;
			gs_paramAreaInfo.upgradeInfo.appAreaInfo.version.build = MAIN_VERSION_BUILD;
			
			if(gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag != 1)//当前版本不在备份区且备份区空
			{
				f_sector = BACKUP_AREA_1_SECTOR_1_NUM;
				f_num_sectors = BACKUP_AREA_1_SIZE/SECTOR_SIZE;
				//擦除扇区
                ret = FlashEraseSector(f_sector, f_num_sectors);
//				printf("FlashEraseSector ret = %d\r\n",ret);
				if(ret == 0)//查擦成功
				{
					//进入临界区
					__disable_irq();
					//解锁
//					HAL_FLASH_Unlock();
					EFM_FWMC_Cmd(ENABLE);
					EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, ENABLE);

					ret = WriteWordsToFlash(BACKUP_AREA_1_SECTOR_1_ADDR,(uint32_t *)VECTOR_TABLE_OFFSET_ADDR, 0x10000);
//					printf("WriteWordsToFlash ret = %d\r\n",ret);
					//上锁
//					HAL_FLASH_Lock();
					EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, DISABLE);
					EFM_FWMC_Cmd(DISABLE);

					//退出临界区
					__enable_irq();
					
					crc = SWCalcBlockCRC((uint32_t *)VECTOR_TABLE_OFFSET_ADDR, 0x10000);
//					printf("crc = %d\r\n",crc);
					if(crc == SWCalcBlockCRC((uint32_t *)BACKUP_AREA_1_SECTOR_1_ADDR, 0x10000))//crc相同说明复制成功
					{
//					printf("backup1 crc ok\r\n");
                        //{FOR TEST BOOTLOAD OTA 
//						gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.UpgradeFlag = 1;
//                        gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.backupAreaNum = 1;
//                        gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.state = 2;
//                        gs_paramAreaInfo.upgradeInfo.upgradeControlInfo.version = gs_paramAreaInfo.upgradeInfo.appAreaInfo.version;
                        //}

						gs_paramAreaInfo.upgradeInfo.backupArea1.codeHwCRC = crc;
						gs_paramAreaInfo.upgradeInfo.backupArea1.state = 1;
						gs_paramAreaInfo.upgradeInfo.backupArea1.hasCodeFlag = 1;
						gs_paramAreaInfo.upgradeInfo.backupArea1.codeSize = 256 * 1024;//256K
						memcpy((uint8_t *)&gs_paramAreaInfo.upgradeInfo.backupArea1.version,\
							(uint8_t *)&gs_paramAreaInfo.upgradeInfo.appAreaInfo.version,\
						sizeof(version_t));

						gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc = 1;
						p_backupAreaInfo = &gs_paramAreaInfo.upgradeInfo.backupArea2;
						
						UpdateParamAreaData();
					}
					
				}
			}
			else if(gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag != 1)//当前版本不在备份区且备份区空
			{
				f_sector = BACKUP_AREA_2_SECTOR_1_NUM;
				f_num_sectors = BACKUP_AREA_2_SIZE/SECTOR_SIZE;
				//擦除扇区
            	ret = FlashEraseSector(f_sector, f_num_sectors);
				if(ret == 0)//查擦成功
				{
					//进入临界区
					__disable_irq();
					//解锁
//					HAL_FLASH_Unlock();
					EFM_FWMC_Cmd(ENABLE);
					EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, ENABLE);

					ret = WriteWordsToFlash(BACKUP_AREA_2_SECTOR_1_ADDR,(uint32_t *)VECTOR_TABLE_OFFSET_ADDR, 0x10000);
					//上锁
//					HAL_FLASH_Lock();
					EFM_SequenceSectorOperateCmd(f_sector, f_num_sectors, DISABLE);
					EFM_FWMC_Cmd(DISABLE);

					//退出临界区
					__enable_irq();
					
					crc = SWCalcBlockCRC((uint32_t *)VECTOR_TABLE_OFFSET_ADDR, 0x10000);
					if(crc == SWCalcBlockCRC((uint32_t *)BACKUP_AREA_2_SECTOR_1_ADDR, 0x10000))//crc相同说明复制成功
					{
						
						gs_paramAreaInfo.upgradeInfo.backupArea2.codeHwCRC = crc;
						gs_paramAreaInfo.upgradeInfo.backupArea2.state = 1;
						gs_paramAreaInfo.upgradeInfo.backupArea2.hasCodeFlag = 1;
						gs_paramAreaInfo.upgradeInfo.backupArea2.codeSize = 256 * 1024;//256K
						memcpy((uint8_t *)&gs_paramAreaInfo.upgradeInfo.backupArea2.version,\
							(uint8_t *)&gs_paramAreaInfo.upgradeInfo.appAreaInfo.version,\
						sizeof(version_t));

						gs_paramAreaInfo.upgradeInfo.appAreaInfo.codeSrc = 2;
						p_backupAreaInfo = &gs_paramAreaInfo.upgradeInfo.backupArea1;
						
						UpdateParamAreaData();
					}
					
				}
			}
		}
	}
    printf( "current ver: %d.%d.%d.%d\n", MAIN_VERSION_TYPE, MAIN_VERSION_MAJOR, MAIN_VERSION_MINOR, MAIN_VERSION_BUILD );
	if(p_backupAreaInfo != NULL)
	{
		SetAnotherVer(p_backupAreaInfo->version);
		printf("\n");
		version = gs_paramAreaInfo.upgradeInfo.appAreaInfo.version;
		printf("appVer:%d.%d.%d.%d\n",version.type,version.major,version.minor,version.build);
		version = gs_paramAreaInfo.upgradeInfo.backupArea1.version;
		printf("backup1Ver:%d.%d.%d.%d\n",version.type,version.major,version.minor,version.build);
		version = gs_paramAreaInfo.upgradeInfo.backupArea2.version;
		printf("backup2Ver:%d.%d.%d.%d\n",version.type,version.major,version.minor,version.build);
		version = p_backupAreaInfo->version;
		printf("anotherVer:%d.%d.%d.%d\n",version.type,version.major,version.minor,version.build);
	}
	else
	{
		printf("backup area is empty\n");
		
	}
	//gs_ta = gc_test[sizeof(gc_test)-1];
		
	return 0;
}

