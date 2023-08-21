#include "save_alarm.h"
#include "flash_op.h"
//stm32头文件
#include "stm32f4xx_flash.h"
#include "socket.h"
#include <string.h>


extern const SOCKET gc_cmdLineSocket;
paramBody_t g_paramBody;
paramSectorInfo_t g_paramSectorInfo;
netDisconnectInfo_t g_netDisconnectInfo = {PARAM_KEY_NET_DISCONNECT,0,0,0,0};
portDisconnectInfo_t g_port1DisconnectInfo = {PARAM_KEY_PORT1_DISCONNECT,0,0,0};
portDisconnectInfo_t g_port2DisconnectInfo = {PARAM_KEY_PORT2_DISCONNECT,0,0,0};
portDisconnectInfo_t g_port3DisconnectInfo = {PARAM_KEY_PORT3_DISCONNECT,0,0,0};
portDisconnectInfo_t g_port4DisconnectInfo = {PARAM_KEY_PORT4_DISCONNECT,0,0,0};
portDisconnectInfo_t g_routerDisconnectInfo = {PARAM_KEY_ROUTER_DISCONNECT,0,0,0};
timeCalibration_t g_timeCalibration = {PARAM_KEY_TIME_CALIBRATION,0,0};
testInfo_t g_testInfo = {PARAM_KEY_TEST,0,0,0,0,0};

int GetParamSectorInfo(paramBody_t *fp_paramBody, paramSectorInfo_t *fp_paramSectorInfo)
{
	int i;
	uint32_t startAddr = PARAM_SECTOR_ADDR_START;
	uint32_t addr;
	uint32_t seqNumb = 0;
	uint32_t len = 0;
	uint32_t hwSrcCRC = 0;
	uint32_t hwCRC = 0;
	
	startAddr = PARAM_SECTOR_ADDR_START;
	
	while(1)
	{
		addr = startAddr;
		
		seqNumb = *((uint32_t *)addr);
		addr += 4;
		len = *((uint32_t *)addr);
		addr += 4;
		

		if(seqNumb == 0xFFFFFFFF && len == 0Xffffffff)//未使用的空间
		{
			fp_paramSectorInfo->lenAll = startAddr - PARAM_SECTOR_ADDR_START;
			fp_paramSectorInfo->startAddr = startAddr;
			return 0;
		}
		else if(len > PARAM_DATA_SIZE)
			return -4;
		else if(seqNumb == 0 && len == 0)//向后寻找0xAA775533
		{
			while(1)
			{
				seqNumb = *((uint32_t *)addr);
				if(seqNumb == 0)
					addr += 4;
				else if(seqNumb == 0xFFFFFFFF)//理论上不会出现这种情况
				{
					fp_paramSectorInfo->lenAll = addr - PARAM_SECTOR_ADDR_START;
					fp_paramSectorInfo->startAddr = addr;
					return -3;
				}
				else if(seqNumb == 0xAA775533)
				{
					addr += 4;
					break;
				}
				else //理论上不会出现这种情况
					return -2;
			}
			startAddr = addr;
			continue;
			
		}
		
		
		hwCRC = SWCalcBlockCRC((uint32_t *)startAddr, (len + 8)/4);
		addr += len;
		hwSrcCRC = *((uint32_t *)addr);
		addr += 4;
		if(hwSrcCRC == hwCRC)
		{
			fp_paramSectorInfo->lastStartAdd = startAddr;
			fp_paramSectorInfo->lastLen = len;
			fp_paramSectorInfo->lenAll = addr - PARAM_SECTOR_ADDR_START;
			fp_paramSectorInfo->startAddr = addr; 
			
			fp_paramBody->seqNumb = seqNumb+1;
		}
		else//异常，未知的问题
		{
			memset((uint8_t *)fp_paramSectorInfo,0,sizeof(paramSectorInfo_t));
			return -1;
		}
		startAddr = addr;
	}
	return -10;
}


int SaveParamInfo(paramBody_t *fp_paramBody, paramSectorInfo_t *fp_paramSectorInfo)
{
	int ret = 0;
	uint32_t hwCRC = 0;
	uint8_t zeroData[PARAM_DATA_SIZE+12] = {0};
	const uint32_t c_temp = 0xAA775533;
	
	if(fp_paramBody->len > PARAM_DATA_SIZE)
		return -101;
	
	if((12 + fp_paramBody->len + fp_paramSectorInfo->startAddr) > PARAM_SECTOR_ADDR_END)
		ClearParamSector(fp_paramBody,fp_paramSectorInfo);
	
	fp_paramBody->hwCRC = SWCalcBlockCRC((uint32_t *)fp_paramBody, (fp_paramBody->len + 8)/4);
	
	memcpy(fp_paramBody->data+fp_paramBody->len,(uint8_t *)&fp_paramBody->hwCRC,4);
	
	if(fp_paramSectorInfo->startAddr < PARAM_SECTOR_ADDR_START || fp_paramSectorInfo->startAddr > PARAM_SECTOR_ADDR_END)
		return -100;
	
	//进入临界区
	__disable_irq();
	//解锁
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
				FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	ret = WriteWordsToFlash(fp_paramSectorInfo->startAddr,(uint32_t *)fp_paramBody, (fp_paramBody->len+12)/4);
	//上锁
	FLASH_Lock();
	//退出临界区
	__enable_irq();
	
	if(ret != 0)
		return ret;
	else
	{
		hwCRC = SWCalcBlockCRC((uint32_t *)fp_paramSectorInfo->startAddr, (fp_paramBody->len + 8)/4);
		if(fp_paramBody->hwCRC != hwCRC)//写入出错，清除该区域
		{
			//进入临界区
			__disable_irq();
			//解锁
			FLASH_Unlock();
			FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
				FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
			ret = WriteWordsToFlash(fp_paramSectorInfo->startAddr,(uint32_t *)zeroData, (fp_paramBody->len+12)/4);
			fp_paramSectorInfo->startAddr += fp_paramBody->len+12;
			FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
				FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
			ret = WriteWordsToFlash(fp_paramSectorInfo->startAddr,(uint32_t *)&c_temp, 1);
			//上锁
			FLASH_Lock();
			//退出临界区
			__enable_irq();
			
			fp_paramSectorInfo->startAddr += 4;
			fp_paramSectorInfo->lenAll = fp_paramSectorInfo->startAddr - PARAM_SECTOR_ADDR_START;
			
			return -102;
		}
		else
		{
			fp_paramSectorInfo->lastStartAdd = fp_paramSectorInfo->startAddr;
			fp_paramSectorInfo->lastLen = fp_paramBody->len+12;
			fp_paramSectorInfo->startAddr += fp_paramBody->len+12;
			fp_paramSectorInfo->lenAll = fp_paramSectorInfo->startAddr - PARAM_SECTOR_ADDR_START;
		}
	}
	
}


int ShowAllParamInfo(paramBody_t *fp_paramBody, paramSectorInfo_t *fp_paramSectorInfo)
{
	
	int i;
	uint32_t startAddr = PARAM_SECTOR_ADDR_START;
	uint32_t addr;
	uint32_t seqNumb = 0;
	uint32_t len = 0;
	uint32_t hwSrcCRC = 0;
	uint32_t hwCRC = 0;
	char *p_exitShow = "exit show!\r\n";
	
	startAddr = PARAM_SECTOR_ADDR_START;
	
	while(1)
	{
		addr = startAddr;
		
		seqNumb = *((uint32_t *)addr);
		addr += 4;
		len = *((uint32_t *)addr);
		addr += 4;
		
		
		if(seqNumb == 0xFFFFFFFF && len == 0Xffffffff)//未使用的空间
		{
			send(gc_cmdLineSocket,(uint8_t *)p_exitShow,strlen(p_exitShow));
			return 0;
		}
		else if(len > PARAM_DATA_SIZE)
			return -4;
		else if(seqNumb == 0 && len == 0)//向后寻找0xAA775533
		{
			while(1)
			{
				seqNumb = *((uint32_t *)addr);
				if(seqNumb == 0)
					addr += 4;
				else if(seqNumb == 0xFFFFFFFF)//理论上不会出现这种情况
				{
					return -3;
				}
				else if(seqNumb == 0xAA775533)
				{
					addr += 4;
					break;
				}
				else //理论上不会出现这种情况
					return -2;
			}
			startAddr = addr;
			continue;
			
		}
		
		
		hwCRC = SWCalcBlockCRC((uint32_t *)startAddr, (len + 8)/4);
		addr += len;
		hwSrcCRC = *((uint32_t *)addr);
		addr += 4;
		if(hwSrcCRC == hwCRC)
		{
			ShwoParamInfo((paramBody_t *)startAddr);
		}
		else//异常，未知的问题
		{
			return -1;
		}
		startAddr = addr;
	}
	return -10;
}


int ShwoParamInfo(paramBody_t *fp_paramBody)
{
	char data[128] = {0};
	uint32_t index = 0;
	uint8_t key;
	
	sprintf(data,"power on %d times:\r\n",fp_paramBody->seqNumb);
	send(gc_cmdLineSocket,(uint8_t *)data,strlen(data));
	
	while(1)
	{
		key = fp_paramBody->data[index];
		memset(data,0,sizeof(data));
		switch(key)
		{
			case PARAM_KEY_NET_DISCONNECT:
			{
				netDisconnectInfo_t *p_info = (netDisconnectInfo_t *)(fp_paramBody->data+index);
				sprintf(data,"net flag=%d,rt=%d,start=%d,end=%d\r\n",p_info->flag,p_info->resetTimes,p_info->start,p_info->end);
				
				send(gc_cmdLineSocket,(uint8_t *)data,strlen(data));
				
				index += sizeof(netDisconnectInfo_t);
			}
			break;
			case PARAM_KEY_PORT1_DISCONNECT:
			case PARAM_KEY_PORT2_DISCONNECT:
			case PARAM_KEY_PORT3_DISCONNECT:
			case PARAM_KEY_PORT4_DISCONNECT:
			case PARAM_KEY_ROUTER_DISCONNECT:
			{
				portDisconnectInfo_t *p_info = (portDisconnectInfo_t *)(fp_paramBody->data+index);
				
				if(key == PARAM_KEY_PORT1_DISCONNECT)
					sprintf(data,"port1 flag=%d,start=%d,end=%d\r\n",p_info->flag,p_info->start,p_info->end);
				else if(key == PARAM_KEY_PORT2_DISCONNECT)
					sprintf(data,"port2 flag=%d,start=%d,end=%d\r\n",p_info->flag,p_info->start,p_info->end);
				else if(key == PARAM_KEY_PORT3_DISCONNECT)
					sprintf(data,"port3 flag=%d,start=%d,end=%d\r\n",p_info->flag,p_info->start,p_info->end);
				else if(key == PARAM_KEY_PORT4_DISCONNECT)
					sprintf(data,"port4 flag=%d,start=%d,end=%d\r\n",p_info->flag,p_info->start,p_info->end);
				else if(key == PARAM_KEY_ROUTER_DISCONNECT)
					sprintf(data,"router flag=%d,start=%d,end=%d\r\n",p_info->flag,p_info->start,p_info->end);
					
				send(gc_cmdLineSocket,(uint8_t *)data,strlen(data));
				
				index += sizeof(portDisconnectInfo_t);
			}
			break;
			case PARAM_KEY_TEST:
			{
				testInfo_t *p_info = (testInfo_t *)(fp_paramBody->data+index);
				sprintf(data,"test u1=%d,u2=%d,u3=%d,u4=%d,u5=%d\r\n",p_info->unit1,p_info->unit2,p_info->unit3,p_info->unit4,p_info->unit5);
				
				send(gc_cmdLineSocket,(uint8_t *)data,strlen(data));
				
				index += sizeof(netDisconnectInfo_t);
			}
			break;
			case PARAM_KEY_TIME_CALIBRATION:
			{
				timeCalibration_t *p_info = (timeCalibration_t *)(fp_paramBody->data+index);
				sprintf(data,"mcu time=%d,pc time=%d\r\n",p_info->mcuTime,p_info->pcTime);
				
				send(gc_cmdLineSocket,(uint8_t *)data,strlen(data));
				
				index += sizeof(timeCalibration_t);
			}
			break;
			default:
				sprintf(data,"unknown param key\r\n");
				send(gc_cmdLineSocket,(uint8_t *)data,strlen(data));
				return -1;
			break;
		}
		
		if(index >= fp_paramBody->len)
			return 0;
	}
}

int AddInfo(paramBody_t *fp_paramBody,void *fp_data)
{
	uint8_t key;
	key = ((uint8_t*)fp_data)[0];
	switch(key)
	{
		case PARAM_KEY_NET_DISCONNECT:
		{
			netDisconnectInfo_t *p_info = (netDisconnectInfo_t *)fp_data;
			if(fp_paramBody->len + sizeof(netDisconnectInfo_t) <= PARAM_DATA_SIZE)
			{
				memcpy(fp_paramBody->data+fp_paramBody->len,(uint8_t *)p_info,sizeof(netDisconnectInfo_t));
				fp_paramBody->len += sizeof(netDisconnectInfo_t);
			}
			else
				return -1;
		}
		break;
		case PARAM_KEY_PORT1_DISCONNECT:
		case PARAM_KEY_PORT2_DISCONNECT:
		case PARAM_KEY_PORT3_DISCONNECT:
		case PARAM_KEY_PORT4_DISCONNECT:
		case PARAM_KEY_ROUTER_DISCONNECT:
		{
			portDisconnectInfo_t *p_info = (portDisconnectInfo_t *)fp_data;
			if(fp_paramBody->len + sizeof(portDisconnectInfo_t) <= PARAM_DATA_SIZE)
			{
				memcpy(fp_paramBody->data+fp_paramBody->len,(uint8_t *)p_info,sizeof(portDisconnectInfo_t));
				fp_paramBody->len += sizeof(portDisconnectInfo_t);
			}
			else
				return -1;
		}
		break;
		case PARAM_KEY_TEST:
		{
			testInfo_t *p_info = (testInfo_t *)fp_data;
			if(fp_paramBody->len + sizeof(testInfo_t) <= PARAM_DATA_SIZE)
			{
				memcpy(fp_paramBody->data+fp_paramBody->len,(uint8_t *)p_info,sizeof(testInfo_t));
				fp_paramBody->len += sizeof(testInfo_t);
			}
			else
				return -1;
		}
		break;
		case PARAM_KEY_TIME_CALIBRATION:
		{
			timeCalibration_t *p_info = (timeCalibration_t *)fp_data;
			if(fp_paramBody->len + sizeof(timeCalibration_t) <= PARAM_DATA_SIZE)
			{
				memcpy(fp_paramBody->data+fp_paramBody->len,(uint8_t *)p_info,sizeof(netDisconnectInfo_t));
				fp_paramBody->len += sizeof(netDisconnectInfo_t);
			}
			else
				return -1;
		}
		break;
		default:
			return -1;
		break;
	}
	
}

int ClearParamSector(paramBody_t *fp_paramBody, paramSectorInfo_t *fp_paramSectorInfo)
{
	int ret = 0;
	//进入临界区
	__disable_irq();
	//1，解锁
	FLASH_Unlock();
	//2，擦除扇区
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
				FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	if(FLASH_COMPLETE != FLASH_EraseSector(PARAM_SECTOR_NUM, VoltageRange_3))
		ret = -1;
	//3，上锁
	FLASH_Lock();
	//退出临界区
	__enable_irq();

	
	fp_paramSectorInfo->lenAll = 0;
	fp_paramSectorInfo->lastStartAdd = 0;
	fp_paramSectorInfo->startAddr = PARAM_SECTOR_ADDR_START;
	fp_paramSectorInfo->lastLen = 0;
	
	return ret;
	
}
