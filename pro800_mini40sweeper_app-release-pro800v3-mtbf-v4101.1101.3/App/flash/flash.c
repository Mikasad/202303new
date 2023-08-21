
#include "flash.h"
static uint32_t GetSector(uint32_t Address);
uint32_t FlashReadWord(uint32_t addr)
{
	uint32_t flash_data;
	flash_data = *(uint32_t*)addr;
//	FLASH_Lock();	
	return flash_data;
}

uint8_t FlashReadByte(uint32_t addr)
{
	uint32_t flash_data;
	flash_data = *(uint8_t*)addr;
//	FLASH_Lock();	
	return flash_data;
}

void FlashWriteWord(uint32_t addr,uint32_t * data)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
						FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
  while( FLASH_ProgramWord(addr,*data) != FLASH_COMPLETE );
//	while( FLASH_ProgramWord(addr+4,*(data+1)) != FLASH_COMPLETE );
	FLASH_Lock();
}

void FlashWriteByte(uint32_t addr,uint8_t * data)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
						FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
  while( FLASH_ProgramByte(addr,*data) != FLASH_COMPLETE );
//	while( FLASH_ProgramWord(addr+4,*(data+1)) != FLASH_COMPLETE );
	FLASH_Lock();
}

void FlashErase(uint32_t addr)
{
	uint32_t start_sector = 0;
	FLASH_Unlock();
	start_sector = GetSector(addr);
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
						FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	while(FLASH_EraseSector(start_sector, VoltageRange_3) != FLASH_COMPLETE);
	FLASH_Lock();
}

/**
  * @brief  根据输入的地址给出它所在的sector
  *					例如：
						uwStartSector = GetSector(FLASH_USER_START_ADDR);
						uwEndSector = GetSector(FLASH_USER_END_ADDR);	
  * @param  Address：地址
  * @retval 地址所在的sector
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else 
	{
		sector = FLASH_Sector_11;  	
  }

  return sector;
}

