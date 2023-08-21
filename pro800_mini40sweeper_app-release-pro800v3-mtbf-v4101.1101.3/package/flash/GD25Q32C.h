#ifndef __GD25Q32C_H_
#define __GD25Q32C_H_

//标准头文件
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define PREPARE_READ_TIMEOUT	5
#define PREPARE_WRITE_OR_ERASE_TIMEOUT	5
#define WRITE_COMPLETE_TIMEOUT	5
#define SECTOR_ERASE_COMPLETE_TIMEOUT	310


typedef struct{
	uint8_t mID;
	uint8_t dID;
	uint8_t type;
	uint8_t capacity;	
}GD25Q16CInfo_t;

extern GD25Q16CInfo_t g_GD25Q16CInfo;

void GetID_90(uint8_t *fp_mID, uint8_t *fp_dID);
void GetID_9F(uint8_t *fp_mID, uint8_t *fp_type, uint8_t *fp_capacity);
void ReadDataBytes(uint32_t f_srcAddr, uint8_t *fp_dataBuf, uint32_t f_len);
int WriteData(uint32_t f_destAddr, uint8_t *fp_data, uint32_t f_len);
int PrepareToWriteOrErase(uint32_t f_timeOutMs);
int WriteOrEraseComplete(uint32_t f_timeOutMs);
int PrepareToReadData(uint32_t f_timeOutMs);
int EraseFlashSector( uint32_t f_addr );
#endif

