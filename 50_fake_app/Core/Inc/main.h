/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define TRUE		1
#define	FALSE		0
//#define ApplicationAddress    0x8020000
#define	PACK_SIZE							0x400
#define	APP_Sector_1					ADDR_FLASH_SECTOR_0		//主程序区
#define	APP_Sector_2					ADDR_FLASH_SECTOR_12		//备份区
#define	APP_Sector_3					ADDR_FLASH_SECTOR_13		//备份区
#define	APP_Sector_4					ADDR_FLASH_SECTOR_1		//备份区
#define	Data_Version_Sector		ADDR_FLASH_SECTOR_2		//程序信息区
#define	BootLoader_Sector			ADDR_FLASH_SECTOR_0		//bootloader缓存区

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base address of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base address of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */
#define ADDR_FLASH_SECTOR_12    ((uint32_t)0x08025000) /* Base address of Sector 11, 128 Kbytes */
#define ADDR_FLASH_SECTOR_13    ((uint32_t)0x08029000) /* Base address of Sector 11, 128 Kbytes */
typedef  void (*pFunction)(void);

typedef enum
{
	Wait,Updating,Copying,Rollback,Fault
}SysStatus_t;
extern SysStatus_t SysState;
typedef struct
{
	uint8_t RollBackFlg;
	uint8_t EraseFlag;
	uint8_t Erase_finished;
	uint8_t WriteFlag;
	uint8_t	Back_finishFlag;
	uint8_t version_flag;
	uint16_t PackNum;
	uint16_t SizeOfArray;
	uint16_t BackUp_PackNum;
	uint32_t Code_Ver;
	uint32_t ByteNum;
	uint32_t BackUp_ByteNum;
}OTA_TypeDef;
extern OTA_TypeDef OTA_Par;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
uint8_t APP_Backup(uint32_t App_addr,uint32_t back_up_addr,uint32_t byte_num,uint8_t pack_num);
void CANx_SendNormalData(CAN_HandleTypeDef *hcanX,uint16_t ID,uint8_t *pData, uint16_t Len);
void SoftReset(void);
void SendErrorCode(uint8_t ErrorCode,uint8_t CodeNum);
uint8_t Jump_to_APP(uint32_t APP_addr);
uint8_t Flash_Write(uint32_t Address,uint8_t *data,uint32_t data_len);
uint8_t Flash_Write_long(uint32_t Address,uint32_t *data,uint32_t data_len);
uint8_t Erase_Sector(uint32_t StartAddress, uint32_t data_len);
int FlashEraseSector( uint32_t f_sector, uint8_t f_num_sectors );
uint8_t WriteData_toFlash(uint8_t *data,uint32_t len,uint32_t address);
uint8_t WriteUint32_tData_toFlash(uint32_t *data,uint32_t len,uint32_t address);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
