#ifndef __FLASH_OP_H_
#define __FLASH_OP_H_

#include "main.h"
#include "version.h"

/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/

//#define VECTOR_TABLE_OFFSET_ADDR 0x08040000
#define VECTOR_TABLE_OFFSET_ADDR 0x00040000

//参数区域所处的扇区
#define PARAM_AREA_SECTOR_NUM 6
//参数区域的起始地址
#define PARAM_AREA_ADDR 0x0000C000

//参数长度区域的地址
#define PARAM_LEN_AREA_ADDR PARAM_AREA_ADDR
//参数长度区域的大小
#define PARAM_LEN_AREA_SIZE sizeof(paramAreaLength_t)
	
//升级信息区域的地址
#define UPGRADE_INFO_AREA_ADDR (PARAM_LEN_AREA_ADDR + PARAM_LEN_AREA_SIZE)
//升级信息区域的大小
#define UPGRADE_INFO_AREA_SIZE sizeof(upgradeInfo_t)
	
//配置信息区域的地址
#define CFG_INFO_AREA_ADDR (UPGRADE_INFO_AREA_ADDR + UPGRADE_INFO_AREA_SIZE)
//配置信息区域的大小
#define CFG_INFO_AREA_SIZE sizeof(cfgInfo_t)
	
//参数数据CRC区域的地址，参数的最后4个字节，记录前面所有参数数据的CRC
#define PARAM_CRC_AREA_ADDR (CFG_INFO_AREA_ADDR + CFG_INFO_AREA_SIZE)
//参数数据CRC区域的大小
#define PARAM_CRC_AREA_SIZE 0x04

//参数区域的总大小
#define PARAM_AREA_SIZE sizeof(paramAreaInfo_t)//(PARAM_LEN_AREA_SIZE + UPGRADE_INFO_AREA_SIZE + CFG_INFO_AREA_SIZE + PARAM_CRC_AREA_SIZE)

//--------------------------------------分割线--------------------------------------------------//
//备份区
#define BACKUP_AREA_1_ADDR 0x00080000
#define BACKUP_AREA_1_SIZE 0x40000

#define BACKUP_AREA_1_SECTOR_1_ADDR BACKUP_AREA_1_ADDR
#define BACKUP_AREA_1_SECTOR_1_NUM 64
#define BACKUP_AREA_1_SECTOR_1_SIZE 0x20000

#define BACKUP_AREA_1_SECTOR_2_ADDR 0x000A0000
#define BACKUP_AREA_1_SECTOR_2_NUM 80
#define BACKUP_AREA_1_SECTOR_2_SIZE 0x20000



#define BACKUP_AREA_2_ADDR 0x000C0000
#define BACKUP_AREA_2_SIZE 0x40000

#define BACKUP_AREA_2_SECTOR_1_ADDR BACKUP_AREA_2_ADDR
#define BACKUP_AREA_2_SECTOR_1_NUM 96
#define BACKUP_AREA_2_SECTOR_1_SIZE 0x20000

#define BACKUP_AREA_2_SECTOR_2_ADDR 0x000E0000
#define BACKUP_AREA_2_SECTOR_2_NUM 112
#define BACKUP_AREA_2_SECTOR_2_SIZE 0x20000

#define UPGRADE_PKG_SIZE 256 //升级包中，一包数据的最大值是256字节


/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/

//写入flash中的数据以4字节对齐，方便使用硬件crc做校验
#pragma pack()

//typedef struct{
//	uint32_t type;
//	uint32_t major;
//	uint32_t minor;
//	uint32_t build;	
//}version_t;





/*
参数区第一部分(长度固定），标明参数区有效数据的长度
参数区第二部分(长度固定），标明升级相关的信息
参数区第三部分(长度不固定），标明配置的信息

上面的第一第二部分在bootloader2中也会用到，所以长度和形式一旦定下来，尽量不要去修改，否则将需要对bootloader2代码做改动
*/
typedef struct{
	uint32_t len;
	uint32_t crcCheck;
}paramAreaLength_t;



/*
备份区信息:
	1，标记，标明该区域是否有代码
	2，版本信息，标明该区域代码的版本
	3，代码总量
	4，状态，标明该区域代码是否正常，如果有问题，具体标出问题

*/
typedef struct{
	uint32_t hasCodeFlag;//是否有代码，0=无，1=有
	version_t version;
	uint32_t codeHwCRC;//完整代码的硬件CRC
	uint32_t codeSize;//字节为单位
	uint32_t state;//备份区的状态,1=正常,2=数据写入中,0=错误或者初始状态

}backupAreaInfo_t;

/*
升级控制信息:
	1，标记，标明是否需要升级
	2，需要更新的版本号
	3，需要更新的备份区域
*/

typedef struct{
	uint32_t UpgradeFlag;//0=不需要，1=需要
	version_t version; 
	uint32_t backupAreaNum;//1-backup1 2-backup2
	uint32_t state;//升级的状态,1=成功写入app，2=等待写入app，
}upgradeControlInfo_t;

/*
APP区域的信息:
	1，当前版本号
	2，代码的来源（备份区1，2或者其他）
*/

typedef struct{
	version_t version; 
	uint32_t codeSrc;//1-backup1 2-backup2,0-其他
}appAreaInfo_t;



/*
升级相关的信息:
	1，两个备份区的信息
	2，标记，标明是否需要复制备份区的代码到APP区
	3，需要更新的版本号
	4，需要更新的备份区域

*/

typedef struct{
	backupAreaInfo_t backupArea1;
	backupAreaInfo_t backupArea2;
	upgradeControlInfo_t upgradeControlInfo;
	appAreaInfo_t appAreaInfo;
}upgradeInfo_t;


/*
参数区域配置信息的数据：
*/
typedef struct{
	uint32_t cfgParam1;
	uint32_t cfgParam2;
	
}cfgInfo_t;

/*
参数区域的数据：
	1，参数区数据的总长度
	2，升级相关的信息
	3，配置信息
	4，参数区域所有数据的crc校验
*/
typedef struct{
	paramAreaLength_t paramAreaLength;
	upgradeInfo_t upgradeInfo;
	cfgInfo_t cfgInfo;
	
	uint32_t crcCheck;
}paramAreaInfo_t;


typedef struct{
	uint32_t getAllMsgPkgFlag;//1=获取到了所有正确的数据包
	uint32_t msgPkgNum;//已获取到的有效数据包最新编号，从1开始，顺序递增
	uint32_t hwCRC;//完整文件的硬件CRC
	uint32_t dataLen;//已写入flash的数据的总长度
	uint8_t saveData[1024];//暂存数据包，到1k后一次性写入
	uint32_t saveDataIndex;//写入暂存区的偏移量,也是暂存区已保存的数据量
	uint32_t saveCRC[256];//每1k数据进行一次硬件crc并保存在这里
	uint32_t saveCRCIndex;//写入CRC的偏移量，也是已经保存的crc的个数
	uint8_t sendCANDataIndex;
	uint32_t sendCANDataNum;

}upgradeProcessInfo_t;

typedef struct{
	uint8_t	can_order_ack_falg;
	uint8_t can_data_ack_falg;
	uint8_t can_error_ack_falg;
	uint8_t can_end_ack_flag;
	uint16_t can_ack_timeout;
	uint8_t can_roll_back_flag;
	uint8_t can_ver_ack_flag;
}canUpgradeAckInfo_t;

/************************************************************************************************************************************
																变量extern声明
*************************************************************************************************************************************/


/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口声明
*************************************************************************************************************************************/



/************************************************************************************************************************************
											对外提供的任务、硬件初始化、软件初始化声明
*************************************************************************************************************************************/
uint32_t GetAllDataFromParamArea(paramAreaInfo_t *fp_paramAreaInfo);
uint32_t GetAllDataFromParamAreaByLen(uint32_t *fp_buf, uint32_t f_bufLen);
//硬件计算，算法：CRC-32/MPEG-2算法
uint32_t SWCalcBlockCRC(uint32_t fp_buf[], uint32_t f_bufLen);
uint32_t SWCalcCRC(uint32_t f_data);

int WriteWordsToFlash(uint32_t f_address,uint32_t *fp_wordData, uint32_t f_len);
int WriteParamDataToParamArea(const paramAreaInfo_t *fp_paramAreaInfo);
int HandleUpgradeData(int32_t f_msgPkgNum,uint32_t f_dataLen,uint8_t *fp_dataBuf,uint32_t f_dataCRC);
int UpdateParamAreaData(void);
uint32_t GetUpgradeDataRecState(void);
void SetUpgradeDataRecState(uint32_t f_state);
void GetBackup1Version(version_t *fp_version);
void GetBackup2Version(version_t *fp_version);
void GetCtrAreaVersion(version_t *fp_version);
void GetAppAreaVersion(version_t *fp_version);
void SetAnotherVer(version_t f_version);
void GetAnotherVer(version_t *fp_version);
void GetBootloadVer(version_t *fp_version);
int VerCmp(version_t f_ver1,version_t f_ver2);
uint8_t GetOtaingFlag(void);


int InitHardwareFlashOp(void);

int InitSoftwareFlashOp(void);


#endif
