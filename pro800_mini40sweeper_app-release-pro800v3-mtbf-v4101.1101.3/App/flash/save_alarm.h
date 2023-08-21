#include <stdint.h>
//检测到钥匙开关关机时保存数据
//0xFFFFFFFF - 结束信号
//写完之后重读CRC错误，清零已写空间。下一个字0xAA775533，之后重新存储。

#define PARAM_SECTOR_ADDR_START 	0x08020000
#define PARAM_SECTOR_ADDR_END 		0x0803FFFF
#define PARAM_SECTOR_SIZE 			0x20000
#define PARAM_SECTOR_NUM 			FLASH_Sector_5

#define PARAM_DATA_SIZE 256

#pragma pack(4)


typedef struct{
	uint32_t seqNumb;	//序号-启动次数，递增  u16
	uint32_t len;		//长度-数据域	u16
	uint8_t data[PARAM_DATA_SIZE];		//数据：n*(key:message)
	uint32_t hwCRC;		//crc（序号-长度-数据）	u32 
}paramBody_t;


//注意第一个字节必须是key  paramKey_t
typedef struct{
	uint8_t key;
	uint8_t flag;//0-直到关机也没有回复；1-恢复了
	uint8_t resetTimes;//w5500重启次数
	uint32_t start;
	uint32_t end;
}netDisconnectInfo_t;

//注意第一个字节必须是key  paramKey_t
typedef struct{
	uint8_t key;
	uint8_t flag;//0-直到关机也没有回复；1-恢复了
	uint32_t start;
	uint32_t end;
}portDisconnectInfo_t;

typedef struct{
	uint8_t key;
	uint32_t mcuTime;
	uint32_t pcTime;
}timeCalibration_t;

typedef struct{
	uint8_t key;
	uint16_t unit1;
	uint8_t unit2;
	uint32_t unit3;
	uint8_t unit4;
	uint16_t unit5;
}testInfo_t;

typedef enum{
	PARAM_KEY_NET_DISCONNECT = 1,	
	PARAM_KEY_TEST = 2,	
	PARAM_KEY_PORT1_DISCONNECT = 3,	
	PARAM_KEY_PORT2_DISCONNECT = 4,	
	PARAM_KEY_PORT3_DISCONNECT = 5,	
	PARAM_KEY_PORT4_DISCONNECT = 6,	
	PARAM_KEY_ROUTER_DISCONNECT = 7,	
	PARAM_KEY_TIME_CALIBRATION = 8,
}paramKey_t;



#pragma pack()

typedef struct{
	uint32_t lenAll;//参数区数据总长度
	uint32_t lastStartAdd;//上次数据的起始地址
	uint32_t lastLen;//上次数据结构的总长度，包括序号和crc
	uint32_t startAddr;	//新数据的起始地址
}paramSectorInfo_t;

extern paramBody_t g_paramBody;
extern paramSectorInfo_t g_paramSectorInfo;
extern netDisconnectInfo_t g_netDisconnectInfo;
extern portDisconnectInfo_t g_port1DisconnectInfo;
extern portDisconnectInfo_t g_port2DisconnectInfo;
extern portDisconnectInfo_t g_port3DisconnectInfo;
extern portDisconnectInfo_t g_port4DisconnectInfo;
extern portDisconnectInfo_t g_routerDisconnectInfo;
extern timeCalibration_t g_timeCalibration;


extern testInfo_t g_testInfo;

int ShwoParamInfo(paramBody_t *fp_paramBody);
int GetParamSectorInfo(paramBody_t *fp_paramBody, paramSectorInfo_t *fp_paramSectorInfo);
int SaveParamInfo(paramBody_t *fp_paramBody, paramSectorInfo_t *fp_paramSectorInfo);
int ShowAllParamInfo(paramBody_t *fp_paramBody, paramSectorInfo_t *fp_paramSectorInfo);
int AddInfo(paramBody_t *fp_paramBody,void *fp_data);
int ClearParamSector(paramBody_t *fp_paramBody, paramSectorInfo_t *fp_paramSectorInfo);


