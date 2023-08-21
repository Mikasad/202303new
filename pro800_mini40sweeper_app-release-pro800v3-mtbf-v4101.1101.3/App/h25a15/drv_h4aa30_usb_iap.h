#ifndef __USB_H4AA_IAP_H__
#define __USB_H4AA_IAP_H__

#include "hc32_ll.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ff.h"
#include "usb_host_def.h"
#include "usb_host_user.h"
#include "usb_host_msc_class.h"
#include "flash_op.h"
#include "hc_can.h"
#include "string.h"
#include "main.h"
#include "hc_gpio.h"

#define   MAX_OTA_WAIT_MS                      (9000U) /*OTA等待驱动器回复的最长时间,驱动器进BootLoader最少要4s*/

#define BIN_FILE_BUFFER_SIZE        ((uint16_t)1024)
#define DOWNLOAD_FILENAME	        "H4AA30.bin"					
 
#if (1)
#define  usb_iap_log(...)		do { \
								printf(__VA_ARGS__); \
								printf("\r\n"); \
} while (0)
#else
#define usb_iap_log(...) do {} while (0)
#endif

typedef enum {
	IAP_OK = 0,				/* (0) Succeeded */
	IAP_MOUNT_ERR,			/* (1) f_mount failed */
	IAP_OPEN_FILE_ERR,		/* (2) f_open failed */
	IAP_FILE_SIZE_ERR,		/* (3) The file size exceeds the maximum  */
	IAP_ERASE_FLASH_ERR,	    /* (4) erase flash app sector failed */
	IAP_WRITE_FLASH_ERR,	    /* (5) write flash app sector failed */
	IAP_READ_FILE_ERR,		/* (6) read bin file failed while write bin file to flash */
	IAP_CRC_ERR,			    /* (7) the CRC value of the data written to flash is different from the value of U disk data CRC  */
} eIapStatus;

typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_START,
  APPLICATION_READY,
  APPLICATION_DISCONNECT
}ApplicationTypeDef;

typedef struct
{
	uint16_t driver_init_flg;                    /*USB驱动已经完成初始化*/
	uint32_t file_size;                          /*驱动器固件字节数*/
	uint8_t  pack_num;                           /*驱动器固件总包数*/
	uint8_t  read_pack_cnt;                      /*读取驱动器第x包*/
	uint32_t read_pack_size;                     /*从U盘读取的一包数据字节数，正常1024，最后一包需要计算*/
	uint8_t  ota_flag;                           /*0 未升级 1升级中 2升级完成 3升级失败*/
	uint8_t  pack_buff[BIN_FILE_BUFFER_SIZE];    /*存放一整包固件的缓存*/
	uint16_t pack_crc;                           /*一包数据的crc校验结果*/
	uint8_t  request_primary_num;                /*驱动器请求数据包序号  大包 */
	uint8_t  request_secondary_num;              /*驱动器请求数据包序号  小包*/
	uint32_t wait_response_tick;                 /*记录等待响应的时间*/

}H4AA_UPGRADE_t;

extern uint32_t otaing_tick;

/*驱动器OTA数据接收处理	 */
void H4aa_OTA_RxCpltCallback(uint32_t id,uint8_t* p_data);
void UsbBeep(void);
/*创建USB任务*/
void USB_IAP_Task_Init(void);


#endif




