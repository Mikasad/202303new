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

#define   MAX_OTA_WAIT_MS                      (9000U) /*OTA�ȴ��������ظ����ʱ��,��������BootLoader����Ҫ4s*/

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
	uint16_t driver_init_flg;                    /*USB�����Ѿ���ɳ�ʼ��*/
	uint32_t file_size;                          /*�������̼��ֽ���*/
	uint8_t  pack_num;                           /*�������̼��ܰ���*/
	uint8_t  read_pack_cnt;                      /*��ȡ��������x��*/
	uint32_t read_pack_size;                     /*��U�̶�ȡ��һ�������ֽ���������1024�����һ����Ҫ����*/
	uint8_t  ota_flag;                           /*0 δ���� 1������ 2������� 3����ʧ��*/
	uint8_t  pack_buff[BIN_FILE_BUFFER_SIZE];    /*���һ�����̼��Ļ���*/
	uint16_t pack_crc;                           /*һ�����ݵ�crcУ����*/
	uint8_t  request_primary_num;                /*�������������ݰ����  ��� */
	uint8_t  request_secondary_num;              /*�������������ݰ����  С��*/
	uint32_t wait_response_tick;                 /*��¼�ȴ���Ӧ��ʱ��*/

}H4AA_UPGRADE_t;

extern uint32_t otaing_tick;

/*������OTA���ݽ��մ���	 */
void H4aa_OTA_RxCpltCallback(uint32_t id,uint8_t* p_data);
void UsbBeep(void);
/*����USB����*/
void USB_IAP_Task_Init(void);


#endif



