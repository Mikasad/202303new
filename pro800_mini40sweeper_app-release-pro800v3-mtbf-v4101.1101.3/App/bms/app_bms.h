#ifndef __APP_GS_BMS__H__
#define __APP_GS_BMS__H__

#include "cmsis_os.h"
#include "string.h"
#include "drv_gs_can_bms.h"
#include "bms.h"
#include "nanopb_tcpip.h"
#include "app_ota.h"
#include "main.h"

typedef struct
{
	uint8_t battery_type;     //电池类型0: 485通讯电池  1:高仙CAN通讯协议电池        
	uint8_t disconnect_flag;  //接收到的命令 0通讯正常 1通讯断链
	uint8_t soc;
	int16_t vol;
	int16_t charge_current;
}BMS_Info_t;   

extern struct CONFIGCOMMAND_CMD  ConfigCommandCMD;
extern BMS_PARAM_T	BmsParam;
extern BMS_Info_t BMS_Info;
extern uint32_t bms_otaing_tick;

void app_bms_task_suspend_resume(app_ota_manage_task_t task_sta);
void app_bms_rx_process_Task(void *pvParameters );
void app_bms_tx_process_Task(void *pvParameters );
void app_bms_reset(void);
uint8_t app_bms_get_reset_status(void);

#endif
