/********************************************************************************
 *文件名            : app_bms.c
 *简  介            : BMS信息读取
 *日     期            : 2022/1/4
 *作  者            : 刘德明/高仙
 ********************************************************************************
 *备注：高仙BMS通讯协议，走CAN通讯
 *
 ********************************************************************************/
#include "app_bms.h"
#include "hc_gpio.h"
#include "hc_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bms.h"
#include "hc_can.h"
#include "anti_drop_collision.h"
#include "app_h25a15_ota.h"
#include "main.h"
#include "app_ota.h"
#include "hc32_ll_can.h"

#define BMS_VER_TRY_MAX_CNT   30

char bms_version[16]; /* BMS 版本号*/
BMS_Info_t BMS_Info = {0};
uint32_t bms_otaing_tick;
static uint8_t s_bms_ver_try_cnt = 0;

static void app_bms_update_data(unsigned char bms_type)
{
    if(bms_type == 0)
    {
        BMS_Info.vol = GetChargeVoltage();
        BMS_Info.charge_current = GetChargeCurrent();
        BMS_Info.soc = (BMS_Info.disconnect_flag ? 66 : bms_get_soc());
    }
    else
    {
        BMS_Info.soc = (BMS_Info.disconnect_flag ? 66 : BmsParam.bat_soc);
        BMS_Info.charge_current = BmsParam.bat_current > 0 ? BmsParam.bat_current : 0;
        BMS_Info.vol = BmsParam.bat_volt;        
    }
}

/**
 * @brief 设置断联状态
 */
static void app_bms_set_link_status(uint8_t val)
{
    BMS_Info.disconnect_flag = val;
}

/**
 * @brief 获取电池连接状态
 * @param  val               Param doc
 * @return staic
 */
static uint8_t app_bms_get_link_status(void)
{
    return BMS_Info.disconnect_flag;
}

void app_bms_reset(void)
{
    bms_reset();
}

uint8_t app_bms_get_reset_status(void)
{
    return bms_get_reset_status();
}

/**
 * @brief 电池查询任务的挂起与恢复
 * @param  task_sta          Param doc
 */
void app_bms_task_suspend_resume(app_ota_manage_task_t task_sta)
{
    if(task_sta == APP_OTA_TASK_SUSPEND)
    {
        vTaskSuspend(bms_tx_task_handle);
    }
    else if(task_sta == APP_OTA_TASK_RESUME)
    {
        vTaskResume(bms_tx_task_handle);
    }
    else
    {
        ;
    }
}

/**
 * @brief 从can口 接收到的数据 解析
 * @return int8_t 
 */
static int8_t app_raw_data_receiv_parse(void)
{
	stc_can_rx_frame_t rx_frame;

	if (xQueueReceive(bms_can2_rx_quene_handle, &rx_frame, 10000) == pdTRUE) // 10S
	{
        #if 0
        printf("rx_frame id = (0x%04x):[", rx_frame.u32ID);
        for(int i = 0; i < 8; i++)
        {
            printf(" 0x%02x",rx_frame.au8Data[i] );
        }
        printf("]\r\n");
        #endif

        /* 电池OTA升级相关 */
        if (rx_frame.u32ID == 0x20 || 
            rx_frame.u32ID == 0x21 ||
          ((rx_frame.u32ID == 0x704) && (rx_frame.au8Data[0] == 0)) ||
          ((rx_frame.u32ID > 0x580) && (rx_frame.u32ID <= 0x5FF)) )
        {
			/* 解析正常数据 */
            ObjdictDispatch2(&rx_frame);
			
			/* 解析ota数据 */
            if (((xTaskGetTickCount() - bms_otaing_tick) < 7000) && (xTaskGetTickCount() > 20000))
            {
                H25A15_Ota_Can_Read(rx_frame.u32ID, rx_frame.au8Data, rx_frame.DLC);
            }
			return 0;
        }
		if (rx_frame.u32ID > 0x600 && rx_frame.u32ID <= 0x6FF)
		{
			ObjdictDispatch3(&rx_frame);
            return 0;
		}
        
        /* 不知道为啥要放在这里,现在没想好要放到哪里去 */
        //if (rx_frame.IDE != 0)
        //{
		//	HandleCAN2RXData(rx_frame);
        //    return -1;
        //}
	}
	else
	{
		printf("bms recv over times ---> %d .\r\n",__LINE__);
        return -1;
	}
    return -1;
}

/**
 * @brief 接收数据 和 处理
 * @param  pvParameters      Param doc
 */
void app_bms_rx_process_Task(void *pvParameters)
{
    static uint32_t disconnect_count = 0;
    /* 加载驱动器对象字典 */
    bms_can2_load_obj();
    vTaskDelay(2500);
    printf("bms rx task running....\r\n");
    bms_constructor();
    while (1)
    {
        /* 上位机配置bms类型 */
        if (ConfigCommandCMD.bms_type & 0x80)
        {
            BMS_Info.battery_type = ConfigCommandCMD.bms_type & 0x0f;
        }

        /* rs485通信电池 */
        if (BMS_Info.battery_type == 0)
        {
            app_bms_update_data(0);
            if (bms_read_battery_info() == -1)
            {
                app_bms_set_link_status(1); /* 断联 */
                /* 如果上位机没有配置过,电池类型 */
                if((ConfigCommandCMD.bms_type & 0x80) == 0)
                {
                    BMS_Info.battery_type = 1;
                }
            }
            else
            {
                app_bms_set_link_status(0); /* 正常 */
            }
//            if(temp)
//            {
//                printf("bat disconnected!!! \r\n");
//            }
        }
        /* 拓邦/比亚迪CAN通讯电池 */
        else 
        {
            app_bms_update_data(1);
            if(app_raw_data_receiv_parse() == -1)
            {
                app_bms_set_link_status(1);
                if((ConfigCommandCMD.bms_type & 0x80) == 0)
                {
                    BMS_Info.battery_type = 0;
                }
            }
            else
            {
                app_bms_set_link_status(0);
            }
        }
    }
}

/**
 * @brief 发送指令
 * @param  pvParameters      Param doc
 */
void app_bms_tx_process_Task(void *pvParameters)
{
    bms_battery_cmd_t cmd = BMS_TB_CMD_VERVION;
    uint32_t bms_up_data_tick = xTaskGetTickCount();

    vTaskDelay(1000);
    printf("bms tx task running....\r\n");
#if 0    
    /* 获取 BMS版本 */
    if( bms_get_version(bms_version) == NULL)
    {
        memcpy(bms_version, "V0.6", 5);
    }
#endif
    while (1)
    {
        if (BMS_Info.battery_type == 0) // 使用旧485通讯电池bms
        {
            if ((get_rs485_bms_version() == 0) && (s_bms_ver_try_cnt < BMS_VER_TRY_MAX_CNT))
            {
                cmd = BMS_TB_CMD_VERVION;
                s_bms_ver_try_cnt++;
            }
            else
            {
                cmd = BMS_TB_CMD_INFO;
            }
            bms_tb_send_cmd(cmd);
            vTaskDelay(1000);
        }
        else
        {
            bms_can2_send_cmd();
            vTaskDelay(30);
        }
    }
}
