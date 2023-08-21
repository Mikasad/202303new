#include "app_ota.h"
#include "drv_ota.h"
#include "nanopb_tcpip.h"
#include "protocol_config.h"
#include "flash_op.h"
#include "app_bms.h"
#include "anti_drop_collision.h"
static uint8_t* s_ota_buf = NULL;

extern void ota_canbus_command_process(CANBusCommand* cmd, uint8_t* buf, uint32_t len);
uint8_t g_can_cmd_flag = 0;
uint8_t g_can_status_flag = 0;
uint8_t g_clear_can_bus = 0;

static uint16_t s_ota_msg_len = 0;
static uint16_t s_ota_buf_len = 0;
static bool s_ota_process_flag = false;
static uint32_t s_ota_current_msg_id = 0;
SemaphoreHandle_t s_ota_mutex = NULL;
uint8_t s_ota_data_len = 0;
static pb_byte_t s_ota_data_buf[OTA_PACKAGE_SIZE] = {0};
static uint32_t s_recv_can_cmd_tick = 0;
static uint32_t s_recv_sub_mcu_cmd_tick = 0;
static uint32_t s_recv_imu_ota_cmd_tick = 0;

extern PhantasMcuUpgradeCommand PhantasMcuUpgrade_CMD;
extern PhantasMcuUpgradePackage PhantasMcuUpgradePackage_CMD;
extern CANBusCommand CanBus_CMD;
extern TaskHandle_t ZLAC_Handle;
extern TaskHandle_t H25A_Handle;
extern canUpgradeAckInfo_t gs_driver_canUpgradeAckInfo;

uint8_t app_ota_type = 0;
static uint8_t task_status = 0;

void set_recv_can_cmd_tick(uint32_t tick)
{
	s_recv_can_cmd_tick = tick;
}

uint32_t get_recv_can_cmd_tick(void)
{
	return s_recv_can_cmd_tick;
}

void set_recv_sub_mcu_cmd_tick(uint32_t tick)
{
	s_recv_sub_mcu_cmd_tick = tick;
}

uint32_t get_recv_sub_mcu_cmd_tick(void)
{
	return s_recv_sub_mcu_cmd_tick;
}

void set_recv_imu_ota_cmd_tick(uint32_t tick)
{
	s_recv_imu_ota_cmd_tick = tick;
}

uint32_t get_recv_imu_ota_cmd_tick(void)
{
	return s_recv_imu_ota_cmd_tick;
}
void set_ota_process_flag(bool flag)
{
	s_ota_process_flag = flag;
}

bool get_ota_process_flag(void)
{
	return s_ota_process_flag;
}

void set_ota_current_msg_id(uint32_t msg_id)
{
	s_ota_current_msg_id = msg_id;
}

uint32_t get_ota_current_msg_id(void)
{
	return s_ota_current_msg_id;
}

void post_to_ota_task(uint32_t msg_id, uint8_t* buf, uint16_t* msg_len, uint16_t buf_len)
{
	#ifdef OTA_BUF
	memmove(s_ota_buf, buf, *msg_len);
	#else
	s_ota_buf = buf;
	#endif
	s_ota_msg_len = *msg_len;
	s_ota_buf_len = buf_len;
	
	set_ota_process_flag(true);
	set_ota_current_msg_id(msg_id);
	xSemaphoreGive(s_ota_mutex);
}

void post_canbus_to_ota_task(uint32_t msg_id)
{
	set_ota_process_flag(true);
	set_ota_current_msg_id(msg_id);
	xSemaphoreGive(s_ota_mutex);
//	printf("post_canbus_to_ota_task msg_id:%d\r\n",msg_id);
}

void copy_ota_package(pb_byte_t *buf, uint32_t len)
{
	if (len > OTA_PACKAGE_SIZE)
		len = OTA_PACKAGE_SIZE;
	memmove(s_ota_data_buf, buf, len);
	s_ota_data_len = len;
}
#include "tcp_server.h"
void send2pc(uint8_t* pc_tx_msg, uint16_t msg_len)
{	
	if(server_tpcb)  
	{
		printf("len:%d\r\n",msg_len);
		portENTER_CRITICAL();
		tcp_write(server_tpcb, pc_tx_msg, msg_len, 1); 
		tcp_output( server_tpcb );	
        tcp_recved(server_tpcb, server_p->tot_len);
        pbuf_free(server_p);	
		portEXIT_CRITICAL();		
	}

}

void can_cmd_send_ack(void)
{
    if(app_ota_type == 0)
    {
        nanopb_fill_ack_msg(CANBUS_COMMAND, s_ota_buf, &s_ota_msg_len, s_ota_buf_len);
    }
    else
    {
        nanopb_fill_ack_msg(CANBUS2_COMMAND, s_ota_buf, &s_ota_msg_len, s_ota_buf_len);
    }
	send2pc(s_ota_buf, s_ota_msg_len);

}

void can_status_send_ack(void)
{
    if(app_ota_type == 0)
    {
        nanopb_fill_canbus_status_ack_msg(CANBUS_STATUS, s_ota_buf, &s_ota_msg_len, s_ota_buf_len);
    }
    else
    {
        nanopb_fill_canbus_status_ack_msg(CANBUS2_STATUS, s_ota_buf, &s_ota_msg_len, s_ota_buf_len);
    }
	send2pc(s_ota_buf, s_ota_msg_len);
}

void sub_mcu_cmd_send_ack(void)
{
	nanopb_fill_ack_msg(PHANTAS_MCU_UPGRADE_COMMAND, s_ota_buf, &s_ota_msg_len, s_ota_buf_len);
	send2pc(s_ota_buf, s_ota_msg_len);
}

void sub_mcu_pkg_send_ack(void)
{
	nanopb_fill_ack_msg(PHANTAS_MCU_UPGRADE_PACKAGE, s_ota_buf, &s_ota_msg_len, s_ota_buf_len);
	send2pc(s_ota_buf, s_ota_msg_len);
}

void imu_cmd_send_ack(void)
{
	nanopb_fill_ack_msg(PHANTAS_MCU_UPGRADE_COMMAND, s_ota_buf, &s_ota_msg_len, s_ota_buf_len);
	send2pc(s_ota_buf, s_ota_msg_len);
}

void imu_pkg_send_ack(void)
{
	nanopb_fill_ack_msg(PHANTAS_MCU_UPGRADE_PACKAGE, s_ota_buf, &s_ota_msg_len, s_ota_buf_len);
	send2pc(s_ota_buf, s_ota_msg_len);
}
extern void set_packet_buf(uint32_t packet_num, uint8_t*buf);
extern void jwt_ota_init(uint32_t bin_size);
extern void set_pc_cmd_type(uint8_t cmd_type);
void app_ota_task(void *pvParameters)
{
    vSemaphoreCreateBinary(s_ota_mutex);
    while (1)
    {
        xSemaphoreTake(s_ota_mutex, portMAX_DELAY);
        OTA_PRT("ota current msg id:%d\r\n", get_ota_current_msg_id());
        g_can_cmd_flag = 0;
        memset(&gs_driver_canUpgradeAckInfo, 0, sizeof(gs_driver_canUpgradeAckInfo));

        switch (s_ota_current_msg_id)
        {
        case RX_CANBUS_COMMAND: // 驱控器升级指令
        case RX_CANBUS2_COMMAND:
            if(s_ota_current_msg_id == RX_CANBUS2_COMMAND)
            {
                if(task_status == 0)
                {
                   
                    app_bms_task_suspend_resume(APP_OTA_TASK_SUSPEND);
                    app_anti_collision_task_suspend_resume(APP_OTA_TASK_SUSPEND);
                    task_status = 1;
                }
            }
            ota_canbus_command_process(&CanBus_CMD, s_ota_data_buf, s_ota_data_len);
            set_recv_can_cmd_tick(HAL_GetTick());

            g_can_cmd_flag = 1;
            break;
        case CANBUS_STATUS:
        case CANBUS2_STATUS:
            g_can_cmd_flag = 0;
            can_status_send_ack();
            break;
        default:
            break;
        }
    }
}

void app_ota_msg_check(void *pvParameters)
{
    uint8_t Time_Out_Package[2] = {0x08, 0xFF}; // 超时,给上位机回复的包

    while (1)
    {
        if (g_can_cmd_flag && (HAL_GetTick() - get_recv_can_cmd_tick() > 7000))
        {
            printf("Recv Can Ota Ack Timeout.\r\n");
            send2pc(Time_Out_Package, 2);
            g_can_cmd_flag = 0;
            if(task_status == 1)
            {
       
                app_bms_task_suspend_resume(APP_OTA_TASK_RESUME);
                app_anti_collision_task_suspend_resume(APP_OTA_TASK_RESUME);
                vTaskDelay(2);
                task_status = 0;
            }
        }

			if (g_can_cmd_flag && gs_driver_canUpgradeAckInfo.can_order_ack_falg) {
				can_cmd_send_ack();
				g_can_cmd_flag = 0;
				gs_driver_canUpgradeAckInfo.can_order_ack_falg = 0;
			}

			if (g_can_cmd_flag && gs_driver_canUpgradeAckInfo.can_data_ack_falg) {
				can_cmd_send_ack();
				g_can_cmd_flag = 0;
				gs_driver_canUpgradeAckInfo.can_data_ack_falg = 0;
			}

        if (g_can_cmd_flag && gs_driver_canUpgradeAckInfo.can_end_ack_flag)
        {
            can_cmd_send_ack();
            g_can_cmd_flag = 0;
            gs_driver_canUpgradeAckInfo.can_end_ack_flag = 0;
            if(task_status == 1)
            {
                app_bms_task_suspend_resume(APP_OTA_TASK_RESUME);
                app_anti_collision_task_suspend_resume(APP_OTA_TASK_RESUME);
                task_status = 0;
            }
        }
			if (g_can_cmd_flag && gs_driver_canUpgradeAckInfo.can_ver_ack_flag) {
				can_cmd_send_ack();
				g_can_cmd_flag = 0;
				gs_driver_canUpgradeAckInfo.can_ver_ack_flag = 0;
			}

			if (g_can_cmd_flag && gs_driver_canUpgradeAckInfo.can_error_ack_falg) {
				can_cmd_send_ack();
				g_can_cmd_flag = 0;
				gs_driver_canUpgradeAckInfo.can_error_ack_falg = 0;
                if(task_status == 1)
                {
                    app_bms_task_suspend_resume(APP_OTA_TASK_RESUME);
                    app_anti_collision_task_suspend_resume(APP_OTA_TASK_RESUME);
                    task_status = 0;
                }
            }
			vTaskDelay(5);            
        }
}





