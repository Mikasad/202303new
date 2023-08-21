#include "h25a15/app_h25a15_ota.h"
#include "flash_op.h"
#include "service.pb.h"
#include "can_task.h"
#include "app_bms.h"
uint8_t g_can_ota_buf[8]  = {0};
uint8_t g_can_ota_buf_len = 0;
static upgradeProcessInfo_t gs_driver_upgradeProcessInfo;

canUpgradeAckInfo_t gs_driver_canUpgradeAckInfo = {0};

extern uint32_t otaing_tick;   //用于升级驱动器时屏蔽无关驱动器数据发送

static int Can1Send( uint32_t f_id, uint8_t f_value[], uint8_t f_len )
{
	can_standard_send(CAN1_UNIT, f_id, f_value, f_len);
	return 0;
}
void H25A15_Ota_Can_Write(uint32_t ID,uint8_t *Data,uint16_t Len)
{
	otaing_tick = xTaskGetTickCount();
	Can1Send(ID, Data,  Len) ;
	vTaskDelay(1);
}
extern CANBusStatusData CanBus_DATA;
void H25A15_Ota_Can_Read(uint32_t ID,uint8_t *pData,uint16_t Len)
{
	uint32_t data = 0;
	switch(ID)
	{
	#if 1
		case CAN_OTA_ORDER_ACK:
			printf("CAN_OTA_ORDER_ACK.\n");
			gs_driver_canUpgradeAckInfo.can_order_ack_falg = true;
			printf("%02X %02X\r\n",pData[0], pData[1]);
			if (0xE6 ==pData[0] && 0x01 == pData[1])
			{
				printf("CAN_OTA_ORDER_ACK.\n");
				gs_driver_canUpgradeAckInfo.can_roll_back_flag = true;
			}
			CanBus_DATA.can_id = ID;
			CanBus_DATA.ide    = 0;
			CanBus_DATA.rtr    = 0;
			memcpy(g_can_ota_buf, pData, Len);
			g_can_ota_buf_len = Len;
			break;
		case CAN_OTA_DATA_ACK:
//			printf("CAN_OTA_DATA_ACK.\n");
			gs_driver_canUpgradeAckInfo.can_data_ack_falg = true;
			CanBus_DATA.can_id = ID;
			CanBus_DATA.ide    = 0;
			CanBus_DATA.rtr    = 0;
			memcpy(g_can_ota_buf, pData, Len);
			g_can_ota_buf_len = Len;
			break;
		case CAN_OTA_ERROR_ACK:
			printf("CAN_OTA_ERROR_ACK.\n");
			gs_driver_canUpgradeAckInfo.can_error_ack_falg = true;
			break;
        case CAN_OTA_END_WHEEL_ACK:
		case CAN_OTA_END_ACK:
        case CAN_OTA_END_BMS_ACK:
			printf("CAN_OTA_END_ACK.\n");
			gs_driver_canUpgradeAckInfo.can_end_ack_flag = true;
			CanBus_DATA.can_id = ID;
			CanBus_DATA.ide    = 0;
			CanBus_DATA.rtr    = 0;
			memcpy(g_can_ota_buf, pData, Len);
			g_can_ota_buf_len = Len;
			break;
        case CAN_OTA_VER_WHEEL_ACK:
		case CAN_OTA_VER_ACK:
			printf("CAN_OTA_VER_ACK.\n");
			/* 按照小端模式，取前四个字节，比较是否是软硬件版本号 */
			data = ((*(uint32_t*)(pData)) & 0xFFFFFF00);
			if (CAN_OTA_SW_VER_IDX == data || CAN_OTA_HW_VER_IDX == data)
			{
				gs_driver_canUpgradeAckInfo.can_ver_ack_flag = true;
				CanBus_DATA.can_id = ID;
				CanBus_DATA.ide    = 0;
				CanBus_DATA.rtr    = 0;
				memcpy(g_can_ota_buf, pData, Len);
				g_can_ota_buf_len = Len;
			}
			break;
    case CAN_OTA_VER_BMS_ACK:
        {
            gs_driver_canUpgradeAckInfo.can_ver_ack_flag = true;
            CanBus_DATA.can_id = ID;
            CanBus_DATA.ide = 0;
            CanBus_DATA.rtr = 0;
            memcpy(g_can_ota_buf, pData, Len);
            g_can_ota_buf_len = Len;            
        }
        break;
		default:
			printf("default.\n");
			break;
	#endif
	
	}
}

void ota_canbus_command_process(CANBusCommand* cmd, uint8_t* buf, uint32_t len)
{
#if 0
	printf("CAN WRITE id: 0x%X\trtr: %d\tide: %d\r\n", cmd->can_id, cmd->rtr, cmd->ide);
	printf("data: ");
	for (int i = 0; i < len; i++)
		printf("%02X ", buf[i]);
	printf("\r\n");
#endif
    /* ??????? can_id */
    if(app_ota_type == 1)
    {
        stc_can_tx_frame_t stcTx1 = {0};
        uint8_t *p_data = buf;
        if (len > 8)
        {
            len = 0;
        }
        stcTx1.u32Ctrl = 0x0UL;
        stcTx1.u32ID = cmd->can_id;
        stcTx1.IDE = 0;        /*0??? 1???*/
        stcTx1.DLC = len;      /*?????*/

        for (uint8_t i = 0U; i < len; i++)
        {
            stcTx1.au8Data[i] = p_data[i];
        }
        CanTxMsg_t tx_msg = {0};
        tx_msg.head = stcTx1;
        
        bms_otaing_tick = xTaskGetTickCount();
//        can_standard_send(CAN2_UNIT, cmd->can_id, buf, len);
        ota_send(&tx_msg);
    }
    else
    { 
        otaing_tick = xTaskGetTickCount();
        can_standard_send(CAN1_UNIT, cmd->can_id, buf, len);
    }
}






