#include "anti_drop_collision.h"
#include "hc_can.h"
#include "anti_collision.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

Anti_Drop_Typedef gt_AntiDrop = {0};
uint8_t g_antiDropStop = 0;
uint8_t g_antiDropDisconnectFlag = 0;

static uint8_t m_anti_drop_type = _eAntiDropType_TF;
static uint8_t m_gcub_disconnect_count = 0;
/**
 * @brief 电池查询任务的挂起与恢复
 * @param  task_sta          Param doc
 */
void app_anti_collision_task_suspend_resume(app_ota_manage_task_t task_sta)
{
    if(task_sta == APP_OTA_TASK_SUSPEND)
    {
        vTaskSuspend(anti_drop_task_handle);
    }
    else if(task_sta == APP_OTA_TASK_RESUME)
    {
        vTaskResume(anti_drop_task_handle);
    }
    else
    {
        ;
    }
}
void ConfigAntiDrop( uint32_t f_enable, uint32_t fp_threshold[] )
{
	gt_AntiDrop.sum_Enable = f_enable;
	if( fp_threshold != NULL )
	{
		gt_AntiDrop.un_channel1_Threshold = fp_threshold[0]?fp_threshold[0]:gt_AntiDrop.un_channel1_Threshold;
		gt_AntiDrop.un_channel2_Threshold = fp_threshold[1]?fp_threshold[1]:gt_AntiDrop.un_channel2_Threshold;
		gt_AntiDrop.un_channel3_Threshold = fp_threshold[2]?fp_threshold[2]:gt_AntiDrop.un_channel3_Threshold;
		gt_AntiDrop.un_channel4_Threshold = fp_threshold[3]?fp_threshold[3]:gt_AntiDrop.un_channel4_Threshold;
	}
}

uint32_t GetAntiDropDisconnectStatus(void)
{
	return g_antiDropDisconnectFlag;
}

uint8_t GetAntiDropStop(void)
{
	return g_antiDropStop;
}

uint8_t GetGcubDisconnectStatus(void)
{
    //只在使能的情况下检测断链
    if( gt_AntiDrop.sum_Enable || GetAntiCollisionEnable() )
        return (m_gcub_disconnect_count>100)?1:0;
    else
        return 0;
}

void SetAntiDropSwitch(uint8_t f_switch)
{
    gt_AntiDrop.output_switch = f_switch;
}

//can2接收处理函数
void HandleCAN2RXData(stc_can_rx_frame_t f_rx_msg)
{
	if(f_rx_msg.IDE == 0)
	{
		return ;  //标准帧退出解析
	}
    switch(f_rx_msg.u32ID) 
    {
        /********************anti_collision************************/
        case 0x0B011038:
        case 0x0B011039:
        case 0x0B01103A:
        case 0x0B01103B:
        case 0x0B01103C:
        case 0x0B01103D:
        case 0x0B01103E:
        case 0x0B01103F:
        case 0x0B011033:
        case 0x0B011048:
        case 0x0B011049:
        case 0x0B01104A:
        case 0x0B01104B:
        case 0x0B01104C:
        case 0x0B01104D:
        case 0x0B01104E:
        case 0x0B01104F:
        case 0x0B011031:
        case 0x0B011051:
            m_gcub_disconnect_count = 0;
            receiveAntiCollisionData(f_rx_msg.u32ID, f_rx_msg.au8Data);
            break;

        /**********************************************************/
        default:
            break;
    }
}

//防跌落防碰撞检测
void AntiDropCollisionTask(void const *pvParameters)
{
    uint32_t last_wake_tick;
	const uint32_t c_frequency = 10;//xms进行一次任务处理
    uint8_t count = 0;
    last_wake_tick = xTaskGetTickCount();

    while(1)
    {
        count++;
        if( count >= 2 )
        {
            count = 0;
            AntiCollisionTask();
        }
        if( m_gcub_disconnect_count <=100 )
            m_gcub_disconnect_count++;
            
        vTaskDelay( c_frequency);
    }
}

/**
 * @brief 接收数据 和 处理
 * @param  pvParameters      Param doc
 */
void app_anti_rx_process_Task(void *pvParameters)
{
	stc_can_rx_frame_t rx_frame;
    vTaskDelay(1000);
    printf("anti rx task running....\r\n");
    while (1)
    {
        if (xQueueReceive(anti_can2_rx_quene_handle, &rx_frame, portMAX_DELAY) == pdTRUE) // 10S
        {
            #if 0
            printf("rx_frame id = (0x%04x):[", rx_frame.u32ID);
            for(int i = 0; i < 8; i++)
            {
                printf(" 0x%02x",rx_frame.au8Data[i] );
            }
            printf("]\r\n");
            #endif
            HandleCAN2RXData(rx_frame);
        }
    }
}
