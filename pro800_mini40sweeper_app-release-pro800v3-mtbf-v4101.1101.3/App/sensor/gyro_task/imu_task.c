#include "imu_task.h"
#include "iwdg_task.h"
#include <string.h>
#include "nanopb_tcpip.h"

#include "hc_gpio.h"
#include "hc_uart2.h"
#include "main.h"
#include "gyro_hi216.h"
#include "gyro_zyf143.h"

extern struct DEVICECOMMAND_CMD DeviceCommandCMD;
extern struct DEVICE_DATA DeviceDATA;

static mesGyro_t m_gyro = {0};
static ImuType_t m_imu_type = IMU_UNKNOW;

static int16_t m_delta_yawangle = 0;

TickType_t m_imu_receive_tick = 0;

static ring_buffer_t imu_data_buffer; // imu数据缓冲区
ImuData_t imu_info = {0};

/**
 * @brief
 * @return ring_buffer_t*
 */
ring_buffer_t *imu_task_data_buffer(void)
{
    return &imu_data_buffer;
}

/**
 * @brief 设置断联状态
 */
void imu_logic_set_link_status(uint8_t val)
{
    imu_info.link_status = val;
}

/**
 * @brief 通过串口发送数据到imu模块
 * @param  fp_data   要发送的数据
 * @param  f_len     发送长度
 */
void imu_task_send_data_module(uint8_t *fp_data, uint8_t f_len)
{
    if ((fp_data != NULL) && (f_len > 1))
    {
        //printf("imu send data.\r\n");
        IMU_SEND_CMD(fp_data, f_len);
    }
    else
    {
        printf("imu send error.\r\n");
    }
}

void GetGyroMes(mesGyro_t *fp_mesGyro)
{
    if (fp_mesGyro != NULL)
    {
        taskENTER_CRITICAL();
        *fp_mesGyro = m_gyro;
        taskEXIT_CRITICAL();
    }
}
uint8_t GetGyroHealthState(void)
{
    return imu_info.link_status; // return GetLinkStatus(&m_gyro_connection_status);
}

int16_t GetImuYaw(void)
{
    return m_gyro.yawAngle;
}

int32_t GetDeltaYaw(void)
{
    return (int32_t)m_delta_yawangle;
}

/**
 * @brief 获取imu的版本号
 * @param  f_version         Param doc
 */
void app_imu_get_version(uint32_t *f_version)
{
    if (f_version != NULL)
    {
        if (m_imu_type == IMU_HI216)
        {
            gyro_hi216_get_version(f_version);
        }
        else if (m_imu_type == IMU_ZYF143)
        {
            gyro_zyf143_get_version(f_version);
        }
    }
}

/**
 * @brief 硬件复位imu模块
 */
static void imu_logic_reset(void)
{
    HAL_GPIO_WritePin(TLY_RST_GPIO_Port, TLY_RST_Pin, GPIO_PIN_RESET); // 打开IMU复位
    vTaskDelay(200);
    HAL_GPIO_WritePin(TLY_RST_GPIO_Port, TLY_RST_Pin, GPIO_PIN_SET); // 取消IMU复位
    vTaskDelay(3000);                                                /* 为保证imu模块正常出数据 */
}

/**
 * @brief 1.通过给模块发指令获取版本号; 2.通过返回的版本号,确定模块类型
 */
static int app_imu_module_type_and_version(void)
{
    imu_pack_t imu_pack;
    uint8_t send_times = 0;

    imu_logic_reset();
    if (imu_pack_quene_handle == NULL)
    {
        m_imu_type = IMU_ZYF143;
        return -1;
    }
    do
    {
        /* 30秒没有获取到版本号,默认是 IMU_ZYF143 */
        if (send_times++ > 50)
        {
            m_imu_type = IMU_ZYF143;
            vTaskDelay(2000);
            xQueueReset(imu_pack_quene_handle);
            printf("imu get version over times.\r\n");
            return -1;
        }
        if (xQueueReceive(imu_pack_quene_handle, &imu_pack, 1000) == pdTRUE)
        {
            if (gyro_hi216_read_version(imu_pack.rx_data, imu_pack.len) == 0)
            {
                printf("HI216 imu len = (%d) ; data = [%s] .\r\n", imu_pack.len, imu_pack.rx_data);
                m_imu_type = IMU_HI216;
                vTaskDelay(2000);
                xQueueReset(imu_pack_quene_handle);
                return 0;
            }
            if (gyro_zyf143_read_version(imu_pack.rx_data, imu_pack.len) == 0)
            {
                printf("ZYF143 imu len = (%d) ; data = [%s] .\r\n", imu_pack.len, imu_pack.rx_data);
                m_imu_type = IMU_ZYF143;
                vTaskDelay(2000);
                xQueueReset(imu_pack_quene_handle);
                return 0;
            }
        }
        else
        {
            printf("get hi216 imu data over times.\r\n");
        }
        vTaskDelay(200);
    } while (1);

    return -1;
}

/**
 * @brief imu数据采集任务
 * @param  pvParameters      Param doc
 */
void app_imu_task(void const *pvParameters)
{
    static uint32_t disconnect_count = 0;
    int ret = 0;
    imu_pack_t imu_pack;

    printf("imu running.\r\n");

    vTaskDelay(1000);

    app_imu_module_type_and_version();

    ring_buffer_create(&imu_data_buffer, IMU_TEMP_BUFFER_SIZE);

    while (1)
    {
        /* 任务循环运行次数 */
        DeviceDATA.imu_task_run_cnt++;

        /* 接收来自imu模组的数据 */
        if (xQueueReceive(imu_pack_quene_handle, &imu_pack, 120) == pdTRUE) // 100ms 20hz
        {
            imu_info.updataTick = xTaskGetTickCount();

            imu_logic_set_link_status(1);
#if 0
            //if (imu_pack.len < 18)
            //{
            //    printf("imu data (%d) \r\n", imu_pack.len);
            //}
            printf("imu data (%d) {",imu_pack.len );
            for(int i = 0;i < imu_pack.len;i++ )
            {
                printf(" 0x%02x", imu_pack.rx_data[i]);
            }
            printf("}\r\n");
#endif
            switch (m_imu_type) /* 接收数据的解析 */
            {
            case IMU_HI216:
                gyro_hi216_parse_data(&m_gyro, imu_pack.rx_data, imu_pack.len);
                break;
            case IMU_ZYF143:
                gyro_zyf143_parse_data(&m_gyro, imu_pack.rx_data, imu_pack.len);
                break;
            default:
                break;
            }
        }
        else
        {
            imu_logic_set_link_status(0);
        }
        vTaskDelay(5);
    }
}

/**********************************************************************************************
 *函数名	: IMU_Logic_Task
 *介	 绍	：IMU数据接收处理任务务函数
 *形  参 : 无
 *返回值 : 无
 ***********************************************************************************************/
void app_imu_logic_task(void *Param)
{
    uint32_t times = 0;
    uint8_t imu_get_version_times = 0;
    uint16_t imu_send_times = 0;
    uint32_t imu_reset_tick = xTaskGetTickCount();
    static uint8_t reset_cnt = 0;

    while (1)
    {
        /* 处理上位机复位imu指令 */
        if (DeviceCommandCMD.reset_imu == 1)
        {
            reset_cnt = 0;
            DeviceDATA.imu_reset_status = 1;
            imu_logic_reset();

            DeviceDATA.imu_reset_status = 0;
            DeviceCommandCMD.reset_imu = 0;
            printf("Imu Logic reset imu.\r\n");
        }

        if (xTaskGetTickCount() - imu_reset_tick > 30000) // 30S
        {
            imu_reset_tick = xTaskGetTickCount();
            if ((reset_cnt < 2) && (!imu_info.link_status))
            {
                printf("imu power reset.\r\n");
                IMU_POWEROFF();
                vTaskDelay(200);
                IMU_POWERON();
                vTaskDelay(3000);
                reset_cnt++;
                imu_info.link_status = 1;
            }
        }
        if (m_imu_type == IMU_UNKNOW)
        {
            switch (imu_get_version_times)
            {
            case 1:
                /* HI216 */
                imu_task_send_data_module((uint8_t *)"AT+INFO\r\n", 9);
                imu_get_version_times = 0;
                break;
            case 0:
                /* ZYF143 */
                imu_task_send_data_module((uint8_t *)"$VERS*", 6);
                imu_get_version_times = 1;
                break;
            default:
                break;
            }
            /* 30秒 没有获取到版本号,默认是 IMU_ZYF143 */
            if (imu_send_times++ > 1500)
            {
                imu_send_times = 0;
                printf("imu not version = (%d) ; \r\n", imu_send_times);
                m_imu_type = IMU_ZYF143;
            }
        }
        vTaskDelay(20);
    }
}
