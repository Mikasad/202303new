#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "imu_task.h"
#include "gyro_zyf143.h"
#include "ringbuffer.h"
#include "nanopb_tcpip.h"

/*
*********************************************************************************************************
*
*    模块名称 : 陀螺仪zyf143模块
*    文件名称 : zyf143.c
*    版    本 :     V1.0.0
*    说    明 :
*            1，本文件主要处理zyf143陀螺仪数据
*    修改记录 :
*        版本号          日期        作者     说明
*        V1.0.0    2020-9-03  ypc       正式发布
*    外设资源占用：
*        串口：        串口2的R/T
*        IO口：    PD5,PD6
*        DMA：    DMA1_Stream5,DMA_Channel_4
*        NVIC：    USART2_IRQn，优先级（3，0）
*********************************************************************************************************
*/

static uint16_t m_zyf143_version = 0; /* 版本号 */

/**
 * @brief 组包 和 对比crc
 * @param  data              Param doc
 * @return int 
 */
static int gyro_zyf143_package_calc_crc(mesGyro_t *fp_gyro, uint8_t *data)
{
    uint8_t temp[2] = {0};
    int16_t buff[8];

    for (int i = 0; i < 8; i++)
    {
        temp[0] = data[2 + i * 2];     // 低位
        temp[1] = data[2 + i * 2 + 1]; // 高位
        buff[i] = ((int16_t)temp[1] << 8) | ((int16_t)temp[0]);
    }
    int16_t cal_crc = (buff[3] + buff[4] + buff[5] + buff[6]);
    if (buff[7] == cal_crc)
    {
        fp_gyro->linearAcceleration_x = (int16_t)buff[0]; // 加速度计x轴原始数据
        fp_gyro->linearAcceleration_y = (int16_t)buff[1]; // 加速度计y轴原始数据
        fp_gyro->linearAcceleration_z = (int16_t)buff[2]; // 加速度计z轴原始数据
        fp_gyro->angularVelocity_z = (int16_t)buff[3];
        fp_gyro->yawAngle = buff[4] / 10 + 1800;   // 航向角
        fp_gyro->pitchAngle = buff[5] / 10 + 1800; // 俯仰角
        fp_gyro->rollAngle = buff[6] / 10 + 1800;  // 横滚角
			  return 0;
    }
    else
    {
        printf("imu crc error [rcv = %d ; cal = %d ].\r\n", buff[7], cal_crc);
        return -1;
    }
}

/**
 * @brief 对数据进行缓存和解包
 * @param  fp_gyro           Param doc
 * @param  fp_indata         Param doc
 * @param  f_len             Param doc
 * @return int 
 */
static int gyro_zyf143_data_handle(mesGyro_t *fp_gyro, uint8_t *fp_indata, uint16_t f_len)
{
    int res = -1;
    uint8_t buffer[IMU_FRAME_LEN];
    uint8_t temp;
    ring_buffer_t* p_imu_buffer = imu_task_data_buffer();
    if(p_imu_buffer == NULL)
        return res;

    ring_buffer_queue_arr(p_imu_buffer, fp_indata, f_len);
    uint16_t buf_items_num = ring_buffer_num_items(p_imu_buffer);

    if(buf_items_num < IMU_FRAME_LEN)
    {
        //printf("gyro zyf143 data len = (%d) warning. \r\n", buf_items_num);
        return res;
    } /* 缓存的没用数据过多，清除掉*/
    else if(buf_items_num > (IMU_TEMP_BUFFER_SIZE - IMU_FRAME_LEN) )
    {
        printf("gyro zyf143 delete = (%d) warning. \r\n", buf_items_num);
        /* 删除队列数据 */
        ring_buffer_queue_arr(p_imu_buffer, buffer, buf_items_num);
        return res;
    }
    else
    {
        for(uint8_t i = 0; i < buf_items_num; i++)
        {
            if(buf_items_num < IMU_FRAME_LEN)
            {
                printf("imu ring buffer not frame.\r\n");
                return -1;
            }
            /* 找帧头1 */
            if( ring_buffer_dequeue(p_imu_buffer, &temp) == 1)
            {
                if(temp == 0xa5)
                {
                    buffer[0] = temp;
                    ring_buffer_dequeue(p_imu_buffer, &temp);
                    /* 找到帧头2 */
                    if(temp == 0xa5)
                    {
                        buffer[1] = temp;
                        uint8_t dat_len = ring_buffer_dequeue_arr(p_imu_buffer, &buffer[2], (IMU_FRAME_LEN - 2) );
                        if( dat_len == (IMU_FRAME_LEN - 2))
                        {
                            res = gyro_zyf143_package_calc_crc(fp_gyro, buffer);
                            return res;
                        }
                        else
                        {
                            printf("ring buffer dequeue arr len = (%d) warning. \r\n", dat_len);
                            return res;
                        }
                    }
                }
            }
            else
            {
                printf("imu ring buffer error.\r\n");
                return -1;
            }
            buf_items_num = ring_buffer_num_items(p_imu_buffer);
        }           
    }
    return res;
}

/**
 * @brief 外部获取版本号
 * @param  fp_version        Param doc
 */
void gyro_zyf143_get_version(uint32_t *fp_version)
{
    if (fp_version != NULL)
    {
        fp_version[0] = 0;
        fp_version[1] = m_zyf143_version / 100;
        fp_version[2] = (m_zyf143_version % 100) / 10;
        fp_version[3] = m_zyf143_version % 10;
    }
}

/**
 * @brief 解析陀螺仪数据
 * @param  f_type            Param doc
 * @param  fp_gyro           Param doc
 * @param  fp_data           Param doc
 * @param  f_len             Param doc
 * @return int
 */
int gyro_zyf143_parse_data(mesGyro_t *fp_gyro, uint8_t *fp_data, uint16_t f_len)
{
    int ret = -1;
                
    ret = gyro_zyf143_data_handle(fp_gyro, fp_data, f_len);

    return ret;
}

/**
 * @brief 读取版本号
 * @param  f_type            Param doc
 * @param  fp_gyro           Param doc
 * @param  fp_data           Param doc
 * @param  f_len             Param doc
 * @return int
 */
int gyro_zyf143_read_version(uint8_t *fp_data, uint16_t f_len)
{
    char *p_version = NULL;
    p_version = strstr((const char *)fp_data, "$ZYVE_GX");
    if (p_version != NULL)
    {
        p_version += 8;
        m_zyf143_version = atoi(p_version);
        return 0;
    }
    p_version = strstr((const char *)fp_data, "$ZY_GX_V");
    if (p_version != NULL)
    {
        uint8_t temp = 0;
        p_version += 8;
        temp = atoi(p_version);
        m_zyf143_version = atoi(p_version + 2) + temp * 10;
        return 0;
    }
    return 1;
}
