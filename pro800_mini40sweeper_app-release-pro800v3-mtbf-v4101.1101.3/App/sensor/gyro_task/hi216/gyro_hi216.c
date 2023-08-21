#include <string.h>
#include "gyro_hi216.h"
#include "imu_task.h"

/*
*********************************************************************************************************
*
*    模块名称 : 陀螺仪_hi216模块
*    文件名称 : gyro_hi216.c
*    版    本 :     V1.0.0
*    说    明 :
*            1，因陀螺仪输出数据的频率较高，数据量较大，所以使用DMA.频率太高了85HZ左右，一次41字节。
*            2，工程包括4个部分：硬件初始化、软件初始化、freertos任务、中断处理
*            3，有关陀螺仪的具体协议可以参阅该陀螺仪的官方文档
*    修改记录 :
*        版本号          日期        作者     说明
*        V1.0.0    2018-11-05  杨臣       正式发布
*        V1.0.1    2018-11-30  杨臣       陀螺仪的pitchAngle、rollAngle、yawAngle需要先处理一下再上传
*       V1.1.0    2020-09-05  ypc       将本文件修改为hi216数据解析功能
*    外设资源占用：
*        串口：        串口2的R/T
*        IO口：    PD5,PD6
*        DMA：    DMA1_Stream5,DMA_Channel_4
*        NVIC：    USART2_IRQn，优先级（3，0）
*********************************************************************************************************
*/

static msgPkgHI216_t gs_msgPkgHI216;
static int32_t m_imu_version[3] = {0, 0, 0};

static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);
static int UpdateHI216MUMMes(mesGyro_t *fp_mesGyro, msgPkgHI216_t *fp_mesPkgHI216);
static int RecGYROHI216Data(uint8_t *fp_data, uint16_t f_len);

/**
 * @brief  初步解析收到的数据
 * @param  fp_data    收到的原始数据
 * @param  f_len      收到的数据长度
 * @return int （1 : 没有获取到数据，0 : 获取到了数据）
 */
static int RecGYROHI216Data(uint8_t *fp_data, uint16_t f_len)
{
    int ret = 1;
    uint32_t lenOnePkg = 0;

    // 3，获取数据长度
    // 4，判断是不是接收到了数据
    if (f_len != 0) // 有时没有收到数据也会进入空闲中断
    {
        // 初步判断数据是否正常
        if (fp_data[0] == 0x5A && fp_data[1] == 0xA5)
        {
            lenOnePkg = fp_data[2] | fp_data[3] << 8;
            lenOnePkg += 6;
            if (f_len >= lenOnePkg)
            {
                memcpy((uint8_t *)&gs_msgPkgHI216, fp_data, lenOnePkg);
                ret = 0;
            }
        }
        else if (fp_data[0] == 0x5A && fp_data[1] == 0x5A && fp_data[2] == 0xA5)
        {
            lenOnePkg = fp_data[3] | fp_data[4] << 8;
            lenOnePkg += 6;
            if (f_len >= lenOnePkg)
            {
                memcpy((uint8_t *)&gs_msgPkgHI216, fp_data + 1, lenOnePkg);
                ret = 0;
            }
        }
    }
    return ret;
}

/**
 * @brief 计算crc
 * @param  currectCrc        Param doc
 * @param  src               Param doc
 * @param  lengthInBytes     Param doc
 */
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    for (uint32_t j = 0; j < lengthInBytes; ++j)
    {
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (uint32_t i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currectCrc = crc;
}
/*
*********************************************************************************************************
*    函 数 名: UpdateHI216MUMMes
*    功能说明: 把数据包中的数据解析到具体的变量中
*    形    参:
*    返 回 值:-1=错误，1=失败，0=成功
*********************************************************************************************************
*/
static int UpdateHI216MUMMes(mesGyro_t *fp_mesGyro, msgPkgHI216_t *fp_mesPkgHI216)
{
    int16_t temp = 0;
    int ret = 0;
    uint8_t index = 0;
    uint8_t id;
    uint8_t flagRec = 0;
    uint16_t len = fp_mesPkgHI216->lenHigh << 8 | fp_mesPkgHI216->lenLow;
    if (fp_mesPkgHI216->type != 0xA5)
        return 1;

    while (1)
    {
        if (index >= len)
            break;
        if (flagRec == 0xFF) // 接收到了未知的数据ID
        {
            ret = -1;
            goto out;
        }
        if (flagRec == 0x0F) // 需要的数据都已经接收
        {
            ret = 0;
            goto out;
        }
        id = fp_mesPkgHI216->data[index++];
        switch (id)
        {

        case 0x90:
            index += 1;
            break;
        case 0xA0:
            flagRec |= 1 << 0;
            fp_mesGyro->linearAcceleration_x = fp_mesPkgHI216->data[index + 0] | (fp_mesPkgHI216->data[index + 1] << 8);
            fp_mesGyro->linearAcceleration_y = fp_mesPkgHI216->data[index + 2] | (fp_mesPkgHI216->data[index + 3] << 8);
            fp_mesGyro->linearAcceleration_z = fp_mesPkgHI216->data[index + 4] | (fp_mesPkgHI216->data[index + 5] << 8);
            index += 6;
            break;
        case 0xB0:
            flagRec |= 1 << 1;
            fp_mesGyro->angularVelocity_x = fp_mesPkgHI216->data[index + 0] | (fp_mesPkgHI216->data[index + 1] << 8);
            fp_mesGyro->angularVelocity_y = fp_mesPkgHI216->data[index + 2] | (fp_mesPkgHI216->data[index + 3] << 8);
            fp_mesGyro->angularVelocity_z = fp_mesPkgHI216->data[index + 4] | (fp_mesPkgHI216->data[index + 5] << 8);
            index += 6;
            break;
        case 0xC0:
            flagRec |= 1 << 2;
            fp_mesGyro->magneticField_x = fp_mesPkgHI216->data[index + 0] | (fp_mesPkgHI216->data[index + 1] << 8);
            fp_mesGyro->magneticField_y = fp_mesPkgHI216->data[index + 2] | (fp_mesPkgHI216->data[index + 3] << 8);
            fp_mesGyro->magneticField_z = fp_mesPkgHI216->data[index + 4] | (fp_mesPkgHI216->data[index + 5] << 8);
            index += 6;
            break;
        case 0xD0:
            flagRec |= 1 << 3;
            temp = fp_mesPkgHI216->data[index + 0] | (fp_mesPkgHI216->data[index + 1] << 8);
            fp_mesGyro->pitchAngle = (temp / 10) + 1800;

            temp = fp_mesPkgHI216->data[index + 2] | (fp_mesPkgHI216->data[index + 3] << 8);
            fp_mesGyro->rollAngle = (temp / 10) + 1800;

            temp = fp_mesPkgHI216->data[index + 4] | (fp_mesPkgHI216->data[index + 5] << 8);
            fp_mesGyro->yawAngle = 3600 - temp;
            index += 6;
            break;
        default:
            flagRec = 0xFF;
            break;
        }
    }
    if (flagRec != 0x0F)
        ret = 1;
    else
        ret = 0;
out:
    return ret;
}

/**
 * @brief 外部获取版本号
 * @param  f_version         Param doc
 */
void gyro_hi216_get_version(uint32_t *f_version)
{
    if (f_version != NULL)
    {
        f_version[0] = 1;
        f_version[1] = m_imu_version[0];
        f_version[2] = m_imu_version[1];
        f_version[3] = m_imu_version[2];
    }
}

/**
 * @brief 解析hi216的数据
 * @param  f_type            Param doc
 * @param  fp_gyro           Param doc
 * @param  fp_data           Param doc
 * @param  f_len             Param doc
 * @return int
 */
int gyro_hi216_parse_data(mesGyro_t *fp_gyro, uint8_t *fp_data, uint16_t f_len)
{
    int ret = 0;
    uint16_t crcCheck = 0; // 计算出来的crc
    uint16_t crcRaw = 0;   // 接收到的crc
    uint16_t len = 0;

    ret = RecGYROHI216Data(fp_data, f_len);
    // 有新的数据写入，开始解析
    if (ret == 0)
    {
        len = gs_msgPkgHI216.lenHigh << 8 | gs_msgPkgHI216.lenLow;
        crcRaw = gs_msgPkgHI216.crcCheckHigh << 8 | gs_msgPkgHI216.crcCheckLow;

        crc16_update(&crcCheck, &gs_msgPkgHI216.head, 4);
        crc16_update(&crcCheck, gs_msgPkgHI216.data, len);

        if (crcRaw == crcCheck)
        {
            ret = UpdateHI216MUMMes(fp_gyro, &gs_msgPkgHI216);
        }
        else
            ret = 1;
    }

    return !ret;
}

/**
 * @brief 读取hi216的版本号
 * @param  f_type            Param doc
 * @param  fp_gyro           Param doc
 * @param  fp_data           Param doc
 * @param  f_len             Param doc
 * @return int
 */
int gyro_hi216_read_version(uint8_t *fp_data, uint16_t f_len)
{
    char *p_buf = NULL;

    p_buf = strstr((const char *)fp_data, "HI216");
    if (NULL != p_buf)
    {
        p_buf += +6;
        m_imu_version[0] = atoi(p_buf);
        p_buf = strstr(p_buf, ".") + 1;
        m_imu_version[1] = atoi(p_buf);
        p_buf = strstr(p_buf, ".") + 1;
        m_imu_version[2] = atoi(p_buf);
        return 0;
    }
    return 1;
}
