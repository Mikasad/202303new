#include "bms.h"
#include "scada.h"
#include <stdio.h>
#include "main.h"
#include "hc_uart1.h"
#include "ringbuffer.h"
#include "nanopb_tcpip.h"
#include "string.h"

#define RS485_BMS_ID        0x06
#define RS485_BMS_FUNC_CODE 0x05
#define RS485_BMS_VER_CODE  0x17
#define RS485_BMS_VER_OFFSET 0x08

#define CALL_FREQ (float)1 // 单位Hz，2秒调用一次则为0.5
#define BMS_BPS_TB 19200   // 拓邦电池bms通讯波特率
#define BMS_BPS_JG 9600    // 精工电池bms通讯波特率
#define BMS_TB_CMD_NUM 2   // 指令数量
#define BMS_TB_FRAME_LEN 8 // 指令帧长度
#define BMS_JG_CMD_NUM 1   // 精工协议要发送的命令条数
#define BMS_TB_CELL_NUM 16 // 拓邦电池电芯个数

uint32_t g_bms_send_count = 0;
uint32_t g_bms_recv_count = 0;

/* 发给电池包的指令 */
uint8_t bms_tb_cmd_get_version[] = {0x06, 0x03, 0x17, 0xD1, 0x3F};
uint8_t bms_tb_cmd_set_rest[] = {0x06, 0x03, 0x30, 0x91, 0x25};
uint8_t bms_tb_cmd_get_info[] = {0x06, 0x06, 0x05, 0x00, 0x10, 0x1F, 0xC4, 0xB9};

/* 数据缓存区 */
static ring_buffer_t bms_data_buffer; // imu数据缓冲区

static int8_t bms_tb_frame_version_handle(uint8_t *data, uint8_t len);
static int8_t bms_tb_frame_data_handle(uint8_t *data, uint8_t len);
static int8_t bms_process_frame_data(uint8_t *data, uint8_t len);

static uint16_t CRC16RTU(uint8_t *f_data, uint32_t f_len);

/*
    协议帧定义
    byte[0] = 1表示只读一次; byte[0] = 0xFF表示周期读取;
    byte[1]表示要发送内容的字节数;
    byte[2]开始为具体数据帧内容。将单次发送的命令排在前面;
*/
static uint8_t m_tb_cmd[BMS_TB_CMD_NUM][10] = {0xFF, 0x08, 0x06, 0x06, 0x05, 0x00, 0x10, 0x1F, 0xC4, 0xB9};
static uint8_t m_jg_cmd[BMS_JG_CMD_NUM][10] = {0xFF, 0x06, 0x7F, 0x10, 0x02, 0x06, 0x12, 0x57};
static BmsBattery_t m_battery_type = BMS_UNKNOW; // 电池类型
static BatteryMsg_t m_battery_msg = {0};

static uint16_t m_bms_disconnect_count = 0;              // 连续未接收到bms回复数据的时间，单位秒
static uint16_t m_bms_max_disconnect_time = 60;          // 允许的bms断连最长时间，默认60秒
static uint16_t m_bms_max_disconnect_time_in_init = 180; // 在刚上电时允许的最大bms断链时间
static uint16_t m_bms_recover_time = 0;                  // bms断链后稳定时间，需大于断连检测时间

/**
 * @brief 读取bms版本号
 * @param  version           Param doc
 * @return char*
 */
char *bms_get_version(char *version)
{
    uint8_t times = 0;
    bms_pack_t bms_pack;

    if (version == NULL)
        return NULL;
    do
    {
        if (xQueueReceive(bms_usart1_quene_handle, &bms_pack, 2000) == pdTRUE)
        {
            if (bms_pack.len > 0)
            {
                memcpy(version, &bms_pack.rx_data, bms_pack.len);
                printf("bms version : %s .\r\n", version);
                return version;
            }
        }
        else
        {
            printf("get bms version over times = %d .\r\n", times);
        }
        times++;
        //1000ms
        
    } while (times < 10); /* 循环读取版本号 10次*/

    return NULL;
}

/**
 * @brief 读取电池信息指令
 */
int16_t bms_tb_send_cmd(uint8_t cmd)
{
    if (cmd >= BMS_TB_CMD_DEF)
        return -1;
    switch (cmd)
    {
    case BMS_TB_CMD_VERVION:
        BMS_CMD_SEND(bms_tb_cmd_get_version, sizeof(bms_tb_cmd_get_version) / sizeof(bms_tb_cmd_get_version[0]));
        break;
    case BMS_TB_CMD_REST:
        BMS_CMD_SEND(bms_tb_cmd_set_rest, sizeof(bms_tb_cmd_set_rest) / sizeof(bms_tb_cmd_set_rest[0]));
        break;
    case BMS_TB_CMD_INFO:
        BMS_CMD_SEND(bms_tb_cmd_get_info, sizeof(bms_tb_cmd_get_info) / sizeof(bms_tb_cmd_get_info[0]));
        break;
    default:
        break;
    }
    return 0;
}

/**
 * @brief 获取电池包信息
 * @param  f_bms             Param doc
 */
int16_t bms_read_battery_info(void)
{
    static uint8_t invalid_pack_count = 0;
    bms_pack_t bms_pack;

    if (xQueueReceive(bms_usart1_quene_handle, &bms_pack, 10000) == pdTRUE) // 10S
    {
        /* 数据长度异常处理 */
        if (bms_pack.len <= 0)
        {
            invalid_pack_count++;
        }
        else
        {
            if(bms_process_frame_data(bms_pack.rx_data, bms_pack.len) == 0)
            {
                invalid_pack_count = 0;
                return 0;
            }
            else
            {
                invalid_pack_count++;
            }
        }
        /* 在收到数据的情况下,运行容错20次 */
        if (invalid_pack_count >= 20)
        {
            invalid_pack_count = 21;
            return -1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        printf("bms recv over times ---> %d .\r\n",__LINE__);
        return -1;
    }
    return -1;
}

/**
 * @brief
 * @return int8_t
 */
static int8_t bms_process_frame_data(uint8_t *data, uint8_t len)
{
    int res = -1;
    ring_buffer_t *p_buffer = &bms_data_buffer;
    uint8_t buffer[BMS_TEMP_BUFFER_SIZE];
    uint8_t buffer_len = 0;
    uint8_t temp;
    uint8_t temp_len = 0;
    uint8_t fun_cmd = 0;

    /* 将原始数据入队 */
    ring_buffer_queue_arr(p_buffer, data, len);
    uint16_t buf_items_num = ring_buffer_num_items(p_buffer);
    if(buf_items_num < APP_BMS_FRAME_LEN)
    {
        printf("bms recv data len = (%d) warning. \r\n", buf_items_num);
        return res;
    } /* 缓存的没用数据过多，清除掉*/
    else if(buf_items_num > (BMS_TEMP_BUFFER_SIZE - APP_BMS_FRAME_LEN) )
    {
        printf("bms recv data delete = (%d) warning. \r\n", buf_items_num);
        /* 删除队列数据 */
        ring_buffer_queue_arr(p_buffer, buffer, buf_items_num);
        return res;
    }
    else
    {
        //for(uint8_t i = 0; i < buf_items_num;i++)
        do{
            if(buf_items_num < APP_BMS_FRAME_LEN)
            {
                printf("bms ring buffer not frame.\r\n");
                return res;
            }
            /* 找帧头 */
            if( ring_buffer_dequeue(p_buffer, &temp) == 1)
            {
                if(RS485_BMS_ID == temp)
                {
                    buffer_len = buffer_len + 1;
                    buffer[0] = temp;
                    /* 获取数据长度 和 命令号 */
                    ring_buffer_dequeue_arr(p_buffer, &buffer[1], 2);
                    temp_len = buffer[1];
                    fun_cmd  = buffer[2];
                    if((RS485_BMS_FUNC_CODE == fun_cmd) || (RS485_BMS_VER_CODE == fun_cmd))
                    {
                        buffer_len = buffer_len + 2;

                        uint8_t dat_len = ring_buffer_dequeue_arr(p_buffer, &buffer[3], (temp_len - 1));
                        if( dat_len == (temp_len - 1))
                        {
                            buffer_len = buffer_len + dat_len;
                            /* 处理一帧数据 */
                            if (RS485_BMS_FUNC_CODE == fun_cmd)
                                res = bms_tb_frame_data_handle(buffer, buffer_len);
                            else if (RS485_BMS_VER_CODE == fun_cmd)
								res = bms_tb_frame_version_handle(buffer, buffer_len);
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
                printf("bms ring buffer error.\r\n");
                return -1;
            }
            buf_items_num = ring_buffer_num_items(p_buffer);
        }while (1);       
    }
    return -1;
}

/**
 * @brief 组包计算
 * @return uint8_t
 */
uint8_t bms_tb_shift_calculation(uint8_t *data, uint8_t offset)
{
    ;
}

//处理拓邦电池版本数据
static int8_t bms_tb_frame_version_handle(uint8_t *data, uint8_t len)
{
    uint16_t crc_src = 0;
    uint16_t crc_cal = 0;
	
    crc_src = (data[len - 1] << 8) | data[len - 2];
    crc_cal = CRC16RTU(data, (len - 2));

    if (crc_src == crc_cal)
    {
        /* 拓邦电池回复的版本号为ASCII，格式为Vx.y */
        m_battery_msg.version = ((data[RS485_BMS_VER_OFFSET] - 0x30) << 8) | (data[RS485_BMS_VER_OFFSET+2] - 0x30);

        return 0;
    }

	return -1;
}

/**
 * @brief 处理一帧完整数据
 * @param  data              Param doc
 * @param  len               Param doc
 * @return int
 */
static int8_t bms_tb_frame_data_handle(uint8_t *data, uint8_t len)
{
    int ret = 0;
    uint16_t crc_src = 0;
    uint16_t crc_cal = 0;
    int i = i, j = 0;
    crc_src = (data[len - 1] << 8) | data[len - 2];
    crc_cal = CRC16RTU(data, (len - 2));

    if (crc_src == crc_cal)
    {
        i = 3;
        m_battery_msg.current = (data[i] << 24) | (data[i + 1] << 16) | (data[i + 2] << 8) | data[i + 3];
        i += 4;
        m_battery_msg.capacityIn = (data[i] << 24) | (data[i + 1] << 16) | (data[i + 2] << 8) | data[i + 3];
        i += 4;
        m_battery_msg.capacityOut = (data[i] << 24) | (data[i + 1] << 16) | (data[i + 2] << 8) | data[i + 3];
        i += 4;
        m_battery_msg.sp = (data[i] << 24) | (data[i + 1] << 16) | (data[i + 2] << 8) | data[i + 3];
        i += 4;
        m_battery_msg.voltage = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.soc = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.cycleTimes = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.protectSt = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.alarmSt = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.bmsSt = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.balanceSt = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.cellTemp[0] = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.cellTemp[1] = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.powerBoardTemp = (data[i] << 8) | data[i + 1];
        i += 2;
        m_battery_msg.temp = (data[i] << 8) | data[i + 1];
        i += 2;
        for (j = 0; j < BMS_TB_CELL_NUM; j++)
        {
            m_battery_msg.cellVoltage[j] = (data[i] << 8) | data[i + 1];
            i += 2;
        }
        m_battery_msg.cellNum = BMS_TB_CELL_NUM;
        m_battery_msg.temp_num = 2;

        m_battery_msg.capacityIn /= 100;
        m_battery_msg.sp /= 100;
    }
    else
    {
        ret = -1;
    }

    return ret;
}

/**
 * @brief 初始化bms相关量
 */
void bms_constructor(void)
{
    ring_buffer_create(&bms_data_buffer, BMS_TEMP_BUFFER_SIZE);
}

/******************************************************
*函数名称:CRC16RTU
*输   入:pszBuf  要校验的数据
        unLength 校验数据的长
*输   出:校验值
*功   能:循环冗余校验-16
         （RTU标准-0xA001）
*******************************************************/
static uint16_t CRC16RTU(uint8_t *f_data, uint32_t f_len)
{
    uint16_t crc = 0xFFFF;
    uint32_t i = 0;

    for (i = 0; i < f_len; i++)
    {
        crc = crc ^ *(f_data + i);
        for (int i = 0; i < 8; i++)
        {
            if (crc & 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 获取电池包的数据
 * @param  battery 赋值缓存
 */
void bms_get_battery_data(BatteryMsg_t *battery)
{
    if (battery != NULL)
    {
        *battery = m_battery_msg;
    }
}

uint16_t GetChargeCurrent(void)
{
    return (m_battery_msg.current > 0) ? m_battery_msg.current : 0;
}

/**
 * @brief 设置 检查 断联时间(多长时间没收到数据视为断联)
 * @param  f_time            Param doc
 */
void bms_set_disconnec_time(uint32_t f_time)
{
    if (f_time >= 10)
    {
        m_bms_max_disconnect_time = f_time;
    }
}

/**
 * @brief 设置断联恢复时间
 * @param  f_time            Param doc
 */
void bms_set_recover_time(uint32_t f_time)
{
    m_bms_recover_time = f_time;
}

/**
 * @brief 获取剩余电量
 * @return uint16_t 
 */
uint16_t bms_get_soc(void)
{
    return m_battery_msg.soc;
}

/**
 * @brief 断联时,设置剩余电量
 * @param  f_soc             Param doc
 */
void bms_set_soc(uint32_t f_soc)
{
    m_battery_msg.soc = f_soc;
}

uint32_t get_rs485_bms_version(void)
{
	return m_battery_msg.version;
}

