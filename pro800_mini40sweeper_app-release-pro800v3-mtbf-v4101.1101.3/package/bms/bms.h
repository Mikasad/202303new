#ifndef _BMS_H
#define _BMS_H

#include "main.h"
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hc_uart1.h"

#define BMS_CMD_SEND(data, len)     \
    do                              \
    {                               \
        usart1_485_send(data, len); \
    } while (0)

#define BMS_TEMP_BUFFER_SIZE 256 /* ringbuffer 缓存长度 */
#define BMS_CELL_NUM 20          /* 电芯数量 */
#define APP_BMS_FRAME_LEN (75) /* 一帧数据长度 */


typedef struct
{
    uint32_t version;
    uint32_t bmsSt;       // BMS状态
    int32_t current;      // 电流
    uint32_t capacityIn;  // 满充容量
    uint32_t capacityOut; // 满放容量
    uint32_t sp;          // 剩余容量
    uint16_t voltage;     // 总电压
    uint16_t soc;         // 剩余电量
    uint16_t cycleTimes;  // 充放电循环次数
    uint16_t protectSt;   // 保护状态
    uint16_t alarmSt;     // 报警状态
    uint16_t balanceSt;   // 均衡状态
    int16_t cellTemp[2];
    int16_t powerBoardTemp;
    int16_t temp;         // 环境温度
    uint16_t cellVoltage[BMS_CELL_NUM];
    int8_t mosfetTemp[2];
    uint8_t cellNum;
    uint8_t temp_num;
} BatteryMsg_t;

typedef enum
{
    BMS_UNKNOW = 0x00,
    BMS_TB,
    BMS_JG,
    BMS_END,
} BmsBattery_t;

typedef enum
{
    BMS_TB_CMD_VERVION = 0,
    BMS_TB_CMD_REST,
    BMS_TB_CMD_INFO,
    BMS_TB_CMD_DEF
}bms_battery_cmd_t;
#if 0
typedef struct
{
    uint8_t rx_data[128];
    uint16_t len;
} bms_pack_t;
#endif
extern uint32_t g_bms_send_count;
extern uint32_t g_bms_recv_count;

void bms_get_battery_data(BatteryMsg_t *battery); /* 获取bms数据 */
void bms_set_disconnec_time(uint32_t f_time);    /* 设置bms最大断连时间 */
void bms_set_recover_time(uint32_t f_time);       /* 设置bms断链回复检测时间 */

uint16_t bms_get_soc(void);
void bms_set_soc(uint32_t f_soc);

int8_t bms_read_info(void);
char *bms_get_version(char *version);
void bms_constructor(void);

int16_t bms_tb_send_cmd(uint8_t cmd);
int16_t bms_read_battery_info(void);
uint32_t get_rs485_bms_version(void);

#endif
