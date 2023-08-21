#ifndef __BMS_CAN__H__
#define __BMS_CAN__H__

#include "hc32_ll.h"
#include "stdio.h"

#define BMS_ID 4

typedef struct
{
    uint32_t boot_version;         // boot版本
    uint32_t app_version;          // 软件版本
    uint32_t hard_version;         // 硬件版本
    uint8_t cell_num;              // 电芯数量
    uint8_t temp_num;              // 测温点数量
    uint8_t bat_soc;               // SOC
    uint8_t bat_soh;               // SOH
    uint16_t bat_volt;             // 总电压
    uint16_t cycle_num;            // 循环次数
    uint16_t protect_status;       // 保护状态字
    uint16_t warn_status;          // 告警状态字
    uint16_t bms_status;           // bms状态字
    uint16_t balance_status;       // 均衡状态字
    int16_t cell_temperature[8];   // 温度
    uint16_t cell_vol[8];          // 单芯电压
    uint32_t last_heart_tick;      // 上次数据反馈的时间
    int32_t bat_current;           // 总电流
    uint32_t rate_cap;             // 额定容量
    uint32_t full_cap;             // 满充容量
    uint32_t surplus_cap;          // 剩余容量
    uint16_t restart_times;        // 重启次数
    uint16_t output_power;         // 放电功率
    uint8_t mos_ctrl;
    uint8_t reset;
} BMS_PARAM_T;

typedef struct
{
    stc_can_rx_frame_t rx_frame;
    uint32_t frame_num;
} bms_can_pack_t;

extern BMS_PARAM_T BmsParam;

void bms_can2_send_cmd(void);

void bms_can2_load_obj(void); /*添加对象字典*/
void bms_can2_send_cmd(void); /*读取BMS信息*/
void bms_reset(void);
uint8_t bms_get_reset_status(void);
void bms_set_mos_ctrl(uint8_t state);
uint8_t bms_get_mos_ctrl(void);
int32_t bms_get_charge_current(void);
#endif
