/********************************************************************************
 *文件名            : drv_gs_can_bms.c
 *简  介            : 自研混合驱动器驱动
 *日     期            : 2022/1/4
 *作  者            : 刘德明/高仙
 ********************************************************************************
 *备注：高仙BMS通讯协议，走CAN通讯
 *
 ********************************************************************************/
#include "drv_gs_can_bms.h"
#include "hc_can.h"
#include "objdict.h"

BMS_PARAM_T BmsParam = {NULL};

/*BMS对象字典*/
const Objdict_t bms_objdict[] =
    {
        // 电芯数量
        {BMS_ID, 0x4010, 0x00, uint8, sizeof(uint8_t), RO, &BmsParam.cell_num, NULL},
        // 测温点数量
        {BMS_ID, 0x4011, 0x00, uint8, sizeof(uint8_t), RO, &BmsParam.temp_num, NULL},
        // 根软件版本
        {BMS_ID, 0x4012, 0x00, uint32, sizeof(uint32_t), RO, &BmsParam.boot_version, NULL},
        // 应用软件版本
        {BMS_ID, 0x4013, 0x00, uint32, sizeof(uint32_t), RO, &BmsParam.app_version, NULL},
        // 硬件版本
        {BMS_ID, 0x4014, 0x00, uint32, sizeof(uint32_t), RO, &BmsParam.hard_version, NULL},
        // 总电压
        {BMS_ID, 0x4016, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.bat_volt, NULL},
        // 电流
        {BMS_ID, 0x4017, 0x00, uint32, sizeof(int32_t), RO, &BmsParam.bat_current, NULL},
        // 额定容量
        {BMS_ID, 0x4018, 0x00, uint32, sizeof(uint32_t), RO, &BmsParam.rate_cap, NULL},
        // 满充容量
        {BMS_ID, 0x4019, 0x00, uint32, sizeof(uint32_t), RO, &BmsParam.full_cap, NULL},
        // 剩余容量
        {BMS_ID, 0x4020, 0x00, uint32, sizeof(uint32_t), RO, &BmsParam.surplus_cap, NULL},
        // SOC
        {BMS_ID, 0x4021, 0x00, uint8, sizeof(uint8_t), RO, &BmsParam.bat_soc, NULL},
        // SOH
        {BMS_ID, 0x4022, 0x00, uint8, sizeof(uint8_t), RO, &BmsParam.bat_soh, NULL},
        // 循环次数
        {BMS_ID, 0x4023, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.cycle_num, NULL},
        // 保护状态字
        {BMS_ID, 0x4024, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.protect_status, NULL},
        // 告警状态字
        {BMS_ID, 0x4025, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.warn_status, NULL},
        // bms状态字
        {BMS_ID, 0x4026, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.bms_status, NULL},
        // 均衡状态字
        {BMS_ID, 0x4027, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.balance_status, NULL},
        // 测温点温度
        {BMS_ID, 0x4028, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.cell_temperature[0], NULL},
        {BMS_ID, 0x4028, 0x01, uint16, sizeof(uint16_t), RO, &BmsParam.cell_temperature[1], NULL},
        {BMS_ID, 0x4028, 0x02, uint16, sizeof(uint16_t), RO, &BmsParam.cell_temperature[2], NULL},
        {BMS_ID, 0x4028, 0x03, uint16, sizeof(uint16_t), RO, &BmsParam.cell_temperature[3], NULL},
        {BMS_ID, 0x4028, 0x04, uint16, sizeof(uint16_t), RO, &BmsParam.cell_temperature[4], NULL},
        {BMS_ID, 0x4028, 0x05, uint16, sizeof(uint16_t), RO, &BmsParam.cell_temperature[5], NULL},
        {BMS_ID, 0x4028, 0x06, uint16, sizeof(uint16_t), RO, &BmsParam.cell_temperature[6], NULL},
        {BMS_ID, 0x4028, 0x07, uint16, sizeof(uint16_t), RO, &BmsParam.cell_temperature[7], NULL},
        // 单芯电压度
        {BMS_ID, 0x4029, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.cell_vol[0], NULL},
        {BMS_ID, 0x4029, 0x01, uint16, sizeof(uint16_t), RO, &BmsParam.cell_vol[1], NULL},
        {BMS_ID, 0x4029, 0x02, uint16, sizeof(uint16_t), RO, &BmsParam.cell_vol[2], NULL},
        {BMS_ID, 0x4029, 0x03, uint16, sizeof(uint16_t), RO, &BmsParam.cell_vol[3], NULL},
        {BMS_ID, 0x4029, 0x04, uint16, sizeof(uint16_t), RO, &BmsParam.cell_vol[4], NULL},
        {BMS_ID, 0x4029, 0x05, uint16, sizeof(uint16_t), RO, &BmsParam.cell_vol[5], NULL},
        {BMS_ID, 0x4029, 0x06, uint16, sizeof(uint16_t), RO, &BmsParam.cell_vol[6], NULL},
        {BMS_ID, 0x4029, 0x07, uint16, sizeof(uint16_t), RO, &BmsParam.cell_vol[7], NULL},
        // 放电功率
        {BMS_ID, 0x4052, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.restart_times, NULL},
        // 放电功率
        {BMS_ID, 0x4055, 0x00, uint16, sizeof(uint16_t), RO, &BmsParam.output_power, NULL},

};

const uint16_t bms_objdict_size = sizeof(bms_objdict) / sizeof(Objdict_t);

/**********************************************************************************************
 *函数名    : BMS_LoadObj
 *介     绍    ：添加对象字典
 *形  参 : 无
 *返回值 : 无
 ***********************************************************************************************/
void bms_can2_load_obj(void)
{
    ObjdictLoad2(bms_objdict, bms_objdict_size);
}

/**********************************************************************************************
 *函数名    : BMS_Info_Read
 *介     绍    ：读取BMS信息
 *形  参 : 无
 *返回值 : 无
 ***********************************************************************************************/
void bms_can2_send_cmd(void)
{
    static uint8_t index = 0;

    Can2ReadSDO(bms_objdict[index].id,
                     bms_objdict[index].index,
                     bms_objdict[index].sub_index,
                     bms_objdict[index].data_type,
                     bms_objdict[index].p_data);

    if (index++ >= (bms_objdict_size - 1))
    {
        index = 0;
    }
}
void bms_reset(void)
{
    uint8_t data[8] = {0x2F, 0x60, 0x40, 0x00, 0xFF, 0x00, 0x00, 0x00};

    can_standard_send(CAN2_UNIT, 0x600+BMS_ID, data, sizeof(data));
}

uint8_t bms_get_reset_status(void)
{
    return ((BmsParam.reset == 0xFF) ? 1 : 0);
}

void bms_set_mos_ctrl(uint8_t state)
{
    uint8_t data[8] = {0x2F, 0x61, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00};

    data[4] = state;
    can_standard_send(CAN2_UNIT, 0x600+BMS_ID, data, sizeof(data));
}

uint8_t bms_get_mos_ctrl(void)
{
    return BmsParam.mos_ctrl;
}

int32_t bms_get_charge_current(void)
{
    return BmsParam.bat_current;
}