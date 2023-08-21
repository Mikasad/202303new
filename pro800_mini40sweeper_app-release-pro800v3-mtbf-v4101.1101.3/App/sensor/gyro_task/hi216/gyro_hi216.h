#ifndef __GYRO_HI216_H_
#define __GYRO_HI216_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"
#include "security_state.h"
#include "imu_task.h"

#define HI216_DATA_BUF_SIZE 128U

#pragma pack(1)
typedef struct{
    uint8_t head;
    uint8_t type;
    uint8_t lenLow;
    uint8_t lenHigh;
    uint8_t crcCheckLow;
    uint8_t crcCheckHigh;
    uint8_t data[HI216_DATA_BUF_SIZE];
}msgPkgHI216_t;
#pragma pack()

void gyro_hi216_get_version(uint32_t *f_version);

int gyro_hi216_parse_data(mesGyro_t *fp_gyro, uint8_t *fp_data, uint16_t f_len);
int gyro_hi216_read_version(uint8_t *fp_data, uint16_t f_len);

#endif
