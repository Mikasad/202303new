#ifndef __GYRO_ZYF143_H_
#define __GYRO_ZYF143_H_

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "semphr.h"
#include "event_groups.h"

#include "security_state.h"
#include "imu_task.h"

#define RCV_BUFFER_LEN_MAX 256U
#define IMU_TEMP_BUFFER_SIZE (256)
#define IMU_FRAME_LEN (18) /* 一帧数据长度 */

extern int gyro_zyf143_read_version(uint8_t* fp_data, uint16_t f_len);
extern int gyro_zyf143_parse_data(mesGyro_t* fp_gyro, uint8_t* fp_data, uint16_t f_len);

extern void gyro_zyf143_get_version(uint32_t *fp_version);

#endif
