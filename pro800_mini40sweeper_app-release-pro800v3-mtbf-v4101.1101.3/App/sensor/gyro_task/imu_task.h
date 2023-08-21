#ifndef _IMU_TASK_H
#define _IMU_TASK_H

#include "hc32_ll.h"
#include "ringbuffer.h"
#include "hc_uart2.h"

#define IMU_TEMP_BUFFER_SIZE (256)

#define IMU_SEND_CMD(data,len) usart2_dma_send(data, len)
#define IMU_POWERON()  HAL_GPIO_WritePin(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin, GPIO_PIN_SET); //陀螺仪上电
#define IMU_POWEROFF() HAL_GPIO_WritePin(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin, GPIO_PIN_RESET);//陀螺仪下电

typedef enum
{
    IMU_UNKNOW = 0,
    IMU_HI216,
    IMU_ZYF143,
    IMU_JC310,
}ImuType_t;

typedef struct{
	int16_t 	pitchAngle;
	int16_t 	rollAngle;
	int16_t 	yawAngle;		
	int16_t 	angularVelocity_x;
	int16_t 	angularVelocity_y;
	int16_t 	angularVelocity_z;
	int16_t 	linearAcceleration_x;
	int16_t 	linearAcceleration_y;
	int16_t 	linearAcceleration_z;
	int16_t 	magneticField_x;
	int16_t 	magneticField_y;
	int16_t 	magneticField_z;
}mesGyro_t;

typedef struct
{
	uint8_t		imuType;				//IMU型号
	uint8_t		link_status;			//IMU型号
	int32_t 	updataTick;   			//数据更新时间  
}ImuData_t; 


void ImuTask(void const *pvParameters);
void GYRO_UART_IRQHandler(void);
void GetGyroMes(mesGyro_t *fp_mesGyro);
void app_imu_get_version(uint32_t *f_version);
uint8_t GetGyroHealthState(void);
int16_t GetImuYaw(void);
int32_t GetDeltaYaw(void);

ring_buffer_t* imu_task_data_buffer(void);
void app_imu_task(void const *pvParameters);
void app_imu_logic_task(void *Param);

#endif
