#ifndef __IMU_H
#define	__IMU_H

#include "stm32f4xx.h"
#include "mytypdef.h"

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);	

#endif /* __IMU_H */
