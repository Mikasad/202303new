#ifndef __MYTYPDEF_H
#define __MYTYPDEF_H

#include "stm32f4xx.h"

typedef union {
	uint8_t data_8u[2];
	int16_t data_16;
	uint16_t data_16u;
} U8_16_ChangeType;

typedef struct {
	int16_t throttle; 
	int16_t pitch; 
	int16_t roll;
	int16_t yaw;
	int16_t key;
	int16_t p_pitch;
	int16_t i_pitch;
	int16_t d_pitch;
	int16_t p_roll;
	int16_t i_roll;
	int16_t d_roll;
	int16_t p_yaw;
	int16_t i_yaw;
	int16_t d_yaw;
	int16_t offset_pitch;
	int16_t offset_roll;
	int16_t offset_yaw;
}	RC_RecDataType;

typedef struct {
	float Throttle;
	float Pitch;
	float Roll;
	float Yaw;
}	ANGLE_DATA_F;

typedef struct {
	short X;
	short Y;
	short Z;
}	SENSOR_DATA_I16;

typedef struct {
	float X;
	float Y;
	float Z;
}	SENSOR_DATA_F;

typedef struct {
	float P;
	float I;
	float D;
	float POUT;
	float IOUT;
	float DOUT;
	float IMAX;
	float SetPoint;
	float NowPoint;
	float LastError;
	float PrerError;
} PID;

#endif
