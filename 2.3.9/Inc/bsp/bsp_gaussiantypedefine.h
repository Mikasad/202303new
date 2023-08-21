#include "canopen_od.h"
#include "bsp_BDCMotor.h"


//typedef struct
//{
//  uint16_t GetADCValue;//获取的ADC值
//  uint16_t FilterValue;//滤波后的ADC值
//  uint16_t MaxValue;   //最大值
//  uint16_t MinValue;   //最小值
//  float    ConversionFactor; //转化系数
//  float    ActualUnitValue;  //转换为实际单位值
//   
//}ADC_ValueParameters_t;
typedef struct  PROTECT_LIMIT
{
  int32_t maxPWM;
  int32_t minPWM;
  int32_t maxAccel;
  int32_t maxDecel;
  int32_t maxCurrent;
  int16_t maxMotorTemperature;
  int16_t maxMosTemperature;
  uint32_t motorOverCurTime;
}MOTOR_PROTECT_LIMIT_CONDITION_T;
typedef struct  DCMotor_Para
{
  uint8_t Enable;
  int32_t TargetPWM;
  int32_t CurPWM;
  uint8_t Direction;
  int16_t Accel;
  int16_t Decel;
  ADC_ValueParameters_t Current;
  MOTOR_PROTECT_LIMIT_CONDITION_T limitcondition;
  int16_t temperature;
  uint8_t brake;
  uint8_t states;
  uint8_t Fault;
}DCMOTOR_PAR_T;
typedef struct  BLDC_Motor
{
  uint8_t Enable;
  uint8_t controlMode;
  int32_t targetVel;
  int32_t velErr; 
  int32_t curVel;
  ADC_ValueParameters_t current;
  MOTOR_PROTECT_LIMIT_CONDITION_T limitcondition;
  int32_t accel;
  int32_t decel;
  int16_t temperature;
  uint8_t brake;
  uint8_t states;
  uint8_t Fault;
}BLDCMOTOR_PAR_T;



extern BLDCMOTOR_PAR_T BLDCMotor1;
extern BLDCMOTOR_PAR_T BLDCMotor2;
extern BLDCMOTOR_PAR_T BLDC_Motoroutside1;
extern BLDCMOTOR_PAR_T BLDC_Motoroutside2;
extern DCMOTOR_PAR_T Two_Way_DCMotor1;
extern DCMOTOR_PAR_T Two_Way_DCMotor2;
extern DCMOTOR_PAR_T Two_Way_DCMotor3;
extern DCMOTOR_PAR_T Two_Way_DCMotor4;
extern DCMOTOR_PAR_T One_Way_DCMotor1;
extern DCMOTOR_PAR_T One_Way_DCMotor2;
extern DCMOTOR_PAR_T Radiotube_Motor1;
extern DCMOTOR_PAR_T Radiotube_Motor2;
extern DCMOTOR_PAR_T Radiotube_Motor3;
extern DCMOTOR_PAR_T Radiotube_Motor4;
