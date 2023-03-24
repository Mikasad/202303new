/**
  ******************************************************************************
  * @file    mc_config.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler 
  *          structures declarations.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */ 
  
#ifndef __MC_CONFIG_H
#define __MC_CONFIG_H

#include "pid_regulator.h"
#include "revup_ctrl.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "ntc_temperature_sensor.h"
#include "pwm_curr_fdbk.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "trajectory_ctrl.h"
#include "pqd_motor_power_measurement.h"
 #include "user_interface.h"

#include "motor_control_protocol.h"
#include "usart_frame_communication_protocol.h"

#include "r3_2_f4xx_pwm_curr_fdbk.h"    

#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"
#include "math.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"

#include "encoder_speed_pos_fdbk.h"
#include "enc_align_ctrl.h"
#include "hall_speed_pos_fdbk.h"
#include "ramp_ext_mngr.h"
#include "circle_limitation.h"
#include "stm32f4xx.h"
#include "bsp_flash.h"
#include "public_global.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */  
extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
extern NTC_Handle_t TempSensorParamsM1;
extern PID_Handle_t PID_PosParamsM1;
extern PosCtrl_Handle_t pPosCtrlM1;
extern PID_Handle_t PID_PosParamsM2;
extern PosCtrl_Handle_t pPosCtrlM2;
extern PWMC_R3_2_Handle_t PWM_Handle_M1;
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2;
extern PID_Handle_t PIDSpeedHandle_M2;
extern PID_Handle_t PIDIqHandle_M2;
extern PID_Handle_t PIDIdHandle_M2;
extern NTC_Handle_t TempSensorParamsM2;
extern PID_Handle_t PID_PosParamsM2;
extern PosCtrl_Handle_t pPosCtrlM2;
extern PWMC_R3_2_Handle_t PWM_Handle_M2;
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1; 
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM2;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM2; 
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM2;
extern ENCODER_Handle_t ENCODER_M1;
extern EncAlign_Handle_t EncAlignCtrlM1;
extern ENCODER_Handle_t ENCODER_M2;
extern EncAlign_Handle_t EncAlignCtrlM2;
extern HALL_Handle_t HALL_M1;
extern RDivider_Handle_t RealBusVoltageSensorParamsM1;
extern VirtualBusVoltageSensor_Handle_t VirtualBusVoltageSensorParamsM2;
extern CircleLimitation_Handle_t CircleLimitationM1;
extern CircleLimitation_Handle_t CircleLimitationM2;
extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;
extern RampExtMngr_Handle_t RampExtMngrHFParamsM2;
extern DOUT_handle_t R_BrakeParamsM1;
extern UI_Handle_t UI_Params;
extern FF_Handle_t FF_M1;
extern UFCP_Handle_t pUSART;



/* USER CODE BEGIN Additional extern */

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO?迆米??﹞車3谷?
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) 
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20)  
#define GPIOE_ODR_Addr    (GPIOE_BASE+20)  
#define GPIOF_ODR_Addr    (GPIOF_BASE+20)    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20)   

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16)  
 
//IO?迆2迄℅¯,????米ㄓ辰?米?IO?迆!
//豕﹞㊣㏒n米??米D?車迆16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //那?3? 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //那?豕? 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //那?3? 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //那?豕? 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //那?3? 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //那?豕? 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //那?3? 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //那?豕? 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //那?3? 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //那?豕?

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //那?3? 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //那?豕?

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //那?3? 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //那?豕?


#define IIC_SCL    PBout(6)         // SCL
#define IIC_SDA    PBout(9)         // SDA	 
#define READ_SDA   PBin(9)          // 怀�蕦DA 
/* USER CODE END Additional extern */  
#define NBR_OF_MOTORS 2

extern PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
extern PID_Handle_t PIDSpeedHandle_M1;
#endif /* __MC_CONFIG_H */
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
