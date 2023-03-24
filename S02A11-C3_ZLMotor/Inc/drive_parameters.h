
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

/**************************
 *** Motor 1 Parameters ***
 **************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM       5000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM       0 /*!< rpm, mechanical,  
                                                           absolute value */
#define MEAS_ERRORS_BEFORE_FAULTS       3 /*!< Number of speed  
                                                             measurement errors before 
                                                             main sensor goes in fault */
/*** Encoder **********************/                                                                                                           
#define ENC_MEAS_ERRORS_BEFORE_FAULTS   3 /*!< Number of failed   
                                                        derived class specific speed 
                                                        measurements before main sensor  
                                                        goes in fault */

#define ENC_INVERT_SPEED                DISABLE  /*!< To be enabled for  
                                                            encoder (main or aux) if  
                                                            measured speed is opposite 
                                                            to real one */        
#define ENC_AVERAGING_FIFO_DEPTH        1 /*!< depth of the FIFO used to 
                                                              average mechanical speed in 
                                                              0.1Hz resolution */
/****** Hall sensors ************/ 
#define HALL_MEAS_ERRORS_BEFORE_FAULTS  3 /*!< Number of failed   
                                                           derived class specific speed 
                                                           measurements before main sensor  
                                                           goes in fault */

#define HALL_AVERAGING_FIFO_DEPTH        6 /*!< depth of the FIFO used to 
                                                           average mechanical speed in 
                                                           0.1Hz resolution */  
#define HALL_MTPA  false                                                            

/* USER CODE BEGIN angle reconstruction M1 */
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */

#define PWM_FREQUENCY   10000
#define PWM_FREQ_SCALING 1

#define LOW_SIDE_SIGNALS_ENABLING        LS_PWM_TIMER
#define SW_DEADTIME_NS                   800 /*!< Dead-time to be inserted  
                                                           by FW, only if low side 
                                                           signals are enabled */
                                                                                         
/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in 
                                                           number of PWM cycles */     
/* Gains values for torque and flux control loops */
//#define PID_TORQUE_KP_DEFAULT         300//2112       
//#define PID_TORQUE_KI_DEFAULT         200
//#define PID_TORQUE_KD_DEFAULT         100
//#define PID_FLUX_KP_DEFAULT           300
//#define PID_FLUX_KI_DEFAULT          200
//#define PID_FLUX_KD_DEFAULT           100

#define PID_TORQUE_KP_DEFAULT         150//2112       
#define PID_TORQUE_KI_DEFAULT         20
#define PID_TORQUE_KD_DEFAULT         0

#define PID_FLUX_KP_DEFAULT           150 
#define PID_FLUX_KI_DEFAULT           20
#define PID_FLUX_KD_DEFAULT           0

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      256
#define TF_KIDIV                      8192
#define TF_KDDIV                      8192
#define TF_KPDIV_LOG                  LOG2(256)
#define TF_KIDIV_LOG                  LOG2(8192)
#define TF_KDDIV_LOG                  LOG2(8192)
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

#define POSITION_LOOP_FREQUENCY_HZ    500 /*!<Execution rate of position control regulation loop (Hz) */
/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ       1000 /*!<Execution rate of speed   
                                                      regulation loop (Hz) */
#define PID_SPEED_KP_DEFAULT         1500//1000 /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT         2000// 2000 /* Workbench compute the gain for 01Hz unit*/

//#define PID_SPEED_KP_DEFAULT          10000/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
//#define PID_SPEED_KI_DEFAULT          3000/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV                      16
#define SP_KIDIV                      256
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2(16)
#define SP_KIDIV_LOG                  LOG2(256)
#define SP_KDDIV_LOG                  LOG2(16)

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX                          MAX_CURRENT//11915

/* Default settings */
#define DEFAULT_CONTROL_MODE           STC_SPEED_MODE /*!< STC_TORQUE_MODE or 
                                                        STC_SPEED_MODE */
#define DEFAULT_TARGET_SPEED_RPM      0//150
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/_RPM)
#define DEFAULT_TORQUE_COMPONENT       0
#define DEFAULT_FLUX_COMPONENT         0

//#define PID_POSITION_KP_GAIN			12000
//#define PID_POSITION_KI_GAIN			200
//#define PID_POSITION_KD_GAIN			1000//1000

#define PID_POSITION_KP_GAIN			1000
#define PID_POSITION_KI_GAIN			0
#define PID_POSITION_KD_GAIN			0//1000

#define PID_POSITION_KPDIV				1024     
#define PID_POSITION_KIDIV				32768
#define PID_POSITION_KDDIV				16
#define PID_POSITION_KPDIV_LOG			LOG2(1024)    
#define PID_POSITION_KIDIV_LOG			LOG2(32768) 
#define PID_POSITION_KDDIV_LOG			LOG2(16) 
#define PID_POSITION_ANGLE_STEP			10.0
#define PID_POSITION_MOV_DURATION		10.0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_PROT_ENABLING        ENABLE
#define UV_VOLTAGE_PROT_ENABLING        ENABLE
#define OV_VOLTAGE_THRESHOLD_V          36 /*!< Over-voltage 
                                                         threshold */
#define UD_VOLTAGE_THRESHOLD_V          16 /*!< Under-voltage 
                                                          threshold */
#if 0
#define ON_OVER_VOLTAGE                 TURN_ON_R_BRAKE /*!< TURN_OFF_PWM, 
                                                         TURN_ON_R_BRAKE or 
                                                         TURN_ON_LOW_SIDES */
#endif /* 0 */
#define R_BRAKE_SWITCH_OFF_THRES_V      28

#define OV_TEMPERATURE_THRESHOLD_C      70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     10 /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /*!< In case ON_OVER_VOLTAGE  
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by 
                                                          power stage) */
/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
#define ALIGNMENT_DURATION              10 /*!< milliseconds */
#define ALIGNMENT_ANGLE_DEG             0 /*!< degrees [0...359] */
//#define FINAL_I_ALIGNMENT               11913    // 10A==11913   R=0.003    11111;5957 /*!< s16A */

#define FINAL_I_ALIGNMENT               7944    // 10A==7943.5   R=0.002     /*!< s16A */

// With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment 
// phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt)  
// being Av the voltage gain between Rshunt and A/D input

#define TRANSITION_DURATION            0  /* Switch over duration, ms */ 
/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(28)

#define  M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(15)

#define  M2_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(15)
/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES (15 + SAMPLING_CYCLE_CORRECTION)


#define FEED_FORWARD_CURRENT_REG_ENABLING ENABLE
#define CONSTANT1_Q                    268336
#define CONSTANT1_D                    268336
#define CONSTANT2_QD                   105
/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Motor 2 Parameters ***
 **************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM2           5000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM2           0 /*!< rpm, mechanical,  
                                                           absolute value */
#define MEAS_ERRORS_BEFORE_FAULTS2       3 /*!< Number of speed  
                                                             measurement errors before 
                                                             main sensor goes in fault */
/*** Encoder **********************/                                                                                                           
#define ENC_MEAS_ERRORS_BEFORE_FAULTS2   3 /*!< Number of failed   
                                                        derived class specific speed 
                                                        measurements before main sensor  
                                                        goes in fault */
#define ENC_INVERT_SPEED2                DISABLE  /*!< To be enabled for  
                                                            encoder (main or aux) if  
                                                            measured speed is opposite 
                                                            to real one */        
#define ENC_AVERAGING_FIFO_DEPTH2        1 /*!< depth of the FIFO used to 
                                                              average mechanical speed in 
                                                              0.1Hz resolution */

/* USER CODE BEGIN angle reconstruction M2 */
#define REV_PARK_ANGLE_COMPENSATION_FACTOR2 0
/* USER CODE END angle reconstruction M2 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* Dual drive specific parameters */
#define FREQ_RATIO                      1  /* Higher PWM frequency/lower PWM frequency */  
#define FREQ_RELATION                   HIGHEST_FREQ  /* It refers to motor 1 and can be 
                                                           HIGHEST_FREQ or LOWEST frequency depending 
                                                           on motor 1 and 2 frequency relationship */
#define FREQ_RELATION2                  HIGHEST_FREQ   /* It refers to motor 2 and can be 
                                                           HIGHEST_FREQ or LOWEST frequency depending 
                                                           on motor 1 and 2 frequency relationship */

/* PWM generation and current reading */
#define PWM_FREQUENCY2                    10000
#define PWM_FREQ_SCALING2 1
 
#define LOW_SIDE_SIGNALS_ENABLING2        LS_PWM_TIMER
#define SW_DEADTIME_NS2                   800 /*!< Dead-time to be inserted  
                                                           by FW, only if low side 
                                                           signals are enabled */
/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE2     1    /*!< FOC execution rate in 
                                                           number of PWM cycles */     
/* Gains values for torque and flux control loops */
//#define PID_TORQUE_KP_DEFAULT2        400// 100 //2112       
//#define PID_TORQUE_KI_DEFAULT2         100//3 
//#define PID_TORQUE_KD_DEFAULT2         100
//#define PID_FLUX_KP_DEFAULT2           400 //2112
//#define PID_FLUX_KI_DEFAULT2           100 //3942
//#define PID_FLUX_KD_DEFAULT2           100

#define PID_TORQUE_KP_DEFAULT2         150// 100 //2112       
#define PID_TORQUE_KI_DEFAULT2         20//3 
#define PID_TORQUE_KD_DEFAULT2         0
#define PID_FLUX_KP_DEFAULT2           150 //2112
#define PID_FLUX_KI_DEFAULT2           20 //3942
#define PID_FLUX_KD_DEFAULT2           0

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV2                      256
#define TF_KIDIV2                      8192
#define TF_KDDIV2                      8192
#define TF_KPDIV_LOG2                  LOG2(256)
#define TF_KIDIV_LOG2                  LOG2(8192)
#define TF_KDDIV_LOG2                  LOG2(8192)

#define TFDIFFERENTIAL_TERM_ENABLING2  DISABLE

#define POSITION_LOOP_FREQUENCY_HZ2    500 /*!<Execution rate of position control regulation loop (Hz) */
/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ2       1000 /*!<Execution rate of speed   
                                                      regulation loop (Hz) */
													  
#define PID_SPEED_KP_DEFAULT2         1500//5000 /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT2          2000 /* Workbench compute the gain for 01Hz unit*/
//#define PID_SPEED_KP_DEFAULT2          10000/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
//#define PID_SPEED_KI_DEFAULT2          3000/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT2          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV2                      16
#define SP_KIDIV2                      256
#define SP_KDDIV2                      16
#define SP_KPDIV_LOG2                  LOG2(16)
#define SP_KIDIV_LOG2                  LOG2(256)
#define SP_KDDIV_LOG2                  LOG2(16)

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV2 */
#define PID_SPEED_INTEGRAL_INIT_DIV2 1 
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV2 */

#define SPD_DIFFERENTIAL_TERM_ENABLING2 DISABLE
#define IQMAX2                          MAX_CURRENT//11915

/* Default settings */
#define DEFAULT_CONTROL_MODE2           STC_SPEED_MODE /*!< STC_TORQUE_MODE or 
                                                        STC_SPEED_MODE */  
#define DEFAULT_TARGET_SPEED_RPM2    0//150
#define DEFAULT_TARGET_SPEED_UNIT2      (DEFAULT_TARGET_SPEED_RPM2*SPEED_UNIT/_RPM)
#define DEFAULT_TORQUE_COMPONENT2       0
#define DEFAULT_FLUX_COMPONENT2         0

//#define PID_POSITION_KP_GAIN2			12000
//#define PID_POSITION_KI_GAIN2			200
//#define PID_POSITION_KD_GAIN2			1000

#define PID_POSITION_KP_GAIN2			50
#define PID_POSITION_KI_GAIN2			0
#define PID_POSITION_KD_GAIN2			0

#define PID_POSITION_KPDIV2				1024     
#define PID_POSITION_KIDIV2				32768
#define PID_POSITION_KDDIV2				16
#define PID_POSITION_KPDIV_LOG2			LOG2(1024)    
#define PID_POSITION_KIDIV_LOG2			LOG2(32768) 
#define PID_POSITION_KDDIV_LOG2			LOG2(16) 
#define PID_POSITION_ANGLE_STEP2			10.0
#define PID_POSITION_MOV_DURATION2		10.0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_PROT_ENABLING2        ENABLE
#define UV_VOLTAGE_PROT_ENABLING2        ENABLE
#define OV_VOLTAGE_THRESHOLD_V2          36 /*!< Over-voltage 
                                                         threshold */
#define UD_VOLTAGE_THRESHOLD_V2          10 /*!< Under-voltage 
                                                          threshold */
#if 0
#define ON_OVER_VOLTAGE2                 TURN_OFF_PWM /*!< TURN_OFF_PWM, 
                                                         TURN_ON_R_BRAKE or 
                                                         TURN_ON_LOW_SIDES */
#endif /* 0 */
                                                         
#define R_BRAKE_SWITCH_OFF_THRES_V2      28

#define OV_TEMPERATURE_THRESHOLD_C2      70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C2     10 /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS2       DISABLE /*!< In case ON_OVER_VOLTAGE  
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by 
                                                          power stage) */
/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
#define ALIGNMENT_DURATION2              1000 /*!< milliseconds */
#define ALIGNMENT_ANGLE_DEG2             0 /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT2               7944 //11913   //5957 /*!< s16A */
// With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment 
// phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt)  
// being Av the voltage gain between Rshunt and A/D input

#define TRANSITION_DURATION2            0  /* Switch over duration, ms */

/******************************   Current sensing Motor 2   **********************/
#define ADC_SAMPLING_CYCLES2 (15 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*__DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
