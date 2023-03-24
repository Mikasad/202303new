/**
  ******************************************************************************
  * @file    mc_config.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler structures.
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
#include "main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_parameters.h"
#include "mc_config.h"
#include "pmsm_motor_parameters.h"



/* USER CODE BEGIN Additional include */
long  HALL_PHASE_SHIFT = -20600;
long  HALL_PHASE_SHIFT2 = -20600;


/* USER CODE END Additional include */

#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#include "pqd_motor_power_measurement.h"
/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
    .wConvFact = PQD_CONVERSION_FACTOR
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1;

PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM2=
{
    .wConvFact = PQD_CONVERSION_FACTOR2
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM2 = &PQD_MotorPowMeasM2;

/**
  * @brief  PI / PID Speed loop parameters Motor 1
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
    .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
    .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT,
    .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
    .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
    .hUpperOutputLimit       = (int16_t)IQMAX,
    .hLowerOutputLimit       = -(int16_t)IQMAX,
    .hKpDivisor          = (uint16_t)SP_KPDIV,
    .hKiDivisor          = (uint16_t)SP_KIDIV,
    .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
    .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
    .hDefKdGain           = 0x0000U,
    .hKdDivisor           = 0x0000U,
    .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 1
  */

FF_Handle_t FF_M1 =
{
    .hVqdLowPassFilterBW    = M1_VQD_SW_FILTER_BW_FACTOR,
    .wDefConstant_1D        = (int32_t)CONSTANT1_D,
    .wDefConstant_1Q        = (int32_t)CONSTANT1_Q,
    .wDefConstant_2         = (int32_t)CONSTANT2_QD,
    .hVqdLowPassFilterBWLOG = M1_VQD_SW_FILTER_BW_FACTOR_LOG
};


PID_Handle_t PIDIqHandle_M1 =
{
    .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
    .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
    .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
    .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
    .hUpperOutputLimit       = INT16_MAX,
    .hLowerOutputLimit       = -INT16_MAX,
    .hKpDivisor          = (uint16_t)TF_KPDIV,
    .hKiDivisor          = (uint16_t)TF_KIDIV,
    .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
    .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
    .hDefKdGain           = 0x0000U,
    .hKdDivisor           = 0x0000U,
    .hKdDivisorPOW2       = 0x0000U,
};
/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PID_Handle_t PIDIdHandle_M1 =
{
    .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT,
    .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT,
    .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
    .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
    .hUpperOutputLimit       = INT16_MAX,
    .hLowerOutputLimit       = -INT16_MAX,
    .hKpDivisor          = (uint16_t)TF_KPDIV,
    .hKiDivisor          = (uint16_t)TF_KIDIV,
    .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,
    .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,
    .hDefKdGain           = 0x0000U,
    .hKdDivisor           = 0x0000U,
    .hKdDivisorPOW2       = 0x0000U,
};

//PID_Handle_t PID_PosParamsM1 =
//{
//  .hDefKpGain          = (int16_t)PID_POSITION_KP_GAIN,
//  .hDefKiGain          = (int16_t)PID_POSITION_KI_GAIN,
//  .hDefKdGain          = (int16_t)PID_POSITION_KD_GAIN,
//  .wUpperIntegralLimit = (int32_t)NOMINAL_CURRENT * (int32_t)PID_POSITION_KIDIV,
//  .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT) * (int32_t)PID_POSITION_KIDIV,
//  .hUpperOutputLimit   = (int16_t)NOMINAL_CURRENT,
//  .hLowerOutputLimit   = -(int16_t)NOMINAL_CURRENT,
//  .hKpDivisor          = (uint16_t)PID_POSITION_KPDIV,
//  .hKiDivisor          = (uint16_t)PID_POSITION_KIDIV,
//  .hKdDivisor          = (uint16_t)PID_POSITION_KDDIV,
//  .hKpDivisorPOW2      = (uint16_t)PID_POSITION_KPDIV_LOG,
//  .hKiDivisorPOW2      = (uint16_t)PID_POSITION_KIDIV_LOG,
//  .hKdDivisorPOW2      = (uint16_t)PID_POSITION_KDDIV_LOG,
//};

PID_Handle_t PID_PosParamsM1 =
{
    .hDefKpGain          = (int16_t)PID_POSITION_KP_GAIN,
    .hDefKiGain          = (int16_t)PID_POSITION_KI_GAIN,
    .hDefKdGain          = (int16_t)PID_POSITION_KD_GAIN,
    .wUpperIntegralLimit = (int32_t)MAX_APPLICATION_SPEED_RPM * (int32_t)PID_POSITION_KIDIV,
    .wLowerIntegralLimit = (int32_t)(-MAX_APPLICATION_SPEED_RPM) * (int32_t)PID_POSITION_KIDIV,
    .hUpperOutputLimit   = (int16_t)MAX_APPLICATION_SPEED_RPM,
    .hLowerOutputLimit   = -(int16_t)MAX_APPLICATION_SPEED_RPM,
    .hKpDivisor          = (uint16_t)PID_POSITION_KPDIV,
    .hKiDivisor          = (uint16_t)PID_POSITION_KIDIV,
    .hKdDivisor          = (uint16_t)PID_POSITION_KDDIV,
    .hKpDivisorPOW2      = (uint16_t)PID_POSITION_KPDIV_LOG,
    .hKiDivisorPOW2      = (uint16_t)PID_POSITION_KIDIV_LOG,
    .hKdDivisorPOW2      = (uint16_t)PID_POSITION_KDDIV_LOG,
};

Trapezoidal_Handle_t pTrapezoidalM1 =
{
    .S1=0,
    .S2=0,
    .S3=0,
    .Sk=0,
    .Sn=0,
    .V0=0,
    .Vt=0,
    .A=2,
    .D=2,
    .S=0,
    .Vav=20,

};

SMC pSMC_Position_StructM1 =
{
    .eSMC = 5,
    .cSMC = 0.2,
    .kSMC = 0.5,
    .Integrator = 0,
    .Output = 0,
    .SMC_OUTPUT_MAX = 400,
};

SMC pSMC_Position_StructM2 =
{
    .eSMC = 5,
    .cSMC = 0.2,
    .kSMC = 0.5,
    .Integrator = 0,
    .Output = 0,
    .SMC_OUTPUT_MAX = 400,
};

SMC pSMC_Speed_StructM1 =
{
    .eSMC = 5,
    .cSMC = 0.05,
    .kSMC = 100,
    .Integrator = 0,
    .Output = 0,
    .SMC_OUTPUT_MAX = 10000,
};

SMC pSMC_Speed_StructM2 =
{
    .eSMC = 5,
    .cSMC = 0.05,
    .kSMC = 100,
    .Integrator = 0,
    .Output = 0,
    .SMC_OUTPUT_MAX = 10000,
};

SMC pSMC_IQ_StructM1 =
{
    .eSMC = 5,
    .cSMC = 0.2,
    .kSMC = 0.5,
    .Integrator = 0,
    .Output = 0,
    .SMC_OUTPUT_MAX = 32767,
};

SMC pSMC_IQ_StructM2 =
{
    .eSMC = 5,
    .cSMC = 0.2,
    .kSMC = 0.5,
    .Integrator = 0,
    .Output = 0,
    .SMC_OUTPUT_MAX = 32767,
};


PosCtrl_Handle_t pPosCtrlM1 =
{
    .SamplingTime  = 1.0f/MEDIUM_FREQUENCY_TASK_RATE,
    .SysTickPeriod = 1.0f/SYS_TICK_FREQUENCY,
    .AlignmentCfg  = TC_ABSOLUTE_ALIGNMENT_NOT_SUPPORTED/*TC_ABSOLUTE_ALIGNMENT_SUPPORTED*/,
    .pTrapezoidal  = &pTrapezoidalM1,
    .pSMC_Position_Struct = &pSMC_Position_StructM1,
};

PID_Handle_t PID_PosParamsM2 =
{
    .hDefKpGain          = (int16_t)PID_POSITION_KP_GAIN2,
    .hDefKiGain          = (int16_t)PID_POSITION_KI_GAIN2,
    .hDefKdGain          = (int16_t)PID_POSITION_KD_GAIN2,
    .wUpperIntegralLimit = (int32_t)NOMINAL_CURRENT2 * (int32_t)PID_POSITION_KIDIV2,
    .wLowerIntegralLimit = (int32_t)(-NOMINAL_CURRENT2) * (int32_t)PID_POSITION_KIDIV2,
    .hUpperOutputLimit   = (int16_t)MAX_APPLICATION_SPEED_RPM2,
    .hLowerOutputLimit   = -(int16_t)MAX_APPLICATION_SPEED_RPM2,
    .hKpDivisor          = (uint16_t)PID_POSITION_KPDIV2,
    .hKiDivisor          = (uint16_t)PID_POSITION_KIDIV2,
    .hKdDivisor          = (uint16_t)PID_POSITION_KDDIV2,
    .hKpDivisorPOW2      = (uint16_t)PID_POSITION_KPDIV_LOG2,
    .hKiDivisorPOW2      = (uint16_t)PID_POSITION_KIDIV_LOG2,
    .hKdDivisorPOW2      = (uint16_t)PID_POSITION_KDDIV_LOG2,
};

Trapezoidal_Handle_t pTrapezoidalM2 =
{
    .S1=0,
    .S2=0,
    .S3=0,
    .Sk=0,
    .Sn=0,
    .V0=0,
    .Vt=0,
    .A=2,
    .D=2,
    .S=0,
    .Vav=20,

};

PosCtrl_Handle_t pPosCtrlM2 =
{
    .SamplingTime  = 1.0f/MEDIUM_FREQUENCY_TASK_RATE2,
    .SysTickPeriod = 1.0f/SYS_TICK_FREQUENCY,
    .AlignmentCfg  = TC_ABSOLUTE_ALIGNMENT_NOT_SUPPORTED/*TC_ABSOLUTE_ALIGNMENT_SUPPORTED*/,
    .pTrapezoidal  = &pTrapezoidalM2,
    .pSMC_Position_Struct = &pSMC_Position_StructM2,
};

/**
  * @brief  SpeednTorque Controller parameters Motor 1
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
    .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE,
    .MaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT),
    .MinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
    .MaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT),
    .MinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT),
    .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,
    .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,
    .ModeDefault =					DEFAULT_CONTROL_MODE,
    .MecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT),
    .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,
    .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,
    .pSMC_Struct = &pSMC_Speed_StructM1,
};
PWMC_R3_2_Handle_t PWM_Handle_M1=
{
    {
        .pFctGetPhaseCurrents              = &R3_2_GetPhaseCurrents,
        .pFctSwitchOffPwm                  = &R3_2_SwitchOffPWM,
        .pFctSwitchOnPwm                   = &R3_2_SwitchOnPWM,
        .pFctCurrReadingCalib              = &R3_2_CurrentReadingCalibration,
        .pFctTurnOnLowSides                = &R3_2_TurnOnLowSides,
        .pFctSetADCSampPointSectX          = &R3_2_SetADCSampPointSectX,
        .pFctIsOverCurrentOccurred         = &R3_2_IsOverCurrentOccurred,
        .pFctOCPSetReferenceVoltage        = MC_NULL,
        .pFctRLDetectionModeEnable         = &R3_2_RLDetectionModeEnable,
        .pFctRLDetectionModeDisable        = &R3_2_RLDetectionModeDisable,
        .pFctRLDetectionModeSetDuty        = &R3_2_RLDetectionModeSetDuty,
        .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,
        .Sector = 0,
		  	.SectorUser_defined = 0,
			  .PreSectorUser_defined = 6,
			  .PrePreSectorUser_defined = 6,
			  .DistinguishingMotorNumber = M1,
        .CntPhA = 0,
        .CntPhB = 0,
        .CntPhC = 0,
        .SWerror = 0,
        .TurnOnLowSidesAction = false,
        .OffCalibrWaitTimeCounter = 0,
        .Motor = M1,
        .RLDetectionMode = false,
        .Ia = 0,
        .Ib = 0,
        .Ic = 0,
        .DTTest = 0,
        .DTCompCnt = DTCOMPCNT,
        .PWMperiod          = PWM_PERIOD_CYCLES,
        .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),
        .Ton                 = TON,
        .Toff                = TOFF
    },
    .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u,
    .PhaseAOffset = 0,
    .PhaseBOffset = 0,
    .PhaseCOffset = 0,

    .pParams_str = &R3_2_ParamsM1
};

/**
  * @brief  PI / PID Speed loop parameters Motor 2
  */
PID_Handle_t PIDSpeedHandle_M2 =
{
    .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT2,
    .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT2,
    .wUpperIntegralLimit = (int32_t)IQMAX2 * (int32_t)SP_KIDIV2,
    .wLowerIntegralLimit = -(int32_t)IQMAX2 * (int32_t)SP_KIDIV2,
    .hUpperOutputLimit       = (int16_t)IQMAX2,
    .hLowerOutputLimit       = -(int16_t)IQMAX2,
    .hKpDivisor          = (uint16_t)SP_KPDIV2,
    .hKiDivisor          = (uint16_t)SP_KIDIV2,
    .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG2,
    .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG2,
    .hDefKdGain           = 0x0000U,
    .hKdDivisor           = 0x0000U,
    .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Iq loop parameters Motor 2
  */
PID_Handle_t PIDIqHandle_M2 =
{
    .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT2,
    .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT2,
    .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,
    .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,
    .hUpperOutputLimit       = INT16_MAX,
    .hLowerOutputLimit       = -INT16_MAX,
    .hKpDivisor          = (uint16_t)TF_KPDIV2,
    .hKiDivisor          = (uint16_t)TF_KIDIV2,
    .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,
    .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,
    .hDefKdGain           = 0x0000U,
    .hKdDivisor           = 0x0000U,
    .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  PI / PID Id loop parameters Motor 2
  */
PID_Handle_t PIDIdHandle_M2 =
{
    .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT2,
    .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT2,
    .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV2,
    .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV2,
    .hUpperOutputLimit       = INT16_MAX,
    .hLowerOutputLimit       = -INT16_MAX,
    .hKpDivisor          = (uint16_t)TF_KPDIV2,
    .hKiDivisor          = (uint16_t)TF_KIDIV2,
    .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG2,
    .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG2,
    .hDefKdGain           = 0x0000U,
    .hKdDivisor           = 0x0000U,
    .hKdDivisorPOW2       = 0x0000U,
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM2 =
{
    ._Super = {
        .bElToMecRatio                     =	POLE_PAIR_NUM2,
        .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT2),
        .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
        .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS2,
        .hMaxReliableMecAccelUnitP         =	65535,
        .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED2,
        .DPPConvFactor                     =  DPP_CONV_FACTOR2,
    },
    .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE2,
    .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE2 * TRANSITION_DURATION2/ 1000.0),
};

/**
  * @brief  SpeednTorque Controller parameters Motor 2
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM2 =
{
    .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE2,
    .MaxAppPositiveMecSpeedUnit =	(uint16_t)(MAX_APPLICATION_SPEED_UNIT2),
    .MinAppPositiveMecSpeedUnit =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
    .MaxAppNegativeMecSpeedUnit =	(int16_t)(-MIN_APPLICATION_SPEED_UNIT2),
    .MinAppNegativeMecSpeedUnit =	(int16_t)(-MAX_APPLICATION_SPEED_UNIT2),
    .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT2,
    .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT2,
    .ModeDefault =					DEFAULT_CONTROL_MODE2,
    .MecSpeedRefUnitDefault =		(int16_t)(DEFAULT_TARGET_SPEED_UNIT2),
    .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT2,
    .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT2,
    .pSMC_Struct = &pSMC_Speed_StructM2,
};
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{

    ._Super = {
        .bElToMecRatio                     =	POLE_PAIR_NUM,
        .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
        .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
        .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
        .hMaxReliableMecAccelUnitP         =	65535,
        .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
        .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
    .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE,
    .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION/ 1000.0),

};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Encoder
  */
ENCODER_Handle_t ENCODER_M1 =
{
    ._Super = {
        .bElToMecRatio                     =	POLE_PAIR_NUM,
        .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
        .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
        .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
        .hMaxReliableMecAccelUnitP         =	65535,
        .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
        .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
    .PulseNumber           =	M1_ENCODER_PPR*4,
    .RevertSignal           =	(FunctionalState)ENC_INVERT_SPEED,
    .SpeedSamplingFreqHz   =	MEDIUM_FREQUENCY_TASK_RATE,
    .SpeedBufferSize       =	ENC_AVERAGING_FIFO_DEPTH,
    .TIMx                  =	TIM4,
    .ICx_Filter            =  M1_ENC_IC_FILTER,

};

/**
  * @brief  Encoder Alignment Controller parameters Motor 1
  */
EncAlign_Handle_t EncAlignCtrlM1 =
{
    .hEACFrequencyHz =	MEDIUM_FREQUENCY_TASK_RATE,
    .hFinalTorque    =	FINAL_I_ALIGNMENT,
    .hElAngle        =	ALIGNMENT_ANGLE_S16,
    .hDurationms     =	ALIGNMENT_DURATION,
    .bElToMecRatio   =	POLE_PAIR_NUM,
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 2 - Encoder
  */
ENCODER_Handle_t ENCODER_M2 =
{
    ._Super = {
        .bElToMecRatio                     =	 POLE_PAIR_NUM2,
        .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT2),
        .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT2),
        .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS2,
        .hMaxReliableMecAccelUnitP         =	65535,
        .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED2,
        .DPPConvFactor                     =  DPP_CONV_FACTOR,
    },
    .PulseNumber           =	 M2_ENCODER_PPR*4,
    .RevertSignal           =	(FunctionalState)ENC_INVERT_SPEED2,
    .SpeedSamplingFreqHz   =	 MEDIUM_FREQUENCY_TASK_RATE2,
    .SpeedBufferSize       =	ENC_AVERAGING_FIFO_DEPTH2,
    .TIMx                  =	TIM2,
    .ICx_Filter            =  M2_ENC_IC_FILTER,
};

/**
  * @brief  Encoder Alignment Controller parameters Motor 2
  */
EncAlign_Handle_t EncAlignCtrlM2 =
{
    .hEACFrequencyHz =	MEDIUM_FREQUENCY_TASK_RATE2,
    .hFinalTorque    =	FINAL_I_ALIGNMENT2,
    .hElAngle        =	ALIGNMENT_ANGLE_S162,
    .hDurationms     =	ALIGNMENT_DURATION2,
    .bElToMecRatio   =	POLE_PAIR_NUM2,
};

PWMC_R3_2_Handle_t PWM_Handle_M2=
{
    {
        .pFctGetPhaseCurrents              = &R3_2_GetPhaseCurrents,
        .pFctSwitchOffPwm                  = &R3_2_SwitchOffPWM,
        .pFctSwitchOnPwm                   = &R3_2_SwitchOnPWM,
        .pFctCurrReadingCalib              = &R3_2_CurrentReadingCalibration,
        .pFctTurnOnLowSides                = &R3_2_TurnOnLowSides,
        .pFctSetADCSampPointSectX          = &R3_2_SetADCSampPointSectX,
        .pFctIsOverCurrentOccurred         = &R3_2_IsOverCurrentOccurred,
        .pFctOCPSetReferenceVoltage        = MC_NULL,
        .pFctRLDetectionModeEnable         = &R3_2_RLDetectionModeEnable,
        .pFctRLDetectionModeDisable        = &R3_2_RLDetectionModeDisable,
        .pFctRLDetectionModeSetDuty        = &R3_2_RLDetectionModeSetDuty,
        .hT_Sqrt3 = (PWM_PERIOD_CYCLES2*SQRT3FACTOR)/16384u,
        .Sector = 0,
			  .SectorUser_defined = 0,
				.PreSectorUser_defined = 6,
			  .PrePreSectorUser_defined = 6,
			  .DistinguishingMotorNumber = M2,
        .CntPhA = 0,
        .CntPhB = 0,
        .CntPhC = 0,
        .SWerror = 0,
        .TurnOnLowSidesAction = false,
        .OffCalibrWaitTimeCounter = 0,
        .Motor = M2,
        .RLDetectionMode = false,
        .Ia = 0,
        .Ib = 0,
        .Ic = 0,
        .DTTest = 0,
        .DTCompCnt = DTCOMPCNT2,
        .PWMperiod          = PWM_PERIOD_CYCLES2,
        .OffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000),
        .Ton                 = TON2,
        .Toff                = TOFF2
    },
    .PhaseAOffset = 0,
    .PhaseBOffset = 0,
    .PhaseCOffset = 0,
    .Half_PWMPeriod = PWM_PERIOD_CYCLES2/2u,
    .pParams_str = &R3_2_ParamsM2,

};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - HALL
  */

HALL_Handle_t HALL_M1 =
{
//  ._Super = {
//    .bElToMecRatio                     =	POLE_PAIR_NUM,
//    .hMaxReliableMecSpeedUnit          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT),
//    .hMinReliableMecSpeedUnit          =	(uint16_t)(MIN_APPLICATION_SPEED_UNIT),
//    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,
//    .hMaxReliableMecAccelUnitP         =	65535,
//    .hMeasurementFrequency             =	TF_REGULATION_RATE_SCALED,
//    .DPPConvFactor                     =  DPP_CONV_FACTOR,
//  },
    .SensorPlacement     = HALL_SENSORS_PLACEMENT,
//  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
    .PhaseShift          = (int16_t)(300 * 65536/360),
    .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
    .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH,
    .TIMClockFreq       = HALL_TIM_CLK,
    .TIMx                = TIM4,

    .ICx_Filter          = M1_HALL_IC_FILTER,

    .PWMFreqScaling      = PWM_FREQ_SCALING,
    .HallMtpa            = HALL_MTPA,

// .H1Port             =  M1_HALL_H1_GPIO_Port,
// .H1Pin              =  M1_HALL_H1_Pin,
// .H2Port             =  M1_HALL_H2_GPIO_Port,
// .H2Pin              =  M1_HALL_H2_Pin,
// .H3Port             =  M1_HALL_H3_GPIO_Port,
// .H3Pin              =  M1_HALL_H3_Pin,
};

/**
  * Virtual temperature sensor parameters Motor 1
  */
NTC_Handle_t TempSensorParamsM1 =
{
    .bSensorType = REAL_SENSOR,
    .TempRegConv =
    {
        .regADC = ADC1,
        .channel = MC_ADC_CHANNEL_14,
        .samplingTime = M1_TEMP_SAMPLING_TIME,
    },
    .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
    .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
    .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
    .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
    .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
    .hT0                     = T0_C,
};

/**
  * temperature sensor parameters Motor 2
  */
NTC_Handle_t TempSensorParamsM2 =
{
    .bSensorType = REAL_SENSOR,
    .TempRegConv =
    {
        .regADC = ADC1,
        .channel = MC_ADC_CHANNEL_13,
        .samplingTime = M2_TEMP_SAMPLING_TIME,
    },
    .hLowPassFilterBW        = M2_TEMP_SW_FILTER_BW_FACTOR,
    .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2 - OV_TEMPERATURE_HYSTERESIS_d2),
    .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d2),
    .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT2),
    .wV0                     = (uint16_t)(V0_V2 *65536/ ADC_REFERENCE_VOLTAGE),
    .hT0                     = T0_C2,
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1
  */
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
    ._Super                =
    {
        .SensorType          = REAL_SENSOR,
        .ConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR),
    },

    .VbusRegConv =
    {
        .regADC = ADC1,
        .channel = MC_ADC_CHANNEL_3,
        .samplingTime = M1_VBUS_SAMPLING_TIME,
    },
    .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,
    .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,
    .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,
    .aBuffer = RealBusVoltageSensorFilterBufferM1,
};

/**
  * Virtual bus voltage sensor parameters Motor 2
  */
VirtualBusVoltageSensor_Handle_t VirtualBusVoltageSensorParamsM2 =
{
    ._Super =
    {
        .SensorType       = VIRTUAL_SENSOR,
        .ConversionFactor = 500,
    },

    .ExpectedVbus_d = 1+(NOMINAL_BUS_VOLTAGE_V2 * 65536) / 500,
};

UI_Handle_t UI_Params =
{
    .bDriveNum = 0,
};

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
    .FrequencyHz = TF_REGULATION_RATE
};

/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component   
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
    .MaxModule          = MAX_MODULE,
    .MaxVd          	  = (uint16_t)(MAX_MODULE * 950 / 1000),
    .Circle_limit_table = MMITABLE,
    .Start_index        = START_INDEX,
};
/** RAMP for Motor2.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM2 =
{
    .FrequencyHz = TF_REGULATION_RATE2
};

/**
  * @brief  CircleLimitation Component parameters Motor 2 - Base Component
  */
CircleLimitation_Handle_t CircleLimitationM2 =
{
    .MaxModule          = MAX_MODULE2,
    .MaxVd          	  = (uint16_t)(MAX_MODULE2 * 950 / 1000),
    .Circle_limit_table = MMITABLE2,
    .Start_index        = START_INDEX2,
};

DOUT_handle_t R_BrakeParamsM1 =
{
    .OutputState       = INACTIVE,
    .hDOutputPort      = M1_DISSIPATIVE_BRK_GPIO_Port,
    .hDOutputPin       = M1_DISSIPATIVE_BRK_Pin,
    .bDOutputPolarity  = DOUT_ACTIVE_HIGH
};

UFCP_Handle_t pUSART =
{
    ._Super.RxTimeout = 0,
    .USARTx = USART1,

};

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

