#include "mc_config.h"
#include "version.h"
#include "bsp_app_function.h"
    
long testAdd=1;                                                 /*≤‚ ‘*/
MCU_Version ProgramVersion;                                     /*∞Ê±æ–≈œ¢*/


const PARAMETER_TABLE  PARAMETER[ParaNum] =	 \
{ \
	{0, 		0xFFFF, 			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	0,							POS_MIN,					POS_MAX,								0,									&testAdd},	\
	{1,			_PosRef,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							S16_MIN,					S16_MAX,					PID_POSITION_KP_GAIN,						(long*)&PID_PosParamsM1.hKpGain },	\
	{2,			_MainPos,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							S16_MIN,					S16_MAX,					PID_POSITION_KI_GAIN,						(long*)&PID_PosParamsM1.hKiGain  },	\
	{3,			_deltaPos,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							S16_MIN,					S16_MAX,					PID_POSITION_KD_GAIN,						(long*)&PID_PosParamsM1.hKdGain },	\
	{4,			_VelRef,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					1000,					  (long*)&pTrapezoidalM1.A },	\
	{5,			_Vel,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					1000,						(long*)&pTrapezoidalM1.D },	\
	{6,			_VelErr,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					100,						(long*)&pTrapezoidalM1.Vav},	\
	{7,			_IqRef,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				  1300,					(long*)&PIDSpeedHandle_M1.hKpGain   },	\
	{8,			_Iq,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				  250,					(long*)&PIDSpeedHandle_M1.hKiGain   },	\
	{9,			_IqErr,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				  150,					(long*)&PIDIqHandle_M1.hKpGain },	\

	{10,		_IdRef,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				  7,				  (long*)&PIDIqHandle_M1.hKiGain },	\
	{11,		_Id,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,				  PID_POSITION_KP_GAIN,				  (long*)&PID_PosParamsM2.hKpGain },	\
	{12,		_IdErr,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,				  PID_POSITION_KI_GAIN,					(long*)&PID_PosParamsM2.hKiGain },	\
	{13,		_DebugTestD,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		  0,	1,	            POS_MIN,					POS_MAX,					PID_POSITION_KD_GAIN,						(long*)&PID_PosParamsM2.hKdGain }, \
	{14,		_PosKp,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,			  	POS_MAX,			    1000,				(long*)&pTrapezoidalM2.A},	\
	{15,		_PosKi,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,			  	POS_MAX,			    1000,				(long*)&pTrapezoidalM2.D},	\
	{16,		_VelKp,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,			 	 	POS_MAX,			    100,				(long*)&pTrapezoidalM2.Vav},	\
	{17,		_VelKi,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,				  1300,					(long*)&PIDSpeedHandle_M2.hKpGain  },	\
	{18,		_TorKp,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,			  	POS_MAX,			    250,			(long*)&PIDSpeedHandle_M2.hKiGain  },	\
	{19,		_TorKi,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,				  150,		  	(long*)&PIDIqHandle_M2.hKpGain  },	\
	
	{20,		_MotionSts,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,		    	POS_MAX,		      7,		         (long*)&PIDIqHandle_M2.hKiGain  },	\
	{21,		_Conflt,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,				  0,				(long*)&pCtrlPar[M1].SetPulseMotor},	\
	{22,		_AlarmClrReq,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,						(long*)&ENCODER_M1._Super.wPulseNumber },	\
	{23,		_MotorEn,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,					  (long*)&pCtrlPar[M1].M1M2_VelSet },	\
	{24,		_StatRegs,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					0,						(long*)&pCtrlPar[M2].SetPulseMotor},	\
	{25,		_RecLen,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,		  0,	1,							POS_MIN,		      POS_MAX,		        0,			(long*)&ENCODER_M2._Super.wPulseNumber },	\
	{26,		_RecGap,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,		0,	1,							POS_MIN,				  POS_MAX,				  0,				(long*)NULL   },	\
	{27,		_Reserved,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,		0,	1,							POS_MIN,			    POS_MAX,			   0,				(long*)&pCtrlPar[M1].SetVelMotor },	\
	{28,		_RecTrigval,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,		0,	1,							POS_MIN,			    POS_MAX,			     0,			  (long*)&pCtrlPar[M1].Vel_PLL_Motor },	\
	{29,		_RecTrigPos,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,		      POS_MAX,		      200,	  (long*)&pCtrlPar[M1].hDurationms  },	\

	{30,		_TrigMode,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,		0,	1,							POS_MIN,		      POS_MAX,		       0,		(long*)&pCtrlPar[M2].SetVelMotor },	\
	{31,		_RecData,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,	  AXIS,		0,	1,		          POS_MIN,			    POS_MAX,			     0,				(long*)&pCtrlPar[M2].Vel_PLL_Motor},	\
	{32,		_RatedCurr,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				 200,				  (long*)&pCtrlPar[M2].hDurationms },	\
	{33,		_PeakCurr,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			   	POS_MAX,				0,				  (long*)NULL },	\
	{34,		_PeakCurrTim,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,			(long*)NULL },	\
	{35,		_MaxPhaseCur,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,					(long*)NULL},	\
	{36,		_MaxVel,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,				(long*)NULL},	\
	{37,		_MaxPosErr,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,		      POS_MAX,		     0,			(long*)NULL },	\
	{38,		_MaxVelErr,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,		      POS_MAX,		  3000,		  (long*)&SpeednTorqCtrlM2.MaxAppPositiveMecSpeedUnit },	\
	{39,		_ProtectOn,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,			-3000,					  (long*)&SpeednTorqCtrlM2.MinAppNegativeMecSpeedUnit},	\
	
	{40,		_InjectPoint,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,	        POS_MAX,	   25000,		(long*)&SpeednTorqCtrlM2.MaxPositiveTorque },	\
	{41,		_InjectType,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,		      POS_MAX,		-25000,		(long*)&SpeednTorqCtrlM2.MinNegativeTorque },	\
	{42,		_InjectCurAmp,RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,          POS_MAX,     0,    	(long*)NULL},	\
	{43,		_InjectVelAmp,RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,	        POS_MAX,	  15,	    (long*)&MotorParameters[M1].PolePairNum },	\
	{44,		_InjectPosAmp,RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,	        POS_MAX,	  15,	     (long*)&MotorParameters[M2].PolePairNum },	\
	{45,		_InjectFreq,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,		      POS_MAX,	         	0,	  (long*)NULL },	\
	{46,		_CalcFiltEn,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,						POS_MIN,					POS_MAX,						0,					(long*)NULL },	\
	{47,		_CurrFiltDef,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,		NOFLASH,		AXIS,			0,	1,	          POS_MIN,	        POS_MAX,	        0,	            (long*)&ENCODER_M1._Super.hAvrMecSpeedUnit },	\
	{48,		_CurrFiltOn,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,		NOFLASH,		AXIS,			0,	1,	          POS_MIN,		      POS_MAX,		      0,	            (long*)&ENCODER_M2._Super.hAvrMecSpeedUnit },	\
	{49,		_VelFiltDef,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,		NOFLASH,		AXIS,			0,	1,		        POS_MIN,		      POS_MAX,		      1,		        (long*)&VelocityPLLSwitch },	\
	
	{50,		_VelFiltOn,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,		          POS_MIN,		      POS_MAX,		 0,			(long*)&pCtrlPar[M1].SetTorqueMotor},	\
	{51,		_Ia,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,					(long*)NULL},	\
	{52,		_Ib,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,					(long*)&HallState1 },	\
	{53,		_Ic,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,					(long*)&pCtrlPar[M2].SetTorqueMotor},	\
	{54,		_Va,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,						(long*)NULL },	\
	{55,		_Vb,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,					  (long*)&HallState2 },	\
	{56,		_Vc,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,						(long*)NULL},	\
	{57,		_Vd,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,						(long*)NULL},	\
	{58,		_Vq,					RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						64,						(long*)&pSpeed_Mesa[M2].fPLL_Kp },	\
	{59,		_MorCurrFilt,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					0,						(long*)&pSpeed_Mesa[M2].fPLL_Ki },	\
	
	{60,		_RatedTor,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			64,			(long*)&pSpeed_Mesa[M1].fPLL_Kp },	\
	{61,		_MaxTor,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,			(long*)&pSpeed_Mesa[M1].fPLL_Ki },	\
	{62,		_TorConst,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					0,						(long*)&FOCVars[M1].Iab.a},	\
	{63,		_RatedVel,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,				(long*)&FOCVars[M2].Iab.a},	\
	{64,		_EncType,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,				(long*)&FOCVars[M1].Iab.b},	\
	{65,		_EncRes,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,				(long*)&FOCVars[M2].Iab.b},	\
	{66,		_EncDir,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,						(long*)&FOCVars[M1].Ialphabeta.alpha},	\
	{67,		_EncZeroAng,	RW,		OKINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					0,						(long*)&FOCVars[M2].Ialphabeta.alpha},	\
	{68,		_EncFilt,			RW,		OKINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,				(long*)&FOCVars[M1].Ialphabeta.beta},	\
	{69,		_MotorJm,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,				(long*)&FOCVars[M2].Ialphabeta.beta},	\

	{70,		_Poles,				RW,		OKINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,				(long*)&FOCVars[M1].Iqdref.q},	\
	{71,		_MotorRs,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,				(long*)&FOCVars[M2].Iqdref.q},	\
	{72,		_MotorLs,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,				(long*)&FOCVars[M1].Iqdref.d},	\
	{73,		_SoftLimEn,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,						(long*)&FOCVars[M2].Iqdref.d},	\
	{74,		_SoftRevLim,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,			  (long*)&FOCVars[M1].Iqd.q},	\
	{75,		_SoftFwdLim,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,				(long*)&FOCVars[M2].Iqd.q },	\
	{76,		_ErrLog,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,		        	POS_MIN,					POS_MAX,			0,				  	(long*)&FOCVars[M1].Iqd.d },	\
	{77,		_MaxAcc,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,			(long*)&FOCVars[M2].Iqd.d },	\
	{78,		_MaxDec,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,			(long*)&FOCVars[M1].Vqd.q},	\
	{79,		_DirectMode,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,			0,						(long*)&FOCVars[M2].Vqd.q},	\
	
	{80,		_Jerk,				RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					0,					(long*)&FOCVars[M1].Vqd.d },	\
	{81,		_MotionMode,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,		      POS_MAX,		0,		(long*)&FOCVars[M2].Vqd.d },	\
	{82,		_AbsTrgt,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					0,					  (long*)&FOCVars[M1].Valphabeta.alpha},		\
	{83,		_RelTrgt,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					0,						(long*)&FOCVars[M2].Valphabeta.alpha},		\
	{84,		_Speed,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,					(long*)&FOCVars[M1].Valphabeta.beta},		\
	{85,		_Accel,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			  	POS_MAX,				0,					(long*)&FOCVars[M2].Valphabeta.beta},		\
	{86,		_Decel,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,				  (long*)&STM[M1].hFaultOccurred },		\
	{87,		_EmrgDec,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,					(long*)&STM[M2].hFaultOccurred },		\
	{88,		_MotionReson, RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,	        POS_MAX,	    0,	      (long*)NULL},	\
	{89,		_RptWait,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0,				(long*)NULL},	\
	
  {90,		_CtrlMode,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,		      POS_MAX,		  0,			(long*)NULL},	\
	{91,		_ReverseDir,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						0,						(long*)NULL},	\
	{92,		_ProtectMask,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,			    POS_MAX,			0	,		(long*)&pCtrlPar[M1].MotorTemperature},	\
	{93,		_Vbus,        RW,   OKINMOTN,  	OKMOTON,  NOARRAY,  NOFLASH,    AXIS,   0,  1,              POS_MIN,       	  POS_MAX,         0,          (long*)&pCtrlPar[M2].MotorTemperature},        \
	{94,		_Reserved,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,		0,	0,							POS_MIN,					POS_MAX,					0,						(long*)&VBS_AvBusVoltage_V},	\
	{95,		_MaxPwm,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				0,				(long*)NULL},	\
	{96,		_EmgerStpReq,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						2,						(long*)&MotorParameters[M1].ConstantTorque},	\
	{97,		_DisableStpOp,RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,				  POS_MAX,				3,				(long*)&MotorParameters[M1].ConstantInertia},	\
	{98,		_FaultStpOp,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,		      POS_MAX,		6500,		(long*)&MotorParameters[M1].RatedCurrent},	\
	{99,		_DeviceAdd,		RW,		NOINMOTN,		NOMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,		      POS_MAX,		5000,			(long*)&MotorParameters[M1].RatedTorque}, \

	{100,		_FWVerion,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,					POS_MAX,					1500,						(long*)&MotorParameters[M1].RatedSpeed}, \
	{101,		_HWVerion,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,					POS_MAX,					16384,						(long*)&MotorParameters[M1].PulseNumber}, \
	{102,		_VersionDate,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,					POS_MAX,					20000,						(long*)&MotorParameters[M1].PeakCurrent}, \
	{103,		_InVelLimRev,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					20000,							(long*)&MotorParameters[M1].PeakTicks}, \
	{104,		_InVelLimFwd,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							SW_MIN,						POS_MAX,					20000,							(long*)&MotorParameters[M1].MAXPhaseCurrent}, \
	{105,		_StuckCurr,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							SW_MIN,						POS_MAX,					2000,						(long*)&MotorParameters[M1].MAXSpeed}, \
	{106,		_StuckTime,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							SW_MIN,						POS_MAX,					50,						(long*)&MotorParameters[M1].LockedSpeed}, \
	{107,		_StuckVel,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							SW_MIN,						POS_MAX,					10000,						(long*)&MotorParameters[M1].ShortCircuit}, \
	{108,		_ComtMode,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,	AXIS,			0,	1,	            POS_MIN,					POS_MAX,					4000,						(long*)&MotorParameters[M1].LockeTicks}, \
	{109,		_ComtInc,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					0,						(long*)NULL}, \
	
	{110,		_ComtStep,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					2,						(long*)&MotorParameters[M2].ConstantTorque}, \
	{111,		_StartComt,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,					POS_MAX,						3,						(long*)&MotorParameters[M2].ConstantInertia}, \
	{112,		_InTargetTol,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,	   	    POS_MAX,			6500,			(long*)&MotorParameters[M2].RatedCurrent}, \
	{113,		_InTargetTim,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,		      POS_MAX,		    5000,			(long*)&MotorParameters[M2].RatedTorque}, \
	{114,		_TriggerID,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,					POS_MAX,					1500,						(long*)&MotorParameters[M2].RatedSpeed}, \
	{115,		_PdPos,				RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					16384,						(long*)&MotorParameters[M2].PulseNumber}, \
	{116,		_TrgtVel,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					20000,						(long*)&MotorParameters[M2].PeakCurrent}, \
	{117,		_TrgtTor,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					20000,						(long*)&MotorParameters[M2].PeakTicks}, \
	{118,		_DInPort,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,					POS_MAX,					20000,						(long*)&MotorParameters[M2].MAXPhaseCurrent}, \
	{119,		_Motor_mode,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,						2000,						(long*)&MotorParameters[M2].MAXSpeed}, \
	
	{120,		_TriggerSubID,RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,					POS_MAX,					50,						(long*)&MotorParameters[M2].LockedSpeed}, \
	{121,		_DInLog,		  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,				  POS_MAX,				10000,				(long*)&MotorParameters[M2].ShortCircuit}, \
	{122,		_DInFilt,			RW,		NOINMOTN,		NOMOTON,	NOARRAY,	YSFLASH,		AXIS,		0,	1,							POS_MIN,			    POS_MAX,			4000,				(long*)&MotorParameters[M2].LockeTicks}, \
	{123,		_DInputMask,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,		NOFLASH,		AXIS,		0,	1,	            POS_MIN,			    POS_MAX,			0,				(long*)NULL}, \
	{124,		_DInMode,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,		NOFLASH,		AXIS,		0,	1,		          POS_MIN,			    POS_MAX,			0,				(long*)NULL}, \
	{125,		_DOutPort,	  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,		0,	1,							POS_MIN,			    POS_MAX,			0,			(long*)NULL}, \
	{126,		_OutLog,		  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,		0,	1,							POS_MIN,		    	POS_MAX,			    0,				(long*)NULL}, \
	{127,		_DOutMask,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,		YSFLASH,		AXIS,		0,	1,              POS_MIN,			    POS_MAX,			120,			(long*)&MotorParameters[M1].MaxTemperature}, \
	{128,		_DOutMode,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,		YSFLASH,		AXIS,		0,	1,	            POS_MIN,			    POS_MAX,			120,				(long*)&MotorParameters[M2].MaxTemperature}, \
	{129,		_HomingExec,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,						POS_MAX,						0,						(long*)&SpeednTorqCtrlM1.SpeedRefUnitExt }, \
	
	{130,		_HomingOn,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,							POS_MIN,						POS_MAX,						0,						(long*)&SpeednTorqCtrlM2.SpeedRefUnitExt }, \
	{131,		_HomingStat,	RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,							POS_MIN,					POS_MAX,					60000,						&MotorParameters[M1].OverTemperatureTicks}, \
	{132,		_HomingDef,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,		YSFLASH,		AXIS,			0,	1,POS_MIN,					POS_MAX,					60000,						&MotorParameters[M2].OverTemperatureTicks}, \
};



#if MAXON_AGREEMENT
PARAMETER_TABLE Maxon_Parameter[MaxonPara_num] = \
{
  {0, 		0xFFFF, 			    RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		NOAXIS,		0,	0,				POS_MIN,				  POS_MAX,					POS_DFLT,							&testAdd},	\
  {1, 		_MotorCurrent, 	  RO,		NOINMOTN,		NOINMOTN,	ARRAY,	  NOFLASH,		AXIS,			0,	1,				IQREF_MIN,				IQREF_MAX,				IQREF_DFLT,						&pMotorParSet.iq[A_AXIS]},	\
  {2, 		_MotorSpeed, 		  RO,		NOINMOTN,		NOINMOTN,	ARRAY,	  NOFLASH,		AXIS,			0,	1,				VEL_MIN,					VEL_MAX,					VEL_DFLT,							&NULL},	\
  {3, 		_DigtalIn, 			  RO,		NOINMOTN,		NOINMOTN,	ARRAY,	  NOFLASH,		AXIS,			0,	1,				POS_MIN,					POS_MAX,					POS_DFLT,							&pDInPar.dInPort},	\
  {4, 		_ErrCode, 			  RO,		NOINMOTN,		NOINMOTN,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				CONFLT_MIN,				CONFLT_MAX,				CONFLT_DFLT,					&NULL},	\
  {5, 		_ControlWord, 	  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				POS_MIN,				  POS_MAX,					POS_DFLT,							&testAdd},	\
  {6, 		_StatusWord, 	    RO,		NOINMOTN,		NOINMOTN,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				POS_MIN,					POS_MAX,					POS_DFLT,							&NULL},	\
  {7, 		_OperationMode,   RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				MOTIONMODE_MIN,		MOTIONMODE_MAX,		MOTIONMODE_DFLT,			&pMotionPar.motionMode[A_AXIS]},	\
  {8, 		_ActualPos, 		  RO,		NOINMOTN,		NOINMOTN,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				POS_MIN,				  POS_MAX,					POS_DFLT,							&NULL},	\
  {9, 		_TargetVel, 		  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SPEED_MIN,				SPEED_MAX,				SPEED_DFLT,						&pMotionPar.speed[A_AXIS]},\
  {10, 		_MotorAccel, 		  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				ACCEL_MIN,				ACCEL_MAX,				ACCEL_DFLT,						&pMotionPar.accel[A_AXIS]},\
  {11,		_MotorDecel,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,				ACCEL_MIN,			  ACCEL_MAX,				ACCEL_DFLT,					  &pMotionPar.decel[A_AXIS]},\
  {12,		_EmcyStop_Decel,  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	YSFLASH,		AXIS,			0,	1,				ACCEL_MIN,			  ACCEL_MAX,				ACCEL_DFLT,					  &pMotionPar.emrgDec[A_AXIS]},\
  {13,		_ClrErrEn,	      RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,					  SW_MAX,						SW_DFLT,						  &NULL},\
  {14,		_QuickStop_Req,	  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &NULL},\
  {15,		_MotorEnable,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &NULL},\
  {16,		_MotorDisable,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &NULL},\
  {17,		_ControlMode,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &NULL},	\
  {18,		_HomingEnable,		RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &pPosCtrlPar.homingCtrlPar.homingExec[A_AXIS]}, \
  {19,		_HomingState,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &pPosCtrlPar.homingCtrlPar.HomStats[A_AXIS]}, \
  {20,		_HomingMode,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &pPosCtrlPar.homingCtrlPar.homingDef[A_AXIS][19]},\
  {21,		_StartSwitch,			RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &pcommuntationPar.StartComt[A_AXIS]},\
  {22,		_SaveFlash,			  RW,		OKINMOTN,		OKMOTON,	NOARRAY,	NOFLASH,		AXIS,			0,	1,				SW_MIN,						SW_MAX,						SW_DFLT,						  &glSave[A_AXIS]}
};
#endif

