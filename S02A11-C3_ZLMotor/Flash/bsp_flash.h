#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

#include "public_h.h"

#define STM32F4x_Flash_Base			((uint32_t)0x08000000)
//定义主存储器中每个扇区起始地址
//主存储区块1（1MB芯片扇区定义）
#define ADDR_FLASH_SECTOR_0			((uint32_t)0x08000000)
#define ADDR_FLASH_SECTOR_1			((uint32_t)0x08004000)
#define ADDR_FLASH_SECTOR_2			((uint32_t)0x08008000)
#define ADDR_FLASH_SECTOR_3			((uint32_t)0x0800C000)
#define ADDR_FLASH_SECTOR_4			((uint32_t)0x08010000)
#define ADDR_FLASH_SECTOR_5			((uint32_t)0x08020000)
#define ADDR_FLASH_SECTOR_6			((uint32_t)0x08040000)
#define ADDR_FLASH_SECTOR_7			((uint32_t)0x08060000)
#define ADDR_FLASH_SECTOR_8			((uint32_t)0x08080000)
#define ADDR_FLASH_SECTOR_9			((uint32_t)0x080A0000)
#define ADDR_FLASH_SECTOR_10		((uint32_t)0x080C0000)
#define ADDR_FLASH_SECTOR_11		((uint32_t)0x080E0000)
//主存储区块2（2MB芯片扇区定义）
#define ADDR_FLASH_SECTOR_12		((uint32_t)0x08100000)
#define ADDR_FLASH_SECTOR_13		((uint32_t)0x08104000)
#define ADDR_FLASH_SECTOR_14		((uint32_t)0x08108000)
#define ADDR_FLASH_SECTOR_15		((uint32_t)0x0810C000)
#define ADDR_FLASH_SECTOR_16		((uint32_t)0x08110000)
#define ADDR_FLASH_SECTOR_17		((uint32_t)0x08120000)
#define ADDR_FLASH_SECTOR_18		((uint32_t)0x08140000)
#define ADDR_FLASH_SECTOR_19		((uint32_t)0x08160000)
#define ADDR_FLASH_SECTOR_20		((uint32_t)0x08180000)
#define ADDR_FLASH_SECTOR_21		((uint32_t)0x081A0000)
#define ADDR_FLASH_SECTOR_22		((uint32_t)0x081C0000)
#define ADDR_FLASH_SECTOR_23		((uint32_t)0x081E0000)

#define FLASH_WAITETIME    1000//等待上次操作完成时间（单位：）
#define FLASH_ERROR   1
#define FLASH_SUCCESS 0
#define Atlas_Flash_Addr   0x08040004  //sector six
#define LenData_Length  1

#define MAX_MODULE      32111 
#define START_INDEX     61        //98%

//Add parameters default value
#define	POS_MIN							-2147483648
#define	POS_MAX							2147483647
#define	POS_DFLT						0
//
#define	VEL_MIN							-2147483648
#define	VEL_MAX							2147483647
#define	VEL_DFLT						0
//
#define	DEFAULT_MIN					-2147483648
#define	DEFAULT_MAX					2147483647
#define	DEFAULT_DFLT				0
//
#define	POLEPRS_MIN					1
#define	POLEPRS_MAX					50
#define	POLEPRS_DFLT				4
//
#define	ENCRES_MIN					1000
#define	ENCRES_MAX					16777216
#define	ENCRES_DFLT					10000
//
#define	IQREF_MIN						-MAX_CURRENT_COMMAND
#define	IQREF_MAX						MAX_CURRENT_COMMAND
#define	IQREF_DFLT					0
//
#define	PEAKTIME_MIN				1								// in ms
#define	PEAKTIME_MAX				50000
#define	PEAKTIME_DFLT				500
//
#define	MAXVEL_MIN					0
#define	MAXVEL_MAX					MAX_SPEED
#define	MAXVEL_DFLT					100000
//
#define	MAXACCEL_MIN				0
#define	MAXACCEL_MAX				2000000000
#define MAXACCEL_DFLT				10000000
//
#define	MAXPOSERR_MIN				0
#define	MAXPOSERR_MAX				2000000000		//max position error
#define	MAXPOSERR_DFLT			20
//
#define	MAXVELERR_MIN				0
#define	MAXVELERR_MAX				MAX_SPEED
#define	MAXVELERR_DFLT			32768
//
//DI/DO DEFINE
#define DINPORT_SIZE        8
#define	DINFILT_MAX					100
#define	DOUTPORT_SIZE				12//2
#define DOUTPORT_NUM        4
//
#define	POSGAIN_MIN					0
#define	POSGAIN_MAX					32768
#define	POSGAIN_DFLT				0
//
#define	VELGAIN_MIN					0
#define	VELGAIN_MAX					1000000
#define	VELGAIN_DFLT				0
//
#define	VELKI_MIN						0
#define	VELKI_MAX						100000
#define	VELKI_DFLT					0
//
#define	CURRGAIN_MIN				0
#define	CURRGAIN_MAX				100000
#define	CURRGAIN_DFLT				0
//
#define	CURRKI_MIN					0
#define	CURRKI_MAX					10000
#define	CURRKI_DFLT					0
//
//
#define	PLLKP_MIN						0
#define	PLLKP_MAX						32768
#define	PLLKP_DFLT					5
//
#define	PLLKI_MIN						0
#define	PLLKI_MAX						100000
#define	PLLKI_DFLT					0
//
#define	MOTIONSTAT_MIN			-2147483648
#define	MOTIONSTAT_MAX			2147483647
#define	MOTIONSTAT_DFLT			0
//
#define	CONFLT_MIN					-2147483648
#define	CONFLT_MAX					2147483647
#define	CONFLT_DFLT					0
//
#define POINT_NUM_MAX				4000  
#define POINT_NUM_MIN 			0
#define POINT_NUM_DFLT 			1000
//
#define	RECGAP_MIN			1
#define	RECGAP_MAX			2147483647
#define	RECGAP_DFLT			1
//
#define	RECTRIGPOS_MIN		0
#define	RECTRIGPOS_MAX		100
#define	RECTRIGPOS_DFLT		0
//
#define	RECTRIGTYP_MIN		0
#define	RECTRIGTYP_MAX		7
#define	RECTRIGTYP_DFLT		0
//
//
#define	INJECTTYPE_MIN		INJECT_TYPE_NONE
#define	INJECTTYPE_MAX		INJECT_TYPE_LAST
#define	INJECTTYPE_DFLT		INJECT_TYPE_SIN_DIRECT
//
#define	INJECTPOINT_MIN		INJECT_POINT_CURRREF
#define	INJECTPOINT_MAX		INJECT_POINT_LAST
#define	INJECTPOINT_DFLT	INJECT_POINT_CURRREF
//
#define	INJECTCURRAMP_MIN	0
#define	INJECTCURRAMP_MAX	MAX_CURRENT_COMMAND/2
#define	INJECTCURRAMP_DFLT	MAX_CURRENT_COMMAND/20
//
#define	INJECTVELAMP_MIN	0
#define	INJECTVELAMP_MAX	MAX_SPEED
#define	INJECTVELAMP_DFLT	10000
//
#define	INJECTPOSAMP_MIN	0
#define	INJECTPOSAMP_MAX	2147483647
#define	INJECTPOSAMP_DFLT	100
//
#define	INJECTFREQ_MIN		0
#define	INJECTFREQ_MAX		800000				// This is up to 8000 Hz, since we divide by 100 when using glInjectFreq
#define	INJECTFREQ_DFLT		1000
//extern long	glInjectFreq[NUMBER_OF_AXES];
//
#define	INJECTEDVALUE_MIN	-2147483648
#define	INJECTEDVALUE_MAX	2147483647
#define	INJECTEDVALUE_DFLT	0
//extern long	glInjectedValue[NUMBER_OF_AXES];


//
#define	CURRFILTDEF_MIN		-2147483648
#define	CURRFILTDEF_MAX		2147483647
#define	CURRFILTDEF_DFLT	NO_FILTER
//currFiltDef[NUMBER_OF_AXES][CURRFILT_SIZE];
//
#define	CURRFILTON_MIN		0
#define	CURRFILTON_MAX		1
#define	CURRFILTON_DFLT		0
//currFiltOn[NUMBER_OF_AXES][CURRFILTON_SIZE];
//
#define	VELFILTDEF_MIN		-2147483648
#define	VELFILTDEF_MAX		2147483647
#define	VELFILTDEF_DFLT		NO_FILTER
//velFiltDef[NUMBER_OF_AXES][VELFILT_SIZE];
//
#define	VELFILTON_MIN		0
#define	VELFILTON_MAX		1
#define	VELFILTON_DFLT		0
//velFiltOn[NUMBER_OF_AXES][VELFILTON_SIZE];

// SWITCH VAR //
#define SW_MIN					0
#define SW_MAX					1
#define SW_DFLT					0
//
#define RATEDTOR_MIN		0
#define	RATEDTOR_MAX		5000000
#define RATEDTOR_DFLT	0
//
#define	ENCTYPE_MIN			ENCODER_TYPE_UNKNOWN
#define	ENCTYPE_MAX			LAST_ENCODER_TYPE
#define	ENCTYPE_DFLT		ENCODER_TYPE_INCREMENTAL
//
#define	MOTORRS_MIN					2
#define	MOTORRS_MAX					100000
#define	MOTORRS_DFLT				2
//
#define	MOTORLS_MIN					2
#define	MOTORLS_MAX					100000
#define	MOTORLS_DFLT				2
//
#define	AUTOGJM_MIN			1
#define	AUTOGJM_MAX			2147483647
#define	AUTOGJM_DFLT		1000  // 2.38e-4*2^24
//
#define	ENCFILT_MIN			0
#define	ENCFILT_MAX			255
#define	ENCFILT_DFLT		0
//
#define	VA_MIN				-2147483648
#define	VA_MAX				2147483647
#define	VA_DFLT				0
//
#define	REVPLIM_MIN			-2147483648
#define	REVPLIM_MAX			2147483647
#define	REVPLIM_DFLT		-2000000000
//
#define	FWDPLIM_MIN			-2147483648
#define	FWDPLIM_MAX			2147483647
#define	FWDPLIM_DFLT		2000000000
//
#define	JERK_MIN			0
#define	JERK_MAX			1000
#define	JERK_DFLT			0
//
#define	MOTIONMODE_MIN		MOTION_MODE_INVALID
#define	MOTIONMODE_MAX		LAST_MOTION_MODE
#define	MOTIONMODE_DFLT		POSITION_OP_MODE_MT			
//
#define	SPEED_MIN			-MAX_SPEED
#define	SPEED_MAX			MAX_SPEED
#define	SPEED_DFLT		10
//
#define	ACCEL_MIN			MIN_ACCELERATION
#define	ACCEL_MAX			MAX_ACCELERATION
#define	ACCEL_DFLT		1000
//
#define PROTMASK_MIN			0
#define PROTMASK_MAX			0x7FFFFFFF	
#define PROTMASK_DFLT			0x7FFFFFF8			//默认情况下 屏蔽ABZ短线检测
//
#define VBUS_MIN          0
#define VBUS_MAX          100000
#define VBUS_DFLT         5000
//
#define	MOTIONREASON_MIN	0
#define	MOTIONREASON_MAX	30
#define	MOTIONREASON_DFLT	0
//
#define	RPTWAIT_MIN			0
#define	RPTWAIT_MAX			2147483647
#define	RPTWAIT_DFLT		0
//
#define	CTRL_MODE_MIN			NO_OP_MODE
#define	CTRL_MODE_MAX			LAST_OPERATION_MODE
#define	CTRL_MODE_DFLT		POSITION_OP_MODE
//
#define	MAXPWM_MIN			0
#define	MAXPWM_MAX			65536	//MAX_PWM_RANGE - 100		// todo: Maybe we should consider deadtime ...WSQ 20191016
#define	MAXPWM_DFLT			32767//MAX_PWM_RANGE * 0.95							// The default is 90%
//
#define STOPOP_MIN			0
#define STOPOP_MAX			1
#define STOPOP_DFLT			0
//
#define FAULT_STOP_MIN			0
#define FAULT_STOP_MAX			3
#define FAULT_STOP_DFLT			0
//
#define DEVCE_ADD_MIN			0
#define DEVCE_ADD_MAX			255
#define DEVCE_ADD_DFLT			1
//
#define	DINLOG_MIN			0
#define	DINLOG_MAX			255
#define	DINLOG_DFLT			0
//
#define	DINFILT_MIN			0
#define	DINFILT_MAX			100
#define	DINFILT_DFLT		0
//
#define DINMASK_MIN				0
#define DINMASK_MAX				4095
#define DINMASK_DFLT			0
//
#define	DINMODE_MIN			0
#define	DINMODE_MAX			MAX_INPUT_MODE
#define	DINMODE_DFLT		0
//
#define	DOUTMODE_MIN		0
#define	DOUTMODE_MAX		MAX_OUTPUT_MODE
#define	DOUTMODE_DFLT		0
//
#define	DOUTPORT_MIN		0
#define	DOUTPORT_MAX		255
#define	DOUTPORT_DFLT		0
//
#define	DOUTLOG_MIN			0
#define	DOUTLOG_MAX			255
#define	DOUTLOG_DFLT		0
//
#define DOUTMASK_MIN		0
#define DOUTMASK_MAX		4095
#define DOUTMASK_DFLT		0
//
#define	POSFFW_MIN				0
#define	POSFFW_MAX				65535
#define	POSFFW_DFLT				0
//
#define	POSFFWFILT_MIN		1
#define	POSFFWFILT_MAX		64
#define	POSFFWFILT_DFLT		3
//
#define	STUCK_CURR_MIN		0
#define	STUCK_CURR_MAX		MAX_CURRENT_COMMAND
#define	STUCK_CURR_DFLT		0
//
#define	STUCK_TIME_MIN		0
#define	STUCK_TIME_MAX		MAX_CURRENT_COMMAND
#define	STUCK_TIME_DFLT		0
//
#define	STUCK_VEL_MIN		0
#define	STUCK_VEL_MAX		MAX_CURRENT_COMMAND
#define	STUCK_VEL_DFLT		0
//
#define	INTAGTOL_MIN		-1
#define	INTAGTOL_MAX		2147483647
#define	INTAGTOL_DFLT		50								// 50 [counts] as the default
//
#define	INTAGTIME_MIN		0
#define	INTAGTIME_MAX		SAMPLES_PER_SECOND				// 1 second as the maximum value for in-target decision. Can be any number.
#define	INTAGTIME_DFLT	(SAMPLES_PER_SECOND >> 8)		// 1000ms/256 = 3.9ms as the default
//
#define	BRKONDLY_MIN		0
#define	BRKONDLY_MAX		100000
#define	BRKONDLY_DFLT		200
//
#define	BRKOFFDLY_MIN		0
#define	BRKOFFDLY_MAX		100000
#define	BRKOFFDLY_DFLT	50
//
#define	HOMEWALLCURR_MIN		0
#define	HOMEWALLCURR_MAX		100000
#define	HOMEWALLCURR_DFLT		0
//
#define	HOMEWALLTIME_MIN		0
#define	HOMEWALLTIME_MAX		2000
#define	HOMEWALLTIME_DFLT		100
//
//
#define	REGENON_MIN			5000
#define	REGENON_MAX			60000
#define	REGENON_DFLT		30000
//
#define	REGENOFF_MIN		3000
#define	REGENOFF_MAX		600000
#define	REGENOFF_DFLT		26000

#define	ModBusAdd_MIN		0
#define	ModBusAdd_MAX		127
#define	ModBusAdd_DFLT	1

#define	MODULATEVQ_MIN		100  //标定电压另定义
#define	MODULATEVQ_MAX		MAX_MODULE
#define	MODULATEVQ_DFLT	  2000

// Parameters Macoll define//
#define 	RO					1		// read only
#define 	RW					0
#define 	OKINMOTN		1		// Allow modified when motor in motion
#define		NOINMOTN		0
#define  	OKMOTON			1		//Allow modified when motor enable
#define 	NOMOTON			0
#define 	ARRAY				1		//parameter is array variable
#define		NOARRAY			0
#define  	YSFLASH			1		//CAN be save to flash
#define 	NOFLASH			0
#define  	AXIS				1		//parameters is Aixs par
#define 	NOAXIS			0

#define		_PosRef				0x2001			//ID: 1
#define		_MainPos			0x2002
#define		_deltaPos			0x2003
#define		_VelRef				0x2004
#define		_Vel					0x2005
#define		_VelErr				0x2006
#define		_IqRef				0x2007
#define		_Iq						0x2008
#define		_IqErr				0x2009
#define		_IdRef				0x200A			// ID: 10

#define		_Id						0x200B
#define		_IdErr				0x200C
#define		_DebugTestD		0x200D
#define		_PosKp				0x200E
#define		_PosKi				0x200F
#define		_VelKp				0x2010
#define		_VelKi				0x2011
#define		_TorKp				0x2012
#define		_TorKi				0x2013
#define		_MotionSts		0x2014
#define		_Conflt				0x2015
#define		_AlarmClrReq	0x2016
#define		_MotorEn			0x2017
#define		_StatRegs			0x2018
#define		_RecLen				0x2019
#define		_RecGap				0x201A
//#define		_Reserved			0x201B
#define		_RecTrigval		0x201C			// 触发值
#define		_RecTrigPos		0x201D			// 触发点时刻
#define		_TrigMode			0x201E			// 触发类型
#define		_RecData			0x201F			// 

#define		_RatedCurr		0x2020
#define		_PeakCurr			0x2021
#define		_PeakCurrTim	0x2022
#define		_MaxPhaseCur	0x2023
#define		_MaxVel				0x2024
#define		_MaxPosErr		0x2025
#define		_MaxVelErr		0x2026
#define		_ProtectOn		0x2027
#define		_InjectPoint	0x2028			// ID: 40

#define		_InjectType		0x2029
#define		_InjectCurAmp	0x202A
#define		_InjectVelAmp	0x202B
#define		_InjectPosAmp	0x202C
#define		_InjectFreq		0x202D
#define		_CalcFiltEn		0x202E
#define		_CurrFiltDef	0x202F
#define		_CurrFiltOn		0x2030
#define		_VelFiltDef		0x2031
#define		_VelFiltOn		0x2032			// ID: 50

#define		_Ia						0x2033
#define		_Ib						0x2034
#define		_Ic						0x2035
#define		_Va						0x2036
#define		_Vb						0x2037
#define		_Vc						0x2038
#define		_Vd						0x2039
#define		_Vq						0x203A
#define		_MorCurrFilt	0x203B
#define		_RatedTor			0x203C			// ID: 60

#define		_MaxTor				0x203D
#define		_TorConst			0x203E
#define		_RatedVel			0x203F
#define		_EncType			0x2040
#define		_EncRes				0x2041
#define		_EncDir				0x2042
#define		_EncZeroAng		0x2043
#define		_EncFilt			0x2044
#define		_MotorJm			0x2045
#define		_Poles				0x2046			// ID: 70

#define  	_MotorRs			0x2047
#define  	_MotorLs			0x2048
#define  	_SoftLimEn		0x2049
#define		_SoftRevLim		0x204A
#define		_SoftFwdLim		0x204B
#define		_ErrLog				0x204C
#define		_MaxAcc				0x204D
#define  	_MaxDec				0x204E
#define  	_DirectMode		0x204F
#define  	_Jerk					0x2050			// ID: 80

#define  	_MotionMode		0x2051
#define  	_AbsTrgt			0x2052
#define  	_RelTrgt			0x2053
#define  	_Speed				0x2054
#define  	_Accel				0x2055
#define  	_Decel				0x2056
#define  	_EmrgDec			0x2057
#define  	_MotionReson	0x2058
#define  	_RptWait			0x2059
#define		_CtrlMode			0x205A			// ID: 90

#define  	_ReverseDir		0x205B
#define  	_ProtectMask	0x205C
#define  	_Vbus					0x205D
//#define  	_Reserved		0x205E
#define  	_MaxPwm				0x205F
#define  	_EmgerStpReq	0x2060
#define  	_DisableStpOp	0x2061
#define  	_FaultStpOp		0x2062
#define  	_DeviceAdd			0x2063
#define  	_FWVerion			0x2064		// ID: 100

#define  	_HWVerion			0x2065
#define  	_VersionDate	0x2066
#define  	_InVelLimRev	0x2067
#define  	_InVelLimFwd	0x2068
#define  	_StuckCurr		0x2069
#define  	_StuckTime		0x206A
#define  	_StuckVel			0x206B
#define  	_ComtMode			0x206C
#define  	_ComtInc			0x206D
#define  	_ComtStep			0x206E		// ID: 110

#define  	_StartComt		0x206F	
#define  	_InTargetTol	0x2070
#define  	_InTargetTim	0x2071
#define 	_TriggerID    0x2072
#define 	_PdPos        0x2073
#define  	_TrgtVel		  0x2074
#define  	_TrgtTor		  0x2075
#define  	_DInPort			0x2076
#define  	_Motor_mode		0x2077
#define  	_TriggerSubID	0x2078	// ID: 120

#define  	_DInLog				0x2079
#define  	_DInFilt			0x207A
#define  	_DInputMask		0x207B
#define  	_DInMode			0x207C
#define  	_DOutPort			0x207D
#define  	_OutLog				0x207E
#define  	_DOutMask			0x207F
#define  	_DOutMode			0x2080
#define  	_HomingExec		0x2081
#define  	_HomingOn			0x2082		// ID: 130

#define  	_HomingStat		0x2083
#define  	_HomingDef		0x2084
#define  	_Speed_SW			0x2085		
#define  	_Pll_Kp				0x2086
#define  	_Pll_Ki				0x2087
#define  	_PosFFW				0x2088	
#define  	_PosFFWfilt		0x2089
#define  	_SetPosition	0x208A
#define  	_DOutMode_Pin	0x208B		
#define  	_BrkOffDly		0x208C		//ID: 140


#define  	_BrkOnDly			0x208D				
#define  	_HomeWallTime	0x208E		
#define  	_HomeWallCurr	0x208F
#define  	_RegenOn			0x2090
#define  	_RegenOff			0x2091	
#define  	_Save					0x2092
#define  	_HallStatData	0x2093
#define  	_HallDisP			0x2094		
#define  	_HallDisN			0x2095		
#define  	_ModBus_Add		0x2096	// ID: 150
#define   _ModulateVq   0x2097
#define   _StuckProtSw  0x2098  //堵转保护开关
//Host Computer


#define  	_Reserved		0x2000

#if MAXON_AGREEMENT

#define _MotorCurrent     0x30D1    //电机实际电流
#define _MotorSpeed       0x30D3    //电机实际速度
#define _DigtalIn         0x3141    //数字输入逻辑状态
#define _ErrCode          0x603F    //驱动器错误代码
#define _ControlWord      0x6040    //驱动器控制字(No Used)
#define _StatusWord       0x6041    //驱动器状态字
#define _OperationMode    0x6060    //运动模式
#define _ActualPos        0x6064    //电机实际位置  
#define _TargetVel        0x60FF    //参考速度

#define _MotorEnable      _MotorEn      //Motor Enable/Disable
#define _ClrErrEn         _AlarmClrReq  //Clear Err
#define _QuickStop_Req    _EmgerStpReq  //Motion-stop 请求
#define _MotorAccel       _Accel        //加速度
#define _MotorDecel       _Decel        //减速度
#define _EmcyStop_Decel   _EmrgDec      //急停减速度

#define _MotorDisable     0x6045        //Motor Disable
#define _ControlMode      _CtrlMode     //控制模式
#define _HomingEnable     _HomingExec   //回零开启请求
#define _HomingState      _HomingStat   //回零状态
#define _HomingMode       _HomingDef    //pPosCtrlPar.homingCtrlPar.homingDef[A_AXIS][19]
#define _StartSwitch      _StartComt    //开启换向请求
#define _SaveFlash        _Save         //存储Flash
  
//此处是注释
//关于Maxon通信
//通信格式改动：0%
//通信内容改动：驱动器错误代码  -参考Atalas驱动器
//              驱动器状态字  -参考Atalas驱动器
//              运动模式  -参考Atalas驱动器
//通信内容删减：驱动器状态字
//通信内容增添：电机失能/使能对象，地址_MotorEnable，数据内容为1/0  -参考Atalas驱动器
//              错误清除请求对象，地址-_ClrErrEn，数据内容为1/0  -参考Atalas驱动器
//              运动停止请求对象，地址_QuickStop_Req，数据内容为1/0  -参考Atalas驱动器
//              加速度、减速度、急停减速度 -参考Atalas驱动器

//新增通信示例
//电机使能：90 02 68 04 01 17 20 00 01 00 00 00 A6 01
//电机失能：90 02 68 04 01 17 20 00 00 00 00 00 96 36

//清除驱动器错误：90 02 68 04 01 16 20 00 01 00 00 00 75 46

//运动停止请求：90 02 68 04 01 60 20 00 01 00 00 00 B2 76
//清除运动停止请求：90 02 68 04 01 60 20 00 00 00 00 00 82 41 

//加速度：90 02 68 04 01 55 20 00 xx xx xx xx CRC_L CRCH
//减速度：90 02 68 04 01 56 20 00 xx xx xx xx CRC_L CRCH
//急停减速度：90 02 68 04 01 57 20 00 xx xx xx xx CRC_L CRCH
#endif


u8 Atlas_Write_Flash(u32 WriteAddr,long *array);
u8 Atlas_Read_Flash(u32 WriteAddr,long *array);
u8 Flash_Erase(u32 WriteAddr,s32 len);

void Atlas_LoadDefaultPar(void);
void LoadFlashParams(void);

extern  u32 Flash_ReadWord(u32 faddr);
extern  u8 Flash_GetSector(u32 addr);
extern  void Flash_WriteData(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);
extern  void Flash_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);

#endif



















