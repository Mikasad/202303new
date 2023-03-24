/*
 * Altas_IWM_ErrorHandle.h
 *
 *  Created on: 201909
 *      Author:WSQ
 */


#ifndef INCLUDE_ALTAS_ERRORHANDLE_H_
#define INCLUDE_ALTAS_ERRORHANDLE_H_


#define	ERRLOG_SIZE								65
#define ALARM_NUM_MAX							10




#define	CON_FLT_NONE						   	0		//No fault has occurred

// error code defines
#define ERR_CURR_OVER_MAX						0x001		//	过电流		
#define ERR_CURR_SAT_OVER_PEAK_TIME	0x002		//	电机参考电流过高	
#define ERR_PHASE_A_CURR						0x003		//	A相电流过高		
#define ERR_PHASE_B_CURR						0x004		//	B相电流过高	 		
#define ERR_PHASE_C_CURR						0x005		//	C相电流过高	
#define ERR_OVER_VEL_ERR						0x006   //   over vel err
#define ERR_OVER_POS_ERR						0x007  	// 	 over pos err
#define	ERR_CURR_12_OVER_LOAD				0x008  	//   1.2 multiply Overload
#define	ERR_CURR_15_OVER_LOAD				0x009  	//   1.5 multiply Overload
#define	ERR_CURR_20_OVER_LOAD				0x00A  	//10   2.0 multiply Overload
#define	ERR_CURR_25_OVER_LOAD				0x00B 	//11   2.5 multiply Overload
#define	ERR_CURR_30_OVER_LOAD				0x00C 	//12   3.0 multiply Overload
#define ERR_POS_FOLLOWING_ERR				0x00D   //13

#define ERR_ENC_DISCNNET_A					0x00E	  // 编码器A相未连接 14
#define ERR_ENC_DISCNNET_B					0x00F	  // 编码器B相未连接15
#define ERR_ENC_DISCNNET_Z					0x010	  // 编码器Z相未连接16

#define ERR_HARD_LIM_STATUS						0x011	//  限位状态错误，检查正反限位信号17

#define ERR_HOMING_UNEXPECTED_MOTOR_OFF			0x012  //18
#define ERR_DI_HOMING_FUNC_NOT_ASSIGNED			0x013  //19
#define ERR_HOMING_TIME_OUT						      0x014  //20
#define ERR_HOMING_START_IN_MOTION					0x015  //21
#define ERR_HOMING_UNEXPECTED_STOP_REQ			0x016  //22
#define ERR_125_TIMES_OVER_VELOCITY					0x017  //23
#define	ERR_REGEN_RES_OVER_LOAD		    			0x018  //24
#define	ERR_HALL_STUDY_ERR	    						0x019  //25

#define ERR_SERVO_MAX_LIMIT_VOLTAGE         0x01A  //26
#define ERR_MOTOR_STUCK_ERR                 0x01B  //27

/**************新增DRV8323S错误类型*******************/
#define ERR_UNDERVOLTAGE_LOCKOUT            0x01C  //28
#define ERR_CHARGE_PUMP_UNDERVOLTAGE        0x01D  //29
#define ERR_VDS_OVERCURRENT                 0x01E  //30
#define ERR_SENSE_OVERCURRENT               0x01F  //31
#define ERR_GATE_DRIVE_FAULT                0x020  //32
#define ERR_OVERTEMPERATURE_SHUTDOWN        0x021  //33
#define ERR_OVERTEMPERATURE_WARNING         0x022  //34

#define TOTAL_ERR_TYPE_NUMS					(0x22 + 1)

// phase current
typedef struct _phase_curr
{
	long 				curr[3];
	long 				currCnt[3];
	long 				ctrlFault[3];
	long 				errCode[3];
} PHASE_CURR;

#define OVER_LOAD_TYPE_CNT			6
// mean current
typedef struct _mean_curr
{
	long 				curr;
	long 				currCnt[OVER_LOAD_TYPE_CNT];		// [1.2, 1.5, 2.0, 2.5, 3.0 contCurr peakCurr]
	float 				factor[OVER_LOAD_TYPE_CNT];
	long 				cntMax[OVER_LOAD_TYPE_CNT];
	long 				ctrlFault[OVER_LOAD_TYPE_CNT];
	long 				errCode[OVER_LOAD_TYPE_CNT];
	long 				cmpVal[OVER_LOAD_TYPE_CNT];
} LOAD_CURR;

// error regs
typedef struct _error_regs
{
	long 				ctrlFault;
	long 				errCode;
	short 			logFlag;
	long 				errToLog;
	long 				errTimes;
	long 				errIa;
	long 				errIb;
	long 				errVel;
	long 				errPosRef;
	long 				errPos;
	long 				errVdc;
} ERROR_REGS;

//gsDealW 	errorDeal			bit judjest
#define	ERR_ALARM_BIT					0x0001
#define ERR_ALARM							0x0000		//	alarm
#define ERR_WARNING						0x0001		//	warning

#define ERR_MOTOR_OFF_BIT			0x0010
#define ERR_MOTOR_ON					0x0000		//	no matter motor off
#define ERR_MOTOR_OFF					0x0010		//	must motor off

#define ERR_RESET_ALLOW_BIT		0x1000
#define ERR_RESET_ALLOW				0x1000		//	error allow reset when no power off   //attentaion please: some error can be reseted auotmation,as regen on/off
#define ERR_RESET_NOT_ALLOW		0x0000		//	error not allow reset must power off


extern PHASE_CURR			phaseCurr[NUMBER_OF_AXES];
extern ERROR_REGS			errRegs[NUMBER_OF_AXES];
extern 	short					gsErrlogIndxCtr[];
extern long 					glErrTime;
extern short 					gsErrLogFlag;
extern short					gsDealW;

extern long						glErrLog[NUMBER_OF_AXES][ERRLOG_SIZE];
extern long    				glErrLogPrev[NUMBER_OF_AXES][ERRLOG_SIZE];
extern float       		gfVelHistory[NUMBER_OF_AXES][16];//TODO:20190912

extern void 					ErrorRelatedVarsInit(void);
extern void						AllErrorCounterReset(unsigned int axisNum);
extern void     			PosErrorCheck(unsigned int axisNum);
extern void     			VelErrorCheck(unsigned int axisNum);
extern void 					OverPhaseCurrentCheck(unsigned int axisNum);
extern void 					OverLoadCurrentCheck(unsigned int axisNum);
extern void 					PeakCurrentDurationCheck(unsigned int axisNum);
extern void 					OverVelocityCheck(unsigned int axisNum);
extern void						ClearErrorRegs(void);
extern void 					ErrorLog(long errorDeal, long errCode, short axisNum);
extern void						ErrorClearCheck(void);
extern void 					RegenProtectCheck(void);
extern void           OverVoltageCheck(void);
extern void           MotorStuckCheck(void);

#endif /* INCLUDE_SSM_ERRORHANDLE_H_ */
