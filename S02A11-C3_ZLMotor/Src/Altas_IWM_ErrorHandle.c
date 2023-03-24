/*
 * Altas_IWM_ErrorHandle.c
 *
 *  Created on: 2019
 *      Author: WS
 */

#include "public_h.h"

PHASE_CURR				phaseCurr[NUMBER_OF_AXES];
LOAD_CURR					loadCurr[NUMBER_OF_AXES];
ERROR_REGS				errRegs[NUMBER_OF_AXES];
long							glOverCurrCounter[NUMBER_OF_AXES] = {0};
long 							glErrTime = 1;
short 						gsErrLogFlag = 0;
short							gsDealW;
short 						gsErrlogIndxCtr[NUMBER_OF_AXES] = {0};

long							glErrLog[NUMBER_OF_AXES][ERRLOG_SIZE];
long    					glErrLogPrev[NUMBER_OF_AXES][ERRLOG_SIZE];
long       				glRegenProtectCnt = 0,glRegenED = 0;
long							glRegenTimeCounterOneSecondPrt = 0;
unsigned int      giOverVolCnt = 0;
long              glBusVoltage = 0;
unsigned int      giStuckTimeCnt;
extern long       StuckProtSw;
// phase current init
void PhaseCurrentInit(void)
{
    Uint16 i = 0, j = 0;

    for (i = 0; i < NUMBER_OF_AXES; i++)
    {
        for (j = 0; j < 3; j++)			// init curr val and cnt to zero
        {
            phaseCurr[i].curr[j] = 0;
            phaseCurr[i].currCnt[j] = 0;
        }

        phaseCurr[i].errCode[0] = ERR_PHASE_A_CURR;					// error code
        phaseCurr[i].errCode[1] = ERR_PHASE_B_CURR;
        phaseCurr[i].errCode[2] = ERR_PHASE_C_CURR;
    }
}

// load current init
void LoadCurrentInit(void)
{
    Uint16 i = 0, j = 0;

    for (i = 0; i < NUMBER_OF_AXES; i++)		// axis [0..1]
    {
        loadCurr[i].curr = 0;

        for (j = 0; j < OVER_LOAD_TYPE_CNT; j++)
        {
            loadCurr[i].currCnt[j] = 0;
        }

        loadCurr[i].factor[0] = 1.2;			// over load times
        loadCurr[i].factor[1] = 1.5;
        loadCurr[i].factor[2] = 2.0;
        loadCurr[i].factor[3] = 2.5;
        loadCurr[i].factor[4] = 3.0;
        if (pMotorParSet.tBasePar.ratedCurr[i] == 0)
            pMotorParSet.tBasePar.ratedCurr[i] = 1;
        loadCurr[i].factor[5] = 1.0 * pMotorParSet.tBasePar.maxPhaseCurr[i] / pMotorParSet.tBasePar.ratedCurr[i];

        loadCurr[i].cntMax[0] = OVER_12_LOAD_COUNTER_MAX;		// over load max counter
        loadCurr[i].cntMax[1] = OVER_15_LOAD_COUNTER_MAX;
        loadCurr[i].cntMax[2] = OVER_20_LOAD_COUNTER_MAX;
        loadCurr[i].cntMax[3] = OVER_25_LOAD_COUNTER_MAX;
        loadCurr[i].cntMax[4] = OVER_30_LOAD_COUNTER_MAX;
        loadCurr[i].cntMax[5] = OVER_CURRENT_COUNTER_MAX;

        loadCurr[i].errCode[0] = ERR_CURR_12_OVER_LOAD;			// error code
        loadCurr[i].errCode[1] = ERR_CURR_15_OVER_LOAD;
        loadCurr[i].errCode[2] = ERR_CURR_20_OVER_LOAD;
        loadCurr[i].errCode[3] = ERR_CURR_25_OVER_LOAD;
        loadCurr[i].errCode[4] = ERR_CURR_30_OVER_LOAD;
        loadCurr[i].errCode[5] = ERR_CURR_OVER_MAX;

        loadCurr[i].cmpVal[0] = pMotorParSet.tBasePar.ratedCurr[i] * 6 / 5;			// 1.2
        loadCurr[i].cmpVal[1] = pMotorParSet.tBasePar.ratedCurr[i] * 3 / 2;			// 1.5
        loadCurr[i].cmpVal[2] = pMotorParSet.tBasePar.ratedCurr[i] * 2;					// 2.0
        loadCurr[i].cmpVal[3] = pMotorParSet.tBasePar.ratedCurr[i] * 5 / 2;			// 2.5
        loadCurr[i].cmpVal[4] = pMotorParSet.tBasePar.ratedCurr[i] * 3;					// 3.0
        loadCurr[i].cmpVal[5] = pMotorParSet.tBasePar.ratedCurr[i] * loadCurr[i].factor[5];
    }
}

void ErrorRelatedVarsInit(void)
{
    PhaseCurrentInit();
    LoadCurrentInit();
}

// clear error regs
void ClearErrorRegs(void)
{
    short i = 0;

    for (i = 0; i < NUMBER_OF_AXES; i++)
    {
        errRegs[i].ctrlFault = 0;
        errRegs[i].errCode = 0;
        errRegs[i].logFlag = 0;
        errRegs[i].errToLog = 0;
        errRegs[i].errTimes = 0;
        errRegs[i].errIa = 0;
        errRegs[i].errIb = 0;
        errRegs[i].errVel = 0;
        errRegs[i].errPosRef = 0;
        errRegs[i].errPos = 0;
        errRegs[i].errVdc = 0;
    }
}

// alarm info record and coe emergency data set
// just record for 1 time, and set the send CoE request to Master only 1 time, or this may cause traffic problem
void AlarmInfoRecord(Uint16 axisNum, Uint16 errorCode)
{

    glErrLog[axisNum][gsErrlogIndxCtr[axisNum] + 1] = pAxisPar.conFlt[axisNum];
    glErrLog[axisNum][gsErrlogIndxCtr[axisNum] + 2] = glErrTime++;
    glErrLog[axisNum][gsErrlogIndxCtr[axisNum] + 3] = Feedback_Data.I_u;
    glErrLog[axisNum][gsErrlogIndxCtr[axisNum] + 4] = Feedback_Data.I_v;
    glErrLog[axisNum][gsErrlogIndxCtr[axisNum] + 5] = pAxisPar.vel[axisNum][0];
    glErrLog[axisNum][gsErrlogIndxCtr[axisNum] + 6] = pAxisPar.posRef[axisNum];//pAxisPar.[axisNum]; TODO:...20190912
    glErrLog[axisNum][gsErrlogIndxCtr[axisNum] + 7] = pAxisPar.mainPos[axisNum];
    glErrLog[axisNum][gsErrlogIndxCtr[axisNum] + 8] = pVbusPar.glVBus;

    gsErrlogIndxCtr[axisNum] = gsErrlogIndxCtr[axisNum] + 8;

    if (gsErrlogIndxCtr[axisNum] == (ERRLOG_SIZE - 1))
    {
        gsErrlogIndxCtr[axisNum] = 0;
    }
}

// error log
/*
 * 	errorType�� 	0~***  reserved
 *
 * 				0x01 		GENERAL_TYPE
 * 				0x02		COMMUNICATION_TYPE
 * 				0x03		MOTION_CTRL_TYPE
 * 								#define	ERROR_TYPE_1	0x01
 *								#define ERROR_TYPE_2	0x02
 *								#define	ERROR_TYPE_3	0x03

 *	errorDeal��	bit0:   0--ALM  				1--WARN
 *							bit1:		0--nomater			1--must motor off
 *							bit2:		reserved
 *							bit3:		0--allow reset	1--not allow reset(must power off)
 *	errCode:
 *
 */
void ErrorLog(long errorDeal, long errCode, short axisNum)
{
    if (!pAxisPar.sAlarmClcRequest[axisNum])  // if there is no error clc request, we can record the error
    {
        pAxisPar.conFlt[axisNum] = errCode;
        AlarmInfoRecord(axisNum, errCode);			// alarm info record, used for display and error log

        gsDealW = errorDeal;										//assign to global var
        if (errorDeal & ERR_MOTOR_OFF_BIT)
        {
            pAxisPar.motorEn[axisNum] = MOTOR_OFF;
            glIsrTicker[axisNum] = 0;						// reset ticker
            glWaitSetBrakeOn[axisNum] = 1;			// set brake on request   todo:
        }
    }
}
void AllErrorCounterReset(Uint16 axisNum)
{
    Uint16 i = 0;

    // over phase current counter
    for (i = 0; i < 3; i++)
    {
        phaseCurr[axisNum].currCnt[i] = 0;
    }

    // over load current counter
    for (i = 0; i < OVER_LOAD_TYPE_CNT; i++)
    {
        loadCurr[axisNum].currCnt[i] = 0;
    }

    // peak current counter
    glOverCurrCounter[axisNum] = 0;
}

void PosErrorCheck(Uint16 axisNum)
{
    if ((labs(pAxisPar.posErr[axisNum]) > pProtectPar.maxPosErr[axisNum]) && (pProtectPar.protectMask[axisNum] & 0x08))  //bit3 protectmask
    {
        ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW, ERR_OVER_POS_ERR, axisNum);
    }
}

void VelErrorCheck(Uint16 axisNum)
{
    if ((labs(pAxisPar.velErr[axisNum]) > pProtectPar.maxVelErr[axisNum]) && (pProtectPar.protectMask[axisNum] & 0x10))
    {
        ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW, ERR_OVER_VEL_ERR, axisNum);
    }
}

// over phase current handle
void OverPhaseCurrentCheck(Uint16 axisNum)
{
    Uint16 i = 0;
//	Uint16 errType = 0;

    if (pProtectPar.protectMask[axisNum] & 0x20)
    {
        phaseCurr[axisNum].curr[0] = pMotorParSet.ia[A_AXIS];//Feedback_Data.I_u;
        phaseCurr[axisNum].curr[1] = pMotorParSet.ib[A_AXIS];//Feedback_Data.I_v;
        phaseCurr[axisNum].curr[2] = pMotorParSet.ic[A_AXIS];//-(Feedback_Data.I_u + Feedback_Data.I_v);

        for (i = 0; i < 3; i++)				// loop for phase A, B, C
        {
            if (labs(phaseCurr[axisNum].curr[i]) > pMotorParSet.tBasePar.maxPhaseCurr[axisNum])
            {
                phaseCurr[axisNum].currCnt[i]++;

                if (phaseCurr[axisNum].currCnt[i] >= OVER_CURRENT_COUNTER_MAX)			// time out, report error
                {
//					if (i == 0)
//					{
//						errType = ERR_PHASE_A_CURR;
//					}
//					else if (i == 1)
//					{
//						errType = ERR_PHASE_B_CURR;
//					}
//					else if (i == 2)
//					{
//						errType = ERR_PHASE_C_CURR;
//					}

                    ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW, phaseCurr[axisNum].errCode[i], axisNum);	// error log
                }
            }
            else
            {
                phaseCurr[axisNum].currCnt[i] = 0;
            }
        }
    }
}

// over mean current handle
void OverLoadCurrentCheck(Uint16 axisNum)
{
    Uint16 i = 0;
//	Uint16 errType = 0;

    if (pProtectPar.protectMask[axisNum] & 0x40)
    {
        loadCurr[axisNum].curr = pMotorParSet.motorCurrAbsFilt[axisNum];

        for (i = 0; i < OVER_LOAD_TYPE_CNT; i++)
        {
            if (labs(loadCurr[axisNum].curr > loadCurr[axisNum].cmpVal[i]))
            {
                loadCurr[axisNum].currCnt[i]++;

                if (loadCurr[axisNum].currCnt[i] > loadCurr[axisNum].cntMax[i])					// time out, report error
                {
//					if (i == 0)
//					{
//						errType = ERR_CURR_12_OVER_LOAD;
//					}
//					else if (i == 1)
//					{
//						errType = ERR_CURR_15_OVER_LOAD;
//					}
//					else if (i == 2)
//					{
//						errType = ERR_CURR_20_OVER_LOAD;
//					}
//					else if (i == 3)
//					{
//						errType = ERR_CURR_25_OVER_LOAD;
//					}
//					else if (i == 4)
//					{
//						errType = ERR_CURR_30_OVER_LOAD;
//					}
//					else if (i == 5)
//					{
//						errType = ERR_CURR_OVER_MAX;
//					}
                    ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW, loadCurr[axisNum].errCode[i], axisNum);	// error code log
                }
            }
            else
            {
                loadCurr[axisNum].currCnt[i] = 0;
            }
        }
    }
}

// peak current duration check
void PeakCurrentDurationCheck(Uint16 axisNum)
{
    if ((pAxisPar.statRegs[axisNum] & STAT_REG_CURR_SAT_SET) && (pProtectPar.protectMask[axisNum] & 0x80))
    {
        glOverCurrCounter[axisNum]++;
        if (glOverCurrCounter[axisNum] > ((pProtectPar.peakCurrTime[axisNum]) * 10))
        {
            ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW, ERR_CURR_SAT_OVER_PEAK_TIME, axisNum);		// error code log
        }
    }
    else
    {
        glOverCurrCounter[axisNum] = 0;
    }
}

// 1.25 over velocity check
void OverVelocityCheck(Uint16 axisNum)
{
    // need to check when there is no control fault?
    if ((labs(pAxisPar.vel[axisNum][0]) > ((pProtectPar.maxVel[axisNum] >> 3) * 10)) && (pProtectPar.protectMask[axisNum] & 0x200)) //) && (pAxisPar.conFlt[axisNum] == CON_FLT_NONE)) // Actually, the feedback is allowed to be 25% above the maximal velocity command
    {
        ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW, ERR_125_TIMES_OVER_VELOCITY, axisNum);		// error code log
    }
}

// this func. moved to the background task for the ERR TYPE is too large and costs a lot of time
// and during this process, new error log should not be recorded
void ErrorClearCheck(void)
{
    Uint16 i = 0;
    Uint16 cntTestI = 0;

    for (i = 0; i < NUMBER_OF_AXES; i++)
    {
        if (pAxisPar.sAlarmClcRequest[i])			            // if there is alarm clear request
        {
            if (pAxisPar.conFlt[i] != CON_FLT_NONE)
            {
                if (gsDealW & ERR_RESET_ALLOW_BIT)							//check if the error canbe cleared
                {
                    /*���driver����SPI��ʽ��enableӲ����ʽ*/
                    if ((pAxisPar.conFlt[i]>27)&&(pAxisPar.conFlt[i]<35))
                    {
                        DRV8323S_SPI_WriteRead(0x1001);   //Driver Control Register 0x02(OTW=0b)	���spi���
//							cntTestI ++;
//						  DRV_ENABLE_OFF;
//						  glDebugTestD[29] = 2;
//              for(cntTestI = 1;cntTestI < 10000;cntTestI++)
//							{
//							}
//							DRV_ENABLE_ON;
                        glDebugTestD[29] = 1;
                    }
                    /*���driver����*/
                    pAxisPar.conFlt[i] = 0;		                    // clear control fault
                    pMotionPar.motionReason[i] = MOTION_REASON_END_NONE;//TODO: check other place 20190917
                    pAxisPar.sAlarmClcRequest[i] = 0;
                    RsCOM.Sci_ErrClr_En = 0;
                    gsMachineStat[A_AXIS] = DISABLE_SWITCH_ON; //todo: checking it again ...20190912
                }
                else
                {
                    ;
                    //TODO: think about ....
                    // gsMachineStat[A_AXIS] = DISABLE_SWITCH_ON; //todo: CHECKING change together...
                }
            }
            else
            {
                pAxisPar.sAlarmClcRequest[i] = 0;
                if (gsMachineStat[A_AXIS] == FAULT)
                    gsMachineStat[A_AXIS] = DISABLE_SWITCH_ON;
            }
        }
    }
}

void RegenProtectCheck(void)
{
    if (pProtectPar.protectMask[A_AXIS] & 0x400)
    {
        if ((pVbusPar.glVBus >= pProtectPar.regenOn) && (pAxisPar.conFlt[A_AXIS] != ERR_REGEN_RES_OVER_LOAD))
        {
            glRegenProtectCnt++;
            pAxisPar.statRegs[A_AXIS] = pAxisPar.statRegs[A_AXIS] | STAT_REG_REGENRATION_SET;
            Break_In_On;
        }
        if ((pVbusPar.glVBus <= pProtectPar.regenOff) || (pAxisPar.conFlt[A_AXIS] == ERR_REGEN_RES_OVER_LOAD))
        {
            pAxisPar.statRegs[A_AXIS] = pAxisPar.statRegs[A_AXIS] & STAT_REG_GENERATION_CLEAR;
            Break_In_Off;
        }

        if ((pAxisPar.statRegs[A_AXIS] & STAT_REG_REGENRATION_SET) != 0) //1s����
        {
            glRegenTimeCounterOneSecondPrt++;
            if((glRegenTimeCounterOneSecondPrt > REGEN_OVER_LOAD_COUNTER_MAX) && (pAxisPar.conFlt[A_AXIS] != ERR_REGEN_RES_OVER_LOAD))
            {
                pAxisPar.statRegs[A_AXIS] = pAxisPar.statRegs[A_AXIS] & STAT_REG_GENERATION_CLEAR;
                Break_In_Off;
                pAxisPar.conFlt[A_AXIS] = ERR_REGEN_RES_OVER_LOAD;
                ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW, ERR_REGEN_RES_OVER_LOAD, A_AXIS);	// error code log
            }
        }
        else
        {
            glRegenTimeCounterOneSecondPrt = 0;
        }
    }
}

//�����ѹ(���޹�ѹ)
void OverVoltageCheck(void)
{
    if (pProtectPar.protectMask[A_AXIS] & 0x800)
    {
        glBusVoltage = pVbusPar.glVBus;

        if(glBusVoltage > SERVO_MAX_LIMIT_VOLTAGE)
        {
            giOverVolCnt++;
            if(giOverVolCnt > pProtectPar.maxVoltageTime[A_AXIS]*10)
            {
                ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW,ERR_SERVO_MAX_LIMIT_VOLTAGE,A_AXIS);
                pAxisPar.statRegs[A_AXIS] |= STAT_REG_OVER_MAX_VBUS_SET;
            }
        }
        else
        {
            giOverVolCnt = 0;
            pAxisPar.statRegs[A_AXIS] &= STAT_REG_OVER_MAX_VBUS_CLEAR;
        }
    }
}

//�����ת
void MotorStuckCheck(void)
{
    //Modify by WSQ 20191215 (������)
//  if (pProtectPar.protectMask[A_AXIS] & 0x1000)
    {

        if((labs(pMotorParSet.currRef[A_AXIS]) >= pOlckedRotor.pStuckCurr) && (pAxisPar.vel[A_AXIS][0] <= pOlckedRotor.pStuckVel))
        {
            giStuckTimeCnt++;
            if(giStuckTimeCnt > (pOlckedRotor.pStuckTime*10))
            {
                //Modify by WSQ 20191215 (������)
                giStuckTimeCnt = pOlckedRotor.pStuckTime*10;
                if (pOlckedRotor.pStuckCurr > pMotorParSet.tBasePar.ratedCurr[A_AXIS])
                {
                    pMotorParSet.currRef[A_AXIS] = pMotorParSet.tBasePar.ratedCurr[A_AXIS];
                }
                if ((pProtectPar.protectMask[A_AXIS] & 0x1000)&&(StuckProtSw == 1))         // ��Ӷ�ת�ж������������ת��������StuckProtSw
                    ErrorLog(ERR_ALARM | ERR_MOTOR_OFF | ERR_RESET_ALLOW,ERR_MOTOR_STUCK_ERR,A_AXIS);
            }
        }
        else
        {
            giStuckTimeCnt = 0;
        }
    }
}


