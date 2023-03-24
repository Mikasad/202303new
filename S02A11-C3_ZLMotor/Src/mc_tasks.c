/*
*******************************************************************************
 *版    权： 2021-xxxx,  GaussianRobot
 *文 件 名： mc_task.c 函数
 *简    介： 中频任务 以及 高频任务FOC 电机测速 以及电机所有保护功能
 *作    者： LMmotor\忘川
 *日    期： 2022.1.4
 *功能描述：
*******************************************************************************
 *备注：
*******************************************************************************
*/
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
#include "bsp_app_function.h"
#include "Usart_Labview.h"
#include "stdlib.h"

extern uint8_t HALL_CC_First,HALL2_CC_First;
extern u8 HallStudyFlag1,HallStudyFlag2;
extern int32_t hSpeedRef_Pos;
extern u8 TempID1,TempID2,TempID3,TempID4;
extern u8 _heartbeatErrorFlag;
extern int16_t HAL_Init_Electrical_Angle,HAL_Init_Electrical_Angle2,HAL_CommutationAngle2;
extern s16 Angle_Switch2,Angle_Switch;

extern long LabView_Uart_TxBuffer[4][2010];       /*LABVIEW相关参数*/
extern u16 Uart_TxBuffer_CNT ;
extern u16 Uart_TxBuffer_CNT1;

extern void HALL1_Init_Electrical_Angle( void );
extern void HALL2_Init_Electrical_Angle( void );

extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];
extern s16 PWM_Init_Electrical_Angle,PWM_Init_Electrical_Angle2;

int32_t TIM2_CNT,TIM4_CNT;
int32_t TIM2_CNT_Temp,TIM4_CNT_Temp ;
int32_t VBS_AvBusVoltage_V;                       /*母线电压*/
pSpeedMesa pSpeed_Mesa[NBR_OF_MOTORS];            /*PLL测速算法使用相关参数*/
uint8_t bMCBootCompleted = 0;                     /*启动完成标志*/
LOAD_CURR					loadCurr[NUMBER_OF_AXES];       /*过流比较参数*/
float gfInjectPhase[2] = {0};                     /*参考波形输入参数，用于测试电机响应*/
long glTemp_Inject = 0 ;                          /*参考波形输入参数，用于测试电机响应*/
int16_t MotorParRatedCurr[NUMBER_OF_AXES];        /*电机额定电流*/
int16_t ENC_ElAngle,HALL_ElAngle,ERR_ElAngle,ENC_ElAngle2;
float IQFactor = 1;                               /*理论值为 二分之根号三 约等于0.866 实测修正0.84  更改为相电流保护*/
uint16_t Nominal_Current = 9528; //1191*8
uint32_t BusVoltageFaultsFlag = 0;
uint32_t Hall_Protect_Cnt = 0;
uint32_t CodeReturn = MC_NO_ERROR;
unsigned short  Torque_temp=5000,Flux_temp=5000,Torque_Vq=0,Flux_Vd=0;
int16_t FOC_angle,FOC_angle2;
unsigned short angle_temp=15,ElAngleCountre=0;
int16_t ElAngle[100],ElAngleErr;
int16_t hElAngle;
long Torque_Flux1 = 3000,Torque_Flux2 = 3000;

#define MAX_AXES 2
#define FOC_ARRAY_LENGTH 2
#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)
#define VBUS_TEMP_ERR_MASK ~(0 | 0 | 0)
#define VBUS_TEMP_ERR_MASK2 ~(0 | 0 | 0)

FOCVars_t FOCVars[NBR_OF_MOTORS];
MCI_Handle_t Mci[NBR_OF_MOTORS];
MCI_Handle_t * oMCInterface[NBR_OF_MOTORS];
MCT_Handle_t MCT[NBR_OF_MOTORS];
STM_Handle_t STM[NBR_OF_MOTORS];
SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
PID_Handle_t *pPIDId[NBR_OF_MOTORS];
EncAlign_Handle_t *pEAC[NBR_OF_MOTORS];
RDivider_Handle_t *pBusSensorM1;
FF_Handle_t *pFF[NBR_OF_MOTORS];     /* only if M1 or M2 has FF */
MotorParameters_t MotorParameters[NBR_OF_MOTORS];
PosCtrl_Handle_t *pPosCtrl[NBR_OF_MOTORS];
PID_Handle_t *pPIDPosCtrl[NBR_OF_MOTORS];

Trapezoidal_Handle_t *pTrapezoidal[NBR_OF_MOTORS];
SMC *pSMC_Struct[NBR_OF_MOTORS];

VirtualBusVoltageSensor_Handle_t *pBusSensorM2;
NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
RampExtMngr_Handle_t *pREMNG[NBR_OF_MOTORS];   /*!< Ramp manager used to modify the Iq ref
                                                    during the start-up switch over.*/

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;
static volatile uint16_t hMFTaskCounterM2 = 0;
static volatile uint16_t hBootCapDelayCounterM2 = 0;
static volatile uint16_t hStopPermanencyCounterM2 = 0;
static uint8_t FOC_array[FOC_ARRAY_LENGTH]= { 0, 0 };
static uint8_t FOC_array_head = 0; // Next obj to be executed
static uint8_t FOC_array_tail = 0; // Last arrived


void TSK_MediumFrequencyTaskM1(void);
void FOC_Clear(uint8_t bMotor);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
static uint16_t FOC_CurrControllerM2(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(void);
void TSK_SafetyTask_RBRK(uint8_t motor);
void TSK_MediumFrequencyTaskM2(void);
void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM2(void);
void TSK_SetStopPermanencyTimeM2(uint16_t SysTickCount);
bool TSK_StopPermanencyTimeHasElapsedM2(void);
float AxisVelocityPLLCalc(TIM_TypeDef * TIMx , pSpeedMesa * speedmeas , MotorParameters_t * motorpar);
void AxisVelocityPLLCalcInit(void);
void UI_Scheduler(void);
void MotorParametersM1_Init(void);
void MotorParametersM2_Init(void);
void LoadCurrentInit(void);
uint16_t OverLoadCurrentCheck(uint16_t axisNum);

/*------------------------------------------------
  * @function :启动电机控制
  * @input    :@pMCIList 电机控制接口 @pMCTList 电机控制实体
  * @output   :无
  * @explain  :初始化各个指针
  * @author   :wangchuan
  * @date     :2023/1/5
  ------------------------------------------------*/
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS],MCT_Handle_t* pMCTList[NBR_OF_MOTORS] )
{
    /* USER CODE BEGIN MCboot 0 */

    /* USER CODE END MCboot 0 */

    /**************************************/
    /*    State machine initialization    */
    /**************************************/
    STM_Init(&STM[M1]);                         /*M1状态机初始化*/

    bMCBootCompleted = 0;                       /*启动未完成*/
    pCLM[M1] = &CircleLimitationM1;             /*极限圆限制https://blog.csdn.net/qq_33233481/article/details/111473884  https://blog.csdn.net/wanrenqi/article/details/106462770*/


    pFF[M1] = &FF_M1; /* only if M1 has FF */

    pCLM[M2] = &CircleLimitationM2;

    /**********************************************************/
    /*    PWM and current sensing component initialization    */
    /**********************************************************/
    pwmcHandle[M1] = &PWM_Handle_M1._Super;
    R3_2_Init(&PWM_Handle_M1);                  /*初始化MCU相关的TIM,ADC*/
    pwmcHandle[M2] = &PWM_Handle_M2._Super;
    R3_2_Init(&PWM_Handle_M2);
    /* USER CODE BEGIN MCboot 1 */

    /* USER CODE END MCboot 1 */

    /**************************************/
    /*    Start timers synchronously      */
    /**************************************/
    startTimers();                              /*使用TIM2来触启动其他定时器,TIM1,TIM8都可以被TIM2触发启动*/

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M1);         /*M1初始化速度环参数*/
    pPIDSpeed[M1] = &PIDSpeedHandle_M1;

    /******************************************************/
    /*   Main speed sensor component initialization       */
    /******************************************************/
    pSTC[M1] = &SpeednTorqCtrlM1;
    ENC_Init (&ENCODER_M1);

    /******************************************************/
    /*   Main encoder alignment component initialization  */
    /******************************************************/
    EAC_Init(&EncAlignCtrlM1,pSTC[M1],&VirtualSpeedSensorM1,&ENCODER_M1);
//    EncAlignCtrlM1.hElAngle = HAL_Init_Electrical_Angle;
//    EncAlignCtrlM1.hElAngle = PWM_Init_Electrical_Angle;
//    ENCODER_M1._Super.hElAngle = HAL_Init_Electrical_Angle;
    pEAC[M1] = &EncAlignCtrlM1;


    /******************************************************/
    /*   Position Control component initialization        */
    /******************************************************/
    pPIDPosCtrl[M1] = &PID_PosParamsM1;
    PID_HandleInit(pPIDPosCtrl[M1]);

    pPosCtrl[M1] = &pPosCtrlM1;
    TC_Init(pPosCtrl[M1], pPIDPosCtrl[M1], &SpeednTorqCtrlM1, &ENCODER_M1);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M1],pPIDSpeed[M1], &ENCODER_M1._Super);

    /******************************************************/
    /*   Auxiliary speed sensor component initialization  */
    /******************************************************/
//  HALL_Init (&HALL_M1);

    /****************************************************/
    /*   Virtual speed sensor component initialization  */
    /****************************************************/
    VSS_Init (&VirtualSpeedSensorM1);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);
    pPIDIq[M1] = &PIDIqHandle_M1;
    pPIDId[M1] = &PIDIdHandle_M1;

    /********************************************************/
    /*   Bus voltage sensor component initialization        */
    /********************************************************/
    pBusSensorM1 = &RealBusVoltageSensorParamsM1;
    RVBS_Init(pBusSensorM1);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M1] = &PQD_MotorPowMeasM1;
    pMPM[M1]->pVBS = &(pBusSensorM1->_Super);
    pMPM[M1]->pFOCVars = &FOCVars[M1];

    pR_Brake[M1] = &R_BrakeParamsM1;
    DOUT_SetOutputState(pR_Brake[M1],INACTIVE);

    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
    NTC_Init(&TempSensorParamsM1);
    pTemperatureSensor[M1] = &TempSensorParamsM1;

    /*******************************************************/
    /*   Feed forward component initialization             */
    /*******************************************************/
    FF_Init(pFF[M1],&(pBusSensorM1->_Super),pPIDId[M1],pPIDIq[M1]);

    pREMNG[M1] = &RampExtMngrHFParamsM1;
    REMNG_Init(pREMNG[M1]);

    FOC_Clear(M1);
    FOCVars[M1].bDriveInput = EXTERNAL;
    FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
    FOCVars[M1].UserIdref = STC_GetDefaultIqdref(pSTC[M1]).d;
    oMCInterface[M1] = & Mci[M1];
// MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1] );
    MCI_Init(oMCInterface[M1], &STM[M1], pSTC[M1], &FOCVars[M1], pPosCtrl[M1]);
    MCI_ExecSpeedRamp(oMCInterface[M1],
                      STC_GetMecSpeedRefUnitDefault(pSTC[M1]),0); /*First command to STC*/
    pMCIList[M1] = oMCInterface[M1];
    MCT[M1].pPIDSpeed = pPIDSpeed[M1];
    MCT[M1].pPIDIq = pPIDIq[M1];
    MCT[M1].pPIDId = pPIDId[M1];
    MCT[M1].pPIDFluxWeakening = MC_NULL; /* if M1 doesn't has FW */
    MCT[M1].pPWMnCurrFdbk = pwmcHandle[M1];
    MCT[M1].pRevupCtrl = MC_NULL;              /* only if M1 is not sensorless*/
    MCT[M1].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &ENCODER_M1;
    MCT[M1].pSpeedSensorAux = (SpeednPosFdbk_Handle_t *) &HALL_M1;
    MCT[M1].pSpeedSensorVirtual = MC_NULL;
    MCT[M1].pSpeednTorqueCtrl = pSTC[M1];
    MCT[M1].pStateMachine = &STM[M1];
    MCT[M1].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M1];
    MCT[M1].pBusVoltageSensor = &(pBusSensorM1->_Super);
    MCT[M1].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM1*/
    MCT[M1].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM1*/
    MCT[M1].pMPM =  (MotorPowMeas_Handle_t*)pMPM[M1];
    MCT[M1].pFW = MC_NULL;
    MCT[M1].pFF = pFF[M1];

    MCT[M1].pPosCtrl = pPosCtrl[M1];

    MCT[M1].pSCC = MC_NULL;
    MCT[M1].pOTT = MC_NULL;
    pMCTList[M1] = &MCT[M1];

    /******************************************************/
    /*   Motor 2 features initialization                  */
    /******************************************************/

    /**************************************/
    /*    State machine initialization    */
    /**************************************/
    STM_Init(&STM[M2]);                              /*状态机初始化(IDLE,无错误)*/

    /******************************************************/
    /*   PID component initialization: speed regulation   */
    /******************************************************/
    PID_HandleInit(&PIDSpeedHandle_M2);
    pPIDSpeed[M2] = &PIDSpeedHandle_M2;              /*M2速度环参数初始化*/

    /***********************************************************/
    /*   Main speed  sensor initialization: speed regulation   */
    /***********************************************************/
    pSTC[M2] = &SpeednTorqCtrlM2;                    /*速度扭矩控制*/
    ENC_Init (&ENCODER_M2);                          /*编码器初始化*/

    /******************************************************/
    /*   Main encoder alignment component initialization  */
    /******************************************************/
    EAC_Init(&EncAlignCtrlM2,pSTC[M2],&VirtualSpeedSensorM2,&ENCODER_M2);
//    EncAlignCtrlM2.hElAngle = HAL_Init_Electrical_Angle2;
//    ENCODER_M2._Super.hElAngle = HAL_Init_Electrical_Angle2;
//    EncAlignCtrlM2.hElAngle = PWM_Init_Electrical_Angle2;
    pEAC[M2] = &EncAlignCtrlM2;

    /******************************************************/
    /*   Position Control component initialization        */
    /******************************************************/
    pPIDPosCtrl[M2] = &PID_PosParamsM2;
    PID_HandleInit(pPIDPosCtrl[M2]);

    pPosCtrl[M2] = &pPosCtrlM2;
    TC_Init(pPosCtrl[M2], pPIDPosCtrl[M2], &SpeednTorqCtrlM2, &ENCODER_M2);

    /******************************************************/
    /*   Speed & torque component initialization          */
    /******************************************************/
    STC_Init(pSTC[M2],pPIDSpeed[M2], &ENCODER_M2._Super);
    /****************************************************/
    /*   Virtual speed sensor component initialization  */
    /****************************************************/
    VSS_Init (&VirtualSpeedSensorM2);

    /********************************************************/
    /*   PID component initialization: current regulation   */
    /********************************************************/
    PID_HandleInit(&PIDIqHandle_M2);                 /*Iq*/
    PID_HandleInit(&PIDIdHandle_M2);                 /*Id*/
    pPIDIq[M2] = &PIDIqHandle_M2;
    pPIDId[M2] = &PIDIdHandle_M2;

    /**********************************************************/
    /*   Virtual bus voltage sensor component initialization  */
    /**********************************************************/
    pBusSensorM2 = &VirtualBusVoltageSensorParamsM2; /* powerboard configuration: Rdivider or Virtual*/
    VVBS_Init(pBusSensorM2);

    /*************************************************/
    /*   Power measurement component initialization  */
    /*************************************************/
    pMPM[M2] = &PQD_MotorPowMeasM2;
    pMPM[M2]->pVBS = &(pBusSensorM2->_Super);
    pMPM[M2]->pFOCVars = &FOCVars[M2];

    /*******************************************************/
    /*   Temperature measurement component initialization  */
    /*******************************************************/
    NTC_Init(&TempSensorParamsM2);
    pTemperatureSensor[M2] = &TempSensorParamsM2;
    pREMNG[M2] = &RampExtMngrHFParamsM2;
    REMNG_Init(pREMNG[M2]);
    FOC_Clear(M2);
    FOCVars[M2].bDriveInput = EXTERNAL;
    FOCVars[M2].Iqdref = STC_GetDefaultIqdref(pSTC[M2]);         /*默认的Iq*/
    FOCVars[M2].UserIdref = STC_GetDefaultIqdref(pSTC[M2]).d;
    oMCInterface[M2] = &Mci[M2];
    MCI_Init(oMCInterface[M2], &STM[M2], pSTC[M2], &FOCVars[M2], pPosCtrl[M2]);
    //MCI_Init(oMCInterface[M2], &STM[M2], pSTC[M2], &FOCVars[M2] );
    MCI_ExecSpeedRamp(oMCInterface[M2],
                      STC_GetMecSpeedRefUnitDefault(pSTC[M2]),0); /*First command to STC默认为速度模式 ,设置默认的转速*/
    pMCIList[M2] = oMCInterface[M2];
    MCT[M2].pPIDSpeed = pPIDSpeed[M2];
    MCT[M2].pPIDIq = pPIDIq[M2];
    MCT[M2].pPIDId = pPIDId[M2];
    MCT[M2].pPIDFluxWeakening = MC_NULL; /* if M2 doesn't has FW */
    MCT[M2].pPWMnCurrFdbk = pwmcHandle[M2];
    MCT[M2].pRevupCtrl = MC_NULL;              /* only if M2 is not sensorless*/
    MCT[M2].pSpeedSensorMain = (SpeednPosFdbk_Handle_t *) &ENCODER_M2;
    MCT[M2].pSpeedSensorAux = MC_NULL;
    MCT[M2].pSpeedSensorVirtual = MC_NULL;
    MCT[M2].pSpeednTorqueCtrl = pSTC[M2];
    MCT[M2].pStateMachine = &STM[M2];
    MCT[M2].pTemperatureSensor = (NTC_Handle_t *) pTemperatureSensor[M2];
    MCT[M2].pBusVoltageSensor = &(pBusSensorM2->_Super);
    MCT[M2].pBrakeDigitalOutput = MC_NULL;   /* brake is defined, oBrakeM2*/
    MCT[M2].pNTCRelay = MC_NULL;             /* relay is defined, oRelayM2*/
    MCT[M2].pMPM = (MotorPowMeas_Handle_t*)pMPM[M2];
    MCT[M2].pFW = MC_NULL;
    MCT[M2].pFF = MC_NULL;
    MCT[M2].pPosCtrl = pPosCtrl[M2];
    MCT[M2].pSCC = MC_NULL;
    pMCTList[M2] = &MCT[M2];

//    HALL_Init_Electrical_Angle( &HALL_M1 );
//    HALL1_Init_Electrical_Angle() ;
    AxisVelocityPLLCalcInit();

    Atlas_LoadDefaultPar();//加载对象表格中的默认值
    LoadFlashParams(); //读取FLASH中的参数值
    LoadCurrentInit(); //过流保护初始化
    MotorParametersM1_Init();
    MotorParametersM2_Init();

    ReferenceInjectionLoopPar_Init();
    /* USER CODE END MCboot 2 */
    bMCBootCompleted = 1;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task. 500us
 */
/*------------------------------------------------
* @function :电机控制任务
* @input    :
* @output   :
* @explain  :执行MC_Scheduler,该函数在Systick中被调用,执行频率是 500us
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void MC_RunMotorControlTasks(void)
{
    if ( bMCBootCompleted )
    {
        /* ** Medium Frequency Tasks ** */
        MC_Scheduler();

        /* Safety task is run after Medium Frequency task so that
         * it can overcome actions they initiated if needed. */
        TSK_SafetyTask();

        /* ** User Interface Task ** */
        UI_Scheduler();

    }
}

/*------------------------------------------------
* @function :电机控制调度
* @input    :
* @output   :
* @explain  :执行中等频率任务,调用该函数 执行频率是 500us
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void MC_Scheduler(void)
{
    if (bMCBootCompleted == 1)
    {
        if(hMFTaskCounterM1 > 0u)                                          /*【M1电机】*/
        {
            hMFTaskCounterM1--;                                            /*中频计数器*/
            if(STM[M1].bState != 6 )
            {
                ENC_ElAngle = ENC_CalcAngle (&ENCODER_M1);
                pSpeed_Mesa[M1].fTsw = 0.001;                              /*1ms固定周期，使用锁相环测速，测速与周期直接相关*/
                pCtrlPar[M1].Vel_PLL_Motor = AxisVelocityPLLCalc(TIM4,&pSpeed_Mesa[M1],&MotorParameters[M1]);
                pCtrlPar[M1]._Vel_PLL_Motor = -pCtrlPar[M1].Vel_PLL_Motor;
                pSTC[M1]->VelocityPLLCalc = pCtrlPar[M1].Vel_PLL_Motor;
                SpeedRelMerge_two_to_one(&pCtrlPar[M1],&pCtrlPar[M2]);     /*合并两个电机的转速*/
            }
            Emrgency_Stop();                                               /*急停检测函数*/
            if(Angle_Switch == 3&&Angle_Switch2 == 3)                      /*换向完成*/
            {
                Independent_Motor_Control();
            }
        }
        else
        {
            TSK_MediumFrequencyTaskM1();                                    /*执行中等频率的任务1ms*/
            hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;                     /*M1计数器复位*/
        }
				
        if(hMFTaskCounterM2 > 0u)                                           /*【M1电机】*/
        {
//			if(hMFTaskCounterM2 == MF_TASK_OCCURENCE_TICKS2)
//			{
//				TSK_MediumFrequencyTaskM2();
//			}
            hMFTaskCounterM2--;
            if(STM[M2].bState != 6 )
            {
                ENC_ElAngle2 = ENC_CalcAngle (&ENCODER_M2);
                pSpeed_Mesa[M2].fTsw  = 0.001;                               /*1ms固定周期，使用锁相环测速 FOC 当中测速又修改为0.0001 （10K）*/
                pCtrlPar[M2].Vel_PLL_Motor = AxisVelocityPLLCalc(TIM2,&pSpeed_Mesa[M2],&MotorParameters[M2]);
                pCtrlPar[M2]._Vel_PLL_Motor = -pCtrlPar[M2].Vel_PLL_Motor;   /*实际速度取反两处，一处为未使能时，一处为使能时*/
                pSTC[M2]->VelocityPLLCalc = pCtrlPar[M2].Vel_PLL_Motor;
                SpeedRelMerge_two_to_one(&pCtrlPar[M1],&pCtrlPar[M2]);       /*合并两个电机的转速*/
            }

        }
        else
        {
            TSK_MediumFrequencyTaskM2();                                     /*执行中等频率的任务1ms*/
            hMFTaskCounterM2 = MF_TASK_OCCURENCE_TICKS2;                     /*M2计数器复位*/
        }
        if(hBootCapDelayCounterM1 > 0u)
        {
            hBootCapDelayCounterM1--;                                        /*M1自举电容充电时间*/                      
        }
        if(hStopPermanencyCounterM1 > 0u)
        {
            hStopPermanencyCounterM1--;                                      /*M1状态机STOP的持续时间*/
        }
        if(hBootCapDelayCounterM2 > 0u)                                      /*M2自举电容充电时间*/     
        {
            hBootCapDelayCounterM2--;
        }
        if(hStopPermanencyCounterM2 > 0u)                                    /*M2状态机STOP的持续时间*/
        {
            hStopPermanencyCounterM2--;
        }
    }
    else
    {
    }
}
/*------------------------------------------------
* @function :执行中等频率的任务
* @input    :
* @output   :
* @explain  : 测量电机转速(ENC),状态机->启动或者停止电机
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void TSK_MediumFrequencyTaskM1(void)
{
    State_t StateM1;
    int16_t wAux = 0;
    (void) ENC_CalcAvrgMecSpeedUnit( &ENCODER_M1, &wAux );     /*计算平均机械转速*/
    PQD_CalcElMotorPower( pMPM[M1] );                          /*计算电机平均功率，未使用*/
    StateM1 = STM_GetState( &STM[M1] );                        /*获取状态机*/

    switch ( StateM1 )
    {

    case IDLE:                                                 /*在IDLE中进行初始位置定位*/
        if ( EAC_GetRestartState( &EncAlignCtrlM1 ) )          /*如果在编码器对齐请求后重新启动，则返回true*/
        {
            /* The Encoder Restart State is true: the IDLE state has been entered
             * after Encoder alignment was performed. The motor can now be started. */
//            EAC_SetRestartState( &EncAlignCtrlM1,false );

            STM_NextState( &STM[M1], IDLE_START );
        }
        CNT_1MS[M1] = 0;
        PositionModeFlag1 = 0;                                          /*等于0为了让锁轴状态下过流之后，轮子继续转动之后，再使能锁轴位置没有更新*/

        if(bMCBootCompleted)
        {
//		  MotorParametersM1_Init();
            if(HALL_CC_First<1)
            {
                Uart_TxBuffer_CNT = 0 ;
                Uart_TxBuffer_CNT1 =0 ;
                HALL1_Init_Electrical_Angle();                           /*初始定位函数*/

                EncAlignCtrlM1.hElAngle = HAL_Init_Electrical_Angle;     /*获取到的位置值给到电角度*/
                ENCODER_M1._Super.hElAngle = HAL_Init_Electrical_Angle ;
                TIM4_CNT_Temp = (s32)(HAL_Init_Electrical_Angle * ENCODER_M1.PulseNumber)/(ENCODER_M1._Super.bElToMecRatio*65536);   /*计算当前位置编码器CNT值应该为多少*/
                if(TIM4_CNT_Temp<0)                                      /*定时器CNT值不能为负值*/
                {  
                    TIM4_CNT = TIM4_CNT_Temp + ENCODER_M1.PulseNumber/ENCODER_M1._Super.bElToMecRatio ;
                }
                else
                {
                    TIM4_CNT = TIM4_CNT_Temp ;
                }
                TIM4->CNT = (u16)TIM4_CNT;                               /*注意此处赋值CNT不能超过ARR值*/

                pSpeed_Mesa[M1].fAngle  = ( LL_TIM_GetCounter ( TIM4 )+1)*4-1;
                pSpeed_Mesa[M1].fPreAngle = pSpeed_Mesa[M1].fAngle;
                pSpeed_Mesa[M1].fAngleLast = pSpeed_Mesa[M1].fAngle ;
                if(pCtrlPar[M1].SetVelMotor == 0)
                {
                    ENCODER_M1._Super.wPulseNumber = TIM4->CNT;
                    ENCODER_M1.hmecAngle =  ENCODER_M1._Super.wPulseNumber ;
                    ENCODER_M1.hPremecAngle = ENCODER_M1.hmecAngle ;
                }

//                ENC_ElAngle = ENC_CalcAngle (&ENCODER_M1);
//                ENCODER_M1._Super.wPulseNumber = 0; //不清零会导致上电初始位置不为0
            }
        }

        break;

    case IDLE_START:

//        if ( EAC_IsAligned( &EncAlignCtrlM1 ) == false )
//        {
//            /* The encoder is not aligned. It needs to be and the alignment procedure will make
//             * the state machine go back to IDLE. Setting the Restart State to true ensures that
//             * the start up procedure will carry on after alignment. */
//            EAC_SetRestartState( &EncAlignCtrlM1, true );

//            STM_NextState( &STM[M1], IDLE_ALIGNMENT );
//            break;
//        }
        TIM1->CR2 = 0x00002A00;                                        /*关闭阻尼模式*/
        TIM8->CR2 = 0x00002A00; 
        R3_2_TurnOnLowSides( pwmcHandle[M1] );                         /*先导通下桥臂MOS管*/
        TSK_SetChargeBootCapDelayM1( CHARGE_BOOT_CAP_TICKS );          /*设置自举电容充电时间*/
        STM_NextState( &STM[M1], CHARGE_BOOT_CAP );                    /*-> CHARGE_BOOT_CAP*/
        break;

    case CHARGE_BOOT_CAP:
        if ( TSK_ChargeBootCapDelayHasElapsedM1() )                    /*等待充电时间*/
        {
            PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );    /*开始读取电机电流值校准*/
            STM_NextState(&STM[M1],OFFSET_CALIB);                      /*-> OFFSET_CALIB*/
        }
        break;

    case OFFSET_CALIB:
        if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )   /*读取电流值之前可能需要等待一段时间,*/
        {
            STM_NextState( &STM[M1], CLEAR );
        }
        break;

    case CLEAR:
        ENC_Clear( &ENCODER_M1 );                                     /*清空速度值*/

        if ( STM_NextState( &STM[M1], START ) == true )               /*-> START*/
        {
            FOC_Clear( M1 );                                          /*复位FOC变量*/

            R3_2_SwitchOnPWM( pwmcHandle[M1] );                       /*启动PWM*/
        }
        break;

    case IDLE_ALIGNMENT:                                              /*与IDLE_START一致*/
        R3_2_TurnOnLowSides( pwmcHandle[M1] );                        /*打开下管充电*/
        TSK_SetChargeBootCapDelayM1( CHARGE_BOOT_CAP_TICKS );         /*充电延迟时间*/
        STM_NextState( &STM[M1], ALIGN_CHARGE_BOOT_CAP );             /*  -> ALIGN_CHARGE_BOOT_CAP*/
        break;

    case ALIGN_CHARGE_BOOT_CAP:
        if ( TSK_ChargeBootCapDelayHasElapsedM1() )                   /*如果充电时间已经到达*/
        {
            PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_START );   /*没有电流时，进行偏置电压的校准*/
            STM_NextState(&STM[M1],ALIGN_OFFSET_CALIB);
        }
        break;

    case ALIGN_OFFSET_CALIB:
        if ( PWMC_CurrentReadingCalibr( pwmcHandle[M1], CRC_EXEC ) )  /*没有电流时，进行偏置电压的校准*/
        {
            STM_NextState( &STM[M1], ALIGN_CLEAR );
        }
        break;

    case ALIGN_CLEAR:
        FOCVars[M1].bDriveInput = EXTERNAL;
        STC_SetSpeedSensor( pSTC[M1], &VirtualSpeedSensorM1._Super );
        EAC_StartAlignment( &EncAlignCtrlM1 );                        /*启动编码器对齐过程*/

        if ( STM_NextState( &STM[M1], ALIGNMENT ) == true )
        {
            FOC_Clear( M1 );
            R3_2_SwitchOnPWM( pwmcHandle[M1] );
        }
        break;

    case START:
    {
//        TC_EncAlignmentCommand(pPosCtrl[M1]);
        STM_NextState( &STM[M1], START_RUN ); /* only for sensored*/
    }
    break;

    case ALIGNMENT:
        EncAlignCtrlM1.hElAngle = HAL_Init_Electrical_Angle;//PWM_Init_Electrical_Angle
//        EncAlignCtrlM1.hElAngle = PWM_Init_Electrical_Angle;
//        if ( !EAC_Exec( &EncAlignCtrlM1 ) )
//        {
//            HALL_CC_First = 0;
//            qd_t IqdRef;
//            IqdRef.q = 0;
////      IqdRef.d = STC_CalcTorqueReference( pSTC[M1] );
//            IqdRef.d = 0;//STC_CalcTorqueReference( pSTC[M1] );
//            FOCVars[M1].Iqdref = IqdRef;
//        }
//        else
        {
            R3_2_SwitchOffPWM( pwmcHandle[M1] );
            STC_SetControlMode( pSTC[M1], STC_SPEED_MODE );
            STC_SetSpeedSensor( pSTC[M1], &ENCODER_M1._Super );
            STM_NextState( &STM[M1], ANY_STOP );
        }
        break;

    case START_RUN:                                             
    {
//        FOC_InitAdditionalMethods(M1);
//        FOC_CalcCurrRef( M1 );
        STM_NextState( &STM[M1], RUN );
    }
    STC_ForceSpeedReferenceToCurrentSpeed( pSTC[M1] );          /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands( oMCInterface[M1] );               /* Exec the speed ramp after changing of the speed sensor */

    break;

    case RUN:
        TC_PositionRegulation(pPosCtrl[M1]);
        if (pPosCtrl[M1]->PositionControlRegulation == ENABLE)
        {
            pCtrlPar[M1].SetVelMotor = hSpeedRef_Pos ;
        }
        MCI_ExecBufferedCommands( oMCInterface[M1] );           /*执行缓存指令里面的指令*/
        pSTC[M1]->VelocityPLLCalc = pCtrlPar[M1].Vel_PLL_Motor;
        SpeedRelMerge_two_to_one(&pCtrlPar[M1],&pCtrlPar[M2]);  /*合并两个电机的转速*/
//		  pSTC[M1]->VelocityPLLCalc = (int16_t)imuData.Gy*10;
        FOC_CalcCurrRef( M1 );                                  /*FOC 更新目标值(电流值)*/
        break;

    case ANY_STOP:
        R3_2_SwitchOffPWM( pwmcHandle[M1] );                    /*关闭PWM*/
        FOC_Clear( M1 );                                        /*复位FOC变量*/
        MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M1] );
        TSK_SetStopPermanencyTimeM1( STOPPERMANENCY_TICKS );    /*设置停止持续时间*/
        STM_NextState( &STM[M1], STOP );
        break;

    case STOP:
        if ( TSK_StopPermanencyTimeHasElapsedM1() )             /*等待持续时间*/
        {
            STM_NextState( &STM[M1], STOP_IDLE );               /*-> STOP_IDLE*/
        }
        break;

    case STOP_IDLE:
        STM_NextState( &STM[M1], IDLE );                        /* -> IDLE*/
        break;

    default:
        break;
    }
}
/*------------------------------------------------
* @function :复位FOC的变量
* @input    :@bMotor 电机编号
* @output   :无
* @explain  :复位变量,关闭PWM
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void FOC_Clear(uint8_t bMotor)
{
    ab_t NULL_ab = {(int16_t)0, (int16_t)0};
    qd_t NULL_qd = {(int16_t)0, (int16_t)0};
    alphabeta_t NULL_alphabeta = {(int16_t)0, (int16_t)0};

    FOCVars[bMotor].Iab = NULL_ab;
    FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
    FOCVars[bMotor].Iqd = NULL_qd;
    FOCVars[bMotor].Iqdref = NULL_qd;
    FOCVars[bMotor].hTeref = (int16_t)0;
    FOCVars[bMotor].Vqd = NULL_qd;
    FOCVars[bMotor].Valphabeta = NULL_alphabeta;
    FOCVars[bMotor].hElAngle = (int16_t)0;

    PID_SetIntegralTerm(pPIDIq[bMotor], (int32_t)0);
    PID_SetIntegralTerm(pPIDId[bMotor], (int32_t)0);

    STC_Clear(pSTC[bMotor]);

    PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

    if (pFF[bMotor])
    {
        FF_Clear(pFF[bMotor]);
    }
}

/**
  * @brief  Use this method to initialize additional methods (if any) in
  *         START_TO_RUN state
  * @param  bMotor related motor it can be M1 or M2
  * @retval none
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor)
{
    /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */
//    if (pFF[bMotor])
//    {
//        FF_InitFOCAdditionalMethods(pFF[bMotor]);
//    }
    /* USER CODE END FOC_InitAdditionalMethods 0 */
}
/*------------------------------------------------
* @function :更新目标值
* @input    :@bMotor 电机编号
* @output   :无
* @explain  :计算新的目标值,使用Ramp的过程需要更新目标值,如果是速度模式则会再执行一次PID算法 
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{

    /* USER CODE BEGIN FOC_CalcCurrRef 0 */

    /* USER CODE END FOC_CalcCurrRef 0 */
    if(FOCVars[bMotor].bDriveInput == INTERNAL)
    {
        FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
        FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;
    }
}
/*------------------------------------------------
* @function :自举电容充电时间设置
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
    hBootCapDelayCounterM1 = hTickCount;
}
/*------------------------------------------------
* @function :查询自举电容充电是否已完成
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
    bool retVal = false;
    if (hBootCapDelayCounterM1 == 0)
    {
        retVal = true;
    }
    return (retVal);
}

/*------------------------------------------------
* @function :设置STOP持续时间
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
    hStopPermanencyCounterM1 = hTickCount;
}
/*------------------------------------------------
* @function :查询STOP持续时间
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
    bool retVal = false;
    if (hStopPermanencyCounterM1 == 0)
    {
        retVal = true;
    }
    return (retVal);
}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/*------------------------------------------------
* @function :M2的中频任务
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak void TSK_MediumFrequencyTaskM2(void)
{
    /* USER CODE BEGIN MediumFrequencyTask M2 0 */
    
    /* USER CODE END MediumFrequencyTask M2 0 */
    State_t StateM2;
    int16_t wAux = 0;
    (void) ENC_CalcAvrgMecSpeedUnit( &ENCODER_M2, &wAux );
    PQD_CalcElMotorPower( pMPM[M2] );

    StateM2 = STM_GetState( &STM[M2] );

    switch ( StateM2 )
    {

    case IDLE:
        if ( EAC_GetRestartState( &EncAlignCtrlM2 ) )
        {
            /* The Encoder Restart State is true: the IDLE state has been entered
             * after Encoder alignment was performed. The motor can now be started. */
            EAC_SetRestartState( &EncAlignCtrlM2,false );

            STM_NextState( &STM[M2], IDLE_START );
        }
        CNT_1MS[M2] = 0;
        PositionModeFlag2 = 0;                                         /*等于0为了让锁轴状态下过流之后，轮子继续转动之后，再使能锁轴位置没有更新*/
        if(bMCBootCompleted)
        {
            if(HALL2_CC_First<1)                                       /*初始定位，修改为小于1*/
            {
                Uart_TxBuffer_CNT = 0 ;
                Uart_TxBuffer_CNT1 =0 ;
                HALL2_Init_Electrical_Angle();                         /*初始定位函数*/
							
                EncAlignCtrlM2.hElAngle = HAL_Init_Electrical_Angle2;  /*获取到的位置值给到电角度*/
                ENCODER_M2._Super.hElAngle = HAL_Init_Electrical_Angle2 ;
                TIM2_CNT_Temp = (s32)(HAL_Init_Electrical_Angle2 * ENCODER_M2.PulseNumber)/(ENCODER_M2._Super.bElToMecRatio*65536);    /*计算当前位置编码器CNT值应该为多少*/
                if(TIM2_CNT_Temp<0)                                    /*定时器CNT值不能为负值*/
                {
                    TIM2_CNT = TIM2_CNT_Temp + ENCODER_M2.PulseNumber/ENCODER_M2._Super.bElToMecRatio ;
                }
                else
                {
                    TIM2_CNT = TIM2_CNT_Temp ;
                }
                TIM2->CNT = (u16)TIM2_CNT;                             /*注意此处赋值CNT不能超过ARR值*/

                pSpeed_Mesa[M2].fAngle  = ( LL_TIM_GetCounter ( TIM2 )+1)*4-1;
                pSpeed_Mesa[M2].fPreAngle = pSpeed_Mesa[M2].fAngle;
                pSpeed_Mesa[M2].fAngleLast = pSpeed_Mesa[M2].fAngle ;
                if(pCtrlPar[M2].SetVelMotor == 0)
                {
                    ENCODER_M2._Super.wPulseNumber = TIM2->CNT;
                    ENCODER_M2.hmecAngle =  ENCODER_M2._Super.wPulseNumber ;
                    ENCODER_M2.hPremecAngle = ENCODER_M2.hmecAngle ;
                }
//                ENC_ElAngle2 = ENC_CalcAngle (&ENCODER_M2);
//                ENCODER_M2._Super.wPulseNumber = 0;
            }
        }
        break;

    case IDLE_START:
        /*  only for encoder*/
        R3_2_TurnOnLowSides( pwmcHandle[M2] );
        TSK_SetChargeBootCapDelayM2( CHARGE_BOOT_CAP_TICKS2 );
        STM_NextState( &STM[M2], CHARGE_BOOT_CAP );
        break;

    case CHARGE_BOOT_CAP:
        if ( TSK_ChargeBootCapDelayHasElapsedM2() )
        {
            PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_START );
            STM_NextState( &STM[M2], OFFSET_CALIB );
        }
        break;

    case OFFSET_CALIB:
        if ( PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_EXEC ) )
        {
            STM_NextState( &STM[M2], CLEAR );
        }
        break;

    case CLEAR:
        ENC_Clear( &ENCODER_M2 );

        if ( STM_NextState( &STM[M2], START ) == true )
        {
            FOC_Clear(M2);
            R3_2_SwitchOnPWM( pwmcHandle[M2] );
        }
        break;

        /*  only for encoder*/
    case IDLE_ALIGNMENT:
        R3_2_TurnOnLowSides( pwmcHandle[M2] );
        TSK_SetChargeBootCapDelayM2( CHARGE_BOOT_CAP_TICKS2 );
        STM_NextState( &STM[M2], ALIGN_CHARGE_BOOT_CAP );
        break;

    case ALIGN_CHARGE_BOOT_CAP:
        if ( TSK_ChargeBootCapDelayHasElapsedM2() )
        {
            PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_START );
            STM_NextState( &STM[M2], ALIGN_OFFSET_CALIB );
        }
        break;

    case ALIGN_OFFSET_CALIB:
        if ( PWMC_CurrentReadingCalibr( pwmcHandle[M2], CRC_EXEC ) )
        {
            STM_NextState( &STM[M2], ALIGN_CLEAR );
        }
        break;

    case ALIGN_CLEAR:
        FOCVars[M2].bDriveInput = EXTERNAL;
        STC_SetSpeedSensor( pSTC[M2], &VirtualSpeedSensorM2._Super );
        EAC_StartAlignment( &EncAlignCtrlM2 );

        if ( STM_NextState( &STM[M2], ALIGNMENT ) == true )
        {
            FOC_Clear( M2 );
            R3_2_SwitchOnPWM( pwmcHandle[M2] );
        }
        break;

    case START:
    {
//      TC_EncAlignmentCommand(pPosCtrl[M2]);
        STM_NextState( &STM[M2], START_RUN );
    }
    break;

    /*  only for encoder*/
    case ALIGNMENT:
        EncAlignCtrlM2.hElAngle = HAL_Init_Electrical_Angle2;
        if ( !EAC_Exec( &EncAlignCtrlM2 ) )
        {
            HALL2_CC_First=0;
            qd_t IqdRef;
            IqdRef.q = 0;
            IqdRef.d = 0;   //STC_CalcTorqueReference( pSTC[M2] );
            FOCVars[M2].Iqdref = IqdRef;
        }
        else
        {
            R3_2_SwitchOffPWM( pwmcHandle[M2] );
            STC_SetControlMode( pSTC[M2], STC_SPEED_MODE );
            STC_SetSpeedSensor( pSTC[M2], &ENCODER_M2._Super );
            STM_NextState( &STM[M2], ANY_STOP );
        }
        break;

    case START_RUN:
    {
//        FOC_InitAdditionalMethods( M2 );
//        FOC_CalcCurrRef( M2 );
        STM_NextState(&STM[M2], RUN);
    }
    STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M2]); /* Init the reference speed to current speed */
    MCI_ExecBufferedCommands(oMCInterface[M2]); /* Exec the speed ramp after changing of the speed sensor */
    break;

    case RUN:
        TC_PositionRegulation(pPosCtrl[M2]);
        if (pPosCtrl[M2]->PositionControlRegulation == ENABLE)
        {
            pCtrlPar[M2].SetVelMotor = hSpeedRef_Pos ;
        }
        MCI_ExecBufferedCommands( oMCInterface[M2] );
        pSTC[M2]->VelocityPLLCalc = pCtrlPar[M2].Vel_PLL_Motor;
        SpeedRelMerge_two_to_one(&pCtrlPar[M1],&pCtrlPar[M2]); //合并两个电机的转速
//				pSTC[M2]->VelocityPLLCalc = ENCODER_M2._Super.hAvrMecSpeedUnit;
        FOC_CalcCurrRef( M2 );
        break;

    case ANY_STOP:
        R3_2_SwitchOffPWM( pwmcHandle[M2] );
        FOC_Clear( M2 );
        MPM_Clear( (MotorPowMeas_Handle_t*) pMPM[M2] );
        TSK_SetStopPermanencyTimeM2( STOPPERMANENCY_TICKS2 );
        STM_NextState(&STM[M2], STOP);
        break;

    case STOP:
        if ( TSK_StopPermanencyTimeHasElapsedM2() )
        {
            STM_NextState( &STM[M2], STOP_IDLE );
        }
        break;

    case STOP_IDLE:
        STM_NextState( &STM[M2], IDLE );
        break;

    default:
        break;
    }
}
/*------------------------------------------------
* @function :自举电容充电时间设置
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void TSK_SetChargeBootCapDelayM2(uint16_t hTickCount)
{
    hBootCapDelayCounterM2 = hTickCount;
}

/*------------------------------------------------
* @function :查询自举电容充电是否已完成
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak bool TSK_ChargeBootCapDelayHasElapsedM2(void)
{
    bool retVal = false;
    if (hBootCapDelayCounterM2 == 0)
    {
        retVal = true;
    }
    return (retVal);
}
/*------------------------------------------------
* @function :设置STOP持续时间
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak void TSK_SetStopPermanencyTimeM2(uint16_t hTickCount)
{
    hStopPermanencyCounterM2 = hTickCount;
}
/*------------------------------------------------
* @function :查询STOP持续时间
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak bool TSK_StopPermanencyTimeHasElapsedM2(void)
{
    bool retVal = false;
    if (hStopPermanencyCounterM2 == 0)
    {
        retVal = true;
    }
    return (retVal);
}


#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/*------------------------------------------------
* @function :高频任务
* @input    :
* @output   :
* @explain  :执行频率是ADC的采样频率,也就是PWM频率
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak uint8_t TSK_HighFrequencyTask(void)
{
    uint8_t bMotorNbr = 0;
    uint16_t hFOCreturn;
    bMotorNbr = FOC_array[FOC_array_head];                           /*分配前50us和后50us*/
    if (bMotorNbr == M1)                                             /*100us一次*/
    {
        if(HALL_CC_First<2&&(HallStudyFlag1==0))
        {
            ENC_CalcAngle (&ENCODER_M1);                             /*计算电角度*/
        }
        else
        {
            ENC_ElAngle = ENC_CalcAngle (&ENCODER_M1);               /*100us获取到的电角度给霍尔学习累加和FOC计算*/
            HALL_CC_First = 2;
        }
        hFOCreturn = FOC_CurrControllerM1();                         /*M1Foc计算*/
    }
    else /* bMotorNbr != M1 */                   /*100us一次*/
    {
        if( (HALL2_CC_First < 2) && (HallStudyFlag2==0) )
        {
            ENC_CalcAngle (&ENCODER_M2);
        }
        else
        {
            ENC_ElAngle2 = ENC_CalcAngle (&ENCODER_M2);
            HALL2_CC_First = 2;
        }
        hFOCreturn = FOC_CurrControllerM2();                         /*M2Foc计算*/
    }

    if(hFOCreturn == MC_FOC_DURATION)                                /*检查FOC执行情况，暂未使用  在执行FOC算法期间出现定时器更新事件*/
    {
//    STM_FaultProcessing(&STM[bMotorNbr], MC_FOC_DURATION, 0);      /*错误报警*/
    }
    else
    {
        if (bMotorNbr == M1)
        {
        }
        else // bMotorNbr != M1
        {
        }
    }
    FOC_array_head++;                                               /*前50us执行M1Foc,后50us执行M2Foc*/
    if (FOC_array_head == FOC_ARRAY_LENGTH)
    {
        FOC_array_head = 0;
    }
    return bMotorNbr;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/*------------------------------------------------
* @function :FOC电流控制器
* @input    :
* @output   :
* @explain  :FOC电流控制器
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
inline uint16_t FOC_CurrControllerM1(void)
{
    qd_t Iqd, Vqd;
    ab_t Iab;
    alphabeta_t Ialphabeta, Valphabeta;

    int16_t hElAngle;
    uint16_t hCodeError;
    SpeednPosFdbk_Handle_t *speedHandle;
    pSpeed_Mesa[M1].fTsw =  0.0001;                                               /*运行时修改PLL测速周期*/
    pCtrlPar[M1].Vel_PLL_Motor =AxisVelocityPLLCalc(TIM4,&pSpeed_Mesa[M1],&MotorParameters[M1]);
    pCtrlPar[M1]._Vel_PLL_Motor = -pCtrlPar[M1].Vel_PLL_Motor;
    speedHandle = STC_GetSpeedSensor(pSTC[M1]);                  
    hElAngle = SPD_GetElAngle(speedHandle);                                       /*电角度*/
    PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);                                  /*获取相电流 根据扇区获取A,B,C相电流 Ia, Ib, Ic*/
    Ialphabeta = MCM_Clarke(Iab);                                                 /*Clarke变换 Ia,Ib,Ic -> Iα,Iβ*/
    Iqd = MCM_Park(Ialphabeta, hElAngle);                                         /*Park变换   Iα,Iβ - >  Iq,Id*/
    if(loopTestPar.InjectPoint[M1] == INJECT_POINT_CURRREF&&loopTestPar.swEn[M1])
    {
        ReferenceInjection( &loopTestPar.InjectPoint[M1],
                            &loopTestPar.InjectType[M1],
                            &gfInjectPhase[M1],
                            &loopTestPar.InjectedValue[M1],
                            &loopTestPar.InjectFreq[M1],
                            &loopTestPar.InjectCurrAmp[M1],
                            &glTemp_Inject,
                            &FOCVars[M1].Iqdref.q);
    }


    Vqd.d = PI_Controller(pPIDId[M1],
                          (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);                /*PI控制器   Iq -> Vd*/

    Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);    /*PI控制器   Id -> Vq*/

//	if(PID_SMC_Flag[0] == 1)                                                       /*滑膜控制器相关*/
//	{
//		Vqd.q = SMC_General((int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q  ,  &pSMC_IQ_StructM1);
//	}
//	else if(PID_SMC_Flag[0] == 0)
//	{
//	  Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
//	}
//	else if(PID_SMC_Flag[0] == 2)
//	{
//	  Vqd.q = FOCVars[M1].Iqdref.q ;
//		Vqd.d=0;
//	}




    Vqd = FF_VqdConditioning(pFF[M1],Vqd);

    Vqd = Circle_Limitation(pCLM[M1], Vqd);                                          /* 圆限制 100%*/
    hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;   /**/

    if(Angle_Switch==1)                                                              /*拖动到电角度为0的位置启动*/
    {
        FOC_angle += angle_temp;
        ElAngleErr = FOC_angle - hElAngle;
        hElAngle = FOC_angle;
        Vqd.q=Torque_Flux1;
        Vqd.d=0;
    }
    else if(Angle_Switch==2)                                                          /*强制拖动旋转*/
    {
        ENCODER_M1.Angle_Compensation = 0 ;
        hElAngle += ElAngleErr;
        FOC_angle2 = hElAngle;
        Vqd.q=Torque_Flux1;
        Vqd.d=0;
        Flux_Vd = 0;
    }
    else if(Angle_Switch==0)
    {
//		TIM3->CNT = 0;
//	  hElAngle += ElAngleErr;
//		FOC_angle2 = hElAngle;
//    Vqd.q=0;
//
//		if(Flux_Vd < Flux_temp )
//		{
//			Flux_Vd++;
//			Vqd.d = Flux_Vd;
//		}

        TIM4->CNT = 0;
        hElAngle += ElAngleErr;
        FOC_angle2 = hElAngle;
        Vqd.q=0;
        Vqd.d=Torque_Flux1;
        HALL_CC_First = 2;
        ENCODER_M1.Angle_Compensation = 0 ;

    }

    if(Uart_TxBuffer_CNT1 < LabView_Uart_TxBuffersize)
    {
#ifdef UART_DEBUG
        LV_Uart_TxBuffer[0][Uart_TxBuffer_CNT1] = SpeednTorqCtrlM1.SpeedRefUnitExt/65536;//FOCVars[M2].Iab.a Sum_Of_Phase_CurrentM2[0]/datalittle
        LV_Uart_TxBuffer[1][Uart_TxBuffer_CNT1] = pCtrlPar[M1].Vel_PLL_Motor;//FOCVars[M1].Iqd.q  FOCVars[M2].Iab.b Sum_Of_Phase_CurrentM2[1]/datalittle;.SpeedRefUnitExt/65536
        LV_Uart_TxBuffer[2][Uart_TxBuffer_CNT1] = SpeednTorqCtrlM2.SpeedRefUnitExt/65536;//Missing_phase_cnt[M2]*1000-(FOCVars[M2].Iab.a+FOCVars[M2].Iab.b)FOCVars[M2].Iqd.q 10
        LV_Uart_TxBuffer[3][Uart_TxBuffer_CNT1] = pCtrlPar[M2].Vel_PLL_Motor;//FOCVars[M2].Iqdref.q (s16)(SpeednTorqCtrlM1.SpeedRefUnitExt*10/65536);LabView_Uart_TxBuffer
#endif

#ifdef Usart_Labview
        LabView_Uart_TxBuffer[0][Uart_TxBuffer_CNT1] = *PARAMETER[TempID1].lpParam ;
        LabView_Uart_TxBuffer[1][Uart_TxBuffer_CNT1] = *PARAMETER[TempID2].lpParam ;//HallState1 * 1000 ; //FOCVars[M1].Iqd.q  ;
        LabView_Uart_TxBuffer[2][Uart_TxBuffer_CNT1] = *PARAMETER[TempID3].lpParam ;//HallState2 * 1000 ;//pCtrlPar[M1].Vel_PLL_Motor*10 ;
        LabView_Uart_TxBuffer[3][Uart_TxBuffer_CNT1] = *PARAMETER[TempID4].lpParam ;//(s16)(SpeednTorqCtrlM1.SpeedRefUnitExt*10/65536);LabView_Uart_TxBuffer
#endif
    }
    Uart_TxBuffer_CNT1++;
    if(Uart_TxBuffer_CNT1 >LabView_Uart_TxBuffersize-1)
    {
        Uart_TxBuffer_CNT1 = LabView_Uart_TxBuffersize-1 ;
    }
    Valphabeta = MCM_Rev_Park(Vqd, hElAngle);                                  /*Rev_Park变换 Vq, Vd -> Vα,Vβ*/
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);             /*SVPWM实现函数*/
    FOCVars[M1].Vqd = Vqd;
    FOCVars[M1].Iab = Iab;
    FOCVars[M1].Ialphabeta = Ialphabeta;
    FOCVars[M1].Iqd = Iqd;
    FOCVars[M1].Valphabeta = Valphabeta;
    FOCVars[M1].hElAngle = hElAngle;
    FF_DataProcess(pFF[M1]);
    return(hCodeError);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/*------------------------------------------------
* @function :FOC电流控制器
* @input    :
* @output   :
* @explain  :FOC电流控制器
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
inline uint16_t FOC_CurrControllerM2(void)
{
    ab_t Iab;
    alphabeta_t Ialphabeta, Valphabeta;
    qd_t Iqd, Vqd;

    int16_t hElAngle;
    uint16_t hCodeError;
    SpeednPosFdbk_Handle_t *speedHandle;
    pSpeed_Mesa[M2].fTsw = 0.0001;
    pCtrlPar[M2].Vel_PLL_Motor = AxisVelocityPLLCalc(TIM2,&pSpeed_Mesa[M2],&MotorParameters[M2]);
    pCtrlPar[M2]._Vel_PLL_Motor = -pCtrlPar[M2].Vel_PLL_Motor;//实际速度取反
    speedHandle = STC_GetSpeedSensor(pSTC[M2]);
    hElAngle = SPD_GetElAngle(speedHandle);
    PWMC_GetPhaseCurrents(pwmcHandle[M2], &Iab);
    Ialphabeta = MCM_Clarke(Iab);
    Iqd = MCM_Park(Ialphabeta, hElAngle);

    if(loopTestPar.InjectPoint[M2] == INJECT_POINT_CURRREF&&loopTestPar.swEn[M2])
    {
        ReferenceInjection( &loopTestPar.InjectPoint[M2],
                            &loopTestPar.InjectType[M2],
                            &gfInjectPhase[M2],
                            &loopTestPar.InjectedValue[M2],
                            &loopTestPar.InjectFreq[M2],
                            &loopTestPar.InjectCurrAmp[M2],
                            &glTemp_Inject,
                            &FOCVars[M2].Iqdref.q);
    }
    Vqd.q = PI_Controller(pPIDIq[M2],
                          (int32_t)(FOCVars[M2].Iqdref.q) - Iqd.q);

//	if(PID_SMC_Flag2[0] == 1)
//	{
//		Vqd.q = SMC_General((int32_t)(FOCVars[M2].Iqdref.q) - Iqd.q  ,  &pSMC_IQ_StructM2);
//	}
//	else if(PID_SMC_Flag2[0] == 0)
//	{
//	  Vqd.q = PI_Controller(pPIDIq[M2], (int32_t)(FOCVars[M2].Iqdref.q) - Iqd.q);
//	}
//	else if(PID_SMC_Flag2[0] == 2)
//	{
//	  Vqd.q = FOCVars[M2].Iqdref.q ;
//	}


    Vqd.d = PI_Controller(pPIDId[M2],
                          (int32_t)(FOCVars[M2].Iqdref.d) - Iqd.d);

    Vqd = Circle_Limitation(pCLM[M2], Vqd);
    hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR2;

    if(Angle_Switch2==1)
    {
        FOC_angle += angle_temp;
        ElAngleErr = FOC_angle - hElAngle;
        hElAngle = FOC_angle;
        Vqd.q=Torque_Flux2;
        Vqd.d=0;
    }
    else if(Angle_Switch2==2)
    {

        hElAngle += ElAngleErr;
        FOC_angle2 = hElAngle;
        Vqd.q=Torque_Flux2;
        Vqd.d=0;
        Flux_Vd = 0;
        ENCODER_M2.Angle_Compensation = 0 ;
    }
    else if(Angle_Switch2==0)
    {
        TIM2->CNT = 0;
        hElAngle += ElAngleErr;
        FOC_angle2 = hElAngle;
        Vqd.q=0;
        Vqd.d=Torque_Flux2;
        HALL2_CC_First = 2;
        ENCODER_M2.Angle_Compensation = 0 ;
//		TIM2->CNT = 0;
//	  hElAngle += ElAngleErr;
//		FOC_angle2 =  0;// hElAngle;
//		hElAngle = 0;
//    Vqd.q=0;
//
//    if(Flux_Vd < Flux_temp )
//		{
//			Flux_Vd++;
//			Vqd.d = Flux_Vd;
//		}
    }


    Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M2], Valphabeta);
    FOCVars[M2].Vqd = Vqd;
    FOCVars[M2].Iab = Iab;
    FOCVars[M2].Ialphabeta = Ialphabeta;
    FOCVars[M2].Iqd = Iqd;
    FOCVars[M2].Valphabeta = Valphabeta;
    FOCVars[M2].hElAngle = hElAngle;
    return(hCodeError);
}
/*------------------------------------------------
* @function : PLL测速
* @input    : 编码器定时器指针，测速计算指针，电机参数指针
* @output   :
* @explain  :
* @author   : wangchuan
* @date     : 2023/1/3
------------------------------------------------*/
float AxisVelocityPLLCalc(TIM_TypeDef * TIMx , pSpeedMesa * speedmeas , MotorParameters_t * motorpar)
{
    speedmeas->fAngle  = ( LL_TIM_GetCounter ( TIMx )+1)*4-1;
    if(speedmeas->fAngle > motorpar->PulseNumber*3 && speedmeas->fPreAngle <  motorpar->PulseNumber)
    {
        speedmeas->fAngleLast += motorpar->PulseNumber*4;
        speedmeas->fProportional_Term = speedmeas->fPLL_Kp*(speedmeas->fAngle - speedmeas->fAngleLast);
        speedmeas->fIntegral_Term += speedmeas->fPLL_Ki*(speedmeas->fAngle - speedmeas->fAngleLast);
        speedmeas->fSpeedRPM_Temp =  speedmeas->fProportional_Term +  speedmeas->fIntegral_Term;
        speedmeas->fAngleLast = speedmeas->fAngleLast + speedmeas->fSpeedRPM_Temp *speedmeas->fTsw;
    }
    else if(speedmeas->fPreAngle>motorpar->PulseNumber*3 && speedmeas->fAngle<motorpar->PulseNumber)
    {
        speedmeas->fAngleLast -= motorpar->PulseNumber*4;
        speedmeas->fProportional_Term = speedmeas->fPLL_Kp*(speedmeas->fAngle - speedmeas->fAngleLast);
        speedmeas->fIntegral_Term += speedmeas->fPLL_Ki*(speedmeas->fAngle - speedmeas->fAngleLast);
        speedmeas->fSpeedRPM_Temp =  speedmeas->fProportional_Term +  speedmeas->fIntegral_Term;
        speedmeas->fAngleLast = speedmeas->fAngleLast + speedmeas->fSpeedRPM_Temp *speedmeas->fTsw;
    }
    else
    {
        speedmeas->fProportional_Term = speedmeas->fPLL_Kp*(speedmeas->fAngle - speedmeas->fAngleLast);
        speedmeas->fIntegral_Term += speedmeas->fPLL_Ki*(speedmeas->fAngle - speedmeas->fAngleLast);
        speedmeas->fSpeedRPM_Temp =  speedmeas->fProportional_Term +  speedmeas->fIntegral_Term;
        speedmeas->fAngleLast = speedmeas->fAngleLast + speedmeas->fSpeedRPM_Temp *speedmeas->fTsw;
    }
    speedmeas->fSpeedCorrection = speedmeas->fSpeedRPM_Temp/speedmeas->fSpeed_CorrectionFactor; //105 单位为：0.1转
    speedmeas->fSpeedFilter = speedmeas->fKfilter*speedmeas->fSpeedFilter +speedmeas->fSpeedCorrection*(1-speedmeas->fKfilter);
    speedmeas->fSpeedRPM = speedmeas->fSpeedFilter;
    speedmeas->fPreAngle = speedmeas->fAngle;
    return   speedmeas->fSpeedRPM;


}
/*------------------------------------------------
* @function :锁相环参数初始话
* @input    :
* @output   :
* @explain  :测速
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
void AxisVelocityPLLCalcInit(void)
{
    pSpeed_Mesa[M2].fPLL_Kp = 500;
    pSpeed_Mesa[M2].fPLL_Ki = 0;
    pSpeed_Mesa[M2].fTsw = 0.0001;
    pSpeed_Mesa[M2].fKfilter = 0.1;
    pSpeed_Mesa[M2].fSpeed_CorrectionFactor = 109.35;//655
    pSpeed_Mesa[M2].fSpeedFilter = 0;

    pSpeed_Mesa[M1].fSpeedFilter = 0;
    pSpeed_Mesa[M1].fPLL_Kp = 500;
    pSpeed_Mesa[M1].fPLL_Ki = 0;
    pSpeed_Mesa[M1].fTsw = 0.0001;
    pSpeed_Mesa[M1].fKfilter = 0.1;
    pSpeed_Mesa[M1].fSpeed_CorrectionFactor = 109.35;
    pSpeed_Mesa[M1].fAngle  = ( LL_TIM_GetCounter ( TIM4 )+1)*4-1;
    pSpeed_Mesa[M1].fPreAngle = pSpeed_Mesa[M1].fAngle;
    pSpeed_Mesa[M1].fAngleLast = pSpeed_Mesa[M1].fAngle;


}
/*------------------------------------------------
* @function :安全任务
* @input    :
* @output   :
* @explain  : 500us执行一次
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak void TSK_SafetyTask(void)
{
    /* USER CODE BEGIN TSK_SafetyTask 0 */

    /* USER CODE END TSK_SafetyTask 0 */
    if (bMCBootCompleted == 1)
    {
        /* Second drive */
        TSK_SafetyTask_PWMOFF();                                                                    /*安全任务*/
        /* User conversion execution */
        RCM_ExecUserConv ();
        /* USER CODE BEGIN TSK_SafetyTask 1 */
        pCtrlPar[M1].MotorTemperature = (int32_t)NTC_GetAvTemp_C(pTemperatureSensor[M1]);           /*M1温度采集*/
        pCtrlPar[M2].MotorTemperature = (int32_t)NTC_GetAvTemp_C(pTemperatureSensor[M2]);           /*M2温度采集*/
        VBS_AvBusVoltage_V = (int32_t)VBS_GetAvBusVoltage_V(&(pBusSensorM1->_Super));               /*母线电压采集*/
    }
}
/*------------------------------------------------
* @function :各种保护检查 
* @input    :
* @output   :
* @explain  :TSK_SafetyTask中执行 500us轮询
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void TSK_SafetyTask_PWMOFF(void)
{
    uint32_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK, VBUS_TEMP_ERR_MASK2};
    BusVoltageFaultsFlag = RVBS_CalcAvVbus(pBusSensorM1);                         /*母线电压检查只需要一次*/
    for(u8 counter=0; counter<NUMBER_OF_AXES; counter++)
    {
        CodeReturn |= OverLoadCurrentCheck(counter);                              /*过流保护*/
        CodeReturn |= errMask[counter] & NTC_CalcAvTemp(pTemperatureSensor[counter]); /* check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
        CodeReturn |= PWMC_CheckOverCurrent(pwmcHandle[counter]);                 /*brake保护*/
        CodeReturn |= MotorStuckCheck(counter);                                   /*堵转检查*/
        CodeReturn |= StallCheck(counter);                                        /*失速检查*/
        CodeReturn |= DefaultPhaseCheckLinkFunction(counter);                     /*缺相检查*/
        if(VBS_AvBusVoltage_V>20)/*如果电压大于20V，为了避免开机推动机器进行霍尔检测，如果电压大于5V 霍尔应该是处于正常工作状态*/
        {
            Hall_Protect_Cnt++;                                                   /*500us一次，因为有两个电机所以速度*2*/
            if(Hall_Protect_Cnt > 10000)                                          /*2.5s后开始霍尔检测*/
            {
                Hall_Protect_Cnt = 10000;
//                CodeReturn |= Hal_Enc_Err_Check(counter);                         /*霍尔检查*/
            }
        }
        else
        {
            Hall_Protect_Cnt = 0;                                                 /*直接等于0，不进行霍尔错误检查*/
        }
        CodeReturn |= Motor_Over_Temperature(counter);                            /*温度检测*/
        if(BusVoltageFaultsFlag == MC_NO_ERROR)                                   /*过压欠压自恢复*/
        {
//            STM[counter].hFaultOccurred = STM[counter].hFaultOccurred&(~MC_OVER_VOLT);
//            STM[counter].hFaultOccurred = STM[counter].hFaultOccurred&(~MC_UNDER_VOLT);
        }
        CodeReturn |= BusVoltageFaultsFlag;
        if(_heartbeatErrorFlag >0)                                                /*断线保护检查，該標志位在生命周期文件中置位为2，两个电机所以执行两次*/
        {
            _heartbeatErrorFlag--;
            CodeReturn |= ERR_CAN_COMMUNICATION;
        }
        STM_FaultProcessing(&STM[counter], CodeReturn, ~CodeReturn);              /* Update the STM according error code */
        CodeReturn = MC_NO_ERROR;                                                 /*第一个电机使用完之后，将CodeReturn清零，独立错误会传递给第二个*/
        switch (STM_GetState(&STM[counter]))                                      /* Acts on PWM outputs in case of faults */
        {
        case FAULT_NOW:

            if (pEAC[counter] != MC_NULL)
            {
                EAC_SetRestartState( pEAC[counter], false );                      /* reset Encoder state */
            }
            PWMC_SwitchOffPWM(pwmcHandle[counter]);
            FOC_Clear(counter);
            MPM_Clear((MotorPowMeas_Handle_t*)pMPM[counter]);
            if(counter == M1)                                                     /*电机1故障 保证如果有一个电机出错两个电机都停止*/
            {
                MC_StopMotor2();
            }
            else                                                                  /*电机2故障*/
            {
                MC_StopMotor1();
            }
            break;
        case FAULT_OVER:
            PWMC_SwitchOffPWM(pwmcHandle[counter]);                               /*关闭PWM，并且关闭自动输出使能*/

            break;
        default:
            break;
        }
    }
}
/*------------------------------------------------
* @function :未使用
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak void TSK_SafetyTask_RBRK(uint8_t bMotor)
{
    uint32_t CodeReturn = MC_NO_ERROR;
    STM_FaultProcessing(&STM[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */
    switch (STM_GetState(&STM[bMotor]))
    {
    case FAULT_NOW:
        /* reset Encoder state */
        if (pEAC[bMotor] != MC_NULL)
        {
            EAC_SetRestartState( pEAC[bMotor], false );
        }
        PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
        FOC_Clear(bMotor);
        MPM_Clear((MotorPowMeas_Handle_t*)pMPM[bMotor]);
        break;
    case FAULT_OVER:
        PWMC_SwitchOffPWM(pwmcHandle[bMotor]);
        break;
    default:
        break;
    }

}

#if defined (CCMRAM_ENABLED)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section (".ccmram")))
#endif
#endif
/*------------------------------------------------
* @function :这个函数在双MC的情况下由TIMx_UP_IRQHandler调用，他允许在ADC_ISR上的FOC执行之前预留一半的PWM周期
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/

__weak void TSK_DualDriveFIFOUpdate(uint8_t Motor)
{
    FOC_array[FOC_array_tail] = Motor;
    FOC_array_tail++;
    if (FOC_array_tail == FOC_ARRAY_LENGTH)
    {
        FOC_array_tail = 0;
    }
}
/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
__weak MCI_Handle_t * GetMCI(uint8_t bMotor)
{
    MCI_Handle_t * retVal = MC_NULL;
    if (bMotor < NBR_OF_MOTORS)
    {
        retVal = oMCInterface[bMotor];
    }
    return retVal;
}

/**
  * @brief  This function returns the reference of the MCTuning relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCT_Handle_t motor control tuning handler for the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
__weak MCT_Handle_t* GetMCT(uint8_t bMotor)
{
    MCT_Handle_t* retVal = MC_NULL;
    if (bMotor < NBR_OF_MOTORS)
    {
        retVal = &MCT[bMotor];
    }
    return retVal;
}

/*------------------------------------------------
* @function :HardFault执行任务
* @input    :
* @output   :
* @explain  :由于硬件问题进入HardFault IRQ的时候调用,关断PWM
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
__weak void TSK_HardwareFaultTask(void)
{
    R3_2_SwitchOffPWM(pwmcHandle[M1]);
    STM_FaultProcessing(&STM[M1], MC_SW_ERROR, 0);
    R3_2_SwitchOffPWM(pwmcHandle[M2]);
    STM_FaultProcessing(&STM[M2], MC_SW_ERROR, 0);
}
/*------------------------------------------------
* @function :锁定电机控制的GPIO引脚，防止被意外配置
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/5
------------------------------------------------*/
__weak void mc_lock_pins (void)
{
    LL_GPIO_LockPin(M2_PWM_UL_GPIO_Port, M2_PWM_UL_Pin);
    LL_GPIO_LockPin(M2_PWM_VL_GPIO_Port, M2_PWM_VL_Pin);
    LL_GPIO_LockPin(M2_PWM_WL_GPIO_Port, M2_PWM_WL_Pin);
    LL_GPIO_LockPin(M2_PWM_UH_GPIO_Port, M2_PWM_UH_Pin);
    LL_GPIO_LockPin(M2_OCP_GPIO_Port, M2_OCP_Pin);
    LL_GPIO_LockPin(M2_PWM_VH_GPIO_Port, M2_PWM_VH_Pin);
    LL_GPIO_LockPin(M2_PWM_WH_GPIO_Port, M2_PWM_WH_Pin);
//LL_GPIO_LockPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);
//LL_GPIO_LockPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin);
//LL_GPIO_LockPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin);
    LL_GPIO_LockPin(M1_CURR_AMPL_W_GPIO_Port, M1_CURR_AMPL_W_Pin);
    LL_GPIO_LockPin(M2_CURR_AMPL_W_GPIO_Port, M2_CURR_AMPL_W_Pin);
    LL_GPIO_LockPin(M1_ENCODER_A_GPIO_Port, M1_ENCODER_A_Pin);
    LL_GPIO_LockPin(M1_ENCODER_B_GPIO_Port, M1_ENCODER_B_Pin);
    LL_GPIO_LockPin(M2_ENCODER_B_GPIO_Port, M2_ENCODER_B_Pin);
    LL_GPIO_LockPin(M2_ENCODER_A_GPIO_Port, M2_ENCODER_A_Pin);
    LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
    LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
//LL_GPIO_LockPin(M1_OCP_GPIO_Port, M1_OCP_Pin);
    LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
    LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
    LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
    LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
    LL_GPIO_LockPin(M1_DISSIPATIVE_BRK_GPIO_Port, M1_DISSIPATIVE_BRK_Pin);
    LL_GPIO_LockPin(M1_CURR_AMPL_U_GPIO_Port, M1_CURR_AMPL_U_Pin);
    LL_GPIO_LockPin(M1_CURR_AMPL_V_GPIO_Port, M1_CURR_AMPL_V_Pin);
    LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
    LL_GPIO_LockPin(M2_CURR_AMPL_U_GPIO_Port, M2_CURR_AMPL_U_Pin);
    LL_GPIO_LockPin(M2_CURR_AMPL_V_GPIO_Port, M2_CURR_AMPL_V_Pin);
}

/*------------------------------------------------
* @function :过流保护限制值设置
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
void LoadCurrentInit(void)//500us
{
    uint16_t i = 0, j = 0;
    for (i = 0; i < NUMBER_OF_AXES; i++)
    {
        loadCurr[i].curr = 0;
        for (j = 0; j < OVER_LOAD_TYPE_CNT; j++)
        {
            loadCurr[i].currCnt[j] = 0;
        }

        MotorParRatedCurr[i] = MotorParameters[i].RatedCurrent*ADC_AMPER_Ratio;/*额定电流6500ma * 0.397  397代表1A*/

        loadCurr[i].factor[0] = 2.3;	       /*2.3倍过载  14.95A*/
        loadCurr[i].factor[1] = 3.0;         /*3.0倍过载  19.5A*/
        loadCurr[i].factor[2] = 4.0;         /*4.0倍过载  26A*/
        loadCurr[i].factor[3] = 4.5;         /*4.5倍过载  29.25A*/

        loadCurr[i].cntMax[0] = 240000;		   /*120S*/
        loadCurr[i].cntMax[1] = 120000;      /*60S*/
        loadCurr[i].cntMax[2] = 4000;        /*2s*/
        loadCurr[i].cntMax[3] = 2000;        /*1s*/

        loadCurr[i].errCode[0] = ERR_CURR_13_OVER_LOAD;    /*错误代码*/
        loadCurr[i].errCode[1] = ERR_CURR_15_OVER_LOAD;
        loadCurr[i].errCode[2] = ERR_CURR_OVER_MAX;
        loadCurr[i].errCode[3] = ERR_CURR_OVER_MAX;

        loadCurr[i].cmpVal[0] = MotorParRatedCurr[i]*loadCurr[i].factor[0]; /*过流保护值*/
        loadCurr[i].cmpVal[1] = MotorParRatedCurr[i]*loadCurr[i].factor[1]; /*计算：(额定电流*ADC_AMPER_Ratio*过载倍数/397)A */
        loadCurr[i].cmpVal[2] = MotorParRatedCurr[i]*loadCurr[i].factor[2];
        loadCurr[i].cmpVal[3] = MotorParRatedCurr[i]*loadCurr[i].factor[3];
    }
}
/*------------------------------------------------
* @function :过流错误检查
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
uint16_t OverLoadCurrentCheck(uint16_t axisNum)
{
    uint16_t i = 0;
//	uint16_t ErrType[axisNum];
    {
        loadCurr[axisNum].curr = FOCVars[axisNum].Iqd.q*IQFactor;                    /*Iq*系数 给中间变量*/
        for (i = 0; i < OVER_LOAD_TYPE_CNT; i++)                                     /*过流保护阶段4个*/
        {
            if (labs(loadCurr[axisNum].curr)> loadCurr[axisNum].cmpVal[i])           /*依次判断4个阶段，如果电流超过阈值*/
            {
                loadCurr[axisNum].currCnt[i]++;                                      /*某阶段成立*/
                if (loadCurr[axisNum].currCnt[i] > loadCurr[axisNum].cntMax[i])			 /*时间条件满足*/
                {
                    loadCurr[axisNum].currCnt[i] = loadCurr[axisNum].cntMax[i]+1;    /*限制cnt最大值*/
                    if( i!=3)                                                        /*前三个阶段*/
                    {
                        loadCurr[axisNum].ctrlFault[i]= loadCurr[axisNum].errCode[i];
                        loadCurr[axisNum].ErrCode |= loadCurr[axisNum].ctrlFault[i];

                    }
                    else                                                             /*最后一个阶段*/
                    {
                        if(abs(pSTC[axisNum]->SPD->hAvrMecSpeedUnit) < 20)           /*转速小于12RPM，与实际转速存在0.6倍关系*/
                        {
                            loadCurr[axisNum].ctrlFault[i]= loadCurr[axisNum].errCode[i];
                        }
                        loadCurr[axisNum].ErrCode |= loadCurr[axisNum].ctrlFault[i];
                    }

                }
            }
            else            /*如果电流没有超过阈值*/
            {
                if(loadCurr[axisNum].currCnt[i] > 10)
                {
                    loadCurr[axisNum].currCnt[i]-- ;
                }
                else
                {
                    loadCurr[axisNum].currCnt[i] = 0;
                    loadCurr[axisNum].ctrlFault[i]= 0;
                    loadCurr[axisNum].ErrCode  &= ~(1<<(8+i));                       /*清除错误*/
                }
            }
        }
    }
    return loadCurr[axisNum].ErrCode ;
}
/*------------------------------------------------
* @function :电机1部分参数初始化
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
void MotorParametersM1_Init(void)
{
    ENCODER_M1._Super.bElToMecRatio = MotorParameters[M1].PolePairNum ;
    ENCODER_M1.TIMx->ARR = MotorParameters[M1].PulseNumber-1;
    ENCODER_M1.PulseNumber = MotorParameters[M1].PulseNumber;
    ENCODER_M1.U32MAXdivPulseNumber = UINT32_MAX / ( uint32_t )( ENCODER_M1.PulseNumber ) + 1;
    EncAlignCtrlM1.bElToMecRatio = MotorParameters[M1].PolePairNum ;
    HALL_M1._Super.bElToMecRatio = MotorParameters[M1].PolePairNum ;
    SpeednTorqCtrlM1.MaxPositiveTorque = MotorParameters[M1].MAXPhaseCurrent*ADC_AMPER_Ratio;
    SpeednTorqCtrlM1.MinNegativeTorque = -MotorParameters[M1].MAXPhaseCurrent*ADC_AMPER_Ratio;
    MotorParRatedCurr[M1] = MotorParameters[M1].RatedCurrent*ADC_AMPER_Ratio;
    PID_PosParamsM1.hUpperOutputLimit = (int16_t)MotorParameters[M1].MAXSpeed;
    PID_PosParamsM1.hLowerOutputLimit = -(int16_t)MotorParameters[M1].MAXSpeed;
    SpeednTorqCtrlM1.MaxAppPositiveMecSpeedUnit  = (int16_t)MotorParameters[M1].MAXSpeed * SPEED_UNIT/_RPM;
    SpeednTorqCtrlM1.MinAppNegativeMecSpeedUnit  = -(int16_t)MotorParameters[M1].MAXSpeed * SPEED_UNIT/_RPM;
    ENCODER_M1._Super.hMaxReliableMecSpeedUnit = (int16_t)1.15*MotorParameters[M1].MAXSpeed * SPEED_UNIT/_RPM;
//    PIDSpeedHandle_M1.wUpperIntegralLimit = (int32_t)MotorParameters[M1].MAXPhaseCurrent*ADC_AMPER_Ratio*(int32_t)SP_KIDIV ;
//    PIDSpeedHandle_M1.wLowerIntegralLimit = -(int32_t)MotorParameters[M1].MAXPhaseCurrent*ADC_AMPER_Ratio*(int32_t)SP_KIDIV ;
//    PIDSpeedHandle_M1.hUpperOutputLimit = (int16_t)MotorParameters[M1].MAXPhaseCurrent*ADC_AMPER_Ratio ;
//    PIDSpeedHandle_M1.hLowerOutputLimit = -(int16_t)MotorParameters[M1].MAXPhaseCurrent*ADC_AMPER_Ratio ;


}
/*------------------------------------------------
* @function :电机2部分参数初始化
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/4
------------------------------------------------*/
void MotorParametersM2_Init(void)
{
    ENCODER_M2._Super.bElToMecRatio = MotorParameters[M2].PolePairNum ;
    ENCODER_M2.TIMx->ARR = MotorParameters[M2].PulseNumber-1;
    ENCODER_M2.PulseNumber = MotorParameters[M2].PulseNumber;
    ENCODER_M2.U32MAXdivPulseNumber = UINT32_MAX / ( uint32_t )( ENCODER_M2.PulseNumber ) + 1;
    EncAlignCtrlM2.bElToMecRatio = MotorParameters[M2].PolePairNum ;
    SpeednTorqCtrlM2.MaxPositiveTorque = MotorParameters[M2].MAXPhaseCurrent*ADC_AMPER_Ratio;
    SpeednTorqCtrlM2.MinNegativeTorque = -MotorParameters[M2].MAXPhaseCurrent*ADC_AMPER_Ratio;
    MotorParRatedCurr[M2] = MotorParameters[M2].RatedCurrent*ADC_AMPER_Ratio;
//    PID_PosParamsM2.hUpperOutputLimit = (int16_t)MotorParameters[M2].MAXSpeed;
//    PID_PosParamsM2.hLowerOutputLimit = -(int16_t)MotorParameters[M2].MAXSpeed;
    SpeednTorqCtrlM2.MaxAppPositiveMecSpeedUnit  = (int16_t)MotorParameters[M2].MAXSpeed * SPEED_UNIT/_RPM;
    SpeednTorqCtrlM2.MinAppNegativeMecSpeedUnit  = -(int16_t)MotorParameters[M2].MAXSpeed * SPEED_UNIT/_RPM;
    ENCODER_M2._Super.hMaxReliableMecSpeedUnit = (int16_t)1.15*MotorParameters[M2].MAXSpeed * SPEED_UNIT/_RPM;
//    PIDSpeedHandle_M2.wUpperIntegralLimit = (int32_t)MotorParameters[M2].MAXPhaseCurrent*ADC_AMPER_Ratio*(int32_t)SP_KIDIV2 ;
//    PIDSpeedHandle_M2.wLowerIntegralLimit = -(int32_t)MotorParameters[M2].MAXPhaseCurrent*ADC_AMPER_Ratio*(int32_t)SP_KIDIV2 ;
//    PIDSpeedHandle_M2.hUpperOutputLimit = (int16_t)MotorParameters[M2].MAXPhaseCurrent*ADC_AMPER_Ratio ;
//    PIDSpeedHandle_M2.hLowerOutputLimit = -(int16_t)MotorParameters[M2].MAXPhaseCurrent*ADC_AMPER_Ratio ;
}
