#ifndef __BSP_APP_FUNCTION_H
#define __BSP_APP_FUNCTION_H
#include "main.h"
#include "public_global.h"
#include "ds402.h"
typedef struct LOOP_TEST_Ttag
{
	long				swEn[2];
	long				InjectType[2];
	long				InjectPoint[2];
	long				InjectCurrAmp[2];
	long				InjectVelAmp[2];
	long				InjectPosAmp[2];
	long				InjectFreq[2];
	long				InjectedValue[2];

} LOOP_TEST_T;
typedef struct SEVERO_CONTORL_PAR_Ttag
{
    int32_t LastSetVelMotor;             /*上一次给定速度*/
    int32_t SetVelMotor;                 /*给定速度*/
    int32_t Vel_PLL_Motor;               /*实际速度*/
	  int32_t _Vel_PLL_Motor;              /*实际速度取反值，为了兼容中菱电机*/
    int32_t SetTorqueMotor;              /*给定力矩*/
	  int32_t SetPulseMotor ;              /*给定参考位置*/
    int32_t M1M2_VelSet;                 /*同步模式下的两个电机的速度高16位代表M1电机低16位代表M2电机转速*/
    int32_t M1M2_VelRel;                 /*同步模式下的两个电机的实际转速*/
    int32_t torqueLimit;                 /*力矩限制最大值*/
	  int32_t MotorTemperature;            /*电机实际温度*/
    uint32_t hDurationms;	               /*加速度减速度持续时间*/
    uint32_t quick_stop_decel;           /*急停减速度*/
	  
} SEVERO_CONTORL_PAR_T;
extern s16 HALL_CH1_ValueDelta1;
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDPosCtrl[NBR_OF_MOTORS];
extern s16 Hall_AvElAngle1CNT ;                                        /*M1极对数计数*/
extern s32 Hall_AvElAngle1Sum;                                        /*M1一圈电角度总和*/                                          
extern s16 Hall_AvElAngle2CNT ;                                       /*M2一圈的平均电角度*/
extern s32 Hall_AvElAngle2Sum ;                                        /*M2一圈电角度总和*/
extern uint8_t HALL_CC_First,HALL2_CC_First,HALL_CC_First11;
extern s16 Angle_Switch2,Angle_Switch;
extern u16 HallStudyCNT,HallStudyCNT2,Hall_EAC_Ticks;
extern u8 HallStudyFlag1,HallStudyFlag2; 
extern uint32_t led_1ms_cnt;
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern PosCtrl_Handle_t *pPosCtrl[NBR_OF_MOTORS];
extern MotorParameters_t MotorParameters[NBR_OF_MOTORS];
extern u8 PendingOpcode_handleWaitStatus[NBR_OF_MOTORS];
extern uint16_t CNT_1MS[MAX_AXES];     
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
extern SEVERO_CONTORL_PAR_T pCtrlPar[NUMBER_OF_AXES];
extern SEVERO_CONTORL_PAR_T pCtrlPar_M1;
extern SEVERO_CONTORL_PAR_T pCtrlPar_M2;
extern u8 relative_location_flag[MAX_AXES];
extern u8 absolute_location_flag[MAX_AXES];
extern u16 Sync_Async_Control;
extern long RelativeLocation[MAX_AXES];
extern uint32_t Missing_phase_error_code[2];
extern uint8_t PositionModeFlag1,PositionModeFlag2;
extern LOOP_TEST_T   loopTestPar;
extern uint16_t BrakeFiltersCnt[2];
extern u8 Damping_Mode_ON_OFF;
extern int16_t HAL_Init_Electrical_Angle,HAL_Init_Electrical_Angle2;
extern s16 HALL_CH1_ValueOffset,HALL_CH2_ValueOffset;
#define	INJECT_TYPE_NONE						0
#define	INJECT_TYPE_SIN_DIRECT			1
#define	INJECT_TYPE_SQUARE_DIRECT		2
#define	INJECT_TYPE_SIN_ADD					3
#define INJECT_TYPE_LAST						3

#define	INJECT_POINT_CURRREF				0
#define	INJECT_POINT_VELREF					1
#define	INJECT_POINT_POSREF					2
#define	INJECT_POINT_POS						3
#define INJECT_POINT_LAST						3


#define NEGATIVE          (s8)-1
#define POSITIVE          (s8)1
#define NEGATIVE_SWAP     (s8)-2
#define POSITIVE_SWAP     (s8)2
#define ERROR1             (s8)3
#define MISSTEP           (s8)-3

#define STOPISON  0
#define STOPISOFF 1
#define LED_ON		1
#define	LED_OFF		0
#define LED_LOW_SPEED 		40
#define LED_HIGH_SPEED		20

#define DAMPING_OFF  0
#define DAMPING_ON  1
#define STATE_DAMPING_IS_ON 2
#define STATE_DAMPING_IS_OFF 3
#define TURN_ON_THREE_LOW  0
#define TURN_OFF_THREE_LOW 0x00002A00

#define RED_LED1_ON   HAL_GPIO_WritePin(Fault_1_GPIO_Port,Fault_1_Pin,GPIO_PIN_RESET) 
#define RED_LED1_OFF  HAL_GPIO_WritePin(Fault_1_GPIO_Port,Fault_1_Pin,GPIO_PIN_SET) 
#define RED_LED1_TOGGLE HAL_GPIO_TogglePin(Fault_1_GPIO_Port,Fault_1_Pin);

#define RED_LED2_ON   HAL_GPIO_WritePin(Fault_2_GPIO_Port,Fault_2_Pin,GPIO_PIN_RESET) 
#define RED_LED2_OFF  HAL_GPIO_WritePin(Fault_2_GPIO_Port,Fault_2_Pin,GPIO_PIN_SET) 
#define RED_LED2_TOGGLE HAL_GPIO_TogglePin(Fault_2_GPIO_Port,Fault_2_Pin);

#define GREEN_LED_ON  HAL_GPIO_WritePin(Normal_GPIO_Port,Normal_Pin,GPIO_PIN_RESET) 
#define GREEN_LED_OFF HAL_GPIO_WritePin(Normal_GPIO_Port,Normal_Pin,GPIO_PIN_SET) 
#define GREEN_LED_TOGGLE HAL_GPIO_TogglePin(Normal_GPIO_Port,Normal_Pin);


void Independent_Motor_Control(void);
void MC_SetFault(u32 hFault_type);
void DisplayErrorLed_Handle(void);
void Display_error(void);
void LEDSet(u8 led1,u8 led2,u8 led_time,u8 led_mode,u8 DispCounterTemp);
void ReferenceInjection( \
						long	*injectPoint, \
						long	*injectType, \
						float	*injectPhase, \
						long	*injectValue, \
						long	*injectFreq, \
						long	*injectAmp, \
						long	*injectInit, \
						int32_t	*refOut \
					  );
void ReferenceInjectionLoopPar_Init(void);
void TorqueLimit(u8 axes);
uint32_t MotorStuckCheck(u8 axes);
uint32_t DefaultPhaseCheck(u8 axes);
uint32_t StallCheck(u8 axes);
void BubbleSort(float a[], float n);
void swap(float *x,float *y);
uint32_t DefaultPhaseCheckLinkFunction(u8 axes);
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver,u32 hardware_version);
void HardVersion_Init(MCU_Version *pVersion,u8 funtype,u8 vol,u8 cur_max,u8 update_ver);
void SpeedRelMerge_two_to_one(SEVERO_CONTORL_PAR_T *parM1,SEVERO_CONTORL_PAR_T *parM2);
void SpeedSetMerge_two_to_one(SEVERO_CONTORL_PAR_T *parM1,SEVERO_CONTORL_PAR_T *parM2,uint8_t mode);
uint32_t Hal_Enc_Err_Check(u8 axes);
uint32_t Motor_Over_Temperature(u8 axes);
void Emrgency_Stop(void);
uint8_t Motor_EnableOrDisable(Drive_Data *pDriver_Data);
void HALL1_Init_Electrical_Angle( void );
void HALL2_Init_Electrical_Angle( void );
u8 HALL_GetPhase2(void);
u8 HALL_GetPhase1(void);
long l_abs(long a);
int32_t in32abs(int32_t a);
void Clear_Brake_Filters_Cnt(void);
void Damping_Of_Motor(void);
void Auto_Start_function(void);
void M1M2_Initial_Position(void);
#endif

