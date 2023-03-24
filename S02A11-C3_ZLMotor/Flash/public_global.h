#ifndef __PUBLIC_GLOBAL_H__
#define __PUBLIC_GLOBAL_H__

#include "public_h.h"

/*-----------------------------------�ṹ��-------------------------------------*/
/*-----����ӡ���ݽṹ��-----*/
typedef struct
{
		struct
		{
			double aim;
			double feed;
			double error;
		}print_speed;			 //�ٶȴ�ӡ
		 struct
		{
			double aim;
			double feed;
			double error;
		}print_iq;			     //Iq��ӡ 
		struct
		{
			double aim;
			double feed;
			double error;
		}print_position;		 //λ�ô�ӡ
		 struct
		{
			double aim;
			double feed;
			double error;
		}print_id;				 //Id��ӡ
}print_output;
			
/*-----SVPWM�ṹ��-----*/
typedef struct
{                                      
	int16_t A;
	int16_t B;
	int16_t C;
	int16_t N;
	float K;
	float M;
	int32_t X;
	int32_t Y;
	int32_t Z;
	int32_t T1;
	int32_t T2;
	int32_t T0;
	int32_t A_PWM;
	int32_t B_PWM;
	int32_t C_PWM;
} SVPWM;
/*-----�������ṹ��-----*/
typedef struct
{
	int32_t QEP;
	int16_t angle;
	float SPEED;
	int32_t POSITION;
	float I_u;
	float I_v;
	float I_alpha;
	float I_beta;
	float I_q;
	float I_d;
	float Uref_alpha;
	float Uref_beta;
	float U_q;
	float U_d;
} Feedback;
/*-----PID�������ṹ��-----*/
typedef struct
{
	int Instance;
	long	Kp;
	long  Ki;
	long  Kd;
	float Error;                 //���
	float iError;                //���Ļ���
	float dError;                //����΢��
	float LastError;             //�ϴε����
	int32_t Lower_Limit_Output;  //�����С����
	int32_t Upper_Limit_Output;  //����������
	int32_t Upper_Limit_Integral;//��������С����
	int32_t Lower_Limit_Integral;//�������������	
	float Proportional_Term;
	float Integral_Term; 
	float Derivative_Term;
	float Output;
	float LastOutput;
} PID;
/*-----��ģ�����ṹ��-----*/
typedef struct
{
  float Error;              //���
  float DError;             //����΢��
  float PreError;           //�ϴ����
  float Upper_Limit_Output;
  float Lower_Limit_Output;
  float DReference;         //�����΢��
  float PreReference;       //�ϴ�����
  float	DDReference;        //����Ķ���΢��
  float PreDReference;      //�ϴ������΢��
  float	cSMC;           
  float kSMC;
  float ySMC;
  float s;
  float Output;
} SMC;
/*-----���Ǻ����ṹ��-----*/
typedef struct
{
  float hCos;
  float hSin;
} Trig_Components;

/*-----������˹�����洢�ṹ��-----*/
//�������Խṹ��
typedef struct
{
	unsigned			bReadOnly : 1; 		//��д����
	unsigned			bOKInMotion: 1; 	//�Ƿ������˶����޸�
	unsigned			bOKMotorOn : 1; 	//�Ƿ�����ʹ��ʱ�޸�
	unsigned			bIsArray : 1;   	//�Ƿ��������� 
	unsigned			bSaveToFlash : 1; 	//�Ƿ�������Flsash
	unsigned			bAxisRelated: 1; 	//�Ƿ������
	unsigned			bReserved1 : 2;

}PARAMETER_ATTRIBUTES;
//������˹�������ݲ�����ṹ��
//typedef struct 
//{
//	short								sParID;                                                                                  
//	short								sAddress; 		//ͨѶ������ַ	
//	PARAMETER_ATTRIBUTES		        stAttributes;   //�������
//	short								sMaxArrayIndex; //�����������ֵ
//	long								lMinValue; 	    //��Сȡֵ 
//	long 								lMaxValue; 		//���ȡֵ��Χ
//	long								lDefaultValue;  //Ĭ��ֵ
//	long*								lpParam; 		//��Ӧ����ӳ��

//}PARAMETER_TABLE;

typedef struct 
{
	long funcEn;
	long funcState;
	u8 funcType;
}DEBUG_HANDLE;
//��λ�����Թ���״̬��
typedef struct 
{
	long Oscp;  //ʾ�����߼�ִ��״̬��
	long Upload;//ʾ�������ݰ��ϴ�״̬��
	long Indep_WR;//����������д״̬
	long Interface;//GUI���ܽ������Upload
	long InterType;//GUI���ܽ�������
	long ParaUpload;//Atlas para upload
  long ParaDownload;//Atlas para download
}DEBUG_STATE;
//ͨ��Э��ṹ��
typedef struct 
{
	long Sci_BaudRate[3];		//������3����λ[9600 19200 115200]
	long Sci_Resp_Cmd;			//�����ظ�CMD�Ĵ���
	u16 Sci_RsCheckDate;     //У������
	u16 Sci_TsCheckDate;
	long Sci_ErrClr_En;
	short Para_addr;
	long Sci_state;
	char Sci_SxBuffer[15];
	char Sci_RxBuffer[15];
  long Obj_data;
	
	u8 Sci_Monitor_En;
	u32 Sci_Timing;
}SCICOM;
typedef struct 
{
  uint16_t  pDateCheck;
  uint16_t  pArray[10];
  uint16_t  pDateLen;
}_CRC;

/*--------------------------------�ⲿ��������----------------------------------*/
/*--------ϵͳ��ʱ����----------*/
extern int8_t  tim_cnt_1ms;
extern float tick_1ms;
extern u16 T1_CC4_Triger;
/*--------��ʱ�������----------*/
extern u32 systic_cnt;
extern u8 time_out_flag;            
extern u32 time_out_cnt;   
extern u32 time_pos[2];
extern u8 function_comp_flag[2];
extern u16 System_error_table[ERROR_NUM];
extern u32 time_used_note;
extern u16 function_handler;
/*--------ϵͳ���ܱ���----------*/
extern  int8_t Motor_state;
extern long Motor_mode;
extern long Pre_Motor_mode;
extern int8_t Mode_switch_flag;
extern int8_t Motor_brake;
extern int8_t Motor_brake_cnt;
extern int8_t Motor_direction;
extern u8  bSector;
extern uint16_t Printf_speed;		//Max:10kHz
extern u16 TIM_CC4_Triger;
/*-----------------T����������----------------*/
#define T_Mesure   0  
#define M_Mesure   1
extern int32_t tim5_cnt,Capture_tim5_t,Capture_tim2_t;
extern unsigned int tim5_inter_cnt,tim2_inter_cnt;
extern unsigned int STA_Capture_tim5,STA_Capture_tim2;
extern float SPEED_Former,SPEED_Later;				
extern int32_t SPEED_Direction;
extern unsigned CapEdge_flag;
extern float Sampling_period;												
extern unsigned int M1_cnt,M2_cnt,M1_complete_flag;
extern int32_t CaptureNumber_last;
extern int32_t CaptureNumber_former[2];//��������ǰ���ٶȽ���
extern STM_Handle_t STM[NBR_OF_MOTORS];
extern MotorParameters_t MotorParameters[NBR_OF_MOTORS];

extern s32 wTimePhA, wTimePhB, wTimePhC;
extern u16  hTimePhA, hTimePhB, hTimePhC;
extern u16 Uart_N_100us;
extern u8 uart_counter;
//extern u8 Uart_N_100us_Flag;
//extern u8 TxBuffer[TxBufferSize];      //���ͻ�����
/*--------����������----------*/
extern  PID PID_Flux;
extern  PID PID_Torque;
extern  PID PID_Speed;
extern  PID PID_Position;
extern float Torque_Reference;
extern float Flux_Reference;
extern long Speed_Reference;
extern long Position_Reference;
extern long Speed_refer;
/*--------��ģ����----------*/
extern  SMC SMC_Struct;
extern	float eSMC;
extern	float cSMC;           
extern	float kSMC;
/*--------��������������----------*/
extern int32_t  CaptureNumber[2];
extern int32_t OverflowCount; 
extern float Pre_Feedback_speed;
extern float Last_Feedback_speed;
extern float FILTER_Ratio;
extern u8 		PreDirFlag,LastDirFlag;
/*--------������������----------*/
#define Hall_DisMeasure 1
extern long  Ha11_Distance_P[6] ;             //��������
extern long  Ha11_Angle_P[6];                 //�����Ƕ�
extern long  Ha11_Distance_N[6] ;             //��������
extern long  Ha11_Angle_N[6];                 //�����Ƕ�
extern int16_t  Ha11_Distance[6];
extern long  zero_position;           //�����λ�ͱ�������λ֮�����
extern int16_t  Disposable_zero_position;//����ִ�б���
extern short		Zero_DetectState_Flag;
extern long			glHallStatData[];
/*--------ADC��������----------*/
extern u16 ADC_Init_table[ADC_Channel_cnt][ADC_Init_N];
extern u16 ADC_Init_offset_sum[ADC_Channel_cnt];
extern u16 ADC_Init_offset_avg[ADC_Channel_cnt];
extern int32_t  ADC_ConvertedValueLocal[ADC_Channel_cnt];
extern uint32_t  ADC_ConvertedValue[ADC_Channel_cnt];
extern float CURRENT_temp_A;
extern float CURRENT_temp_B;
extern float CURRENT_temp_C;
extern uint32_t DMA_Transfer_Complete_Count;
/*-----�ٶ��˲�����[ƽ����]-----*/
extern int16_t  filter_buf[FILTER_N];
extern int32_t  filter_sum;
extern int32_t  Speed_temp;
/*--------FOC���ݱ���----------*/
extern Feedback    Feedback_Data;
extern SVPWM       SVPWM_Data;   
extern Trig_Components Vector_Components;
extern int16_t  Udc;              //Bus Voltage
extern float    Uref1,Uref2,Uref3;
extern float cos_da;
extern float sin_da;
extern float  Angle_Bias;
extern int32_t TIM3_CNT_COUNTER;
/*---------�ٶ�/λ�ñ仯���ƿ��Ʊ���---------*/
extern __IO int position_stable;
extern float Limit_OutChangeRate;

extern int8_t 						Hall_state;
extern int16_t  					Electrical_Angle;	
extern long								glDebugTestD[];  //for test data 
extern short   						gsMachineStat[];
//extern long								glInTargetTol[]; 
//extern long								glInTargetTime[]; 
extern long								glInTargetStat[];
extern long 							loopCalcMask[];
extern long								glPreControlMode[];
extern float       				gfVelHistorySum[];
extern long	     					guOldestVelIndex[];
extern long        				glIsrTicker[];
extern long				glWaitSetBrakeOn[];
extern long				glBrkSpeed[];
extern long				glBrkOnDly[];
extern long				glBrkOffDly[];
extern short      gsBrkState[];
extern short			gsCorrectEncAngFlag;
extern short			gsCorrectEncAngFlag2;
extern long				glPreMotorHallSector;
extern short			gsRuningDirFlag;
extern short			gsSetPosFlag;

/*---------ʾ������ر���---------*/
extern long gssiRecLength;  
extern long gssiRecLength_pre;
extern float gfRecTime;
extern long gssiRecGap;      
extern long gssiRecGap_pre;
extern long gssiRecTrigSrc;       
extern long gssiRecTrigval;        
extern long gssiRecTrigPos;        
extern long gssiRecTrigType;       
extern long gssiRecData[4000];     
extern long gssiRecStart;          
extern long gssiRecStop;           
extern long gssiRecTrigForce;    
extern long PreUpdate[9];  
extern long testAdd;
extern Uint16 gsEncConnected;
extern Uint16 gsEncConnectSts;
extern int16  	gsABZDisconnectCnt[][3];
extern long		glIndexPos[];
extern long 	glPosLimitSetReq[];
extern short	gsEncoder_Z[];
extern short	gsEncoder_Z_Pre[];
extern long		glEncZTimeCnt[];
extern long		Zphase_position;
extern short	Zero_DetectionState[];
extern long   glJerkTemp[];
extern Uint16 guSetSmoothShapingBufferReq[];
extern long		glSave[];
extern u8 		SVpwm_switch;
extern long		glElecRes[];

extern int 		AbsAngle_test;


extern u16 Record_cnt;
extern u16 Record_num;
extern u8 Start_Record_flag;    
extern u8 Stop_Record_flag;  
extern long DebugErr_reg;
extern long OspRecID[4]; 
extern long OspRecSubID[4];
extern long TriggerID;
extern long TriggerSubID;
extern long Trigger_mode;
extern u8 Trigger_time;
extern u8 Trigger_mode_flag;

extern float CURR_SAMP_CONDUCT;     //�����絼
extern float CURR_SAMP_AMP;        //�Ŵ����ĵ���
/*---------������˹�����洢��ر���---------*/
extern const PARAMETER_TABLE  PARAMETER[];
extern PARAMETER_TABLE Maxon_Parameter[];
/*---------����(ʾ����)�����洢��ر���---------*/
extern long RecDATA[DebugNum];
extern long RecID[DebugNum];
extern long RecSubID[DebugSubNum];
extern long RecNum;
extern u8 UserCmd_Type;
extern DEBUG_STATE STATE;
extern DEBUG_HANDLE pDebug;
extern u8 pDebug_En;
extern u8 pInterface_En;
extern long FlashArray[];

extern long pTest[30];
extern u8 DrvBitArray[11];  // drv8323��ȡ������bit���

/*---------������˹ͨ��Э����ر���---------*/
extern SCICOM RsCOM;
extern u8 Speed_Switch ;
extern u32 Cnt_100us;                   //100us����
extern u16 Cnt_100us_N;                 //N��100us
//MAXON
extern _CRC MaxonCrc;
/*--------------------------------�ⲿ��������----------------------------------*/
/*--------��ʱ�����----------*/
extern TIM_HandleTypeDef TIM2_Handler; 
extern TIM_HandleTypeDef htimx_Encoder;
extern TIM_HandleTypeDef TIM5_Handler; 
/*--------FOC�㷨����----------*/
extern void  Get_Currents(void);
extern void Clark(float I_u,float I_v);
extern void Park(float I_alpha,float I_beta,float w);
extern void Clark_Park(float I_u,float I_v,float w);
extern void Park_n(float U_d,float U_q);
extern Trig_Components Trig_Functions(int16_t);
extern void  Svpwm_CalcDutyCycles(void);
extern void  FOC_Cal(void);
/*--------PID����----------*/
extern void  PID_Init (PID *PID_Torque,PID *PID_Flux,PID *PID_Speed,PID *PID_Position);
extern float PID_Regulator(float Reference, float PresentFeedback, PID *PID_Struct);
/*--------��ģ�㷨����----------*/
extern float SMC_Position(float Reference, float PresentFeedback,float Speed);
extern float SMC_Speed(float Reference, float Feedback);
/*--------�ٶ��˲�����----------*/
extern float Speed_Filter(float speed);
extern float OneRC_filter(float PreData,float LastData,float k_filter);
extern float OneRC_dynamic_filter(float PreData,float LastData,float k_filter);
/*--------�ٶ�/λ�ÿ��ƺ���----------*/
extern int32_t Position_Ref_SET(float a,float v,float P,float t);
extern int32_t Speed_Ref_SET(float a,float v,float tick);
extern int32_t Speed_brake(float a,float v,float t);
/*--------��ʱ������----------*/
extern u8 Time_out(u16 function,u32 time_medium,u32 time_heavy,u8 flag);
extern u32 Time_out_error(u8 PreemptPriority,u8 SubPriority);
extern void Time_out_error_query(u16 *function);
/*--------��������----------*/
extern uint16_t CalcFieldCRC(uint16_t* pDataArray, uint16_t ArrayLength);
extern int32_t in32abs(int32_t a);  //����ֵ����[���ͣ�int32_t]
extern float f_abs(float a);         //����ֵ����[���ͣ�float]
extern short Atlas_GetID_FroAddr(short addr);
extern short Maxon_GetID_FroAddr(short addr);
extern void BusVoltage_calculate(void);

//extern certen outside Funcs add
extern  void 	MotorOnRequestHandler(void);
extern  void 	GetEncPosition(void);
extern  short ServoDrvStateMachine(short , short );
extern 	short Get_CurrentOpFlag(long);
extern  void 	PosCmd(void);
extern  void 	VelCmd(void);
extern  void 	TorCmd(void);
extern  void 	AxisVelocityCalc(void);
extern  void 	AxisVelocityPLLCalc(void);
extern  void 	MotionReferenceSet(short );
extern  void  PositionModeCurve(short );
extern  void  VelocityModeCurve(short);
extern  void  TorqueModeCurve(short);
extern  void  OperationModeSwitch(short );
extern  void  CtrlVariablesReset(short);
extern  void 	VelocityThresholdCheck(short);
extern  void 	ControlErrorMonitor(short);
extern  void  InitIdentity(void);
//extern  void  PositionProfileCalc(short );
extern  float DivSqrt(float );
extern void ADVANCED_TIM1_Init(void);
extern void debud_uart(void);
extern void SVPWM_3ShuntCalcDutyCycles (void);
extern void InitParams(void);
extern void homingProcess(void);
extern void LimitationMotionHandle(short );
extern void SetSmoothShapingBuff(short);
extern void DigitOutputProcess(void);
#endif
/*------------------------------------------------------------------------------*/

