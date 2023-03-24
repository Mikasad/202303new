#ifndef __PUBLIC_GLOBAL_H__
#define __PUBLIC_GLOBAL_H__

#include "public_h.h"

/*-----------------------------------结构体-------------------------------------*/
/*-----待打印数据结构体-----*/
typedef struct
{
		struct
		{
			double aim;
			double feed;
			double error;
		}print_speed;			 //速度打印
		 struct
		{
			double aim;
			double feed;
			double error;
		}print_iq;			     //Iq打印 
		struct
		{
			double aim;
			double feed;
			double error;
		}print_position;		 //位置打印
		 struct
		{
			double aim;
			double feed;
			double error;
		}print_id;				 //Id打印
}print_output;
			
/*-----SVPWM结构体-----*/
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
/*-----反馈量结构体-----*/
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
/*-----PID控制器结构体-----*/
typedef struct
{
	int Instance;
	long	Kp;
	long  Ki;
	long  Kd;
	float Error;                 //误差
	float iError;                //误差的积分
	float dError;                //误差的微分
	float LastError;             //上次的误差
	int32_t Lower_Limit_Output;  //输出最小限制
	int32_t Upper_Limit_Output;  //输出最大限制
	int32_t Upper_Limit_Integral;//积分项最小限制
	int32_t Lower_Limit_Integral;//积分项最大限制	
	float Proportional_Term;
	float Integral_Term; 
	float Derivative_Term;
	float Output;
	float LastOutput;
} PID;
/*-----滑模参数结构体-----*/
typedef struct
{
  float Error;              //误差
  float DError;             //误差的微分
  float PreError;           //上次误差
  float Upper_Limit_Output;
  float Lower_Limit_Output;
  float DReference;         //输入的微分
  float PreReference;       //上次输入
  float	DDReference;        //输入的二阶微分
  float PreDReference;      //上次输入的微分
  float	cSMC;           
  float kSMC;
  float ySMC;
  float s;
  float Output;
} SMC;
/*-----三角函数结构体-----*/
typedef struct
{
  float hCos;
  float hSin;
} Trig_Components;

/*-----阿特拉斯参数存储结构体-----*/
//参数属性结构体
typedef struct
{
	unsigned			bReadOnly : 1; 		//读写属性
	unsigned			bOKInMotion: 1; 	//是否允许运动中修改
	unsigned			bOKMotorOn : 1; 	//是否允许使能时修改
	unsigned			bIsArray : 1;   	//是否数组类型 
	unsigned			bSaveToFlash : 1; 	//是否允许保存Flsash
	unsigned			bAxisRelated: 1; 	//是否轴相关
	unsigned			bReserved1 : 2;

}PARAMETER_ATTRIBUTES;
//阿特拉斯驱动数据参数表结构体
//typedef struct 
//{
//	short								sParID;                                                                                  
//	short								sAddress; 		//通讯参数地址	
//	PARAMETER_ATTRIBUTES		        stAttributes;   //相关属性
//	short								sMaxArrayIndex; //数组索引最大值
//	long								lMinValue; 	    //最小取值 
//	long 								lMaxValue; 		//最大取值范围
//	long								lDefaultValue;  //默认值
//	long*								lpParam; 		//对应变量映射

//}PARAMETER_TABLE;

typedef struct 
{
	long funcEn;
	long funcState;
	u8 funcType;
}DEBUG_HANDLE;
//上位机调试功能状态量
typedef struct 
{
	long Oscp;  //示波器逻辑执行状态量
	long Upload;//示波器数据包上传状态量
	long Indep_WR;//独立参数读写状态
	long Interface;//GUI功能界面参数Upload
	long InterType;//GUI功能界面类型
	long ParaUpload;//Atlas para upload
  long ParaDownload;//Atlas para download
}DEBUG_STATE;
//通信协议结构体
typedef struct 
{
	long Sci_BaudRate[3];		//波特率3个挡位[9600 19200 115200]
	long Sci_Resp_Cmd;			//本机回复CMD寄存器
	u16 Sci_RsCheckDate;     //校验数据
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

/*--------------------------------外部变量声明----------------------------------*/
/*--------系统定时变量----------*/
extern int8_t  tim_cnt_1ms;
extern float tick_1ms;
extern u16 T1_CC4_Triger;
/*--------超时处理变量----------*/
extern u32 systic_cnt;
extern u8 time_out_flag;            
extern u32 time_out_cnt;   
extern u32 time_pos[2];
extern u8 function_comp_flag[2];
extern u16 System_error_table[ERROR_NUM];
extern u32 time_used_note;
extern u16 function_handler;
/*--------系统功能变量----------*/
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
/*-----------------T法测速数据----------------*/
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
extern int32_t CaptureNumber_former[2];//控制周期前的速度结算
extern STM_Handle_t STM[NBR_OF_MOTORS];
extern MotorParameters_t MotorParameters[NBR_OF_MOTORS];

extern s32 wTimePhA, wTimePhB, wTimePhC;
extern u16  hTimePhA, hTimePhB, hTimePhC;
extern u16 Uart_N_100us;
extern u8 uart_counter;
//extern u8 Uart_N_100us_Flag;
//extern u8 TxBuffer[TxBufferSize];      //发送缓冲区
/*--------控制器参数----------*/
extern  PID PID_Flux;
extern  PID PID_Torque;
extern  PID PID_Speed;
extern  PID PID_Position;
extern float Torque_Reference;
extern float Flux_Reference;
extern long Speed_Reference;
extern long Position_Reference;
extern long Speed_refer;
/*--------滑模参数----------*/
extern  SMC SMC_Struct;
extern	float eSMC;
extern	float cSMC;           
extern	float kSMC;
/*--------编码器采样数据----------*/
extern int32_t  CaptureNumber[2];
extern int32_t OverflowCount; 
extern float Pre_Feedback_speed;
extern float Last_Feedback_speed;
extern float FILTER_Ratio;
extern u8 		PreDirFlag,LastDirFlag;
/*--------霍尔采样数据----------*/
#define Hall_DisMeasure 1
extern long  Ha11_Distance_P[6] ;             //霍尔距离
extern long  Ha11_Angle_P[6];                 //霍尔角度
extern long  Ha11_Distance_N[6] ;             //霍尔距离
extern long  Ha11_Angle_N[6];                 //霍尔角度
extern int16_t  Ha11_Distance[6];
extern long  zero_position;           //电机零位和编码器零位之间距离
extern int16_t  Disposable_zero_position;//单次执行变量
extern short		Zero_DetectState_Flag;
extern long			glHallStatData[];
/*--------ADC采样数据----------*/
extern u16 ADC_Init_table[ADC_Channel_cnt][ADC_Init_N];
extern u16 ADC_Init_offset_sum[ADC_Channel_cnt];
extern u16 ADC_Init_offset_avg[ADC_Channel_cnt];
extern int32_t  ADC_ConvertedValueLocal[ADC_Channel_cnt];
extern uint32_t  ADC_ConvertedValue[ADC_Channel_cnt];
extern float CURRENT_temp_A;
extern float CURRENT_temp_B;
extern float CURRENT_temp_C;
extern uint32_t DMA_Transfer_Complete_Count;
/*-----速度滤波变量[平均法]-----*/
extern int16_t  filter_buf[FILTER_N];
extern int32_t  filter_sum;
extern int32_t  Speed_temp;
/*--------FOC数据变量----------*/
extern Feedback    Feedback_Data;
extern SVPWM       SVPWM_Data;   
extern Trig_Components Vector_Components;
extern int16_t  Udc;              //Bus Voltage
extern float    Uref1,Uref2,Uref3;
extern float cos_da;
extern float sin_da;
extern float  Angle_Bias;
extern int32_t TIM3_CNT_COUNTER;
/*---------速度/位置变化趋势控制变量---------*/
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

/*---------示波器相关变量---------*/
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

extern float CURR_SAMP_CONDUCT;     //采样电导
extern float CURR_SAMP_AMP;        //放大倍数的倒数
/*---------阿特拉斯参数存储相关变量---------*/
extern const PARAMETER_TABLE  PARAMETER[];
extern PARAMETER_TABLE Maxon_Parameter[];
/*---------调试(示波器)参数存储相关变量---------*/
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
extern u8 DrvBitArray[11];  // drv8323读取的数据bit存放

/*---------阿特拉斯通信协议相关变量---------*/
extern SCICOM RsCOM;
extern u8 Speed_Switch ;
extern u32 Cnt_100us;                   //100us计数
extern u16 Cnt_100us_N;                 //N次100us
//MAXON
extern _CRC MaxonCrc;
/*--------------------------------外部函数声明----------------------------------*/
/*--------定时器句柄----------*/
extern TIM_HandleTypeDef TIM2_Handler; 
extern TIM_HandleTypeDef htimx_Encoder;
extern TIM_HandleTypeDef TIM5_Handler; 
/*--------FOC算法函数----------*/
extern void  Get_Currents(void);
extern void Clark(float I_u,float I_v);
extern void Park(float I_alpha,float I_beta,float w);
extern void Clark_Park(float I_u,float I_v,float w);
extern void Park_n(float U_d,float U_q);
extern Trig_Components Trig_Functions(int16_t);
extern void  Svpwm_CalcDutyCycles(void);
extern void  FOC_Cal(void);
/*--------PID函数----------*/
extern void  PID_Init (PID *PID_Torque,PID *PID_Flux,PID *PID_Speed,PID *PID_Position);
extern float PID_Regulator(float Reference, float PresentFeedback, PID *PID_Struct);
/*--------滑模算法函数----------*/
extern float SMC_Position(float Reference, float PresentFeedback,float Speed);
extern float SMC_Speed(float Reference, float Feedback);
/*--------速度滤波函数----------*/
extern float Speed_Filter(float speed);
extern float OneRC_filter(float PreData,float LastData,float k_filter);
extern float OneRC_dynamic_filter(float PreData,float LastData,float k_filter);
/*--------速度/位置控制函数----------*/
extern int32_t Position_Ref_SET(float a,float v,float P,float t);
extern int32_t Speed_Ref_SET(float a,float v,float tick);
extern int32_t Speed_brake(float a,float v,float t);
/*--------超时处理函数----------*/
extern u8 Time_out(u16 function,u32 time_medium,u32 time_heavy,u8 flag);
extern u32 Time_out_error(u8 PreemptPriority,u8 SubPriority);
extern void Time_out_error_query(u16 *function);
/*--------公共函数----------*/
extern uint16_t CalcFieldCRC(uint16_t* pDataArray, uint16_t ArrayLength);
extern int32_t in32abs(int32_t a);  //绝对值函数[类型：int32_t]
extern float f_abs(float a);         //绝对值函数[类型：float]
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

