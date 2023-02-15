#include "Agreement.h"
#include "stm32f4xx_hal_usart.h"
#include "flash.h"
#include "bsp_BDCMotor.h"
#include "function.h"
#include "zd_driver.h"
static void Agreement_funF4(void);
static void Agreement_funF1(void);
static void Agreement_funF3(void);
static void Agreement_fun3(void);
static void Agreement_funEF(void);
static void Agreement_funF2(void);
void Startagreement(void);
void ReturnError1(void);
void QuickRead(void);
static void ReturnRight(void);
uint8_t CheckSum(uint8_t *Ptr,uint8_t Num );
uint8_t reflag_485;
uint8_t reflag;
uint8_t Flash_Writesign = 0;
uint16_t count_485=0;
uint16_t count=0;
uint8_t Buffer_485[20]= {0};
uint8_t aRxBuffer1[200]= {0};
uint8_t SendBuf[200]= {0};
__IO uint8_t SendBuf1[4]= {0};  //正确返回指令发送区域
__IO uint8_t SendBuf2[5]= {0};
__IO uint8_t SendBuf3[4]= {0};
UART_HandleTypeDef husart_debug;
u8 txflag=0;
void FlashRight();
void changestartstop(void);
int ErrTime1;/* 报警指令间隔时间*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 串口硬件初始化配置
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */


/**
  * 函数功能: 串口硬件反初始化配置
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */


/**
  * 函数功能: 串口参数配置.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int32_t transbuf[ParaNum];

u8 SendFlag,ReceiveFlag;

static void changestartstop(void)
{
	if(SendFlag==1)
	{
	transbuf[18]=MotorControl[3].PWM_DutySet;
	transbuf[19]=MotorControl[4].PWM_DutySet;
	transbuf[20]=MotorControl[7].PWM_DutySet;
	transbuf[21]=MotorControl[8].PWM_DutySet;
	transbuf[23]=MotorControl[10].PWM_DutySet;
	transbuf[24]=MotorControl[11].PWM_DutySet;
	transbuf[25]=MotorControl[12].PWM_DutySet;
	transbuf[26]=MotorControl[13].PWM_DutySet;
	transbuf[27]=MotorControl[5].Current.DeepFilterVAL;                                //MotorControl[5].Speed_Set; //无刷1速度给定
	transbuf[28]=MotorControl[1].Hall.HALL_CaptureValue;                               //MotorControl[6].Speed_Set; //无刷2速度给定
	transbuf[29]=MotorControl[5].Current.SetValue;                                                                      //MotorControl[5].Speed_Ref; //无刷1速度参考
	transbuf[30]=MotorControl[6].Speed_Ref; //无刷2速度参考	
//	transbuf[27]=MotorControl[5].Speed_Set; //无刷1速度给定
//	transbuf[28]=MotorControl[6].Speed_Set; //无刷2速度给定
//	transbuf[29]=MotorControl[5].Speed_Ref; //无刷1速度参考
//	transbuf[30]=MotorControl[6].Speed_Ref; //无刷2速度参考	
  transbuf[33]= MotorControl[5].Speed_Real;  //无刷1速度
	transbuf[34]= MotorControl[6].Speed_Real;  //无刷2速度
	transbuf[35]=MotorControl[8].Speed_Real; 
	transbuf[36]=MotorControl[7].Speed_Real;	
	transbuf[37]=MotorControl[5].SpeedLimit; 
	transbuf[38]=MotorControl[6].SpeedLimit;
	transbuf[44]= MotorControl[0].Current.FilterValue;   //电机0电流
	transbuf[45]= MotorControl[1].Current.FilterValue;   //电机1电流
	transbuf[46]= MotorControl[2].Current.FilterValue;   //电机2电流
	transbuf[47]= MotorControl[5].Current.FilterValue;   //电机3电流
	transbuf[48]= MotorControl[5].Current.ADCValue_Ref;   //电机4电流
	transbuf[49]= MotorControl[5].Current.DeepFilterVAL;   //电机5电流
	transbuf[50]= MotorControl[5].Current.GetADCValue;   //电机6电流
	transbuf[51]= MotorControl[9].Current.FilterValue;   //电机9电流
	transbuf[52]=MotorControl[5].Pole_Paires;            //无刷1极对数
	transbuf[53]=MotorControl[6].Pole_Paires ;           //无刷2极对数
	transbuf[54]=MotorControl[5].Direction ;
	transbuf[55]=MotorControl[6].Direction ;
	transbuf[56]=MotorControl[0].Acceleration; //电机0加速度
	transbuf[57]=MotorControl[1].Acceleration; //电机1加速度	
	transbuf[58]=MotorControl[2].Acceleration; //电机2加速度
	transbuf[59]=MotorControl[3].Acceleration; //电机3加速度	
	transbuf[60]=MotorControl[4].Acceleration; //电机4加速度
	transbuf[61]=MotorControl[5].Acceleration; //电机5加速度
	transbuf[62]=MotorControl[6].Acceleration; //电机6加速度	
	transbuf[63]=MotorControl[7].Acceleration; //电机7加速度
	transbuf[64]=MotorControl[8].Acceleration; //电机8加速度	
	transbuf[65]=MotorControl[9].Acceleration; //电机9加速度
	transbuf[66]=MotorControl[10].Acceleration; //电机10加速度	
	transbuf[67]=MotorControl[11].Acceleration; //电机11加速度
	transbuf[68]=MotorControl[12].Acceleration; //电机12加速度
	transbuf[69]=MotorControl[13].Acceleration; //电机13加速度		
	transbuf[70]=MotorControl[0].Deceleration; //电机0减速度
	transbuf[71]=MotorControl[1].Deceleration; //电机1减速度
	transbuf[72]=MotorControl[2].Deceleration; //电机2减速度
	transbuf[73]=MotorControl[3].Deceleration; //电机3减速度
	transbuf[74]=MotorControl[4].Deceleration; //电机4减速度
	transbuf[75]=MotorControl[5].Deceleration; //电机5减速度
	transbuf[76]=MotorControl[6].Deceleration; //电机6减速度
	transbuf[77]=MotorControl[7].Deceleration; //电机7减速度
	transbuf[78]=MotorControl[8].Deceleration; //电机8减速度
	transbuf[79]=MotorControl[9].Deceleration; //电机9减速度
	transbuf[80]=MotorControl[10].Deceleration; //电机10减速度
	transbuf[81]=MotorControl[11].Deceleration; //电机11减速度
	transbuf[82]=MotorControl[12].Deceleration; //电机12减速度
	transbuf[83]=MotorControl[13].Deceleration; //电机13减速度
	transbuf[101]=VoltVar.BUS;                  //母线电压
	transbuf[120]=PID_Speed_InitStruct[0].hKp_Gain;  //无刷1kp
	transbuf[121]=PID_Speed_InitStruct[1].hKp_Gain;  //无刷2kp
	transbuf[126]=PID_Speed_InitStruct[0].hKi_Gain;  //无刷1ki
	transbuf[127]=PID_Speed_InitStruct[1].hKi_Gain;  //无刷2ki
	transbuf[134]=MotorControl[0].Current.MaxValue1;
	transbuf[135]=MotorControl[1].Current.MaxValue1;
	transbuf[136]=MotorControl[2].Current.MaxValue1;
	transbuf[137]=MotorControl[9].Current.MaxValue1;
	transbuf[138]=MotorControl[3].Current.MaxValue1;
	transbuf[139]=MotorControl[4].Current.MaxValue1;
	transbuf[140]=MotorControl[5].Current.MaxValue1;
	transbuf[141]=MotorControl[6].Current.MaxValue1;
	transbuf[142]=MotorControl[0].Current.MaxValue2;
	transbuf[143]=MotorControl[1].Current.MaxValue2;
	transbuf[144]=MotorControl[2].Current.MaxValue2;
	transbuf[145]=MotorControl[9].Current.MaxValue2;
	transbuf[146]=MotorControl[3].Current.MaxValue2;
	transbuf[147]=MotorControl[4].Current.MaxValue2;
	transbuf[148]=MotorControl[5].Current.MaxValue2;
	transbuf[149]=MotorControl[6].Current.MaxValue2;
	transbuf[150]=MotorControl[0].Current.OFCnt1;
	transbuf[151]=MotorControl[1].Current.OFCnt1;
	transbuf[152]=MotorControl[2].Current.OFCnt1;
	transbuf[153]=MotorControl[9].Current.OFCnt1;
	transbuf[154]=MotorControl[3].Current.OFCnt1;
	transbuf[155]=MotorControl[4].Current.OFCnt1;
	transbuf[156]=MotorControl[5].Current.OFCnt1;
	transbuf[157]=MotorControl[6].Current.OFCnt1;
	transbuf[158]=MotorControl[0].Current.OFCnt2; 
	transbuf[159]=MotorControl[1].Current.OFCnt2; 
	transbuf[160]=MotorControl[2].Current.OFCnt2; 
	transbuf[161]=MotorControl[9].Current.OFCnt2; 
	transbuf[162]=MotorControl[3].Current.OFCnt2; 
	transbuf[163]=MotorControl[4].Current.OFCnt2;
  transbuf[164]=MotorControl[5].Current.OFCnt2;  
	transbuf[165]=MotorControl[6].Current.OFCnt2;
	transbuf[166]=MotorControl[5].Hall.HallState;
	transbuf[167]=MotorControl[6].Hall.HallState;
  transbuf[170]=HALL_Study[0].StudySectorCnt3;
	transbuf[171]=HALL_Study[1].StudySectorCnt3;
	transbuf[172]=HALL_Study[0].HallSector;
	transbuf[173]=HALL_Study[1].HallSector;
	transbuf[174]=HALL_Study[0].HallCommPWM;
  transbuf[175]=HALL_Study[1].HallCommPWM;	
//	transbuf[176]=Push_motor_calibrationFLAG;
	transbuf[177]=wGlobal_Flags;
	transbuf[178]=MotorState;
	transbuf[179]=TIM4->PSC;
	transbuf[180]=TIM10->PSC;

	transbuf[187]=MotorControl[0].Hall.HALL_CaptureValue;
  transbuf[188]=MotorControl[1].Hall.HALL_CaptureValue;
  transbuf[189]=MotorControl[2].Hall.HALL_CaptureValue;
  transbuf[190]=MotorControl[9].Hall.HALL_CaptureValue;
  transbuf[191]=MotorControl[5].Hall.HALL_CaptureValue;
  transbuf[192]=MotorControl[6].Hall.HALL_CaptureValue;		
	transbuf[194]=HALL_Study[0].HallTab[0];
	transbuf[195]=HALL_Study[0].HallTab[1];
	transbuf[196]=HALL_Study[0].HallTab[2];
	transbuf[197]=HALL_Study[0].HallTab[3];
	transbuf[198]=HALL_Study[0].HallTab[4];
	transbuf[199]=HALL_Study[0].HallTab[5];
	transbuf[200]=HALL_Study[1].HallTab[0];
	transbuf[201]=HALL_Study[1].HallTab[1];
	transbuf[202]=HALL_Study[1].HallTab[2];
	transbuf[203]=HALL_Study[1].HallTab[3];
	transbuf[204]=HALL_Study[1].HallTab[4];
	transbuf[205]=HALL_Study[1].HallTab[5];
	transbuf[206]=OverFlow_Cnt[0];
  transbuf[207]=OverFlow_Cnt[1];
  transbuf[208]=OverFlow_Cnt[2];
  transbuf[209]=OverFlow_Cnt[3];
  transbuf[210]=OverFlow_Cnt[4];
  transbuf[211]=Err_codeHis[0];
  transbuf[212]=Err_codeHis[1];
  transbuf[213]=Err_codeHis[2];
  transbuf[214]=Err_codeHis[3];
  transbuf[215]=Err_codeHis[4];
  transbuf[216]=Err_codeHis[5];
  transbuf[217]=Err_codeHis[6];
  transbuf[218]=Err_codeHis[7];
  transbuf[219]=Err_codeHis[8];
  transbuf[220]=Err_codeHis[9];
  transbuf[221]=Err_codeTick[0];
  transbuf[222]=Err_codeTick[1];
  transbuf[223]=Err_codeTick[2];
  transbuf[224]=Err_codeTick[3];
  transbuf[225]=Err_codeTick[4];
  transbuf[226]=Err_codeTick[5];
  transbuf[227]=Err_codeTick[6];
  transbuf[228]=Err_codeTick[7];
  transbuf[229]=Err_codeTick[8];
  transbuf[230]=Err_codeTick[9];
	SendFlag=0;
  }
		
	/*以下为需要写入的数据*/
	 if(ReceiveFlag==1)
	{
		MotorControl[0].Motor_Start_Stop=transbuf[1];
		MotorControl[1].Motor_Start_Stop=transbuf[2];
		MotorControl[2].Motor_Start_Stop=transbuf[3];
		MotorControl[3].Motor_Start_Stop=transbuf[4];
		MotorControl[4].Motor_Start_Stop=transbuf[5];
		MotorControl[5].Motor_Start_Stop=transbuf[6];
		MotorControl[6].Motor_Start_Stop=transbuf[7];
		MotorControl[7].Motor_Start_Stop=transbuf[8];
		MotorControl[8].Motor_Start_Stop=transbuf[9];
		MotorControl[9].Motor_Start_Stop=transbuf[10];
		MotorControl[10].Motor_Start_Stop=transbuf[11];
		MotorControl[11].Motor_Start_Stop=transbuf[12];
		MotorControl[12].Motor_Start_Stop=transbuf[13];
		MotorControl[13].Motor_Start_Stop=transbuf[14];
		MotorControl[3].PWM_DutySet=transbuf[18];
		MotorControl[4].PWM_DutySet=transbuf[19];
		MotorControl[7].PWM_DutySet=transbuf[20];
		MotorControl[8].PWM_DutySet=transbuf[21];
		MotorControl[10].PWM_DutySet=transbuf[23];
		MotorControl[11].PWM_DutySet=transbuf[24];
		MotorControl[12].PWM_DutySet=transbuf[25];
		MotorControl[13].PWM_DutySet=transbuf[26];
		MotorControl[5].Speed_Set=transbuf[27]; //无刷1速度给定
		MotorControl[6].Speed_Set=transbuf[28]; //无刷2速度给定
	
	 MotorControl[5].Pole_Paires=transbuf[52];            //无刷1极对数
	 MotorControl[6].Pole_Paires=transbuf[53];           //无刷2极对数	
	 MotorControl[5].Direction=transbuf[54];  //无刷电机方向
	 MotorControl[6].Direction=transbuf[55];
   MotorControl[0].Acceleration=transbuf[56]; //电机0加速度
	 MotorControl[1].Acceleration=transbuf[57]; //电机1加速度	
	 MotorControl[2].Acceleration=transbuf[58]; //电机2加速度
	 MotorControl[3].Acceleration=transbuf[59]; //电机3加速度	
	 MotorControl[4].Acceleration=transbuf[60]; //电机4加速度
	 MotorControl[5].Acceleration=transbuf[61]; //电机5加速度
	 MotorControl[6].Acceleration=transbuf[62]; //电机6加速度	
	 MotorControl[7].Acceleration=transbuf[63]; //电机7加速度
	 MotorControl[8].Acceleration=transbuf[64]; //电机8加速度	
	 MotorControl[9].Acceleration=transbuf[65]; //电机9加速度
	 MotorControl[10].Acceleration=transbuf[66]; //电机10加速度	
	 MotorControl[11].Acceleration=transbuf[67]; //电机11加速度
	 MotorControl[12].Acceleration=transbuf[68]; //电机12加速度
	 MotorControl[13].Acceleration=transbuf[69]; //电机13加速度	
	 MotorControl[0].Deceleration=transbuf[70]; //电机0减速度
	 MotorControl[1].Deceleration=transbuf[71]; //电机1减速度
	 MotorControl[2].Deceleration=transbuf[72]; //电机2减速度
	 MotorControl[3].Deceleration=transbuf[73]; //电机3减速度
	 MotorControl[4].Deceleration=transbuf[74]; //电机4减速度
	 MotorControl[5].Deceleration=transbuf[75]; //电机5减速度
	 MotorControl[6].Deceleration=transbuf[76]; //电机6减速度
	 MotorControl[7].Deceleration=transbuf[77]; //电机7减速度
	 MotorControl[8].Deceleration=transbuf[78]; //电机8减速度
	 MotorControl[9].Deceleration=transbuf[79]; //电机9减速度
	 MotorControl[10].Deceleration=transbuf[80]; //电机10减速度
	 MotorControl[11].Deceleration=transbuf[81]; //电机11减速度
	 MotorControl[12].Deceleration=transbuf[82]; //电机12减速度
	 MotorControl[13].Deceleration=transbuf[83]; //电机13减速度
	MotorControl[0].Fault_Flag=transbuf[84];   //电机0错误标记
	MotorControl[1].Fault_Flag=transbuf[85];   //电机1错误标记
	MotorControl[2].Fault_Flag=transbuf[89];   //电机2错误标记
	MotorControl[3].Fault_Flag=transbuf[90];   //电机3错误标记
	MotorControl[4].Fault_Flag=transbuf[90];   //电机4错误标记
 	MotorControl[5].Fault_Flag=transbuf[89];   //电机5错误标记
	MotorControl[6].Fault_Flag=transbuf[90];   //电机6错误标记
	MotorControl[7].Fault_Flag=transbuf[91];   //电机7错误标记
	MotorControl[8].Fault_Flag=transbuf[92];   //电机8错误标记
	MotorControl[9].Fault_Flag=transbuf[93];   //电机9错误标记
	MotorControl[10].Fault_Flag=transbuf[94];   //电机10错误标记
	MotorControl[11].Fault_Flag=transbuf[95];   //电机11错误标记
 	MotorControl[12].Fault_Flag=transbuf[96];   //电机12错误标记
	MotorControl[13].Fault_Flag=transbuf[97];   //电机13错误标记
	PID_Speed_InitStruct[0].hKp_Gain=transbuf[120];  //无刷1kp
	PID_Speed_InitStruct[1].hKp_Gain=transbuf[121];  //无刷2kp
	PID_Speed_InitStruct[0].hKi_Gain=transbuf[126];  //无刷1ki
	PID_Speed_InitStruct[1].hKi_Gain=transbuf[127];  //无刷2ki
	HALL_Study[0].StudySectorCnt3=transbuf[170];
	HALL_Study[1].StudySectorCnt3=transbuf[171];
	HALL_Study[0].HallCommPWM=transbuf[174];
  HALL_Study[1].HallCommPWM=transbuf[175];	
//	Push_motor_calibrationFLAG=transbuf[176];	
	wGlobal_Flags=transbuf[177];	
	MotorState=transbuf[178];	
	TIM4->PSC=transbuf[179];	
	TIM10->PSC=transbuf[180];	
	
	MotorControl[0].Location_Set=transbuf[181];
	MotorControl[1].Location_Set=transbuf[182];//位置给定
	MotorControl[2].Location_Set=transbuf[183];
	MotorControl[9].Location_Set=transbuf[184];
	HALL_Study[0].HallTab[0]=transbuf[194]; /*霍尔学习*/
	HALL_Study[0].HallTab[1]=transbuf[195];
	HALL_Study[0].HallTab[2]=transbuf[196];
	HALL_Study[0].HallTab[3]=transbuf[197];
	HALL_Study[0].HallTab[4]=transbuf[198];
	HALL_Study[0].HallTab[5]=transbuf[199];
	HALL_Study[1].HallTab[0]=transbuf[200];
	HALL_Study[1].HallTab[1]=transbuf[201];
	HALL_Study[1].HallTab[2]=transbuf[202];
	HALL_Study[1].HallTab[3]=transbuf[203];
	HALL_Study[1].HallTab[4]=transbuf[204];
	HALL_Study[1].HallTab[5]=transbuf[205];
	ReceiveFlag=0;
	 }
}


const PARAMETER_TABLE  PARAMETER[ParaNum] =
{   \
   {0,1000,WR,NOFLASH,(long*)&transbuf[0]},
	{1,1000,WR,NOFLASH,(long*)&transbuf[1]},	
	{2,1001,WR,NOFLASH,(long*)&transbuf[2]},	
	{3,1002,WR,NOFLASH,(long*)&transbuf[3]},	
	{4,1003,WR,NOFLASH,(long*)&transbuf[4]},  
	{5,1004,WR,NOFLASH,(long*)&transbuf[5]},  
	{6,1005,WR,NOFLASH,(long*)&transbuf[6]},  
	{7,1006,WR,NOFLASH,(long*)&transbuf[7]},  //电机6使能
	{8,1007,WR,NOFLASH,(long*)&transbuf[8]},  //电机7使能
	{9,1008,WR,NOFLASH,(long*)&transbuf[9]},  //电机8使能
	{10,1009,WR,NOFLASH,(long*)&transbuf[10]}, //电机9使能
	{11,1010,WR,NOFLASH,(long*)&transbuf[11]}, //电机10使能
	{12,1011,WR,NOFLASH,(long*)&transbuf[12]}, //电机11使能
	{13,1012,WR,NOFLASH,(long*)&transbuf[13]}, //电机12使能
	{14,1013,WR,NOFLASH,(long*)&transbuf[14]}, //电机13使能
	{15,1014,WR,NOFLASH,(long*)&transbuf[15]}, //电机0PWM
	{16,1015,WR,NOFLASH,(long*)&transbuf[16]},	 //1PWM
	{17,1016,WR,NOFLASH,(long*)&transbuf[17]},	 //2PWM，加了位置环，无法通过dutyset控制
	{18,1017,WR,NOFLASH,(long*)&transbuf[18]},	 //3PWM
	{19,1018,WR,NOFLASH,(long*)&transbuf[19]},  //4PWM
	{20,1019,WR,NOFLASH,(long*)&transbuf[20]},  //7PWM
	{21,1020,WR,NOFLASH,(long*)&transbuf[21]},  //8PWM
	{22,1021,WR,NOFLASH,(long*)&transbuf[22]},  //9PWM
	{23,1022,WR,NOFLASH,(long*)&transbuf[23]},  //10PWM
	{24,1023,WR,NOFLASH,(long*)&transbuf[24]},  //11PWM
	{25,1024,WR,NOFLASH,(long*)&transbuf[25]},  //12PWM
	{26,1025,WR,NOFLASH,(long*)&transbuf[26]},  //13PWM
	{27,1026,WR,NOFLASH,(long*)&transbuf[27]},  //无刷速度给定1
	{28,1027,WR,NOFLASH,(long*)&transbuf[28]},  //无刷速度给定2
	{29,1028,OR,NOFLASH,(long*)&transbuf[29]},  //无刷1速度参考
	{30,1029,OR,NOFLASH,(long*)&transbuf[30]},  //无刷2速度参考
	{31,1030,OR,NOFLASH,(long*)&transbuf[31]},  //无刷1速度偏差
	
	{32,1031,OR,NOFLASH,(long*)&transbuf[32]},	 //无刷2速度偏差
	{33,1032,OR,NOFLASH,(long*)&transbuf[33]},	 //无刷1实际速度
	{34,1033,OR,NOFLASH,(long*)&transbuf[34]},	 //无刷2实际速度
	{35,1034,OR,NOFLASH,(long*)&transbuf[35]},  //风机1实际速度
	{36,1035,OR,NOFLASH,(long*)&transbuf[36]},  //边刷实际速度
	{37,1036,WR,YSFLASH,(long*)&transbuf[37]},                    //无刷1最大速度限制
	{38,1037,WR,YSFLASH,(long*)&transbuf[38]},                    //无刷2最大速度限制
	{39,1038,OR,NOFLASH,(long*)&transbuf[39]},  //电机0实际位置
	{40,1039,OR,NOFLASH,(long*)&transbuf[40]},  //电机1实际位置
	{41,1040,OR,NOFLASH,(long*)&transbuf[41]},  //电机2实际位置
	{42,1041,OR,NOFLASH,(long*)&transbuf[42]},  //电机9实际位置
	{43,1042,OR,NOFLASH,(long*)&transbuf[43]},  //电机4实际位置
	{44,1043,OR,NOFLASH,(long*)&transbuf[44]},  //电机0电流
	{45,1044,OR,NOFLASH,(long*)&transbuf[45]},  //电机1电流
	{46,1045,OR,NOFLASH,(long*)&transbuf[46]},  //电机2电流
	{47,1046,OR,NOFLASH,(long*)&transbuf[47]},  //电机3电流
	  
	{48,1047,OR,NOFLASH,(long*)&transbuf[48]},	 //电机4电流
	{49,1048,OR,NOFLASH,(long*)&transbuf[49]},	 //电机5电流
	{50,1049,OR,NOFLASH,(long*)&transbuf[50]},	 //电机6电流
	{51,1050,OR,NOFLASH,(long*)&transbuf[51]},  //电机9电流
	{52,1051,WR,YSFLASH,(long*)&transbuf[52]},  //无刷1极对数
	{53,1052,WR,YSFLASH,(long*)&transbuf[53]},  //无刷2极对数
	{54,1053,WR,NOFLASH,(long*)&transbuf[54]},          //无刷1运行方向
	{55,1054,WR,NOFLASH,(long*)&transbuf[55]},          //无刷2运行方向
	{56,1055,WR,YSFLASH,(long*)&transbuf[56]},  //电机0加速度
	{57,1056,WR,YSFLASH,(long*)&transbuf[57]},  //电机1加速度
	{58,1057,WR,YSFLASH,(long*)&transbuf[58]},  //电机2加速度
	{59,1058,WR,YSFLASH,(long*)&transbuf[59]},  //电机3加速度
	{60,1059,WR,YSFLASH,(long*)&transbuf[60]},  //电机4加速度
	{61,1060,WR,YSFLASH,(long*)&transbuf[61]},  //电机5加速度
	{62,1061,WR,YSFLASH,(long*)&transbuf[62]},  //6加速度
	{63,1062,WR,YSFLASH,(long*)&transbuf[63]},  //7加速度
	
	{64,1063,WR,YSFLASH,(long*)&transbuf[64]},	 //8加速度
	{65,1064,WR,YSFLASH,(long*)&transbuf[65]},	 //9加速度
	{66,1065,WR,YSFLASH,(long*)&transbuf[66]},	 //10加速度
	{67,1066,WR,YSFLASH,(long*)&transbuf[67]},  //11加速度
	{68,1067,WR,YSFLASH,(long*)&transbuf[68]},  //12加速度
	{69,1068,WR,YSFLASH,(long*)&transbuf[69]},  //13加速度
	{70,1069,WR,YSFLASH,(long*)&transbuf[70]},  //0减速度
	{71,1070,WR,YSFLASH,(long*)&transbuf[71]},  //1减速度
	{72,1071,WR,YSFLASH,(long*)&transbuf[72]},  //2减速度
	{73,1072,WR,YSFLASH,(long*)&transbuf[73]},  //3减速度
	{74,1073,WR,YSFLASH,(long*)&transbuf[74]},  //4减速度
	{75,1074,WR,YSFLASH,(long*)&transbuf[75]},             //5减速度
	{76,1075,WR,YSFLASH,(long*)&transbuf[76]},             //6减速度
	{77,1076,WR,YSFLASH,(long*)&transbuf[77]},  //7减速度
	{78,1077,WR,YSFLASH,(long*)&transbuf[78]},  //8减速度
	{79,1078,WR,YSFLASH,(long*)&transbuf[79]},  //9减速度
	
	{80,1079,WR,YSFLASH,(long*)&transbuf[80]},	 //10减速度
	{81,1080,WR,YSFLASH,(long*)&transbuf[81]},	 //11减速度
	{82,1081,WR,YSFLASH,(long*)&transbuf[82]},	 //12减速度
	{83,1082,WR,YSFLASH,(long*)&transbuf[83]},  //13减速度
	{84,1083,WR,NOFLASH,(long*)&transbuf[84]},  //0错误标记
	{85,1084,WR,NOFLASH,(long*)&transbuf[85]},  //1错误标记
	{86,1085,WR,NOFLASH,(long*)&transbuf[86]},  //2错误标记
	{87,1086,WR,NOFLASH,(long*)&transbuf[87]},  //3错误标记
	{88,1087,WR,NOFLASH,(long*)&transbuf[88]},  //4错误标记
	{89,1088,WR,NOFLASH,(long*)&transbuf[89]},              //5错误标记
	{90,1089,WR,NOFLASH,(long*)&transbuf[90]},              //6错误标记
	{91,1090,WR,NOFLASH,(long*)&transbuf[91]},  //7错误标记
	{92,1091,WR,NOFLASH,(long*)&transbuf[92]},  //8错误标记
	{93,1092,WR,NOFLASH,(long*)&transbuf[93]},  //9错误标记
	{94,1093,WR,NOFLASH,(long*)&transbuf[94]},  //10错误标记
	{95,1094,WR,NOFLASH,(long*)&transbuf[95]},  //11错误标记
	
	{96,1095,WR,NOFLASH,(long*)&transbuf[96]},	 //12错误标记
	{97,1096,WR,NOFLASH,(long*)&transbuf[97]},	 //13错误标记
	{98,1097,OR,NOFLASH,(long*)&transbuf[98]},	 //无刷0温度
	{99,1098,OR,NOFLASH,(long*)&transbuf[99]},  //无刷1温度
	{100,1099,OR,NOFLASH,(long*)&transbuf[100]},  //风机温度
	{101,1100,OR,NOFLASH,(long*)&transbuf[101]},  //母线电压
	{102,1101,WR,NOFLASH,(long*)&transbuf[102]},  //0刹车信号
	{103,1102,WR,NOFLASH,(long*)&transbuf[103]},  //1刹车信号
	{104,1103,WR,NOFLASH,(long*)&transbuf[104]},  //2刹车信号
	{105,1104,WR,NOFLASH,(long*)&transbuf[105]},  //3刹车信号
	{106,1105,WR,NOFLASH,(long*)&transbuf[106]},  //4刹车信号
	{107,1106,WR,NOFLASH,(long*)&transbuf[107]},  //5刹车信号
	{108,1107,WR,NOFLASH,(long*)&transbuf[108]},  //6刹车信号
	{109,1108,WR,NOFLASH,(long*)&transbuf[109]},  //7刹车信号
	{110,1109,WR,NOFLASH,(long*)&transbuf[110]},  //8刹车信号
	{111,1110,WR,NOFLASH,(long*)&transbuf[111]},  //9刹车信号
	
	{112,1111,WR,NOFLASH,(long*)&transbuf[112]},	 //10刹车信号
	{113,1112,WR,NOFLASH,(long*)&transbuf[113]},		//11刹车信号
	{114,1113,WR,NOFLASH,(long*)&transbuf[114]},	  //12刹车信号
	{115,1114,WR,NOFLASH,(long*)&transbuf[115]},    //13刹车信号
	{116,1115,WR,YSFLASH,(long*)&transbuf[116]},    //电机0kp
	{117,1116,WR,YSFLASH,(long*)&transbuf[117]},    //电机1kp
	{118,1117,WR,YSFLASH,(long*)&transbuf[118]},    //电机2kp
	{119,1118,WR,YSFLASH,(long*)&transbuf[119]},     //电机9kp
	{120,1119,WR,YSFLASH,(long*)&transbuf[120]},     //电机5kp
	{121,1120,WR,YSFLASH,(long*)&transbuf[121]},    //电机6kp
	{122,1121,WR,YSFLASH,(long*)&transbuf[122]},    //电机0ki
	{123,1122,WR,YSFLASH,(long*)&transbuf[123]},     //电机1ki
	{124,1123,WR,YSFLASH,(long*)&transbuf[124]},     //2ki
	{125,1124,WR,YSFLASH,(long*)&transbuf[125]},    //9ki
	{126,1125,WR,YSFLASH,(long*)&transbuf[126]},    //5ki
	{127,1126,WR,YSFLASH,(long*)&transbuf[127]},   //6ki
	
	{128,1127,WR,YSFLASH,(long*)&transbuf[128]},	 //0kd
	{129,1128,WR,YSFLASH,(long*)&transbuf[129]},	 //1kd
	{130,1129,WR,YSFLASH,(long*)&transbuf[130]},	 //2kd
	{131,1130,WR,YSFLASH,(long*)&transbuf[131]},   //9kd
	{132,1131,WR,YSFLASH,(long*)&transbuf[132]},   //5kd
	{133,1132,WR,YSFLASH,(long*)&transbuf[133]},   //6kd
	{134,1133,OR,YSFLASH,(long*)&transbuf[134]},    //0第一阶段电流限制
	{135,1134,OR,YSFLASH,(long*)&transbuf[135]},    //1第一阶段电流限制
	{136,1135,OR,YSFLASH,(long*)&transbuf[136]},    //2第一阶段电流限制
	{137,1136,OR,YSFLASH,(long*)&transbuf[137]},    //9第一阶段电流限制
	{138,1137,OR,YSFLASH,(long*)&transbuf[138]},    //3第一阶段电流限制
	{139,1138,OR,YSFLASH,(long*)&transbuf[139]},    //4第一阶段电流限制
	{140,1139,OR,YSFLASH,(long*)&transbuf[140]},              //5第一阶段电流限制
	{141,1140,OR,YSFLASH,(long*)&transbuf[141]},              //6第一阶段电流限制
	{142,1141,OR,YSFLASH,(long*)&transbuf[142]},    //0第二阶段电流限制
	{143,1142,OR,YSFLASH,(long*)&transbuf[143]},    //1第二阶段电流限制
	
	{144,1143,OR,YSFLASH,(long*)&transbuf[144]},	   //2第二阶段电流限制
	{145,1144,OR,YSFLASH,(long*)&transbuf[145]},		//9二阶段电流限制
	{146,1145,OR,YSFLASH,(long*)&transbuf[146]},	   //3二阶段电流限制
	{147,1146,OR,YSFLASH,(long*)&transbuf[147]},     //4二阶段电流限制
	{148,1147,OR,YSFLASH,(long*)&transbuf[148]},             //5二阶段电流限制
	{149,1148,OR,YSFLASH,(long*)&transbuf[149]},             //6二阶段电流限制
	{150,1149,OR,YSFLASH,(long*)&transbuf[150]},     //0一阶错误累计
	{151,1150,OR,YSFLASH,(long*)&transbuf[151]},     //1一阶错误累计
	{152,1151,OR,YSFLASH,(long*)&transbuf[152]},     //2一阶错误累计
	{153,1152,OR,YSFLASH,(long*)&transbuf[153]},     //3一阶错误累计
	{154,1153,OR,YSFLASH,(long*)&transbuf[154]},     //4一阶错误累计
	{155,1154,OR,YSFLASH,(long*)&transbuf[155]},     //5一阶错误累计
	{156,1155,OR,YSFLASH,(long*)&transbuf[156]},     //6一阶错误累计
	{157,1156,OR,YSFLASH,(long*)&transbuf[157]},     //7一阶错误累计
	{158,1157,OR,YSFLASH,(long*)&transbuf[158]},    //0二阶错误累计
	{159,1158,OR,YSFLASH,(long*)&transbuf[159]},    //1二阶错误累计
	
	{160,1159,OR,YSFLASH,(long*)&transbuf[160]},	   //2二阶阶错误累计
	{161,1160,OR,YSFLASH,(long*)&transbuf[161]},		 //3二阶阶错误累计
	{162,1161,OR,YSFLASH,(long*)&transbuf[162]},	   //4二阶阶错误累计
	{163,1162,OR,YSFLASH,(long*)&transbuf[163]},     //5二阶阶错误累计
	{164,1163,OR,YSFLASH,(long*)&transbuf[164]},      //6二阶阶错误累计
	{165,1164,OR,YSFLASH,(long*)&transbuf[165]},       //7二阶阶错误累计
	{166,1165,OR,NOFLASH,(long*)&transbuf[166]},       //无刷1霍尔当前值
	{167,1166,OR,NOFLASH,(long*)&transbuf[167]},       //无刷2霍尔当前值
	{168,1167,OR,YSFLASH,(long*)&transbuf[168]},       //无刷1霍尔学习数组
	{169,1168,OR,YSFLASH,(long*)&transbuf[169]},       //无刷2霍尔学习数组
	{170,1169,WR,YSFLASH,(long*)&transbuf[170]},       //无刷1单次换向时间
	{171,1170,WR,YSFLASH,(long*)&transbuf[171]},       //无刷2单次换向时间
	{172,1171,WR,YSFLASH,(long*)&transbuf[172]},       //无刷1学习换向次数
	{173,1172,WR,YSFLASH,(long*)&transbuf[173]},       //无刷2学习换向次数
	{174,1173,WR,YSFLASH,(long*)&transbuf[174]},       //无刷1学习pwm
	{175,1174,WR,YSFLASH,(long*)&transbuf[175]},       //无刷2学习pwm
	
	{176,1175,WR,NOFLASH,(long*)&transbuf[176]},	     //推杆标定标志 
	{177,1176,WR,NOFLASH,(long*)&transbuf[177]},		   //全局错误
	{178,1177,WR,NOFLASH,(long*)&transbuf[178]},	     //无刷1运行状态
	{179,1178,WR,YSFLASH,(long*)&transbuf[179]},      //水泵1频率，用于双轴时表示无刷2运行状态
	{180,1179,WR,YSFLASH,(long*)&transbuf[180]},       //水泵2频率
  {181,1180,WR,NOFLASH,(long*)&transbuf[181]},        //电机0位置给定
  {182,1181,WR,NOFLASH,(long*)&transbuf[182]},        //电机1位置给定
  {183,1182,OR,NOFLASH,(long*)&transbuf[183]},        //电机2位置给定
  {184,1183,OR,NOFLASH,(long*)&transbuf[184]},        //电机9位置给定
  {185,1184,OR,NOFLASH,(long*)&transbuf[185]},       
  {186,1185,OR,NOFLASH,(long*)&transbuf[186]},      
  {187,1186,OR,NOFLASH,(long*)&transbuf[187]},        //电机0实际位置
  {188,1187,OR,NOFLASH,(long*)&transbuf[188]},        //电机1实际位置
  {189,1188,OR,NOFLASH,(long*)&transbuf[189]},        //电机2实际位置
  {190,1189,OR,NOFLASH,(long*)&transbuf[190]},        //电机9实际位置
  {191,1190,OR,NOFLASH,(long*)&transbuf[191]},        //电机5实际位置
	{192,1191,OR,NOFLASH,(long*)&transbuf[192]},        //电机6实际位置
  {193,1192,OR,NOFLASH,(long*)&transbuf[193]},            
	{194,1,WR,YSFLASH,(long*)&transbuf[194]},        //无刷1霍尔学习数组
	{195,1,WR,YSFLASH,(long*)&transbuf[195]},        //无刷1霍尔学习数组
	{196,1,WR,YSFLASH,(long*)&transbuf[196]},        //无刷1霍尔学习数组
	{197,1,WR,YSFLASH,(long*)&transbuf[197]},        //无刷1霍尔学习数组
	{198,1,WR,YSFLASH,(long*)&transbuf[198]},        //无刷1霍尔学习数组
	{199,1,WR,YSFLASH,(long*)&transbuf[199]},        //无刷1霍尔学习数组
	{200,1,WR,YSFLASH,(long*)&transbuf[200]},        //无刷2霍尔学习数组
	{201,1,WR,YSFLASH,(long*)&transbuf[201]},        //无刷2霍尔学习数组
	{202,1,WR,YSFLASH,(long*)&transbuf[202]},        //无刷2霍尔学习数组
	{203,1,WR,YSFLASH,(long*)&transbuf[203]},        //无刷2霍尔学习数组
	{204,1,WR,YSFLASH,(long*)&transbuf[204]},        //无刷2霍尔学习数组
	{205,1,WR,YSFLASH,(long*)&transbuf[205]},        //无刷2霍尔学习数组
	{206,1206,WR,YSFLASH,(long*)&transbuf[206]},			//电机5过流次数记录
	{207,1207,WR,YSFLASH,(long*)&transbuf[207]},			//电机6过流次数记录
	{208,1208,WR,YSFLASH,(long*)&transbuf[208]},			//推杆过流次数记录
	{209,1209,WR,YSFLASH,(long*)&transbuf[209]},			//电机3，4过流次数记录
	{210,1210,WR,YSFLASH,(long*)&transbuf[210]},				//电机10，11，12，13过流次数记录
	{211,1211,OR,YSFLASH,(long*)&transbuf[211]},			//错误历史1
	{212,1212,OR,YSFLASH,(long*)&transbuf[212]},			//错误历史2
	{213,1213,OR,YSFLASH,(long*)&transbuf[213]},			//错误历史3
	{214,1214,OR,YSFLASH,(long*)&transbuf[214]},			//错误历史4
	{215,1215,OR,YSFLASH,(long*)&transbuf[215]},			//错误历史5
	{216,1216,OR,YSFLASH,(long*)&transbuf[216]},			//错误历史6
	{217,1217,OR,YSFLASH,(long*)&transbuf[217]},			//错误历史7
	{218,1218,OR,YSFLASH,(long*)&transbuf[218]},			//错误历史8
	{219,1219,OR,YSFLASH,(long*)&transbuf[219]},			//错误历史9
	{220,1220,OR,YSFLASH,(long*)&transbuf[220]},			//错误历史10
	{221,1221,OR,YSFLASH,(long*)&transbuf[221]},			//错误时间1
	{222,1222,OR,YSFLASH,(long*)&transbuf[222]},			//错误时间2
	{223,1223,OR,YSFLASH,(long*)&transbuf[223]},			//错误时间3
	{224,1224,OR,YSFLASH,(long*)&transbuf[224]},			//错误时间4
	{225,1225,OR,YSFLASH,(long*)&transbuf[225]},			//错误时间5
	{226,1226,OR,YSFLASH,(long*)&transbuf[226]},			//错误时间6
	{227,1227,OR,YSFLASH,(long*)&transbuf[227]},			//错误时间7
	{228,1228,OR,YSFLASH,(long*)&transbuf[228]},			//错误时间8
	{229,1229,OR,YSFLASH,(long*)&transbuf[229]},			//错误时间9
	{230,1230,OR,YSFLASH,(long*)&transbuf[230]},			//错误时间10
};
uint32_t  test=0;
extern uint8_t aRxBuffer[50];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//    test++;
//    HAL_UART_Receive_IT(&huart4,aRxBuffer,1);

}
void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 57600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler(); 
    } 
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */

}

void MX_USART4_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler(); 
    } 
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */

}
void USART3_IRQHandler()                     //串口中断处理函数
{
    if(__HAL_USART_GET_FLAG(&huart3,USART_FLAG_RXNE)!= RESET) // 接收中断：接收到数据
    {
        uint8_t data;
        data=READ_REG(huart3.Instance->DR); // 读取数据
        /*新加的东西*/
        if(count_485==0) // 如果是重新接收到数据帧，开启串口空闲中断
        {
            __HAL_UART_CLEAR_FLAG(&huart3,USART_FLAG_IDLE); // 清除空闲中断标志
            __HAL_UART_ENABLE_IT(&huart3,USART_IT_IDLE);     // 使能空闲中断
        }

        Buffer_485[count_485++]=data;//将数据保存在aRxBuffer中
    }
    else	if(__HAL_USART_GET_FLAG(&huart3,USART_FLAG_IDLE)!= RESET) // 串口空闲中断 新加的
    {
        __HAL_UART_CLEAR_FLAG(&huart3,USART_FLAG_IDLE); // 清除空闲中断标志
        __HAL_UART_DISABLE_IT(&huart3,USART_IT_IDLE);    // 关闭空闲中断
//			  B86_state(Buffer_485,count_485);
//			  Bm_state(Buffer_485,count_485);
//				count_485=0;
        reflag_485=1;		                                 // 数据帧置位，标识接收到一个完整数据帧
    }
		
}

void DEBUG_USARTx_IRQHandler()                     //串口4中断处理函数
{
    if(__HAL_USART_GET_FLAG(&huart4,USART_FLAG_RXNE)!= RESET) // 接收中断：接收到数据
    {
        uint8_t data;
        data=READ_REG(huart4.Instance->DR); // 读取数据
        /*新加的东西*/
        if(count==0) // 如果是重新接收到数据帧，开启串口空闲中断
        {
            __HAL_UART_CLEAR_FLAG(&huart4,USART_FLAG_IDLE); // 清除空闲中断标志
            __HAL_UART_ENABLE_IT(&huart4,USART_IT_IDLE);     // 使能空闲中断
        }

        aRxBuffer1[count++]=data;//将数据保存在aRxBuffer中
    }
    else	if(__HAL_USART_GET_FLAG(&huart4,USART_FLAG_IDLE)!= RESET) // 串口空闲中断 新加的
    {
        __HAL_UART_CLEAR_FLAG(&huart4,USART_FLAG_IDLE); // 清除空闲中断标志
        __HAL_UART_DISABLE_IT(&huart4,USART_IT_IDLE);    // 关闭空闲中断
        reflag=1;		                                 // 数据帧置位，标识接收到一个完整数据帧
        count=0;
    }
		
}

void Flash_WriteCheck(void)
{
	if(Flash_Writesign)
	{
		if(InternalFlash(transbuf,ParaNum))
			Flash_Writesign=0;
	}
}

void Startagreement()
{
    if(reflag==1)             //收到数据啦
    {

        if(aRxBuffer1[0]==FRAME_START)                          // 判断首位是不是5A
        {
            if(aRxBuffer1[1]==0x03)                           //判断功能码//下位机传数据给上位机
            {
								 SendFlag=1;
								 changestartstop();
                if(CheckSum((uint8_t*)&aRxBuffer1[0],aRxBuffer1[2]+3)==aRxBuffer1[aRxBuffer1[2]+3])	  //03功能码校验
                {
									Agreement_fun3();
									ReturnRight();
                }

            }

            if((aRxBuffer1[1]==0xE2) && (aRxBuffer1[2]==0x00)&& (aRxBuffer1[3]==0x3C))
            {
                if(InternalFlash(transbuf,ParaNum)==1)
                  FlashRight();
            }


            if(aRxBuffer1[1]==0x06)                           //上位机控制下位机
            {
                if(CheckSum((uint8_t*)&aRxBuffer1[0],7)==aRxBuffer1[7])	  //06功能码校验
                {
									ReceiveFlag=1;
									if((aRxBuffer1[2]==0xB2)|| (aRxBuffer1[2]==0xB3))    //运行状态单独处理，其为enum常量
								  {
										*PARAMETER[aRxBuffer1[2]].lpParam=aRxBuffer1[3];
										changestartstop();
										ReturnRight();
								  }
									else
									{
										*PARAMETER[aRxBuffer1[2]].lpParam=aRxBuffer1[3]|aRxBuffer1[4]<<8|aRxBuffer1[5]<<16|aRxBuffer1[6]<<24;
										changestartstop();
										ReturnRight();
									}					
                }		
            }
        }
        reflag=0;
    }
}


void QuickRead()
{
    if(aRxBuffer1[0]==FRAME_START)                          // 判断首位是不是5A
    {

        if(aRxBuffer1[1]==0xEF)                           //判断功能码//下位机传数据给上位机
        {
		  SendFlag=1;
		  changestartstop();
		  if(CheckSum((uint8_t*)&aRxBuffer1[0],aRxBuffer1[2]+3)==aRxBuffer1[aRxBuffer1[2]+3])	  //校验           
		  Agreement_funEF();   
        }
		else if(aRxBuffer1[1]==0xF2)
		  {
			SendFlag=1;
			changestartstop();
            if(CheckSum((uint8_t*)&aRxBuffer1[0],4)==aRxBuffer1[4])	  //校验
			Agreement_funF2();
		  }
			else if(aRxBuffer1[1]==0xF1)
		  {
			SendFlag=1;
			changestartstop();
		   if(CheckSum((uint8_t*)&aRxBuffer1[0],3)==aRxBuffer1[3])	  //校验
			 {
				Agreement_funF1();
			 }
		  }
			else if(aRxBuffer1[1]==0xF3)
		  {
				SendFlag=1;
				changestartstop();
             if(CheckSum((uint8_t*)&aRxBuffer1[0],5)==aRxBuffer1[5])	  //校验
			 {
				Agreement_funF3();
			 }
		  }
			else if(aRxBuffer1[1]==0xF4)
		  {
			SendFlag=1;
			changestartstop();
			if(CheckSum((uint8_t*)&aRxBuffer1[0],6)==aRxBuffer1[6])	  //校验
			 {
				Agreement_funF4();
			 }
		  }
    }
}



void Agreement_fun3()         //读寄存器
{
    int j;
    SendBuf[0]=FRAME_START;   //发送数据第一个字节
    SendBuf[1]=0x03;            //发送数据第二一个字节
    SendBuf[2]=aRxBuffer1[2];           //发送数据第三个字节
    for(j=1; j<=aRxBuffer1[2]; j++)   //循环发送数据
    {
        SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[2+j]].lpParam)&0xFF;
        SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[2+j]].lpParam)>>8&0xFF;
        SendBuf[4*(j-1)+5]= ((*PARAMETER[aRxBuffer1[2+j]].lpParam)>>16)&0xFF;
        SendBuf[4*(j-1)+6]= ((*PARAMETER[aRxBuffer1[2+j]].lpParam)>>24);
    }
    SendBuf[4*aRxBuffer1[2]+3]=CheckSum((uint8_t*)&SendBuf[0],4*aRxBuffer1[2]+3);
    HAL_UART_Transmit(&huart4,(uint8_t *)&SendBuf,4*aRxBuffer1[2]+4,1000);   //发送数据给上位机
}



void Agreement_funEF()         //监控器读寄存器（实时）
{
    int j;
    SendBuf[0]=FRAME_START;   //发送数据第一个字节
    SendBuf[1]=0xEF;            //发送数据第二一个字节
    SendBuf[2]=aRxBuffer1[2];           //发送数据第三个字节
    for(j=1; j<=aRxBuffer1[2]; j++)   //循环发送数据
    {
        SendBuf[4*(j-1)+3]=  *PARAMETER[aRxBuffer1[2+j]].lpParam&0xFF;
        SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[2+j]].lpParam>>8)&0xFF;
        SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[2+j]].lpParam>>16)&0XFF;
        SendBuf[4*(j-1)+6]= (*PARAMETER[aRxBuffer1[2+j]].lpParam>>24)&0XFF;
    } 
    SendBuf[4*aRxBuffer1[2]+3]=CheckSum((uint8_t*)&SendBuf[0],4*aRxBuffer1[2]+3);   //发送CRC校验位
    HAL_UART_Transmit(&huart4,(uint8_t *)&SendBuf,4*aRxBuffer1[2]+4,500);   //发送数据给上位机(加了校验位，所以为加4)
}

void Agreement_funF2()             //示波器功能   //波特率115200(bps) ＝ 115200 (位/秒) = 11.25 (KB/秒)= 11520 (字节/秒) 						  
{
	int j;
	SendBuf[0]=FRAME_START;   //发送数据第一个字节
	SendBuf[1]=0xF2;            //发送数据第二一个字节
	for(j=1;j<=2;j++)     //循环发送数据
	{
		SendBuf[4*(j-1)+2]=  *PARAMETER[aRxBuffer1[1+j]].lpParam&0xFF;
		SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>8)&0xFF;
		SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>16)&0XFF;
		SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>24)&0XFF;
	}
	   SendBuf[10]=CheckSum((uint8_t*)&SendBuf[0],10);   //发送CRC校验位
	   HAL_UART_Transmit(&huart4,(uint8_t *)&SendBuf,11,200);   //发送数据给上位机(加了校验位，所以为加4)
}

void Agreement_funF1()             //示波器功能   //波特率115200(bps) ＝ 115200 (位/秒) = 11.25 (KB/秒)= 11520 (字节/秒) 						  
{
	int j;
	SendBuf[0]=FRAME_START;   //发送数据第一个字节
	SendBuf[1]=0xF1;            //发送数据第二一个字节
	for(j=1;j<=1;j++)     //循环发送数据
	{
		SendBuf[4*(j-1)+2]=  *PARAMETER[aRxBuffer1[1+j]].lpParam&0xFF;
		SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>8)&0xFF;
		SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>16)&0XFF;
		SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>24)&0XFF;
	}
	   SendBuf[6]=CheckSum((uint8_t*)&SendBuf[0],6);   //发送CRC校验位
	   HAL_UART_Transmit(&huart4,(uint8_t *)&SendBuf,7,200);   //发送数据给上位机(加了校验位，所以为加4)
}

void Agreement_funF3()             //示波器功能   //波特率115200(bps) ＝ 115200 (位/秒) = 11.25 (KB/秒)= 11520 (字节/秒) 						  
{
	int j;
	SendBuf[0]=FRAME_START;   //发送数据第一个字节
	SendBuf[1]=0xF3;            //发送数据第二一个字节
	for(j=1;j<=3;j++)     //循环发送数据
	{
		SendBuf[4*(j-1)+2]=  *PARAMETER[aRxBuffer1[1+j]].lpParam&0xFF;
		SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>8)&0xFF;
		SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>16)&0XFF;
		SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>24)&0XFF;
	}
	   SendBuf[14]=CheckSum((uint8_t*)&SendBuf[0],14);   //发送CRC校验位
	   HAL_UART_Transmit(&huart4,(uint8_t *)&SendBuf,15,200);   //发送数据给上位机(加了校验位，所以为加4)
}

void Agreement_funF4()             //示波器功能   //波特率115200(bps) ＝ 115200 (位/秒) = 11.25 (KB/秒)= 11520 (字节/秒) 						  
{
	int j;
	SendBuf[0]=FRAME_START;   //发送数据第一个字节
	SendBuf[1]=0xF4;            //发送数据第二一个字节
	for(j=1;j<=4;j++)     //循环发送数据
	{
		SendBuf[4*(j-1)+2]=  *PARAMETER[aRxBuffer1[1+j]].lpParam&0xFF;
		SendBuf[4*(j-1)+3]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>8)&0xFF;
		SendBuf[4*(j-1)+4]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>16)&0XFF;
		SendBuf[4*(j-1)+5]= (*PARAMETER[aRxBuffer1[1+j]].lpParam>>24)&0XFF;
	}
	   SendBuf[18]=CheckSum((uint8_t*)&SendBuf[0],18);   //发送CRC校验位
	   HAL_UART_Transmit(&huart4,(uint8_t *)&SendBuf,19,200);   //发送数据给上位机(加了校验位，所以为加4)
}


void ReturnRight()         //返回正确指令
{
    SendBuf1[0]=FRAME_START;   //发送数据第一个字节
    SendBuf1[1]=0x06;            //发送数据第二一个字节
    SendBuf1[2]=0x00;           //发送数据第三个字节
    SendBuf1[3]=0xF0;
    HAL_UART_Transmit(&huart4,(uint8_t *)&SendBuf1,4,1000);   //发送数据给上位机
}


uint8_t CheckSum(uint8_t *Ptr,uint8_t Num )        //校验数据
{
    uint16_t Sum = 0;
    while(Num--)
    {
        Sum += *Ptr;
        Ptr++;
    }
    Sum=Sum%0x100;               //取余
    return Sum;
}





void ReturnError1()        //发生错误
{
	  uint8_t code0[8]={0x5A,0xE1,0x54,0x00,0x00,0x00,0x01,0x90};  //电机0错误
		uint8_t code1[8]={0x5A,0xE1,0x55,0x00,0x00,0x00,0x01,0x91};  //电机1错误
		uint8_t code2[8]={0x5A,0xE1,0x56,0x00,0x00,0x00,0x01,0x92};  //电机2错误
		uint8_t code3[8]={0x5A,0xE1,0x57,0x00,0x00,0x00,0x01,0x93};  //电机3错误
		uint8_t code4[8]={0x5A,0xE1,0x58,0x00,0x00,0x00,0x01,0x94};  //电机4错误
		uint8_t code5[8]={0x5A,0xE1,0x59,0x00,0x00,0x00,0x01,0x95};  //电机5错误
		uint8_t code6[8]={0x5A,0xE1,0x5A,0x00,0x00,0x00,0x01,0x96};  //电机6错误
		uint8_t code7[8]={0x5A,0xE1,0x5B,0x00,0x00,0x00,0x01,0x97};  //电机7错误
		uint8_t code8[8]={0x5A,0xE1,0x5C,0x00,0x00,0x00,0x01,0x98};  //电机8错误
		uint8_t code9[8]={0x5A,0xE1,0x5D,0x00,0x00,0x00,0x01,0x99};  //电机9错误
		uint8_t code10[8]={0x5A,0xE1,0x5E,0x00,0x00,0x00,0x01,0x9A};  //电机10错误
		uint8_t code11[8]={0x5A,0xE1,0x5F,0x00,0x00,0x00,0x01,0x9B};  //电机11错误
		uint8_t code12[8]={0x5A,0xE1,0x60,0x00,0x00,0x00,0x01,0x9C};  //电机12错误
		uint8_t code13[8]={0x5A,0xE1,0x61,0x00,0x00,0x00,0x01,0x9D};  //电机13错误
		uint8_t err0[8]={0x5A,0xE1,0xB1,0x00,0x00,0x00,0x00,0x00};
		if(wGlobal_Flags!=0)     //全局错误
		 {
			 ErrTime1++;
				if(ErrTime1==100)
		  {  
				ErrTime1=0;
				err0[3]=(wGlobal_Flags>>24)&0xFF;
				err0[4]=(wGlobal_Flags>>16)&0xFF;
				err0[5]=(wGlobal_Flags>>8)&0xFF;
				err0[6]= wGlobal_Flags&0xFF;
				err0[7]=CheckSum((uint8_t*)&err0[0],7);
			  HAL_UART_Transmit(&huart4,(uint8_t *)&err0,8,1000);
			}
		}
		

		if(MotorControl[0].Fault_Flag==1)        //电机0报错
		{
			MotorControl[0].Fault_Cnt++; 
			if(MotorControl[0].Fault_Cnt==100)
			{
				MotorControl[0].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code0,8,1000);
			}
		}
			if(MotorControl[1].Fault_Flag==1)        //电机1报错
		{
			MotorControl[1].Fault_Cnt++; 
			if(MotorControl[1].Fault_Cnt==100)
			{
				MotorControl[1].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code1,8,1000);
			}
		}
		  if(MotorControl[2].Fault_Flag==1)        //电机2报错
		{
			MotorControl[2].Fault_Cnt++; 
			if(MotorControl[2].Fault_Cnt==100)
			{
				MotorControl[2].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code2,8,1000);
			}
		}
		    if(MotorControl[3].Fault_Flag==1)        //电机3报错
		{
			MotorControl[3].Fault_Cnt++; 
			if(MotorControl[3].Fault_Cnt==100)
			{
				MotorControl[3].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code3,8,1000);
			}
		}
		
		  if(MotorControl[4].Fault_Flag==1)        //电机4报错
		{
			MotorControl[4].Fault_Cnt++; 
			if(MotorControl[4].Fault_Cnt==100)
			{
				MotorControl[4].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code4,8,1000);
			}
		}
		
		  if(MotorControl[5].Fault_Flag==1)        //电机5报错
		{
			MotorControl[5].Fault_Cnt++; 
			if(MotorControl[5].Fault_Cnt==100)
			{
				MotorControl[5].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code5,8,1000);
			}
		}
		
		  if(MotorControl[6].Fault_Flag==1)        //电机6报错
		{
			MotorControl[6].Fault_Cnt++; 
			if(MotorControl[6].Fault_Cnt==100)
			{
				MotorControl[6].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code6,8,1000);
			}
		}
		
		    if(MotorControl[7].Fault_Flag==1)        //电机7报错
		{
			MotorControl[7].Fault_Cnt++; 
			if(MotorControl[7].Fault_Cnt==100)
			{
				MotorControl[7].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code7,8,1000);
			}
		}
		if(MotorControl[8].Fault_Flag==1)        //电机8报错
		{
			MotorControl[8].Fault_Cnt++; 
			if(MotorControl[8].Fault_Cnt==100)
			{
				MotorControl[8].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code8,8,1000);
			}
		}
			if(MotorControl[9].Fault_Flag==1)        //电机9报错
		{
			MotorControl[9].Fault_Cnt++; 
			if(MotorControl[9].Fault_Cnt==100)
			{
				MotorControl[9].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code9,8,1000);
			}
		}
		  if(MotorControl[10].Fault_Flag==1)        //电机10报错
		{
			MotorControl[10].Fault_Cnt++; 
			if(MotorControl[10].Fault_Cnt==100)
			{
				MotorControl[10].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code10,8,1000);
			}
		}
		    if(MotorControl[11].Fault_Flag==1)        //电机11报错
		{
			MotorControl[11].Fault_Cnt++; 
			if(MotorControl[11].Fault_Cnt==100)
			{
				MotorControl[11].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code11,8,1000);
			}
		}
		
		  if(MotorControl[12].Fault_Flag==1)        //电机12报错
		{
			MotorControl[12].Fault_Cnt++; 
			if(MotorControl[12].Fault_Cnt==100)
			{
				MotorControl[12].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code12,8,1000);
			}
		}
		
		  if(MotorControl[13].Fault_Flag==1)        //电机13报错
		{
			MotorControl[13].Fault_Cnt++; 
			if(MotorControl[13].Fault_Cnt==100)
			{
				MotorControl[13].Fault_Cnt=0;
			  HAL_UART_Transmit(&huart4,(uint8_t *)&code13,8,1000);
			}
		}
}
void FlashRight()
{
    SendBuf3[0]=FRAME_START;   //发送数据第一个字节
    SendBuf3[1]=0xE2;            //发送数据第二一个字节
    SendBuf3[2]=0x00;           //发送数据第三个字节
    SendBuf3[3]=0xF0;
    HAL_UART_Transmit(&huart4,(uint8_t *)&SendBuf3,4,1000);   //发送数据给上位机
}