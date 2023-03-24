/*
*******************************************************************************
 *版    权： 2021-xxxx,  GaussianRobot
 *文 件 名： usart_labview.c主函数
 *简    介： 用于初始化MCU外设
 *作    者： LMmotor\忘川
 *日    期： 2023.1.3
 *功能描述： 
*******************************************************************************
 *备注：         小部分其他功能：LABVIEW用于打印调试
*******************************************************************************
*/

#include "Usart_Labview.h"
#include "public_global.h"
#include "ds402.h"
#include "stm32f4xx_hal_usart.h"
#include "stdlib.h"
#include "bsp_app_function.h"
char str_table[Rec_Num][Rec_Len]; 
long Lab_RxDate[20] = {0};    
long TempSpeedMotor1=0,TempSpeedMotor2=0;                        /*中间变量*/
float SetPositionMotor1=0,SetPositionMotor2=0;

u16 Uart_TxBuffer_CNT = 0;
u16 Uart_TxBuffer_CNT1 =0;
u8 uart_counter=10;                                              /*用于切换示波器打印参数*/
uint8_t UsartRxBuffer[LABVIEWRXBUFFERSIZE] = {0};                /*串口接收数组*/
uint8_t UsartTxBuffer[LABVIEWRXBUFFERSIZE] = {0};                /*串口发送数组*/
uint8_t Labview_uart_Flag_CNT = 5;                               /*示波器刷新时间*/
uint8_t TempID1 = 27,TempID2 = 28,TempID3 = 30,TempID4 = 31;     /*串口示波器打印的数据号*/
uint8_t RefreshCmd_Type;                                         /*持续刷新界面*/
uint8_t g_uart_recv_ok;                                          /*接收一包数据完成*/
uint8_t Labview_uart_100us_Flag = 1 ;                            /*100us打印数据标志*/
uint8_t Labview_uart_Flag=0;                                     /*1ms自增一次*/
int16_t LV_TxBuffer[10];                                         /*print函数打印的临时缓存*/

extern HALL_Handle_t HALL_M1;
extern LOAD_CURR	loadCurr[NUMBER_OF_AXES];
extern MotorParameters_t MotorParameters[NBR_OF_MOTORS];
extern void MotorParametersM1_Init(void);                        /*参数初始化函数*/
extern void MotorParametersM2_Init(void);
/*提供外部申明用于labview打印参数*/
extern STM_Handle_t STM[2];        
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDSpeed[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PosCtrl_Handle_t *pPosCtrl[NBR_OF_MOTORS];
extern int32_t hSpeedRef_Pos ;
extern PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
extern FOCVars_t FOCVars[NBR_OF_MOTORS];


#ifdef UART_DEBUG
#define LV_TxBufnum 4010
int16_t LV_Uart_TxBuffer[4][LV_TxBufnum] = {0};
#endif

#ifdef Usart_Labview
long LabView_Uart_TxBuffer[4][LabView_Uart_TxBuffersize] = {0};
#endif

void Usart_Labview_Analyze(void)
{
    uint32_t dmarequest = 0x00U;
	  static uint8_t entemp1 = 0,entemp2 = 0;                                    /*使能失能函数判断是否使能成功*/
	  static uint8_t UsartState = 0;                                             /*状态机切换*/
    int i;
    switch (UsartState)
    {
    case 0x00:                                                                 /*等待接收完成标志*/
        if(g_uart_recv_ok)
        {
            g_uart_recv_ok = 0;
            UsartState = UsartRxBuffer[1];                                     /*接收数组中第一个数据，为功能码*/
        }
        break;
    case 0xFE:  
        UsartState = 0x00;
        break;
    case 0x01:                                                                 /*电机1参数读取指令（电机1基本参数、电机1保护参数、电机1PID参数）*/
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x81;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+2] = (*PARAMETER[43].lpParam>>(i*8))&0xFF;         /*极对数*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+6] = (*PARAMETER[96].lpParam>>(i*8))&0xFF;         /*力矩常数*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+10] = (*PARAMETER[97].lpParam>>(i*8))&0xFF;        /*电机惯量*/
        }

        TempSpeedMotor1 = *PARAMETER[98].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+14] = (TempSpeedMotor1>>(i*8))&0xFF;               /*额定电流*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+18] = (*PARAMETER[99].lpParam>>(i*8))&0xFF;        /*额定转矩*/
        }
        TempSpeedMotor1 = *PARAMETER[100].lpParam*0.1;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+22] = (TempSpeedMotor1>>(i*8))&0xFF;               /*额定转速*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+26] = (*PARAMETER[101].lpParam>>(i*8))&0xFF;       /*编码器分辨率*/
        }
        TempSpeedMotor1 = *PARAMETER[102].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+30] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1峰值电流*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+34] = (*PARAMETER[103].lpParam>>(i*8))&0xFF;        /*M1峰值电流持续时间*/
        }

        TempSpeedMotor1 = *PARAMETER[104].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+38] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1最大相电流*/
        }
        TempSpeedMotor1 = *PARAMETER[105].lpParam*0.1;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+42] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1最大速度*/
        }
        TempSpeedMotor1 = *PARAMETER[106].lpParam*0.1;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+46] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1堵转速度*/
        }

        TempSpeedMotor1 = *PARAMETER[107].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+50] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1堵转电流*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+54] = (*PARAMETER[108].lpParam>>(i*8))&0xFF;        /*M1堵转时间*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+58] = (*PARAMETER[9].lpParam>>(i*8))&0xFF;          /*M1电流环P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+62] = (*PARAMETER[10].lpParam>>(i*8))&0xFF;         /*M1电流环I*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+66] = (*PARAMETER[7].lpParam>>(i*8))&0xFF;          /*M1速度环P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+70] = (*PARAMETER[8].lpParam>>(i*8))&0xFF;          /*M1速度环P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+74] = (*PARAMETER[1].lpParam>>(i*8))&0xFF;          /*M1位置环P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+78] = (*PARAMETER[2].lpParam>>(i*8))&0xFF;          /*M1位置环I*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+82] = (*PARAMETER[3].lpParam>>(i*8))&0xFF;          /*M1位置环D*/
        }
        UsartTxBuffer[86] = (int8_t)(*PARAMETER[49].lpParam);                   /*pll速度环开关*/
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+87] = (*PARAMETER[60].lpParam>>(i*8))&0xFF;         /*M1pllKp*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+91] = (*PARAMETER[61].lpParam>>(i*8))&0xFF;         /*M1pllKi*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+95] = (*PARAMETER[91].lpParam>>(i*8))&0xFF;         /*暂无*/
        }
        UsartTxBuffer[99] = 0x2F;
        HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 100);
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x02:                                                                  /*电机1参数读取指令（电机1基本参数、电机1保护参数、电机1PID参数）*/
      
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x84;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+2] = (*PARAMETER[44].lpParam>>(i*8))&0xFF;          /*M2极对数*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+6] = (*PARAMETER[110].lpParam>>(i*8))&0xFF;         /*M2力矩常数*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+10] = (*PARAMETER[111].lpParam>>(i*8))&0xFF;        /*M2电机惯量*/
        }


        TempSpeedMotor2 = *PARAMETER[112].lpParam/ADC_AMPER_Ratio;     
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+14] = (TempSpeedMotor2>>(i*8))&0xFF;                /*额定电流*/
        }


        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+18] = (*PARAMETER[113].lpParam>>(i*8))&0xFF;        /*额定转矩*/
        }
        TempSpeedMotor2 = *PARAMETER[114].lpParam*0.1;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+22] = (TempSpeedMotor2>>(i*8))&0xFF;                /*额定速度*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+26] = (*PARAMETER[115].lpParam>>(i*8))&0xFF;        /*编码器分辨率*/
        }
        TempSpeedMotor2 = *PARAMETER[116].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+30] = (TempSpeedMotor2>>(i*8))&0xFF;                /*峰值电流*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+34] = (*PARAMETER[117].lpParam>>(i*8))&0xFF;        /*峰值电流持续时间*/
        }

        TempSpeedMotor2 = *PARAMETER[118].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+38] = (TempSpeedMotor2>>(i*8))&0xFF;                /*最大相电流*/
        }

        TempSpeedMotor2 = *PARAMETER[38].lpParam*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+42] = (TempSpeedMotor2>>(i*8))&0xFF;                /*最大速度*/
        }

        TempSpeedMotor2 = *PARAMETER[120].lpParam*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+46] = (TempSpeedMotor2>>(i*8))&0xFF;                /*堵转速度*/
        }

        TempSpeedMotor2 = *PARAMETER[121].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+50] = (TempSpeedMotor2>>(i*8))&0xFF;                /*堵转电流*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+54] = (*PARAMETER[122].lpParam>>(i*8))&0xFF;        /*堵转时间*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+58] = (*PARAMETER[19].lpParam>>(i*8))&0xFF;         /*P_current*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+62] = (*PARAMETER[20].lpParam>>(i*8))&0xFF;         /*I_current*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+66] = (*PARAMETER[17].lpParam>>(i*8))&0xFF;         /*P_speed*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+70] = (*PARAMETER[18].lpParam>>(i*8))&0xFF;         /*I_speed*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+74] = (*PARAMETER[11].lpParam>>(i*8))&0xFF;         /*P_position*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+78] = (*PARAMETER[12].lpParam>>(i*8))&0xFF;         /*I_position*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+82] = (*PARAMETER[13].lpParam>>(i*8))&0xFF;         /*D_position*/
        }
        UsartTxBuffer[86] = (int8_t)(*PARAMETER[49].lpParam);                   /*pll速度环开关*/
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+87] = (*PARAMETER[58].lpParam>>(i*8))&0xFF;         /*PLL_P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+91] = (*PARAMETER[59].lpParam>>(i*8))&0xFF;         /*PLL_I*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+95] = (*PARAMETER[90].lpParam>>(i*8))&0xFF;        /*暂无*/
        }
        UsartTxBuffer[99] = 0x2F;
        HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 100);
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x03:                                                                /*参数监控界面*/
        RefreshCmd_Type = 1;                                                  /*数据刷新*/
        UsartState = 0xFE;
        break;
    case 0x04:                                                                /*电机使能*/
        if(UsartRxBuffer[2] == 3)                                             /*使能两个电机*/
        {
            entemp1 = Motor_EN_OFF(1,M1);                                     /*使能成功返回1*/
            entemp2 = Motor_EN_OFF(1,M2);                                     /*使能成功返回1*/
            if(entemp1+entemp2 == 2)                                          /*如果两个电机都使能成功*/
            {
                entemp1 = 0;
                entemp2 = 0;
                RefreshCmd_Type = 0;
                UsartState = 0xFE;
            }
            else
            {
            }
        }
        else if(UsartRxBuffer[2] == 0x01||UsartRxBuffer[2] == 0x02)          /*使能单个电机*/
        {
            entemp1 = Motor_EN_OFF(1,UsartRxBuffer[2]-1);             
            if(entemp1)
            {
                entemp1 = 0;
                RefreshCmd_Type = 0;
                UsartState = 0xFE;
            }
            else
            {
               
            }
        }
        else
        {
            RefreshCmd_Type = 0;
            UsartState = 0xFE;
        }
        break;
    case 0x05:                                                             /*使能指令*/
        if(UsartRxBuffer[2] == 3)                                          /*失能两个电机*/
        {
            entemp1 = Motor_EN_OFF(0,M1);                                  /*失能M1*/
            entemp2 =	Motor_EN_OFF(0,M2);                                  /*失能M2*/
            if(entemp1+entemp2 == 2)
            {
                entemp1 = 0;
                entemp2 = 0;
                RefreshCmd_Type = 0;
                UsartState = 0xFE;
            }
            else
            {
                
            }
        }
        else if(UsartRxBuffer[2] == 0x01||UsartRxBuffer[2] == 0x02)        /*失能单个电机*/
        {
            entemp1 = Motor_EN_OFF(0,UsartRxBuffer[2]-1);
            if(entemp1)                                                    /*失能成功*/
            {
                entemp1 = 0;
                RefreshCmd_Type = 0;
                UsartState = 0xFE;
            }
            else
            {
                /*错误处理*/
            }
        }
        else
        {
            RefreshCmd_Type = 0;
            UsartState = 0xFE;
        }
        break;
    case 0x06:                                                             /*电机急停指令，只能在速度模式下使用的急停*/
        pCtrlPar[M1].SetVelMotor = 0;
        pCtrlPar[M2].SetVelMotor = 0;
        pCtrlPar[M1].M1M2_VelSet = 0;
        MC_ProgramSpeedRampMotor1( pCtrlPar[M1].SetVelMotor, 50 );
        MC_ProgramSpeedRampMotor2( pCtrlPar[M2].SetVelMotor, 50 );
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x07:                                                             /*电机换向指令*/
        if(UsartRxBuffer[2] == 0x01 )                                      /*电机1换向模式选择*/
        {
            if(UsartRxBuffer[3] == 0x01)
            {
                HallStudyFlag1 = 2;	                                       /*编码器模式*/
            }
            else if(UsartRxBuffer[3] == 0x02)
            {
                HallStudyFlag1 = 1;                                        /*霍尔学习模式*/
            }
        }
        else if(UsartRxBuffer[2] == 0x02)                                  /*电机2换向模式选择*/
        {
            if(UsartRxBuffer[3] == 0x01)
            {
                HallStudyFlag2 = 2;                                        /*编码器模式*/
            }
            else if(UsartRxBuffer[3] == 0x02)
            {
                HallStudyFlag2 = 1;                                        /*霍尔学习模式*/
            }
        }
        else if(UsartRxBuffer[2] == 0x03)                                  /*电机1和2换向模式选择*/
        {
            if(UsartRxBuffer[3] == 0x01)
            {
                HallStudyFlag1 = 2;                                        /*编码器模式*/
                HallStudyFlag2 = 2;
            }
            else if(UsartRxBuffer[3] == 0x02)
            {
                HallStudyFlag1 = 1;                                        /*霍尔学习模式*/
                HallStudyFlag2 = 1;
            }
        }
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x08:                                                             /*电机运行模式*/
        if(UsartRxBuffer[2] == 0x03 )                                      /*两个电机一起*/
        {
            Switch_Motor_Operation_Mode(UsartRxBuffer[3],M1);
            Switch_Motor_Operation_Mode(UsartRxBuffer[3],M2);
        }
        else if( UsartRxBuffer[2] == 0x01||UsartRxBuffer[2] == 0x02 )
        {
            Switch_Motor_Operation_Mode(UsartRxBuffer[3],UsartRxBuffer[2]-1);

        }
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x09:                                                            /*报警清除*/
        Driver_Data[M1].device_control_data.Conrtolword = 0x80;
        RefreshCmd_Type = 1;
        UsartState = 0xFE;
        break;
    case 0x0A:                                                            /*独立参数读取*/
        if(UsartRxBuffer[2] == 0)                                         /*读取ID*/
        {
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[UsartRxBuffer[3]].lpParam>>(i*8))&0xFF;   /*ID数据发送*/
            }
            UsartTxBuffer[0] = 0xAA;
            UsartTxBuffer[1] = 0x8A;
            UsartTxBuffer[6] = 0x2F;
            HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 7);
        }
        else if(UsartRxBuffer[2] == 1)                                    /*写入ID*/
        {
            *PARAMETER[UsartRxBuffer[3]].lpParam = Usart_ReadRegValue(UsartRxBuffer,4);
        }
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x0B:                                                           /*读取力矩参数*/
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x87;
        switch (UsartRxBuffer[2])
        {
        case 0x01 :
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[50].lpParam>>(i*8))&0xFF;/*电机1目标力矩*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[53].lpParam>>(i*8))&0xFF;/*电机2目标力矩*/
            }
            UsartTxBuffer[10] = 0x2F;
            HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 11);
            break;
        case 0x02:
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[50].lpParam>>(i*8))&0xFF;/*电机2目标力矩*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[53].lpParam>>(i*8))&0xFF;/*电机2目标力矩*/
            }
            UsartTxBuffer[10] = 0x2F;
            HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 11);
            break;
        case 0x03:  //!!!待定
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[50].lpParam>>(i*8))&0xFF;/*电机1目标力矩*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[53].lpParam>>(i*8))&0xFF;/*电机2目标力矩*/
            }
            UsartTxBuffer[10] = 0x2F;
            HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 11);
            break;
        default:
            break;
        }
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x0C: /*读取速度参数*/
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x88;
        TempSpeedMotor1 = (long)((float)(*PARAMETER[27].lpParam/10));
        TempSpeedMotor2 = (long)((float)(*PARAMETER[30].lpParam/10));
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+2] = (TempSpeedMotor1>>(i*8))&0xFF;             /*电机1目标速度*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+6] = (*PARAMETER[29].lpParam>>(i*8))&0xFF;      /*电机1目标时间*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+10] = (TempSpeedMotor2>>(i*8))&0xFF;            /*电机2目标速度*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+14] = (*PARAMETER[32].lpParam>>(i*8))&0xFF;     /*电机2目标时间 改20201207*/
        }
        UsartTxBuffer[18] = 0x2F;
        HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 19);
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;

    case 0x0D:                                                             /*读取位置参数*/
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x89;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+2] = (*PARAMETER[21].lpParam>>(i*8))&0xFF;     /*电机1目标位置*/
        }
        TempSpeedMotor1 = (*PARAMETER[6].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+6] = (TempSpeedMotor1>>(i*8))&0xFF;            /*电机1位置环目标速度*/
        }
        TempSpeedMotor1 = (*PARAMETER[4].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+10] = (TempSpeedMotor1>>(i*8))&0xFF;           /*电机1目标加速度*/
        }
        TempSpeedMotor1 = (*PARAMETER[5].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+14] = (TempSpeedMotor1>>(i*8))&0xFF;           /*电机1目标减速度*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+18] = (*PARAMETER[24].lpParam>>(i*8))&0xFF;    /*电机2目标位置*/
        }
        TempSpeedMotor2 = (*PARAMETER[16].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+22] = (TempSpeedMotor2>>(i*8))&0xFF;           /*电机2位置环目标速度*/
        }
        TempSpeedMotor2 = (*PARAMETER[14].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+26] = (TempSpeedMotor2>>(i*8))&0xFF;           /*电机2目标加速度*/
        }
        TempSpeedMotor2 = (*PARAMETER[15].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+30] = (TempSpeedMotor2>>(i*8))&0xFF;           /*电机2目标减速度*/
        }

        UsartTxBuffer[34] = 0x2F;
        HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 35);
        RefreshCmd_Type = 0;
        UsartState = 0xFE;



        break;
    case 0x0E:                                                           /*未起作用，暂留*/
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x0F:                                                          /*同步模式开关*/
        Sync_Async_Control = (uint16_t)UsartRxBuffer[2];
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x11:                                                          /*设置电机1基本配置参数*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);    /*接收数据处理*/
        }

        *PARAMETER[95].lpParam = Lab_RxDate[0];                          /*极对数*/
        *PARAMETER[96].lpParam = Lab_RxDate[1];                          /*力矩常数*/
        *PARAMETER[97].lpParam = Lab_RxDate[2];                          /*电机惯量*/
        *PARAMETER[98].lpParam = (long)(Lab_RxDate[3]);                  /*额定电流*/
        *PARAMETER[99].lpParam = Lab_RxDate[4];                          /*额定转矩*/
        *PARAMETER[100].lpParam = Lab_RxDate[5];                         /*额定转速*/
        *PARAMETER[101].lpParam = Lab_RxDate[6];                         /*编码器分辨率*/
        MotorParametersM1_Init();
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x12:                                                           /*设置电机1保护参数*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }

        *PARAMETER[102].lpParam = (long)(Lab_RxDate[0]);                 /*峰值电流*/
        *PARAMETER[103].lpParam = Lab_RxDate[1];                         /*峰值电流持续时间*/
        *PARAMETER[104].lpParam = Lab_RxDate[2];                         /*最大相电流*/
        *PARAMETER[105].lpParam = Lab_RxDate[3];	                       /*最大速度*/
        *PARAMETER[106].lpParam = Lab_RxDate[4];                         /*堵转速度*/
        *PARAMETER[107].lpParam = (long)(Lab_RxDate[5]);                 /*堵转电流*/
        *PARAMETER[108].lpParam = Lab_RxDate[6];                         /*堵转时间*/

        MotorParametersM1_Init();
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x13:                                                           /*设置电机1PID参数*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }
        Lab_RxDate[7] = UsartRxBuffer[30];                               /*PLL开关*/
        Lab_RxDate[8] = Usart_ReadRegValue(UsartRxBuffer,31);
        Lab_RxDate[9] = Usart_ReadRegValue(UsartRxBuffer,35);
        Lab_RxDate[10] = Usart_ReadRegValue(UsartRxBuffer,39);
        *PARAMETER[9].lpParam = Lab_RxDate[0];                           /*Kp_Iq*/
        *PARAMETER[10].lpParam = Lab_RxDate[1];                          /*Ki_Iq*/
        *PARAMETER[7].lpParam = Lab_RxDate[2];                           /*Kp_Speed*/
        *PARAMETER[8].lpParam = Lab_RxDate[3];	                         /*Ki_Speed*/
        *PARAMETER[1].lpParam = Lab_RxDate[4];                           /*Kp_Position*/
        *PARAMETER[2].lpParam = Lab_RxDate[5];                           /*Ki_Position*/
        *PARAMETER[3].lpParam = Lab_RxDate[6];                           /*Kd_Position*/
        *PARAMETER[49].lpParam = Lab_RxDate[7];                          /*PLL开关待定*/
        *PARAMETER[60].lpParam = Lab_RxDate[8];                          /*Kp_PLL*/
        *PARAMETER[61].lpParam = Lab_RxDate[9];                          /*Kd_PLL*/
        *PARAMETER[91].lpParam = Lab_RxDate[10];                         /*DIV_PLL*/
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x14:                                                           /*设置电机2基本配置参数*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }
        *PARAMETER[109].lpParam = Lab_RxDate[0];                          /*极对数*/
        *PARAMETER[110].lpParam = Lab_RxDate[1];                          /*力矩常数*/
        *PARAMETER[111].lpParam = Lab_RxDate[2];                          /*电机惯量*/
        *PARAMETER[112].lpParam = (long)(Lab_RxDate[3]*ADC_AMPER_Ratio);	/*额定电流*/
        *PARAMETER[113].lpParam = Lab_RxDate[4];  	                      /*额定转矩*/
        *PARAMETER[114].lpParam = Lab_RxDate[5];                          /*额定转速*/
        *PARAMETER[115].lpParam = Lab_RxDate[6]; 	                        /*编码器分辨率*/

        MotorParametersM2_Init();
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x15:     	/**/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }
        *PARAMETER[116].lpParam = (long)(Lab_RxDate[0]);                  /*峰值电流*/
        *PARAMETER[117].lpParam = Lab_RxDate[1];                          /*峰值电流持续时间*/
        *PARAMETER[118].lpParam = Lab_RxDate[2]; 	                        /*最大相电流*/
        *PARAMETER[38].lpParam = Lab_RxDate[3];		                        /*最大速度*/
        *PARAMETER[120].lpParam = Lab_RxDate[4]; 	                        /*堵转速度*/
        *PARAMETER[121].lpParam = (long)(Lab_RxDate[5]);                  /*堵转电流*/
        *PARAMETER[122].lpParam = Lab_RxDate[6];                          /*堵转时间*/

        MotorParametersM2_Init();
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x16:                                                           	/*设置电机2PID参数*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }
        Lab_RxDate[7] = UsartRxBuffer[30];                                /*PLL开关*/
        Lab_RxDate[8] = Usart_ReadRegValue(UsartRxBuffer,31);
        Lab_RxDate[9] = Usart_ReadRegValue(UsartRxBuffer,35);
        Lab_RxDate[10] = Usart_ReadRegValue(UsartRxBuffer,39);
        *PARAMETER[19].lpParam = Lab_RxDate[0];                           /*Kp_Iq*/
        *PARAMETER[20].lpParam = Lab_RxDate[1];                           /*Ki_Iq*/
        *PARAMETER[17].lpParam = Lab_RxDate[2];                           /*Kp_Speed*/
        *PARAMETER[18].lpParam = Lab_RxDate[3];	                          /*Ki_Speed*/
        *PARAMETER[11].lpParam = Lab_RxDate[4];                           /*Kp_Position*/
        *PARAMETER[12].lpParam = Lab_RxDate[5];                           /*Ki_Position*/
        *PARAMETER[13].lpParam = Lab_RxDate[6];                           /*Kd_Position*/
        *PARAMETER[49].lpParam = Lab_RxDate[7];                           /*PLL开关待定*/
        *PARAMETER[58].lpParam = Lab_RxDate[8];                           /*Kp_PLL*/
        *PARAMETER[59].lpParam = Lab_RxDate[9];                           /*Kd_PLL*/
        *PARAMETER[90].lpParam = Lab_RxDate[10];                          /*DIV_PLL*/
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x17:                                                            /*设置电机力矩参数*/
        if(UsartRxBuffer[2] == 0x01)                                      /*电机1*/
        {
            *PARAMETER[50].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
        }
        else if(UsartRxBuffer[2] == 0x02)                                 /*电机2*/
        {
            *PARAMETER[53].lpParam = Usart_ReadRegValue(UsartRxBuffer,7); /*修改motor2数据位202021205*/
        }
        else if(UsartRxBuffer[2] == 0x03)                                 /*电机1-2*/
        {
            *PARAMETER[50].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
            *PARAMETER[53].lpParam = Usart_ReadRegValue(UsartRxBuffer,7); /*增加motor2数据位202021205*/
        }
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x18:			                                                      /*设置点击速度参数*/
        if(UsartRxBuffer[2] == 0x01)                                      /*电机1*/
        {
            *PARAMETER[27].lpParam = Usart_ReadRegValue(UsartRxBuffer,3)*10;
            *PARAMETER[29].lpParam = Usart_ReadRegValue(UsartRxBuffer,7);
        }
        else if(UsartRxBuffer[2] == 0x02)                                 /*电机2*/
        {
            *PARAMETER[30].lpParam = Usart_ReadRegValue(UsartRxBuffer,11)*10;
            *PARAMETER[32].lpParam = Usart_ReadRegValue(UsartRxBuffer,15);
        }
        else if(UsartRxBuffer[2] == 0x03)                                 /*电机1-2*/
        {
            *PARAMETER[27].lpParam = Usart_ReadRegValue(UsartRxBuffer,3)*10;
            *PARAMETER[30].lpParam = Usart_ReadRegValue(UsartRxBuffer,11)*10;
            *PARAMETER[29].lpParam = Usart_ReadRegValue(UsartRxBuffer,7);
            *PARAMETER[32].lpParam = Usart_ReadRegValue(UsartRxBuffer,15);
            if(Sync_Async_Control ==1) 
            {
                *PARAMETER[23].lpParam =(((*PARAMETER[30].lpParam&0x0000FFFF)<<16)|(*PARAMETER[27].lpParam&0x0000FFFF));
            }
        }
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x19:                                                                           /*设置电机位置参数*/
        if(UsartRxBuffer[2] == 0x01)                                                     /*电机1*/
        {
            *PARAMETER[21].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);                /*目标位置*/
            *PARAMETER[6].lpParam = Usart_ReadRegValue(UsartRxBuffer,7)/0.6;             /*目标速度*/
            *PARAMETER[4].lpParam = Usart_ReadRegValue(UsartRxBuffer,11)/0.6;            /*加速度*/
            *PARAMETER[5].lpParam = Usart_ReadRegValue(UsartRxBuffer,15)/0.6;            /*减速度*/
            SetPositionMotor1 = pCtrlPar[M1].SetPulseMotor*RAD_PULSE/M1_ENCODER_PPR/16;  /*增加 1000*pi/M1_ENCODER_PPR/2*/
        }
        else if(UsartRxBuffer[2] == 0x02)                                                /*电机2*/
        {
            *PARAMETER[24].lpParam = Usart_ReadRegValue(UsartRxBuffer,19);               /*目标位置*/
            *PARAMETER[16].lpParam = Usart_ReadRegValue(UsartRxBuffer,23)/0.6;           /*目标速度*/
            *PARAMETER[14].lpParam = Usart_ReadRegValue(UsartRxBuffer,27)/0.6;           /*加速度*/
            *PARAMETER[15].lpParam = Usart_ReadRegValue(UsartRxBuffer,31)/0.6;           /*减速度*/
            SetPositionMotor2 = pCtrlPar[M2].SetPulseMotor*RAD_PULSE/M2_ENCODER_PPR/16;  /*增加 1000*pi/M1_ENCODER_PPR/2*/
        }
        else if(UsartRxBuffer[2] == 0x03)                                                /*电机1-2*/
        {
            *PARAMETER[21].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
            *PARAMETER[24].lpParam = Usart_ReadRegValue(UsartRxBuffer,19);
            *PARAMETER[6].lpParam = Usart_ReadRegValue(UsartRxBuffer,7)/0.6;
            *PARAMETER[4].lpParam = Usart_ReadRegValue(UsartRxBuffer,11)/0.6;
            *PARAMETER[5].lpParam = Usart_ReadRegValue(UsartRxBuffer,15)/0.6;
            *PARAMETER[16].lpParam = Usart_ReadRegValue(UsartRxBuffer,23)/0.6;
            *PARAMETER[14].lpParam = Usart_ReadRegValue(UsartRxBuffer,27)/0.6;
            *PARAMETER[15].lpParam = Usart_ReadRegValue(UsartRxBuffer,31)/0.6;
            SetPositionMotor1 = pCtrlPar[M1].SetPulseMotor*RAD_PULSE/M1_ENCODER_PPR/16;  
            SetPositionMotor2 = pCtrlPar[M2].SetPulseMotor*RAD_PULSE/M2_ENCODER_PPR/16;  
        }
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x1A:                                                                           /*设置电机换向参数（换向电流，仅设置霍尔传感器！）*/
        if(UsartRxBuffer[2] == 0x01)                                                     /*电机1*/
        {
            *PARAMETER[56].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
        }
        else if(UsartRxBuffer[2] == 0x02)                                                /*电机2*/
        {
            *PARAMETER[57].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
        }
        else if(UsartRxBuffer[2] == 0x03)                                                /*电机1-2*/
        {
            *PARAMETER[56].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
            *PARAMETER[57].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
        }
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x1B:                                                                           /*下发示波器参数*/
        Labview_uart_100us_Flag = 0 ;
        switch(UsartRxBuffer[2])
        {
        case 0:
            TempID1 = 70;
            break;
        case 1:
            TempID1 = 74;
            break;
        case 2:
            TempID1 = 129;
            break;
        case 3:
            TempID1 = 28;
            break;
        case 4:
            TempID1 = 21;
            break;
        case 5:
            TempID1 = 22;
            break;
        case 6:
            TempID1 = 71;
            break;
        case 7:
            TempID1 = 75;
            break;
        case 8:
            TempID1 = 130;
            break;
        case 9:
            TempID1 = 31;
            break;
        case 10:
            TempID1 = 24;
            break;
        case 11:
            TempID1 = 25;
            break;
        default:
            break;
        }
        switch(UsartRxBuffer[3])
        {
        case 0:
            TempID2 = 70;
            break;
        case 1:
            TempID2 = 74;
            break;
        case 2:
            TempID2 = 129;
            break;
        case 3:
            TempID2 = 28;
            break;
        case 4:
            TempID2 = 21;
            break;
        case 5:
            TempID2 = 22;
            break;
        case 6:
            TempID2 = 71;
            break;
        case 7:
            TempID2 = 75;
            break;
        case 8:
            TempID2 = 130;
            break;
        case 9:
            TempID2 = 31;
            break;
        case 10:
            TempID2 = 24;
            break;
        case 11:
            TempID2 = 25;
            break;
        default:
            break;
        }
        switch(UsartRxBuffer[4])
        {
        case 0:
            TempID3 = 70;
            break;
        case 1:
            TempID3 = 74;
            break;
        case 2:
            TempID3 = 129;
            break;
        case 3:
            TempID3 = 28;
            break;
        case 4:
            TempID3 = 21;
            break;
        case 5:
            TempID3 = 22;
            break;
        case 6:
            TempID3 = 71;
            break;
        case 7:
            TempID3 = 75;
            break;
        case 8:
            TempID3 = 130;
            break;
        case 9:
            TempID3 = 31;
            break;
        case 10:
            TempID3 = 24;
            break;
        case 11:
            TempID3 = 25;
            break;
        default:
            break;
        }
        switch(UsartRxBuffer[5])
        {
        case 0:
            TempID4 = 70;
            break;
        case 1:
            TempID4 = 74;
            break;
        case 2:
            TempID4 = 129;
            break;
        case 3:
            TempID4 = 28;
            break;
        case 4:
            TempID4 = 21;
            break;
        case 5:
            TempID4 = 22;
            break;
        case 6:
            TempID4 = 71;
            break;
        case 7:
            TempID4 = 75;
            break;
        case 8:
            TempID4 = 130;
            break;
        case 9:
            TempID4 = 31;
            break;
        case 10:
            TempID4 = 24;
            break;
        case 11:
            TempID4 = 25;
            break;
        default:
            break;
        }
        UsartState = 0xFE;
        RefreshCmd_Type = 2;
        break;
    case 0x1C:                                               /*下发示波器ID参数*/
        TempID1 = UsartRxBuffer[2];
        TempID2 = UsartRxBuffer[3];
        TempID3 = UsartRxBuffer[4];
        TempID4 = UsartRxBuffer[5];
        UsartState = 0xFE;
        RefreshCmd_Type = 2;

        Uart_TxBuffer_CNT = 0 ;
        Uart_TxBuffer_CNT1 =0 ;
        Labview_uart_100us_Flag = 1 ;
        if(TempID1 == TempID2 && TempID3 == TempID2 && TempID3 == TempID4)
        {
            Labview_uart_100us_Flag = 0 ;
        }
        break;
    case 0x1D:                                               /*flash参数保存*/
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x1E:                                               /*上传参数表变量*/
        DualDrv_Parameter_Upload();
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x1F:                                               /*下发参数表变量*/
        DualDrv_Parameter_Download();
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    }


    if(RefreshCmd_Type == 2)	                               /*示波器界面*/
    {
        if(Labview_uart_Flag >= Labview_uart_Flag_CNT)       /*5ms*/
        {
            Labview_uart_Flag = 0;
            UsartTxBuffer[0] = 0xAA;
            UsartTxBuffer[1] = 0x80;

            if(Labview_uart_100us_Flag == 1)
            {
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+2] = (LabView_Uart_TxBuffer[0][Uart_TxBuffer_CNT]>>(i*8))&0xFF;/*电机1实际速度*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+6] = (LabView_Uart_TxBuffer[1][Uart_TxBuffer_CNT]>>(i*8))&0xFF;/*电机1实际电流*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+10] = (LabView_Uart_TxBuffer[2][Uart_TxBuffer_CNT]>>(i*8))&0xFF;/*电机1实际位置!*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+14] = (LabView_Uart_TxBuffer[3][Uart_TxBuffer_CNT]>>(i*8))&0xFF;/*电机2实际速度*/
                }
                UsartTxBuffer[18] = 0x2F;
								
                dmarequest = HAL_IS_BIT_SET(husart_debug.Instance->CR3, USART_CR3_DMAT);            /*此处为hal库的bug暂时没有好的办法解决，如果需要持续DMA发送就需要该段代码*/
                if ((husart_debug.gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
                {
                    CLEAR_BIT(husart_debug.Instance->CR3, USART_CR3_DMAT);                          /* 终止DMA数据流 */
                  
                    if (husart_debug.hdmatx != NULL)
                    {
                        HAL_DMA_Abort(husart_debug.hdmatx);
                    }
                    CLEAR_BIT(husart_debug.Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
                     
                    husart_debug.gState = HAL_UART_STATE_READY;                                     /* At end of Tx process, restore huart->gState to Ready */
                }
                HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 19);
                Uart_TxBuffer_CNT++;
                if(Uart_TxBuffer_CNT>LabView_Uart_TxBuffersize-1)
                {
                    Uart_TxBuffer_CNT = 0 ;
                    Uart_TxBuffer_CNT1 =0 ;
                }
            }
            else
            {
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+2] = (*PARAMETER[TempID1].lpParam>>(i*8))&0xFF;                /*电机1实际速度*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+6] = (*PARAMETER[TempID2].lpParam>>(i*8))&0xFF;                /*电机1实际电流*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+10] = (*PARAMETER[TempID3].lpParam>>(i*8))&0xFF;                /*电机1实际位置!*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+14] = (*PARAMETER[TempID4].lpParam>>(i*8))&0xFF;                /*电机2实际速度*/
                }
                UsartTxBuffer[18] = 0x2F;
								
                dmarequest = HAL_IS_BIT_SET(husart_debug.Instance->CR3, USART_CR3_DMAT);            /*此处为hal库的bug暂时没有好的办法解决，如果需要持续DMA发送就需要该段代码*/
                if ((husart_debug.gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
                {
                    CLEAR_BIT(husart_debug.Instance->CR3, USART_CR3_DMAT);                          /* Abort the UART DMA Tx stream */
                    
                    if (husart_debug.hdmatx != NULL)
                    {
                        HAL_DMA_Abort(husart_debug.hdmatx);
                    }
                    CLEAR_BIT(husart_debug.Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
									
                    husart_debug.gState = HAL_UART_STATE_READY;                                     /* At end of Tx process, restore huart->gState to Ready */
                }
                HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 19);
            }

        }
    }
    else if(RefreshCmd_Type == 1)                                         /*监控界面*/
    {
        if(Labview_uart_Flag > Labview_uart_Flag_CNT*5)                   /*25ms*/
        {
            Labview_uart_Flag = 0;
            UsartTxBuffer[0] = 0xAA;
            UsartTxBuffer[1] = 0x90;
            TempSpeedMotor1 = *PARAMETER[28].lpParam*0.1;                 /*缩小10倍单位为rpm*/
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (TempSpeedMotor1>>(i*8))&0xFF;       /*电机1实际速度*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[74].lpParam>>(i*8))&0xFF;/*电机1实际电流*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+10] = (*PARAMETER[22].lpParam>>(i*8))&0xFF;/*电机1实际位置!*/
            }
            TempSpeedMotor2 = *PARAMETER[31].lpParam*0.1;
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+14] = (TempSpeedMotor2>>(i*8))&0xFF;       /*电机2实际速度*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+18] = (*PARAMETER[75].lpParam>>(i*8))&0xFF;/*电机2实际电流*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+22] = (*PARAMETER[25].lpParam>>(i*8))&0xFF;/*电机2实际位置*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+26] = (*PARAMETER[86].lpParam>>(i*8))&0xFF;/*电机1状态*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+30] = (*PARAMETER[87].lpParam>>(i*8))&0xFF;/*电机2状态（新增）*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+34] = (*PARAMETER[94].lpParam>>(i*8))&0xFF;/*母线电压！*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+38] = (*PARAMETER[92].lpParam>>(i*8))&0xFF;/*电机1温度！*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+42] = (*PARAMETER[93].lpParam>>(i*8))&0xFF;/*电机2温度！*/
            }
            UsartTxBuffer[46] = 0x2F;
						
            dmarequest = HAL_IS_BIT_SET(husart_debug.Instance->CR3, USART_CR3_DMAT); /*此处为hal库的bug暂时没有好的办法解决*/
            if ((husart_debug.gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
            {
                CLEAR_BIT(husart_debug.Instance->CR3, USART_CR3_DMAT);
							
                if (husart_debug.hdmatx != NULL)                            /* Abort the UART DMA Tx stream */
                {
                    HAL_DMA_Abort(husart_debug.hdmatx);
                }
                CLEAR_BIT(husart_debug.Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));
								
                husart_debug.gState = HAL_UART_STATE_READY;                 /* At end of Tx process, restore huart->gState to Ready */
            }
            HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 47);
        }
        else if(Labview_uart_Flag % Labview_uart_Flag_CNT == 0 && Labview_uart_Flag< 41 && Labview_uart_Flag> 4) //50ms
        {

            UsartTxBuffer[0] = 0xAA;
            UsartTxBuffer[1] = 0x80;
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[TempID1].lpParam>>(i*8))&0xFF;/*电机1实际速度*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[TempID2].lpParam>>(i*8))&0xFF;/*电机1实际电流*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+10] = (*PARAMETER[TempID3].lpParam>>(i*8))&0xFF;/*电机1实际位置!*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+14] = (*PARAMETER[TempID4].lpParam>>(i*8))&0xFF;/*电机2实际速度*/
            }
            UsartTxBuffer[18] = 0x2F;
            HAL_UART_Transmit(&husart_debug, &UsartTxBuffer[0],19,0xffff);
            memset(UsartTxBuffer,0,sizeof(UsartTxBuffer));                      /*清除缓存*/

        }
    }
}


/*------------------------------------------------
* @function :数据读取
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
long Usart_ReadRegValue(u8 RxBuffer[],int i)
{
    long Date = 0,date1 = 0,date2 = 0,date3 = 0,date4 = 0;
    date1 = RxBuffer[i];
    date2 = RxBuffer[i+1]<<8;
    date3 = RxBuffer[i+2]<<16;
    date4 = RxBuffer[i+3]<<24;
    Date = date1+date2+date3+date4;
    return Date;
}

/*------------------------------------------------
* @function :数据发送
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
long Usart_Labview_SendRegValue(long TxDate)
{
    int i;
    long Date = 0;
    long TxBuffer[4] = {0};
    for(i=0; i<4; i++)
    {
        TxBuffer[3-i] = (TxDate>>(i*8))&0xff;
    }
    Date = (TxBuffer[3]<<24)+(TxBuffer[2]<<16)+(TxBuffer[1]<<8)+TxBuffer[0];
    return Date;
}
/*------------------------------------------------
* @function :上传参数
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void DualDrv_Parameter_Upload(void)
{
    static u16 i = 0;
    printf("*/");
    for(i=0; i<ParaNum; i++)
    {
        if(PARAMETER[i].stAttributes.bSaveToFlash == 1)
        {
            printf("ID=%d/IX=%ld/",PARAMETER[i].sParID,*PARAMETER[i].lpParam );
        }
    }
    printf("#");                                       /*发送结束符*/
}

/*------------------------------------------------
* @function :下发参数表变量
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void DualDrv_Parameter_Download(void)
{
    static u16 i = 0,q = 0,para_id = 0,lenth = 0,j=0;

    for(i=0; i<datalens-1; i++)
    {
        if((UsartRxBuffer[i] == 'I')&&(UsartRxBuffer[i+1] == 'D')&&(UsartRxBuffer[i+2] == '='))
        {
            j = i;
            while((UsartRxBuffer[j+3] != '/')&&(j<i+Rec_Len)) j++;
            for(lenth = i; lenth < j; lenth++)
            {
                str_table[0][lenth-i] = UsartRxBuffer[lenth+3];
            }
            para_id = atoi(*(str_table+0));                     /*atoi (表示 ascii to integer)是把字符串转换成整型数的一个函数*/
            memset(*(str_table+0),'\0',sizeof(*(str_table+0)));
            j = j+4;
            q = j;
            while((UsartRxBuffer[q+3] != '/')&&(q<j+Rec_Len)) q++;
            for(lenth = j; lenth < q; lenth++)
            {
                str_table[0][lenth-j] = UsartRxBuffer[lenth+3];
            }
            *(PARAMETER[para_id].lpParam) = atoi(*(str_table+0)); /*atoi (表示 ascii to integer)是把字符串转换成整型数的一个函数*/
            memset(*(str_table+0),'\0',sizeof(*(str_table+0)));
            j = q+4;
        }
    }
}
/*------------------------------------------------
* @function :电机失能使能函数
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
uint8_t Motor_EN_OFF(_Bool on ,uint8_t motornum)
{
	  static uint32_t tick[2]= {0},lasttick[2]= {0};                     /*Motor_EN_OFF函数的计时*/
		static uint8_t motor_en[2] = {0};
    if(on == 1)
    {
        tick[motornum] = HAL_GetTick();
        if(tick[motornum] - lasttick[motornum] > 40 )                  /*40ms间隔*/
        {
            lasttick[motornum]  = tick[motornum];
            if(STM[motornum].bState != 6)                              /*如果电机没有使能*/
            {
                switch(motor_en[motornum])
                {
                case 0x00:     
                    Driver_Data[motornum].device_control_data.Conrtolword = 0x06;
                    motor_en[motornum] = 1;
                    break;
                case 0x01:
                    Driver_Data[motornum].device_control_data.Conrtolword = 0x07;
                    motor_en[motornum] = 2;
                    break;
                case 0x02:
                    Driver_Data[motornum].device_control_data.Conrtolword = 0x0F;
                    motor_en[motornum] = 0;
                    break;
                default:
                    break;
                }
                return  0 ;
            }
            else                                                      /*使能成功*/
            {
                return 1;
            }
        }
        else
        {
            return  0 ;
        }
    }
    else
    {
        Driver_Data[motornum].device_control_data.Conrtolword = 0x02;
        if(STM[motornum].bState != 6)
        {
            return 1;                                                /*失能成功*/
        }
        else
        {
            return 0;
        }
    }
}
/*------------------------------------------------
* @function :用于上位机切换操作模式
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void Switch_Motor_Operation_Mode(uint8_t mode , uint8_t motornum)   /*3 速度模式  1 位置模式  4 力矩模式*/
{
    Driver_Data[motornum].device_control_data.Modes_of_operation = mode;
}
/*------------------------------------------------
* @function :串口打印功能
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void Labview_uart(void)
{
    u16 temp_tab[5];
        switch(uart_counter)
        {
        case 0:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;
            temp_tab[1] = 23055;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;
            temp_tab[2] = 24555;//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = 30000;//speed_now;//ac_cur_aver_adj;
            temp_tab[4] = 20000 ;//Emf_AD[1];

            break;
        case 1:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;
            temp_tab[1] = (s16)pTrapezoidalM1.Sn*100;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;
            temp_tab[2] = (s16)pTrapezoidalM1.Vm*1000;//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = (s16)100;//speed_now;//ac_cur_aver_adj;
            temp_tab[4] = pCtrlPar[M1].Vel_PLL_Motor/15.9155*1000 ;//Emf_AD[1];

            break;
        case 2:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;
            temp_tab[1] = (s16)pMCI[M1]->pPosCtrl->Omega*1000;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;
            temp_tab[2] = (s16)pMCI[M1]->pPosCtrl->Theta*100;;//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = (s16)pMCI[M1]->pPosCtrl->Acceleration*1000;;//speed_now;//ac_cur_aver_adj;pAxisPar.velErr[A_AXIS]
            temp_tab[4] = (s16)pMCI[M1]->pPosCtrl->Jerk*1000; ;//Emf_AD[1];

            break;
        case 3:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;SMC_DError
            temp_tab[1] = (s16)ENCODER_M1._Super.wPulseNumber ;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;Position_Reference,Position_Encoder
            temp_tab[2] = (s16)pTrapezoidalM1.Sn;//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = (s16)ENCODER_M1._Super.hAvrMecSpeedUnit;//speed_now;//ac_cur_aver_adj;
            temp_tab[4] = (s16)pCtrlPar[M1].Vel_PLL_Motor ;//Emf_AD[1];IMU_PulseNumber

            break;
        case 4:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;
            temp_tab[1] = (s16)ENCODER_M1._Super.wPulseNumber;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;
            temp_tab[2] = (s16)(ENCODER_M1._Super.IMU_PulseNumber);//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = (s16)(pTrapezoidalM1.Sn);//speed_now;//ac_cur_aver_adj;
            temp_tab[4] = (s16) (pCtrlPar[M1].Vel_PLL_Motor) ;//Emf_AD[1];

            break;
        case 5:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;
            temp_tab[1] = (u16)FOCVars[1].Iqdref.q;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;
            temp_tab[2] = (u16)FOCVars[1].Iqd.q;//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = (u16)(SpeednTorqCtrlM2.SpeedRefUnitExt/65536);//speed_now;//ac_cur_aver_adj;pAxisPar.velErr[A_AXIS]
            temp_tab[4] = (u16)pCtrlPar[M2].Vel_PLL_Motor ;//Emf_AD[1];
            break;

        case 6:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;
            temp_tab[1] = (s16)hSpeedRef_Pos;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;
            temp_tab[2] = (s16)(SpeednTorqCtrlM1.SpeedRefUnitExt/65536);//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = (s16)(s16)ENCODER_M1._Super.hAvrMecSpeedUnit;//speed_now;//ac_cur_aver_adj;pAxisPar.velErr[A_AXIS]
            temp_tab[4] = (s16)pCtrlPar[M1].Vel_PLL_Motor;//Emf_AD[1];
            break;

        case 7:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;SMC_DError
            temp_tab[1] = (s16)pCtrlPar[M1].Vel_PLL_Motor;//hSpeedRef_Pos speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;Position_Reference,Position_Encoder
            temp_tab[2] = (s16)pCtrlPar[M1].Vel_PLL_Motor;//wError(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = (s16)(SpeednTorqCtrlM1.SpeedRefUnitExt/65536);//speed_now;//ac_cur_aver_adj;
            temp_tab[4] = (u16)pCtrlPar[M1].Vel_PLL_Motor;//(s16)(ENCODER_M1._Super.IMU_PulseNumber) ;//Emf_AD[1];

            break;

        case 8:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;
            temp_tab[1] = pwmcHandle[M1]->CntPhA;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average,Speed_now
            temp_tab[2] = pwmcHandle[M1]->CntPhB;//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = pwmcHandle[M1]->CntPhC;//speed_now;//ac_cur_aver_adj;
            temp_tab[4] = FOCVars[M1].hElAngle ;//Emf_AD[1];
            break;
        case 9:
            temp_tab[0] = 0x5A0A;;//TIM3->CCR1 >> 1;
            temp_tab[1] = FOCVars[M1].Valphabeta.alpha;//speed_now;//(u16)Emf_Alpha;//Emf_AD[2];//h_BusV_Average;Position_Reference,Position_Encoder
            temp_tab[2] = FOCVars[M1].Valphabeta.beta;//(u16)Emf_Beta;//Emf_AD[0];//module_2;//module;//Stat_Curr_q_d.qI_Component1;
            temp_tab[3] = FOCVars[M2].Iqd.q ;//speed_now;//ac_cur_aver_adj;
            temp_tab[4] = FOCVars[M2].Iqd.d ;//Emf_AD[1];
            break;
#ifdef UART_DEBUG
        case 10:
            temp_tab[0] = 0x5A0A;;//
            temp_tab[1] = (u16)LV_Uart_TxBuffer[0][Uart_TxBuffer_CNT];//
            temp_tab[2] = (u16)LV_Uart_TxBuffer[1][Uart_TxBuffer_CNT];//
            temp_tab[3] = (u16)LV_Uart_TxBuffer[2][Uart_TxBuffer_CNT];//
            temp_tab[4] = (u16)LV_Uart_TxBuffer[3][Uart_TxBuffer_CNT] ;//
//            Uart_TxBuffer_CNT++;//如果按照100us打印数据就要解开屏蔽
            if(Uart_TxBuffer_CNT>LV_TxBufnum-1)
            {
                Uart_TxBuffer_CNT = 0 ;
                Uart_TxBuffer_CNT1 =0 ;
            }
            break;
#endif

        default:
            break;
        }

        LV_TxBuffer[0] = temp_tab[0]/256;
        LV_TxBuffer[1] = temp_tab[0]%256;

        LV_TxBuffer[2] = temp_tab[1]/256;
        LV_TxBuffer[3] = temp_tab[1]%256;

        LV_TxBuffer[4] = temp_tab[2]/256;
        LV_TxBuffer[5] = temp_tab[2]%256;

        LV_TxBuffer[6] = temp_tab[3]/256;
        LV_TxBuffer[7] = temp_tab[3]%256;

        LV_TxBuffer[8] = temp_tab[4]/256;
        LV_TxBuffer[9] = temp_tab[4]%256;

        printf("%c%c%c%c%c%c%c%c%c%c",LV_TxBuffer[0],LV_TxBuffer[1],LV_TxBuffer[2],LV_TxBuffer[3],LV_TxBuffer[4],LV_TxBuffer[5],LV_TxBuffer[6],LV_TxBuffer[7],LV_TxBuffer[8],LV_TxBuffer[9]);

}
