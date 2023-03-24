/*
*******************************************************************************
 *��    Ȩ�� 2021-xxxx,  GaussianRobot
 *�� �� ���� usart_labview.c������
 *��    �飺 ���ڳ�ʼ��MCU����
 *��    �ߣ� LMmotor\����
 *��    �ڣ� 2023.1.3
 *���������� 
*******************************************************************************
 *��ע��         С�����������ܣ�LABVIEW���ڴ�ӡ����
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
long TempSpeedMotor1=0,TempSpeedMotor2=0;                        /*�м����*/
float SetPositionMotor1=0,SetPositionMotor2=0;

u16 Uart_TxBuffer_CNT = 0;
u16 Uart_TxBuffer_CNT1 =0;
u8 uart_counter=10;                                              /*�����л�ʾ������ӡ����*/
uint8_t UsartRxBuffer[LABVIEWRXBUFFERSIZE] = {0};                /*���ڽ�������*/
uint8_t UsartTxBuffer[LABVIEWRXBUFFERSIZE] = {0};                /*���ڷ�������*/
uint8_t Labview_uart_Flag_CNT = 5;                               /*ʾ����ˢ��ʱ��*/
uint8_t TempID1 = 27,TempID2 = 28,TempID3 = 30,TempID4 = 31;     /*����ʾ������ӡ�����ݺ�*/
uint8_t RefreshCmd_Type;                                         /*����ˢ�½���*/
uint8_t g_uart_recv_ok;                                          /*����һ���������*/
uint8_t Labview_uart_100us_Flag = 1 ;                            /*100us��ӡ���ݱ�־*/
uint8_t Labview_uart_Flag=0;                                     /*1ms����һ��*/
int16_t LV_TxBuffer[10];                                         /*print������ӡ����ʱ����*/

extern HALL_Handle_t HALL_M1;
extern LOAD_CURR	loadCurr[NUMBER_OF_AXES];
extern MotorParameters_t MotorParameters[NBR_OF_MOTORS];
extern void MotorParametersM1_Init(void);                        /*������ʼ������*/
extern void MotorParametersM2_Init(void);
/*�ṩ�ⲿ��������labview��ӡ����*/
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
	  static uint8_t entemp1 = 0,entemp2 = 0;                                    /*ʹ��ʧ�ܺ����ж��Ƿ�ʹ�ܳɹ�*/
	  static uint8_t UsartState = 0;                                             /*״̬���л�*/
    int i;
    switch (UsartState)
    {
    case 0x00:                                                                 /*�ȴ�������ɱ�־*/
        if(g_uart_recv_ok)
        {
            g_uart_recv_ok = 0;
            UsartState = UsartRxBuffer[1];                                     /*���������е�һ�����ݣ�Ϊ������*/
        }
        break;
    case 0xFE:  
        UsartState = 0x00;
        break;
    case 0x01:                                                                 /*���1������ȡָ����1�������������1�������������1PID������*/
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x81;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+2] = (*PARAMETER[43].lpParam>>(i*8))&0xFF;         /*������*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+6] = (*PARAMETER[96].lpParam>>(i*8))&0xFF;         /*���س���*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+10] = (*PARAMETER[97].lpParam>>(i*8))&0xFF;        /*�������*/
        }

        TempSpeedMotor1 = *PARAMETER[98].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+14] = (TempSpeedMotor1>>(i*8))&0xFF;               /*�����*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+18] = (*PARAMETER[99].lpParam>>(i*8))&0xFF;        /*�ת��*/
        }
        TempSpeedMotor1 = *PARAMETER[100].lpParam*0.1;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+22] = (TempSpeedMotor1>>(i*8))&0xFF;               /*�ת��*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+26] = (*PARAMETER[101].lpParam>>(i*8))&0xFF;       /*�������ֱ���*/
        }
        TempSpeedMotor1 = *PARAMETER[102].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+30] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1��ֵ����*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+34] = (*PARAMETER[103].lpParam>>(i*8))&0xFF;        /*M1��ֵ��������ʱ��*/
        }

        TempSpeedMotor1 = *PARAMETER[104].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+38] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1��������*/
        }
        TempSpeedMotor1 = *PARAMETER[105].lpParam*0.1;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+42] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1����ٶ�*/
        }
        TempSpeedMotor1 = *PARAMETER[106].lpParam*0.1;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+46] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1��ת�ٶ�*/
        }

        TempSpeedMotor1 = *PARAMETER[107].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+50] = (TempSpeedMotor1>>(i*8))&0xFF;                /*M1��ת����*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+54] = (*PARAMETER[108].lpParam>>(i*8))&0xFF;        /*M1��תʱ��*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+58] = (*PARAMETER[9].lpParam>>(i*8))&0xFF;          /*M1������P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+62] = (*PARAMETER[10].lpParam>>(i*8))&0xFF;         /*M1������I*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+66] = (*PARAMETER[7].lpParam>>(i*8))&0xFF;          /*M1�ٶȻ�P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+70] = (*PARAMETER[8].lpParam>>(i*8))&0xFF;          /*M1�ٶȻ�P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+74] = (*PARAMETER[1].lpParam>>(i*8))&0xFF;          /*M1λ�û�P*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+78] = (*PARAMETER[2].lpParam>>(i*8))&0xFF;          /*M1λ�û�I*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+82] = (*PARAMETER[3].lpParam>>(i*8))&0xFF;          /*M1λ�û�D*/
        }
        UsartTxBuffer[86] = (int8_t)(*PARAMETER[49].lpParam);                   /*pll�ٶȻ�����*/
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
            UsartTxBuffer[i+95] = (*PARAMETER[91].lpParam>>(i*8))&0xFF;         /*����*/
        }
        UsartTxBuffer[99] = 0x2F;
        HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 100);
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x02:                                                                  /*���1������ȡָ����1�������������1�������������1PID������*/
      
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x84;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+2] = (*PARAMETER[44].lpParam>>(i*8))&0xFF;          /*M2������*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+6] = (*PARAMETER[110].lpParam>>(i*8))&0xFF;         /*M2���س���*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+10] = (*PARAMETER[111].lpParam>>(i*8))&0xFF;        /*M2�������*/
        }


        TempSpeedMotor2 = *PARAMETER[112].lpParam/ADC_AMPER_Ratio;     
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+14] = (TempSpeedMotor2>>(i*8))&0xFF;                /*�����*/
        }


        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+18] = (*PARAMETER[113].lpParam>>(i*8))&0xFF;        /*�ת��*/
        }
        TempSpeedMotor2 = *PARAMETER[114].lpParam*0.1;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+22] = (TempSpeedMotor2>>(i*8))&0xFF;                /*��ٶ�*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+26] = (*PARAMETER[115].lpParam>>(i*8))&0xFF;        /*�������ֱ���*/
        }
        TempSpeedMotor2 = *PARAMETER[116].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+30] = (TempSpeedMotor2>>(i*8))&0xFF;                /*��ֵ����*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+34] = (*PARAMETER[117].lpParam>>(i*8))&0xFF;        /*��ֵ��������ʱ��*/
        }

        TempSpeedMotor2 = *PARAMETER[118].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+38] = (TempSpeedMotor2>>(i*8))&0xFF;                /*��������*/
        }

        TempSpeedMotor2 = *PARAMETER[38].lpParam*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+42] = (TempSpeedMotor2>>(i*8))&0xFF;                /*����ٶ�*/
        }

        TempSpeedMotor2 = *PARAMETER[120].lpParam*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+46] = (TempSpeedMotor2>>(i*8))&0xFF;                /*��ת�ٶ�*/
        }

        TempSpeedMotor2 = *PARAMETER[121].lpParam/ADC_AMPER_Ratio;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+50] = (TempSpeedMotor2>>(i*8))&0xFF;                /*��ת����*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+54] = (*PARAMETER[122].lpParam>>(i*8))&0xFF;        /*��תʱ��*/
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
        UsartTxBuffer[86] = (int8_t)(*PARAMETER[49].lpParam);                   /*pll�ٶȻ�����*/
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
            UsartTxBuffer[i+95] = (*PARAMETER[90].lpParam>>(i*8))&0xFF;        /*����*/
        }
        UsartTxBuffer[99] = 0x2F;
        HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 100);
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x03:                                                                /*������ؽ���*/
        RefreshCmd_Type = 1;                                                  /*����ˢ��*/
        UsartState = 0xFE;
        break;
    case 0x04:                                                                /*���ʹ��*/
        if(UsartRxBuffer[2] == 3)                                             /*ʹ���������*/
        {
            entemp1 = Motor_EN_OFF(1,M1);                                     /*ʹ�ܳɹ�����1*/
            entemp2 = Motor_EN_OFF(1,M2);                                     /*ʹ�ܳɹ�����1*/
            if(entemp1+entemp2 == 2)                                          /*������������ʹ�ܳɹ�*/
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
        else if(UsartRxBuffer[2] == 0x01||UsartRxBuffer[2] == 0x02)          /*ʹ�ܵ������*/
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
    case 0x05:                                                             /*ʹ��ָ��*/
        if(UsartRxBuffer[2] == 3)                                          /*ʧ���������*/
        {
            entemp1 = Motor_EN_OFF(0,M1);                                  /*ʧ��M1*/
            entemp2 =	Motor_EN_OFF(0,M2);                                  /*ʧ��M2*/
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
        else if(UsartRxBuffer[2] == 0x01||UsartRxBuffer[2] == 0x02)        /*ʧ�ܵ������*/
        {
            entemp1 = Motor_EN_OFF(0,UsartRxBuffer[2]-1);
            if(entemp1)                                                    /*ʧ�ܳɹ�*/
            {
                entemp1 = 0;
                RefreshCmd_Type = 0;
                UsartState = 0xFE;
            }
            else
            {
                /*������*/
            }
        }
        else
        {
            RefreshCmd_Type = 0;
            UsartState = 0xFE;
        }
        break;
    case 0x06:                                                             /*�����ָͣ�ֻ�����ٶ�ģʽ��ʹ�õļ�ͣ*/
        pCtrlPar[M1].SetVelMotor = 0;
        pCtrlPar[M2].SetVelMotor = 0;
        pCtrlPar[M1].M1M2_VelSet = 0;
        MC_ProgramSpeedRampMotor1( pCtrlPar[M1].SetVelMotor, 50 );
        MC_ProgramSpeedRampMotor2( pCtrlPar[M2].SetVelMotor, 50 );
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x07:                                                             /*�������ָ��*/
        if(UsartRxBuffer[2] == 0x01 )                                      /*���1����ģʽѡ��*/
        {
            if(UsartRxBuffer[3] == 0x01)
            {
                HallStudyFlag1 = 2;	                                       /*������ģʽ*/
            }
            else if(UsartRxBuffer[3] == 0x02)
            {
                HallStudyFlag1 = 1;                                        /*����ѧϰģʽ*/
            }
        }
        else if(UsartRxBuffer[2] == 0x02)                                  /*���2����ģʽѡ��*/
        {
            if(UsartRxBuffer[3] == 0x01)
            {
                HallStudyFlag2 = 2;                                        /*������ģʽ*/
            }
            else if(UsartRxBuffer[3] == 0x02)
            {
                HallStudyFlag2 = 1;                                        /*����ѧϰģʽ*/
            }
        }
        else if(UsartRxBuffer[2] == 0x03)                                  /*���1��2����ģʽѡ��*/
        {
            if(UsartRxBuffer[3] == 0x01)
            {
                HallStudyFlag1 = 2;                                        /*������ģʽ*/
                HallStudyFlag2 = 2;
            }
            else if(UsartRxBuffer[3] == 0x02)
            {
                HallStudyFlag1 = 1;                                        /*����ѧϰģʽ*/
                HallStudyFlag2 = 1;
            }
        }
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x08:                                                             /*�������ģʽ*/
        if(UsartRxBuffer[2] == 0x03 )                                      /*�������һ��*/
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
    case 0x09:                                                            /*�������*/
        Driver_Data[M1].device_control_data.Conrtolword = 0x80;
        RefreshCmd_Type = 1;
        UsartState = 0xFE;
        break;
    case 0x0A:                                                            /*����������ȡ*/
        if(UsartRxBuffer[2] == 0)                                         /*��ȡID*/
        {
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[UsartRxBuffer[3]].lpParam>>(i*8))&0xFF;   /*ID���ݷ���*/
            }
            UsartTxBuffer[0] = 0xAA;
            UsartTxBuffer[1] = 0x8A;
            UsartTxBuffer[6] = 0x2F;
            HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 7);
        }
        else if(UsartRxBuffer[2] == 1)                                    /*д��ID*/
        {
            *PARAMETER[UsartRxBuffer[3]].lpParam = Usart_ReadRegValue(UsartRxBuffer,4);
        }
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x0B:                                                           /*��ȡ���ز���*/
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x87;
        switch (UsartRxBuffer[2])
        {
        case 0x01 :
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[50].lpParam>>(i*8))&0xFF;/*���1Ŀ������*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[53].lpParam>>(i*8))&0xFF;/*���2Ŀ������*/
            }
            UsartTxBuffer[10] = 0x2F;
            HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 11);
            break;
        case 0x02:
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[50].lpParam>>(i*8))&0xFF;/*���2Ŀ������*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[53].lpParam>>(i*8))&0xFF;/*���2Ŀ������*/
            }
            UsartTxBuffer[10] = 0x2F;
            HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 11);
            break;
        case 0x03:  //!!!����
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (*PARAMETER[50].lpParam>>(i*8))&0xFF;/*���1Ŀ������*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[53].lpParam>>(i*8))&0xFF;/*���2Ŀ������*/
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
    case 0x0C: /*��ȡ�ٶȲ���*/
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x88;
        TempSpeedMotor1 = (long)((float)(*PARAMETER[27].lpParam/10));
        TempSpeedMotor2 = (long)((float)(*PARAMETER[30].lpParam/10));
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+2] = (TempSpeedMotor1>>(i*8))&0xFF;             /*���1Ŀ���ٶ�*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+6] = (*PARAMETER[29].lpParam>>(i*8))&0xFF;      /*���1Ŀ��ʱ��*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+10] = (TempSpeedMotor2>>(i*8))&0xFF;            /*���2Ŀ���ٶ�*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+14] = (*PARAMETER[32].lpParam>>(i*8))&0xFF;     /*���2Ŀ��ʱ�� ��20201207*/
        }
        UsartTxBuffer[18] = 0x2F;
        HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 19);
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;

    case 0x0D:                                                             /*��ȡλ�ò���*/
        UsartTxBuffer[0] = 0xAA;
        UsartTxBuffer[1] = 0x89;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+2] = (*PARAMETER[21].lpParam>>(i*8))&0xFF;     /*���1Ŀ��λ��*/
        }
        TempSpeedMotor1 = (*PARAMETER[6].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+6] = (TempSpeedMotor1>>(i*8))&0xFF;            /*���1λ�û�Ŀ���ٶ�*/
        }
        TempSpeedMotor1 = (*PARAMETER[4].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+10] = (TempSpeedMotor1>>(i*8))&0xFF;           /*���1Ŀ����ٶ�*/
        }
        TempSpeedMotor1 = (*PARAMETER[5].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+14] = (TempSpeedMotor1>>(i*8))&0xFF;           /*���1Ŀ����ٶ�*/
        }
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+18] = (*PARAMETER[24].lpParam>>(i*8))&0xFF;    /*���2Ŀ��λ��*/
        }
        TempSpeedMotor2 = (*PARAMETER[16].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+22] = (TempSpeedMotor2>>(i*8))&0xFF;           /*���2λ�û�Ŀ���ٶ�*/
        }
        TempSpeedMotor2 = (*PARAMETER[14].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+26] = (TempSpeedMotor2>>(i*8))&0xFF;           /*���2Ŀ����ٶ�*/
        }
        TempSpeedMotor2 = (*PARAMETER[15].lpParam)*0.6;
        for(i=0; i<4; i++)
        {
            UsartTxBuffer[i+30] = (TempSpeedMotor2>>(i*8))&0xFF;           /*���2Ŀ����ٶ�*/
        }

        UsartTxBuffer[34] = 0x2F;
        HAL_UART_Transmit_DMA(&husart_debug,UsartTxBuffer, 35);
        RefreshCmd_Type = 0;
        UsartState = 0xFE;



        break;
    case 0x0E:                                                           /*δ�����ã�����*/
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x0F:                                                          /*ͬ��ģʽ����*/
        Sync_Async_Control = (uint16_t)UsartRxBuffer[2];
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x11:                                                          /*���õ��1�������ò���*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);    /*�������ݴ���*/
        }

        *PARAMETER[95].lpParam = Lab_RxDate[0];                          /*������*/
        *PARAMETER[96].lpParam = Lab_RxDate[1];                          /*���س���*/
        *PARAMETER[97].lpParam = Lab_RxDate[2];                          /*�������*/
        *PARAMETER[98].lpParam = (long)(Lab_RxDate[3]);                  /*�����*/
        *PARAMETER[99].lpParam = Lab_RxDate[4];                          /*�ת��*/
        *PARAMETER[100].lpParam = Lab_RxDate[5];                         /*�ת��*/
        *PARAMETER[101].lpParam = Lab_RxDate[6];                         /*�������ֱ���*/
        MotorParametersM1_Init();
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x12:                                                           /*���õ��1��������*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }

        *PARAMETER[102].lpParam = (long)(Lab_RxDate[0]);                 /*��ֵ����*/
        *PARAMETER[103].lpParam = Lab_RxDate[1];                         /*��ֵ��������ʱ��*/
        *PARAMETER[104].lpParam = Lab_RxDate[2];                         /*��������*/
        *PARAMETER[105].lpParam = Lab_RxDate[3];	                       /*����ٶ�*/
        *PARAMETER[106].lpParam = Lab_RxDate[4];                         /*��ת�ٶ�*/
        *PARAMETER[107].lpParam = (long)(Lab_RxDate[5]);                 /*��ת����*/
        *PARAMETER[108].lpParam = Lab_RxDate[6];                         /*��תʱ��*/

        MotorParametersM1_Init();
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x13:                                                           /*���õ��1PID����*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }
        Lab_RxDate[7] = UsartRxBuffer[30];                               /*PLL����*/
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
        *PARAMETER[49].lpParam = Lab_RxDate[7];                          /*PLL���ش���*/
        *PARAMETER[60].lpParam = Lab_RxDate[8];                          /*Kp_PLL*/
        *PARAMETER[61].lpParam = Lab_RxDate[9];                          /*Kd_PLL*/
        *PARAMETER[91].lpParam = Lab_RxDate[10];                         /*DIV_PLL*/
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x14:                                                           /*���õ��2�������ò���*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }
        *PARAMETER[109].lpParam = Lab_RxDate[0];                          /*������*/
        *PARAMETER[110].lpParam = Lab_RxDate[1];                          /*���س���*/
        *PARAMETER[111].lpParam = Lab_RxDate[2];                          /*�������*/
        *PARAMETER[112].lpParam = (long)(Lab_RxDate[3]*ADC_AMPER_Ratio);	/*�����*/
        *PARAMETER[113].lpParam = Lab_RxDate[4];  	                      /*�ת��*/
        *PARAMETER[114].lpParam = Lab_RxDate[5];                          /*�ת��*/
        *PARAMETER[115].lpParam = Lab_RxDate[6]; 	                        /*�������ֱ���*/

        MotorParametersM2_Init();
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x15:     	/**/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }
        *PARAMETER[116].lpParam = (long)(Lab_RxDate[0]);                  /*��ֵ����*/
        *PARAMETER[117].lpParam = Lab_RxDate[1];                          /*��ֵ��������ʱ��*/
        *PARAMETER[118].lpParam = Lab_RxDate[2]; 	                        /*��������*/
        *PARAMETER[38].lpParam = Lab_RxDate[3];		                        /*����ٶ�*/
        *PARAMETER[120].lpParam = Lab_RxDate[4]; 	                        /*��ת�ٶ�*/
        *PARAMETER[121].lpParam = (long)(Lab_RxDate[5]);                  /*��ת����*/
        *PARAMETER[122].lpParam = Lab_RxDate[6];                          /*��תʱ��*/

        MotorParametersM2_Init();
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x16:                                                           	/*���õ��2PID����*/
        for(i=0; i<7; i++)
        {
            Lab_RxDate[i] = Usart_ReadRegValue(UsartRxBuffer,2+4*i);
        }
        Lab_RxDate[7] = UsartRxBuffer[30];                                /*PLL����*/
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
        *PARAMETER[49].lpParam = Lab_RxDate[7];                           /*PLL���ش���*/
        *PARAMETER[58].lpParam = Lab_RxDate[8];                           /*Kp_PLL*/
        *PARAMETER[59].lpParam = Lab_RxDate[9];                           /*Kd_PLL*/
        *PARAMETER[90].lpParam = Lab_RxDate[10];                          /*DIV_PLL*/
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x17:                                                            /*���õ�����ز���*/
        if(UsartRxBuffer[2] == 0x01)                                      /*���1*/
        {
            *PARAMETER[50].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
        }
        else if(UsartRxBuffer[2] == 0x02)                                 /*���2*/
        {
            *PARAMETER[53].lpParam = Usart_ReadRegValue(UsartRxBuffer,7); /*�޸�motor2����λ202021205*/
        }
        else if(UsartRxBuffer[2] == 0x03)                                 /*���1-2*/
        {
            *PARAMETER[50].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
            *PARAMETER[53].lpParam = Usart_ReadRegValue(UsartRxBuffer,7); /*����motor2����λ202021205*/
        }
        RefreshCmd_Type = 0;
        UsartState = 0xFE;
        break;
    case 0x18:			                                                      /*���õ���ٶȲ���*/
        if(UsartRxBuffer[2] == 0x01)                                      /*���1*/
        {
            *PARAMETER[27].lpParam = Usart_ReadRegValue(UsartRxBuffer,3)*10;
            *PARAMETER[29].lpParam = Usart_ReadRegValue(UsartRxBuffer,7);
        }
        else if(UsartRxBuffer[2] == 0x02)                                 /*���2*/
        {
            *PARAMETER[30].lpParam = Usart_ReadRegValue(UsartRxBuffer,11)*10;
            *PARAMETER[32].lpParam = Usart_ReadRegValue(UsartRxBuffer,15);
        }
        else if(UsartRxBuffer[2] == 0x03)                                 /*���1-2*/
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
    case 0x19:                                                                           /*���õ��λ�ò���*/
        if(UsartRxBuffer[2] == 0x01)                                                     /*���1*/
        {
            *PARAMETER[21].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);                /*Ŀ��λ��*/
            *PARAMETER[6].lpParam = Usart_ReadRegValue(UsartRxBuffer,7)/0.6;             /*Ŀ���ٶ�*/
            *PARAMETER[4].lpParam = Usart_ReadRegValue(UsartRxBuffer,11)/0.6;            /*���ٶ�*/
            *PARAMETER[5].lpParam = Usart_ReadRegValue(UsartRxBuffer,15)/0.6;            /*���ٶ�*/
            SetPositionMotor1 = pCtrlPar[M1].SetPulseMotor*RAD_PULSE/M1_ENCODER_PPR/16;  /*���� 1000*pi/M1_ENCODER_PPR/2*/
        }
        else if(UsartRxBuffer[2] == 0x02)                                                /*���2*/
        {
            *PARAMETER[24].lpParam = Usart_ReadRegValue(UsartRxBuffer,19);               /*Ŀ��λ��*/
            *PARAMETER[16].lpParam = Usart_ReadRegValue(UsartRxBuffer,23)/0.6;           /*Ŀ���ٶ�*/
            *PARAMETER[14].lpParam = Usart_ReadRegValue(UsartRxBuffer,27)/0.6;           /*���ٶ�*/
            *PARAMETER[15].lpParam = Usart_ReadRegValue(UsartRxBuffer,31)/0.6;           /*���ٶ�*/
            SetPositionMotor2 = pCtrlPar[M2].SetPulseMotor*RAD_PULSE/M2_ENCODER_PPR/16;  /*���� 1000*pi/M1_ENCODER_PPR/2*/
        }
        else if(UsartRxBuffer[2] == 0x03)                                                /*���1-2*/
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
    case 0x1A:                                                                           /*���õ�������������������������û�������������*/
        if(UsartRxBuffer[2] == 0x01)                                                     /*���1*/
        {
            *PARAMETER[56].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
        }
        else if(UsartRxBuffer[2] == 0x02)                                                /*���2*/
        {
            *PARAMETER[57].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
        }
        else if(UsartRxBuffer[2] == 0x03)                                                /*���1-2*/
        {
            *PARAMETER[56].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
            *PARAMETER[57].lpParam = Usart_ReadRegValue(UsartRxBuffer,3);
        }
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x1B:                                                                           /*�·�ʾ��������*/
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
    case 0x1C:                                               /*�·�ʾ����ID����*/
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
    case 0x1D:                                               /*flash��������*/
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x1E:                                               /*�ϴ����������*/
        DualDrv_Parameter_Upload();
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    case 0x1F:                                               /*�·����������*/
        DualDrv_Parameter_Download();
        UsartState = 0xFE;
        RefreshCmd_Type = 0;
        break;
    }


    if(RefreshCmd_Type == 2)	                               /*ʾ��������*/
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
                    UsartTxBuffer[i+2] = (LabView_Uart_TxBuffer[0][Uart_TxBuffer_CNT]>>(i*8))&0xFF;/*���1ʵ���ٶ�*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+6] = (LabView_Uart_TxBuffer[1][Uart_TxBuffer_CNT]>>(i*8))&0xFF;/*���1ʵ�ʵ���*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+10] = (LabView_Uart_TxBuffer[2][Uart_TxBuffer_CNT]>>(i*8))&0xFF;/*���1ʵ��λ��!*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+14] = (LabView_Uart_TxBuffer[3][Uart_TxBuffer_CNT]>>(i*8))&0xFF;/*���2ʵ���ٶ�*/
                }
                UsartTxBuffer[18] = 0x2F;
								
                dmarequest = HAL_IS_BIT_SET(husart_debug.Instance->CR3, USART_CR3_DMAT);            /*�˴�Ϊhal���bug��ʱû�кõİ취����������Ҫ����DMA���;���Ҫ�öδ���*/
                if ((husart_debug.gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
                {
                    CLEAR_BIT(husart_debug.Instance->CR3, USART_CR3_DMAT);                          /* ��ֹDMA������ */
                  
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
                    UsartTxBuffer[i+2] = (*PARAMETER[TempID1].lpParam>>(i*8))&0xFF;                /*���1ʵ���ٶ�*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+6] = (*PARAMETER[TempID2].lpParam>>(i*8))&0xFF;                /*���1ʵ�ʵ���*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+10] = (*PARAMETER[TempID3].lpParam>>(i*8))&0xFF;                /*���1ʵ��λ��!*/
                }
                for(i=0; i<4; i++)
                {
                    UsartTxBuffer[i+14] = (*PARAMETER[TempID4].lpParam>>(i*8))&0xFF;                /*���2ʵ���ٶ�*/
                }
                UsartTxBuffer[18] = 0x2F;
								
                dmarequest = HAL_IS_BIT_SET(husart_debug.Instance->CR3, USART_CR3_DMAT);            /*�˴�Ϊhal���bug��ʱû�кõİ취����������Ҫ����DMA���;���Ҫ�öδ���*/
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
    else if(RefreshCmd_Type == 1)                                         /*��ؽ���*/
    {
        if(Labview_uart_Flag > Labview_uart_Flag_CNT*5)                   /*25ms*/
        {
            Labview_uart_Flag = 0;
            UsartTxBuffer[0] = 0xAA;
            UsartTxBuffer[1] = 0x90;
            TempSpeedMotor1 = *PARAMETER[28].lpParam*0.1;                 /*��С10����λΪrpm*/
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+2] = (TempSpeedMotor1>>(i*8))&0xFF;       /*���1ʵ���ٶ�*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[74].lpParam>>(i*8))&0xFF;/*���1ʵ�ʵ���*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+10] = (*PARAMETER[22].lpParam>>(i*8))&0xFF;/*���1ʵ��λ��!*/
            }
            TempSpeedMotor2 = *PARAMETER[31].lpParam*0.1;
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+14] = (TempSpeedMotor2>>(i*8))&0xFF;       /*���2ʵ���ٶ�*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+18] = (*PARAMETER[75].lpParam>>(i*8))&0xFF;/*���2ʵ�ʵ���*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+22] = (*PARAMETER[25].lpParam>>(i*8))&0xFF;/*���2ʵ��λ��*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+26] = (*PARAMETER[86].lpParam>>(i*8))&0xFF;/*���1״̬*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+30] = (*PARAMETER[87].lpParam>>(i*8))&0xFF;/*���2״̬��������*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+34] = (*PARAMETER[94].lpParam>>(i*8))&0xFF;/*ĸ�ߵ�ѹ��*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+38] = (*PARAMETER[92].lpParam>>(i*8))&0xFF;/*���1�¶ȣ�*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+42] = (*PARAMETER[93].lpParam>>(i*8))&0xFF;/*���2�¶ȣ�*/
            }
            UsartTxBuffer[46] = 0x2F;
						
            dmarequest = HAL_IS_BIT_SET(husart_debug.Instance->CR3, USART_CR3_DMAT); /*�˴�Ϊhal���bug��ʱû�кõİ취���*/
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
                UsartTxBuffer[i+2] = (*PARAMETER[TempID1].lpParam>>(i*8))&0xFF;/*���1ʵ���ٶ�*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+6] = (*PARAMETER[TempID2].lpParam>>(i*8))&0xFF;/*���1ʵ�ʵ���*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+10] = (*PARAMETER[TempID3].lpParam>>(i*8))&0xFF;/*���1ʵ��λ��!*/
            }
            for(i=0; i<4; i++)
            {
                UsartTxBuffer[i+14] = (*PARAMETER[TempID4].lpParam>>(i*8))&0xFF;/*���2ʵ���ٶ�*/
            }
            UsartTxBuffer[18] = 0x2F;
            HAL_UART_Transmit(&husart_debug, &UsartTxBuffer[0],19,0xffff);
            memset(UsartTxBuffer,0,sizeof(UsartTxBuffer));                      /*�������*/

        }
    }
}


/*------------------------------------------------
* @function :���ݶ�ȡ
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
* @function :���ݷ���
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
* @function :�ϴ�����
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
    printf("#");                                       /*���ͽ�����*/
}

/*------------------------------------------------
* @function :�·����������
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
            para_id = atoi(*(str_table+0));                     /*atoi (��ʾ ascii to integer)�ǰ��ַ���ת������������һ������*/
            memset(*(str_table+0),'\0',sizeof(*(str_table+0)));
            j = j+4;
            q = j;
            while((UsartRxBuffer[q+3] != '/')&&(q<j+Rec_Len)) q++;
            for(lenth = j; lenth < q; lenth++)
            {
                str_table[0][lenth-j] = UsartRxBuffer[lenth+3];
            }
            *(PARAMETER[para_id].lpParam) = atoi(*(str_table+0)); /*atoi (��ʾ ascii to integer)�ǰ��ַ���ת������������һ������*/
            memset(*(str_table+0),'\0',sizeof(*(str_table+0)));
            j = q+4;
        }
    }
}
/*------------------------------------------------
* @function :���ʧ��ʹ�ܺ���
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
uint8_t Motor_EN_OFF(_Bool on ,uint8_t motornum)
{
	  static uint32_t tick[2]= {0},lasttick[2]= {0};                     /*Motor_EN_OFF�����ļ�ʱ*/
		static uint8_t motor_en[2] = {0};
    if(on == 1)
    {
        tick[motornum] = HAL_GetTick();
        if(tick[motornum] - lasttick[motornum] > 40 )                  /*40ms���*/
        {
            lasttick[motornum]  = tick[motornum];
            if(STM[motornum].bState != 6)                              /*������û��ʹ��*/
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
            else                                                      /*ʹ�ܳɹ�*/
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
            return 1;                                                /*ʧ�ܳɹ�*/
        }
        else
        {
            return 0;
        }
    }
}
/*------------------------------------------------
* @function :������λ���л�����ģʽ
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void Switch_Motor_Operation_Mode(uint8_t mode , uint8_t motornum)   /*3 �ٶ�ģʽ  1 λ��ģʽ  4 ����ģʽ*/
{
    Driver_Data[motornum].device_control_data.Modes_of_operation = mode;
}
/*------------------------------------------------
* @function :���ڴ�ӡ����
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
//            Uart_TxBuffer_CNT++;//�������100us��ӡ���ݾ�Ҫ�⿪����
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
