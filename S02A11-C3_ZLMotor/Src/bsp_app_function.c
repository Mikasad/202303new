/*
*******************************************************************************
 *��    Ȩ�� 2021-xxxx,  GaussianRobot
 *�� �� ���� bsp_app_function.c ����
 *��    �飺 ����������й����Ժ�����Ӧ��д�ڸ��ļ�����
 *��    �ߣ� LMmotor\����
 *��    �ڣ� 2022.1.4
 *����������
*******************************************************************************
 *��ע��
*******************************************************************************
*/
#include "bsp_app_function.h"
#include "ds402.h"
#include "mc_config.h "
#include "stdlib.h"
#include "bsp_CAN.h"


/*LEDSet DisplayErrorLed_Handle*/
uint32_t DisplayCounter = 0;                                     /*ָʾ����˸Ƶ��*/

/*Independent_Motor_Control*/
uint8_t PositionModeFlag1=0,PositionModeFlag2=0;                 /*0����Ҫ����λ�û����ᣬ����������Ժ��Զ���Ϊ1����λ�û�����*/
uint8_t relative_location_flag[MAX_AXES] = {0};                  /*���λ�ÿ���1��ִ�����λ�� ִ��һ�����Ժ��Զ���0*/
uint8_t absolute_location_flag[MAX_AXES] = {0};                  /*����λ�ÿ��Ʋ���*/
uint8_t P_V_I_Switch_using[MAX_AXES];                            /*����ģʽ�л���־λ*/
uint8_t modetemp[MAX_AXES]= {3,3};                               /*�ϵ紦���ٶ�ģʽ����ֵ��ʼ��Ϊ3*/
uint8_t  LockAxleON_OFF = 0 ;                                    /*���Ὺ��*/
uint8_t initial2_positioning_timing = 0;                         /*��ʼ��λ������*/
uint8_t initial_positioning_timing = 0;                          /*��ʼ��λ������*/
uint16_t Sync_Async_Control = 1;                                 /*ͬ��ģʽ�첽ģʽѡ�����*/
int32_t Countrer_1ms_CNT=200,Countrer2_1ms_CNT=200;              /*���ڵ��12��λ�û������ʱ*/
long RelativeLocation[MAX_AXES] = {0};                           /*���λ�òο�*/
float CurrentPosition1,CurrentPosition2;                         /*���ڵ��12��λ�û����ᣬ�Լ���mc_it.c���л�ȡ�������Ժ����pPosCtrl[M1]->Theta��λ�ù滮�й�*/


/*DefaultPhaseCheck*/
uint32_t  giStuckTimeCnt[NUMBER_OF_AXES] = {0,0};                /*��תʱ�����*/

/*ClearPhaseCheckPar  DefaultPhaseCheck*/
long IabcCurrentTemp[2][3] = {0};                                /*��������Ķ�ά���飬abc�����������*/
float cur_calculated_value[2] = {0};                             /*300ms�����ۼ������Ժ󣬼��㣺(Imax-Imin)/Imax ��Χ[0,1]*/
float iabcurrenttemporary[3] = {0};                              /*���������������������������*/
uint16_t  lostPhaseCheckCnt[2] = {0};                            /*�������ʱȱ�������*/
uint16_t  lostPhaseCheckCntForLaunch[2] = {0};                   /*�������ʱȱ�������*/
uint32_t Missing_phase_error_code[2] = {MC_NO_ERROR};            /*ȱ��������*/
uint32_t MISSING_PAHSE_JUDGE_TIME = 10000;                       /*����2000ms���󱨴���ֵ̫��������������*/

/*void Emrgency_Stop(void)*/
uint16_t EMG_State3,EMG_State4 = 2;                              /*��ͣ��ťSTOPISOFF(1) or STOPISON(0)*/
uint16_t  giStallTimeCnt[NUMBER_OF_AXES] = {0};                  /*ʧ�ټ���ۼ�ʱ�����*/
uint32_t hall_error_cnt[2] = {0,0};                              /*�����������ۼ�ʱ�����*/
uint32_t temperature_cnt[2] = {0};                               /*����¶ȱ���ʱ���ۼƼ���*/

Trig_Components Vector_Components;                               /*��������SIN���ߡ��������ڲ��Ե��������ٶȻ���Ӧ*/
LOOP_TEST_T	 loopTestPar;								                         /*��������SIN���ߡ��������ڲ��Ե��������ٶȻ���Ӧ*/
u8    Damping_Mode_ON_OFF = 0;                                   /*����ģʽ����*/
SEVERO_CONTORL_PAR_T pCtrlPar_M1=                                /*��SEVERO_CONTORL_PAR_T���͵�pCtrlPar_M1����ֵ�������ڽӿڣ����� pCtrlPar[M1]  = *(&pCtrlPar_M1); �õ�ַָ��ָ��*/
{
    .SetVelMotor = 0,
    .Vel_PLL_Motor = 0,
    .SetTorqueMotor = 0,
    .SetPulseMotor = 0,
    .hDurationms = 200,
    .torqueLimit = 7940,
    .quick_stop_decel = 50,
};
SEVERO_CONTORL_PAR_T pCtrlPar_M2=
{
    .SetVelMotor = 0,
    .Vel_PLL_Motor = 0,
    .SetTorqueMotor = 0,
    .SetPulseMotor = 0,
    .hDurationms = 200,
    .torqueLimit = 7940,
    .quick_stop_decel = 50,
};

SEVERO_CONTORL_PAR_T pCtrlPar[NUMBER_OF_AXES];                         /*��������SEVERO_CONTORL_PAR_T���͵Ľṹ��*/

/*����ѧϰ��ز���*/
int16_t Hall_AvElAngle1 = 0;                                           /*M1һȦ��ƽ����Ƕ� = Hall_AvElAngle1Sum /Hall_AvElAngle1CNT */
int16_t Hall_AvElAngle1CNT = 0;                                        /*M1����������*/
int16_t Hall_AvElAngle2 = 0;                                           /*M2һȦ��ƽ����Ƕ�*/
int16_t Hall_AvElAngle2CNT = 0;                                        /*M2����������*/
int16_t Hall_AvElAngle2Temp,Hall_AvElAngle1Temp;
int16_t Angle_Switch2=3,Angle_Switch = 3;
uint16_t HallStudyCNT=0,HallStudyCNT2=0,Hall_EAC_Ticks = 3000;         /*����ѧϰ����ʱ*/
int32_t Hall_AvElAngle1Sum = 0;                                        /*M1һȦ��Ƕ��ܺ�*/
int32_t Hall_AvElAngle2Sum = 0;                                        /*M2һȦ��Ƕ��ܺ�*/
uint8_t HallStudyFlag1=0,HallStudyFlag2=0;                             /*����ѧϰ��־��ֻ�е���ʱʹ��*/
uint8_t HALL_CC_First=0,HALL2_CC_First=0,HALL_CC_First11=0;            /*����״������ж�����*/
/*------------------------------------------------
Function:�ٶ�ģʽ�ٶ�ģʽ����ģʽ�л�&����ָ��ִ�����
Input   :No
Output  :No
Explain :Modes_of_operation
					1 λ��ģʽ  	Ĭ���ٶ�ģʽ
					3 �ٶ�ģʽ
					4 ����ģʽ
         ���ڣ�500us
------------------------------------------------*/
void Independent_Motor_Control(void)
{
    static long delay_200ms[2]= {200,200};
    Drive_Data *pCiA402Axis;
    u16 counter = 0;
    static long location_cache[MAX_AXES] = {0};
    for(counter = 0; counter < MAX_AXES; counter++)
    {
        if(!Driver_Data[counter].device_control_data.bAxisIsActive)       /*ע�⸳��ֵ1*/
        {
            continue;
        }
        if(Driver_Data[M1].device_control_data.Modes_of_operation == 3\
                || Driver_Data[M1].device_control_data.Modes_of_operation == 4) /*�����M1�ٶ�ģʽ��������ģʽ����ôĬ�Ͻ���������Ĳ���ģʽ����Ϊһ�£���Ϊ��λ��ֻ����һ������Ŀ�����*/
        {
            Driver_Data[M2].device_control_data.Modes_of_operation = Driver_Data[M1].device_control_data.Modes_of_operation; /*ͬ��ģʽ�£�����M1����Ŀ���ģʽ���ɣ�*/
        }
        if(Sync_Async_Control == 1)                                       /*0��M1��M2�첽����  1��M1��M2ͬ�����ƣ��������⣩*/
        {
            Driver_Data[M2].device_control_data.Modes_of_operation = Driver_Data[M1].device_control_data.Modes_of_operation; /*ͬ��ģʽ�£�����M1����Ŀ���ģʽ���ɣ�*/
        }

        pCiA402Axis = &Driver_Data[counter];                              /*ָ�븳ֵ*/

        if(pCiA402Axis->device_control_data.Modes_of_operation == 1)      /*λ��ģʽ*/
        {
            if(modetemp[counter] != 1)                                    /*switch��ѡ��������Ϊλ��ģʽ*/
            {
                modetemp[counter] = 2;                                    /*case2 Ϊ �л���λ��ģʽ���м���̣����case2�������������л���λ��ģʽ*/
                P_V_I_Switch_using[counter] = 3;
            }
        }
        else if(pCiA402Axis->device_control_data.Modes_of_operation == 3) /*�ٶ�ģʽ*/
        {
            if(modetemp[counter] != 3)                                    /*switch��ѡ��������Ϊ�ٶ�ģʽ*/
            {
                pCtrlPar[M1].M1M2_VelSet = 0;                             /*���ͬ��ģʽ�µ��ٶȲο����ٶ�ģʽ�¸ò��������·��ٶȣ�����ģʽ�¸ò��������·����أ�*///
                pCtrlPar[M1].SetVelMotor = 0;                             /*���M1����첽ģʽ���ٶȲο�*/
                pCtrlPar[M2].SetVelMotor = 0;
                pCtrlPar[M1].SetTorqueMotor = 0;                          /*���M1����첽ģʽ�����زο�*/
                pCtrlPar[M2].SetTorqueMotor = 0;
                pPIDSpeed[M1]->wIntegralTerm = 0;                         /*�ٶȻ����������*/
                pPIDSpeed[M2]->wIntegralTerm = 0;
                modetemp[counter] = 5;                                    /*case5Ϊ�л����ٶ�ģʽ���м����������case5�������������л���case 3�ٶ�ģʽ*/
            }
        }
        else if(pCiA402Axis->device_control_data.Modes_of_operation == 4) /*����ģʽ*/
        {
            if(modetemp[counter]!=4)
            {
                /*�״ν�������ģʽ��Ҫ������زο�*/
                pCtrlPar[M1].M1M2_VelSet = 0;                             /*���ͬ��ģʽ�����زο����ٶ�ģʽ�¸ò��������·��ٶȣ�����ģʽ�¸ò��������·�����*/
                pCtrlPar[M1].SetTorqueMotor = 0;
                pCtrlPar[M2].SetTorqueMotor = 0;
                pCtrlPar[M1].SetVelMotor = 0;                             /*���M1����첽ģʽ���ٶȲο�*/
                pCtrlPar[M2].SetVelMotor = 0;
                pPIDSpeed[M1]->wIntegralTerm = 0;                         /*�ٶȻ����������*/
                pPIDSpeed[M2]->wIntegralTerm = 0;
                modetemp[counter] = 4;                                    /*���������л�������ģʽֱ���л���OK*/
            }
        }
        /*end*/
        if(Sync_Async_Control == 1)                                       /*0��M1��M2�첽����  1��M1��M2ͬ�����ƣ��������⣩*/
        {
            SpeedSetMerge_two_to_one(&pCtrlPar[M1],&pCtrlPar[M2],modetemp[counter]);
        }


        switch(modetemp[counter])
        {
        case 0:
            break;
        case 1 :                                                          /*��λ��ģʽ��*/

            if(counter == M1)                                             /*���1λ�û�*/
            {
                if(relative_location_flag[counter] == 1)                  /*���λ��ģʽ����*/
                {
                    relative_location_flag[counter] = 0;
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;    /*��λ�ÿ���ģʽ����*/
                    location_cache[counter] = ENCODER_M1._Super.wPulseNumber+pCtrlPar[M1].SetPulseMotor; /*ÿ�θ���λ���ڵ�ǰ��λ���ϼ��ϸ���λ��*/
                    MC_ProgramPositionCommandMotor1(location_cache[counter] , 0 );
                    location_cache[counter] = 0;                          /*��0,�´�ʹ��*/
                }
                else if(absolute_location_flag[counter] == 1)             /*����λ��ģʽ����*/
                {
                    absolute_location_flag[counter] = 0;
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;    /*��λ�ÿ���ģʽ����*/
                    MC_ProgramPositionCommandMotor1( pCtrlPar[M1].SetPulseMotor, 0 );
                }
            }
            else if(counter == M2)                                        /*���2λ�û�*/
            {
                if(relative_location_flag[counter] == 1)                  /*���λ��ģʽ����*/
                {
                    relative_location_flag[counter] = 0;                  /*ִ������Ժ��Զ���0����������Ҫÿ���·����λ���Ժ���д���λΪ1����ִ�����λ��ָ��ò���Ŀǰû�м�������ֵ䣬���Բ�����*/
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;    /*��λ�ÿ���ģʽ����*/
                    location_cache[counter] = ENCODER_M2._Super.wPulseNumber+pCtrlPar[M2].SetPulseMotor; /*ÿ�θ���λ���ڵ�ǰ��λ���ϼ��ϸ���λ��*/
                    MC_ProgramPositionCommandMotor2( location_cache[counter], 0 );
                    location_cache[counter] = 0;
                }
                else if(absolute_location_flag[counter] == 1)             /*����λ��ģʽ����*/
                {
                    absolute_location_flag[counter] = 0;
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;    /*��λ�ÿ���ģʽ����*/
                    MC_ProgramPositionCommandMotor2( pCtrlPar[M2].SetPulseMotor, 0 );
                }
            }
            break;
        case 2 :                                                          /*�ٶ�ģʽ��λ��ģʽ�л�*/
            if(counter == M1)
            {
                pCtrlPar[M1].SetVelMotor = 0 ;
                pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
                MC_ProgramSpeedRampMotor1(pCtrlPar[M1].SetVelMotor, 100 );
                delay_200ms[counter]--;
                if(delay_200ms[counter] < 0)
                {
                    delay_200ms[counter] = 200;
                    if(P_V_I_Switch_using[counter] == 2)                 /*���M1  λ�û�----->�ٶȻ�*/
                    {
                        if(pCtrlPar[M1].Vel_PLL_Motor==0)
                        {
                            modetemp[counter] = 3;                       /*�л����ٶȻ�*/
                            CNT_1MS[counter] = 0;
                            PositionModeFlag1 = 0;                       /*�ر�λ�û�����*/
                        }
                    }
                    else  if(P_V_I_Switch_using[counter] == 3)           /*���M1 �ٶȻ�----->λ�û�*/
                    {
                        if(pCtrlPar[M1].Vel_PLL_Motor ==0)
                        {
                            modetemp[counter] = 1;                       /*�л���λ�û�*/
                            pCtrlPar[M1].SetPulseMotor = MC_GetCurrentPosition1();
                            pPosCtrl[M1]->Theta =  pCtrlPar[M1].SetPulseMotor;
//                            if(pPosCtrl[M1]->pTrapezoidal->PreS != pPosCtrl[M1]->Theta)
//                            {
//                                pPosCtrl[M1]->pTrapezoidal->PreS = pPosCtrl[M1]->Theta;
//                            }
                        }
                    }
                }
            }
            else if(counter == M2)
            {
                pCtrlPar[M2].SetVelMotor = 0 ;
                pMCI[M2]->pPosCtrl->PositionControlRegulation = 0;
                MC_ProgramSpeedRampMotor2(pCtrlPar[M2].SetVelMotor, 100 );
                delay_200ms[counter]--;
                if(delay_200ms[counter] < 0)
                {
                    delay_200ms[counter] = 200;
                    if(P_V_I_Switch_using[counter] == 2)
                    {
                        if(pCtrlPar[M2].Vel_PLL_Motor ==0)
                        {
                            modetemp[counter] = 3;                       /*�л����ٶȻ�*/
                            CNT_1MS[counter] = 0;
                            PositionModeFlag2 = 0;                       /*�ر�λ�û�����*/
                        }
                    }
                    else  if(P_V_I_Switch_using[counter] == 3)
                    {
                        if(pCtrlPar[M2].Vel_PLL_Motor ==0)
                        {
                            modetemp[counter] = 1;                       /*�л���λ�û�*/
                            pCtrlPar[M2].SetPulseMotor = MC_GetCurrentPosition2();
//                            pPosCtrl[M2]->pTrapezoidal->S = pPosCtrl[M2]->Theta ;
                            pPosCtrl[M2]->Theta  =  pCtrlPar[M2].SetPulseMotor ;
//                            if(pPosCtrl[M2]->pTrapezoidal->PreS != pPosCtrl[M2]->pTrapezoidal->S)
//                            {
//                                pPosCtrl[M2]->pTrapezoidal->PreS = pPosCtrl[M2]->pTrapezoidal->S;
//                            }
                        }
                    }
                }
            }

            break;
        case 3 :  /*���ٶ�ģʽ��*/

            if(counter == M1&&STM[M1].bState > 5)
            {
                Countrer_1ms_CNT = pCtrlPar[M1].hDurationms+1000;
                if(LockAxleON_OFF == 0)                                                                               /*�ٶȻ�����*/
                {
                    if(PendingOpcode_handleWaitStatus[counter] == TRUE)                                               /*���ִ�п���ͣ������*/
                    {
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 1\
                                ||Driver_Data[counter].device_control_data.Quick_stop_option_code == 5)                     /*����6084���õļ��ٶ�ʱ��ͣ��*/
                        {
                            MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms);
                        }
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 2\
                                ||Driver_Data[counter].device_control_data.Quick_stop_option_code == 6)                      /*����6085���õļ��ٶ�ʱ��ͣ��*/
                        {
                            MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[M1].quick_stop_decel);  /*�����pCtrlPar[M1].quick_stop_decel��Ҫ�޸�M1M2����*/
                        }
                    }
                    else if(EMG_State3 ==STOPISOFF)                                                                    /*�������У���ͣ����δ������*/
                    {
                        MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms );
                    }

                    CurrentPosition1 =  MC_GetCurrentPosition1();
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
                    CNT_1MS[counter] = 0;
                    PositionModeFlag1 = 0;
                }
                else if(LockAxleON_OFF == 1)  /*M1λ�û�����δʹ�ã���Ƴ���Ϊ�˿����ٶȻ���б��������ܵ�������*/
                {
                    if(pCtrlPar[M1].SetVelMotor != 0)
                    {
                        MC_ProgramSpeedRampMotor1( pCtrlPar[M1].SetVelMotor, pCtrlPar[M1].hDurationms );
                        CurrentPosition1 =  MC_GetCurrentPosition1();
                        pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
                        CNT_1MS[counter] = 0;
                        PositionModeFlag1 = 0;
                    }
                    else
                    {
                        if(PositionModeFlag1 == 0)
                        {
                            if(CNT_1MS[counter] >= Countrer_1ms_CNT )
                            {
                                if(pCtrlPar[M1].Vel_PLL_Motor == 0)
                                {
                                    if((CNT_1MS[counter] < Countrer_1ms_CNT+50)&&(CNT_1MS[counter] > Countrer_1ms_CNT+10))
                                    {
                                        CurrentPosition1 =  MC_GetCurrentPosition1();
                                        pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;
                                        MC_ProgramPositionCommandMotor1( CurrentPosition1, 0 );
                                    }
                                    else if(CNT_1MS[counter] >= Countrer_1ms_CNT+50)
                                    {
                                        PositionModeFlag1 = 1;
                                        pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;
                                        CNT_1MS[counter] = Countrer_1ms_CNT+50 ;
                                    }
                                    else
                                    {
                                        MC_ProgramSpeedRampMotor1( pCtrlPar[M1].SetVelMotor, pCtrlPar[M1].hDurationms );
                                    }

                                }
                                else
                                {
                                    CNT_1MS[counter] = Countrer_1ms_CNT ;
                                    MC_ProgramSpeedRampMotor1( pCtrlPar[M1].SetVelMotor, pCtrlPar[M1].hDurationms );
                                }
                            }
                            else
                            {
                                MC_ProgramSpeedRampMotor1( pCtrlPar[M1].SetVelMotor, pCtrlPar[M1].hDurationms );
                                CurrentPosition1 =  MC_GetCurrentPosition1();
                                pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
                            }
                        }
                        else if(PositionModeFlag1 == 1)
                        {
                            CNT_1MS[counter] = Countrer_1ms_CNT+500;
                            pCtrlPar[M1].SetPulseMotor =  CurrentPosition1;
                            MC_ProgramPositionCommandMotor1( pCtrlPar[M1].SetPulseMotor, 0 );
                        }
                    }
                }
            }
            else if (counter == M2&&  STM[M2].bState > 5)
            {
                Countrer2_1ms_CNT = pCtrlPar[M2].hDurationms+1000;
                if(LockAxleON_OFF == 0)                                /*�ٶȻ�����*/
                {
                    if(PendingOpcode_handleWaitStatus[counter] == TRUE)/*���ִ�п���ͣ������*/
                    {
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 1||Driver_Data[counter].device_control_data.Quick_stop_option_code == 5)/*����6084���õļ��ٶ�ʱ��ͣ��*/
                        {
                            MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms);
                        }
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 2||Driver_Data[counter].device_control_data.Quick_stop_option_code == 6)/*����6085���õļ��ٶ�ʱ��ͣ��*/
                        {
                            MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[M1].quick_stop_decel);    /*�����pCtrlPar[M1].quick_stop_decelM1M2����   ��Ҫ�޸�*/
                        }
                    }
                    else if(EMG_State3 ==STOPISOFF)                    /*�������У���ͣ����δ������*/
                    {
                        MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms );
                    }
                    CurrentPosition2 =  MC_GetCurrentPosition2();
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 0;
                    CNT_1MS[counter] = 0;
                    PositionModeFlag2 = 0;
                }
                else if(LockAxleON_OFF == 1)                          /*M2λ�û�����δʹ�ã���Ƴ���Ϊ�˿����ٶȻ���б��������ܵ�������*/
                {   if(pCtrlPar[M2].SetVelMotor != 0)
                    {
                        MC_ProgramSpeedRampMotor2( pCtrlPar[M2].SetVelMotor, pCtrlPar[M2].hDurationms );
                        CurrentPosition2 =  MC_GetCurrentPosition2();
                        pMCI[M2]->pPosCtrl->PositionControlRegulation = 0;
                        CNT_1MS[counter] = 0;
                        PositionModeFlag2 = 0;
                    }
                    else
                    {
                        if(PositionModeFlag2 == 0)
                        {
                            if(CNT_1MS[counter] >= Countrer2_1ms_CNT )
                            {
                                if(pCtrlPar[M2].Vel_PLL_Motor == 0)
                                {
                                    if((CNT_1MS[counter] < Countrer2_1ms_CNT+50)&&(CNT_1MS[counter] > Countrer2_1ms_CNT+10))
                                    {
                                        CurrentPosition2 =  MC_GetCurrentPosition2();
                                        pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;
                                        MC_ProgramPositionCommandMotor2( CurrentPosition2, 0 );
                                    }
                                    else if(CNT_1MS[counter] >= Countrer2_1ms_CNT+50)
                                    {
                                        PositionModeFlag2 = 1;
                                        pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;
                                        CNT_1MS[counter] = Countrer2_1ms_CNT+50 ;
                                    }
                                    else
                                    {
                                        MC_ProgramSpeedRampMotor2( pCtrlPar[M2].SetVelMotor, pCtrlPar[M2].hDurationms );
                                    }

                                }
                                else
                                {
                                    CNT_1MS[counter] = Countrer2_1ms_CNT ;
                                    MC_ProgramSpeedRampMotor2( pCtrlPar[M2].SetVelMotor, pCtrlPar[M2].hDurationms );
                                }
                            }
                            else
                            {
                                MC_ProgramSpeedRampMotor2( pCtrlPar[M2].SetVelMotor, pCtrlPar[M2].hDurationms );
                                CurrentPosition2 =  MC_GetCurrentPosition2();
                                pMCI[M2]->pPosCtrl->PositionControlRegulation = 0;
                            }
                        }
                        else if(PositionModeFlag2 == 1)
                        {
                            CNT_1MS[counter] = Countrer2_1ms_CNT+500 ;
                            pCtrlPar[M2].SetPulseMotor =  CurrentPosition2;
                            MC_ProgramPositionCommandMotor2( pCtrlPar[M2].SetPulseMotor, 0 );
                        }
                    }
                }
            }
            break;
        case 4 :                                             /*��ǰ��������ģʽ*/
            if(counter==M1)
            {
                MC_ProgramTorqueRampMotor1(pCtrlPar[M1].SetTorqueMotor, 0);    /*�����·�*/
            }
            if(counter==M2)
            {
                MC_ProgramTorqueRampMotor2(pCtrlPar[M2].SetTorqueMotor, 0);
            }

            break;
        case 5 :                                             /*����ģʽ�л����ٶ�ģʽ��������ٶȾ͸���������ʹʵ���ٶ�����������0*/

            if(counter==M1)
            {
                int32_t k1 = 30,vel1 = 0,Iqtemp1 = 0;
                vel1 = pCtrlPar[M1].Vel_PLL_Motor;
                Iqtemp1 = -1*k1*vel1;                        /*k1���������Գ����Ĳ���*/
                if( 11900 < Iqtemp1)
                {
                    Iqtemp1 = 11900;
                }
                else if(Iqtemp1 < -11900)
                {
                    Iqtemp1 = -11900;
                }
                MC_ProgramTorqueRampMotor1(Iqtemp1, 100);   /*������������*/
            }
            else if(counter==M2)
            {
                int32_t k2 = 30,vel2 = 0,Iqtemp2 = 0;
                vel2 = pCtrlPar[M2].Vel_PLL_Motor;
                Iqtemp2 = -1*k2*vel2;
                if(Iqtemp2>11900)
                {
                    Iqtemp2 = 11900;
                }
                else if(Iqtemp2 < -11900)
                {
                    Iqtemp2 = -11900;
                }
                MC_ProgramTorqueRampMotor2(Iqtemp2, 100);
            }
            delay_200ms[counter]--;                            /*�����л����������������µ����ٶȲ�Ϊ0����һֱ�����л����ٶ�ģʽ�������ﳵ*/
            if(delay_200ms[counter] < 0)
            {
                delay_200ms[counter] = 100;
//                if(abs(pCtrlPar[counter].Vel_PLL_Motor) < 30)/*���ٶ����������Գ����Ĳ��ᷢ����������������δ˴�ʱΪ�����¼�ͣ��������ֹͣ*/
                {
                    pSTC[M1]->SpeedRefUnitExt   = -65460;      /*������ٶȲο������ڴ����ػ��л����ٶȻ��������*/
                    pSTC[M2]->SpeedRefUnitExt   = -65460;
                    modetemp[counter] = 3;                     /*�л����ٶȻ�*/
                }
            }
            break;

        default:
            break;
        }
    }
}
/*------------------------------------------------
Function:led������
Input   :led1: 1/��������  0/�����Ʋ���
				 led2: 1/��������  0/�����Ʋ���
				 led_time:��˸�Ĵ���
				 led_mode:1/����һ�� 0/����
				 led_speed:��˸���ٶ�
Output  :No
Explain :No
------------------------------------------------*/
void LEDSet(u8 led1,u8 led2,u8 led_time,u8 led_mode,u8 led_speed)
{
    /*led1*/
    if(led1==0)
    {
        RED_LED1_OFF;
    }
    else if(led_mode)                            	  /*����*/
    {
        if(DisplayCounter<2*led_speed)              /*10ms����һ��*/
        {
            RED_LED1_ON;
        }
        else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
        {
            RED_LED1_TOGGLE;
        }
    }
    else if(DisplayCounter<=(2*led_time)*led_speed)	/*����*/
    {
        if((DisplayCounter-1)%led_speed==0)
        {
            if((DisplayCounter/led_speed)%2==0)
            {
                RED_LED1_ON;
            }
            else
            {
                RED_LED1_OFF;
            }
        }
    }
    else if(DisplayCounter<2*(led_time+1)*led_speed)
    {
        RED_LED1_OFF;
    }

    //led2
    if(led2==0)
    {
        RED_LED2_OFF;
    }
    else if(led_mode)
    {
        if(DisplayCounter<2*led_speed)
        {
            RED_LED2_ON;
        }
        else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
        {
            RED_LED2_TOGGLE;
        }
    }
    else if(DisplayCounter<=(2*led_time)*led_speed)
    {
        if((DisplayCounter-1)%led_speed==0)
        {
            if((DisplayCounter/led_speed)%2==0)
            {
                RED_LED2_ON;
            }
            else
            {
                RED_LED2_OFF;
            }
        }
    }
    else if(DisplayCounter<2*(led_time+1)*led_speed)
    {
        RED_LED2_OFF;
    }

    //time clear
    if(led_mode)
    {
        if(DisplayCounter>(4*led_time+2)*led_speed)
        {
            DisplayCounter=0;
        }
    }
    else if(DisplayCounter>(2*led_time+2)*led_speed)
    {
        DisplayCounter=0;
    }
}
/*------------------------------------------------
Function:���ô����־
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u32 wGlobal_Flags = 0;
u32 display_led = 0;
void MC_SetFault(u32 hFault_type)
{
    wGlobal_Flags |= hFault_type;         /*ȫ�ִ���ָʾ*/
    if(display_led !=0)                   /*�����ǰ�Ѿ���һ�������ڷ���*/
    {
        display_led |=hFault_type;        /*��������*/
        display_led &=hFault_type;        /*�����һ��������ʾ��ǰ�����Ĵ���*/
    }
    else
        display_led  |= hFault_type;      /*ֻ������ǰ��������*/
}
/*------------------------------------------------
Function:��������־
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_ClearFault(u32 hFault_type)
{
    wGlobal_Flags &= ~hFault_type;        /*���ȫ�ִ���*/
    display_led &= ~hFault_type;          /*�����ǰ��������*/

}
/*------------------------------------------------
Function:�ֱ�����������Ĵ���λ������������wGlobal_Flags��������
Input   :No
Output  :No
Explain :����֮���ȡһ�������Ϳ��Բ鿴���еĴ���Ҳ����ָʾ�Ƶ���ʾ
------------------------------------------------*/
void Display_error(void)
{
    /*M1M2��������*/
    if((STM[M1].hFaultOccurred&MC_OVER_VOLT)==MC_OVER_VOLT||(STM[M2].hFaultOccurred&MC_OVER_VOLT)==MC_OVER_VOLT)    /*����������������һ������������λ*/
    {
        MC_SetFault(MC_OVER_VOLT);                                                                                  /*��ô����λȫ�ֱ����еĴ���ָʾ*/
    }
    else
    {
        MC_ClearFault(MC_OVER_VOLT);
    }
    if((STM[M1].hFaultOccurred&MC_UNDER_VOLT)==MC_UNDER_VOLT||(STM[M2].hFaultOccurred&MC_UNDER_VOLT)==MC_UNDER_VOLT)/*ע��ͬ��*/
    {
        MC_SetFault(MC_UNDER_VOLT);
    }
    else
    {
        MC_ClearFault(MC_UNDER_VOLT);
    }
    if((STM[M1].hFaultOccurred&ERR_CAN_COMMUNICATION)==ERR_CAN_COMMUNICATION||(STM[M2].hFaultOccurred&ERR_CAN_COMMUNICATION)==ERR_CAN_COMMUNICATION)
    {
        MC_SetFault(ERR_CAN_COMMUNICATION);                                                                         /*��ô����λȫ�ֱ����еĴ���ָʾ*/
    }
    else
    {
        MC_ClearFault(ERR_CAN_COMMUNICATION);
    }
    /*����������еĴ���ʹ��forѭ�����жϣ�ԭ��һ��ֻ�Ǵ�����Ŀ��*/
    for(u8 i =0; i<2; i++)
    {
        if((STM[i].hFaultOccurred&MC_BREAK_IN)==MC_BREAK_IN)    /*M1��·������λ*/
        {
            if(i==0)
            {
                if((wGlobal_Flags&MC_BREAK_IN_M1)==0)          /*���ȫ�ִ�����û�иô���*/
                {
                    MC_SetFault(MC_BREAK_IN_M1);               /*��ô��λȫ�ִ����־*/
                }
            }
            else
            {
                if((wGlobal_Flags&MC_BREAK_IN_M2)==0)          /*M2���*/
                {
                    MC_SetFault(MC_BREAK_IN_M2);
                }
            }
        }
        else                                                   /*������������������������ȫ�ִ���*/
        {
            if(i==0)
            {
                MC_ClearFault(MC_BREAK_IN_M1);
            }
            else
            {
                MC_ClearFault(MC_BREAK_IN_M2);
            }
        }
        if((   STM[i].hFaultOccurred&ERR_CURR_13_OVER_LOAD)==ERR_CURR_13_OVER_LOAD\
                ||(STM[i].hFaultOccurred&ERR_CURR_15_OVER_LOAD)==ERR_CURR_15_OVER_LOAD\
                ||(STM[i].hFaultOccurred&ERR_CURR_OVER_MAX)    ==ERR_CURR_OVER_MAX)    /*���н׶εĹ���������ͬһ����������*/

        {
            if(i==0)
            {
                if((wGlobal_Flags&ERR_CURR_OVER_MAX_M1)==0)
                {
                    MC_SetFault(ERR_CURR_OVER_MAX_M1);
                }
            }
            else
            {
                if((wGlobal_Flags&ERR_CURR_OVER_MAX_M2)==0)
                {
                    MC_SetFault(ERR_CURR_OVER_MAX_M2);
                }
            }
        }
        else
        {
            if(i==0)
            {
                MC_ClearFault(ERR_CURR_OVER_MAX_M1);
            }
            else
            {
                MC_ClearFault(ERR_CURR_OVER_MAX_M2);
            }
        }
        if((STM[i].hFaultOccurred&ERR_MOTOR_STUCK_ERR)==ERR_MOTOR_STUCK_ERR)
        {
            if(i==0)
            {
                if((wGlobal_Flags&ERR_MOTOR_STUCK_ERR_M1)==0)
                {
                    MC_SetFault(ERR_MOTOR_STUCK_ERR_M1);
                }
            }
            else
            {
                if((wGlobal_Flags&ERR_MOTOR_STUCK_ERR_M2)==0)
                {
                    MC_SetFault(ERR_MOTOR_STUCK_ERR_M2);
                }
            }
        }
        else
        {
            if(i==0)
            {
                MC_ClearFault(ERR_MOTOR_STUCK_ERR_M1);
            }
            else
            {
                MC_ClearFault(ERR_MOTOR_STUCK_ERR_M2);
            }
        }
        if((STM[i].hFaultOccurred&ERR_OVER_VEL_ERR)==ERR_OVER_VEL_ERR)
        {
            if(i==0)
            {
                if((wGlobal_Flags&ERR_OVER_VEL_ERR_M1)==0)
                {
                    MC_SetFault(ERR_OVER_VEL_ERR_M1);
                }

            }
            else
            {
                if((wGlobal_Flags&ERR_OVER_VEL_ERR_M2)==0)
                {
                    MC_SetFault(ERR_OVER_VEL_ERR_M2);
                }

            }
        }
        else
        {
            if(i==0)
            {
                MC_ClearFault(ERR_OVER_VEL_ERR_M1);
            }
            else
            {
                MC_ClearFault(ERR_OVER_VEL_ERR_M2);
            }
        }
        if((STM[i].hFaultOccurred&ERR_LOST_PHASE)==ERR_LOST_PHASE)
        {
            if(i==0)
            {
                if((wGlobal_Flags&ERR_LOST_PHASE_M1)==0)
                {
                    MC_SetFault(ERR_LOST_PHASE_M1);
                }
            }
            else
            {
                if((wGlobal_Flags&ERR_LOST_PHASE_M2)==0)
                {
                    MC_SetFault(ERR_LOST_PHASE_M2);
                }
            }
        }
        else
        {
            if(i==0)
            {
                MC_ClearFault(ERR_LOST_PHASE_M1);
            }
            else
            {
                MC_ClearFault(ERR_LOST_PHASE_M2);
            }
        }

        if((STM[i].hFaultOccurred&ERR_HALL)==ERR_HALL)
        {
            if(i==0)
            {
                if((wGlobal_Flags&ERR_HALL_M1)==0)
                {
                    MC_SetFault(ERR_HALL_M1);
                }
            }
            else
            {
                if((wGlobal_Flags&ERR_HALL_M2)==0)
                {
                    MC_SetFault(ERR_HALL_M2);
                }
            }
        }
        else
        {
            if(i==0)
            {
                MC_ClearFault(ERR_HALL_M1);
            }
            else
            {
                MC_ClearFault(ERR_HALL_M2);
            }
        }
        if((STM[i].hFaultOccurred&MC_OVER_TEMP)==MC_OVER_TEMP)
        {
            if(i==0)
            {
                if((wGlobal_Flags&MC_OVER_TEMP_M1)==0)
                {
                    MC_SetFault(MC_OVER_TEMP_M1);
                }
            }
            else
            {
                if((wGlobal_Flags&MC_OVER_TEMP_M2)==0)
                {
                    MC_SetFault(MC_OVER_TEMP_M2);
                }
            }
        }
        else
        {
            if(i==0)
            {
                MC_ClearFault(MC_OVER_TEMP_M1);
            }
            else
            {
                MC_ClearFault(MC_OVER_TEMP_M2);
            }
        }
    }
}
/*------------------------------------------------
Function:����������ָʾ
Input   :No
Output  :No
Explain :10ms����һ�Σ�ִ������શ�ʱ������,
------------------------------------------------*/
void DisplayErrorLed_Handle(void)
{
    static u8 DisplayGreenCnt;
    DisplayCounter++;                                     /*���ں����˸Ƶ��*/
    DisplayGreenCnt++;                                    /*�����̵�һֱ��˸*/

    if(DisplayGreenCnt>100)
    {
        GREEN_LED_TOGGLE;
        DisplayGreenCnt=0;
    }
    if(wGlobal_Flags==MC_NO_ERROR)
    {
        RED_LED1_OFF;
        RED_LED2_OFF;
    }
    else
    {
        switch(display_led)
        {
        case MC_OVER_VOLT:                                 /*������Ƴ���*/
            RED_LED1_ON;
            RED_LED2_ON;
            break;
        case MC_UNDER_VOLT:                                /*Ƿѹ��������*/
            RED_LED2_OFF;
            RED_LED1_ON;
            break;
        case ERR_CAN_COMMUNICATION:                        /*�����ƿ���1��*/
            LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
            break;

        case ERR_CURR_OVER_MAX_M1:                         /*�����ƿ���2��*/
            LEDSet(LED_OFF,LED_ON,2,0,LED_HIGH_SPEED);
            break;
        case ERR_CURR_OVER_MAX_M2:                         /*�����ƿ���2��*/
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case ERR_OVER_VEL_ERR_M1:                          /*�����ƿ���3��*/
            LEDSet(LED_OFF,LED_ON,3,0,LED_HIGH_SPEED);
            break;
        case ERR_OVER_VEL_ERR_M2:                          /*�����ƿ���3��*/
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case ERR_MOTOR_STUCK_ERR_M1:                       /*�����ƿ���4��*/
            LEDSet(LED_OFF,LED_ON,4,0,LED_HIGH_SPEED);
            break;
        case ERR_MOTOR_STUCK_ERR_M2:                       /*�����ƿ���4��*/
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;

        case MC_BREAK_IN_M1:                               /*�����ƿ���5��*/
            LEDSet(LED_OFF,LED_ON,5,0,LED_HIGH_SPEED);
            break;
        case MC_BREAK_IN_M2:                               /*�����ƿ���5��*/
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;
        case ERR_LOST_PHASE_M1:                            /*�����ƿ���6��*/
            LEDSet(LED_OFF,LED_ON,6,0,LED_HIGH_SPEED);
            break;
        case ERR_LOST_PHASE_M2:                            /*�����ƿ���6��*/
            LEDSet(LED_ON,LED_OFF,6,0,LED_HIGH_SPEED);
            break;

        case ERR_HALL_M1:                                  /*�����ƿ���7��*/
            LEDSet(LED_OFF,LED_ON,7,0,LED_HIGH_SPEED);
            break;
        case ERR_HALL_M2:                                  /*�����ƿ���7��*/
            LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
            break;
        case MC_OVER_TEMP_M1:                              /*�����ƿ���8��*/
            LEDSet(LED_OFF,LED_ON,8,0,LED_HIGH_SPEED);
            break;
        case MC_OVER_TEMP_M2:                              /*�����ƿ���8��*/
            LEDSet(LED_ON,LED_OFF,8,0,LED_HIGH_SPEED);
            break;
//    case ERR_CURR_OVER_MAX:
//        break;
//    case ERR_MOTOR_STUCK_ERR:
//        break;
//    case ERR_OVER_VEL_ERR:
//        break;

        default:
            RED_LED1_OFF;
            RED_LED2_OFF;//�˴����������緢���˲����Զ��ָ��Ĵ����ٷ������Զ��ָ��Ĵ��󣨱���Ƿѹ��ѹ�������Զ��ָ�����֮�󣬲��ɻָ��Ĵ��󽫲���ʾ��ƣ���������Ϊ��ƹرգ��̵Ʋ���˸��
            //2022.3.17���������˵bug���Զ��ָ�֮���ж�ȫ�ִ������λ�Ĵ��󣬽����λ�Ĵ���ֵ��display_led
            for(u8 i = 0; i<32; i++)
            {
                u32 temp = 0;
                temp = (wGlobal_Flags>>i)&0x0000001;
                if(temp==1)
                {
                    display_led = temp<<i;
                    break;
                }
            }
            break;
        }
    }
}
/*------------------------------------------------
Function:�ο�ע�룬��������Ӧλ�á��ٶȺ͵������ƻ�·
Input   :@injectPoint
         @injectType ��������
         @injectValue
         @injectFreq Ƶ��
         @injectAmp  ��ֵ
         @injectInit
         @*refOut  ������������ο������ٶȲο�
Output  :No
Explain :��������SIN���ߡ��������ڲ��Ե��������ٶȻ���Ӧ���޸������һ����ڲ�����������long ��Ϊint32_t,��Ϊ��������ο�������int32_t����ᱨ�澯��
------------------------------------------------*/
void ReferenceInjection( \
                         long	*injectPoint, \
                         long	*injectType, \
                         float	*injectPhase, \
                         long	*injectValue, \
                         long	*injectFreq, \
                         long	*injectAmp, \
                         long	*injectInit, \
                         int32_t	*refOut \
                       )
{
    float Phasecycle;
//	// calc inject phase first
//	if ((*injectPoint == INJECT_POINT_POSREF) || (*injectPoint == INJECT_POINT_VELREF))
//	{
//		*injectPhase += 65535 * SAMPLE_TIME * (*injectFreq);
//	}
//	else
    {   //(opt)
        Phasecycle = 65535*SAMPLE_TIME;
        Phasecycle = Phasecycle*(*injectFreq);
        *injectPhase = *injectPhase + Phasecycle;
    }
    if (*injectPhase > 65535)
    {
        *injectPhase -= 65535;
    }

    switch (*injectType)
    {
    case INJECT_TYPE_NONE:					  /*none*/
    {
        *injectPhase = 0.0;

        if (*injectPoint == INJECT_POINT_POSREF)	// a little special when pos ref inject
        {
            *injectInit = *refOut;
        }

        break;
    }

    case INJECT_TYPE_SIN_DIRECT:			 /*sin*/
    {
        Vector_Components = MCM_Trig_Functions((int16_t)(*injectPhase));   /*time:530ns��math..�еĿ⺯��*/
        *injectValue = (long) (*injectAmp * Vector_Components.hSin/32768);
        if (*injectPoint == INJECT_POINT_POSREF)
        {
            *refOut = *injectInit + *injectValue;
        }
        else
        {
            *refOut = *injectValue;
        }

        break;
    }

    case INJECT_TYPE_SQUARE_DIRECT:			/*���β�*/
    {
        if (*injectPhase < 32768)
        {
            *injectValue = *injectAmp;
        }
        else
        {
            *injectValue = -(*injectAmp);
        }

        if (*injectPoint == INJECT_POINT_POSREF)
        {
            *refOut = *injectInit + *injectValue;
        }
        else
        {
            *refOut = *injectValue;
        }

        break;
    }

    default:
        break;
    }
}
/*------------------------------------------------
* @function :sin���ߣ���·���Ե������ٶȻ���Ӧ������ʼ��
* @input    :����д
* @output   :����д
* @explain  :ReferenceInjection�ú����Ĳ���
* @author   :����
* @date     :2022/12/16
------------------------------------------------*/
void ReferenceInjectionLoopPar_Init(void)
{
    loopTestPar.InjectPoint[M1] = INJECT_POINT_CURRREF;
    loopTestPar.InjectType[M1] = INJECT_TYPE_SIN_DIRECT;  /*sin����*/
    loopTestPar.InjectFreq[M1] = 3;
    loopTestPar.InjectCurrAmp[M1] = 1000;

    loopTestPar.InjectPoint[M2] = INJECT_POINT_CURRREF;
    loopTestPar.InjectType[M2] = INJECT_TYPE_SIN_DIRECT;  /*sin����*/
    loopTestPar.InjectFreq[M2] = 3;
    loopTestPar.InjectCurrAmp[M2] = 1000;
}

/*------------------------------------------------
Function:�����������
Input   :No
Output  :No
Explain :δʹ��
------------------------------------------------*/
void TorqueLimit(u8 axes)
{
    if(FOCVars[axes].Iqdref.q > pCtrlPar[axes].torqueLimit)
    {
        FOCVars[axes].Iqdref.q = pCtrlPar[axes].torqueLimit;
    }
    else if(FOCVars[axes].Iqdref.q < -pCtrlPar[axes].torqueLimit)
    {
        FOCVars[axes].Iqdref.q = -pCtrlPar[axes].torqueLimit;
    }
}
/*------------------------------------------------
Function:��ת���
Input   :No
Output  :������룺0x00001000
Explain :ִ�����ڣ�500us  ��תʱ��2000ms  ��ת�ٶ� 5RPM ��ת������Iq��
------------------------------------------------*/
uint32_t MotorStuckCheck(u8 axes)
{
    uint32_t errorcode = MC_NO_ERROR;
    if(1&&STM[axes].bState == RUN)           /*��ת���ش򿪵��ʹ��*/
    {
        if((in32abs(FOCVars[axes].Iqdref.q) > MotorParameters[axes].ShortCircuit)\
                && (in32abs(pCtrlPar[axes].Vel_PLL_Motor) <= MotorParameters[axes].LockedSpeed)) /*�ο���������ת������ʵ���ٶȣ���ת�ٶ�*/
        {
            if(in32abs(FOCVars[axes].Iqd.q) > MotorParameters[axes].ShortCircuit/2)          /*�ο���������ת������һ��*/
            {
                giStuckTimeCnt[axes]++;
            }

            if(giStuckTimeCnt[axes] > MotorParameters[axes].LockeTicks)                      /* ��תʱ��2000ms ����*/
            {
                errorcode = ERR_MOTOR_STUCK_ERR;
                giStuckTimeCnt[axes] = 0;
                return errorcode;
            }
        }
        else if(giStuckTimeCnt[axes]>0)
        {
            giStuckTimeCnt[axes]--;
        }
    }
    else
    {
        if(giStuckTimeCnt[axes]>0)
        {
            giStuckTimeCnt[axes]--;
        }
    }
    return errorcode;
}
/*------------------------------------------------
Function:��������
Input   :No
Output  :No
Explain :Ϊ��ȱ���⵱�еĵ���������������֮���������
------------------------------------------------*/
void swap(float *x,float *y)
{
    float temp= 0;
    temp=*x;
    *x=*y;
    *y=temp;
}
/*------------------------------------------------
* @function :���㷨��a[]�е�Ԫ�ش�С�������������
* @input    :@param��Ҫ��������� @param�����С
* @output   :����д
* @explain  :����д
* @author   :����
* @date     :2022/12/16
------------------------------------------------*/
void BubbleSort(float a[], float n)
{
    uint32_t j= 0;
    for(uint32_t i = 0; i < n - 1; i++)
    {
        for(j = n - 1; j > i; j--)
        {
            if(a[j - 1]>a[j])
            {
                swap(&a[j - 1],&a[j]);      /*Ϊ������������a[j] �� a[j - 1] ���н���*/
            }
            else
            {}
        }
    }
}
/*------------------------------------------------
Function:ȱ����ĳ�ʼ������
Input   :No
Output  :No
Explain :��ֹ�ڵ����ͣʱ��Ĳ������ң����µ�ȱ����
------------------------------------------------*/
void ClearPhaseCheckPar(void)
{
    IabcCurrentTemp[M1][0] = 0;           /*�����������*/
    IabcCurrentTemp[M1][1] = 0;
    IabcCurrentTemp[M1][2] = 0;
    lostPhaseCheckCnt[M1] = 0;            /*���������������*/
    Missing_phase_error_code[M1] = 0;     /*������뻺��*/
    cur_calculated_value[M1] = 0;         /*�Ƚ�ֵ���*/
    iabcurrenttemporary[0] = 0;           /*��Ҫ�������������ۼӺ����*/
    iabcurrenttemporary[1] = 0;
    iabcurrenttemporary[2] = 0;


    IabcCurrentTemp[M2][0] = 0;           /*M2������*/
    IabcCurrentTemp[M2][1] = 0;
    IabcCurrentTemp[M2][2] = 0;
    lostPhaseCheckCnt[M2] = 0;
    Missing_phase_error_code[M2] = 0;
    cur_calculated_value[M1] = 0;
    iabcurrenttemporary[0] = 0;
    iabcurrenttemporary[1] = 0;
    iabcurrenttemporary[2] = 0;
}/*------------------------------------------------
Function��ȱ����Ĵ��󱨾�����
Input   :No
Output  :No
Explain :�ú�����Ϊ�˽�ADC�жϵ��е�ȱ���ж���500us���еĹ��ϴ�������ϵ��һ��
------------------------------------------------*/
uint32_t DefaultPhaseCheckLinkFunction(u8 axes)
{
    uint32_t temp = 0;
    temp = Missing_phase_error_code[axes];
    Missing_phase_error_code[axes] = 0;      /*������������ȱ��������RUN״̬�м��㣬�ڱ���֮���������ڽ��м�����Ա���֮����ʾFAULT_NOW*/
    return temp;
}
/*------------------------------------------------
* @function :int32�������������ֵ
* @input    :����д
* @output   :����д
* @explain  :����д
* @author   :����
* @date     :2022/12/16
------------------------------------------------*/
int32_t in32abs(int32_t a)
{
    int32_t temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;

    return temp;
}
/*------------------------------------------------
* @function :long�������������ֵ
* @input    :����д
* @output   :����д
* @explain  :����д
* @author   :����
* @date     :2022/12/16
------------------------------------------------*/
long l_abs(long a)
{
    long temp;
    if(a>0) temp = a;
    else if(a<0) temp = -a;
    return temp;
}
/*------------------------------------------------
Function:ȱ���⺯��
Input   :No
Output  :No
Explain :50us����һ��
------------------------------------------------*/
uint32_t DefaultPhaseCheck(u8 axes)
{
//    static uint32_t  giPhaseTimeCnt[NUMBER_OF_AXES] = {0,0};
    uint32_t errorcode = MC_NO_ERROR;
    if(STM[axes].bState == RUN&&pCtrlPar[axes].SetVelMotor!=0\
            &&l_abs(FOCVars[axes].Iqdref.q) > l_abs(pPIDSpeed[axes]->hUpperOutputLimit-1))   /*������ʱ���Ѿ�ȱ�ࡿ,�ο������ܴ�*/
    {
        if(in32abs(FOCVars[axes].Iqd.q)<100&&pCtrlPar[axes].Vel_PLL_Motor<50)           /*���û������Iq<100�����ٶ�С��5RPM��������*/
        {
            lostPhaseCheckCntForLaunch[axes]++;                                         /*ȱ�������1*/
        }
        else
        {
            if(lostPhaseCheckCntForLaunch[axes]>0)
            {
                lostPhaseCheckCntForLaunch[axes]--;
            }
        }
        if(lostPhaseCheckCntForLaunch[axes]>(uint16_t)(MISSING_PAHSE_JUDGE_TIME/3))                   /*����300ms���󱨴��������������� �˴�������1S ��Ҫ��ͬ��
					                                                            �˴��������ϳ��ֹ�һ�Σ����һ������û�壬��һ�����ӻ�����쳣����ת����
				                                                              ���ʱ������������������ѱ�����*/
        {
            errorcode = ERR_LOST_PHASE;
            lostPhaseCheckCntForLaunch[axes] = 0;                                       /*���������� */
        }
    }
    else
    {
        if(lostPhaseCheckCntForLaunch[axes]>0)
        {
            lostPhaseCheckCntForLaunch[axes]--;
        }
    }
    if(1&&STM[axes].bState == RUN && abs(pCtrlPar[axes].SetVelMotor)>100)              /*�����й�����ȱ�ࡿ  ��������ٶ����ƣ�ͣ��qidong��ʱ�����ȱ�࣬�˴���׮���Ƶ�����Ӱ��ȱ��*/
    {
        lostPhaseCheckCnt[axes]++;                                                     /*ȱ�������2*/
        if(lostPhaseCheckCnt[axes] > MISSING_PAHSE_JUDGE_TIME)                         /*ȱ����һ����������Ϊ1000ms*/
        {
            for(u8 i =0; i<3; i++)
            {
                iabcurrenttemporary[i] = IabcCurrentTemp[axes][i];                     /*��������ʱ�����ݼ�¼������ʹ��*/
            }
            BubbleSort(iabcurrenttemporary,3);
            cur_calculated_value[axes] = (iabcurrenttemporary[2] - iabcurrenttemporary[0])/iabcurrenttemporary[2];  /*���ֵ��ȥ��Сֵ�������ֵ����ֵ��Χ0-1 ����������ֵԼΪ0.0�����쳣Խ�ӽ�1*/
            if(cur_calculated_value[axes] > 0.8f)                                      /*�Ѿ���������0.8Ϊ����ֵ*/
            {
                errorcode = ERR_LOST_PHASE;
                IabcCurrentTemp[axes][0] = 0;
                IabcCurrentTemp[axes][1] = 0;
                IabcCurrentTemp[axes][2] = 0;
                lostPhaseCheckCnt[axes] = 0;
            }
            else //if(lostPhaseCheckCnt[axes] > 6000)                                  /*û�з���������һ���������ڽ�����һ����������Ϊ300ms*/
            {
                IabcCurrentTemp[axes][0] = 0;
                IabcCurrentTemp[axes][1] = 0;
                IabcCurrentTemp[axes][2] = 0;
                lostPhaseCheckCnt[axes] = 0;
            }
        }
        else                                                                           /*��һ�����������ڣ����е��������ۼ�*/
        {
            if(FOCVars[axes].Iab.a>0)                                                  /*�����Ϊsin*/
            {
                IabcCurrentTemp[axes][0] += FOCVars[axes].Iab.a;
            }
            else
            {
                IabcCurrentTemp[axes][0] += (-FOCVars[axes].Iab.a);
            }
            if(FOCVars[axes].Iab.b>0)                                                 /*�����Ϊsin*/
            {
                IabcCurrentTemp[axes][1] += FOCVars[axes].Iab.b;
            }
            else                                                                      /*���IbС��0*/
            {
                IabcCurrentTemp[axes][1] += (-FOCVars[axes].Iab.b);
            }
            if(((FOCVars[axes].Iab.a+FOCVars[axes].Iab.b)*-1)>0)                      /*��� Ic > 0*/
            {
                IabcCurrentTemp[axes][2] += (-1)*(FOCVars[axes].Iab.a+FOCVars[axes].Iab.b);
            }
            else                                                                      /*��� Ic < 0*/
            {
                IabcCurrentTemp[axes][2] += (FOCVars[axes].Iab.a+FOCVars[axes].Iab.b);
            }
        }
    }
    else
    {
        IabcCurrentTemp[axes][0] = 0;
        IabcCurrentTemp[axes][1] = 0;
        IabcCurrentTemp[axes][2] = 0;
        lostPhaseCheckCnt[axes] = 0;
        cur_calculated_value[axes] = 0;
    }
    return errorcode;
}

/*------------------------------------------------
Function:ʧ�ټ��
Input   :No
Output  :������룺ERR_OVER_VEL_ERR 0x00010000
Explain :ִ�����ڣ�500us ���⣺����BUG����ٶ�0�ٶ�ʧ��֮����ת���,����֮��ֱ��ʹ�ܵ�����ܻؽ����������
��ʧ�ܣ�λ�û�������֮ǰ��λ��
MotorParameters[axes].MAXSpeed�ñ���Ϊ�������õ��������ٶ����ƣ���ֵ��Ҫ��������PID���ڵĹ���
------------------------------------------------*/
uint32_t StallCheck(u8 axes)
{
    int32_t veltemp = 0;
    uint32_t errorcode = MC_NO_ERROR;
    uint16_t MAX_giStallTimeCnt[NUMBER_OF_AXES] = {200,200};       /*100ms����*/
//  int32_t  SetVel_Multiply_VelPLL = 0;
    if(1&&STM[axes].bState == RUN)                                 /*ʧ�ٱ������ش�&&���ʹ��*/
    {
        veltemp = in32abs(pCtrlPar[axes].Vel_PLL_Motor);
        if(veltemp > MotorParameters[axes].MAXSpeed)               /*��������ٶȱ���֮*/
        {
            giStallTimeCnt[axes]++;
            if(giStallTimeCnt[axes]>MAX_giStallTimeCnt[axes])      /*100ms����*/
            {
                errorcode = ERR_OVER_VEL_ERR;
                giStallTimeCnt[axes] = 0;
                return errorcode;
            }
        }
        else if(giStallTimeCnt[axes]>0)                            /*��������ת�ٷ�Χ*/
        {
            giStallTimeCnt[axes]--;
        }
    }
    else
    {
        giStallTimeCnt[axes] = 0;                                   /*���δʹ��ֱ������*/
    }
    return errorcode;
}
/*------------------------------------------------
Function:MCU�汾��
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver,u32 hardware_version)
{
    pVersion->uVersionPartOfRobotType = robotype;				              /*��Ŀ����	S��*/
    pVersion->uVersionPartOfMainVersion = main_ver;			              /*��汾��*/
    pVersion->uVersionPartOfFunVersion = fun_ver;						          /*���ܰ汾��*/
    pVersion->uVersionPartOfSmallVersion = small_ver;		              /*С�汾�ţ�����bug�Լ�����΢��*/
    pVersion->uVersionFullVersion = (robotype<<24)+(main_ver<<16)+(fun_ver<<8)+small_ver;
}
/*------------------------------------------------
Function:Ӳ���汾��
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void HardVersion_Init(MCU_Version *pVersion,u8 funtype,u8 vol,u8 cur_max,u8 update_ver)
{
    pVersion->uHardwareVersion = (funtype<<24)+(vol<<16)+(cur_max<<8)+update_ver;
}
/*------------------------------------------------
Function:�ϲ��������ʵ��ת��
Input   :No
Output  :No
Explain :��16λΪM1�������16λΪM2���
------------------------------------------------*/
void SpeedRelMerge_two_to_one(SEVERO_CONTORL_PAR_T *parM1,SEVERO_CONTORL_PAR_T *parM2)
{
    int32_t Veltemp1,Veltemp2;

    Veltemp1 = parM2->Vel_PLL_Motor&0x0000FFFF;        /*�ó���16λ*/
    Veltemp2 = Veltemp1<<16;
    Veltemp1 = parM1->Vel_PLL_Motor&0x0000FFFF;
    parM1->M1M2_VelRel = Veltemp1|Veltemp2;             /*M1Ϊ��16λ��M2λ��16λ*/
}
/*------------------------------------------------
Function:��������������ת��
Input   :No
Output  :No
Explain :��16λΪM1�������16λΪM2���
------------------------------------------------*/
void SpeedSetMerge_two_to_one(SEVERO_CONTORL_PAR_T *parM1,SEVERO_CONTORL_PAR_T *parM2,uint8_t mode)
{
    int16_t m1veltemp,m2veltemp;

    m1veltemp = parM1->M1M2_VelSet>>16;                  /*ȡ��16λ*/
    m2veltemp = parM1->M1M2_VelSet&0X0000FFFF;           /*ȡ��16λ*/
    if(mode == 3)                                        /*�ٶ�ģʽ*/
    {
        parM1->SetTorqueMotor = 0;                       /*�ٶ�ģʽ�½���������Ϊ0*/
        parM2->SetTorqueMotor = 0;
        parM1->SetVelMotor = -m2veltemp;                 /*��16λ��M1,��Ҫ��ֵ��16λ�ı������ٽ��ñ�����ֵ��32λ*/
        parM2->SetVelMotor = -m1veltemp;                 /*��16λ��M2�������ڸ�-�ٶȴ�������  �˴�ȡ��Ϊ����Ӧ������*/
    }
    else if(mode == 4)                                   /*����ģʽ*/
    {
        parM1->SetVelMotor = 0;                          /*����ģʽ�½��ٶ�����Ϊ0*/
        parM2->SetVelMotor = 0;
        parM1->SetTorqueMotor = m2veltemp;               /*��16λ��M1*/
        parM2->SetTorqueMotor = m1veltemp;               /*��16λ��M2*/
    }
}
/*------------------------------------------------
Function:�����������Լ����������
Input   :No
Output  :No
Explain :Ϊ�˷�ֹ�󱨣�����޸�Ϊ2s�������
------------------------------------------------*/
uint32_t Hal_Enc_Err_Check(u8 axes)
{
    u8 temp = 0;
    uint32_t errorcode = 0;
    if(axes == M1)
    {
        temp = HALL_GetPhase1();
    }
    if(axes == M2)
    {
        temp = HALL_GetPhase2();
    }
    if(temp == 7||temp == 0)
    {
        hall_error_cnt[axes]++;                     /*500usһ��*/
        if(hall_error_cnt[axes] > 4000)             /*2sȷ��Ϊ��������*/
        {
            hall_error_cnt[axes] = 4000;
            errorcode = ERR_HALL;
        }
    }
    else
    {
        if(hall_error_cnt[axes] > 0)
        {
            hall_error_cnt[axes]--;
        }
    }
    return errorcode;
}
/*------------------------------------------------
Function:����¶ȱ���
Input   :No
Output  :No
Explain :500usһ�� Ĭ��120�� 30s����
------------------------------------------------*/
uint32_t Motor_Over_Temperature(u8 axes)
{
    uint32_t errorcode = 0;
    if(pCtrlPar[axes].MotorTemperature > MotorParameters[axes].MaxTemperature && STM[axes].bState == RUN)   /*120�ȱ���*/
    {
        temperature_cnt[axes] ++;
    }
    else
    {
        if(temperature_cnt[axes]>0)
        {
            temperature_cnt[axes]--;
        }
    }
    if(temperature_cnt[axes] > MotorParameters[axes].OverTemperatureTicks)                                  /*30s����*/
    {
        temperature_cnt[axes] = 0;
        errorcode = MC_OVER_TEMP;
    }
    return errorcode;
}
/*------------------------------------------------
Function:���ʹ�ܺ���
Input   :��ͣ����
Output  :No
Explain :�����ڼ�ͣ�źŰ����Ժ���ã�����ط������ã�ͬ��ģʽ���������ΪDrive_Data[M1]��ͬ�����������,ֻ����ͬ��ģʽ��
------------------------------------------------*/
uint8_t Motor_Enable(Drive_Data *pDriver_Data)
{
    static uint8_t EnableStep[2] = {0,0};                /*�������*/
    static uint16_t Statusword = 0;
    Statusword = pDriver_Data->device_control_data.Statusword;
    if((Statusword & 0x006F) == 0x0027)                 /*����Ѿ�ʹ��*/
    {
        EnableStep[0] = 0;                              /*ʹ���Ժ󽫲�������*/
        return 1;//����Ѿ�ʹ��״̬
    }
    else
    {
        if(EnableStep[0] ==0)
        {
            pCtrlPar[M1].SetVelMotor = 0;               /*����ٶȲο�*/
            pCtrlPar[M2].SetVelMotor = 0;
            pCtrlPar[M1].M1M2_VelSet = 0;
            EnableStep[0] = 1;
        }
        else if((Statusword & 0x006F) == 0x0021)        /*������6������ɣ�״̬���л�Ϊ0x0231*/
        {
            EnableStep[0] = 2;
        }
        else if((Statusword & 0x006F) == 0x0023)        /*������7������ɣ�״̬���л�Ϊ0x0233*/
        {
            EnableStep[0] = 3;
        }

        if(EnableStep[0] == 1)
        {
            pDriver_Data->device_control_data.Conrtolword = 0x0006;
            return 0;
        }
        else if(EnableStep[0] == 2)
        {
            pDriver_Data->device_control_data.Conrtolword = 0x0007;
            return 0;
        }
        else if(EnableStep[0] == 3)
        {
            pDriver_Data->device_control_data.Conrtolword = 0x000F;
            return 0;
        }
    }
    return 0;
}
/*------------------------------------------------
Function:��ͣ��⹦��
Input   :No
Output  :No
Explain :����Ϊ0������Ϊ1
------------------------------------------------*/
void Emrgency_Stop(void)
{
    EMG_State3 = (Start_Stop_GPIO_Port->IDR>>0)&(0x01);
    if(EMG_State3 != EMG_State4)                                                 /*��֤ÿ��״̬�ı�ֻ����һ�β�����һֱִ�л�Ӱ��CAN�������豸����*/
    {
        if(EMG_State3 == STOPISON)                                               /*��ͣ������*/
        {
            if(Motor_Enable(Driver_Data) == TRUE)
            {
                Driver_Data[M1].device_control_data.Quick_stop_option_code = 6;  /*����6085 ���õļ�ͣ���ٶ�ͣ����ͣ��������*/
                Driver_Data[M1].device_control_data.Conrtolword = 0x0002;        /*����ͣ������*/
//                P_V_I_Switch = 2;                                              /*���ڴ���λ�û���ͣ�źţ���˾��챲�����λ�û����Դ˴��ִ�����Ҫ���ڼ�¼��δ����λ�û���ͣ��*/
                EMG_State4 = EMG_State3;                                         /* EMG_State3�����STOPISON*/
            }
            else
            {
                /*ʹ��ʧ��*/

            }
        }
        else    /*��ͣû�б�����*/
        {

            Driver_Data[M1].device_control_data.Quick_stop_option_code = 1;      /*����6084 ���õļ��ٶ�ͣ���� ͣ��������*/
            Driver_Data[M1].device_control_data.Conrtolword = 0x0002;            /*����ͣ��*/
            EMG_State4 = EMG_State3;                                             /*EMG_State3����� STOPISOFF*/

        }
    }
    else
    {
        /*���һֱ���ڰ��»��߷ǶϿ��������в���*/
    }

}
/*------------------------------------------------
* @function :���2��ʼ�Ƕȶ�λ
* @input    :����2״̬
* @output   :�����ʼλ��
* @explain  :���ݻ����������г�ʼ�ֶ�λ����λΪÿ�������ĵ�Ƕ�Ϊ30�ȵ�λ��
* @author   :LM
* @date     :2022/12/16
------------------------------------------------*/

void HALL2_Init_Electrical_Angle( void )
{
    static uint8_t motor2_exit_clear_flag = 0;
    HallState2 = HALL_GetPhase2();
    switch ( HallState2 )
    {
    case STATE_5:
        HAL_Init_Electrical_Angle2 = (int16_t)(HALL_PHASE_SHIFT2) + S16_60_PHASE_SHIFT/2;
        break;
    case STATE_1:
        HAL_Init_Electrical_Angle2 = (int16_t)(HALL_PHASE_SHIFT2) + S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT/2;
        break;
    case STATE_3:
        HAL_Init_Electrical_Angle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT/2);
        break;
    case STATE_2:
        HAL_Init_Electrical_Angle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2)-S16_120_PHASE_SHIFT
                                     - S16_60_PHASE_SHIFT/2 );
        break;
    case STATE_6:
        HAL_Init_Electrical_Angle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) - S16_60_PHASE_SHIFT -  S16_60_PHASE_SHIFT/2 );
        break;
    case STATE_4:
        HAL_Init_Electrical_Angle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) - S16_60_PHASE_SHIFT/2 );
        break;
    default:
        HallState2 = HALL_GetPhase2();
        initial2_positioning_timing++;
        break;
    }
    if(initial2_positioning_timing > 5)                        /*5ms��ȡ5������������Ǵ����*/
    {
        initial2_positioning_timing = 6;                       /*�˴����Ǳ���ʲô����*/
    }
    if(HallState2 != 0 && HallState2 != 7)                     /*����*/
    {
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);              /*�������жϷ��ڴ˴�Ŀ����Ϊ�˷�ֹ�����ϵ��ƶ����³�ʼ��λʧ������*/
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        if(motor2_exit_clear_flag == 0)                        /*��̬��������ʼ��λ����Ժ�ֻ��λһ��*/
        {
            motor2_exit_clear_flag++;
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_U_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_V_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_W_Pin);
        }
    }
}
/*------------------------------------------------
* @function :���1��ʼ�Ƕȶ�λ
* @input    :����1״̬
* @output   :�����ʼλ��
* @explain  :���ݻ����������г�ʼ�ֶ�λ����λΪÿ�������ĵ�Ƕ�Ϊ30�ȵ�λ��
* @author   :����
* @date     :2022/12/16
------------------------------------------------*/
void HALL1_Init_Electrical_Angle( void )
{
    static uint8_t motor1_exit_clear_flag = 0;
    HallState1 = HALL_GetPhase1();
    switch ( HallState1 )
    {
    case STATE_5:
        HAL_Init_Electrical_Angle = (int16_t)(HALL_PHASE_SHIFT) + S16_60_PHASE_SHIFT/2;
        break;
    case STATE_1:
        HAL_Init_Electrical_Angle = (int16_t)(HALL_PHASE_SHIFT) + S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT/2;
        break;
    case STATE_3:
        HAL_Init_Electrical_Angle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT/2);
        break;
    case STATE_2:
        HAL_Init_Electrical_Angle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT)-S16_120_PHASE_SHIFT
                                    - S16_60_PHASE_SHIFT/2 );
        break;
    case STATE_6:
        HAL_Init_Electrical_Angle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) - S16_60_PHASE_SHIFT -  S16_60_PHASE_SHIFT/2 );
        break;
    case STATE_4:
        HAL_Init_Electrical_Angle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) - S16_60_PHASE_SHIFT/2 );
        break;
    default:
        HallState1 = HALL_GetPhase1();
        initial_positioning_timing++;
        break;
    }
    if(initial_positioning_timing > 5)                           /*5ms��ȡ5������������Ǵ����*/
    {
        initial_positioning_timing = 6;                          /*�˴����Ǳ���ʲô����*/
    }
    if(HallState1 != 0 && HallState1 != 7)                       /*����*/
    {
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);              /*�������жϷ��ڴ˴�Ŀ����Ϊ�˷�ֹ�����ϵ��ƶ����³�ʼ��λʧ������*/
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        if(motor1_exit_clear_flag == 0)                          /*��̬��������ʼ��λ����Ժ�ֻ��λһ��*/
        {
            motor1_exit_clear_flag++;
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_U_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_V_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_W_Pin);
        }
    }
}
/*------------------------------------------------
* @function :��ȡ���2�����ź�
* @input    :�ⲿ��ƽ
* @output   :123456
* @explain  :����д
* @author   :����д
* @date     :2022/12/16
------------------------------------------------*/
u8 HALL_GetPhase2(void)
{
    int32_t tmp = 0;
    tmp |= HAL_GPIO_ReadPin(M2_HALL_V_GPIO_Port, M2_HALL_V_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(M2_HALL_U_GPIO_Port, M2_HALL_U_Pin);//W(C)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(M2_HALL_W_GPIO_Port, M2_HALL_W_Pin);//W(C)
    return (u8)(tmp & 0x0007); //ȡ����λ
}
/*------------------------------------------------
* @function :��ȡ���1�����ź�
* @input    :�ⲿ��ƽ
* @output   :123456
* @explain  :UVWΪʲô������˳����д�����������UVW�������⣬
             �������������ͨ�Ժ�һֱ��������
* @author   :����
* @date     :2022/12/16
------------------------------------------------*/
u8 HALL_GetPhase1(void)
{
    int32_t tmp = 0;

    tmp |= HAL_GPIO_ReadPin(M1_HALL_V_GPIO_Port, M1_HALL_V_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(M1_HALL_U_GPIO_Port, M1_HALL_U_Pin);//W(C)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(M1_HALL_W_GPIO_Port, M1_HALL_W_Pin);//W(C)

    return (u8)(tmp & 0x0007); //ȡ����λ
}
/*------------------------------------------------
* @function :���Brake�˲�����
* @input    :����д
* @output   :����д
* @explain  :100ms����һ��
* @author   :����
* @date     :2022/12/16
------------------------------------------------*/
void Clear_Brake_Filters_Cnt(void)
{
    BrakeFiltersCnt[0] = 0;
    BrakeFiltersCnt[1] = 0;
}
/*------------------------------------------------
* @function :����ģʽ����
* @input    :
* @output   :
* @explain  :����֮�����յ�����ͨ��֮�������ģʽ
             damping_mode_flag���յ�CANָ���Ժ���Ϊ1����ֹδ�ϵ��ƶ����������ѹ������ģʽ
             ȥ��Ƿѹ�����´�����ģʽ
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void Damping_Of_Motor(void)
{
    /*��������Զ�������ģʽ*/
    if((wGlobal_Flags != 0)&&(damping_mode_flag == 1)&&((wGlobal_Flags&MC_UNDER_VOLT) == 0))
    {
        Damping_Mode_ON_OFF = DAMPING_ON;
    }
    /*�����������Զ��ر�����ģʽ*/
    if(wGlobal_Flags == 0)                             /*�������������˳�����ģʽ[027�汾�޸�]*/
    {
        Damping_Mode_ON_OFF = DAMPING_OFF;             /*�ر�����ģʽ*/
    }
    if(Damping_Mode_ON_OFF == DAMPING_ON&&STM[M1].bState != RUN && STM[M2].bState != RUN)
    {
        /*�������¹�*/
        TIM1->CR2 = TURN_ON_THREE_LOW;
        TIM8->CR2 = TURN_ON_THREE_LOW;
        Damping_Mode_ON_OFF = STATE_DAMPING_IS_ON;     /*�����¹ܵ�״̬Ϊ��*/
    }
    else if (Damping_Mode_ON_OFF == DAMPING_OFF)       /*�ر�����ģʽ*/
    {
        /*�ر������¹�*/
        TIM1->CR2  = TURN_OFF_THREE_LOW;
        TIM8->CR2  = TURN_OFF_THREE_LOW;
        Damping_Mode_ON_OFF = STATE_DAMPING_IS_OFF;    /*�����¹ܵ�״̬Ϊ��ر�*/
    }

}
/*------------------------------------------------
* @function :�ϵ�����������
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void Auto_Start_function(void)
{
    if(led_1ms_cnt<1000)
    {
        Driver_Data[M1].device_control_data.Conrtolword = 0x06;
    }
    else if(led_1ms_cnt<1100)
    {
        Driver_Data[M1].device_control_data.Conrtolword = 0x07;
    }
    else if(led_1ms_cnt<1200)
    {
        Driver_Data[M1].device_control_data.Conrtolword = 0x0F;
    }
    else if(led_1ms_cnt<1300)
    {
        pCtrlPar[M1].M1M2_VelSet = 0x000005DC;
    }
}
/*------------------------------------------------
* @function :M1M2�����ʼλ��ѧϰ
* @input    :
* @output   :
* @explain  :����Ϊ�˵õ� HALL_PHASE_SHIFT HallStudyFlag1����д1��֮ǰӦ��ʹ�ܵ��������д��1�Ժ������ӹ̶�λ��������תһ���Զ�ʧ�ܡ�
             2��������϶�����Ƕ�Ϊ0��λ��
* @author   :wangchuan
* @date     :2022/12/30
------------------------------------------------*/
void M1M2_Initial_Position(void)
{
    if(HallStudyFlag1 == 1)                                                   /*M1����ѧϰѰ�ҳ�ʼλ��ƫ��*/
    {
        CNT_1MS[M1] = 0;
        if(HallStudyCNT < Hall_EAC_Ticks)                                     /*��ʱ3��*/
        {
            Angle_Switch = 0;                                                 /*FOC�н�����϶���һ����Ƕ�Ϊ0��λ��*/
        }
        else if(HallStudyCNT < 3*Hall_EAC_Ticks)                              /*��ʱ6�룬�ϼƣ�9�룬�Ѿ�����3��*/
        {
            if(Hall_AvElAngle1CNT >=MotorParameters[M1].PolePairNum)          /*ת��1RPM*/
            {
                if(Hall_AvElAngle1CNT == MotorParameters[M1].PolePairNum)     /*ת��1RPM*/
                {
                    Hall_AvElAngle1 = (Hall_AvElAngle1Sum/Hall_AvElAngle1CNT);/*ƽ��ÿ�����ڵĵ�Ƕ�*/
                    Hall_AvElAngle1Temp = (u16)Hall_AvElAngle1;
                    HALL_M1.PhaseShift = Hall_AvElAngle1;
                    HALL_PHASE_SHIFT = (s16)((u32)Hall_AvElAngle1Temp);       /*�õ���ʼλ��*/
                }
                Hall_AvElAngle1CNT = MotorParameters[M1].PolePairNum+1;
                Hall_AvElAngle1Sum =0;
//                    HALL_CC_First = 2;
                pCtrlPar[M1].SetVelMotor = 0;
                Angle_Switch = 0;                                              /*ѧϰ����л�״̬*/
                MC_StopMotor1();

            }
            else
            {
                Angle_Switch = 2;                                              /*FOC��Angle_Switch=2Ϊ�϶������ת*/
            }
        }
        else
        {
            HallStudyFlag1 = 0;                                                /*��ѧϰ���õ��Ĳ������*/
            HallStudyCNT = 0;
            Hall_AvElAngle1CNT =0;
            Hall_AvElAngle1Sum =0;
            CurrentPosition1 =  MC_GetCurrentPosition1();
            pPIDSpeed[M1]->wIntegralTerm = 0x00000000UL;
            pPIDIq[M1]->wIntegralTerm = 0x00000000UL;
            pPIDPosCtrl[M1]->wIntegralTerm = 0x00000000UL;
            pPosCtrl[M1]->Theta = CurrentPosition1 ;
            Angle_Switch = 3;
            ENCODER_M1.Angle_Compensation = 0 ;                                /**/
            HALL_CC_First = 0;

        }

    }
    else if(HallStudyFlag1 == 2)                                               /*������϶�����Ƕ�0λ*/
    {
        MC_StartMotor1();                                                      /*ʹ�ܵ��*/
        CNT_1MS[M1] = 0;
        if(HallStudyCNT < Hall_EAC_Ticks)                                      /*��ʱ3��*/
        {
            Angle_Switch = 0;                                                  /*FOC�и���Vq*/
        }
        else
        {
            MC_StopMotor1();                                                   /*ʧ�ܵ��*/
            HallStudyFlag1 = 0;                                                /*�϶����*/
            HallStudyCNT = 0;
            Angle_Switch = 3;
            ENCODER_M1._Super.wPulseNumber	= 0;                               /*���ʵ��λ����0*/
            CurrentPosition1 =  MC_GetCurrentPosition1();
            ENCODER_M1.Angle_Compensation = 0 ;                                /*�ǶȲ���ֵ��0*/
            HALL_CC_First = 2;

        }
        HALL_CH1_ValueOffset = HALL_CH1_ValueDelta1;
    }
    else
    {
        HallStudyCNT = 0;
    }


    if(HallStudyFlag2 == 1)                                                     /*M1����ѧϰѰ�ҳ�ʼλ��ƫ��*/
    {
        CNT_1MS[M2] = 0;
        if(HallStudyCNT2 < Hall_EAC_Ticks)
        {
            Angle_Switch2 = 0;
        }
        else if(HallStudyCNT2 < 3*Hall_EAC_Ticks)
        {
            if(Hall_AvElAngle2CNT >=MotorParameters[M2].PolePairNum)
            {
                if(Hall_AvElAngle2CNT == MotorParameters[M2].PolePairNum)
                {
                    Hall_AvElAngle2 = (Hall_AvElAngle2Sum/Hall_AvElAngle2CNT);
                    Hall_AvElAngle2Temp = (u16)Hall_AvElAngle2;
                    HALL_PHASE_SHIFT2 = (s16)((u32)Hall_AvElAngle2Temp);
                }
                Hall_AvElAngle2CNT = MotorParameters[M2].PolePairNum+1;
                Hall_AvElAngle2Sum =0;
//                    HALL2_CC_First = 0;
                pCtrlPar[M2].SetVelMotor = 0;
                Angle_Switch2 = 0;
                MC_StopMotor2();

            }
            else
            {
                Angle_Switch2 = 2;
            }
        }
        else
        {
            HallStudyFlag2 = 0;
            HallStudyCNT2 = 0;
            Hall_AvElAngle2CNT =0;
            Hall_AvElAngle2Sum =0;
            CurrentPosition2 =  MC_GetCurrentPosition2();
            pPIDSpeed[M2]->wIntegralTerm = 0x00000000UL;
            pPIDIq[M2]->wIntegralTerm = 0x00000000UL;
            pPIDPosCtrl[M2]->wIntegralTerm = 0x00000000UL;
            pPosCtrl[M2]->Theta =CurrentPosition2 ;
            Angle_Switch2 = 3;
            ENCODER_M2.Angle_Compensation = 0 ;
            HALL2_CC_First = 0;
        }

    }
    else if(HallStudyFlag2 == 2)
    {
        MC_StartMotor2();
        CNT_1MS[M2] = 0;
        if(HallStudyCNT2 < Hall_EAC_Ticks)
        {
            Angle_Switch2 = 0;
        }
        else
        {
            MC_StopMotor2();
            HallStudyFlag2 = 0;
            HallStudyCNT2 = 0;
            Angle_Switch2 = 3;
            ENCODER_M2._Super.wPulseNumber	= 0;
            CurrentPosition2 =  MC_GetCurrentPosition2();
            ENCODER_M2.Angle_Compensation = 0 ;
            HALL2_CC_First = 2;
        }
//            HALL_CH2_ValueOffset = HALL_CH2_ValueDelta1;

    }
    else
    {
        HallStudyCNT2 = 0;
    }
}

