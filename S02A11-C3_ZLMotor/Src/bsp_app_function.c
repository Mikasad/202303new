/*
*******************************************************************************
 *版    权： 2021-xxxx,  GaussianRobot
 *文 件 名： bsp_app_function.c 函数
 *简    介： 电机控制所有功能性函数都应该写在该文件夹下
 *作    者： LMmotor\忘川
 *日    期： 2022.1.4
 *功能描述：
*******************************************************************************
 *备注：
*******************************************************************************
*/
#include "bsp_app_function.h"
#include "ds402.h"
#include "mc_config.h "
#include "stdlib.h"
#include "bsp_CAN.h"


/*LEDSet DisplayErrorLed_Handle*/
uint32_t DisplayCounter = 0;                                     /*指示灯闪烁频率*/

/*Independent_Motor_Control*/
uint8_t PositionModeFlag1=0,PositionModeFlag2=0;                 /*0：需要进行位置环锁轴，在锁轴完成以后自动置为1，打开位置环控制*/
uint8_t relative_location_flag[MAX_AXES] = {0};                  /*相对位置控制1：执行相对位置 执行一次完以后自动置0*/
uint8_t absolute_location_flag[MAX_AXES] = {0};                  /*绝对位置控制参数*/
uint8_t P_V_I_Switch_using[MAX_AXES];                            /*三种模式切换标志位*/
uint8_t modetemp[MAX_AXES]= {3,3};                               /*上电处于速度模式，该值初始化为3*/
uint8_t  LockAxleON_OFF = 0 ;                                    /*锁轴开关*/
uint8_t initial2_positioning_timing = 0;                         /*初始定位错误检查*/
uint8_t initial_positioning_timing = 0;                          /*初始定位错误检查*/
uint16_t Sync_Async_Control = 1;                                 /*同步模式异步模式选择参数*/
int32_t Countrer_1ms_CNT=200,Countrer2_1ms_CNT=200;              /*用于电机12的位置环锁轴计时*/
long RelativeLocation[MAX_AXES] = {0};                           /*相对位置参考*/
float CurrentPosition1,CurrentPosition2;                         /*用于电机12的位置环锁轴，以及在mc_it.c当中获取到数据以后给到pPosCtrl[M1]->Theta与位置规划有关*/


/*DefaultPhaseCheck*/
uint32_t  giStuckTimeCnt[NUMBER_OF_AXES] = {0,0};                /*堵转时间计数*/

/*ClearPhaseCheckPar  DefaultPhaseCheck*/
long IabcCurrentTemp[2][3] = {0};                                /*两个电机的二维数组，abc三相电流缓存*/
float cur_calculated_value[2] = {0};                             /*300ms电流累计排序以后，计算：(Imax-Imin)/Imax 范围[0,1]*/
float iabcurrenttemporary[3] = {0};                              /*单个电机三相电流，输入给排序函数*/
uint16_t  lostPhaseCheckCnt[2] = {0};                            /*电机运行时缺相检查计数*/
uint16_t  lostPhaseCheckCntForLaunch[2] = {0};                   /*电机启动时缺相检查计数*/
uint32_t Missing_phase_error_code[2] = {MC_NO_ERROR};            /*缺相错误代码*/
uint32_t MISSING_PAHSE_JUDGE_TIME = 10000;                       /*持续2000ms错误报错，该值太低重载启动起不来*/

/*void Emrgency_Stop(void)*/
uint16_t EMG_State3,EMG_State4 = 2;                              /*急停按钮STOPISOFF(1) or STOPISON(0)*/
uint16_t  giStallTimeCnt[NUMBER_OF_AXES] = {0};                  /*失速检查累计时间计数*/
uint32_t hall_error_cnt[2] = {0,0};                              /*霍尔错误检查累计时间计数*/
uint32_t temperature_cnt[2] = {0};                               /*电机温度保护时间累计计数*/

Trig_Components Vector_Components;                               /*用于生成SIN曲线、方波用于测试电流环和速度环响应*/
LOOP_TEST_T	 loopTestPar;								                         /*用于生成SIN曲线、方波用于测试电流环和速度环响应*/
u8    Damping_Mode_ON_OFF = 0;                                   /*阻尼模式开关*/
SEVERO_CONTORL_PAR_T pCtrlPar_M1=                                /*给SEVERO_CONTORL_PAR_T类型的pCtrlPar_M1赋初值用于用于接口，最终 pCtrlPar[M1]  = *(&pCtrlPar_M1); 该地址指针指向*/
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

SEVERO_CONTORL_PAR_T pCtrlPar[NUMBER_OF_AXES];                         /*定义两个SEVERO_CONTORL_PAR_T类型的结构体�*/

/*霍尔学习相关参数*/
int16_t Hall_AvElAngle1 = 0;                                           /*M1一圈的平均电角度 = Hall_AvElAngle1Sum /Hall_AvElAngle1CNT */
int16_t Hall_AvElAngle1CNT = 0;                                        /*M1极对数计数*/
int16_t Hall_AvElAngle2 = 0;                                           /*M2一圈的平均电角度*/
int16_t Hall_AvElAngle2CNT = 0;                                        /*M2极对数计数*/
int16_t Hall_AvElAngle2Temp,Hall_AvElAngle1Temp;
int16_t Angle_Switch2=3,Angle_Switch = 3;
uint16_t HallStudyCNT=0,HallStudyCNT2=0,Hall_EAC_Ticks = 3000;         /*霍尔学习，计时*/
int32_t Hall_AvElAngle1Sum = 0;                                        /*M1一圈电角度总和*/
int32_t Hall_AvElAngle2Sum = 0;                                        /*M2一圈电角度总和*/
uint8_t HallStudyFlag1=0,HallStudyFlag2=0;                             /*霍尔学习标志，只有调试时使用*/
uint8_t HALL_CC_First=0,HALL2_CC_First=0,HALL_CC_First11=0;            /*电机首次启动判断条件*/
/*------------------------------------------------
Function:速度模式速度模式力矩模式切换&三环指令执行入口
Input   :No
Output  :No
Explain :Modes_of_operation
					1 位置模式  	默认速度模式
					3 速度模式
					4 力矩模式
         周期：500us
------------------------------------------------*/
void Independent_Motor_Control(void)
{
    static long delay_200ms[2]= {200,200};
    Drive_Data *pCiA402Axis;
    u16 counter = 0;
    static long location_cache[MAX_AXES] = {0};
    for(counter = 0; counter < MAX_AXES; counter++)
    {
        if(!Driver_Data[counter].device_control_data.bAxisIsActive)       /*注意赋初值1*/
        {
            continue;
        }
        if(Driver_Data[M1].device_control_data.Modes_of_operation == 3\
                || Driver_Data[M1].device_control_data.Modes_of_operation == 4) /*如果是M1速度模式或者力矩模式，那么默认将两个电机的操作模式设置为一致，因为上位机只控制一个电机的控制字*/
        {
            Driver_Data[M2].device_control_data.Modes_of_operation = Driver_Data[M1].device_control_data.Modes_of_operation; /*同步模式下：控制M1电机的控制模式即可！*/
        }
        if(Sync_Async_Control == 1)                                       /*0：M1和M2异步控制  1：M1和M2同步控制（兼容中菱）*/
        {
            Driver_Data[M2].device_control_data.Modes_of_operation = Driver_Data[M1].device_control_data.Modes_of_operation; /*同步模式下：控制M1电机的控制模式即可！*/
        }

        pCiA402Axis = &Driver_Data[counter];                              /*指针赋值*/

        if(pCiA402Axis->device_control_data.Modes_of_operation == 1)      /*位置模式*/
        {
            if(modetemp[counter] != 1)                                    /*switch的选择条件不为位置模式*/
            {
                modetemp[counter] = 2;                                    /*case2 为 切换到位置模式的中间过程，达成case2当中条件，会切换到位置模式*/
                P_V_I_Switch_using[counter] = 3;
            }
        }
        else if(pCiA402Axis->device_control_data.Modes_of_operation == 3) /*速度模式*/
        {
            if(modetemp[counter] != 3)                                    /*switch的选择条件不为速度模式*/
            {
                pCtrlPar[M1].M1M2_VelSet = 0;                             /*清除同步模式下的速度参考，速度模式下该参数代表下发速度，力矩模式下该参数代表下发力矩，*///
                pCtrlPar[M1].SetVelMotor = 0;                             /*清除M1电机异步模式下速度参考*/
                pCtrlPar[M2].SetVelMotor = 0;
                pCtrlPar[M1].SetTorqueMotor = 0;                          /*清除M1电机异步模式下力矩参考*/
                pCtrlPar[M2].SetTorqueMotor = 0;
                pPIDSpeed[M1]->wIntegralTerm = 0;                         /*速度环积分项清除*/
                pPIDSpeed[M2]->wIntegralTerm = 0;
                modetemp[counter] = 5;                                    /*case5为切换到速度模式的中间条件，达成case5当中条件，会切换到case 3速度模式*/
            }
        }
        else if(pCiA402Axis->device_control_data.Modes_of_operation == 4) /*力矩模式*/
        {
            if(modetemp[counter]!=4)
            {
                /*首次进入力矩模式需要清除力矩参考*/
                pCtrlPar[M1].M1M2_VelSet = 0;                             /*清除同步模式下力矩参考，速度模式下该参数代表下发速度，力矩模式下该参数代表下发力矩*/
                pCtrlPar[M1].SetTorqueMotor = 0;
                pCtrlPar[M2].SetTorqueMotor = 0;
                pCtrlPar[M1].SetVelMotor = 0;                             /*清除M1电机异步模式下速度参考*/
                pCtrlPar[M2].SetVelMotor = 0;
                pPIDSpeed[M1]->wIntegralTerm = 0;                         /*速度环积分项清除*/
                pPIDSpeed[M2]->wIntegralTerm = 0;
                modetemp[counter] = 4;                                    /*由其他环切换到力矩模式直接切换就OK*/
            }
        }
        /*end*/
        if(Sync_Async_Control == 1)                                       /*0：M1和M2异步控制  1：M1和M2同步控制（兼容中菱）*/
        {
            SpeedSetMerge_two_to_one(&pCtrlPar[M1],&pCtrlPar[M2],modetemp[counter]);
        }


        switch(modetemp[counter])
        {
        case 0:
            break;
        case 1 :                                                          /*【位置模式】*/

            if(counter == M1)                                             /*电机1位置环*/
            {
                if(relative_location_flag[counter] == 1)                  /*相对位置模式开关*/
                {
                    relative_location_flag[counter] = 0;
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;    /*打开位置控制模式开关*/
                    location_cache[counter] = ENCODER_M1._Super.wPulseNumber+pCtrlPar[M1].SetPulseMotor; /*每次给定位置在当前的位置上加上给定位置*/
                    MC_ProgramPositionCommandMotor1(location_cache[counter] , 0 );
                    location_cache[counter] = 0;                          /*清0,下次使用*/
                }
                else if(absolute_location_flag[counter] == 1)             /*绝对位置模式开关*/
                {
                    absolute_location_flag[counter] = 0;
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;    /*打开位置控制模式开关*/
                    MC_ProgramPositionCommandMotor1( pCtrlPar[M1].SetPulseMotor, 0 );
                }
            }
            else if(counter == M2)                                        /*电机2位置环*/
            {
                if(relative_location_flag[counter] == 1)                  /*相对位置模式开关*/
                {
                    relative_location_flag[counter] = 0;                  /*执行完成以后自动置0，操作就需要每次下发相对位置以后，在写入该位为1方可执行相对位置指令，该参数目前没有加入对象字典，所以不可用*/
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;    /*打开位置控制模式开关*/
                    location_cache[counter] = ENCODER_M2._Super.wPulseNumber+pCtrlPar[M2].SetPulseMotor; /*每次给定位置在当前的位置上加上给定位置*/
                    MC_ProgramPositionCommandMotor2( location_cache[counter], 0 );
                    location_cache[counter] = 0;
                }
                else if(absolute_location_flag[counter] == 1)             /*绝对位置模式开关*/
                {
                    absolute_location_flag[counter] = 0;
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;    /*打开位置控制模式开关*/
                    MC_ProgramPositionCommandMotor2( pCtrlPar[M2].SetPulseMotor, 0 );
                }
            }
            break;
        case 2 :                                                          /*速度模式和位置模式切换*/
            if(counter == M1)
            {
                pCtrlPar[M1].SetVelMotor = 0 ;
                pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
                MC_ProgramSpeedRampMotor1(pCtrlPar[M1].SetVelMotor, 100 );
                delay_200ms[counter]--;
                if(delay_200ms[counter] < 0)
                {
                    delay_200ms[counter] = 200;
                    if(P_V_I_Switch_using[counter] == 2)                 /*电机M1  位置环----->速度环*/
                    {
                        if(pCtrlPar[M1].Vel_PLL_Motor==0)
                        {
                            modetemp[counter] = 3;                       /*切换到速度环*/
                            CNT_1MS[counter] = 0;
                            PositionModeFlag1 = 0;                       /*关闭位置环控制*/
                        }
                    }
                    else  if(P_V_I_Switch_using[counter] == 3)           /*电机M1 速度环----->位置环*/
                    {
                        if(pCtrlPar[M1].Vel_PLL_Motor ==0)
                        {
                            modetemp[counter] = 1;                       /*切换到位置环*/
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
                            modetemp[counter] = 3;                       /*切换到速度环*/
                            CNT_1MS[counter] = 0;
                            PositionModeFlag2 = 0;                       /*关闭位置环控制*/
                        }
                    }
                    else  if(P_V_I_Switch_using[counter] == 3)
                    {
                        if(pCtrlPar[M2].Vel_PLL_Motor ==0)
                        {
                            modetemp[counter] = 1;                       /*切换到位置环*/
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
        case 3 :  /*【速度模式】*/

            if(counter == M1&&STM[M1].bState > 5)
            {
                Countrer_1ms_CNT = pCtrlPar[M1].hDurationms+1000;
                if(LockAxleON_OFF == 0)                                                                               /*速度环锁轴*/
                {
                    if(PendingOpcode_handleWaitStatus[counter] == TRUE)                                               /*如果执行快速停机命令*/
                    {
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 1\
                                ||Driver_Data[counter].device_control_data.Quick_stop_option_code == 5)                     /*按照6084设置的减速度时间停机*/
                        {
                            MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms);
                        }
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 2\
                                ||Driver_Data[counter].device_control_data.Quick_stop_option_code == 6)                      /*按照6085设置的减速度时间停机*/
                        {
                            MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[M1].quick_stop_decel);  /*这里的pCtrlPar[M1].quick_stop_decel不要修改M1M2共用*/
                        }
                    }
                    else if(EMG_State3 ==STOPISOFF)                                                                    /*正常运行，急停开关未被按下*/
                    {
                        MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms );
                    }

                    CurrentPosition1 =  MC_GetCurrentPosition1();
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
                    CNT_1MS[counter] = 0;
                    PositionModeFlag1 = 0;
                }
                else if(LockAxleON_OFF == 1)  /*M1位置环锁轴未使用，设计初期为了考虑速度环在斜坡锁轴可能导致溜坡*/
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
                if(LockAxleON_OFF == 0)                                /*速度环锁轴*/
                {
                    if(PendingOpcode_handleWaitStatus[counter] == TRUE)/*如果执行快速停机命令*/
                    {
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 1||Driver_Data[counter].device_control_data.Quick_stop_option_code == 5)/*按照6084设置的减速度时间停机*/
                        {
                            MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms);
                        }
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 2||Driver_Data[counter].device_control_data.Quick_stop_option_code == 6)/*按照6085设置的减速度时间停机*/
                        {
                            MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[M1].quick_stop_decel);    /*这里的pCtrlPar[M1].quick_stop_decelM1M2共用   不要修改*/
                        }
                    }
                    else if(EMG_State3 ==STOPISOFF)                    /*正常运行，急停开关未被按下*/
                    {
                        MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms );
                    }
                    CurrentPosition2 =  MC_GetCurrentPosition2();
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 0;
                    CNT_1MS[counter] = 0;
                    PositionModeFlag2 = 0;
                }
                else if(LockAxleON_OFF == 1)                          /*M2位置环锁轴未使用，设计初期为了考虑速度环在斜坡锁轴可能导致溜坡*/
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
        case 4 :                                             /*当前处于力矩模式*/
            if(counter==M1)
            {
                MC_ProgramTorqueRampMotor1(pCtrlPar[M1].SetTorqueMotor, 0);    /*力矩下发*/
            }
            if(counter==M2)
            {
                MC_ProgramTorqueRampMotor2(pCtrlPar[M2].SetTorqueMotor, 0);
            }

            break;
        case 5 :                                             /*力矩模式切换到速度模式，如果有速度就给反向力矩使实际速度无限趋近于0*/

            if(counter==M1)
            {
                int32_t k1 = 30,vel1 = 0,Iqtemp1 = 0;
                vel1 = pCtrlPar[M1].Vel_PLL_Motor;
                Iqtemp1 = -1*k1*vel1;                        /*k1是整车调试出来的参数*/
                if( 11900 < Iqtemp1)
                {
                    Iqtemp1 = 11900;
                }
                else if(Iqtemp1 < -11900)
                {
                    Iqtemp1 = -11900;
                }
                MC_ProgramTorqueRampMotor1(Iqtemp1, 100);   /*给定反向力矩*/
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
            delay_200ms[counter]--;                            /*这样切换可能在外力条件下导致速度不为0，就一直不能切换到速度模式，导致溜车*/
            if(delay_200ms[counter] < 0)
            {
                delay_200ms[counter] = 100;
//                if(abs(pCtrlPar[counter].Vel_PLL_Motor) < 30)/*该速度是整车调试出来的不会发生抖动的情况、屏蔽此处时为了拍下急停机器立即停止*/
                {
                    pSTC[M1]->SpeedRefUnitExt   = -65460;      /*清除该速度参考否则在从力矩环切换到速度环电机会震动*/
                    pSTC[M2]->SpeedRefUnitExt   = -65460;
                    modetemp[counter] = 3;                     /*切换到速度环*/
                }
            }
            break;

        default:
            break;
        }
    }
}
/*------------------------------------------------
Function:led灯设置
Input   :led1: 1/下面红灯亮  0/下面红灯不亮
				 led2: 1/上面红灯亮  0/上面红灯不亮
				 led_time:闪烁的次数
				 led_mode:1/慢闪一次 0/快闪
				 led_speed:闪烁的速度
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
    else if(led_mode)                            	  /*慢闪*/
    {
        if(DisplayCounter<2*led_speed)              /*10ms自增一次*/
        {
            RED_LED1_ON;
        }
        else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
        {
            RED_LED1_TOGGLE;
        }
    }
    else if(DisplayCounter<=(2*led_time)*led_speed)	/*快闪*/
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
Function:设置错误标志
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u32 wGlobal_Flags = 0;
u32 display_led = 0;
void MC_SetFault(u32 hFault_type)
{
    wGlobal_Flags |= hFault_type;         /*全局错误指示*/
    if(display_led !=0)                   /*如果当前已经有一个错误在发生*/
    {
        display_led |=hFault_type;        /*两个错误*/
        display_led &=hFault_type;        /*清除上一个错误，显示当前发生的错误*/
    }
    else
        display_led  |= hFault_type;      /*只报警当前错误类型*/
}
/*------------------------------------------------
Function:清除错误标志
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_ClearFault(u32 hFault_type)
{
    wGlobal_Flags &= ~hFault_type;        /*清除全局错误*/
    display_led &= ~hFault_type;          /*清除当前错误类型*/

}
/*------------------------------------------------
Function:分别检测两个电机的错误位，进行整合在wGlobal_Flags变量当中
Input   :No
Output  :No
Explain :整合之后读取一个变量就可以查看所有的错误，也方便指示灯的显示
------------------------------------------------*/
void Display_error(void)
{
    /*M1M2公共错误*/
    if((STM[M1].hFaultOccurred&MC_OVER_VOLT)==MC_OVER_VOLT||(STM[M2].hFaultOccurred&MC_OVER_VOLT)==MC_OVER_VOLT)    /*如果两个电机的其中一个独立错误被置位*/
    {
        MC_SetFault(MC_OVER_VOLT);                                                                                  /*那么就置位全局变量中的错误指示*/
    }
    else
    {
        MC_ClearFault(MC_OVER_VOLT);
    }
    if((STM[M1].hFaultOccurred&MC_UNDER_VOLT)==MC_UNDER_VOLT||(STM[M2].hFaultOccurred&MC_UNDER_VOLT)==MC_UNDER_VOLT)/*注释同上*/
    {
        MC_SetFault(MC_UNDER_VOLT);
    }
    else
    {
        MC_ClearFault(MC_UNDER_VOLT);
    }
    if((STM[M1].hFaultOccurred&ERR_CAN_COMMUNICATION)==ERR_CAN_COMMUNICATION||(STM[M2].hFaultOccurred&ERR_CAN_COMMUNICATION)==ERR_CAN_COMMUNICATION)
    {
        MC_SetFault(ERR_CAN_COMMUNICATION);                                                                         /*那么就置位全局变量中的错误指示*/
    }
    else
    {
        MC_ClearFault(ERR_CAN_COMMUNICATION);
    }
    /*两个电机各有的错误使用for循环来判断，原理一样只是错误数目多*/
    for(u8 i =0; i<2; i++)
    {
        if((STM[i].hFaultOccurred&MC_BREAK_IN)==MC_BREAK_IN)    /*M1短路错误被置位*/
        {
            if(i==0)
            {
                if((wGlobal_Flags&MC_BREAK_IN_M1)==0)          /*如果全局错误中没有该错误*/
                {
                    MC_SetFault(MC_BREAK_IN_M1);               /*那么置位全局错误标志*/
                }
            }
            else
            {
                if((wGlobal_Flags&MC_BREAK_IN_M2)==0)          /*M2电机*/
                {
                    MC_SetFault(MC_BREAK_IN_M2);
                }
            }
        }
        else                                                   /*如果电机独立错误被清除，则清除全局错误*/
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
                ||(STM[i].hFaultOccurred&ERR_CURR_OVER_MAX)    ==ERR_CURR_OVER_MAX)    /*所有阶段的过流都报告同一个过流错误*/

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
Function:驱动器错误指示
Input   :No
Output  :No
Explain :10ms运行一次，执行与嘀嗒定时器当中,
------------------------------------------------*/
void DisplayErrorLed_Handle(void)
{
    static u8 DisplayGreenCnt;
    DisplayCounter++;                                     /*用于红灯闪烁频率*/
    DisplayGreenCnt++;                                    /*用于绿灯一直闪烁*/

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
        case MC_OVER_VOLT:                                 /*两个红灯常量*/
            RED_LED1_ON;
            RED_LED2_ON;
            break;
        case MC_UNDER_VOLT:                                /*欠压下面红灯亮*/
            RED_LED2_OFF;
            RED_LED1_ON;
            break;
        case ERR_CAN_COMMUNICATION:                        /*两个灯快闪1次*/
            LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
            break;

        case ERR_CURR_OVER_MAX_M1:                         /*上面红灯快闪2次*/
            LEDSet(LED_OFF,LED_ON,2,0,LED_HIGH_SPEED);
            break;
        case ERR_CURR_OVER_MAX_M2:                         /*下面红灯快闪2次*/
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case ERR_OVER_VEL_ERR_M1:                          /*上面红灯快闪3次*/
            LEDSet(LED_OFF,LED_ON,3,0,LED_HIGH_SPEED);
            break;
        case ERR_OVER_VEL_ERR_M2:                          /*下面红灯快闪3次*/
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case ERR_MOTOR_STUCK_ERR_M1:                       /*上面红灯快闪4次*/
            LEDSet(LED_OFF,LED_ON,4,0,LED_HIGH_SPEED);
            break;
        case ERR_MOTOR_STUCK_ERR_M2:                       /*下面红灯快闪4次*/
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;

        case MC_BREAK_IN_M1:                               /*上面红灯快闪5次*/
            LEDSet(LED_OFF,LED_ON,5,0,LED_HIGH_SPEED);
            break;
        case MC_BREAK_IN_M2:                               /*下面红灯快闪5次*/
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;
        case ERR_LOST_PHASE_M1:                            /*上面红灯快闪6次*/
            LEDSet(LED_OFF,LED_ON,6,0,LED_HIGH_SPEED);
            break;
        case ERR_LOST_PHASE_M2:                            /*下面红灯快闪6次*/
            LEDSet(LED_ON,LED_OFF,6,0,LED_HIGH_SPEED);
            break;

        case ERR_HALL_M1:                                  /*上面红灯快闪7次*/
            LEDSet(LED_OFF,LED_ON,7,0,LED_HIGH_SPEED);
            break;
        case ERR_HALL_M2:                                  /*下面红灯快闪7次*/
            LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
            break;
        case MC_OVER_TEMP_M1:                              /*上面红灯快闪8次*/
            LEDSet(LED_OFF,LED_ON,8,0,LED_HIGH_SPEED);
            break;
        case MC_OVER_TEMP_M2:                              /*下面红灯快闪8次*/
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
            RED_LED2_OFF;//此处申明：比如发生了不可自动恢复的错误，再发生可自动恢复的错误（比如欠压过压），在自动恢复错误之后，不可恢复的错误将不显示红灯（表现现象为红灯关闭，绿灯不闪烁）
            //2022.3.17解决上面所说bug，自动恢复之后，判断全局错误最低位的错误，将最低位的错误赋值给display_led
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
Function:参考注入，用于自适应位置、速度和电流控制回路
Input   :@injectPoint
         @injectType 曲线类型
         @injectValue
         @injectFreq 频率
         @injectAmp  赋值
         @injectInit
         @*refOut  输出给到电流参考或者速度参考
Output  :No
Explain :用于生成SIN曲线、方波用于测试电流环和速度环响应，修改了最后一个入口参数的类型由long 改为int32_t,因为输入参数参考电流是int32_t否则会报告警告
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
        Vector_Components = MCM_Trig_Functions((int16_t)(*injectPhase));   /*time:530ns，math..中的库函数*/
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

    case INJECT_TYPE_SQUARE_DIRECT:			/*矩形波*/
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
* @function :sin曲线，环路测试电流环速度环响应参数初始化
* @input    :请填写
* @output   :请填写
* @explain  :ReferenceInjection该函数的参数
* @author   :忘川
* @date     :2022/12/16
------------------------------------------------*/
void ReferenceInjectionLoopPar_Init(void)
{
    loopTestPar.InjectPoint[M1] = INJECT_POINT_CURRREF;
    loopTestPar.InjectType[M1] = INJECT_TYPE_SIN_DIRECT;  /*sin曲线*/
    loopTestPar.InjectFreq[M1] = 3;
    loopTestPar.InjectCurrAmp[M1] = 1000;

    loopTestPar.InjectPoint[M2] = INJECT_POINT_CURRREF;
    loopTestPar.InjectType[M2] = INJECT_TYPE_SIN_DIRECT;  /*sin曲线*/
    loopTestPar.InjectFreq[M2] = 3;
    loopTestPar.InjectCurrAmp[M2] = 1000;
}

/*------------------------------------------------
Function:最大力矩限制
Input   :No
Output  :No
Explain :未使用
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
Function:堵转检查
Input   :No
Output  :错误代码：0x00001000
Explain :执行周期：500us  堵转时间2000ms  堵转速度 5RPM 堵转电流（Iq）
------------------------------------------------*/
uint32_t MotorStuckCheck(u8 axes)
{
    uint32_t errorcode = MC_NO_ERROR;
    if(1&&STM[axes].bState == RUN)           /*堵转开关打开电机使能*/
    {
        if((in32abs(FOCVars[axes].Iqdref.q) > MotorParameters[axes].ShortCircuit)\
                && (in32abs(pCtrlPar[axes].Vel_PLL_Motor) <= MotorParameters[axes].LockedSpeed)) /*参考电流＞堵转电流，实际速度＜堵转速度*/
        {
            if(in32abs(FOCVars[axes].Iqd.q) > MotorParameters[axes].ShortCircuit/2)          /*参考电流＞堵转电流的一半*/
            {
                giStuckTimeCnt[axes]++;
            }

            if(giStuckTimeCnt[axes] > MotorParameters[axes].LockeTicks)                      /* 堵转时间2000ms 报错*/
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
Function:交换排序
Input   :No
Output  :No
Explain :为了缺相检测当中的电流进行排序排序之后进行运算
------------------------------------------------*/
void swap(float *x,float *y)
{
    float temp= 0;
    temp=*x;
    *x=*y;
    *y=temp;
}
/*------------------------------------------------
* @function :本算法将a[]中的元素从小到到大进行排序
* @input    :@param需要排序的数组 @param数组大小
* @output   :请填写
* @explain  :请填写
* @author   :忘川
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
                swap(&a[j - 1],&a[j]);      /*为交换函数，将a[j] 与 a[j - 1] 进行交换*/
            }
            else
            {}
        }
    }
}
/*------------------------------------------------
Function:缺相检测的初始化函数
Input   :No
Output  :No
Explain :防止在电机启停时候的参数紊乱，导致的缺相误报
------------------------------------------------*/
void ClearPhaseCheckPar(void)
{
    IabcCurrentTemp[M1][0] = 0;           /*电流缓存清除*/
    IabcCurrentTemp[M1][1] = 0;
    IabcCurrentTemp[M1][2] = 0;
    lostPhaseCheckCnt[M1] = 0;            /*电机缓存计器数清除*/
    Missing_phase_error_code[M1] = 0;     /*错误代码缓存*/
    cur_calculated_value[M1] = 0;         /*比较值清除*/
    iabcurrenttemporary[0] = 0;           /*需要排序的三相电流累加和清除*/
    iabcurrenttemporary[1] = 0;
    iabcurrenttemporary[2] = 0;


    IabcCurrentTemp[M2][0] = 0;           /*M2电机清除*/
    IabcCurrentTemp[M2][1] = 0;
    IabcCurrentTemp[M2][2] = 0;
    lostPhaseCheckCnt[M2] = 0;
    Missing_phase_error_code[M2] = 0;
    cur_calculated_value[M1] = 0;
    iabcurrenttemporary[0] = 0;
    iabcurrenttemporary[1] = 0;
    iabcurrenttemporary[2] = 0;
}/*------------------------------------------------
Function：缺相检测的错误报警函数
Input   :No
Output  :No
Explain :该函数是为了将ADC中断当中的缺相判断与500us当中的故障处理函数联系在一起
------------------------------------------------*/
uint32_t DefaultPhaseCheckLinkFunction(u8 axes)
{
    uint32_t temp = 0;
    temp = Missing_phase_error_code[axes];
    Missing_phase_error_code[axes] = 0;      /*问题起因：由于缺相检测是在RUN状态中计算，在报错之后电机不会在进行检测所以报错之后显示FAULT_NOW*/
    return temp;
}
/*------------------------------------------------
* @function :int32类型数据求绝对值
* @input    :请填写
* @output   :请填写
* @explain  :请填写
* @author   :忘川
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
* @function :long类型数据求绝对值
* @input    :请填写
* @output   :请填写
* @explain  :请填写
* @author   :忘川
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
Function:缺相检测函数
Input   :No
Output  :No
Explain :50us运行一次
------------------------------------------------*/
uint32_t DefaultPhaseCheck(u8 axes)
{
//    static uint32_t  giPhaseTimeCnt[NUMBER_OF_AXES] = {0,0};
    uint32_t errorcode = MC_NO_ERROR;
    if(STM[axes].bState == RUN&&pCtrlPar[axes].SetVelMotor!=0\
            &&l_abs(FOCVars[axes].Iqdref.q) > l_abs(pPIDSpeed[axes]->hUpperOutputLimit-1))   /*【启动时就已经缺相】,参考电流很大*/
    {
        if(in32abs(FOCVars[axes].Iqd.q)<100&&pCtrlPar[axes].Vel_PLL_Motor<50)           /*电机没电流（Iq<100），速度小于5RPM，不过流*/
        {
            lostPhaseCheckCntForLaunch[axes]++;                                         /*缺相检测计数1*/
        }
        else
        {
            if(lostPhaseCheckCntForLaunch[axes]>0)
            {
                lostPhaseCheckCntForLaunch[axes]--;
            }
        }
        if(lostPhaseCheckCntForLaunch[axes]>(uint16_t)(MISSING_PAHSE_JUDGE_TIME/3))                   /*持续300ms错误报错，并且清除错误计数 此处与下面1S 需要不同，
					                                                            此处在整机上出现过一次，如果一个轮子没插，另一个轮子会带动异常轮子转动，
				                                                              如果时间过长可能这个错误很难报出来*/
        {
            errorcode = ERR_LOST_PHASE;
            lostPhaseCheckCntForLaunch[axes] = 0;                                       /*清除错误计数 */
        }
    }
    else
    {
        if(lostPhaseCheckCntForLaunch[axes]>0)
        {
            lostPhaseCheckCntForLaunch[axes]--;
        }
    }
    if(1&&STM[axes].bState == RUN && abs(pCtrlPar[axes].SetVelMotor)>100)              /*【运行过程中缺相】  如果不加速度限制，停机qidong的时候会误报缺相，此处对桩限制电流不影响缺相*/
    {
        lostPhaseCheckCnt[axes]++;                                                     /*缺相检测计数2*/
        if(lostPhaseCheckCnt[axes] > MISSING_PAHSE_JUDGE_TIME)                         /*缺相检测一个测试周期为1000ms*/
        {
            for(u8 i =0; i<3; i++)
            {
                iabcurrenttemporary[i] = IabcCurrentTemp[axes][i];                     /*将错误发生时的数据记录，测试使用*/
            }
            BubbleSort(iabcurrenttemporary,3);
            cur_calculated_value[axes] = (iabcurrenttemporary[2] - iabcurrenttemporary[0])/iabcurrenttemporary[2];  /*最大值减去最小值除以最大值，该值范围0-1 ，正常运行值约为0.0几，异常越接近1*/
            if(cur_calculated_value[axes] > 0.8f)                                      /*已经发生错误0.8为经验值*/
            {
                errorcode = ERR_LOST_PHASE;
                IabcCurrentTemp[axes][0] = 0;
                IabcCurrentTemp[axes][1] = 0;
                IabcCurrentTemp[axes][2] = 0;
                lostPhaseCheckCnt[axes] = 0;
            }
            else //if(lostPhaseCheckCnt[axes] > 6000)                                  /*没有发生错误并且一个测试周期结束，一个测试周期为300ms*/
            {
                IabcCurrentTemp[axes][0] = 0;
                IabcCurrentTemp[axes][1] = 0;
                IabcCurrentTemp[axes][2] = 0;
                lostPhaseCheckCnt[axes] = 0;
            }
        }
        else                                                                           /*在一个测试周期内，进行电流数据累加*/
        {
            if(FOCVars[axes].Iab.a>0)                                                  /*相电流为sin*/
            {
                IabcCurrentTemp[axes][0] += FOCVars[axes].Iab.a;
            }
            else
            {
                IabcCurrentTemp[axes][0] += (-FOCVars[axes].Iab.a);
            }
            if(FOCVars[axes].Iab.b>0)                                                 /*相电流为sin*/
            {
                IabcCurrentTemp[axes][1] += FOCVars[axes].Iab.b;
            }
            else                                                                      /*如果Ib小于0*/
            {
                IabcCurrentTemp[axes][1] += (-FOCVars[axes].Iab.b);
            }
            if(((FOCVars[axes].Iab.a+FOCVars[axes].Iab.b)*-1)>0)                      /*如果 Ic > 0*/
            {
                IabcCurrentTemp[axes][2] += (-1)*(FOCVars[axes].Iab.a+FOCVars[axes].Iab.b);
            }
            else                                                                      /*如果 Ic < 0*/
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
Function:失速检查
Input   :No
Output  :错误代码：ERR_OVER_VEL_ERR 0x00010000
Explain :执行周期：500us 问题：存在BUG电机速度0速度失能之后旋转电机,报错之后直接使能电机可能回进入软件错误
再失能，位置环锁轴在之前的位置
MotorParameters[axes].MAXSpeed该变量为最终设置的软件最大速度限制，该值需要包含由于PID调节的过冲
------------------------------------------------*/
uint32_t StallCheck(u8 axes)
{
    int32_t veltemp = 0;
    uint32_t errorcode = MC_NO_ERROR;
    uint16_t MAX_giStallTimeCnt[NUMBER_OF_AXES] = {200,200};       /*100ms报错*/
//  int32_t  SetVel_Multiply_VelPLL = 0;
    if(1&&STM[axes].bState == RUN)                                 /*失速保护开关打开&&电机使能*/
    {
        veltemp = in32abs(pCtrlPar[axes].Vel_PLL_Motor);
        if(veltemp > MotorParameters[axes].MAXSpeed)               /*超过最大速度保护之*/
        {
            giStallTimeCnt[axes]++;
            if(giStallTimeCnt[axes]>MAX_giStallTimeCnt[axes])      /*100ms报错*/
            {
                errorcode = ERR_OVER_VEL_ERR;
                giStallTimeCnt[axes] = 0;
                return errorcode;
            }
        }
        else if(giStallTimeCnt[axes]>0)                            /*处于正常转速范围*/
        {
            giStallTimeCnt[axes]--;
        }
    }
    else
    {
        giStallTimeCnt[axes] = 0;                                   /*电机未使能直接清零*/
    }
    return errorcode;
}
/*------------------------------------------------
Function:MCU版本号
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver,u32 hardware_version)
{
    pVersion->uVersionPartOfRobotType = robotype;				              /*项目类型	S线*/
    pVersion->uVersionPartOfMainVersion = main_ver;			              /*大版本号*/
    pVersion->uVersionPartOfFunVersion = fun_ver;						          /*功能版本号*/
    pVersion->uVersionPartOfSmallVersion = small_ver;		              /*小版本号，代码bug以及参数微调*/
    pVersion->uVersionFullVersion = (robotype<<24)+(main_ver<<16)+(fun_ver<<8)+small_ver;
}
/*------------------------------------------------
Function:硬件版本号
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void HardVersion_Init(MCU_Version *pVersion,u8 funtype,u8 vol,u8 cur_max,u8 update_ver)
{
    pVersion->uHardwareVersion = (funtype<<24)+(vol<<16)+(cur_max<<8)+update_ver;
}
/*------------------------------------------------
Function:合并两个电机实际转速
Input   :No
Output  :No
Explain :低16位为M1电机，高16位为M2电机
------------------------------------------------*/
void SpeedRelMerge_two_to_one(SEVERO_CONTORL_PAR_T *parM1,SEVERO_CONTORL_PAR_T *parM2)
{
    int32_t Veltemp1,Veltemp2;

    Veltemp1 = parM2->Vel_PLL_Motor&0x0000FFFF;        /*拿出低16位*/
    Veltemp2 = Veltemp1<<16;
    Veltemp1 = parM1->Vel_PLL_Motor&0x0000FFFF;
    parM1->M1M2_VelRel = Veltemp1|Veltemp2;             /*M1为低16位，M2位高16位*/
}
/*------------------------------------------------
Function:拆分两个电机设置转速
Input   :No
Output  :No
Explain :低16位为M1电机，高16位为M2电机
------------------------------------------------*/
void SpeedSetMerge_two_to_one(SEVERO_CONTORL_PAR_T *parM1,SEVERO_CONTORL_PAR_T *parM2,uint8_t mode)
{
    int16_t m1veltemp,m2veltemp;

    m1veltemp = parM1->M1M2_VelSet>>16;                  /*取高16位*/
    m2veltemp = parM1->M1M2_VelSet&0X0000FFFF;           /*取低16位*/
    if(mode == 3)                                        /*速度模式*/
    {
        parM1->SetTorqueMotor = 0;                       /*速度模式下将力矩设置为0*/
        parM2->SetTorqueMotor = 0;
        parM1->SetVelMotor = -m2veltemp;                 /*低16位给M1,需要赋值给16位的变量，再将该变量赋值给32位*/
        parM2->SetVelMotor = -m1veltemp;                 /*高16位给M2，否则在给-速度存在问题  此处取反为了适应中菱电机*/
    }
    else if(mode == 4)                                   /*力矩模式*/
    {
        parM1->SetVelMotor = 0;                          /*力矩模式下将速度设置为0*/
        parM2->SetVelMotor = 0;
        parM1->SetTorqueMotor = m2veltemp;               /*低16位给M1*/
        parM2->SetTorqueMotor = m1veltemp;               /*高16位给M2*/
    }
}
/*------------------------------------------------
Function:霍尔出错检查以及编码器检查
Input   :No
Output  :No
Explain :为了防止误报，最后修改为2s检测周期
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
        hall_error_cnt[axes]++;                     /*500us一次*/
        if(hall_error_cnt[axes] > 4000)             /*2s确认为霍尔错误*/
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
Function:电机温度保护
Input   :No
Output  :No
Explain :500us一次 默认120度 30s保护
------------------------------------------------*/
uint32_t Motor_Over_Temperature(u8 axes)
{
    uint32_t errorcode = 0;
    if(pCtrlPar[axes].MotorTemperature > MotorParameters[axes].MaxTemperature && STM[axes].bState == RUN)   /*120度保护*/
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
    if(temperature_cnt[axes] > MotorParameters[axes].OverTemperatureTicks)                                  /*30s保护*/
    {
        temperature_cnt[axes] = 0;
        errorcode = MC_OVER_TEMP;
    }
    return errorcode;
}
/*------------------------------------------------
Function:电机使能函数
Input   :急停按键
Output  :No
Explain :用于在急停信号按下以后调用，其余地方不调用，同步模式下输入参数为Drive_Data[M1]共同控制两个电机,只考虑同步模式下
------------------------------------------------*/
uint8_t Motor_Enable(Drive_Data *pDriver_Data)
{
    static uint8_t EnableStep[2] = {0,0};                /*两个电机*/
    static uint16_t Statusword = 0;
    Statusword = pDriver_Data->device_control_data.Statusword;
    if((Statusword & 0x006F) == 0x0027)                 /*电机已经使能*/
    {
        EnableStep[0] = 0;                              /*使能以后将步骤清零*/
        return 1;//电机已经使能状态
    }
    else
    {
        if(EnableStep[0] ==0)
        {
            pCtrlPar[M1].SetVelMotor = 0;               /*清空速度参考*/
            pCtrlPar[M2].SetVelMotor = 0;
            pCtrlPar[M1].M1M2_VelSet = 0;
            EnableStep[0] = 1;
        }
        else if((Statusword & 0x006F) == 0x0021)        /*控制字6发送完成，状态字切换为0x0231*/
        {
            EnableStep[0] = 2;
        }
        else if((Statusword & 0x006F) == 0x0023)        /*控制字7发送完成，状态字切换为0x0233*/
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
Function:急停检测功能
Input   :No
Output  :No
Explain :按下为0，正常为1
------------------------------------------------*/
void Emrgency_Stop(void)
{
    EMG_State3 = (Start_Stop_GPIO_Port->IDR>>0)&(0x01);
    if(EMG_State3 != EMG_State4)                                                 /*保证每次状态改变只进行一次操作，一直执行会影响CAN口其他设备控制*/
    {
        if(EMG_State3 == STOPISON)                                               /*急停被按下*/
        {
            if(Motor_Enable(Driver_Data) == TRUE)
            {
                Driver_Data[M1].device_control_data.Quick_stop_option_code = 6;  /*按照6085 设置的急停减速度停机，停机后锁轴*/
                Driver_Data[M1].device_control_data.Conrtolword = 0x0002;        /*快速停机命令*/
//                P_V_I_Switch = 2;                                              /*用于处理位置环急停信号，公司轮毂不运行位置环所以此处粗处理，主要用于记录，未测试位置环急停。*/
                EMG_State4 = EMG_State3;                                         /* EMG_State3恒等于STOPISON*/
            }
            else
            {
                /*使能失败*/

            }
        }
        else    /*急停没有被按下*/
        {

            Driver_Data[M1].device_control_data.Quick_stop_option_code = 1;      /*按照6084 设置的减速度停机， 停机后自由*/
            Driver_Data[M1].device_control_data.Conrtolword = 0x0002;            /*快速停机*/
            EMG_State4 = EMG_State3;                                             /*EMG_State3恒等于 STOPISOFF*/

        }
    }
    else
    {
        /*如果一直处于按下或者非断开，不进行操作*/
    }

}
/*------------------------------------------------
* @function :电机2初始角度定位
* @input    :霍尔2状态
* @output   :电机初始位置
* @explain  :根据霍尔扇区进行初始粗定位，定位为每个扇区的电角度为30度的位置
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
    if(initial2_positioning_timing > 5)                        /*5ms读取5次如果霍尔还是错误的*/
    {
        initial2_positioning_timing = 6;                       /*此处考虑报告什么错误*/
    }
    if(HallState2 != 0 && HallState2 != 7)                     /*正常*/
    {
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);              /*将开启中断发在此处目的是为了防止机器上电推动导致初始定位失败问题*/
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        if(motor2_exit_clear_flag == 0)                        /*静态变量，初始定位完成以后只定位一次*/
        {
            motor2_exit_clear_flag++;
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_U_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_V_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_W_Pin);
        }
    }
}
/*------------------------------------------------
* @function :电机1初始角度定位
* @input    :霍尔1状态
* @output   :电机初始位置
* @explain  :根据霍尔扇区进行初始粗定位，定位为每个扇区的电角度为30度的位置
* @author   :忘川
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
    if(initial_positioning_timing > 5)                           /*5ms读取5次如果霍尔还是错误的*/
    {
        initial_positioning_timing = 6;                          /*此处考虑报告什么错误*/
    }
    if(HallState1 != 0 && HallState1 != 7)                       /*正常*/
    {
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);              /*将开启中断发在此处目的是为了防止机器上电推动导致初始定位失败问题*/
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        if(motor1_exit_clear_flag == 0)                          /*静态变量，初始定位完成以后只定位一次*/
        {
            motor1_exit_clear_flag++;
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_U_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_V_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_W_Pin);
        }
    }
}
/*------------------------------------------------
* @function :获取电机2霍尔信号
* @input    :外部电平
* @output   :123456
* @explain  :请填写
* @author   :请填写
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
    return (u8)(tmp & 0x0007); //取低三位
}
/*------------------------------------------------
* @function :获取电机1霍尔信号
* @input    :外部电平
* @output   :123456
* @explain  :UVW为什么不按照顺序来写，是首批电机UVW线序问题，
             按照这个霍尔调通以后一直延续至今
* @author   :忘川
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

    return (u8)(tmp & 0x0007); //取低三位
}
/*------------------------------------------------
* @function :清除Brake滤波计数
* @input    :请填写
* @output   :请填写
* @explain  :100ms调用一次
* @author   :忘川
* @date     :2022/12/16
------------------------------------------------*/
void Clear_Brake_Filters_Cnt(void)
{
    BrakeFiltersCnt[0] = 0;
    BrakeFiltersCnt[1] = 0;
}
/*------------------------------------------------
* @function :阻尼模式功能
* @input    :
* @output   :
* @explain  :报错之后并且收到正常通信之后打开阻尼模式
             damping_mode_flag在收到CAN指令以后置为1，防止未上电推动机器报告过压打开阻尼模式
             去掉欠压错误下打开阻尼模式
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void Damping_Of_Motor(void)
{
    /*如果出错自动打开阻尼模式*/
    if((wGlobal_Flags != 0)&&(damping_mode_flag == 1)&&((wGlobal_Flags&MC_UNDER_VOLT) == 0))
    {
        Damping_Mode_ON_OFF = DAMPING_ON;
    }
    /*如果错误被清除自动关闭阻尼模式*/
    if(wGlobal_Flags == 0)                             /*如果错误被清除则退出阻尼模式[027版本修改]*/
    {
        Damping_Mode_ON_OFF = DAMPING_OFF;             /*关闭阻尼模式*/
    }
    if(Damping_Mode_ON_OFF == DAMPING_ON&&STM[M1].bState != RUN && STM[M2].bState != RUN)
    {
        /*打开三个下管*/
        TIM1->CR2 = TURN_ON_THREE_LOW;
        TIM8->CR2 = TURN_ON_THREE_LOW;
        Damping_Mode_ON_OFF = STATE_DAMPING_IS_ON;     /*三个下管的状态为打开*/
    }
    else if (Damping_Mode_ON_OFF == DAMPING_OFF)       /*关闭阻尼模式*/
    {
        /*关闭三个下管*/
        TIM1->CR2  = TURN_OFF_THREE_LOW;
        TIM8->CR2  = TURN_OFF_THREE_LOW;
        Damping_Mode_ON_OFF = STATE_DAMPING_IS_OFF;    /*三个下管的状态为打关闭*/
    }

}
/*------------------------------------------------
* @function :上电自启动函数
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
* @function :M1M2电机初始位置学习
* @input    :
* @output   :
* @explain  :最终为了得到 HALL_PHASE_SHIFT HallStudyFlag1参数写1：之前应该使能电机，空载写入1以后电机，从固定位置启动旋转一周自动失能。
             2：将电机拖动到电角度为0的位置
* @author   :wangchuan
* @date     :2022/12/30
------------------------------------------------*/
void M1M2_Initial_Position(void)
{
    if(HallStudyFlag1 == 1)                                                   /*M1霍尔学习寻找初始位置偏差*/
    {
        CNT_1MS[M1] = 0;
        if(HallStudyCNT < Hall_EAC_Ticks)                                     /*计时3秒*/
        {
            Angle_Switch = 0;                                                 /*FOC中将电机拖动到一个电角度为0的位置*/
        }
        else if(HallStudyCNT < 3*Hall_EAC_Ticks)                              /*计时6秒，合计：9秒，已经过了3秒*/
        {
            if(Hall_AvElAngle1CNT >=MotorParameters[M1].PolePairNum)          /*转过1RPM*/
            {
                if(Hall_AvElAngle1CNT == MotorParameters[M1].PolePairNum)     /*转过1RPM*/
                {
                    Hall_AvElAngle1 = (Hall_AvElAngle1Sum/Hall_AvElAngle1CNT);/*平均每个周期的电角度*/
                    Hall_AvElAngle1Temp = (u16)Hall_AvElAngle1;
                    HALL_M1.PhaseShift = Hall_AvElAngle1;
                    HALL_PHASE_SHIFT = (s16)((u32)Hall_AvElAngle1Temp);       /*得到初始位置*/
                }
                Hall_AvElAngle1CNT = MotorParameters[M1].PolePairNum+1;
                Hall_AvElAngle1Sum =0;
//                    HALL_CC_First = 2;
                pCtrlPar[M1].SetVelMotor = 0;
                Angle_Switch = 0;                                              /*学习完成切换状态*/
                MC_StopMotor1();

            }
            else
            {
                Angle_Switch = 2;                                              /*FOC中Angle_Switch=2为拖动电机旋转*/
            }
        }
        else
        {
            HallStudyFlag1 = 0;                                                /*将学习中用到的参数清除*/
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
    else if(HallStudyFlag1 == 2)                                               /*将电机拖动到电角度0位*/
    {
        MC_StartMotor1();                                                      /*使能电机*/
        CNT_1MS[M1] = 0;
        if(HallStudyCNT < Hall_EAC_Ticks)                                      /*定时3秒*/
        {
            Angle_Switch = 0;                                                  /*FOC中给定Vq*/
        }
        else
        {
            MC_StopMotor1();                                                   /*失能电机*/
            HallStudyFlag1 = 0;                                                /*拖动完成*/
            HallStudyCNT = 0;
            Angle_Switch = 3;
            ENCODER_M1._Super.wPulseNumber	= 0;                               /*电机实际位置清0*/
            CurrentPosition1 =  MC_GetCurrentPosition1();
            ENCODER_M1.Angle_Compensation = 0 ;                                /*角度补偿值清0*/
            HALL_CC_First = 2;

        }
        HALL_CH1_ValueOffset = HALL_CH1_ValueDelta1;
    }
    else
    {
        HallStudyCNT = 0;
    }


    if(HallStudyFlag2 == 1)                                                     /*M1霍尔学习寻找初始位置偏差*/
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

