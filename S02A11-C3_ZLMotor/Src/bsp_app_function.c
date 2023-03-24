/*
*******************************************************************************
 *°æ    È¨£º 2021-xxxx,  GaussianRobot
 *ÎÄ ¼ş Ãû£º bsp_app_function.c º¯Êı
 *¼ò    ½é£º µç»ú¿ØÖÆËùÓĞ¹¦ÄÜĞÔº¯Êı¶¼Ó¦¸ÃĞ´ÔÚ¸ÃÎÄ¼ş¼ĞÏÂ
 *×÷    Õß£º LMmotor\Íü´¨
 *ÈÕ    ÆÚ£º 2022.1.4
 *¹¦ÄÜÃèÊö£º
*******************************************************************************
 *±¸×¢£º
*******************************************************************************
*/
#include "bsp_app_function.h"
#include "ds402.h"
#include "mc_config.h "
#include "stdlib.h"
#include "bsp_CAN.h"


/*LEDSet DisplayErrorLed_Handle*/
uint32_t DisplayCounter = 0;                                     /*Ö¸Ê¾µÆÉÁË¸ÆµÂÊ*/

/*Independent_Motor_Control*/
uint8_t PositionModeFlag1=0,PositionModeFlag2=0;                 /*0£ºĞèÒª½øĞĞÎ»ÖÃ»·ËøÖá£¬ÔÚËøÖáÍê³ÉÒÔºó×Ô¶¯ÖÃÎª1£¬´ò¿ªÎ»ÖÃ»·¿ØÖÆ*/
uint8_t relative_location_flag[MAX_AXES] = {0};                  /*Ïà¶ÔÎ»ÖÃ¿ØÖÆ1£ºÖ´ĞĞÏà¶ÔÎ»ÖÃ Ö´ĞĞÒ»´ÎÍêÒÔºó×Ô¶¯ÖÃ0*/
uint8_t absolute_location_flag[MAX_AXES] = {0};                  /*¾ø¶ÔÎ»ÖÃ¿ØÖÆ²ÎÊı*/
uint8_t P_V_I_Switch_using[MAX_AXES];                            /*ÈıÖÖÄ£Ê½ÇĞ»»±êÖ¾Î»*/
uint8_t modetemp[MAX_AXES]= {3,3};                               /*ÉÏµç´¦ÓÚËÙ¶ÈÄ£Ê½£¬¸ÃÖµ³õÊ¼»¯Îª3*/
uint8_t  LockAxleON_OFF = 0 ;                                    /*ËøÖá¿ª¹Ø*/
uint8_t initial2_positioning_timing = 0;                         /*³õÊ¼¶¨Î»´íÎó¼ì²é*/
uint8_t initial_positioning_timing = 0;                          /*³õÊ¼¶¨Î»´íÎó¼ì²é*/
uint16_t Sync_Async_Control = 1;                                 /*Í¬²½Ä£Ê½Òì²½Ä£Ê½Ñ¡Ôñ²ÎÊı*/
int32_t Countrer_1ms_CNT=200,Countrer2_1ms_CNT=200;              /*ÓÃÓÚµç»ú12µÄÎ»ÖÃ»·ËøÖá¼ÆÊ±*/
long RelativeLocation[MAX_AXES] = {0};                           /*Ïà¶ÔÎ»ÖÃ²Î¿¼*/
float CurrentPosition1,CurrentPosition2;                         /*ÓÃÓÚµç»ú12µÄÎ»ÖÃ»·ËøÖá£¬ÒÔ¼°ÔÚmc_it.cµ±ÖĞ»ñÈ¡µ½Êı¾İÒÔºó¸øµ½pPosCtrl[M1]->ThetaÓëÎ»ÖÃ¹æ»®ÓĞ¹Ø*/


/*DefaultPhaseCheck*/
uint32_t  giStuckTimeCnt[NUMBER_OF_AXES] = {0,0};                /*¶Â×ªÊ±¼ä¼ÆÊı*/

/*ClearPhaseCheckPar  DefaultPhaseCheck*/
long IabcCurrentTemp[2][3] = {0};                                /*Á½¸öµç»úµÄ¶şÎ¬Êı×é£¬abcÈıÏàµçÁ÷»º´æ*/
float cur_calculated_value[2] = {0};                             /*300msµçÁ÷ÀÛ¼ÆÅÅĞòÒÔºó£¬¼ÆËã£º(Imax-Imin)/Imax ·¶Î§[0,1]*/
float iabcurrenttemporary[3] = {0};                              /*µ¥¸öµç»úÈıÏàµçÁ÷£¬ÊäÈë¸øÅÅĞòº¯Êı*/
uint16_t  lostPhaseCheckCnt[2] = {0};                            /*µç»úÔËĞĞÊ±È±Ïà¼ì²é¼ÆÊı*/
uint16_t  lostPhaseCheckCntForLaunch[2] = {0};                   /*µç»úÆô¶¯Ê±È±Ïà¼ì²é¼ÆÊı*/
uint32_t Missing_phase_error_code[2] = {MC_NO_ERROR};            /*È±Ïà´íÎó´úÂë*/
uint32_t MISSING_PAHSE_JUDGE_TIME = 10000;                       /*³ÖĞø2000ms´íÎó±¨´í£¬¸ÃÖµÌ«µÍÖØÔØÆô¶¯Æğ²»À´*/

/*void Emrgency_Stop(void)*/
uint16_t EMG_State3,EMG_State4 = 2;                              /*¼±Í£°´Å¥STOPISOFF(1) or STOPISON(0)*/
uint16_t  giStallTimeCnt[NUMBER_OF_AXES] = {0};                  /*Ê§ËÙ¼ì²éÀÛ¼ÆÊ±¼ä¼ÆÊı*/
uint32_t hall_error_cnt[2] = {0,0};                              /*»ô¶û´íÎó¼ì²éÀÛ¼ÆÊ±¼ä¼ÆÊı*/
uint32_t temperature_cnt[2] = {0};                               /*µç»úÎÂ¶È±£»¤Ê±¼äÀÛ¼Æ¼ÆÊı*/

Trig_Components Vector_Components;                               /*ÓÃÓÚÉú³ÉSINÇúÏß¡¢·½²¨ÓÃÓÚ²âÊÔµçÁ÷»·ºÍËÙ¶È»·ÏìÓ¦*/
LOOP_TEST_T	 loopTestPar;								                         /*ÓÃÓÚÉú³ÉSINÇúÏß¡¢·½²¨ÓÃÓÚ²âÊÔµçÁ÷»·ºÍËÙ¶È»·ÏìÓ¦*/
u8    Damping_Mode_ON_OFF = 0;                                   /*×èÄáÄ£Ê½¿ª¹Ø*/
SEVERO_CONTORL_PAR_T pCtrlPar_M1=                                /*¸øSEVERO_CONTORL_PAR_TÀàĞÍµÄpCtrlPar_M1¸³³õÖµÓÃÓÚÓÃÓÚ½Ó¿Ú£¬×îÖÕ pCtrlPar[M1]  = *(&pCtrlPar_M1); ¸ÃµØÖ·Ö¸ÕëÖ¸Ïò*/
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

SEVERO_CONTORL_PAR_T pCtrlPar[NUMBER_OF_AXES];                         /*¶¨ÒåÁ½¸öSEVERO_CONTORL_PAR_TÀàĞÍµÄ½á¹¹Ìå£*/

/*»ô¶ûÑ§Ï°Ïà¹Ø²ÎÊı*/
int16_t Hall_AvElAngle1 = 0;                                           /*M1Ò»È¦µÄÆ½¾ùµç½Ç¶È = Hall_AvElAngle1Sum /Hall_AvElAngle1CNT */
int16_t Hall_AvElAngle1CNT = 0;                                        /*M1¼«¶ÔÊı¼ÆÊı*/
int16_t Hall_AvElAngle2 = 0;                                           /*M2Ò»È¦µÄÆ½¾ùµç½Ç¶È*/
int16_t Hall_AvElAngle2CNT = 0;                                        /*M2¼«¶ÔÊı¼ÆÊı*/
int16_t Hall_AvElAngle2Temp,Hall_AvElAngle1Temp;
int16_t Angle_Switch2=3,Angle_Switch = 3;
uint16_t HallStudyCNT=0,HallStudyCNT2=0,Hall_EAC_Ticks = 3000;         /*»ô¶ûÑ§Ï°£¬¼ÆÊ±*/
int32_t Hall_AvElAngle1Sum = 0;                                        /*M1Ò»È¦µç½Ç¶È×ÜºÍ*/
int32_t Hall_AvElAngle2Sum = 0;                                        /*M2Ò»È¦µç½Ç¶È×ÜºÍ*/
uint8_t HallStudyFlag1=0,HallStudyFlag2=0;                             /*»ô¶ûÑ§Ï°±êÖ¾£¬Ö»ÓĞµ÷ÊÔÊ±Ê¹ÓÃ*/
uint8_t HALL_CC_First=0,HALL2_CC_First=0,HALL_CC_First11=0;            /*µç»úÊ×´ÎÆô¶¯ÅĞ¶ÏÌõ¼ş*/
/*------------------------------------------------
Function:ËÙ¶ÈÄ£Ê½ËÙ¶ÈÄ£Ê½Á¦¾ØÄ£Ê½ÇĞ»»&Èı»·Ö¸ÁîÖ´ĞĞÈë¿Ú
Input   :No
Output  :No
Explain :Modes_of_operation
					1 Î»ÖÃÄ£Ê½  	Ä¬ÈÏËÙ¶ÈÄ£Ê½
					3 ËÙ¶ÈÄ£Ê½
					4 Á¦¾ØÄ£Ê½
         ÖÜÆÚ£º500us
------------------------------------------------*/
void Independent_Motor_Control(void)
{
    static long delay_200ms[2]= {200,200};
    Drive_Data *pCiA402Axis;
    u16 counter = 0;
    static long location_cache[MAX_AXES] = {0};
    for(counter = 0; counter < MAX_AXES; counter++)
    {
        if(!Driver_Data[counter].device_control_data.bAxisIsActive)       /*×¢Òâ¸³³õÖµ1*/
        {
            continue;
        }
        if(Driver_Data[M1].device_control_data.Modes_of_operation == 3\
                || Driver_Data[M1].device_control_data.Modes_of_operation == 4) /*Èç¹ûÊÇM1ËÙ¶ÈÄ£Ê½»òÕßÁ¦¾ØÄ£Ê½£¬ÄÇÃ´Ä¬ÈÏ½«Á½¸öµç»úµÄ²Ù×÷Ä£Ê½ÉèÖÃÎªÒ»ÖÂ£¬ÒòÎªÉÏÎ»»úÖ»¿ØÖÆÒ»¸öµç»úµÄ¿ØÖÆ×Ö*/
        {
            Driver_Data[M2].device_control_data.Modes_of_operation = Driver_Data[M1].device_control_data.Modes_of_operation; /*Í¬²½Ä£Ê½ÏÂ£º¿ØÖÆM1µç»úµÄ¿ØÖÆÄ£Ê½¼´¿É£¡*/
        }
        if(Sync_Async_Control == 1)                                       /*0£ºM1ºÍM2Òì²½¿ØÖÆ  1£ºM1ºÍM2Í¬²½¿ØÖÆ£¨¼æÈİÖĞÁâ£©*/
        {
            Driver_Data[M2].device_control_data.Modes_of_operation = Driver_Data[M1].device_control_data.Modes_of_operation; /*Í¬²½Ä£Ê½ÏÂ£º¿ØÖÆM1µç»úµÄ¿ØÖÆÄ£Ê½¼´¿É£¡*/
        }

        pCiA402Axis = &Driver_Data[counter];                              /*Ö¸Õë¸³Öµ*/

        if(pCiA402Axis->device_control_data.Modes_of_operation == 1)      /*Î»ÖÃÄ£Ê½*/
        {
            if(modetemp[counter] != 1)                                    /*switchµÄÑ¡ÔñÌõ¼ş²»ÎªÎ»ÖÃÄ£Ê½*/
            {
                modetemp[counter] = 2;                                    /*case2 Îª ÇĞ»»µ½Î»ÖÃÄ£Ê½µÄÖĞ¼ä¹ı³Ì£¬´ï³Écase2µ±ÖĞÌõ¼ş£¬»áÇĞ»»µ½Î»ÖÃÄ£Ê½*/
                P_V_I_Switch_using[counter] = 3;
            }
        }
        else if(pCiA402Axis->device_control_data.Modes_of_operation == 3) /*ËÙ¶ÈÄ£Ê½*/
        {
            if(modetemp[counter] != 3)                                    /*switchµÄÑ¡ÔñÌõ¼ş²»ÎªËÙ¶ÈÄ£Ê½*/
            {
                pCtrlPar[M1].M1M2_VelSet = 0;                             /*Çå³ıÍ¬²½Ä£Ê½ÏÂµÄËÙ¶È²Î¿¼£¬ËÙ¶ÈÄ£Ê½ÏÂ¸Ã²ÎÊı´ú±íÏÂ·¢ËÙ¶È£¬Á¦¾ØÄ£Ê½ÏÂ¸Ã²ÎÊı´ú±íÏÂ·¢Á¦¾Ø£¬*///
                pCtrlPar[M1].SetVelMotor = 0;                             /*Çå³ıM1µç»úÒì²½Ä£Ê½ÏÂËÙ¶È²Î¿¼*/
                pCtrlPar[M2].SetVelMotor = 0;
                pCtrlPar[M1].SetTorqueMotor = 0;                          /*Çå³ıM1µç»úÒì²½Ä£Ê½ÏÂÁ¦¾Ø²Î¿¼*/
                pCtrlPar[M2].SetTorqueMotor = 0;
                pPIDSpeed[M1]->wIntegralTerm = 0;                         /*ËÙ¶È»·»ı·ÖÏîÇå³ı*/
                pPIDSpeed[M2]->wIntegralTerm = 0;
                modetemp[counter] = 5;                                    /*case5ÎªÇĞ»»µ½ËÙ¶ÈÄ£Ê½µÄÖĞ¼äÌõ¼ş£¬´ï³Écase5µ±ÖĞÌõ¼ş£¬»áÇĞ»»µ½case 3ËÙ¶ÈÄ£Ê½*/
            }
        }
        else if(pCiA402Axis->device_control_data.Modes_of_operation == 4) /*Á¦¾ØÄ£Ê½*/
        {
            if(modetemp[counter]!=4)
            {
                /*Ê×´Î½øÈëÁ¦¾ØÄ£Ê½ĞèÒªÇå³ıÁ¦¾Ø²Î¿¼*/
                pCtrlPar[M1].M1M2_VelSet = 0;                             /*Çå³ıÍ¬²½Ä£Ê½ÏÂÁ¦¾Ø²Î¿¼£¬ËÙ¶ÈÄ£Ê½ÏÂ¸Ã²ÎÊı´ú±íÏÂ·¢ËÙ¶È£¬Á¦¾ØÄ£Ê½ÏÂ¸Ã²ÎÊı´ú±íÏÂ·¢Á¦¾Ø*/
                pCtrlPar[M1].SetTorqueMotor = 0;
                pCtrlPar[M2].SetTorqueMotor = 0;
                pCtrlPar[M1].SetVelMotor = 0;                             /*Çå³ıM1µç»úÒì²½Ä£Ê½ÏÂËÙ¶È²Î¿¼*/
                pCtrlPar[M2].SetVelMotor = 0;
                pPIDSpeed[M1]->wIntegralTerm = 0;                         /*ËÙ¶È»·»ı·ÖÏîÇå³ı*/
                pPIDSpeed[M2]->wIntegralTerm = 0;
                modetemp[counter] = 4;                                    /*ÓÉÆäËû»·ÇĞ»»µ½Á¦¾ØÄ£Ê½Ö±½ÓÇĞ»»¾ÍOK*/
            }
        }
        /*end*/
        if(Sync_Async_Control == 1)                                       /*0£ºM1ºÍM2Òì²½¿ØÖÆ  1£ºM1ºÍM2Í¬²½¿ØÖÆ£¨¼æÈİÖĞÁâ£©*/
        {
            SpeedSetMerge_two_to_one(&pCtrlPar[M1],&pCtrlPar[M2],modetemp[counter]);
        }


        switch(modetemp[counter])
        {
        case 0:
            break;
        case 1 :                                                          /*¡¾Î»ÖÃÄ£Ê½¡¿*/

            if(counter == M1)                                             /*µç»ú1Î»ÖÃ»·*/
            {
                if(relative_location_flag[counter] == 1)                  /*Ïà¶ÔÎ»ÖÃÄ£Ê½¿ª¹Ø*/
                {
                    relative_location_flag[counter] = 0;
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;    /*´ò¿ªÎ»ÖÃ¿ØÖÆÄ£Ê½¿ª¹Ø*/
                    location_cache[counter] = ENCODER_M1._Super.wPulseNumber+pCtrlPar[M1].SetPulseMotor; /*Ã¿´Î¸ø¶¨Î»ÖÃÔÚµ±Ç°µÄÎ»ÖÃÉÏ¼ÓÉÏ¸ø¶¨Î»ÖÃ*/
                    MC_ProgramPositionCommandMotor1(location_cache[counter] , 0 );
                    location_cache[counter] = 0;                          /*Çå0,ÏÂ´ÎÊ¹ÓÃ*/
                }
                else if(absolute_location_flag[counter] == 1)             /*¾ø¶ÔÎ»ÖÃÄ£Ê½¿ª¹Ø*/
                {
                    absolute_location_flag[counter] = 0;
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 1;    /*´ò¿ªÎ»ÖÃ¿ØÖÆÄ£Ê½¿ª¹Ø*/
                    MC_ProgramPositionCommandMotor1( pCtrlPar[M1].SetPulseMotor, 0 );
                }
            }
            else if(counter == M2)                                        /*µç»ú2Î»ÖÃ»·*/
            {
                if(relative_location_flag[counter] == 1)                  /*Ïà¶ÔÎ»ÖÃÄ£Ê½¿ª¹Ø*/
                {
                    relative_location_flag[counter] = 0;                  /*Ö´ĞĞÍê³ÉÒÔºó×Ô¶¯ÖÃ0£¬²Ù×÷¾ÍĞèÒªÃ¿´ÎÏÂ·¢Ïà¶ÔÎ»ÖÃÒÔºó£¬ÔÚĞ´Èë¸ÃÎ»Îª1·½¿ÉÖ´ĞĞÏà¶ÔÎ»ÖÃÖ¸Áî£¬¸Ã²ÎÊıÄ¿Ç°Ã»ÓĞ¼ÓÈë¶ÔÏó×Öµä£¬ËùÒÔ²»¿ÉÓÃ*/
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;    /*´ò¿ªÎ»ÖÃ¿ØÖÆÄ£Ê½¿ª¹Ø*/
                    location_cache[counter] = ENCODER_M2._Super.wPulseNumber+pCtrlPar[M2].SetPulseMotor; /*Ã¿´Î¸ø¶¨Î»ÖÃÔÚµ±Ç°µÄÎ»ÖÃÉÏ¼ÓÉÏ¸ø¶¨Î»ÖÃ*/
                    MC_ProgramPositionCommandMotor2( location_cache[counter], 0 );
                    location_cache[counter] = 0;
                }
                else if(absolute_location_flag[counter] == 1)             /*¾ø¶ÔÎ»ÖÃÄ£Ê½¿ª¹Ø*/
                {
                    absolute_location_flag[counter] = 0;
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 1;    /*´ò¿ªÎ»ÖÃ¿ØÖÆÄ£Ê½¿ª¹Ø*/
                    MC_ProgramPositionCommandMotor2( pCtrlPar[M2].SetPulseMotor, 0 );
                }
            }
            break;
        case 2 :                                                          /*ËÙ¶ÈÄ£Ê½ºÍÎ»ÖÃÄ£Ê½ÇĞ»»*/
            if(counter == M1)
            {
                pCtrlPar[M1].SetVelMotor = 0 ;
                pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
                MC_ProgramSpeedRampMotor1(pCtrlPar[M1].SetVelMotor, 100 );
                delay_200ms[counter]--;
                if(delay_200ms[counter] < 0)
                {
                    delay_200ms[counter] = 200;
                    if(P_V_I_Switch_using[counter] == 2)                 /*µç»úM1  Î»ÖÃ»·----->ËÙ¶È»·*/
                    {
                        if(pCtrlPar[M1].Vel_PLL_Motor==0)
                        {
                            modetemp[counter] = 3;                       /*ÇĞ»»µ½ËÙ¶È»·*/
                            CNT_1MS[counter] = 0;
                            PositionModeFlag1 = 0;                       /*¹Ø±ÕÎ»ÖÃ»·¿ØÖÆ*/
                        }
                    }
                    else  if(P_V_I_Switch_using[counter] == 3)           /*µç»úM1 ËÙ¶È»·----->Î»ÖÃ»·*/
                    {
                        if(pCtrlPar[M1].Vel_PLL_Motor ==0)
                        {
                            modetemp[counter] = 1;                       /*ÇĞ»»µ½Î»ÖÃ»·*/
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
                            modetemp[counter] = 3;                       /*ÇĞ»»µ½ËÙ¶È»·*/
                            CNT_1MS[counter] = 0;
                            PositionModeFlag2 = 0;                       /*¹Ø±ÕÎ»ÖÃ»·¿ØÖÆ*/
                        }
                    }
                    else  if(P_V_I_Switch_using[counter] == 3)
                    {
                        if(pCtrlPar[M2].Vel_PLL_Motor ==0)
                        {
                            modetemp[counter] = 1;                       /*ÇĞ»»µ½Î»ÖÃ»·*/
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
        case 3 :  /*¡¾ËÙ¶ÈÄ£Ê½¡¿*/

            if(counter == M1&&STM[M1].bState > 5)
            {
                Countrer_1ms_CNT = pCtrlPar[M1].hDurationms+1000;
                if(LockAxleON_OFF == 0)                                                                               /*ËÙ¶È»·ËøÖá*/
                {
                    if(PendingOpcode_handleWaitStatus[counter] == TRUE)                                               /*Èç¹ûÖ´ĞĞ¿ìËÙÍ£»úÃüÁî*/
                    {
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 1\
                                ||Driver_Data[counter].device_control_data.Quick_stop_option_code == 5)                     /*°´ÕÕ6084ÉèÖÃµÄ¼õËÙ¶ÈÊ±¼äÍ£»ú*/
                        {
                            MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms);
                        }
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 2\
                                ||Driver_Data[counter].device_control_data.Quick_stop_option_code == 6)                      /*°´ÕÕ6085ÉèÖÃµÄ¼õËÙ¶ÈÊ±¼äÍ£»ú*/
                        {
                            MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[M1].quick_stop_decel);  /*ÕâÀïµÄpCtrlPar[M1].quick_stop_decel²»ÒªĞŞ¸ÄM1M2¹²ÓÃ*/
                        }
                    }
                    else if(EMG_State3 ==STOPISOFF)                                                                    /*Õı³£ÔËĞĞ£¬¼±Í£¿ª¹ØÎ´±»°´ÏÂ*/
                    {
                        MC_ProgramSpeedRampMotor1( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms );
                    }

                    CurrentPosition1 =  MC_GetCurrentPosition1();
                    pMCI[M1]->pPosCtrl->PositionControlRegulation = 0;
                    CNT_1MS[counter] = 0;
                    PositionModeFlag1 = 0;
                }
                else if(LockAxleON_OFF == 1)  /*M1Î»ÖÃ»·ËøÖáÎ´Ê¹ÓÃ£¬Éè¼Æ³õÆÚÎªÁË¿¼ÂÇËÙ¶È»·ÔÚĞ±ÆÂËøÖá¿ÉÄÜµ¼ÖÂÁïÆÂ*/
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
                if(LockAxleON_OFF == 0)                                /*ËÙ¶È»·ËøÖá*/
                {
                    if(PendingOpcode_handleWaitStatus[counter] == TRUE)/*Èç¹ûÖ´ĞĞ¿ìËÙÍ£»úÃüÁî*/
                    {
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 1||Driver_Data[counter].device_control_data.Quick_stop_option_code == 5)/*°´ÕÕ6084ÉèÖÃµÄ¼õËÙ¶ÈÊ±¼äÍ£»ú*/
                        {
                            MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms);
                        }
                        if(Driver_Data[counter].device_control_data.Quick_stop_option_code == 2||Driver_Data[counter].device_control_data.Quick_stop_option_code == 6)/*°´ÕÕ6085ÉèÖÃµÄ¼õËÙ¶ÈÊ±¼äÍ£»ú*/
                        {
                            MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[M1].quick_stop_decel);    /*ÕâÀïµÄpCtrlPar[M1].quick_stop_decelM1M2¹²ÓÃ   ²»ÒªĞŞ¸Ä*/
                        }
                    }
                    else if(EMG_State3 ==STOPISOFF)                    /*Õı³£ÔËĞĞ£¬¼±Í£¿ª¹ØÎ´±»°´ÏÂ*/
                    {
                        MC_ProgramSpeedRampMotor2( pCtrlPar[counter].SetVelMotor, pCtrlPar[counter].hDurationms );
                    }
                    CurrentPosition2 =  MC_GetCurrentPosition2();
                    pMCI[M2]->pPosCtrl->PositionControlRegulation = 0;
                    CNT_1MS[counter] = 0;
                    PositionModeFlag2 = 0;
                }
                else if(LockAxleON_OFF == 1)                          /*M2Î»ÖÃ»·ËøÖáÎ´Ê¹ÓÃ£¬Éè¼Æ³õÆÚÎªÁË¿¼ÂÇËÙ¶È»·ÔÚĞ±ÆÂËøÖá¿ÉÄÜµ¼ÖÂÁïÆÂ*/
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
        case 4 :                                             /*µ±Ç°´¦ÓÚÁ¦¾ØÄ£Ê½*/
            if(counter==M1)
            {
                MC_ProgramTorqueRampMotor1(pCtrlPar[M1].SetTorqueMotor, 0);    /*Á¦¾ØÏÂ·¢*/
            }
            if(counter==M2)
            {
                MC_ProgramTorqueRampMotor2(pCtrlPar[M2].SetTorqueMotor, 0);
            }

            break;
        case 5 :                                             /*Á¦¾ØÄ£Ê½ÇĞ»»µ½ËÙ¶ÈÄ£Ê½£¬Èç¹ûÓĞËÙ¶È¾Í¸ø·´ÏòÁ¦¾ØÊ¹Êµ¼ÊËÙ¶ÈÎŞÏŞÇ÷½üÓÚ0*/

            if(counter==M1)
            {
                int32_t k1 = 30,vel1 = 0,Iqtemp1 = 0;
                vel1 = pCtrlPar[M1].Vel_PLL_Motor;
                Iqtemp1 = -1*k1*vel1;                        /*k1ÊÇÕû³µµ÷ÊÔ³öÀ´µÄ²ÎÊı*/
                if( 11900 < Iqtemp1)
                {
                    Iqtemp1 = 11900;
                }
                else if(Iqtemp1 < -11900)
                {
                    Iqtemp1 = -11900;
                }
                MC_ProgramTorqueRampMotor1(Iqtemp1, 100);   /*¸ø¶¨·´ÏòÁ¦¾Ø*/
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
            delay_200ms[counter]--;                            /*ÕâÑùÇĞ»»¿ÉÄÜÔÚÍâÁ¦Ìõ¼şÏÂµ¼ÖÂËÙ¶È²»Îª0£¬¾ÍÒ»Ö±²»ÄÜÇĞ»»µ½ËÙ¶ÈÄ£Ê½£¬µ¼ÖÂÁï³µ*/
            if(delay_200ms[counter] < 0)
            {
                delay_200ms[counter] = 100;
//                if(abs(pCtrlPar[counter].Vel_PLL_Motor) < 30)/*¸ÃËÙ¶ÈÊÇÕû³µµ÷ÊÔ³öÀ´µÄ²»»á·¢Éú¶¶¶¯µÄÇé¿ö¡¢ÆÁ±Î´Ë´¦Ê±ÎªÁËÅÄÏÂ¼±Í£»úÆ÷Á¢¼´Í£Ö¹*/
                {
                    pSTC[M1]->SpeedRefUnitExt   = -65460;      /*Çå³ı¸ÃËÙ¶È²Î¿¼·ñÔòÔÚ´ÓÁ¦¾Ø»·ÇĞ»»µ½ËÙ¶È»·µç»ú»áÕğ¶¯*/
                    pSTC[M2]->SpeedRefUnitExt   = -65460;
                    modetemp[counter] = 3;                     /*ÇĞ»»µ½ËÙ¶È»·*/
                }
            }
            break;

        default:
            break;
        }
    }
}
/*------------------------------------------------
Function:ledµÆÉèÖÃ
Input   :led1: 1/ÏÂÃæºìµÆÁÁ  0/ÏÂÃæºìµÆ²»ÁÁ
				 led2: 1/ÉÏÃæºìµÆÁÁ  0/ÉÏÃæºìµÆ²»ÁÁ
				 led_time:ÉÁË¸µÄ´ÎÊı
				 led_mode:1/ÂıÉÁÒ»´Î 0/¿ìÉÁ
				 led_speed:ÉÁË¸µÄËÙ¶È
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
    else if(led_mode)                            	  /*ÂıÉÁ*/
    {
        if(DisplayCounter<2*led_speed)              /*10ms×ÔÔöÒ»´Î*/
        {
            RED_LED1_ON;
        }
        else if(DisplayCounter%(2*led_speed)==0&&DisplayCounter<4*led_time*led_speed)
        {
            RED_LED1_TOGGLE;
        }
    }
    else if(DisplayCounter<=(2*led_time)*led_speed)	/*¿ìÉÁ*/
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
Function:ÉèÖÃ´íÎó±êÖ¾
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u32 wGlobal_Flags = 0;
u32 display_led = 0;
void MC_SetFault(u32 hFault_type)
{
    wGlobal_Flags |= hFault_type;         /*È«¾Ö´íÎóÖ¸Ê¾*/
    if(display_led !=0)                   /*Èç¹ûµ±Ç°ÒÑ¾­ÓĞÒ»¸ö´íÎóÔÚ·¢Éú*/
    {
        display_led |=hFault_type;        /*Á½¸ö´íÎó*/
        display_led &=hFault_type;        /*Çå³ıÉÏÒ»¸ö´íÎó£¬ÏÔÊ¾µ±Ç°·¢ÉúµÄ´íÎó*/
    }
    else
        display_led  |= hFault_type;      /*Ö»±¨¾¯µ±Ç°´íÎóÀàĞÍ*/
}
/*------------------------------------------------
Function:Çå³ı´íÎó±êÖ¾
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MC_ClearFault(u32 hFault_type)
{
    wGlobal_Flags &= ~hFault_type;        /*Çå³ıÈ«¾Ö´íÎó*/
    display_led &= ~hFault_type;          /*Çå³ıµ±Ç°´íÎóÀàĞÍ*/

}
/*------------------------------------------------
Function:·Ö±ğ¼ì²âÁ½¸öµç»úµÄ´íÎóÎ»£¬½øĞĞÕûºÏÔÚwGlobal_Flags±äÁ¿µ±ÖĞ
Input   :No
Output  :No
Explain :ÕûºÏÖ®ºó¶ÁÈ¡Ò»¸ö±äÁ¿¾Í¿ÉÒÔ²é¿´ËùÓĞµÄ´íÎó£¬Ò²·½±ãÖ¸Ê¾µÆµÄÏÔÊ¾
------------------------------------------------*/
void Display_error(void)
{
    /*M1M2¹«¹²´íÎó*/
    if((STM[M1].hFaultOccurred&MC_OVER_VOLT)==MC_OVER_VOLT||(STM[M2].hFaultOccurred&MC_OVER_VOLT)==MC_OVER_VOLT)    /*Èç¹ûÁ½¸öµç»úµÄÆäÖĞÒ»¸ö¶ÀÁ¢´íÎó±»ÖÃÎ»*/
    {
        MC_SetFault(MC_OVER_VOLT);                                                                                  /*ÄÇÃ´¾ÍÖÃÎ»È«¾Ö±äÁ¿ÖĞµÄ´íÎóÖ¸Ê¾*/
    }
    else
    {
        MC_ClearFault(MC_OVER_VOLT);
    }
    if((STM[M1].hFaultOccurred&MC_UNDER_VOLT)==MC_UNDER_VOLT||(STM[M2].hFaultOccurred&MC_UNDER_VOLT)==MC_UNDER_VOLT)/*×¢ÊÍÍ¬ÉÏ*/
    {
        MC_SetFault(MC_UNDER_VOLT);
    }
    else
    {
        MC_ClearFault(MC_UNDER_VOLT);
    }
    if((STM[M1].hFaultOccurred&ERR_CAN_COMMUNICATION)==ERR_CAN_COMMUNICATION||(STM[M2].hFaultOccurred&ERR_CAN_COMMUNICATION)==ERR_CAN_COMMUNICATION)
    {
        MC_SetFault(ERR_CAN_COMMUNICATION);                                                                         /*ÄÇÃ´¾ÍÖÃÎ»È«¾Ö±äÁ¿ÖĞµÄ´íÎóÖ¸Ê¾*/
    }
    else
    {
        MC_ClearFault(ERR_CAN_COMMUNICATION);
    }
    /*Á½¸öµç»ú¸÷ÓĞµÄ´íÎóÊ¹ÓÃforÑ­»·À´ÅĞ¶Ï£¬Ô­ÀíÒ»ÑùÖ»ÊÇ´íÎóÊıÄ¿¶à*/
    for(u8 i =0; i<2; i++)
    {
        if((STM[i].hFaultOccurred&MC_BREAK_IN)==MC_BREAK_IN)    /*M1¶ÌÂ·´íÎó±»ÖÃÎ»*/
        {
            if(i==0)
            {
                if((wGlobal_Flags&MC_BREAK_IN_M1)==0)          /*Èç¹ûÈ«¾Ö´íÎóÖĞÃ»ÓĞ¸Ã´íÎó*/
                {
                    MC_SetFault(MC_BREAK_IN_M1);               /*ÄÇÃ´ÖÃÎ»È«¾Ö´íÎó±êÖ¾*/
                }
            }
            else
            {
                if((wGlobal_Flags&MC_BREAK_IN_M2)==0)          /*M2µç»ú*/
                {
                    MC_SetFault(MC_BREAK_IN_M2);
                }
            }
        }
        else                                                   /*Èç¹ûµç»ú¶ÀÁ¢´íÎó±»Çå³ı£¬ÔòÇå³ıÈ«¾Ö´íÎó*/
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
                ||(STM[i].hFaultOccurred&ERR_CURR_OVER_MAX)    ==ERR_CURR_OVER_MAX)    /*ËùÓĞ½×¶ÎµÄ¹ıÁ÷¶¼±¨¸æÍ¬Ò»¸ö¹ıÁ÷´íÎó*/

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
Function:Çı¶¯Æ÷´íÎóÖ¸Ê¾
Input   :No
Output  :No
Explain :10msÔËĞĞÒ»´Î£¬Ö´ĞĞÓëàÖàª¶¨Ê±Æ÷µ±ÖĞ,
------------------------------------------------*/
void DisplayErrorLed_Handle(void)
{
    static u8 DisplayGreenCnt;
    DisplayCounter++;                                     /*ÓÃÓÚºìµÆÉÁË¸ÆµÂÊ*/
    DisplayGreenCnt++;                                    /*ÓÃÓÚÂÌµÆÒ»Ö±ÉÁË¸*/

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
        case MC_OVER_VOLT:                                 /*Á½¸öºìµÆ³£Á¿*/
            RED_LED1_ON;
            RED_LED2_ON;
            break;
        case MC_UNDER_VOLT:                                /*Ç·Ñ¹ÏÂÃæºìµÆÁÁ*/
            RED_LED2_OFF;
            RED_LED1_ON;
            break;
        case ERR_CAN_COMMUNICATION:                        /*Á½¸öµÆ¿ìÉÁ1´Î*/
            LEDSet(LED_ON,LED_ON,1,0,LED_HIGH_SPEED);
            break;

        case ERR_CURR_OVER_MAX_M1:                         /*ÉÏÃæºìµÆ¿ìÉÁ2´Î*/
            LEDSet(LED_OFF,LED_ON,2,0,LED_HIGH_SPEED);
            break;
        case ERR_CURR_OVER_MAX_M2:                         /*ÏÂÃæºìµÆ¿ìÉÁ2´Î*/
            LEDSet(LED_ON,LED_OFF,2,0,LED_HIGH_SPEED);
            break;

        case ERR_OVER_VEL_ERR_M1:                          /*ÉÏÃæºìµÆ¿ìÉÁ3´Î*/
            LEDSet(LED_OFF,LED_ON,3,0,LED_HIGH_SPEED);
            break;
        case ERR_OVER_VEL_ERR_M2:                          /*ÏÂÃæºìµÆ¿ìÉÁ3´Î*/
            LEDSet(LED_ON,LED_OFF,3,0,LED_HIGH_SPEED);
            break;

        case ERR_MOTOR_STUCK_ERR_M1:                       /*ÉÏÃæºìµÆ¿ìÉÁ4´Î*/
            LEDSet(LED_OFF,LED_ON,4,0,LED_HIGH_SPEED);
            break;
        case ERR_MOTOR_STUCK_ERR_M2:                       /*ÏÂÃæºìµÆ¿ìÉÁ4´Î*/
            LEDSet(LED_ON,LED_OFF,4,0,LED_HIGH_SPEED);
            break;

        case MC_BREAK_IN_M1:                               /*ÉÏÃæºìµÆ¿ìÉÁ5´Î*/
            LEDSet(LED_OFF,LED_ON,5,0,LED_HIGH_SPEED);
            break;
        case MC_BREAK_IN_M2:                               /*ÏÂÃæºìµÆ¿ìÉÁ5´Î*/
            LEDSet(LED_ON,LED_OFF,5,0,LED_HIGH_SPEED);
            break;
        case ERR_LOST_PHASE_M1:                            /*ÉÏÃæºìµÆ¿ìÉÁ6´Î*/
            LEDSet(LED_OFF,LED_ON,6,0,LED_HIGH_SPEED);
            break;
        case ERR_LOST_PHASE_M2:                            /*ÏÂÃæºìµÆ¿ìÉÁ6´Î*/
            LEDSet(LED_ON,LED_OFF,6,0,LED_HIGH_SPEED);
            break;

        case ERR_HALL_M1:                                  /*ÉÏÃæºìµÆ¿ìÉÁ7´Î*/
            LEDSet(LED_OFF,LED_ON,7,0,LED_HIGH_SPEED);
            break;
        case ERR_HALL_M2:                                  /*ÏÂÃæºìµÆ¿ìÉÁ7´Î*/
            LEDSet(LED_ON,LED_OFF,7,0,LED_HIGH_SPEED);
            break;
        case MC_OVER_TEMP_M1:                              /*ÉÏÃæºìµÆ¿ìÉÁ8´Î*/
            LEDSet(LED_OFF,LED_ON,8,0,LED_HIGH_SPEED);
            break;
        case MC_OVER_TEMP_M2:                              /*ÏÂÃæºìµÆ¿ìÉÁ8´Î*/
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
            RED_LED2_OFF;//´Ë´¦ÉêÃ÷£º±ÈÈç·¢ÉúÁË²»¿É×Ô¶¯»Ö¸´µÄ´íÎó£¬ÔÙ·¢Éú¿É×Ô¶¯»Ö¸´µÄ´íÎó£¨±ÈÈçÇ·Ñ¹¹ıÑ¹£©£¬ÔÚ×Ô¶¯»Ö¸´´íÎóÖ®ºó£¬²»¿É»Ö¸´µÄ´íÎó½«²»ÏÔÊ¾ºìµÆ£¨±íÏÖÏÖÏóÎªºìµÆ¹Ø±Õ£¬ÂÌµÆ²»ÉÁË¸£©
            //2022.3.17½â¾öÉÏÃæËùËµbug£¬×Ô¶¯»Ö¸´Ö®ºó£¬ÅĞ¶ÏÈ«¾Ö´íÎó×îµÍÎ»µÄ´íÎó£¬½«×îµÍÎ»µÄ´íÎó¸³Öµ¸ødisplay_led
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
Function:²Î¿¼×¢Èë£¬ÓÃÓÚ×ÔÊÊÓ¦Î»ÖÃ¡¢ËÙ¶ÈºÍµçÁ÷¿ØÖÆ»ØÂ·
Input   :@injectPoint
         @injectType ÇúÏßÀàĞÍ
         @injectValue
         @injectFreq ÆµÂÊ
         @injectAmp  ¸³Öµ
         @injectInit
         @*refOut  Êä³ö¸øµ½µçÁ÷²Î¿¼»òÕßËÙ¶È²Î¿¼
Output  :No
Explain :ÓÃÓÚÉú³ÉSINÇúÏß¡¢·½²¨ÓÃÓÚ²âÊÔµçÁ÷»·ºÍËÙ¶È»·ÏìÓ¦£¬ĞŞ¸ÄÁË×îºóÒ»¸öÈë¿Ú²ÎÊıµÄÀàĞÍÓÉlong ¸ÄÎªint32_t,ÒòÎªÊäÈë²ÎÊı²Î¿¼µçÁ÷ÊÇint32_t·ñÔò»á±¨¸æ¾¯¸æ
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
        Vector_Components = MCM_Trig_Functions((int16_t)(*injectPhase));   /*time:530ns£¬math..ÖĞµÄ¿âº¯Êı*/
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

    case INJECT_TYPE_SQUARE_DIRECT:			/*¾ØĞÎ²¨*/
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
* @function :sinÇúÏß£¬»·Â·²âÊÔµçÁ÷»·ËÙ¶È»·ÏìÓ¦²ÎÊı³õÊ¼»¯
* @input    :ÇëÌîĞ´
* @output   :ÇëÌîĞ´
* @explain  :ReferenceInjection¸Ãº¯ÊıµÄ²ÎÊı
* @author   :Íü´¨
* @date     :2022/12/16
------------------------------------------------*/
void ReferenceInjectionLoopPar_Init(void)
{
    loopTestPar.InjectPoint[M1] = INJECT_POINT_CURRREF;
    loopTestPar.InjectType[M1] = INJECT_TYPE_SIN_DIRECT;  /*sinÇúÏß*/
    loopTestPar.InjectFreq[M1] = 3;
    loopTestPar.InjectCurrAmp[M1] = 1000;

    loopTestPar.InjectPoint[M2] = INJECT_POINT_CURRREF;
    loopTestPar.InjectType[M2] = INJECT_TYPE_SIN_DIRECT;  /*sinÇúÏß*/
    loopTestPar.InjectFreq[M2] = 3;
    loopTestPar.InjectCurrAmp[M2] = 1000;
}

/*------------------------------------------------
Function:×î´óÁ¦¾ØÏŞÖÆ
Input   :No
Output  :No
Explain :Î´Ê¹ÓÃ
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
Function:¶Â×ª¼ì²é
Input   :No
Output  :´íÎó´úÂë£º0x00001000
Explain :Ö´ĞĞÖÜÆÚ£º500us  ¶Â×ªÊ±¼ä2000ms  ¶Â×ªËÙ¶È 5RPM ¶Â×ªµçÁ÷£¨Iq£©
------------------------------------------------*/
uint32_t MotorStuckCheck(u8 axes)
{
    uint32_t errorcode = MC_NO_ERROR;
    if(1&&STM[axes].bState == RUN)           /*¶Â×ª¿ª¹Ø´ò¿ªµç»úÊ¹ÄÜ*/
    {
        if((in32abs(FOCVars[axes].Iqdref.q) > MotorParameters[axes].ShortCircuit)\
                && (in32abs(pCtrlPar[axes].Vel_PLL_Motor) <= MotorParameters[axes].LockedSpeed)) /*²Î¿¼µçÁ÷£¾¶Â×ªµçÁ÷£¬Êµ¼ÊËÙ¶È£¼¶Â×ªËÙ¶È*/
        {
            if(in32abs(FOCVars[axes].Iqd.q) > MotorParameters[axes].ShortCircuit/2)          /*²Î¿¼µçÁ÷£¾¶Â×ªµçÁ÷µÄÒ»°ë*/
            {
                giStuckTimeCnt[axes]++;
            }

            if(giStuckTimeCnt[axes] > MotorParameters[axes].LockeTicks)                      /* ¶Â×ªÊ±¼ä2000ms ±¨´í*/
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
Function:½»»»ÅÅĞò
Input   :No
Output  :No
Explain :ÎªÁËÈ±Ïà¼ì²âµ±ÖĞµÄµçÁ÷½øĞĞÅÅĞòÅÅĞòÖ®ºó½øĞĞÔËËã
------------------------------------------------*/
void swap(float *x,float *y)
{
    float temp= 0;
    temp=*x;
    *x=*y;
    *y=temp;
}
/*------------------------------------------------
* @function :±¾Ëã·¨½«a[]ÖĞµÄÔªËØ´ÓĞ¡µ½µ½´ó½øĞĞÅÅĞò
* @input    :@paramĞèÒªÅÅĞòµÄÊı×é @paramÊı×é´óĞ¡
* @output   :ÇëÌîĞ´
* @explain  :ÇëÌîĞ´
* @author   :Íü´¨
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
                swap(&a[j - 1],&a[j]);      /*Îª½»»»º¯Êı£¬½«a[j] Óë a[j - 1] ½øĞĞ½»»»*/
            }
            else
            {}
        }
    }
}
/*------------------------------------------------
Function:È±Ïà¼ì²âµÄ³õÊ¼»¯º¯Êı
Input   :No
Output  :No
Explain :·ÀÖ¹ÔÚµç»úÆôÍ£Ê±ºòµÄ²ÎÊıÎÉÂÒ£¬µ¼ÖÂµÄÈ±ÏàÎó±¨
------------------------------------------------*/
void ClearPhaseCheckPar(void)
{
    IabcCurrentTemp[M1][0] = 0;           /*µçÁ÷»º´æÇå³ı*/
    IabcCurrentTemp[M1][1] = 0;
    IabcCurrentTemp[M1][2] = 0;
    lostPhaseCheckCnt[M1] = 0;            /*µç»ú»º´æ¼ÆÆ÷ÊıÇå³ı*/
    Missing_phase_error_code[M1] = 0;     /*´íÎó´úÂë»º´æ*/
    cur_calculated_value[M1] = 0;         /*±È½ÏÖµÇå³ı*/
    iabcurrenttemporary[0] = 0;           /*ĞèÒªÅÅĞòµÄÈıÏàµçÁ÷ÀÛ¼ÓºÍÇå³ı*/
    iabcurrenttemporary[1] = 0;
    iabcurrenttemporary[2] = 0;


    IabcCurrentTemp[M2][0] = 0;           /*M2µç»úÇå³ı*/
    IabcCurrentTemp[M2][1] = 0;
    IabcCurrentTemp[M2][2] = 0;
    lostPhaseCheckCnt[M2] = 0;
    Missing_phase_error_code[M2] = 0;
    cur_calculated_value[M1] = 0;
    iabcurrenttemporary[0] = 0;
    iabcurrenttemporary[1] = 0;
    iabcurrenttemporary[2] = 0;
}/*------------------------------------------------
Function£ºÈ±Ïà¼ì²âµÄ´íÎó±¨¾¯º¯Êı
Input   :No
Output  :No
Explain :¸Ãº¯ÊıÊÇÎªÁË½«ADCÖĞ¶Ïµ±ÖĞµÄÈ±ÏàÅĞ¶ÏÓë500usµ±ÖĞµÄ¹ÊÕÏ´¦Àíº¯ÊıÁªÏµÔÚÒ»Æğ
------------------------------------------------*/
uint32_t DefaultPhaseCheckLinkFunction(u8 axes)
{
    uint32_t temp = 0;
    temp = Missing_phase_error_code[axes];
    Missing_phase_error_code[axes] = 0;      /*ÎÊÌâÆğÒò£ºÓÉÓÚÈ±Ïà¼ì²âÊÇÔÚRUN×´Ì¬ÖĞ¼ÆËã£¬ÔÚ±¨´íÖ®ºóµç»ú²»»áÔÚ½øĞĞ¼ì²âËùÒÔ±¨´íÖ®ºóÏÔÊ¾FAULT_NOW*/
    return temp;
}
/*------------------------------------------------
* @function :int32ÀàĞÍÊı¾İÇó¾ø¶ÔÖµ
* @input    :ÇëÌîĞ´
* @output   :ÇëÌîĞ´
* @explain  :ÇëÌîĞ´
* @author   :Íü´¨
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
* @function :longÀàĞÍÊı¾İÇó¾ø¶ÔÖµ
* @input    :ÇëÌîĞ´
* @output   :ÇëÌîĞ´
* @explain  :ÇëÌîĞ´
* @author   :Íü´¨
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
Function:È±Ïà¼ì²âº¯Êı
Input   :No
Output  :No
Explain :50usÔËĞĞÒ»´Î
------------------------------------------------*/
uint32_t DefaultPhaseCheck(u8 axes)
{
//    static uint32_t  giPhaseTimeCnt[NUMBER_OF_AXES] = {0,0};
    uint32_t errorcode = MC_NO_ERROR;
    if(STM[axes].bState == RUN&&pCtrlPar[axes].SetVelMotor!=0\
            &&l_abs(FOCVars[axes].Iqdref.q) > l_abs(pPIDSpeed[axes]->hUpperOutputLimit-1))   /*¡¾Æô¶¯Ê±¾ÍÒÑ¾­È±Ïà¡¿,²Î¿¼µçÁ÷ºÜ´ó*/
    {
        if(in32abs(FOCVars[axes].Iqd.q)<100&&pCtrlPar[axes].Vel_PLL_Motor<50)           /*µç»úÃ»µçÁ÷£¨Iq<100£©£¬ËÙ¶ÈĞ¡ÓÚ5RPM£¬²»¹ıÁ÷*/
        {
            lostPhaseCheckCntForLaunch[axes]++;                                         /*È±Ïà¼ì²â¼ÆÊı1*/
        }
        else
        {
            if(lostPhaseCheckCntForLaunch[axes]>0)
            {
                lostPhaseCheckCntForLaunch[axes]--;
            }
        }
        if(lostPhaseCheckCntForLaunch[axes]>(uint16_t)(MISSING_PAHSE_JUDGE_TIME/3))                   /*³ÖĞø300ms´íÎó±¨´í£¬²¢ÇÒÇå³ı´íÎó¼ÆÊı ´Ë´¦ÓëÏÂÃæ1S ĞèÒª²»Í¬£¬
					                                                            ´Ë´¦ÔÚÕû»úÉÏ³öÏÖ¹ıÒ»´Î£¬Èç¹ûÒ»¸öÂÖ×ÓÃ»²å£¬ÁíÒ»¸öÂÖ×Ó»á´ø¶¯Òì³£ÂÖ×Ó×ª¶¯£¬
				                                                              Èç¹ûÊ±¼ä¹ı³¤¿ÉÄÜÕâ¸ö´íÎóºÜÄÑ±¨³öÀ´*/
        {
            errorcode = ERR_LOST_PHASE;
            lostPhaseCheckCntForLaunch[axes] = 0;                                       /*Çå³ı´íÎó¼ÆÊı */
        }
    }
    else
    {
        if(lostPhaseCheckCntForLaunch[axes]>0)
        {
            lostPhaseCheckCntForLaunch[axes]--;
        }
    }
    if(1&&STM[axes].bState == RUN && abs(pCtrlPar[axes].SetVelMotor)>100)              /*¡¾ÔËĞĞ¹ı³ÌÖĞÈ±Ïà¡¿  Èç¹û²»¼ÓËÙ¶ÈÏŞÖÆ£¬Í£»úqidongµÄÊ±ºò»áÎó±¨È±Ïà£¬´Ë´¦¶Ô×®ÏŞÖÆµçÁ÷²»Ó°ÏìÈ±Ïà*/
    {
        lostPhaseCheckCnt[axes]++;                                                     /*È±Ïà¼ì²â¼ÆÊı2*/
        if(lostPhaseCheckCnt[axes] > MISSING_PAHSE_JUDGE_TIME)                         /*È±Ïà¼ì²âÒ»¸ö²âÊÔÖÜÆÚÎª1000ms*/
        {
            for(u8 i =0; i<3; i++)
            {
                iabcurrenttemporary[i] = IabcCurrentTemp[axes][i];                     /*½«´íÎó·¢ÉúÊ±µÄÊı¾İ¼ÇÂ¼£¬²âÊÔÊ¹ÓÃ*/
            }
            BubbleSort(iabcurrenttemporary,3);
            cur_calculated_value[axes] = (iabcurrenttemporary[2] - iabcurrenttemporary[0])/iabcurrenttemporary[2];  /*×î´óÖµ¼õÈ¥×îĞ¡Öµ³ıÒÔ×î´óÖµ£¬¸ÃÖµ·¶Î§0-1 £¬Õı³£ÔËĞĞÖµÔ¼Îª0.0¼¸£¬Òì³£Ô½½Ó½ü1*/
            if(cur_calculated_value[axes] > 0.8f)                                      /*ÒÑ¾­·¢Éú´íÎó0.8Îª¾­ÑéÖµ*/
            {
                errorcode = ERR_LOST_PHASE;
                IabcCurrentTemp[axes][0] = 0;
                IabcCurrentTemp[axes][1] = 0;
                IabcCurrentTemp[axes][2] = 0;
                lostPhaseCheckCnt[axes] = 0;
            }
            else //if(lostPhaseCheckCnt[axes] > 6000)                                  /*Ã»ÓĞ·¢Éú´íÎó²¢ÇÒÒ»¸ö²âÊÔÖÜÆÚ½áÊø£¬Ò»¸ö²âÊÔÖÜÆÚÎª300ms*/
            {
                IabcCurrentTemp[axes][0] = 0;
                IabcCurrentTemp[axes][1] = 0;
                IabcCurrentTemp[axes][2] = 0;
                lostPhaseCheckCnt[axes] = 0;
            }
        }
        else                                                                           /*ÔÚÒ»¸ö²âÊÔÖÜÆÚÄÚ£¬½øĞĞµçÁ÷Êı¾İÀÛ¼Ó*/
        {
            if(FOCVars[axes].Iab.a>0)                                                  /*ÏàµçÁ÷Îªsin*/
            {
                IabcCurrentTemp[axes][0] += FOCVars[axes].Iab.a;
            }
            else
            {
                IabcCurrentTemp[axes][0] += (-FOCVars[axes].Iab.a);
            }
            if(FOCVars[axes].Iab.b>0)                                                 /*ÏàµçÁ÷Îªsin*/
            {
                IabcCurrentTemp[axes][1] += FOCVars[axes].Iab.b;
            }
            else                                                                      /*Èç¹ûIbĞ¡ÓÚ0*/
            {
                IabcCurrentTemp[axes][1] += (-FOCVars[axes].Iab.b);
            }
            if(((FOCVars[axes].Iab.a+FOCVars[axes].Iab.b)*-1)>0)                      /*Èç¹û Ic > 0*/
            {
                IabcCurrentTemp[axes][2] += (-1)*(FOCVars[axes].Iab.a+FOCVars[axes].Iab.b);
            }
            else                                                                      /*Èç¹û Ic < 0*/
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
Function:Ê§ËÙ¼ì²é
Input   :No
Output  :´íÎó´úÂë£ºERR_OVER_VEL_ERR 0x00010000
Explain :Ö´ĞĞÖÜÆÚ£º500us ÎÊÌâ£º´æÔÚBUGµç»úËÙ¶È0ËÙ¶ÈÊ§ÄÜÖ®ºóĞı×ªµç»ú,±¨´íÖ®ºóÖ±½ÓÊ¹ÄÜµç»ú¿ÉÄÜ»Ø½øÈëÈí¼ş´íÎó
ÔÙÊ§ÄÜ£¬Î»ÖÃ»·ËøÖáÔÚÖ®Ç°µÄÎ»ÖÃ
MotorParameters[axes].MAXSpeed¸Ã±äÁ¿Îª×îÖÕÉèÖÃµÄÈí¼ş×î´óËÙ¶ÈÏŞÖÆ£¬¸ÃÖµĞèÒª°üº¬ÓÉÓÚPIDµ÷½ÚµÄ¹ı³å
------------------------------------------------*/
uint32_t StallCheck(u8 axes)
{
    int32_t veltemp = 0;
    uint32_t errorcode = MC_NO_ERROR;
    uint16_t MAX_giStallTimeCnt[NUMBER_OF_AXES] = {200,200};       /*100ms±¨´í*/
//  int32_t  SetVel_Multiply_VelPLL = 0;
    if(1&&STM[axes].bState == RUN)                                 /*Ê§ËÙ±£»¤¿ª¹Ø´ò¿ª&&µç»úÊ¹ÄÜ*/
    {
        veltemp = in32abs(pCtrlPar[axes].Vel_PLL_Motor);
        if(veltemp > MotorParameters[axes].MAXSpeed)               /*³¬¹ı×î´óËÙ¶È±£»¤Ö®*/
        {
            giStallTimeCnt[axes]++;
            if(giStallTimeCnt[axes]>MAX_giStallTimeCnt[axes])      /*100ms±¨´í*/
            {
                errorcode = ERR_OVER_VEL_ERR;
                giStallTimeCnt[axes] = 0;
                return errorcode;
            }
        }
        else if(giStallTimeCnt[axes]>0)                            /*´¦ÓÚÕı³£×ªËÙ·¶Î§*/
        {
            giStallTimeCnt[axes]--;
        }
    }
    else
    {
        giStallTimeCnt[axes] = 0;                                   /*µç»úÎ´Ê¹ÄÜÖ±½ÓÇåÁã*/
    }
    return errorcode;
}
/*------------------------------------------------
Function:MCU°æ±¾ºÅ
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void Program_Version_Init(MCU_Version *pVersion,u8 robotype,u8 main_ver,u8 fun_ver,u8 small_ver,u32 hardware_version)
{
    pVersion->uVersionPartOfRobotType = robotype;				              /*ÏîÄ¿ÀàĞÍ	SÏß*/
    pVersion->uVersionPartOfMainVersion = main_ver;			              /*´ó°æ±¾ºÅ*/
    pVersion->uVersionPartOfFunVersion = fun_ver;						          /*¹¦ÄÜ°æ±¾ºÅ*/
    pVersion->uVersionPartOfSmallVersion = small_ver;		              /*Ğ¡°æ±¾ºÅ£¬´úÂëbugÒÔ¼°²ÎÊıÎ¢µ÷*/
    pVersion->uVersionFullVersion = (robotype<<24)+(main_ver<<16)+(fun_ver<<8)+small_ver;
}
/*------------------------------------------------
Function:Ó²¼ş°æ±¾ºÅ
Input   :&MCU_Version ,robotype,main_ver, fun_ver,small_ver
Output  :No
Explain :No
------------------------------------------------*/
void HardVersion_Init(MCU_Version *pVersion,u8 funtype,u8 vol,u8 cur_max,u8 update_ver)
{
    pVersion->uHardwareVersion = (funtype<<24)+(vol<<16)+(cur_max<<8)+update_ver;
}
/*------------------------------------------------
Function:ºÏ²¢Á½¸öµç»úÊµ¼Ê×ªËÙ
Input   :No
Output  :No
Explain :µÍ16Î»ÎªM1µç»ú£¬¸ß16Î»ÎªM2µç»ú
------------------------------------------------*/
void SpeedRelMerge_two_to_one(SEVERO_CONTORL_PAR_T *parM1,SEVERO_CONTORL_PAR_T *parM2)
{
    int32_t Veltemp1,Veltemp2;

    Veltemp1 = parM2->Vel_PLL_Motor&0x0000FFFF;        /*ÄÃ³öµÍ16Î»*/
    Veltemp2 = Veltemp1<<16;
    Veltemp1 = parM1->Vel_PLL_Motor&0x0000FFFF;
    parM1->M1M2_VelRel = Veltemp1|Veltemp2;             /*M1ÎªµÍ16Î»£¬M2Î»¸ß16Î»*/
}
/*------------------------------------------------
Function:²ğ·ÖÁ½¸öµç»úÉèÖÃ×ªËÙ
Input   :No
Output  :No
Explain :µÍ16Î»ÎªM1µç»ú£¬¸ß16Î»ÎªM2µç»ú
------------------------------------------------*/
void SpeedSetMerge_two_to_one(SEVERO_CONTORL_PAR_T *parM1,SEVERO_CONTORL_PAR_T *parM2,uint8_t mode)
{
    int16_t m1veltemp,m2veltemp;

    m1veltemp = parM1->M1M2_VelSet>>16;                  /*È¡¸ß16Î»*/
    m2veltemp = parM1->M1M2_VelSet&0X0000FFFF;           /*È¡µÍ16Î»*/
    if(mode == 3)                                        /*ËÙ¶ÈÄ£Ê½*/
    {
        parM1->SetTorqueMotor = 0;                       /*ËÙ¶ÈÄ£Ê½ÏÂ½«Á¦¾ØÉèÖÃÎª0*/
        parM2->SetTorqueMotor = 0;
        parM1->SetVelMotor = -m2veltemp;                 /*µÍ16Î»¸øM1,ĞèÒª¸³Öµ¸ø16Î»µÄ±äÁ¿£¬ÔÙ½«¸Ã±äÁ¿¸³Öµ¸ø32Î»*/
        parM2->SetVelMotor = -m1veltemp;                 /*¸ß16Î»¸øM2£¬·ñÔòÔÚ¸ø-ËÙ¶È´æÔÚÎÊÌâ  ´Ë´¦È¡·´ÎªÁËÊÊÓ¦ÖĞÁâµç»ú*/
    }
    else if(mode == 4)                                   /*Á¦¾ØÄ£Ê½*/
    {
        parM1->SetVelMotor = 0;                          /*Á¦¾ØÄ£Ê½ÏÂ½«ËÙ¶ÈÉèÖÃÎª0*/
        parM2->SetVelMotor = 0;
        parM1->SetTorqueMotor = m2veltemp;               /*µÍ16Î»¸øM1*/
        parM2->SetTorqueMotor = m1veltemp;               /*¸ß16Î»¸øM2*/
    }
}
/*------------------------------------------------
Function:»ô¶û³ö´í¼ì²éÒÔ¼°±àÂëÆ÷¼ì²é
Input   :No
Output  :No
Explain :ÎªÁË·ÀÖ¹Îó±¨£¬×îºóĞŞ¸ÄÎª2s¼ì²âÖÜÆÚ
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
        hall_error_cnt[axes]++;                     /*500usÒ»´Î*/
        if(hall_error_cnt[axes] > 4000)             /*2sÈ·ÈÏÎª»ô¶û´íÎó*/
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
Function:µç»úÎÂ¶È±£»¤
Input   :No
Output  :No
Explain :500usÒ»´Î Ä¬ÈÏ120¶È 30s±£»¤
------------------------------------------------*/
uint32_t Motor_Over_Temperature(u8 axes)
{
    uint32_t errorcode = 0;
    if(pCtrlPar[axes].MotorTemperature > MotorParameters[axes].MaxTemperature && STM[axes].bState == RUN)   /*120¶È±£»¤*/
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
    if(temperature_cnt[axes] > MotorParameters[axes].OverTemperatureTicks)                                  /*30s±£»¤*/
    {
        temperature_cnt[axes] = 0;
        errorcode = MC_OVER_TEMP;
    }
    return errorcode;
}
/*------------------------------------------------
Function:µç»úÊ¹ÄÜº¯Êı
Input   :¼±Í£°´¼ü
Output  :No
Explain :ÓÃÓÚÔÚ¼±Í£ĞÅºÅ°´ÏÂÒÔºóµ÷ÓÃ£¬ÆäÓàµØ·½²»µ÷ÓÃ£¬Í¬²½Ä£Ê½ÏÂÊäÈë²ÎÊıÎªDrive_Data[M1]¹²Í¬¿ØÖÆÁ½¸öµç»ú,Ö»¿¼ÂÇÍ¬²½Ä£Ê½ÏÂ
------------------------------------------------*/
uint8_t Motor_Enable(Drive_Data *pDriver_Data)
{
    static uint8_t EnableStep[2] = {0,0};                /*Á½¸öµç»ú*/
    static uint16_t Statusword = 0;
    Statusword = pDriver_Data->device_control_data.Statusword;
    if((Statusword & 0x006F) == 0x0027)                 /*µç»úÒÑ¾­Ê¹ÄÜ*/
    {
        EnableStep[0] = 0;                              /*Ê¹ÄÜÒÔºó½«²½ÖèÇåÁã*/
        return 1;//µç»úÒÑ¾­Ê¹ÄÜ×´Ì¬
    }
    else
    {
        if(EnableStep[0] ==0)
        {
            pCtrlPar[M1].SetVelMotor = 0;               /*Çå¿ÕËÙ¶È²Î¿¼*/
            pCtrlPar[M2].SetVelMotor = 0;
            pCtrlPar[M1].M1M2_VelSet = 0;
            EnableStep[0] = 1;
        }
        else if((Statusword & 0x006F) == 0x0021)        /*¿ØÖÆ×Ö6·¢ËÍÍê³É£¬×´Ì¬×ÖÇĞ»»Îª0x0231*/
        {
            EnableStep[0] = 2;
        }
        else if((Statusword & 0x006F) == 0x0023)        /*¿ØÖÆ×Ö7·¢ËÍÍê³É£¬×´Ì¬×ÖÇĞ»»Îª0x0233*/
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
Function:¼±Í£¼ì²â¹¦ÄÜ
Input   :No
Output  :No
Explain :°´ÏÂÎª0£¬Õı³£Îª1
------------------------------------------------*/
void Emrgency_Stop(void)
{
    EMG_State3 = (Start_Stop_GPIO_Port->IDR>>0)&(0x01);
    if(EMG_State3 != EMG_State4)                                                 /*±£Ö¤Ã¿´Î×´Ì¬¸Ä±äÖ»½øĞĞÒ»´Î²Ù×÷£¬Ò»Ö±Ö´ĞĞ»áÓ°ÏìCAN¿ÚÆäËûÉè±¸¿ØÖÆ*/
    {
        if(EMG_State3 == STOPISON)                                               /*¼±Í£±»°´ÏÂ*/
        {
            if(Motor_Enable(Driver_Data) == TRUE)
            {
                Driver_Data[M1].device_control_data.Quick_stop_option_code = 6;  /*°´ÕÕ6085 ÉèÖÃµÄ¼±Í£¼õËÙ¶ÈÍ£»ú£¬Í£»úºóËøÖá*/
                Driver_Data[M1].device_control_data.Conrtolword = 0x0002;        /*¿ìËÙÍ£»úÃüÁî*/
//                P_V_I_Switch = 2;                                              /*ÓÃÓÚ´¦ÀíÎ»ÖÃ»·¼±Í£ĞÅºÅ£¬¹«Ë¾ÂÖì±²»ÔËĞĞÎ»ÖÃ»·ËùÒÔ´Ë´¦´Ö´¦Àí£¬Ö÷ÒªÓÃÓÚ¼ÇÂ¼£¬Î´²âÊÔÎ»ÖÃ»·¼±Í£¡£*/
                EMG_State4 = EMG_State3;                                         /* EMG_State3ºãµÈÓÚSTOPISON*/
            }
            else
            {
                /*Ê¹ÄÜÊ§°Ü*/

            }
        }
        else    /*¼±Í£Ã»ÓĞ±»°´ÏÂ*/
        {

            Driver_Data[M1].device_control_data.Quick_stop_option_code = 1;      /*°´ÕÕ6084 ÉèÖÃµÄ¼õËÙ¶ÈÍ£»ú£¬ Í£»úºó×ÔÓÉ*/
            Driver_Data[M1].device_control_data.Conrtolword = 0x0002;            /*¿ìËÙÍ£»ú*/
            EMG_State4 = EMG_State3;                                             /*EMG_State3ºãµÈÓÚ STOPISOFF*/

        }
    }
    else
    {
        /*Èç¹ûÒ»Ö±´¦ÓÚ°´ÏÂ»òÕß·Ç¶Ï¿ª£¬²»½øĞĞ²Ù×÷*/
    }

}
/*------------------------------------------------
* @function :µç»ú2³õÊ¼½Ç¶È¶¨Î»
* @input    :»ô¶û2×´Ì¬
* @output   :µç»ú³õÊ¼Î»ÖÃ
* @explain  :¸ù¾İ»ô¶ûÉÈÇø½øĞĞ³õÊ¼´Ö¶¨Î»£¬¶¨Î»ÎªÃ¿¸öÉÈÇøµÄµç½Ç¶ÈÎª30¶ÈµÄÎ»ÖÃ
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
    if(initial2_positioning_timing > 5)                        /*5ms¶ÁÈ¡5´ÎÈç¹û»ô¶û»¹ÊÇ´íÎóµÄ*/
    {
        initial2_positioning_timing = 6;                       /*´Ë´¦¿¼ÂÇ±¨¸æÊ²Ã´´íÎó*/
    }
    if(HallState2 != 0 && HallState2 != 7)                     /*Õı³£*/
    {
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);              /*½«¿ªÆôÖĞ¶Ï·¢ÔÚ´Ë´¦Ä¿µÄÊÇÎªÁË·ÀÖ¹»úÆ÷ÉÏµçÍÆ¶¯µ¼ÖÂ³õÊ¼¶¨Î»Ê§°ÜÎÊÌâ*/
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        if(motor2_exit_clear_flag == 0)                        /*¾²Ì¬±äÁ¿£¬³õÊ¼¶¨Î»Íê³ÉÒÔºóÖ»¶¨Î»Ò»´Î*/
        {
            motor2_exit_clear_flag++;
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_U_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_V_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M2_HALL_W_Pin);
        }
    }
}
/*------------------------------------------------
* @function :µç»ú1³õÊ¼½Ç¶È¶¨Î»
* @input    :»ô¶û1×´Ì¬
* @output   :µç»ú³õÊ¼Î»ÖÃ
* @explain  :¸ù¾İ»ô¶ûÉÈÇø½øĞĞ³õÊ¼´Ö¶¨Î»£¬¶¨Î»ÎªÃ¿¸öÉÈÇøµÄµç½Ç¶ÈÎª30¶ÈµÄÎ»ÖÃ
* @author   :Íü´¨
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
    if(initial_positioning_timing > 5)                           /*5ms¶ÁÈ¡5´ÎÈç¹û»ô¶û»¹ÊÇ´íÎóµÄ*/
    {
        initial_positioning_timing = 6;                          /*´Ë´¦¿¼ÂÇ±¨¸æÊ²Ã´´íÎó*/
    }
    if(HallState1 != 0 && HallState1 != 7)                       /*Õı³£*/
    {
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);              /*½«¿ªÆôÖĞ¶Ï·¢ÔÚ´Ë´¦Ä¿µÄÊÇÎªÁË·ÀÖ¹»úÆ÷ÉÏµçÍÆ¶¯µ¼ÖÂ³õÊ¼¶¨Î»Ê§°ÜÎÊÌâ*/
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        if(motor1_exit_clear_flag == 0)                          /*¾²Ì¬±äÁ¿£¬³õÊ¼¶¨Î»Íê³ÉÒÔºóÖ»¶¨Î»Ò»´Î*/
        {
            motor1_exit_clear_flag++;
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_U_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_V_Pin);
            __HAL_GPIO_EXTI_CLEAR_IT(M1_HALL_W_Pin);
        }
    }
}
/*------------------------------------------------
* @function :»ñÈ¡µç»ú2»ô¶ûĞÅºÅ
* @input    :Íâ²¿µçÆ½
* @output   :123456
* @explain  :ÇëÌîĞ´
* @author   :ÇëÌîĞ´
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
    return (u8)(tmp & 0x0007); //È¡µÍÈıÎ»
}
/*------------------------------------------------
* @function :»ñÈ¡µç»ú1»ô¶ûĞÅºÅ
* @input    :Íâ²¿µçÆ½
* @output   :123456
* @explain  :UVWÎªÊ²Ã´²»°´ÕÕË³ĞòÀ´Ğ´£¬ÊÇÊ×Åúµç»úUVWÏßĞòÎÊÌâ£¬
             °´ÕÕÕâ¸ö»ô¶ûµ÷Í¨ÒÔºóÒ»Ö±ÑÓĞøÖÁ½ñ
* @author   :Íü´¨
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

    return (u8)(tmp & 0x0007); //È¡µÍÈıÎ»
}
/*------------------------------------------------
* @function :Çå³ıBrakeÂË²¨¼ÆÊı
* @input    :ÇëÌîĞ´
* @output   :ÇëÌîĞ´
* @explain  :100msµ÷ÓÃÒ»´Î
* @author   :Íü´¨
* @date     :2022/12/16
------------------------------------------------*/
void Clear_Brake_Filters_Cnt(void)
{
    BrakeFiltersCnt[0] = 0;
    BrakeFiltersCnt[1] = 0;
}
/*------------------------------------------------
* @function :×èÄáÄ£Ê½¹¦ÄÜ
* @input    :
* @output   :
* @explain  :±¨´íÖ®ºó²¢ÇÒÊÕµ½Õı³£Í¨ĞÅÖ®ºó´ò¿ª×èÄáÄ£Ê½
             damping_mode_flagÔÚÊÕµ½CANÖ¸ÁîÒÔºóÖÃÎª1£¬·ÀÖ¹Î´ÉÏµçÍÆ¶¯»úÆ÷±¨¸æ¹ıÑ¹´ò¿ª×èÄáÄ£Ê½
             È¥µôÇ·Ñ¹´íÎóÏÂ´ò¿ª×èÄáÄ£Ê½
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void Damping_Of_Motor(void)
{
    /*Èç¹û³ö´í×Ô¶¯´ò¿ª×èÄáÄ£Ê½*/
    if((wGlobal_Flags != 0)&&(damping_mode_flag == 1)&&((wGlobal_Flags&MC_UNDER_VOLT) == 0))
    {
        Damping_Mode_ON_OFF = DAMPING_ON;
    }
    /*Èç¹û´íÎó±»Çå³ı×Ô¶¯¹Ø±Õ×èÄáÄ£Ê½*/
    if(wGlobal_Flags == 0)                             /*Èç¹û´íÎó±»Çå³ıÔòÍË³ö×èÄáÄ£Ê½[027°æ±¾ĞŞ¸Ä]*/
    {
        Damping_Mode_ON_OFF = DAMPING_OFF;             /*¹Ø±Õ×èÄáÄ£Ê½*/
    }
    if(Damping_Mode_ON_OFF == DAMPING_ON&&STM[M1].bState != RUN && STM[M2].bState != RUN)
    {
        /*´ò¿ªÈı¸öÏÂ¹Ü*/
        TIM1->CR2 = TURN_ON_THREE_LOW;
        TIM8->CR2 = TURN_ON_THREE_LOW;
        Damping_Mode_ON_OFF = STATE_DAMPING_IS_ON;     /*Èı¸öÏÂ¹ÜµÄ×´Ì¬Îª´ò¿ª*/
    }
    else if (Damping_Mode_ON_OFF == DAMPING_OFF)       /*¹Ø±Õ×èÄáÄ£Ê½*/
    {
        /*¹Ø±ÕÈı¸öÏÂ¹Ü*/
        TIM1->CR2  = TURN_OFF_THREE_LOW;
        TIM8->CR2  = TURN_OFF_THREE_LOW;
        Damping_Mode_ON_OFF = STATE_DAMPING_IS_OFF;    /*Èı¸öÏÂ¹ÜµÄ×´Ì¬Îª´ò¹Ø±Õ*/
    }

}
/*------------------------------------------------
* @function :ÉÏµç×ÔÆô¶¯º¯Êı
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
* @function :M1M2µç»ú³õÊ¼Î»ÖÃÑ§Ï°
* @input    :
* @output   :
* @explain  :×îÖÕÎªÁËµÃµ½ HALL_PHASE_SHIFT HallStudyFlag1²ÎÊıĞ´1£ºÖ®Ç°Ó¦¸ÃÊ¹ÄÜµç»ú£¬¿ÕÔØĞ´Èë1ÒÔºóµç»ú£¬´Ó¹Ì¶¨Î»ÖÃÆô¶¯Ğı×ªÒ»ÖÜ×Ô¶¯Ê§ÄÜ¡£
             2£º½«µç»úÍÏ¶¯µ½µç½Ç¶ÈÎª0µÄÎ»ÖÃ
* @author   :wangchuan
* @date     :2022/12/30
------------------------------------------------*/
void M1M2_Initial_Position(void)
{
    if(HallStudyFlag1 == 1)                                                   /*M1»ô¶ûÑ§Ï°Ñ°ÕÒ³õÊ¼Î»ÖÃÆ«²î*/
    {
        CNT_1MS[M1] = 0;
        if(HallStudyCNT < Hall_EAC_Ticks)                                     /*¼ÆÊ±3Ãë*/
        {
            Angle_Switch = 0;                                                 /*FOCÖĞ½«µç»úÍÏ¶¯µ½Ò»¸öµç½Ç¶ÈÎª0µÄÎ»ÖÃ*/
        }
        else if(HallStudyCNT < 3*Hall_EAC_Ticks)                              /*¼ÆÊ±6Ãë£¬ºÏ¼Æ£º9Ãë£¬ÒÑ¾­¹ıÁË3Ãë*/
        {
            if(Hall_AvElAngle1CNT >=MotorParameters[M1].PolePairNum)          /*×ª¹ı1RPM*/
            {
                if(Hall_AvElAngle1CNT == MotorParameters[M1].PolePairNum)     /*×ª¹ı1RPM*/
                {
                    Hall_AvElAngle1 = (Hall_AvElAngle1Sum/Hall_AvElAngle1CNT);/*Æ½¾ùÃ¿¸öÖÜÆÚµÄµç½Ç¶È*/
                    Hall_AvElAngle1Temp = (u16)Hall_AvElAngle1;
                    HALL_M1.PhaseShift = Hall_AvElAngle1;
                    HALL_PHASE_SHIFT = (s16)((u32)Hall_AvElAngle1Temp);       /*µÃµ½³õÊ¼Î»ÖÃ*/
                }
                Hall_AvElAngle1CNT = MotorParameters[M1].PolePairNum+1;
                Hall_AvElAngle1Sum =0;
//                    HALL_CC_First = 2;
                pCtrlPar[M1].SetVelMotor = 0;
                Angle_Switch = 0;                                              /*Ñ§Ï°Íê³ÉÇĞ»»×´Ì¬*/
                MC_StopMotor1();

            }
            else
            {
                Angle_Switch = 2;                                              /*FOCÖĞAngle_Switch=2ÎªÍÏ¶¯µç»úĞı×ª*/
            }
        }
        else
        {
            HallStudyFlag1 = 0;                                                /*½«Ñ§Ï°ÖĞÓÃµ½µÄ²ÎÊıÇå³ı*/
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
    else if(HallStudyFlag1 == 2)                                               /*½«µç»úÍÏ¶¯µ½µç½Ç¶È0Î»*/
    {
        MC_StartMotor1();                                                      /*Ê¹ÄÜµç»ú*/
        CNT_1MS[M1] = 0;
        if(HallStudyCNT < Hall_EAC_Ticks)                                      /*¶¨Ê±3Ãë*/
        {
            Angle_Switch = 0;                                                  /*FOCÖĞ¸ø¶¨Vq*/
        }
        else
        {
            MC_StopMotor1();                                                   /*Ê§ÄÜµç»ú*/
            HallStudyFlag1 = 0;                                                /*ÍÏ¶¯Íê³É*/
            HallStudyCNT = 0;
            Angle_Switch = 3;
            ENCODER_M1._Super.wPulseNumber	= 0;                               /*µç»úÊµ¼ÊÎ»ÖÃÇå0*/
            CurrentPosition1 =  MC_GetCurrentPosition1();
            ENCODER_M1.Angle_Compensation = 0 ;                                /*½Ç¶È²¹³¥ÖµÇå0*/
            HALL_CC_First = 2;

        }
        HALL_CH1_ValueOffset = HALL_CH1_ValueDelta1;
    }
    else
    {
        HallStudyCNT = 0;
    }


    if(HallStudyFlag2 == 1)                                                     /*M1»ô¶ûÑ§Ï°Ñ°ÕÒ³õÊ¼Î»ÖÃÆ«²î*/
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

