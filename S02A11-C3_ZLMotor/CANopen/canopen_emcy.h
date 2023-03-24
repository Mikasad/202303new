#ifndef __CANOPEN_EMCY_H
#define __CANOPEN_EMCY_H
#include "canopen_od.h"
#include "canopen_nmt.h"
#include "bsp_can.h"

/* Macro Definition */

/*CANopen Error Code(EMCY) - DS301*/
//具体参见CANopen402 P19
#define OC1_ERR   0x2310
#define OC2_ERR   0x2310
#define OC3_ERR   0x2310
#define OC0_ERR   0x2310
#define OV1_ERR   0x3210
#define OV2_ERR   0x3210
#define OV3_ERR   0x3210
#define OV0_ERR   0x3210
#define THT_ERR   0x4300
#define THN_ERR   0x2310
#define NTC_ERR   0x4310
#define EEP_ERR   0x5530
#define PID_ERR   0x7300
#define OPT_ERR   0x7500
#define OLS_ERR   0x8311
#define OL2_ERR   0x8311
#define BE_ERR    0x7110
#define OHT_ERR   0x7120
#define SCP_ERR   0x2310
#define CPU_ERR   0x6000
//DS402 ERRCODE
#define DC_UNDER_VOL_ERR  0x3220
#define MOTOR_OVERLOAD    0x3230
#define HRD_DEFECT_ERR    0x5280
#define MOTOR_STUCK_ERR      0x7121
#define POS_SENSOR_BREACH_ERR 0x7380
#define HEARTBEAT_TIMEOUT_ERR 0x8130
#define TURN_TO_INIT_INENABLE_ERR 0x8160
#define TURN_TO_STOP_INENABLE_ERR 0x8170
#define PDO_TRANS_LEN_ERR         0x8210

#define VEL_125_OVERLOAD_ERR 0x8400
#define FLOWING_ERR       0x8611 //位置
#define VEL_OVER_MAX_ERR     0x8611
#define NEG_LIMIT_SWITCH_ERR  0x8A80
#define POS_LIMIT_SWITCH_ERR  0x8A81

#define SYS_OVERLOAD_ERR  0xFF01    //
#define AUTO_TUNING_HALL_ERR 0xFF02 //
#define HOMING_PROCESS_ERR   0xFF03
#define SERVO_DRIVER_ERR 0xFF04



//The bit locate in CANopen_ERR_Chache_Register
/* GENERAL */
#define ERR_REG_BIT0_SET (1<<0)
#define ERR_REG_BIT0_CLR 0xFE
/* CURRENT */
#define ERR_REG_BIT1_SET (1<<1)
#define ERR_REG_BIT1_CLR 0xFD
/* VOLTAGE */
#define ERR_REG_BIT2_SET (1<<2)
#define ERR_REG_BIT2_CLR 0xFC
/* THERMAL */
#define ERR_REG_BIT3_SET (1<<3)
#define ERR_REG_BIT3_CLR 0xFB
/* COMMUNICATE */
#define ERR_REG_BIT4_SET (1<<4)
#define ERR_REG_BIT4_CLR 0xFA
/* DEVICE AREEMENT */
#define ERR_REG_BIT5_SET (1<<5)
#define ERR_REG_BIT5_CLR 0xF9
/* RESERVED */
#define ERR_REG_BIT6_SET (1<<6)
#define ERR_REG_BIT6_CLR 0xF8
/* SPECIFIC */
#define ERR_REG_BIT7_SET (1<<7)
#define ERR_REG_BIT7_CLR 0xF7




/* Function Definition */
void emergencyInit(CO_Data *d);
void emergencyStop(CO_Data* d);
void EMCY_errorRecovered(CO_Data* d, unsigned short errCode);
unsigned char EMCY_setError(CO_Data *d, unsigned short errCode, unsigned char errRegMask, unsigned short addInfo);
unsigned int OnNumberOfErrorsUpdate(CO_Data *d, const indextable *unsused_indextable, unsigned char unsused_bSubindex);
unsigned char EmcyMsg_TransSet(Emcy_Trans_Data_status *e,unsigned short errCode,unsigned char errRegMask,unsigned short addInfo);
unsigned char EmcyMsg_TransREQ(CO_Data *d,Emcy_Trans_Data_status *e);
unsigned char SendEmcy(CO_Data *d,unsigned int short errCode,unsigned char errReg,const void *Specific, unsigned char SpecificLength);

#endif
