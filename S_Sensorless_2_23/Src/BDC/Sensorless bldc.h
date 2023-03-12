
extern void SensorlessBLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty);
extern void Sensorless_Start(void);




extern Sensorless_t Sensorless[2];
extern u8 change_temp;
extern void Sensorless_ChangePwm(u8 Mortor_NUM);
void Sensorless_Mototr_Stop(void);
void Bemf_Delay(u8 Motor_NUM) ;
#define SPEED_COEFF      (uint32_t)((100000/4)*60)       /* (rpm=频率/极对数/换向时间)*60    4对极，因为换向时间是一整个霍尔周期，例如霍尔6-6，所以要不需要乘6。如果换向时间代表的是单词换向，例如6-4，则需要整个公式/6*/