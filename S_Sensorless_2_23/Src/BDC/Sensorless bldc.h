
extern void SensorlessBLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty);
extern void Sensorless_Start(void);




extern Sensorless_t Sensorless[2];
extern u8 change_temp;
extern void Sensorless_ChangePwm(u8 Mortor_NUM);
void Sensorless_Mototr_Stop(void);
void Bemf_Delay(u8 Motor_NUM) ;
#define SPEED_COEFF      (uint32_t)((100000/4)*60)       /* (rpm=Ƶ��/������/����ʱ��)*60    4�Լ�����Ϊ����ʱ����һ�����������ڣ��������6-6������Ҫ����Ҫ��6���������ʱ�������ǵ��ʻ�������6-4������Ҫ������ʽ/6*/