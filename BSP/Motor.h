#ifndef _Motor_H_
#define _Motor_H_
#include "stm32f10x.h"

#define  motor_left_mid   Pin_4
#define  motor_righ_mid   Pin_5

#define  motor_left_up    Pin_6
#define  motor_righ_up    Pin_7
#define  motor_left_dw    Pin_8
#define  motor_righ_dw    Pin_9


/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void Motor_Init(void);
void Motor(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4,int16_t duty_l,int16_t duty_r,int16_t duty_sevro,int16_t duty_light);
void sevro_motor(int16_t duty1);
//void sevro_motor(int16_t duty1);
void ad(void);
u16 Get_Adc(u8 ch);   
u16 Get_Adc_Average(u8 ch,u8 times);
#endif
