#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"

/******************************************************************************
							全局变量声明
*******************************************************************************/ 

/* pid变量 */
struct _pid
{
	float buf;
	float err;
	float last_err;
	
	float kp;
	float ki;
	float kd;
	float integral;
	
	float output;
};

extern float throttle1,throttle2,throttle3,throttle4,throttle5,throttle6;
extern int16_t angle_actral;

extern struct _pid Heading;
extern struct _pid depth;
extern struct _pid pos;
/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void PID_Init(void);
void depth_control(uint16_t depth_want,uint16_t angle_want,uint16_t p);
void Heading_control(uint16_t direct,uint16_t p_d,uint16_t forward,uint16_t p_f,uint16_t side,uint16_t p_s);
void LED_Init(void);





#endif
