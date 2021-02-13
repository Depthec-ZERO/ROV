#ifndef _IMU_H_
#define	_IMU_H_
#include "stm32f10x.h"


extern float ax,ay,az;
extern float gx,gy,gz;
extern float Qx,Qy,Qz;
extern float depth_R;

extern uint8_t angle_f[10];





/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void IMU_Get(void);




#endif

