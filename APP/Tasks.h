#ifndef _TASKS_H_
#define	_TASKS_H_
#include "stm32f10x.h"


/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint8_t Bsp_Int_Ok; 
extern float duoji,light_c;
extern uint16_t depth_c,angle_c,forward_c,side_c,direct_c,direct_c1,direct_c2,duoji_c;
extern uint16_t ok,start;
/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void Nvic_Init(void);
void BSP_Int(void);
void Task_100HZ(void);
void Task_20HZ(void);
unsigned short do_crc(u8 *pack, int num);
	 
void duoji_control(int16_t ss);
#endif

