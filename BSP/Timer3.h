#ifndef _TIMER3_H_
#define _TIMER3_H_
#include "stm32f10x.h"


/******************************************************************************
							宏定义
*******************************************************************************/ 
#define Debug1_Pin	GPIO_Pin_12	//用于测量程序运行速率
#define	Debug1_H   	GPIOB->BSRR = Debug1_Pin //高电平
#define	Debug1_L   	GPIOB->BRR  = Debug1_Pin //低电平

#define Debug2_Pin	GPIO_Pin_13		
#define	Debug2_H   	GPIOB->BSRR = Debug2_Pin 
#define	Debug2_L   	GPIOB->BRR  = Debug2_Pin 


/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint32_t Timer3_Count;
extern uint16_t Timer3_Frequency;
extern uint8_t Count_10ms,Count_50ms;




/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void Timer3_Init(uint16_t Handler_Frequency);
void Debug_Pin_Init(void);









#endif




