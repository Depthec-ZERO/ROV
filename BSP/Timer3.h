#ifndef _TIMER3_H_
#define _TIMER3_H_
#include "stm32f10x.h"


/******************************************************************************
							�궨��
*******************************************************************************/ 
#define Debug1_Pin	GPIO_Pin_12	//���ڲ���������������
#define	Debug1_H   	GPIOB->BSRR = Debug1_Pin //�ߵ�ƽ
#define	Debug1_L   	GPIOB->BRR  = Debug1_Pin //�͵�ƽ

#define Debug2_Pin	GPIO_Pin_13		
#define	Debug2_H   	GPIOB->BSRR = Debug2_Pin 
#define	Debug2_L   	GPIOB->BRR  = Debug2_Pin 


/******************************************************************************
							ȫ�ֱ�������
*******************************************************************************/ 
extern uint32_t Timer3_Count;
extern uint16_t Timer3_Frequency;
extern uint8_t Count_10ms,Count_50ms;




/******************************************************************************
							ȫ�ֺ�������
*******************************************************************************/ 
void Timer3_Init(uint16_t Handler_Frequency);
void Debug_Pin_Init(void);









#endif




