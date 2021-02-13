#include "Timer3.h"

uint32_t Timer3_Count = 0;//��¼Timer3�жϴ���
uint16_t Timer3_Frequency;//Timer3�ж�Ƶ��

/******************************************************************************
����ԭ�ͣ�	void Timer3_Init(uint16_t Handler_Frequency)
��    �ܣ�	��ʼ����ʱ��3
��    ����   Handler_FrequencyΪTimer3�ж�Ƶ��
*******************************************************************************/ 
void Timer3_Init(uint16_t Handler_Frequency)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	Timer3_Frequency = Handler_Frequency;
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 1000*1000/Handler_Frequency ;//װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;	//��Ƶϵ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //���ָ�ʱ��
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//����жϱ�־
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);//ʹ�ܶ�ʱ��3
}

/******************************************************************************
����ԭ�ͣ�	void Debug_Pin_Init(void)
��    �ܣ�	��ʼ��debug����
��    ����  ��
*******************************************************************************/ 		    
void Debug_Pin_Init(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��PB�˿�ʱ��
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//PB12�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);//������ʼ��

    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}
 








