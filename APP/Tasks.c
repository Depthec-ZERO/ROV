#include "IMU.h"
#include "myiic.h"
#include "Tasks.h"
#include "Uart.h" 
#include "Control.h"
#include "Timer3.h"
#include "Motor.h"
#include "Protocol.h"
#include "delay.h"
#include "myiic.h"

uint8_t Bsp_Int_Ok = 0;
float duoji,duoji_text,light_c;
uint16_t depth_c,angle_c,forward_c,side_c,direct_c,direct_c1,direct_c2,duoji_c;
uint16_t ok,start;
uint16_t lightt;
extern float throttle11 ,throttle21 ,throttle51 ,throttle61 ;
void duoji_contro(uint16_t ss);
void bond_change()
{
	
	char USART_TX_BUF3[8];
	USART_TX_BUF3[0]=0x59;
	USART_TX_BUF3[1]=0x53;
	USART_TX_BUF3[2]=0x02;
	USART_TX_BUF3[3]=0x09;
	USART_TX_BUF3[4]=0x00;
	USART_TX_BUF3[5]=0x03;
	USART_TX_BUF3[6]=0x0E;
	USART_TX_BUF3[7]=0x26;
	
	
	
	for(int t=0;t<8;t++)
		  {
						
		    	USART_SendData(USART3, USART_TX_BUF3[t]);//�򴮿�1��������(0-4��)
				
		    	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
      }
	
	char USART_TX_BUF4[9];

	USART_TX_BUF4[0]=0x59;
	USART_TX_BUF4[1]=0x53;
	USART_TX_BUF4[2]=0x03;
	USART_TX_BUF4[3]=0x09;
	USART_TX_BUF4[4]=0x00;
	USART_TX_BUF4[5]=0x05;
	USART_TX_BUF4[6]=0x11;
	USART_TX_BUF4[7]=0x2C;
			
	
	
	
	for(int t=0;t<8;t++)
		  {
						
		    	USART_SendData(USART3, USART_TX_BUF3[t]);//�򴮿�1��������(0-4��)
				
		    	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
      }

}

/******************************************************************************
����ԭ�ͣ�	void Nvic_Init(void)
��    �ܣ�	NVIC��ʼ��
*******************************************************************************/ 
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//NVIC_PriorityGroup 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//���ȼ�����
    //Timer3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	//����1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//����3�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//����2�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
//	//�δ�ʱ���ж�
//	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	
}

/******************************************************************************
����ԭ�ͣ�	void BSP_Int(void)
��    �ܣ�	Ӳ��������ʼ��
*******************************************************************************/ 
void BSP_Int(void)
{
	Nvic_Init();         //�ж����ȼ���ʼ��	
	delay_init();
	Uart1_Init(115200);  //����1��ʼ����������115200��8λ���ݣ�1λֹͣλ��������żУ��   ��������
	Uart2_Init(115200);  //Init Uart2  depth gauge
	Uart3_Init(460800);    //����3��ʼ��  �����Ǽ��ٶȼ�
	////bond_change();
	//Uart3_Init(115200);
	ad();                //��ѹ����ģ���ʼ��
	Motor_Init();        //PWM��ʼ��

	
  Motor(0,0,0,0,0,0,0,0);
	Timer3_Init(600);    //Timer3�ж�100HZ    ��ʱ���ж�
	lightt=0;
	Bsp_Int_Ok = 1;
} 

/******************************************************************************
����ԭ�ͣ�	void Task_100HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ100HZ����
*******************************************************************************/ 
void Task_100HZ(void)
{
	Debug1_H;
	IMU_Get();
	Order_Get();
	if(start==0)
	{
	  Motor(0,0,0,0,0,0,0,0);	
	}
	if(start)
	{
		duoji_contro(duoji_c);      					      //�������
		Heading_control(forward_c,20,side_c,20,direct_c,10);  //direction control
		depth_control(depth_c,angle_c,4);                     //depth control
	  ok=1;
    Motor(throttle11,throttle21,throttle51,throttle61,0,0,0,0);
	}
	/*******************�������****************/
	if(duoji>1900)	duoji=1900;
	if(duoji<1100)	duoji=1100;
	TIM2->CCR3 =duoji;            //A2
	
	TIM2->CCR4 = lightt*80000+1;          //A3
	
    Debug1_L;
}

/******************************************************************************
����ԭ�ͣ�	void Task_20HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ20HZ����
*******************************************************************************/ 
void Task_20HZ(void)
{
	Debug2_H;

	//Order_Get();

	
	Debug2_L;
}


/******************************************************************************
����ԭ�ͣ�	void duoji_control(uint16_t ss)
��    �ܣ�	����PID����
*******************************************************************************/ 
void duoji_contro(uint16_t ss)
{
  int16_t data_c;
	if(ss==2)
		data_c=4;
	else if(ss==1)
		data_c=-4;
	else
		data_c=0;
	
	duoji=duoji+data_c;
	//duoji=duoji/2;
	
	if(duoji>1900)  //��5s������ɴ�����޵���С���޵ĵ���
		duoji=1900;
	if(duoji<1100)
		duoji=1100;
}




