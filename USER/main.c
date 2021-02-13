#include "stm32f10x.h"
#include "Timer3.h"
#include "IMU.h"
#include "Tasks.h"
#include "Control.h"
#include "Uart.h" 
#include "Motor.h"

extern float shendu_actual;
extern float temp;
extern u16 USART_RX_STA;
extern volatile uint8_t USART_TX_BUF[26];
extern uint8_t angle_f[10];
extern uint8_t com;
short USART_TX_BUF_2[6];

uint32_t text;
uint32_t t,len=26;
float energe;
float energe_t,thruster_v0,thruster_v1,thruster_v2,thruster_v3,thruster_v4,thruster_v5,thruster_v6,thruster_v7,pump_t,light_t;

void CabinMessage(void);
void reversal(uint16_t * p);
void reversal_all(void);

int main(void)
{

	BSP_Int();	//�ײ�������ʼ��
	
	while(1)
	{
		if(Count_10ms>=12)
		{	
			Count_10ms = 0;
			Task_100HZ();
			energe_t=energe=Get_Adc_Average(ADC_Channel_4,1);     //������ȡ       2.6-3.2Ϊ0-100%
			energe=(energe/4096)*3.3;
			energe=(energe-2.6)*100/0.6;
			if(energe>100)
				energe=100;
		
			text=0;
			com=0;
			/*�������ݴ��Ϳ�ʼ��*/
			USART_TX_BUF[0]  = 0xdd;
			USART_TX_BUF[1]  = 0xdd;
			USART_TX_BUF[2]  = 0xdd;
			
			/*b.datafield�������������ʽ*/
			USART_TX_BUF[3]  =0;(unsigned char)energe_t*10;//��ص�ѹ float((unsigned char)recv_stack[i+3]*0.1)
			
			CabinMessage();//������Ϣ1|1|1|1|11|11;
			//0-1λ:������ 00�� 11���;2-3λ:������ 00�� 11���;4λ:©ˮ��� 1©ˮ;5λ:��е�ۿ��� 0�� 1��;6λ:����״̬ 0�ػ� 1����

			USART_TX_BUF_2[0]  = (short)100*shendu_actual;//�����Ϣ
			USART_TX_BUF_2[1]  = (short)100*temp;//�¶���Ϣ
			USART_TX_BUF_2[2]  = (short)-100*Qx;//pitch
			USART_TX_BUF_2[3]  = (short)100*Qy;//roll
			USART_TX_BUF_2[4]  = (short)100*Qz;//yaw
	


			//�����Ϣ
			USART_TX_BUF[5]  = (unsigned char)(USART_TX_BUF_2[0]);
			USART_TX_BUF[6]  = (unsigned char)(USART_TX_BUF_2[0]>>8);//��8λ��ǰ����8λ�ں�
			//�¶���Ϣ
			USART_TX_BUF[7]  = (unsigned char)(USART_TX_BUF_2[1]);
			USART_TX_BUF[8]  = (unsigned char)(USART_TX_BUF_2[1]>>8);//��8λ��ǰ����8λ�ں�
			//pitch
			USART_TX_BUF[9]  = (unsigned char)(USART_TX_BUF_2[2]);
			USART_TX_BUF[10]  = (unsigned char)(USART_TX_BUF_2[2]>>8);
			//roll
			USART_TX_BUF[11]  = (unsigned char)(USART_TX_BUF_2[3]);
			USART_TX_BUF[12]  = (unsigned char)(USART_TX_BUF_2[3]>>8);
			//yaw
			USART_TX_BUF[13]  = (unsigned char)(USART_TX_BUF_2[4]);
			USART_TX_BUF[14]  = (unsigned char)(USART_TX_BUF_2[4]>>8);
			
					/*pwm��������2000�����Ƶ�ѹ*10�ϴ�*/	
					
			USART_TX_BUF[19]  = (unsigned char)(thruster_v2);//������1
			//USART_TX_BUF[16]  = (unsigned char)(thruster_v0);//������2
			USART_TX_BUF[20]  = (unsigned char)(thruster_v3);//������1
			//USART_TX_BUF[18]  = (unsigned char)(thruster_v0);//������2	
			USART_TX_BUF[15]  = (unsigned char)(thruster_v0);//ˮƽ����1
			//USART_TX_BUF[20]  = (unsigned char)thruster_v0;//ˮƽ����2
			USART_TX_BUF[16]  = (unsigned char)(thruster_v1);//ˮƽ��1 	
			//USART_TX_BUF[22]  = (unsigned char)(thruster_v0);//ˮƽ��2
			
			USART_TX_BUF[23]  = (unsigned char)pump_t/200;//��
			
			USART_TX_BUF_2[5]=do_crc(USART_TX_BUF, 24);
			USART_TX_BUF[24]  = (unsigned char)(USART_TX_BUF_2[5]>>8);//CrcУ���8λ
			USART_TX_BUF[25]  = (unsigned char)(USART_TX_BUF_2[5]);//CrcУ���8λ
			
			

			for(t=0;t<len;t++)
		  {
						
		    	USART_SendData(USART1, USART_TX_BUF[t]);//�򴮿�1��������(0-4��)
				
		    	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
      }
  	}
  	}
  }


void CabinMessage(void)			//0-1λ:������ 00�� 11���;2-3λ:������ 00�� 11���;4λ:©ˮ��� 1©ˮ;5λ:��е�ۿ��� 0�� 1��;6λ:����״̬ 0�ػ� 1����
{
	
		USART_TX_BUF[4]=0;
if(start)
	USART_TX_BUF[4]=0x40;
else
	USART_TX_BUF[4]=0;//power on/off


if(TIM2->CCR1 == 1400)
USART_TX_BUF[4]=USART_TX_BUF[4]|0x20;//light on
if(TIM2->CCR1 == 0)
USART_TX_BUF[4]=USART_TX_BUF[4]|0;//light off

	
if(TIM2->CCR2 == 1600)
USART_TX_BUF[4]=USART_TX_BUF[4]|0x01;//arm on 
if(TIM2->CCR2 == 1300)
USART_TX_BUF[4]=USART_TX_BUF[4]|0;//arm stop
if(TIM2->CCR2 == 1100)
USART_TX_BUF[4]=USART_TX_BUF[4]|0x02;//arm off
	

	
}

void reversal(uint16_t * p)
{
	uint16_t re=* p;
re=re<8;
	*p=*p>8;
	*p+=re;
}
void reversal_all(void)
{
	int i;
	for(i=0;i<5;i++)
	{
		reversal(USART_TX_BUF_2+i);
	}
}

