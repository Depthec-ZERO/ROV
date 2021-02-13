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

	BSP_Int();	//底层驱动初始化
	
	while(1)
	{
		if(Count_10ms>=12)
		{	
			Count_10ms = 0;
			Task_100HZ();
			energe_t=energe=Get_Adc_Average(ADC_Channel_4,1);     //电量读取       2.6-3.2为0-100%
			energe=(energe/4096)*3.3;
			energe=(energe-2.6)*100/0.6;
			if(energe>100)
				energe=100;
		
			text=0;
			com=0;
			/*报文数据传送开始符*/
			USART_TX_BUF[0]  = 0xdd;
			USART_TX_BUF[1]  = 0xdd;
			USART_TX_BUF[2]  = 0xdd;
			
			/*b.datafield：详见操作数格式*/
			USART_TX_BUF[3]  =0;(unsigned char)energe_t*10;//电池电压 float((unsigned char)recv_stack[i+3]*0.1)
			
			CabinMessage();//舱内信息1|1|1|1|11|11;
			//0-1位:三档灯 00关 11最高;2-3位:三档泵 00关 11最高;4位:漏水检测 1漏水;5位:机械臂开关 0关 1开;6位:机器状态 0关机 1开机

			USART_TX_BUF_2[0]  = (short)100*shendu_actual;//深度信息
			USART_TX_BUF_2[1]  = (short)100*temp;//温度信息
			USART_TX_BUF_2[2]  = (short)-100*Qx;//pitch
			USART_TX_BUF_2[3]  = (short)100*Qy;//roll
			USART_TX_BUF_2[4]  = (short)100*Qz;//yaw
	


			//深度信息
			USART_TX_BUF[5]  = (unsigned char)(USART_TX_BUF_2[0]);
			USART_TX_BUF[6]  = (unsigned char)(USART_TX_BUF_2[0]>>8);//低8位在前，高8位在后
			//温度信息
			USART_TX_BUF[7]  = (unsigned char)(USART_TX_BUF_2[1]);
			USART_TX_BUF[8]  = (unsigned char)(USART_TX_BUF_2[1]>>8);//低8位在前，高8位在后
			//pitch
			USART_TX_BUF[9]  = (unsigned char)(USART_TX_BUF_2[2]);
			USART_TX_BUF[10]  = (unsigned char)(USART_TX_BUF_2[2]>>8);
			//roll
			USART_TX_BUF[11]  = (unsigned char)(USART_TX_BUF_2[3]);
			USART_TX_BUF[12]  = (unsigned char)(USART_TX_BUF_2[3]>>8);
			//yaw
			USART_TX_BUF[13]  = (unsigned char)(USART_TX_BUF_2[4]);
			USART_TX_BUF[14]  = (unsigned char)(USART_TX_BUF_2[4]>>8);
			
					/*pwm计数上限2000，控制电压*10上传*/	
					
			USART_TX_BUF[19]  = (unsigned char)(thruster_v2);//垂推左1
			//USART_TX_BUF[16]  = (unsigned char)(thruster_v0);//垂推左2
			USART_TX_BUF[20]  = (unsigned char)(thruster_v3);//垂推右1
			//USART_TX_BUF[18]  = (unsigned char)(thruster_v0);//垂推右2	
			USART_TX_BUF[15]  = (unsigned char)(thruster_v0);//水平推左1
			//USART_TX_BUF[20]  = (unsigned char)thruster_v0;//水平推左2
			USART_TX_BUF[16]  = (unsigned char)(thruster_v1);//水平右1 	
			//USART_TX_BUF[22]  = (unsigned char)(thruster_v0);//水平右2
			
			USART_TX_BUF[23]  = (unsigned char)pump_t/200;//泵
			
			USART_TX_BUF_2[5]=do_crc(USART_TX_BUF, 24);
			USART_TX_BUF[24]  = (unsigned char)(USART_TX_BUF_2[5]>>8);//Crc校验高8位
			USART_TX_BUF[25]  = (unsigned char)(USART_TX_BUF_2[5]);//Crc校验低8位
			
			

			for(t=0;t<len;t++)
		  {
						
		    	USART_SendData(USART1, USART_TX_BUF[t]);//向串口1发送数据(0-4字)
				
		    	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
      }
  	}
  	}
  }


void CabinMessage(void)			//0-1位:三档灯 00关 11最高;2-3位:三档泵 00关 11最高;4位:漏水检测 1漏水;5位:机械臂开关 0关 1开;6位:机器状态 0关机 1开机
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

