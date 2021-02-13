#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
   		   
//IO��������
#define SDA_IN()  {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=8<<24;}
#define SDA_OUT() {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=3<<24;}

//IO��������	 
#define IIC_SCL    PBout(15) //SCL
#define IIC_SDA    PBout(14) //SDA	 
#define READ_SDA   PBin(14)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  



extern u32 P;


























#define CMD_RESET 0x1E    // RESET 0X1E
#define CMD_PROM  0xA0    // READ  PROM
#define High_OSR  0x58



#define ACK   1    // Acknowledge
#define NACK  0    // Not Acknowledge



void MS5803_Send_Write_Comend(u8 conmend);
void MS5803_Send_Read_Comend(void);
void MS5803_PROM_Read(void);
void MS5803_Pressure_Read(void);
void MS5803_Temperature_Read(void);
void MS5803_Pressure_ReadAndDMP(void);


enum  calibration_words
{
   senst= 45269,           //C1ѹ��������

   offt1= 40405,           //C2ѹ������ֵ

   tcs  = 28588,           //C3ѹ���������¶�ϵ��

   tco  = 27134,           //C4ѹ�������¶�ϵ��

   tref = 32441,           //C5�ο��¶�

   tempsens=28742,         //C6�¶ȴ������¶�ϵ��
};








#endif
















