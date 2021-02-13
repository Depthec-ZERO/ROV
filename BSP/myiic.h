#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
   		   
//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=8<<24;}
#define SDA_OUT() {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=3<<24;}

//IO操作函数	 
#define IIC_SCL    PBout(15) //SCL
#define IIC_SDA    PBout(14) //SDA	 
#define READ_SDA   PBin(14)  //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

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
   senst= 45269,           //C1压力灵敏度

   offt1= 40405,           //C2压力补偿值

   tcs  = 28588,           //C3压力灵敏度温度系数

   tco  = 27134,           //C4压力补偿温度系数

   tref = 32441,           //C5参考温度

   tempsens=28742,         //C6温度传感器温度系数
};








#endif
















