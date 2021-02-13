#include "myiic.h"
#include "delay.h"
#include "math.h"
//////////////////////////////////////////////////////////////////////////////////	 
// MS5803-14BA  气压传感器  在水下，每增加1m深度，压强增加1Kpa
//////////////////////////////////////////////////////////////////////////////////
float shendu_actual,temp;
extern uint8_t angle_f[10];
float  dep;

//初始化IIC
void IIC_Init(void)
{					     
					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//先使能外设IO PORTC时钟 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	IIC_SCL=1;
	IIC_SDA=1;

}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(4); 
	IIC_SCL=1;//STOP:when CLK is high DATA change form low to high
 	delay_us(4); 
	IIC_SDA=1;//发送I2C总线结束信号 						   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}





























#define CMD_RESET 0x1E    // RESET 0X1E
#define CMD_PROM  0xA0    // READ  PROM




#define ACK   1    // Acknowledge
#define NACK  0    // Not Acknowledge

u16 coefficient[8];
u32 P_conversion;
u32 T_conversion;
u32 P;
// 从机地址 写 0XEE
// 从机地址 读  0XEF
void MS5803_Send_Write_Comend(u8 conmend)

{
 
     IIC_Start();
	   IIC_Send_Byte(0XEE);
	   IIC_Wait_Ack();
     IIC_Send_Byte(conmend);
	   IIC_Wait_Ack();
     IIC_Stop();

}

void MS5803_Send_Read_Comend(void)

{
     
     IIC_Start();
	   IIC_Send_Byte(0XEF);
	   IIC_Wait_Ack();
    

}












void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号




// 读取校准参数
void MS5803_PROM_Read(void)
// Initialize library for subsequent pressure measurements
{  
	
	  uint8_t highByte; 
		uint8_t lowByte ;
	  uint8_t i;
	MS5803_Send_Write_Comend(CMD_RESET);
	
	delay_ms(5);
		
	
	for(i = 0; i <= 8; i++)
  {
		
		MS5803_Send_Write_Comend(CMD_PROM + (i*2));
		delay_ms(2);
		MS5803_Send_Read_Comend();
		highByte=IIC_Read_Byte(ACK);
	    lowByte=IIC_Read_Byte(NACK);
		IIC_Stop();
		coefficient[i] = (highByte << 8)|lowByte;
	}
	
	
//	for(i = 0; i <= 8; i++)
//	{
//	  printf("C%d   %d  ",i,coefficient[i]);
//	}
	
		

}

//  C0   65535  C1   45269  C2   40405  C3  28588   C4  27134  C5   32441  C6   28742  C7   13
//              C1   45269  C2   40405  C3   28588  C4  27134  C5   32441  C6 28742 

//从传感器中读取压力值



void MS5803_Temperature_Read(void)
{
	
		uint8_t highByte; 
		uint8_t lowByte ;
	    uint8_t middleByte; 
	   
	
		MS5803_Send_Write_Comend(0x58);
		delay_ms(10);
		MS5803_Send_Write_Comend(0x00);
		delay_ms(2);
		MS5803_Send_Read_Comend();
		highByte=IIC_Read_Byte(ACK);
	    middleByte=IIC_Read_Byte(ACK);
		lowByte=IIC_Read_Byte(NACK);
		IIC_Stop();
		T_conversion = (highByte << 16)|(middleByte <<8)|(lowByte);
   // printf("  %ld  ",T_conversion);
}



void MS5803_Pressure_Read(void)
{
	
		uint8_t highByte; 
		uint8_t lowByte ;
	    uint8_t middleByte; 
		MS5803_Send_Write_Comend(0x48);
		delay_ms(10);
		MS5803_Send_Write_Comend(0x00);
		delay_ms(2);
		MS5803_Send_Read_Comend();
		highByte=IIC_Read_Byte(ACK);
	    middleByte=IIC_Read_Byte(ACK);
		lowByte=IIC_Read_Byte(NACK);
		IIC_Stop();
		P_conversion = (highByte << 16)|(middleByte <<8)|(lowByte);
   // printf("  %ld  ",P_conversion);
}







///*对压力进行一阶修正*/

//  int64_t off;

//  int64_t sens;

//  int32_t pres;

//  off=(int64_t)(offt1*pow(2,17)+(tco*dT)/pow(2,6));

//  sens=(int64_t)(senst1*pow(2,16)+(tcs*dT)/pow(2,7));

//  pres=(int32_t)((digitalPressureValue*sens/pow(2,21)-off)/pow(2,15));



//enum  calibration_words
//{
//   senst= 45269,           //C1压力灵敏度

//   offt1= 40405,           //C2压力补偿值

//   tcs  = 28588,           //C3压力灵敏度温度系数

//   tco  = 27134,           //C4压力补偿温度系数

//   tref = 32441,           //C5参考温度

//   tempsens=28742,         //C6温度传感器温度系数
//};

// P_conversion = D1;
// T_conversion = D2;
// profession barometric pressure 1013
// 深度增加一米，压力增加9800pa   1br= 100 kpa   1mbar=100pa  每增加1m增加98mbar 1cm 增加0.98  近似认为为1mbar
void MS5803_Pressure_ReadAndDMP(void)
{
  
	signed long int dT,TEMP;
	signed long long int off,off2;
	signed long long int sens,sens2;
	signed long long int T2;
	double depth_R;
	MS5803_Pressure_Read();
    MS5803_Temperature_Read();
	dT=T_conversion - tref*256;
    sens = (senst * pow(2,15))+ ((tcs * dT )/pow(2,8));
    off  = (offt1 * pow(2,16))+ ((tco * dT) / pow(2,7));
 
	
	
	TEMP=2000+(dT*tempsens/pow(2,23));

////二阶换算	
	if(TEMP<2000)
	{
	 T2=3*dT*dT/pow(2,33);
	 off2=(3*(TEMP-2000)*(TEMP-2000))/2;
	 sens2=(5*(TEMP-2000)*(TEMP-2000))/8;
	 if(TEMP<-1500)
	 {
	 off2=off2+7*(TEMP+1500)*(TEMP+1500);
	 sens2=sens2+4*(TEMP+1500)*(TEMP+1500);
	 }
	}
	else
  {
	 T2=7*dT*dT/pow(2,37);
	 off2=(1*(TEMP-2000)*(TEMP-2000))/16;
	 sens2=0;
	}
	
	TEMP=TEMP-T2;
	off=off-off2;
	sens=sens-sens2;
//printf(" %ld  ",TEMP);
	P = (P_conversion * sens / pow(2,21)- off) /pow(2,15)+20;
//	printf("%ldPa   ",P);
	//depth_R=((P*10-101300)/10091.0429);
	dep=((P*10-79500)/10091.0429);
//	dep=(P/10091.0429);
//	dep=dep-0.798;
//	dep=dep*10;
	shendu_actual=dep*10;
	angle_f[6]=(uint8_t)((int16_t)(dep*100)/200);
	angle_f[7]=(uint8_t)((int16_t)(dep*100)%200);
//	angle_f[6]=1;
//	angle_f[7]=0;
	temp=TEMP;
	
	
	
	
	
	
	
}


