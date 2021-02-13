#include "myiic.h"
#include "delay.h"
#include "math.h"
//////////////////////////////////////////////////////////////////////////////////	 
// MS5803-14BA  ��ѹ������  ��ˮ�£�ÿ����1m��ȣ�ѹǿ����1Kpa
//////////////////////////////////////////////////////////////////////////////////
float shendu_actual,temp;
extern uint8_t angle_f[10];
float  dep;

//��ʼ��IIC
void IIC_Init(void)
{					     
					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTCʱ�� 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	IIC_SCL=1;
	IIC_SDA=1;

}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(4); 
	IIC_SCL=1;//STOP:when CLK is high DATA change form low to high
 	delay_us(4); 
	IIC_SDA=1;//����I2C���߽����ź� 						   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
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
// �ӻ���ַ д 0XEE
// �ӻ���ַ ��  0XEF
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












void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�




// ��ȡУ׼����
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

//�Ӵ������ж�ȡѹ��ֵ



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







///*��ѹ������һ������*/

//  int64_t off;

//  int64_t sens;

//  int32_t pres;

//  off=(int64_t)(offt1*pow(2,17)+(tco*dT)/pow(2,6));

//  sens=(int64_t)(senst1*pow(2,16)+(tcs*dT)/pow(2,7));

//  pres=(int32_t)((digitalPressureValue*sens/pow(2,21)-off)/pow(2,15));



//enum  calibration_words
//{
//   senst= 45269,           //C1ѹ��������

//   offt1= 40405,           //C2ѹ������ֵ

//   tcs  = 28588,           //C3ѹ���������¶�ϵ��

//   tco  = 27134,           //C4ѹ�������¶�ϵ��

//   tref = 32441,           //C5�ο��¶�

//   tempsens=28742,         //C6�¶ȴ������¶�ϵ��
//};

// P_conversion = D1;
// T_conversion = D2;
// profession barometric pressure 1013
// �������һ�ף�ѹ������9800pa   1br= 100 kpa   1mbar=100pa  ÿ����1m����98mbar 1cm ����0.98  ������ΪΪ1mbar
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

////���׻���	
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


