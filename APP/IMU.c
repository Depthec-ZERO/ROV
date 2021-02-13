#include "IMU.h"
#include "math.h"
#include "Uart.h" 
#include "myiic.h"
#include "Control.h"

extern volatile uint8_t Rx_Buffer2[26]; //����3������

float ax,ay,az;
float gx,gy,gz;
float Qx,Qy,Qz;
float q_x[10],q_y[10],q_z[10];
float depth_R;

extern uint16_t ok,start;
uint16_t angle_f1[10];
uint8_t angle_f[10];

extern int16_t angle_actral;

/******************************************************************************
����ԭ�ͣ�	void CK_Check(void)
��    �ܣ�	IMU data check
��    ����  ��
*******************************************************************************/ 
void CK_Check(void)
{
u8 CK1 = 0, CK2 = 0;
for(int i=0;i<20;i++)
{
CK1 = CK1 + Rx_Buffer2[i];
CK2 = CK2 + CK1;
}
}

/******************************************************************************
����ԭ�ͣ�	void IMU_Get(void) 
��    �ܣ�	�õ�ROV��̬����
��    ����  ��
*******************************************************************************/ 
//59 53 |B2 1F |0E |40 0C| 49 D0 13 00 |1D 21 E5 FF| 30 C5 71 F7 |D6 DA 
// 0  1 |2  3  | 4 | 5  6|  7  8  9 10 |11 12 13 14| 15 16 17 18 |19 20
// head   TID   len            pitch       roll        yaw          check
int a;
void IMU_Get(void) 
{
	u8 CK1 = 0, CK2 = 0;
	
	for(int i=2;i<19;i++)
	{
	CK1 = CK1 + Rx_Buffer2[i];
	CK2 = CK2 + CK1;
	}
	a=0;
	if(Rx_Buffer2[19] == CK1 && Rx_Buffer2[20] == CK2 )//���������Ƿ񱣴����
	{ a=1;
		Qx=(Rx_Buffer2[7]+(Rx_Buffer2[8]<<8)+(Rx_Buffer2[9]<<16)+(Rx_Buffer2[10]<<24))*0.000001;
		Qy=(Rx_Buffer2[11]+(Rx_Buffer2[12]<<8)+(Rx_Buffer2[13]<<16)+(Rx_Buffer2[14]<<24))*0.000001;
		Qz=(Rx_Buffer2[15]+(Rx_Buffer2[16]<<8)+(Rx_Buffer2[17]<<16)+(Rx_Buffer2[18]<<24))*0.000001;
		
		
		
		
		
//		if(Rx_Buffer2[1]==0x51)//���ٶ����ݺϳ�
//	  {
//			      ax=(((short)Rx_Buffer2[3]<<8)|Rx_Buffer2[2])/32768.0*16*9.8;//
//            ay=(((short)Rx_Buffer2[5]<<8)|Rx_Buffer2[4])/32768.0*16*9.8;//
//            az=(((short)Rx_Buffer2[7]<<8)|Rx_Buffer2[6])/32768.0*16*9.8;//
//		}                                                    
//	                                                         
//	  if(Rx_Buffer2[1]==0x52)//���ٶ����ݺϳ�
//	  {
//			      gx=(((short)Rx_Buffer2[3]<<8)|Rx_Buffer2[2])/32768.0*2000;
//            gy=(((short)Rx_Buffer2[5]<<8)|Rx_Buffer2[4])/32768.0*2000;
//            gz=(((short)Rx_Buffer2[7]<<8)|Rx_Buffer2[6])/32768.0*2000;;
//		}
//	
//    if(Rx_Buffer2[1]==0x53)//�Ƕ����ݺϳ�
//	  {
//		    Qx=(((short)Rx_Buffer2[3]<<8)|Rx_Buffer2[2])/32768.0*180;
//		      	if(Qx>180)   
//							Qx=Qx-360;
//			        
//        Qy=(((short)Rx_Buffer2[5]<<8)|Rx_Buffer2[4])/32768.0*180;
//		      	if(Qy>180)   
//							Qy=Qy-360;
//			
//        Qz=(((short)Rx_Buffer2[7]<<8)|Rx_Buffer2[6])/32768.0*180;
//			      if(Qz>180)   
//							Qz=Qz-360;	   	
//						
						
			Qx=Qx;    //���Ƕ�ת��Ϊ���Է��͵���
		if(Qy>=0) Qy=Qy-180;
		else Qy = 180+Qy;
			Qz=Qz;
			
///*********************************������ֵ�˲�**********************************************/
//uint16_t m,n;				  
//for(m=0;m<10;m++)				  
//{
//  
//	
//}


				  
				  
				  
				  
/*********************************������ֵ�˲�**********************************************/	
			angle_actral=90-Qx;//90��Ϊƽ��ʱ�Ĳ����Ƕ� ����Ϊ�� ����Ϊ��
						
			angle_f[0]=(uint8_t)((int16_t)(Qy*100)/200);
			angle_f[1]=(uint8_t)((int16_t)(Qy*100)%200);
			angle_f[2]=(uint8_t)((int16_t)(Qx*100)/200);
			angle_f[3]=(uint8_t)((int16_t)(Qx*100)%200);
			angle_f[4]=(uint8_t)((int16_t)(Qz*100)/200);
			angle_f[5]=(uint8_t)((int16_t)(Qz*100)%200);
						
						
			ax=(float)((float)angle_f[0]*2+(float)(angle_f[1])/100);
			ay=(float)((float)angle_f[2]*2+(float)(angle_f[3])/100);
			az=(float)((float)angle_f[4]*2+(float)(angle_f[5])/100);
		}
	
	
    //depth_R = readMS5540();  //return depth 

}



























