#include "IMU.h"
#include "Motor.h"
#include "Control.h"
#include "Protocol.h"
#include "myiic.h"

#include<math.h>
#define d_rate 20

struct _pid Heading;
struct _pid depth;
struct _pid pos;
struct _pid zhi;


extern uint8_t Count_50ms;
float kpp,kii,kdd;
float throttle1,throttle2,throttle3,throttle4,throttle5,throttle6;
uint8_t depth_flag,Heading_flag;
float depth_ess,depth_integration,depth_eess,depth_ess1,motor_out;  //PID各项参数

int16_t angle_actral;
int16_t angle_ess,angle_integration,angle_eess,angle_ess1,angle_out;  //PID各项参数
int16_t angle_ess_max,angle_ess_min,angle_integration_max,angle_integration_min,angle_eess_max,angle_eess_min;//PID各项的限幅

int16_t fixed=0;   //使其在水中漂浮所需要的垂直推力
float depth_blance=0;

float depth_l,shendu_actuals=0;

extern uint16_t depth_ctrl;

extern uint16_t ok,start;
extern float shendu_actual;
extern volatile uint8_t Rx_Buffer1[24];
float test_dp,test_di,test_dd,test_zp,test_zi,test_zd;


extern uint16_t USART_TX_BUF_2[6];
/******************************************************************************
函数原型：	void PID_Init(void)
功    能：	设置PID参数
*******************************************************************************/ 
void PID_Init(void)
{
	static float hp=0,hi=0,hd=0;
	static float dp=0,di=0.0f,dd=0.0f;
	

	hp =Rx_Buffer1[7]/10.0;
	hi =Rx_Buffer1[8]/10.0;
	hd =Rx_Buffer1[9]/10.0;
	

		dp=Rx_Buffer1[4];
		dp=10*dp;
	
	

		di=Rx_Buffer1[5];
		di/=20.0;
	

		dd=Rx_Buffer1[6];
		dd=d_rate*dd;
	
//			Heading.kp=hp;
//			Heading.ki=hi;
//			Heading.kd=hd;
		
			test_dp=depth.kp =dp;
			test_di=depth.ki =di;
			test_dd=depth.kd =dd;

	
	test_zp=zhi.kp = hp;
test_zi=	zhi.ki = hi;
	test_zd=zhi.kd = hd;
	
	
}



#define depth_max 	10.0f	
#define depth_integral_max 	1000.0f	

int16_t pid_count;
/******************************************************************************
函数原型：	void depth_control(void)
功    能：	深度控制PID
*******************************************************************************/ 
float zhuanxiang_test,zhuan_i,zhuan_p,zhuang_d,zhuan_t,targetDepth;
void depth_control(uint16_t depth_want,uint16_t angle_want,uint16_t p)
{
	  static float i = 0,eess_asum;
		static int j=0;
	  float depth_want1,angle_want1;
		int16_t targetDepth1;
		static uint8_t dpid_rest_1,dpid_rest_2;
		targetDepth1=Rx_Buffer1[11]<<8;		targetDepth=(float)((targetDepth1|Rx_Buffer1[10]))/1000.0;
	
		
	  depth_want1=128-depth_want;
	  motor_out=depth_want1;//	  angle_want1=128-angle_want;   //angle_want1范围是-128------128
	
//	if(!((dpid_rest_1==Rx_Buffer1[11])&&(dpid_rest_2==Rx_Buffer1[10])))
//		{
//			dpid_rest_1=Rx_Buffer1[11];
//			dpid_rest_2=Rx_Buffer1[10];
//			
//		}
		
	if(((Rx_Buffer1[3]&7)==2)||((Rx_Buffer1[3]&7)==1)) 
{
			

		 depth_blance=targetDepth;
	

			depth_ess=shendu_actual-depth_blance;  //			shendu_actuals=fabs(sin(++i));
	
			if(depth_ess>50)  depth_ess=50;
			if(depth_ess<-50)  depth_ess=-50;
				
		zhuan_i=	depth_integration+=depth_ess;          //积分项
	        
	   	 
		j++;
		eess_asum+=depth_ess;
		if(j==5)
		{
			zhuan_p=depth_ess1=eess_asum/5.0;
			eess_asum=0;
	
		}
		zhuang_d=depth_eess=depth_ess-depth_ess1;       //微分项
		
		kpp=depth.kp*depth_ess;
		kii=depth.ki*depth_integration;
		kdd=depth.kd*depth_eess;


		
	//integration limination
			if(kii >= 400)
			kii = 400;							 
			else if(kii <= -400)
			kii = -400;
		
			zhuan_t=motor_out=-(depth.kp*depth_ess
																						+kii
																								+depth.kd*depth_eess);
	  
	  
	}
//      angle_ess=angle_want1-angle_actral*4;    //angle_actral也应该由-30-----30转化为-128------128
//	  if(angle_ess>-8&&angle_ess<8)
//		  angle_ess=0;
//	  else
 
//	    {
//		  if(angle_ess>0)
//		  angle_ess=angle_ess-8;
//		  if(angle_ess<0)
//		  angle_ess=angle_ess+8;
//		}
//	  angle_integration+=angle_ess;            //积分项
//	  if(angle_integration>500)
//		  angle_integration=500;
//	  if(angle_integration<-500)
//		  angle_integration=-500;
//	  
//	  angle_eess=angle_ess-angle_ess1;         //微分项      大于零表示偏差加大，因此需要补偿
//	  angle_ess1=angle_ess;
//	  
//      angle_want1=pos.kp*angle_ess+pos.ki*angle_integration+pos.kd*angle_eess;
//	  if(angle_want1>110)
//				angle_want1=110;
//	  if(angle_want1<-110)
//				angle_want1=-110;
			 
	  throttle5=(float)(motor_out*5);     //限幅为-500----500+fixed-angle_want1
	  throttle6=(float)(motor_out*5);	 

	
}
#define angle_max 	 10.0f	
#define angle_integral_max 	1000.0f	


uint8_t hpid_rest_1,hpid_rest_2;
 int16_t qian_intre,qian_ess1;
uint16_t direct2;
	float zhuanxiang_test;
/******************************************************************************
函数原型：	void Heading_control(void)
功    能：	航向PID
*******************************************************************************/ 
void Heading_control(uint16_t forward,uint16_t p_f,uint16_t side,uint16_t p_s,uint16_t direct,uint16_t p_d)
{
	int16_t zhuanxiang_1,zhuanxiang1,qian_ess,qian_eess;
	float zhuanxiang;
	float ThrustAllocDirect,ThrustAllocYaw,ThrustAllocTotAdd,ThrustAllocTotSub,ThrustAllocTotMax;

	 
	int16_t forward1;
	int16_t side1;
	int16_t direct1;
	
	forward1=128-forward;
  throttle1=forward1*p_f/10;
	throttle2=forward1*p_f/10;
	ThrustAllocDirect=forward1*p_f/10;
	
	side1=128-side;
	throttle1-=side1*p_s/10;
	throttle2+=side1*p_s/10;
	
	ThrustAllocYaw=side1*p_s/10;
	
	direct1=0;
	
//	
//	if(direct==4)
//		direct1=128;   // with 128 before change
//	else if(direct==3)
//		direct1=-128;
//	else if(direct==2) // with 4 before change
//		direct1=64;
//	else if(direct==1)
//		direct1=-64;
//	else
//		direct1=0;
	
	
	
/*************************第一种方案************************************/	
// if(direct==0&&Qz>20&&Qz<340&&start==1)    //没有转向操作，就进行闭环
//	{
//	  qian_ess=zhuanxiang1-Qz;
//	  if(qian_ess>-20&&qian_ess<20)
//		  qian_ess=0;
//	  else
//	    {
//		  if(qian_ess>0)
//		  qian_ess=qian_ess-20;
//		  if(qian_ess<0)
//		  qian_ess=qian_ess+20;
//		}
//	  qian_intre+=qian_ess;
//	  qian_eess=qian_ess-qian_ess1;
//	  qian_ess1=qian_ess;
//	  direct1=zhi.kp*qian_ess+zhi.ki*qian_intre+zhi.kd*qian_eess;
//	}
//	else
//	{
//	  zhuanxiang1=Qz;
//      direct1=direct1;
//	}
//	
//	pid_count++;
//	if(pid_count>200)
//	{
//	   pid_count=0;
//	   zhuanxiang1=Qz;
//	}


/*************************第二种方案************************************/	

	if(((Rx_Buffer1[3]&7)==3)||((Rx_Buffer1[3]&7)==1))    //没有转向操作，就进行闭环

	{
			zhuanxiang_1=Rx_Buffer1[13]<<8;		zhuanxiang=(float)(zhuanxiang_1|Rx_Buffer1[12])/1000.0;
		
		if(!((hpid_rest_1==Rx_Buffer1[13])&&(hpid_rest_2==Rx_Buffer1[12])))
		{
			qian_intre=0;
			qian_ess1=0;
			hpid_rest_1=Rx_Buffer1[13];
			hpid_rest_2=Rx_Buffer1[12];
			
		}
		zhuanxiang_test=zhuanxiang;
	  zhuanxiang1=Qz;
	  qian_ess=zhuanxiang-zhuanxiang1;
		
		while(qian_ess>180)
			qian_ess=qian_ess-360;
		
				while(qian_ess<-180)
		
				qian_ess=qian_ess+360;
		zhuanxiang=zhuanxiang1;
		
	

		
//	  if(qian_ess>-5&&qian_ess<5)
//		  qian_ess=0;
//	  else
//	    {
//		  if(qian_ess>0)
//		  qian_ess=qian_ess-5;
//		  if(qian_ess<0)
//		  qian_ess=qian_ess+5;
//		}
//	
//	  if(qian_ess<-20)
//		  qian_ess=-20;
//	  if(qian_ess>20)
//		  qian_ess=20;
	  
	  qian_intre+=qian_ess;
	  qian_eess=qian_ess-qian_ess1;
	   qian_ess1=qian_ess;
	 direct1=zhi.kp*qian_ess+zhi.ki*qian_intre+zhi.kd*qian_eess;	
//	  if(direct1>100)
//        direct1=100;
//	  if(direct1<-100)
//        direct1=-100;
	}
	if(qian_intre>5000)
		qian_intre=5000;
	else if(qian_intre<-5000)
				qian_intre=-5000;

//direct2=direct1;

	
	throttle1-=direct1*p_d/10;
	throttle2+=direct1*p_d/10;
	
		ThrustAllocYaw+=direct1*p_d/10;
	
	ThrustAllocTotAdd=fabs(ThrustAllocYaw+ThrustAllocDirect);
	ThrustAllocTotSub=fabs(ThrustAllocYaw-ThrustAllocDirect);
	
	ThrustAllocTotMax=(ThrustAllocTotAdd<ThrustAllocTotSub) ? ThrustAllocTotSub : ThrustAllocTotAdd;
	
	if(ThrustAllocTotMax>400)
	{
		throttle1=throttle1/ThrustAllocTotMax*400;
		throttle2=throttle2/ThrustAllocTotMax*400;
	}

}










































