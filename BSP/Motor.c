#include "Motor.h"
#include "delay.h"
extern float light_t;
/******************************************************************************
函数原型：	static void Tim2_init(void)
功    能：	Tim2初始化
*******************************************************************************/ 
static void Tim2_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  		  	TIM_OCInitStructure;
	
	//启动时钟
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//PWM频率 = 72000000 / 72 / 20000 = 50Hz
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;       //PWM计数上限	 
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;       //设置用来作为TIM3时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;        //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMx向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);     //根据TIM_TimeBaseStructure中指定的参数初始化外设TIM3
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //选择定时器模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                  //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); //使能TIM2在CCR1上的预装载寄存器
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable); //使能TIM2在CCR2上的预装载寄存器
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); //使能TIM2在CCR3上的预装载寄存器
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); //使能TIM2在CCR4上的预装载寄存器
	
	
	TIM_ARRPreloadConfig(TIM2, ENABLE); //使能TIM2在ARR上的预装载寄存器
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2外设
}
/******************************************************************************
函数原型：	static void Tim4_init(void)
功    能：	Tim4初始化
*******************************************************************************/ 
static void Tim4_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  		  	TIM_OCInitStructure;
	
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	//PWM频率 = 72000000 / 72 / 20000 = 50Hz
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;       //PWM计数上限	 
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;       //设置用来作为TIM4时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;        //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMx向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);     //根据TIM_TimeBaseStructure中指定的参数初始化外设TIM4
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //选择定时器模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                  //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能TIM4在CCR1上的预装载寄存器
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能TIM4在CCR2上的预装载寄存器
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能TIM4在CCR3上的预装载寄存器
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能TIM4在CCR4上的预装载寄存器
	
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //使能TIM4在ARR上的预装载寄存器
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4外设
}

/******************************************************************************
函数原型：	void Motor_Init(void)
功    能：	PWM初始化
分    牛：  Pin_4，Pin_5对应左右两个电机  Pin_6 Pin_7 Pin_8 Pin_9对应左上，右上，
*******************************************************************************/ 
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	//使能电机用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
  //设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ; //对应Tim4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  //GPIO_InitTypeDef GPIO_InitStructure;
//使能电机用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
  //设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  ; //对应Tim2 ch1 ch2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	Tim4_init();
	Tim2_init();
}
extern float energe_t,thruster_v0,thruster_v1,thruster_v2,thruster_v3,thruster_v4,thruster_v5,thruster_v6,thruster_v7;
/******************************************************************************
函数原型：	void Motor_Out(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4,int16_t duty_l,int16_t duty_r,int16_t duty_sevro,int16_t duty_light)
功    能：	电机驱动
*******************************************************************************/ 
void Motor(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4,int16_t duty_l,int16_t duty_r,int16_t duty_sevro,int16_t duty_light)
{
	/****************右前电机*******************/
	if(duty1>450)	  duty1=450;
	if(duty1<-450)	duty1=-450;
	/*****************左前电机******************/
	if(duty2>450)  	duty2=450;
	if(duty2<-450)	duty2=-450;
	/*****************左垂电机******************原左后*/
	if(duty3>450)	  duty3=450;
	if(duty3<-450)	duty3=-450;
	/*****************右垂电机******************原右后*/
	if(duty4>450)	  duty4=450;
	if(duty4<-450)	duty4=-450;
//	/******************左垂推电机*****************/
//	if(duty_l>450)	duty_l=450;
//	if(duty_l<-450)	duty_l=-450;
//	/******************右垂推电机****************/
//	if(duty_r>450)	duty_r=450;
//	if(duty_r<-450)	duty_r=-450;
//	
	TIM4->CCR2 = 1505+duty1;           //B7
	TIM4->CCR1 = 1505+duty2;           //B6
	TIM4->CCR3 = 1505+duty3;           //B8
	TIM4->CCR4 = 1505-duty4;           //B9
//	TIM2->CCR1 = 1500+duty_l;           //A0
//	TIM2->CCR2 = 1500+duty_r;           //A1
//	
	thruster_v0=((TIM4->CCR2-1505)+450)/4.5;//pwm占空比×100，0-100为负，100-200为正;
	thruster_v1=((TIM4->CCR1-1505)+450)/4.5;
	thruster_v2=((TIM4->CCR3-1505)+450)/4.5;
	thruster_v3=((TIM4->CCR4-1505)+450)/4.5;

	
}
/******************************************************************************
函数原型：	void sevro_motor(int16_t duty1)
功    能：  舵机控制
*******************************************************************************/ 
void sevro_motor(int16_t duty1)
{
  if(duty1>500)	  duty1=500;
	if(duty1<-400)	duty1=-400;
	
	TIM2->CCR3 = 1500+duty1;
	
}
/******************************************************************************
函数原型：	void light(int16_t duty1)
功    能：  灯光控制
*******************************************************************************/ 
void light(int16_t duty1)
{
  if(duty1>500)	  duty1=500;
	if(duty1<-400)	duty1=-400;
	
	TIM2->CCR4=light_t = 1500+duty1;
	
}
/******************************************************************************
函数原型：	void ad(void)
功    能：  ad采集模块初始化
*******************************************************************************/ 
void ad(void)
{
	
    ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA6 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	
}
/******************************************************************************
函数原型：	void ad(void)
功    能：  ad采集模块初始化
*******************************************************************************/ 
//adcx=Get_Adc_Average(ADC_Channel_1,10);
//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(2);
	}
	return temp_val/times;
} 	






