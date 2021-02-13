#include "Motor.h"
#include "delay.h"
extern float light_t;
/******************************************************************************
����ԭ�ͣ�	static void Tim2_init(void)
��    �ܣ�	Tim2��ʼ��
*******************************************************************************/ 
static void Tim2_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  		  	TIM_OCInitStructure;
	
	//����ʱ��
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//PWMƵ�� = 72000000 / 72 / 20000 = 50Hz
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;       //PWM��������	 
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;       //����������ΪTIM3ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;        //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMx���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);     //����TIM_TimeBaseStructure��ָ���Ĳ�����ʼ������TIM3
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //ѡ��ʱ��ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                  //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable); //ʹ��TIM2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); //ʹ��TIM2��CCR4�ϵ�Ԥװ�ؼĴ���
	
	
	TIM_ARRPreloadConfig(TIM2, ENABLE); //ʹ��TIM2��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2����
}
/******************************************************************************
����ԭ�ͣ�	static void Tim4_init(void)
��    �ܣ�	Tim4��ʼ��
*******************************************************************************/ 
static void Tim4_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  		  	TIM_OCInitStructure;
	
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	//PWMƵ�� = 72000000 / 72 / 20000 = 50Hz
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;       //PWM��������	 
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;       //����������ΪTIM4ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;        //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMx���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);     //����TIM_TimeBaseStructure��ָ���Ĳ�����ʼ������TIM4
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //ѡ��ʱ��ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                  //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��TIM4��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��TIM4��CCR4�ϵ�Ԥװ�ؼĴ���
	
	
	TIM_ARRPreloadConfig(TIM4, ENABLE); //ʹ��TIM4��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4����
}

/******************************************************************************
����ԭ�ͣ�	void Motor_Init(void)
��    �ܣ�	PWM��ʼ��
��    ţ��  Pin_4��Pin_5��Ӧ�����������  Pin_6 Pin_7 Pin_8 Pin_9��Ӧ���ϣ����ϣ�
*******************************************************************************/ 
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	//ʹ�ܵ���õ�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
  //���õ��ʹ�õ��ùܽ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ; //��ӦTim4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  //GPIO_InitTypeDef GPIO_InitStructure;
//ʹ�ܵ���õ�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
  //���õ��ʹ�õ��ùܽ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  ; //��ӦTim2 ch1 ch2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	Tim4_init();
	Tim2_init();
}
extern float energe_t,thruster_v0,thruster_v1,thruster_v2,thruster_v3,thruster_v4,thruster_v5,thruster_v6,thruster_v7;
/******************************************************************************
����ԭ�ͣ�	void Motor_Out(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4,int16_t duty_l,int16_t duty_r,int16_t duty_sevro,int16_t duty_light)
��    �ܣ�	�������
*******************************************************************************/ 
void Motor(int16_t duty1,int16_t duty2,int16_t duty3,int16_t duty4,int16_t duty_l,int16_t duty_r,int16_t duty_sevro,int16_t duty_light)
{
	/****************��ǰ���*******************/
	if(duty1>450)	  duty1=450;
	if(duty1<-450)	duty1=-450;
	/*****************��ǰ���******************/
	if(duty2>450)  	duty2=450;
	if(duty2<-450)	duty2=-450;
	/*****************�󴹵��******************ԭ���*/
	if(duty3>450)	  duty3=450;
	if(duty3<-450)	duty3=-450;
	/*****************�Ҵ����******************ԭ�Һ�*/
	if(duty4>450)	  duty4=450;
	if(duty4<-450)	duty4=-450;
//	/******************���Ƶ��*****************/
//	if(duty_l>450)	duty_l=450;
//	if(duty_l<-450)	duty_l=-450;
//	/******************�Ҵ��Ƶ��****************/
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
	thruster_v0=((TIM4->CCR2-1505)+450)/4.5;//pwmռ�ձȡ�100��0-100Ϊ����100-200Ϊ��;
	thruster_v1=((TIM4->CCR1-1505)+450)/4.5;
	thruster_v2=((TIM4->CCR3-1505)+450)/4.5;
	thruster_v3=((TIM4->CCR4-1505)+450)/4.5;

	
}
/******************************************************************************
����ԭ�ͣ�	void sevro_motor(int16_t duty1)
��    �ܣ�  �������
*******************************************************************************/ 
void sevro_motor(int16_t duty1)
{
  if(duty1>500)	  duty1=500;
	if(duty1<-400)	duty1=-400;
	
	TIM2->CCR3 = 1500+duty1;
	
}
/******************************************************************************
����ԭ�ͣ�	void light(int16_t duty1)
��    �ܣ�  �ƹ����
*******************************************************************************/ 
void light(int16_t duty1)
{
  if(duty1>500)	  duty1=500;
	if(duty1<-400)	duty1=-400;
	
	TIM2->CCR4=light_t = 1500+duty1;
	
}
/******************************************************************************
����ԭ�ͣ�	void ad(void)
��    �ܣ�  ad�ɼ�ģ���ʼ��
*******************************************************************************/ 
void ad(void)
{
	
    ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA6 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

  
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������
	
}
/******************************************************************************
����ԭ�ͣ�	void ad(void)
��    �ܣ�  ad�ɼ�ģ���ʼ��
*******************************************************************************/ 
//adcx=Get_Adc_Average(ADC_Channel_1,10);
//���ADCֵ
//ch:ͨ��ֵ 0~3
u16 Get_Adc(u8 ch)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
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






