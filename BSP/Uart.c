#include "Uart.h" 


uint8_t TxCount=0;
uint8_t Count=0;
uint8_t com;

volatile uint8_t Tx_Buff1[26],Tx_Buff2[5];	
volatile uint8_t Rx_Buffer1[26],Rx_Buffer2[26],Rx_Buffer3[20];//���ڽ�����
volatile uint8_t com_data1;//����1�������ݻ�����
volatile uint8_t com_data2;//����3�������ݻ�����
float shendu_actual,temp;
volatile uint8_t USART_TX_BUF[26];//���ڷ�����

/*�������´���,֧��printf����,������Ҫѡ��use MicroLIB*/	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x)
{ 
	x = x; 
} 
/*�ض���fputc���� */
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0)
		;//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
ch = ch;
}


/******************************************************************************
����ԭ�ͣ�	void Uart1_Init(uint32_t baud)
��    �ܣ�	���ڳ�ʼ��
��    ����  baud������
*******************************************************************************/ 
void Uart1_Init(uint32_t baud)
{
    //GPIO�˿�����
    GPIO_InitTypeDef  GPIO_InitStructure;
	  USART_InitTypeDef  USART_InitStructure;
	 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
    //USART1_RX	  GPIOA.10��ʼ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

    //USART ��ʼ������
	  USART_InitStructure.USART_BaudRate = baud;//���ڲ�����
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	  USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(USART1, ENABLE); //ʹ�ܴ���1 

}



uint8_t state_machine1,lencnt1;
void USART1_IRQHandler(void) //����1�жϷ������       �����źŽ���
{
	u8 Res;
  com=1;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
  {
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		if(state_machine1 == 0)// Э�����״̬��
		{     
				if(Res == 0xdd)// ���յ�֡ͷǰ��������        
			{	
			    Rx_Buffer1[lencnt1++] = Res; // ���ݱ���
				if(lencnt1>2)
				  state_machine1 = 1; 
			}				
				else         
				{state_machine1= 0;// ״̬����λ
				lencnt1=0;
				}
			} 
		else if(state_machine1 == 1)//���ܿ�������Rx_Buffer1[3]-Rx_Buffer1[23]
		{   
				   Rx_Buffer1[lencnt1++] = Res; // ���ݱ���
				   if(lencnt1>23) //���ݽ������
				   { 
				   	   state_machine1 = 2;
				   	   lencnt1=0;
				   }				   
				 
		} 
		else if(state_machine1 == 2)
		{   
								 
					state_machine1= 0;// ״̬����λ
					lencnt1=0;
		}
  } 

} 






/******************************************************************************
����ԭ�ͣ�	void Uart3_Init(uint32_t baud)
��    �ܣ�	���ڳ�ʼ��
��    ����  baud������
*******************************************************************************/ 
void Uart3_Init(uint32_t baud)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	
	/* ��GPIO��USART������ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	/* ��USART Tx:PB10��GPIO����Ϊ���츴��ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* ��USART Rx:PB11��GPIO����Ϊ��������ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* ����USART���� */
	USART_InitStructure.USART_BaudRate = baud;//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��λ����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һλֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//��������żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ��������ʧ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//����+����ģʽ
	USART_Init(USART3, &USART_InitStructure);
	/* ʹ�ܴ��ڽ����ж� */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	/* ��5����ʹ�� USART�� ������� */
	USART_Cmd(USART3, ENABLE);
	
}


//imu

uint8_t state_machine2,lencnt2=0;
void USART3_IRQHandler(void) //����3�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
	{
		Res =USART_ReceiveData(USART3);	//��ȡ���յ�������
	   if(state_machine2 == 0)// Э�����״̬��
		{     
			if(Res == 0x59)// ���յ�֡ͷ��һ������        
			{	
			    Rx_Buffer2[lencnt2++] = Res; // ���ݱ���
				  state_machine2 = 1; 
			}				
			else         
			{state_machine2 = 0;// ״̬����λ
								lencnt2=0;

			}
			} 
		else if(state_machine2 == 1)
			   {     
				   if(Res==0x53)// ���յ�֡ͷ�ڶ�������  
				   {   
						 state_machine2 = 2;     
						 Rx_Buffer2[lencnt2++] = Res;// ���ݱ���   
				   }
				   else         
					 {state_machine2 = 0;// ״̬����λ
					 								lencnt2=0;

					 }
					 } 
		else if(state_machine2 == 2)
					  {     
						 	Rx_Buffer2[lencnt2++] = Res; // ���ݱ���
						    if(lencnt2>=21) //���ݽ������
							{ 
								 //���ݱ������
								state_machine2 = 0;// ״̬����λ
								lencnt2=0;
					        }
					  }
		  		 
     } 

} 




//depth gauge
/******************************************************************************
����ԭ�ͣ�	void Uart2_Init(uint32_t baud)
��    �ܣ�	���ڳ�ʼ��
��    ����  baud������
*******************************************************************************/ 
void Uart2_Init(uint32_t baud)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	
	/* ��GPIO��USART������ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	/* ��USART Tx:PA2��GPIO����Ϊ���츴��ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* ��USART Rx:PA3��GPIO����Ϊ��������ģʽ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* ����USART���� */
	USART_InitStructure.USART_BaudRate = baud;//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��λ����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һλֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//��������żУ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ��������ʧ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//����+����ģʽ
	USART_Init(USART2, &USART_InitStructure);
	/* ʹ�ܴ��ڽ����ж� */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	/* ��5����ʹ�� USART�� ������� */
	USART_Cmd(USART2, ENABLE);
	
}



//54 3D 32 37 2E 39 35 44 3D 2D 30 2E 30 31 0D 
uint8_t state_machine3,lencnt3=0;
void USART2_IRQHandler(void) //����2�жϷ������
{
	u8 Res;
  com=1;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
  {
		Res =USART_ReceiveData(USART2);	//��ȡ���յ������� 
		if(state_machine3 == 0)// Э�����״̬��
		{     
				if(Res == 0x54)// head receive
				{
					Rx_Buffer3[lencnt3++] = Res; // data restore
				  state_machine3 = 1; 
				}
						
				else         
				{
					state_machine3= 0;
				  lencnt3=0;
				}// ״̬����λ
		} 
		else if(state_machine3 == 1)//
		{   
				   Rx_Buffer3[lencnt3++] = Res; // ���ݱ���
				   if(Res == 0x0D) //���ݽ������
				   { 
				   	   state_machine3 = 0;
				   	   lencnt3=0;
				   }				   
				 
		} 
	}
//	else if(state_machine3 == 2)
//	{   
//							 
//				state_machine3= 0;// ״̬����λ
//				lencnt3=0;
//	}
} 


















//uint8_t state_machine1,lencnt1;
///******************************************************************************
//����ԭ�ͣ�	void USART1_IRQHandler(void)
//��    �ܣ�	�����ж�
//*******************************************************************************/ 
//void USART1_IRQHandler(void)
//{
////	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//ORE�ж�
////	{
////		USART_ReceiveData(USART1);
////	}
////	
////	if(USART1->SR & USART_SR_TC)//�����ж�
////	{
////		USART1->DR = Tx_Buff1[TxCount++];//дDR����жϱ�־          
////		if(TxCount == Count)
////		{
////			USART1->CR1 &= ~USART_CR1_TXEIE;//�ر�TXE�ж�
////		}
////	}
//	
//	if(USART1->SR & USART_SR_RXNE)//�����ж� 
//	{		
//		com_data1 = USART1->DR;

//		
//		
////		USART3->DR = com_data1;		
////		if(state_machine1 == 0)// Э�����״̬��
////		{     
////			if(com_data1 == 0x58)// ���յ�֡ͷ��һ������        
////			{	
////			    Rx_Buffer1[lencnt1++] = com_data1; // ���ݱ���
////				state_machine1 = 1; 
////			}				
////			else         
////				state_machine1= 0;// ״̬����λ
////		} else if(state_machine1 == 1)
////			   {   
////                   Rx_Buffer1[lencnt1++] = com_data1; // ���ݱ���
////				   if(lencnt1>=13) //���ݽ������
////				   { 
////				   	   
////				   	   state_machine1 = 2;
////				   	   lencnt1=0;
////				   }				   
////				 
////			   } else if(state_machine1 == 2)
////			          {   
////			               if(com_data1 == 0x0d)           
////			          	        state_machine1= 3; 		
////			               else         
////			          	        state_machine1= 0;// ״̬����λ
////		              }else if(state_machine1 == 3)
////			                 {   
////			                       if(com_data1 == 0x0a)          
////								   { 
////									   state_machine1= 4; 
////									   Rx_Buffer1[13] = 0x88; //���ݱ������
////								   }
////			                       else         
////			          	                  state_machine1= 0;// ״̬����λ
////		                     }
//		
//	
//	}
//}


///******************************************************************************
//����ԭ�ͣ�	void Uart3_Init(uint32_t baud)
//��    �ܣ�	���ڳ�ʼ��
//��    ����  baud������
//*******************************************************************************/ 
//void Uart3_Init(uint32_t baud)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	
//	/* ��GPIO��USART������ʱ�� */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//	
//	/* ��USART Tx��GPIO����Ϊ���츴��ģʽ */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	/* ��USART Rx��GPIO����Ϊ��������ģʽ */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	/* ����USART���� */
//	USART_InitStructure.USART_BaudRate = baud;//������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//��λ����λ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һλֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//��������żУ��
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ��������ʧ��
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//����+����ģʽ
//	USART_Init(USART3, &USART_InitStructure);
//	
//	/* ʹ�ܴ��ڽ����ж� */
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//	
//	/* ��5����ʹ�� USART�� ������� */
//	USART_Cmd(USART3, ENABLE);
//	
//	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
//		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
//	USART_ClearFlag(USART3, USART_FLAG_TC);//�巢����ɱ�־λ
//}


//uint8_t state_machine2=0,lencnt2;
///******************************************************************************
//����ԭ�ͣ�	void USART3_IRQHandler(void)
//��    �ܣ�	�����ж�
//*******************************************************************************/ 
//void USART3_IRQHandler(void)
//{
//	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//ORE�ж�
//	{
//		USART_ReceiveData(USART3);
//	}
//	
//	if(USART3->SR & USART_SR_TC)//�����ж�
//	{
//		USART3->DR = Tx_Buff2[TxCount++];//дDR����жϱ�־          
//		if(TxCount == Count)
//		{
//			USART3->CR1 &= ~USART_CR1_TXEIE;//�ر�TXE�ж�
//		}
//	}
//	
//	if(USART3->SR & USART_SR_RXNE)//�����ж� 
//	{
//		com_data2 = USART3->DR;

//		
//		
//		
////		USART1->DR = com_data2;		
////	    if(state_machine2 == 0)// Э�����״̬��
////		{     
////			if(com_data2 == 0x55)// ���յ�֡ͷ��һ������        
////			{	
////			    Rx_Buffer2[lencnt2++] = com_data2; // ���ݱ���
////				state_machine2 = 1; 
////			}				
////			else         
////				state_machine2 = 0;// ״̬����λ
////		} else if(state_machine2 == 1)
////			   {     
////				   if((com_data2==0x51)||(com_data2==0x52)||(com_data2==0x53))// ���յ�֡ͷ�ڶ�������  
////				   {   
////						 state_machine2 = 2;     
////						 Rx_Buffer2[lencnt2++] = com_data2;// ���ݱ���   
////				   }
////				   else         
////					     state_machine2 = 0;// ״̬����λ
////			   } else if(state_machine2 == 2)
////					  {     
////						 	Rx_Buffer2[lencnt2++] = com_data2; // ���ݱ���
////						    if(lencnt2>=8) //���ݽ������
////							{ 
////								Rx_Buffer2[8] = 0x77; //���ݱ������
////								state_machine2 = 0;// ״̬����λ
////								lencnt2=0;
////					        }
////					  }
//		
//	}
//}
































////��HEX����ʽ���U8������
//void Printer1HexU8(uint8_t data)
//{
//	Tx_Buff1[Count++] = data;  
//	if(!(USART1->CR1 & USART_CR1_TXEIE))
//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //��TXE�ж�
//}



//void Printer2HexU8(uint8_t data)
//{
//	Tx_Buff2[Count++] = data;  
//	if(!(USART3->CR1 & USART_CR1_TXEIE))
//	USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //��TXE�ж�
//}






////��HEX����ʽ���S16������
//void PrintHexS16(int16_t num)
//{
//	PrintHexU8((uint8_t)(num & 0xff00) >> 8);//�ȷ��͸�8λ���ٷ��͵�8λ
//	PrintHexU8((uint8_t)(num & 0x00ff));
//}
////���ַ�����ʽ���S8������
//void PrintS8(int8_t num)
//{
//	uint8_t  bai,shi,ge;
//	if(num<0)
//	{
//		PrintHexU8('-');
//		num=-num;
//	}
//	else 
//		PrintHexU8(' ');	
//	bai=num/100;
//	shi=num%100/10;
//	ge =num%10;
//	PrintHexU8('0'+bai);
//	PrintHexU8('0'+shi);
//	PrintHexU8('0'+ge);
//}
////���ַ�����ʽ���U8������
//void PrintU8(uint8_t num)
//{
//	uint8_t  bai,shi,ge;
//	bai=num/100;
//	shi=num%100/10;
//	ge =num%10;
//	PrintHexU8('0'+bai);
//	PrintHexU8('0'+shi);
//	PrintHexU8('0'+ge);
//}
////���ַ�����ʽ���S16������ 
//void PrintS16(int16_t num)
//{	
//	uint8_t w5,w4,w3,w2,w1;
//	if(num<0)
//	{
//		PrintHexU8('-');
//		num=-num;
//	}
//	else 
//		PrintHexU8(' ');
//	
//	w5=num%100000/10000;
//	w4=num%10000/1000;
//	w3=num%1000/100;
//	w2=num%100/10;
//	w1=num%10;
//	PrintHexU8('0'+w5);
//	PrintHexU8('0'+w4);
//	PrintHexU8('0'+w3);
//	PrintHexU8('0'+w2);
//	PrintHexU8('0'+w1);
//}
////���ַ�����ʽ���U16������
//void PrintU16(uint16_t num)
//{	
//	uint8_t w5,w4,w3,w2,w1;
//	
//	w5=num%100000/10000;
//	w4=num%10000/1000;
//	w3=num%1000/100;
//	w2=num%100/10;
//	w1=num%10;
//	PrintHexU8(' ');
//	PrintHexU8('0'+w5);
//	PrintHexU8('0'+w4);
//	PrintHexU8('0'+w3);
//	PrintHexU8('0'+w2);
//	PrintHexU8('0'+w1);
//}
////����ַ���
//void PrintString(uint8_t *s)
//{
//	uint8_t *p;
//	p=s;
//	while(*p != '\0')
//	{
//		PrintHexU8(*p);
//		p++;
//	}
//}












