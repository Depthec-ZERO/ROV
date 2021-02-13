#include "Uart.h" 


uint8_t TxCount=0;
uint8_t Count=0;
uint8_t com;

volatile uint8_t Tx_Buff1[26],Tx_Buff2[5];	
volatile uint8_t Rx_Buffer1[26],Rx_Buffer2[26],Rx_Buffer3[20];//串口接收区
volatile uint8_t com_data1;//串口1接收数据缓存区
volatile uint8_t com_data2;//串口3接收数据缓存区
float shendu_actual,temp;
volatile uint8_t USART_TX_BUF[26];//串口发送区

/*加入以下代码,支持printf函数,而不需要选择use MicroLIB*/	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x)
{ 
	x = x; 
} 
/*重定义fputc函数 */
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0)
		;//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
ch = ch;
}


/******************************************************************************
函数原型：	void Uart1_Init(uint32_t baud)
功    能：	串口初始化
参    数：  baud波特率
*******************************************************************************/ 
void Uart1_Init(uint32_t baud)
{
    //GPIO端口设置
    GPIO_InitTypeDef  GPIO_InitStructure;
	  USART_InitTypeDef  USART_InitStructure;
	 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
    //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

    //USART 初始化设置
	  USART_InitStructure.USART_BaudRate = baud;//串口波特率
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE); //使能串口1 

}



uint8_t state_machine1,lencnt1;
void USART1_IRQHandler(void) //串口1中断服务程序       控制信号接受
{
	u8 Res;
  com=1;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
  {
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		if(state_machine1 == 0)// 协议解析状态机
		{     
				if(Res == 0xdd)// 接收到帧头前三个数据        
			{	
			    Rx_Buffer1[lencnt1++] = Res; // 数据保存
				if(lencnt1>2)
				  state_machine1 = 1; 
			}				
				else         
				{state_machine1= 0;// 状态机复位
				lencnt1=0;
				}
			} 
		else if(state_machine1 == 1)//接受控制数据Rx_Buffer1[3]-Rx_Buffer1[23]
		{   
				   Rx_Buffer1[lencnt1++] = Res; // 数据保存
				   if(lencnt1>23) //数据接收完毕
				   { 
				   	   state_machine1 = 2;
				   	   lencnt1=0;
				   }				   
				 
		} 
		else if(state_machine1 == 2)
		{   
								 
					state_machine1= 0;// 状态机复位
					lencnt1=0;
		}
  } 

} 






/******************************************************************************
函数原型：	void Uart3_Init(uint32_t baud)
功    能：	串口初始化
参    数：  baud波特率
*******************************************************************************/ 
void Uart3_Init(uint32_t baud)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	
	/* 打开GPIO和USART部件的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	/* 将USART Tx:PB10的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 将USART Rx:PB11的GPIO配置为浮空输入模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 配置USART参数 */
	USART_InitStructure.USART_BaudRate = baud;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//八位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//不进行奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//接收+发送模式
	USART_Init(USART3, &USART_InitStructure);
	/* 使能串口接收中断 */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	/* 第5步：使能 USART， 配置完毕 */
	USART_Cmd(USART3, ENABLE);
	
}


//imu

uint8_t state_machine2,lencnt2=0;
void USART3_IRQHandler(void) //串口3中断服务程序
{
	u8 Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		Res =USART_ReceiveData(USART3);	//读取接收到的数据
	   if(state_machine2 == 0)// 协议解析状态机
		{     
			if(Res == 0x59)// 接收到帧头第一个数据        
			{	
			    Rx_Buffer2[lencnt2++] = Res; // 数据保存
				  state_machine2 = 1; 
			}				
			else         
			{state_machine2 = 0;// 状态机复位
								lencnt2=0;

			}
			} 
		else if(state_machine2 == 1)
			   {     
				   if(Res==0x53)// 接收到帧头第二个数据  
				   {   
						 state_machine2 = 2;     
						 Rx_Buffer2[lencnt2++] = Res;// 数据保存   
				   }
				   else         
					 {state_machine2 = 0;// 状态机复位
					 								lencnt2=0;

					 }
					 } 
		else if(state_machine2 == 2)
					  {     
						 	Rx_Buffer2[lencnt2++] = Res; // 数据保存
						    if(lencnt2>=21) //数据接收完毕
							{ 
								 //数据保存完毕
								state_machine2 = 0;// 状态机复位
								lencnt2=0;
					        }
					  }
		  		 
     } 

} 




//depth gauge
/******************************************************************************
函数原型：	void Uart2_Init(uint32_t baud)
功    能：	串口初始化
参    数：  baud波特率
*******************************************************************************/ 
void Uart2_Init(uint32_t baud)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	
	/* 打开GPIO和USART部件的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	/* 将USART Tx:PA2的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 将USART Rx:PA3的GPIO配置为浮空输入模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 配置USART参数 */
	USART_InitStructure.USART_BaudRate = baud;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//八位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//不进行奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//接收+发送模式
	USART_Init(USART2, &USART_InitStructure);
	/* 使能串口接收中断 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	/* 第5步：使能 USART， 配置完毕 */
	USART_Cmd(USART2, ENABLE);
	
}



//54 3D 32 37 2E 39 35 44 3D 2D 30 2E 30 31 0D 
uint8_t state_machine3,lencnt3=0;
void USART2_IRQHandler(void) //串口2中断服务程序
{
	u8 Res;
  com=1;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
  {
		Res =USART_ReceiveData(USART2);	//读取接收到的数据 
		if(state_machine3 == 0)// 协议解析状态机
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
				}// 状态机复位
		} 
		else if(state_machine3 == 1)//
		{   
				   Rx_Buffer3[lencnt3++] = Res; // 数据保存
				   if(Res == 0x0D) //数据接收完毕
				   { 
				   	   state_machine3 = 0;
				   	   lencnt3=0;
				   }				   
				 
		} 
	}
//	else if(state_machine3 == 2)
//	{   
//							 
//				state_machine3= 0;// 状态机复位
//				lencnt3=0;
//	}
} 


















//uint8_t state_machine1,lencnt1;
///******************************************************************************
//函数原型：	void USART1_IRQHandler(void)
//功    能：	串口中断
//*******************************************************************************/ 
//void USART1_IRQHandler(void)
//{
////	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//ORE中断
////	{
////		USART_ReceiveData(USART1);
////	}
////	
////	if(USART1->SR & USART_SR_TC)//发送中断
////	{
////		USART1->DR = Tx_Buff1[TxCount++];//写DR清除中断标志          
////		if(TxCount == Count)
////		{
////			USART1->CR1 &= ~USART_CR1_TXEIE;//关闭TXE中断
////		}
////	}
//	
//	if(USART1->SR & USART_SR_RXNE)//接收中断 
//	{		
//		com_data1 = USART1->DR;

//		
//		
////		USART3->DR = com_data1;		
////		if(state_machine1 == 0)// 协议解析状态机
////		{     
////			if(com_data1 == 0x58)// 接收到帧头第一个数据        
////			{	
////			    Rx_Buffer1[lencnt1++] = com_data1; // 数据保存
////				state_machine1 = 1; 
////			}				
////			else         
////				state_machine1= 0;// 状态机复位
////		} else if(state_machine1 == 1)
////			   {   
////                   Rx_Buffer1[lencnt1++] = com_data1; // 数据保存
////				   if(lencnt1>=13) //数据接收完毕
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
////			          	        state_machine1= 0;// 状态机复位
////		              }else if(state_machine1 == 3)
////			                 {   
////			                       if(com_data1 == 0x0a)          
////								   { 
////									   state_machine1= 4; 
////									   Rx_Buffer1[13] = 0x88; //数据保存完毕
////								   }
////			                       else         
////			          	                  state_machine1= 0;// 状态机复位
////		                     }
//		
//	
//	}
//}


///******************************************************************************
//函数原型：	void Uart3_Init(uint32_t baud)
//功    能：	串口初始化
//参    数：  baud波特率
//*******************************************************************************/ 
//void Uart3_Init(uint32_t baud)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	
//	/* 打开GPIO和USART部件的时钟 */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//	
//	/* 将USART Tx的GPIO配置为推挽复用模式 */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	/* 将USART Rx的GPIO配置为浮空输入模式 */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	/* 配置USART参数 */
//	USART_InitStructure.USART_BaudRate = baud;//波特率
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//八位数据位
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一位停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//不进行奇偶校验
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制失能
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//接收+发送模式
//	USART_Init(USART3, &USART_InitStructure);
//	
//	/* 使能串口接收中断 */
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//	
//	/* 第5步：使能 USART， 配置完毕 */
//	USART_Cmd(USART3, ENABLE);
//	
//	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
//		如下语句解决第1个字节无法正确发送出去的问题 */
//	USART_ClearFlag(USART3, USART_FLAG_TC);//清发送完成标志位
//}


//uint8_t state_machine2=0,lencnt2;
///******************************************************************************
//函数原型：	void USART3_IRQHandler(void)
//功    能：	串口中断
//*******************************************************************************/ 
//void USART3_IRQHandler(void)
//{
//	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//ORE中断
//	{
//		USART_ReceiveData(USART3);
//	}
//	
//	if(USART3->SR & USART_SR_TC)//发送中断
//	{
//		USART3->DR = Tx_Buff2[TxCount++];//写DR清除中断标志          
//		if(TxCount == Count)
//		{
//			USART3->CR1 &= ~USART_CR1_TXEIE;//关闭TXE中断
//		}
//	}
//	
//	if(USART3->SR & USART_SR_RXNE)//接收中断 
//	{
//		com_data2 = USART3->DR;

//		
//		
//		
////		USART1->DR = com_data2;		
////	    if(state_machine2 == 0)// 协议解析状态机
////		{     
////			if(com_data2 == 0x55)// 接收到帧头第一个数据        
////			{	
////			    Rx_Buffer2[lencnt2++] = com_data2; // 数据保存
////				state_machine2 = 1; 
////			}				
////			else         
////				state_machine2 = 0;// 状态机复位
////		} else if(state_machine2 == 1)
////			   {     
////				   if((com_data2==0x51)||(com_data2==0x52)||(com_data2==0x53))// 接收到帧头第二个数据  
////				   {   
////						 state_machine2 = 2;     
////						 Rx_Buffer2[lencnt2++] = com_data2;// 数据保存   
////				   }
////				   else         
////					     state_machine2 = 0;// 状态机复位
////			   } else if(state_machine2 == 2)
////					  {     
////						 	Rx_Buffer2[lencnt2++] = com_data2; // 数据保存
////						    if(lencnt2>=8) //数据接收完毕
////							{ 
////								Rx_Buffer2[8] = 0x77; //数据保存完毕
////								state_machine2 = 0;// 状态机复位
////								lencnt2=0;
////					        }
////					  }
//		
//	}
//}
































////以HEX的形式输出U8型数据
//void Printer1HexU8(uint8_t data)
//{
//	Tx_Buff1[Count++] = data;  
//	if(!(USART1->CR1 & USART_CR1_TXEIE))
//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开TXE中断
//}



//void Printer2HexU8(uint8_t data)
//{
//	Tx_Buff2[Count++] = data;  
//	if(!(USART3->CR1 & USART_CR1_TXEIE))
//	USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //打开TXE中断
//}






////以HEX的形式输出S16型数据
//void PrintHexS16(int16_t num)
//{
//	PrintHexU8((uint8_t)(num & 0xff00) >> 8);//先发送高8位，再发送低8位
//	PrintHexU8((uint8_t)(num & 0x00ff));
//}
////以字符的形式输出S8型数据
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
////以字符的形式输出U8型数据
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
////以字符的形式输出S16型数据 
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
////以字符的形式输出U16型数据
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
////输出字符串
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












