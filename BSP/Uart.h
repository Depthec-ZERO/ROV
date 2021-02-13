#ifndef _UART_H_
#define _UART_H_
#include "stm32f10x.h"
#include "stdio.h" 


#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;     
extern uint8_t com;
/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void Uart1_Init(uint32_t baud);
void Uart2_Init(uint32_t baud);
void Uart3_Init(uint32_t baud);

//void Printer1HexU8(uint8_t data);
//void Printer2HexU8(uint8_t data);


#endif


















//void PrintHexU8(uint8_t data);
//void PrintHexS16(int16_t num);
//void PrintS8( int8_t num);
//void PrintU8(uint8_t num);
//void PrintS16( int16_t num);
//void PrintU16(uint16_t num);
//void PrintString(uint8_t *s);
















