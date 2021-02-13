#ifndef _UART_H_
#define _UART_H_
#include "stm32f10x.h"
#include "stdio.h" 


#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;     
extern uint8_t com;
/******************************************************************************
							ȫ�ֺ�������
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
















