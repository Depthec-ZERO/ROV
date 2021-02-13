/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
#include "Tasks.h"
#include "Timer3.h"



uint8_t Count_10ms,Count_50ms;
float throttle11 = 0,throttle21 = 0,throttle51 = 0,throttle61 = 0;
extern float throttle1,throttle2,throttle3,throttle4,throttle5,throttle6;

void TIM3_IRQHandler(void)//Timer3中断
{	
	if(TIM3->SR & TIM_IT_Update)
	{     
		TIM3->SR = ~TIM_FLAG_Update;//清除中断标志
		
		if( Bsp_Int_Ok == 0 )  return;//硬件未初始化完成，则返回
		Timer3_Count++;
		Count_10ms++;
		Count_50ms++;
		if(throttle1 != throttle11  )          
		{ 
			if(throttle11 < throttle1)
			throttle11 = throttle11 + 1 ;
			if(throttle11 > throttle1)  
			throttle11 = throttle11 - 1 ;
		}
		
       
		
		    if (throttle2 != throttle21  )          
		{ 
			if(throttle21 < throttle2)
			throttle21 = throttle21 + 1 ;
			if(throttle21 > throttle2)  
			throttle21 = throttle21 - 1 ;
			 
    }
		
      
		  if(throttle5 != throttle51  )          
		{ 
			if(throttle51 < throttle5)
			throttle51 = throttle51 + 1 ;
			if(throttle51 > throttle5)  
			throttle51 = throttle51 - 1 ;
			 
      
      			
		}
		
      
		if(throttle6 != throttle61  )          
		{ 
			if(throttle61 < throttle6)
			throttle61 = throttle61 + 1 ;
			if(throttle61 > throttle6)  
			throttle61 = throttle61 - 1 ;
			 
      
      			
		}
	}
}









 
void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
