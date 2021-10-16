#include "include.h"
#include "Ano_FlightDataCal.h"
#include "Drv_RcIn.h"

void NMI_Handler(void)
{
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

void EXTI9_5_IRQHandler(void)  
{  
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)  
    {  
      EXTI_ClearITPendingBit(EXTI_Line7);  
			
			//Fc_Sensor_Get();
    }  
}


void TIM3_IRQHandler(void)
{
	PPM_IRQH();
}

void TIM4_IRQHandler(void)
{
	

}

void USART1_IRQHandler(void)
{
	Uart1_GPS_IRQ();
}

void USART2_IRQHandler(void)
{
	Usart2_IRQ();
}

void USART3_IRQHandler(void)
{
	Usart3_IRQ();
}

void UART4_IRQHandler(void)
{
	Uart4_IRQ();
}

void UART5_IRQHandler(void)
{
	Uart5_IRQ();
}
void USART6_IRQHandler(void)
{
	Sbus_IRQH();
}
