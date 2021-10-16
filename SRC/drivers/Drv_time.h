#ifndef _TIME_H_
#define _TIME_H_

#include "stm32f4xx.h"

#define TICK_PER_SECOND	1000
#define TICK_US	(1000000/TICK_PER_SECOND)

typedef struct
{
	u8 init_flag;
	u32 old;
	u32 now;
	u32 dT;

}	_get_dT_st;

void TIM_INIT(void);
void sys_time(void);

u16 Get_Time(u8,u16,u16);

u32 Get_Delta_T(_get_dT_st * );
u32 SysTick_GetTick(void);

void Cycle_Time_Init(void);

extern volatile uint32_t sysTickUptime;

void Delay_us(uint32_t);
void Delay_ms(uint32_t);
void SysTick_Configuration(void);
uint32_t GetSysTime_us(void);
#endif
