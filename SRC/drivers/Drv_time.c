/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：定时器驱动和滴答配置
**********************************************************************************/

#include "Drv_time.h"
#include "include.h"
#include "Drv_led.h"

#define SYS_TIMx					TIM2
#define SYS_RCC_TIMx			RCC_APB1Periph_TIM2

void TIM_CONF()   //APB1  84M
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* 使能时钟 */
    RCC_APB1PeriphClockCmd ( SYS_RCC_TIMx, ENABLE );

    TIM_DeInit ( SYS_TIMx );

    /* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Period = 1000;

    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    /* 时钟预分频数为72 */
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;

    /* 对外部时钟进行采样的时钟分频,这里没有用到 */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数

    TIM_TimeBaseInit ( SYS_TIMx, &TIM_TimeBaseStructure );

    TIM_ClearFlag ( SYS_TIMx, TIM_FLAG_Update );

    TIM_ITConfig ( SYS_TIMx, TIM_IT_Update, ENABLE );


    TIM_Cmd ( SYS_TIMx, ENABLE );

    RCC_APB1PeriphClockCmd ( SYS_RCC_TIMx , DISABLE );		/*先关闭等待使用*/
}
void TIM_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure;

//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIME_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIME_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );
}

void TIM_INIT()
{
    TIM_CONF();
    TIM_NVIC();

    /* TIM2 重新开时钟，开始计时 */
    RCC_APB1PeriphClockCmd ( SYS_RCC_TIMx , ENABLE );
}


volatile uint32_t sysTickUptime = 0;

void  SysTick_Configuration ( void )
{
    RCC_ClocksTypeDef  rcc_clocks;
    uint32_t         cnts;

    RCC_GetClocksFreq ( &rcc_clocks );

    cnts = ( uint32_t ) rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
    cnts = cnts / 8;

    SysTick_Config ( cnts );
    SysTick_CLKSourceConfig ( SysTick_CLKSource_HCLK_Div8 );
}

uint32_t GetSysTime_us ( void )
{
    register uint32_t ms;
    u32 value;
	do
	{
    ms = sysTickUptime;
    value = ms * TICK_US + ( SysTick->LOAD - SysTick->VAL ) * TICK_US / SysTick->LOAD;
	}
	while(ms != sysTickUptime);
	return value;
}

void Delay_us ( uint32_t us )
{
    uint32_t now = GetSysTime_us();
    while ( GetSysTime_us() - now < us );
}

void Delay_ms ( uint32_t ms )
{
    while ( ms-- )
        Delay_us ( 1000 );
}

u32 systime_ms;

void sys_time()
{
	systime_ms++;
}
u32 SysTick_GetTick(void)
{
	return systime_ms;
}



u32 Get_Delta_T(_get_dT_st *data)
{
    data->old = data->now;	//上一次的时间
    data->now = GetSysTime_us(); //本次的时间
    data->dT = ( ( data->now - data->old ) );//间隔的时间（周期）
	
	if(data->init_flag == 0)
	{
		data->init_flag = 1;//第一次调用时输出 0 作为初始化，以后正常输出
		return 0;
	}
	else
	{
    return data->dT;
	}
}

void SysTick_Handler(void)
{
	sysTickUptime++;
	sys_time();
	LED_1ms_DRV();
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/



