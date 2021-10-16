#ifndef _INCLUDE_H_
#define _INCLUDE_H_

//#include "stm32f4xx.h"
#include "Ano_FcData.h"
#include "Ano_Scheduler.h"
#include "BSP_Init.h"
#include "Ano_DT.h"
#include "Ano_Parameter.h"
#include "Ano_USB.h"
#include "Drv_time.h"
#include "Drv_usart.h"
#include "Drv_Gps.h"
#include "Drv_BSP.h"

//================系统===================
#define HW_ALL		 0xFF
#define HW_TYPE	05
#define SWJ_ADDR 	 0xAF
#define HW_VER	1
#define SW_VER  19
#define BL_VER	0
#define PT_VER	400


#define ANO_DT_USE_USART2 				//开启串口2数传功能
#define ANO_DT_USE_USB_HID				//开启飞控USBHID连接上位机功能
//=======================================
/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_3		//中断分组选择

#define NVIC_PWMIN_P			1			//接收机采集中断配置
#define NVIC_PWMIN_S			0

#define NVIC_TIME_P       2					//定时器中断配置，暂未使用
#define NVIC_TIME_S       1

#define NVIC_UART6_P				4			//串口5中断配置
#define NVIC_UART6_S				1

#define NVIC_UART5_P				4			//串口5中断配置
#define NVIC_UART5_S				1

#define NVIC_UART4_P			2			//串口4中断配置
#define NVIC_UART4_S			1

#define NVIC_UART2_P			2			//串口2中断配置
#define NVIC_UART2_S			0

#define NVIC_UART1_P			3			//串口1中断配置 //gps
#define NVIC_UART1_S			0
/***********************************************/
//================传感器===================
//以下为板载各个传感器的使能引脚配置
#define BMI088_CSPOT_ACC		GPIOD
#define BMI088_CSRCC_ACC		RCC_AHB1Periph_GPIOD
#define BMI088_CSPIN_ACC		GPIO_Pin_0
#define BMI088_CSPOT_GYR		GPIOC
#define BMI088_CSRCC_GYR		RCC_AHB1Periph_GPIOC
#define BMI088_CSPIN_GYR		GPIO_Pin_10
#define AK8975_CS_RCC			RCC_AHB1Periph_GPIOB
#define AK8975_CS_GPIO			GPIOB
#define AK8975_CS_PIN			GPIO_Pin_12
#define SPL06_CS_RCC			RCC_AHB1Periph_GPIOC
#define SPL06_CS_GPIO			GPIOC
#define SPL06_CS_PIN			GPIO_Pin_11


//=========================================
//====
typedef float vec3_f[3];
typedef float vec2_f[2];
typedef s32 vec3_s32[3];
typedef s32 vec2_s32[2];
typedef s16 vec3_s16[3];
typedef s16 vec2_s16[2];

enum _e_magType  {MAG_NUL=0, MAG_AK8975, MAG_AK09915,};

extern u8 MagType;

#endif

