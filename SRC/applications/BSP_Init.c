/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：飞控初始化
**********************************************************************************/
#include "include.h"
#include "Drv_pwm_out.h"
#include "Drv_led.h"
#include "Drv_spi.h"
#include "Drv_icm20602.h"
#include "drv_ak8975.h"
#include "drv_ak09915.h"
#include "drv_spl06.h"
#include "Drv_w25qxx.h"
#include "Drv_i2c_soft.h"
#include "Drv_laser.h"
#include "Ano_FlightCtrl.h"
#include "Drv_adc.h"
#include "Drv_heating.h"
#include "Ano_RC.h"
#include "Ano_Sensor_Basic.h"
#include "Drv_UP_Flow.h"
#include "Ano_DT.h"
#include "Drv_bmi088.h"
#include "Ano_Imu_Data.h"
#include "Drv_BSP.h"

u8 MagType = MAG_NUL;

void CheckSenser(void)
{
	if(DrvAK09915Check())
		MagType = MAG_AK09915;
	else if(DrvAK8975Check())
		MagType = MAG_AK8975;
	else
		MagType = MAG_NUL;
}

u8 of_init_type;
u8 All_Init()
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置

	SysTick_Configuration(); 		//滴答时钟
	
	Delay_ms(100);					//延时
	Drv_LED_Init();					//LED功能初始化
	
	Flash_Init();             		//板载FLASH芯片驱动初始化
	
	Para_Data_Init();              		//参数数据初始化
	
	//接收机初始化
	DrvRcInputInit();
	
	PWM_Out_Init();					//初始化电调输出功能	
	Delay_ms(50);					//延时
	
	Drv_SPI2_init();          		//spi_2初始化，用于读取飞控板上所有传感器，都用SPI读取
	Drv_AK8975CSPin_Init();   		//spi片选初始化
	Drv_SPL06CSPin_Init();    		//spi片选初始化
	DrvGpioSenser088CsPinInit();
	Delay_ms(10);					//延时
	CheckSenser();
	//
	sens_hd_check.gyro_ok = sens_hd_check.acc_ok = 
	DrvBmi088Init();	//陀螺仪加速度计初始化，若初始化成功，则将陀螺仪和加速度的初始化成功标志位赋值
	//
	if(MagType == MAG_NUL)
	{
		sens_hd_check.mag_ok = 0;
		LED_STA.errMag = 1;
	}
	else
	{
		sens_hd_check.mag_ok = 1;       //标记罗盘OK
		if(MagType == MAG_AK09915)
		{
			DrvAk09915Init();
		}
	}
	//
	sens_hd_check.baro_ok = Drv_Spl0601_Init();       		//气压计初始化

	Usb_Hid_Init();					//飞控usb接口的hid初始化
	Delay_ms(100);					//延时
	
	Usart2_Init(115200);			//串口2初始化，函数参数为波特率    ks103波特率9600  激光波特率115200
	Delay_ms(10);					//延时	
//	Uart4_Init(115200);				//首先判断是否连接的是激光模块
//	if(!Drv_Laser_Init())			//激光没有有效连接，则配置为光流模式
//		Uart4_Init(500000);
//	Delay_ms(10);					//延时
//	Usart3_Init(500000);			//连接UWB
//	Delay_ms(10);					//延时
	//
	Usart3_Init(500000);			//连接OPENMV
	//	
//	Uart4_Init(19200);	//接优像光流
//  Uart5_Init(115200);//接大功率激光	
	Uart5_Init(500000); //数传
//	MyDelayMs(200);	
	//优像光流初始化
//	of_init_type = (Drv_OFInit()==0)?0:2;
//	if(of_init_type==2)//优像光流初始化成功
//	{
//		//大功率激光初始化
//		Drv_Laser_Init();	
//	}
//	else if(of_init_type==0)//优像光流初始化失败
//	{
		Uart4_Init(500000);	//接匿名光流
//	}
	//

	Drv_AdcInit();
	Delay_ms(100);					//延时

	All_PID_Init();               		//PID初始化
	
	Delay_ms(100);					//延时		
	Drv_GpsPin_Init();				//GPS初始化 串口1
	
	Drv_HeatingInit();
	//
	Sensor_Basic_Init();
	//
	ANO_DT_Init();
	//传感器灵敏度初始化
	ImuSensitivityInit(Ano_Parame.set.acc_calibrated,(float *)Ano_Parame.set.acc_sensitivity_ref);//test_st.test);//
	//
	AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"SYS init OK!");
	return (1);
}

void DrvGpioCsPinCtrlBmi088Acc(u8 ena)
{
	if ( ena )
        GPIO_ResetBits ( BMI088_CSPOT_ACC, BMI088_CSPIN_ACC );
    else
        GPIO_SetBits ( BMI088_CSPOT_ACC, BMI088_CSPIN_ACC );
}
void DrvGpioCsPinCtrlBmi088Gyr(u8 ena)
{
	if ( ena )
        GPIO_ResetBits ( BMI088_CSPOT_GYR, BMI088_CSPIN_GYR );
    else
        GPIO_SetBits ( BMI088_CSPOT_GYR, BMI088_CSPIN_GYR );
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
