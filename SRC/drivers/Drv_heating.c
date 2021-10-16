#include "Drv_heating.h"
#include "Ano_Imu_Data.h"

void Drv_HeatingInit(void)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd ( RCC_APB2Periph_TIM10, ENABLE );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );

    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource8, GPIO_AF_TIM10 );
	
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 100;
    TIM_TimeBaseStructure.TIM_Prescaler = 840 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit ( TIM10, &TIM_TimeBaseStructure );


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OCInitStructure.TIM_Pulse = 5;
    TIM_OC1Init ( TIM10, &TIM_OCInitStructure );
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig ( TIM10, ENABLE );

    TIM_Cmd ( TIM10, ENABLE );
}

void Drv_HeatingSet(u8 val)
{
	if(val > 99)
		TIM10->CCR1 = 99;
	else
		TIM10->CCR1 = val;
}

#include "include.h"
#include "Ano_math.h"

//用上位机设置恒温功能开关  
//#define USE_THERMOSTATIC 
/*
警告：开启加热恒温功能时一定不能使Thermostatic_Ctrl_Task函数停滞运行，
若出现任何程序异常或者debug操作使Thermostatic_Ctrl_Task停滞运行，
都可能造成飞控硬件损坏，请谨慎使用。
*/


#define EXP_TEMPERATURE 60
#define TEMPERATURE_KP 2.5f
#define TEMPERATURE_KI 5.0f
#define TEMPERATURE_KD 3.5f

//float test_temperature_ctrl_arg[3] ;
float temperature_fb[3],temperature_err,temperature_err_i,temperature_diff,temperature_ctrl_val;
static u16 temperature_cnt;
static u8 thermostatic_en;
void Thermostatic_Ctrl_Task(u8 dT_ms)
{
	//解锁前才允许操作
	if(flag.unlock_sta == 0)
	{
		if(Ano_Parame.set.heatSwitch == 1)//开启恒温功能
		{
			if(thermostatic_en == 0)
			{
				//
				thermostatic_en = 1;
				//复位完成标记
				flag.mems_temperature_ok = 0;
				//
//				sensor.acc_z_auto_CALIBRATE = 1; //重新对准Z轴
//				sensor.gyr_CALIBRATE = 2;//重新校准陀螺仪
				//
				AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"Thermostatic ON......");	
			}
		}
		else
		{
			if(thermostatic_en)
			{
				//
				thermostatic_en = 0;
				//
				AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"Thermostatic OFF,Please Restart ANO_Pioneer_pro!");	
			}
		}
	}
	
	//
	if(thermostatic_en)
	{
		//上次反馈
		temperature_fb[1] = temperature_fb[0];
		//本次反馈
		temperature_fb[0] = st_imuData.f_temperature;//sensor.Tempreature_C;
		//微分
		temperature_diff = (temperature_fb[0] - temperature_fb[1]) *1000/dT_ms;
		//微分先行（用微分预测反馈）
		temperature_fb[2] = temperature_fb[0] + temperature_diff *TEMPERATURE_KD ;//*test_temperature_ctrl_arg[2];
		//计算偏差
		temperature_err = EXP_TEMPERATURE - temperature_fb[2];
		//-----
		if(1)//((temperature_ctrl_val)<100)
		{
			//积分偏差限幅
			temperature_err_i += LIMIT(temperature_err,-10,10) *dT_ms *0.001f;
			//积分限幅
			temperature_err_i = LIMIT(temperature_err_i,-20,20);
		}
		//计算控制输出量
		temperature_ctrl_val = 
		TEMPERATURE_KP *temperature_err // *test_temperature_ctrl_arg[0]
		+ TEMPERATURE_KI *temperature_err_i;// *test_temperature_ctrl_arg[1];
		//
		temperature_ctrl_val = LIMIT(temperature_ctrl_val ,0,100);
		//恒温控制量输出
		Drv_HeatingSet((u8)temperature_ctrl_val); 
		//判断是否完成恒温的升温过程
		if(flag.mems_temperature_ok == 0)
		{		
			if(temperature_fb[0] > (EXP_TEMPERATURE-0.3f) && temperature_fb[0] < (EXP_TEMPERATURE+0.3f))
			{
				//偏差小鱼0.3摄氏度，且持续1500ms，判定升温完成
				if(temperature_cnt<1500)
				{
					temperature_cnt += dT_ms;
				}
				else
				{

						//
						flag.mems_temperature_ok = 1;
						//
						AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"Thermostatic OK!");	
					
				}
			}
			else
			{
				temperature_cnt = 0;
			}
		}
	}
	else //不使用恒温功能
	{
		flag.mems_temperature_ok = 1;
		Drv_HeatingSet((u8)0); 
	}

}

