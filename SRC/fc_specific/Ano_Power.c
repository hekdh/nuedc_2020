#include "Ano_Power.h"
#include "Ano_Parameter.h"
#include "Ano_Filter.h"
#include "Drv_led.h"
#include "Ano_Math.h"

float Plane_Votage = 0;
static float voltage_f = 30000;
static u8 voltage_init_ok;
void Power_UpdateTask(u8 dT_ms)
{
	static s16 voltage_s16;
	float cut_off_freq;
	
	voltage_s16 = AdcValue *8.8653f;//1.128f; ;
	
	if(voltage_init_ok == 0)
	{
		cut_off_freq = 2.0f;
		
		if(voltage_f >2000 && ABS(voltage_s16 - voltage_f) <200)
		{
			voltage_init_ok = 1;
		}
	}	
	else
	{
		cut_off_freq = 0.02f;
	}
	
	LPF_1_(cut_off_freq,dT_ms*1e-3f,voltage_s16,voltage_f);
	

	
	Plane_Votage = voltage_f *0.001f;
//	Plane_Votage = 15;


	if(Plane_Votage<1)
	{
		flag.power_state = 0;//没接电池
	}
	else if(Plane_Votage<Ano_Parame.set.lowest_power_voltage*Ano_Parame.set.bat_cell)
	{
		flag.power_state = 3;//将禁止解锁		
	}
	else
	{
		flag.power_state = 1;
	}
	//
	if(Plane_Votage<1)
	{
		LED_STA.lowVt = 0;//没接电池
	}
	else if(Plane_Votage<Ano_Parame.set.warn_power_voltage*Ano_Parame.set.bat_cell)
	{
		LED_STA.lowVt = 1;
	}
	else if(Plane_Votage>Ano_Parame.set.warn_power_voltage*Ano_Parame.set.bat_cell+0.2f)
	{
		LED_STA.lowVt = 0;//电量足
	}
		

}





