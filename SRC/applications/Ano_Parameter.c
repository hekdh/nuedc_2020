/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
  * 作者   ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：参数配置等
**********************************************************************************/
#include "include.h"
#include "Drv_w25qxx.h"
#include "Ano_FlightCtrl.h"
#include "Drv_led.h"
#include "Ano_Imu_Calibration.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_FcData.h"
#include "Ano_Imu_Data.h"
#include "Ano_MagProcess.h"

union Parameter Ano_Parame;
_parameter_state_st para_sta;


void PID_Rest()
{
//---	姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KP] = 4.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KI] = 2.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KD] = 0.2f; //姿态控制角速度环PID参数
	
	Ano_Parame.set.pid_att_1level[PIT][KP] = 4.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[PIT][KI] = 2.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[PIT][KD] = 0.2f; //姿态控制角速度环PID参数
	
	Ano_Parame.set.pid_att_1level[YAW][KP] = 6.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[YAW][KI] = 0.5f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[YAW][KD] = 0.0f; //姿态控制角速度环PID参数
//---	姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KP] = 7.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KI] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KD] = 0.00f; //姿态控制角度环PID参数
	
	Ano_Parame.set.pid_att_2level[PIT][KP] = 7.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[PIT][KI] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[PIT][KD] = 0.00f; //姿态控制角度环PID参数
	
	Ano_Parame.set.pid_att_2level[YAW][KP] = 5.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[YAW][KI] = 0.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[YAW][KD] = 0.5; //姿态控制角度环PID参数	
//---	高度控制高度速度环PID参数	
	Ano_Parame.set.pid_alt_1level[KP] = 2.0f;          //高度控制高度速度环PID参数
	Ano_Parame.set.pid_alt_1level[KI] = 1.0f;          //高度控制高度速度环PID参数
	Ano_Parame.set.pid_alt_1level[KD] = 0.05f;          //高度控制高度速度环PID参数
//---	高度控制高度环PID参数
	Ano_Parame.set.pid_alt_2level[KP] = 1.0f;           //高度控制高度环PID参数
	Ano_Parame.set.pid_alt_2level[KI] = 0;           //高度控制高度环PID参数(NULL)
	Ano_Parame.set.pid_alt_2level[KD] = 0;           //高度控制高度环PID参数(NULL)
//---	位置控制位置速度环PID参数	
	Ano_Parame.set.pid_loc_1level[KP] = 0.15f;          //位置控制位置速度环PID参数
	Ano_Parame.set.pid_loc_1level[KI] = 0.10f;          //位置控制位置速度环PID参数
	Ano_Parame.set.pid_loc_1level[KD] = 0.00f;          //位置控制位置速度环PID参数
//---	位置控制位置环PID参数
	Ano_Parame.set.pid_loc_2level[KP] = 0;           //位置控制位置环PID参数(NULL)
	Ano_Parame.set.pid_loc_2level[KI] = 0;           //位置控制位置环PID参数(NULL)
	Ano_Parame.set.pid_loc_2level[KD] = 0;           //位置控制位置环PID参数(NULL)
//---	GPS位置控制位置速度环PID参数	
	Ano_Parame.set.pid_gps_loc_1level[KP] = 0.15f;          //位置控制位置速度环PID参数
	Ano_Parame.set.pid_gps_loc_1level[KI] = 0.10f;          //位置控制位置速度环PID参数
	Ano_Parame.set.pid_gps_loc_1level[KD] = 0.00f;          //位置控制位置速度环PID参数
//---	GPS位置控制位置环PID参数
	Ano_Parame.set.pid_gps_loc_2level[KP] = 0.3f;           //位置控制位置环PID参数
	Ano_Parame.set.pid_gps_loc_2level[KI] = 0;           //位置控制位置环PID参数(NULL)
	Ano_Parame.set.pid_gps_loc_2level[KD] = 0;           //位置控制位置环PID参数(NULL)
	
	AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"PID reset!");
}

void Parame_Reset(u8 mode)
{
	if(mode>=1)
	{
		//参数初始化
		//Ano_Parame.set.pwmInMode = PWM;
		Ano_Parame.set.heatSwitch = 0;
		Ano_Parame.set.bat_cell = 3;
		Ano_Parame.set.warn_power_voltage = 3.50f;
		Ano_Parame.set.lowest_power_voltage = 3.40f;
		
		Ano_Parame.set.auto_take_off_height = 0;//cm
		Ano_Parame.set.auto_take_off_speed = 150;
		Ano_Parame.set.auto_landing_speed = 60;
		
		Ano_Parame.set.idle_speed_pwm = 20;//20%
	}
	if(mode==2)
	{
		//
		Ano_Parame.set.acc_calibrated = 0;
		Ano_Parame.set.mag_calibrated = 0;	
	
		for(u8 i = 0;i<3;i++)
		{
			//
			Ano_Parame.set.IEM[0][0] = 1;
			Ano_Parame.set.IEM[0][1] = 0;
			Ano_Parame.set.IEM[0][2] = 0;
			//
			Ano_Parame.set.IEM[1][0] = 0;
			Ano_Parame.set.IEM[1][1] = 1;
			Ano_Parame.set.IEM[1][2] = 0;
			//
			Ano_Parame.set.IEM[2][0] = 0;
			Ano_Parame.set.IEM[2][1] = 0;
			Ano_Parame.set.IEM[2][2] = 1;
			//
			Ano_Parame.set.acc_zero_offset[i] = 0;
			Ano_Parame.set.acc_sensitivity_ref[i] = 1368;
			Ano_Parame.set.gyr_zero_offset[i] = 0;
			Ano_Parame.set.mag_offset[i] = 0;  
			Ano_Parame.set.mag_gain[i] = 200;    
			
			Ano_Parame.set.body_central_pos_cm[i] = 0;
		}
	}
	
	//Parame_Copy_Para2fc();
		
	AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"parameter reset!");
}



static void Ano_Parame_Write(void)
{
	All_PID_Init();	//////存储PID参数后，重新初始化PID	
	Ano_Parame.set.frist_init = SW_VER;

	//Parame_Copy_Fc2para();
	
	Flash_SectorErase ( 0x000000, 1 );							//擦除第一扇区
	Flash_SectorsWrite ( 0x000000, &Ano_Parame.byte[0], 1 );	//将参数写入第一扇区
}

void Ano_Parame_Read(void)
{
	Flash_SectorsRead ( 0x000000, &Ano_Parame.byte[0], 1 );		//读取第一扇区内的参数
	
	if(Ano_Parame.set.frist_init != SW_VER)	//内容没有被初始化，则进行参数初始化工作
	{		
		Parame_Reset(2);
		PID_Rest();
		Ano_Parame_Write();
	}
	
	//Parame_Copy_Para2fc();
	
	
}


void Ano_Parame_Write_task(u16 dT_ms)
{
	//因为写入flash耗时较长，我们飞控做了一个特殊逻辑，在解锁后，是不进行参数写入的，此时会置一个需要写入标志位，等飞机降落锁定后，再写入参数，提升飞行安全性
	//为了避免连续更新两个参数，造成flash写入两次，我们飞控加入一个延时逻辑，参数改变后三秒，才进行写入操作，可以一次写入多项参数，降低flash擦写次数
	if(para_sta.save_en )				//允许存储
	{
		if(para_sta.save_trig == 1) 	//如果触发存储标记1
		{
			LED_STA.saving = 1;
			
			para_sta.time_delay = 0;  	//计时复位
			para_sta.save_trig = 2;   	//触发存储标记2
		}
		
		if(para_sta.save_trig == 2) 	//如果触发存储标记2
		{
			if(para_sta.time_delay<3000) //计时小于3000ms
			{
				para_sta.time_delay += dT_ms; //计时
			}
			else
			{
				
				para_sta.save_trig = 0;  //存储标记复位
				Ano_Parame_Write();      //执行存储
				AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"Set save OK!");
				LED_STA.saving = 0;
			}
		}
		else
		{
			para_sta.time_delay = 0;
		}
		
	}
	else
	{
		para_sta.time_delay = 0;
		para_sta.save_trig = 0;
	}
}

s32 AnoParRead(u16 _id)
{
	s32 p_val = 0;
	switch(_id)
	{
		//====basic
		case PAR_HW_TYPE: p_val = HW_TYPE;	break;
		case PAR_HW_VER: p_val = HW_VER;	break;
		case PAR_SW_VER: p_val = SW_VER;	break;
		//====设置
		case PAR_BAT_CELLS:    p_val = Ano_Parame.set.bat_cell;	break;
		case PAR_LV_WARN_100:  p_val = 1e2f *Ano_Parame.set.warn_power_voltage;	break;
		case PAR_LV_LAND_100:  p_val = 1e2f *Ano_Parame.set.lowest_power_voltage;	break;
		case PAR_HEATSWITCH:   p_val = Ano_Parame.set.heatSwitch;	break;
		case PAR_LANDSPEED:    p_val = Ano_Parame.set.auto_landing_speed;	break;
		case PAR_TAKEOFFSPEED: p_val = Ano_Parame.set.auto_take_off_speed;	break;
		case PAR_TAKEOFFHIGH:  p_val = Ano_Parame.set.auto_take_off_height;	break;
		case PAR_UNLOCKPWM:    p_val = Ano_Parame.set.idle_speed_pwm;	break;
		//====PID
		//角速度
		case PAR_PID_1_P: p_val = 1e3f *Ano_Parame.set.pid_att_1level[ROL][0];	break;
		case PAR_PID_1_I: p_val = 1e3f *Ano_Parame.set.pid_att_1level[ROL][1];	break;
		case PAR_PID_1_D: p_val = 1e3f *Ano_Parame.set.pid_att_1level[ROL][2];	break;
		case PAR_PID_2_P: p_val = 1e3f *Ano_Parame.set.pid_att_1level[PIT][0];	break;
		case PAR_PID_2_I: p_val = 1e3f *Ano_Parame.set.pid_att_1level[PIT][1];	break;
		case PAR_PID_2_D: p_val = 1e3f *Ano_Parame.set.pid_att_1level[PIT][2];	break;
		case PAR_PID_3_P: p_val = 1e3f *Ano_Parame.set.pid_att_1level[YAW][0];	break;
		case PAR_PID_3_I: p_val = 1e3f *Ano_Parame.set.pid_att_1level[YAW][1];	break;
		case PAR_PID_3_D: p_val = 1e3f *Ano_Parame.set.pid_att_1level[YAW][2];	break;
		//角度
		case PAR_PID_4_P: p_val = 1e3f *Ano_Parame.set.pid_att_2level[ROL][0];	break;
		case PAR_PID_4_I: p_val = 1e3f *Ano_Parame.set.pid_att_2level[ROL][1];	break;
		case PAR_PID_4_D: p_val = 1e3f *Ano_Parame.set.pid_att_2level[ROL][2];	break;
		case PAR_PID_5_P: p_val = 1e3f *Ano_Parame.set.pid_att_2level[PIT][0];	break;
		case PAR_PID_5_I: p_val = 1e3f *Ano_Parame.set.pid_att_2level[PIT][1];	break;
		case PAR_PID_5_D: p_val = 1e3f *Ano_Parame.set.pid_att_2level[PIT][2];	break;
		case PAR_PID_6_P: p_val = 1e3f *Ano_Parame.set.pid_att_2level[YAW][0];	break;
		case PAR_PID_6_I: p_val = 1e3f *Ano_Parame.set.pid_att_2level[YAW][1];	break;
		case PAR_PID_6_D: p_val = 1e3f *Ano_Parame.set.pid_att_2level[YAW][2];	break;
		//垂直速度
		case PAR_PID_7_P: p_val = 1e3f *Ano_Parame.set.pid_alt_1level[0];	break;
		case PAR_PID_7_I: p_val = 1e3f *Ano_Parame.set.pid_alt_1level[1];	break;
		case PAR_PID_7_D: p_val = 1e3f *Ano_Parame.set.pid_alt_1level[2];	break;
		//高度
		case PAR_PID_8_P: p_val = 1e3f *Ano_Parame.set.pid_alt_2level[0];	break;
//		case PAR_PID_8_I: p_val = 1e3f *Ano_Parame.set.pid_alt_2level[1]; break;
//		case PAR_PID_8_D: p_val = 1e3f *Ano_Parame.set.pid_alt_2level[2]; break;
		//水平速度
		case PAR_PID_9_P: p_val = 1e3f *Ano_Parame.set.pid_loc_1level[0];	break;
		case PAR_PID_9_I: p_val = 1e3f *Ano_Parame.set.pid_loc_1level[1];	break;
//		case PAR_PID_9_D: p_val = 1e3f *Ano_Parame.set.pid_loc_1level[2];	break;
		//水平位置
//		case PAR_PID_10_P: p_val = 1e3f *Ano_Parame.set.pid_loc_2level[0];	break;
//		case PAR_PID_10_I: p_val = 1e3f *Ano_Parame.set.pid_loc_2level[1];	break;
//		case PAR_PID_10_D: p_val = 1e3f *Ano_Parame.set.pid_loc_2level[2];	break;
		//GPS速度环
		case PAR_PID_11_P: p_val = 1e3f *Ano_Parame.set.pid_gps_loc_1level[0];	break;
		case PAR_PID_11_I: p_val = 1e3f *Ano_Parame.set.pid_gps_loc_1level[1];	break;
//		case PAR_PID_11_D: p_val = 1e3f *Ano_Parame.set.pid_gps_loc_1level[2];	break;
		//GPS位置环
		case PAR_PID_12_P: p_val = 1e3f *Ano_Parame.set.pid_gps_loc_2level[0];	break;
//		case PAR_PID_12_I: p_val = 1e3f *Ano_Parame.set.pid_gps_loc_2level[1];	break;
//		case PAR_PID_12_D: p_val = 1e3f *Ano_Parame.set.pid_gps_loc_2level[2];	break;
		//====sensor_cali
		case CAL_ACC_OFFSET_X:	p_val = Ano_Parame.set.acc_zero_offset[0] * 1000;	break;
		case CAL_ACC_OFFSET_Y:	p_val = Ano_Parame.set.acc_zero_offset[1] * 1000;	break;
		case CAL_ACC_OFFSET_Z:	p_val = Ano_Parame.set.acc_zero_offset[2] * 1000;	break;
		case CAL_ACC_SENSIV_X:	p_val = Ano_Parame.set.acc_sensitivity_ref[0] * 1000;	break;
		case CAL_ACC_SENSIV_Y:	p_val = Ano_Parame.set.acc_sensitivity_ref[1] * 1000;	break;
		case CAL_ACC_SENSIV_Z:	p_val = Ano_Parame.set.acc_sensitivity_ref[2] * 1000;	break;
		case CAL_ACC_IEM_00:	p_val = Ano_Parame.set.IEM[0][0] * 100000;	break;
		case CAL_ACC_IEM_01:	p_val = Ano_Parame.set.IEM[0][1] * 100000;	break;
		case CAL_ACC_IEM_02:	p_val = Ano_Parame.set.IEM[0][2] * 100000;	break;
		case CAL_ACC_IEM_10:	p_val = Ano_Parame.set.IEM[1][0] * 100000;	break;
		case CAL_ACC_IEM_11:	p_val = Ano_Parame.set.IEM[1][1] * 100000;	break;
		case CAL_ACC_IEM_12:	p_val = Ano_Parame.set.IEM[1][2] * 100000;	break;
		case CAL_ACC_IEM_20:	p_val = Ano_Parame.set.IEM[2][0] * 100000;	break;
		case CAL_ACC_IEM_21:	p_val = Ano_Parame.set.IEM[2][1] * 100000;	break;
		case CAL_ACC_IEM_22:	p_val = Ano_Parame.set.IEM[2][2] * 100000;	break;
		case CAL_MAG_OFFSET_X:	p_val = Ano_Parame.set.mag_offset[0] * 1000;	break;
		case CAL_MAG_OFFSET_Y:	p_val = Ano_Parame.set.mag_offset[1] * 1000;	break;
		case CAL_MAG_OFFSET_Z:	p_val = Ano_Parame.set.mag_offset[2] * 1000;	break;
		case CAL_MAG_SENSIV_X:	p_val = Ano_Parame.set.mag_gain[0] * 1000;	break;
		case CAL_MAG_SENSIV_Y:	p_val = Ano_Parame.set.mag_gain[1] * 1000;	break;
		case CAL_MAG_SENSIV_Z:	p_val = Ano_Parame.set.mag_gain[2] * 1000;	break;
		//default一定要注意，如果是飞控没用到的参数，就返回0x80000000，上位机会判断为该参数下位机没用到，会有特定的显示
		default: p_val = 0x80000000;	break;
	}
	return p_val;
}

void AnoParWrite(u16 _id, s32 _val)
{
	//触发写入（实际会停止触发后延时3秒才写入）
	data_save();
	//
	switch(_id)
	{
		//====设置
		case PAR_BAT_CELLS:    Ano_Parame.set.bat_cell = LIMIT(_val,0,65535);	break;
		case PAR_LV_WARN_100:  Ano_Parame.set.warn_power_voltage = 1e-2f *LIMIT(_val,0,65535);	break;
		case PAR_LV_LAND_100:  Ano_Parame.set.lowest_power_voltage = 1e-2f *LIMIT(_val,0,65535);	break;
		case PAR_HEATSWITCH:   Ano_Parame.set.heatSwitch = LIMIT(_val,0,2);	break;
		case PAR_LANDSPEED:    Ano_Parame.set.auto_landing_speed = LIMIT(_val,0,65535);	break;
		case PAR_TAKEOFFSPEED: Ano_Parame.set.auto_take_off_speed = LIMIT(_val,0,65535);	break;
		case PAR_TAKEOFFHIGH:  Ano_Parame.set.auto_take_off_height = LIMIT(_val,0,65535);	break;
		case PAR_UNLOCKPWM:    Ano_Parame.set.idle_speed_pwm = LIMIT(_val,0,65535);	break;
		//====PID
		//角速度
		case PAR_PID_1_P: Ano_Parame.set.pid_att_1level[ROL][0] = 1e-3f  *LIMIT(_val,0,65535);	break;
		case PAR_PID_1_I: Ano_Parame.set.pid_att_1level[ROL][1] = 1e-3f  *LIMIT(_val,0,65535);	break;
		case PAR_PID_1_D: Ano_Parame.set.pid_att_1level[ROL][2] = 1e-3f  *LIMIT(_val,0,65535);	break;
		case PAR_PID_2_P: Ano_Parame.set.pid_att_1level[PIT][0] = 1e-3f  *LIMIT(_val,0,65535);	break;
		case PAR_PID_2_I: Ano_Parame.set.pid_att_1level[PIT][1] = 1e-3f  *LIMIT(_val,0,65535);	break;
		case PAR_PID_2_D: Ano_Parame.set.pid_att_1level[PIT][2] = 1e-3f  *LIMIT(_val,0,65535);	break;
		case PAR_PID_3_P: Ano_Parame.set.pid_att_1level[YAW][0] = 1e-3f  *LIMIT(_val,0,65535);	break;
		case PAR_PID_3_I: Ano_Parame.set.pid_att_1level[YAW][1] = 1e-3f  *LIMIT(_val,0,65535);	break;
		case PAR_PID_3_D: Ano_Parame.set.pid_att_1level[YAW][2] = 1e-3f  *LIMIT(_val,0,65535);	break;
		//角度
		case PAR_PID_4_P: Ano_Parame.set.pid_att_2level[ROL][0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_4_I: Ano_Parame.set.pid_att_2level[ROL][1] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_4_D: Ano_Parame.set.pid_att_2level[ROL][2] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_5_P: Ano_Parame.set.pid_att_2level[PIT][0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_5_I: Ano_Parame.set.pid_att_2level[PIT][1] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_5_D: Ano_Parame.set.pid_att_2level[PIT][2] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_6_P: Ano_Parame.set.pid_att_2level[YAW][0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_6_I: Ano_Parame.set.pid_att_2level[YAW][1] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_6_D: Ano_Parame.set.pid_att_2level[YAW][2] = 1e-3f *LIMIT(_val,0,65535);	break;
		//垂直速度
		case PAR_PID_7_P: Ano_Parame.set.pid_alt_1level[0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_7_I: Ano_Parame.set.pid_alt_1level[1] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_7_D: Ano_Parame.set.pid_alt_1level[2] = 1e-3f *LIMIT(_val,0,65535);	break;
		//高度
		case PAR_PID_8_P: Ano_Parame.set.pid_alt_2level[0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_8_I: Ano_Parame.set.pid_alt_2level[1] = 0;break;//LIMIT(p_val,0,65535);	break;
		case PAR_PID_8_D: Ano_Parame.set.pid_alt_2level[2] = 0;break;//LIMIT(p_val,0,65535);	break;
		//水平速度
		case PAR_PID_9_P: Ano_Parame.set.pid_loc_1level[0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_9_I: Ano_Parame.set.pid_loc_1level[1] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_9_D: Ano_Parame.set.pid_loc_1level[2] = 1e-3f *LIMIT(_val,0,65535);	break;
		//水平位置
		case PAR_PID_10_P: Ano_Parame.set.pid_loc_2level[0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_10_I: Ano_Parame.set.pid_loc_2level[1] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_10_D: Ano_Parame.set.pid_loc_2level[2] = 1e-3f *LIMIT(_val,0,65535);	break;
		//
		case PAR_PID_11_P: Ano_Parame.set.pid_gps_loc_1level[0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_11_I: Ano_Parame.set.pid_gps_loc_1level[1] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_11_D: Ano_Parame.set.pid_gps_loc_1level[2] = 1e-3f *LIMIT(_val,0,65535);	break;
		//
		case PAR_PID_12_P: Ano_Parame.set.pid_gps_loc_2level[0] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_12_I: Ano_Parame.set.pid_gps_loc_2level[1] = 1e-3f *LIMIT(_val,0,65535);	break;
		case PAR_PID_12_D: Ano_Parame.set.pid_gps_loc_2level[2] = 1e-3f *LIMIT(_val,0,65535);	break;
		//====sensor_cali
		case CAL_ACC_OFFSET_X:	Ano_Parame.set.acc_zero_offset[0] = (float)_val / 1000;	break;
		case CAL_ACC_OFFSET_Y:	Ano_Parame.set.acc_zero_offset[1] = (float)_val / 1000;	break;
		case CAL_ACC_OFFSET_Z:	Ano_Parame.set.acc_zero_offset[2] = (float)_val / 1000;	break;
		case CAL_ACC_SENSIV_X:	Ano_Parame.set.acc_sensitivity_ref[0] = (float)_val / 1000;	break;
		case CAL_ACC_SENSIV_Y:	Ano_Parame.set.acc_sensitivity_ref[1] = (float)_val / 1000;	break;
		case CAL_ACC_SENSIV_Z:	Ano_Parame.set.acc_sensitivity_ref[2] = (float)_val / 1000;	break;
		case CAL_ACC_IEM_00:	Ano_Parame.set.IEM[0][0] = (float)_val / 100000;	break;
		case CAL_ACC_IEM_01:	Ano_Parame.set.IEM[0][1] = (float)_val / 100000;	break;
		case CAL_ACC_IEM_02:	Ano_Parame.set.IEM[0][2] = (float)_val / 100000;	break;
		case CAL_ACC_IEM_10:	Ano_Parame.set.IEM[1][0] = (float)_val / 100000;	break;
		case CAL_ACC_IEM_11:	Ano_Parame.set.IEM[1][1] = (float)_val / 100000;	break;
		case CAL_ACC_IEM_12:	Ano_Parame.set.IEM[1][2] = (float)_val / 100000;	break;
		case CAL_ACC_IEM_20:	Ano_Parame.set.IEM[2][0] = (float)_val / 100000;	break;
		case CAL_ACC_IEM_21:	Ano_Parame.set.IEM[2][1] = (float)_val / 100000;	break;
		case CAL_ACC_IEM_22:	
		{
			Ano_Parame.set.IEM[2][2] = (float)_val / 100000;
			//
			st_imu_cali.acc_cali_on = 0;
			Ano_Parame.set.acc_calibrated = 1;
			//传感器灵敏度初始化
			ImuSensitivityInit(Ano_Parame.set.acc_calibrated,(float *)Ano_Parame.set.acc_sensitivity_ref);
			data_save();
		}
		break;
		case CAL_MAG_OFFSET_X:	Ano_Parame.set.mag_offset[0] = (float)_val / 1000;	break;
		case CAL_MAG_OFFSET_Y:	Ano_Parame.set.mag_offset[1] = (float)_val / 1000;	break;
		case CAL_MAG_OFFSET_Z:	Ano_Parame.set.mag_offset[2] = (float)_val / 1000;	break;
		case CAL_MAG_SENSIV_X:	Ano_Parame.set.mag_gain[0] = (float)_val / 1000;	break;
		case CAL_MAG_SENSIV_Y:	Ano_Parame.set.mag_gain[1] = (float)_val / 1000;	break;
		case CAL_MAG_SENSIV_Z:	
		{
			Ano_Parame.set.mag_gain[2] = (float)_val / 1000;
			mag.mag_CALIBRATE = 0;
			Ano_Parame.set.mag_calibrated = 1;
			data_save();
		}
		break;
		default: break;
	}
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
