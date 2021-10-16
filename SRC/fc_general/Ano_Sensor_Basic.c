//默认引用：
#include "Ano_Sensor_Basic.h"
#include "Ano_Math.h"


//数据接口定义：
//=========mapping===========
//需要引用的文件：
#include "Ano_Parameter.h"
#include "Drv_led.h"
#include "Ano_Imu_Data.h"
#include "Ano_Imu_Calibration.h"

//需要调用引用的外部变量：
#define X_POS_OFFSET_CM    (Ano_Parame.set.body_central_pos_cm[X]); //X轴中心偏移存储值
#define Y_POS_OFFSET_CM    (Ano_Parame.set.body_central_pos_cm[Y]); //Y轴中心偏移存储值
#define Z_POS_OFFSET_CM    (Ano_Parame.set.body_central_pos_cm[Z]); //Z轴中心偏移存储值

//需要操作赋值的外部变量：
#define LED_STA_CALI_ACC   (LED_STA.calAcc)
#define LED_STA_CALI_GYR   (LED_STA.calGyr)


//=========mapping===========

void Sensor_Basic_Init()
{
	/*设置重心相对传感器的偏移量*/
	Center_Pos_Set();
	
	//sensor.gyr_CALIBRATE = 2;//开机自动校准陀螺仪
	st_imu_cali.gyr_cali_on = 2;//开机自动校准陀螺仪
}

_center_pos_st center_pos;
_sensor_st sensor;

s32 sensor_val[6];

void Center_Pos_Set()
{
	center_pos.center_pos_cm[X] = X_POS_OFFSET_CM;//+0.0f;
	center_pos.center_pos_cm[Y] = Y_POS_OFFSET_CM;//-0.0f;
	center_pos.center_pos_cm[Z] = Z_POS_OFFSET_CM;//+0.0f;
}

static float gyr_f[5][VEC_XYZ],acc_f[5][VEC_XYZ];

void Sensor_Data_Prepare(u8 dT_ms)
{	
	float hz = 0 ;
	if(dT_ms != 0) hz = 1000/dT_ms;
	
//======================================================================	
	
	/*赋值*/
	for(u8 i=0;i<3;i++)
	{
		sensor_val[G_X + i] = st_imuData.f_gyr_dps_nb[i];
		sensor_val[A_X + i] = st_imuData.f_acc_cmpss_nb[i];
	}
//======================================================================
	
	/*软件低通滤波*/
	for(u8 i=0;i<3;i++)
	{	
		//
		gyr_f[4][X +i] = (sensor_val[G_X + i] );
		acc_f[4][X +i] = (sensor_val[A_X + i] );
		//
		for(u8 j=4;j>0;j--)
		{
			//
			gyr_f[j-1][X +i] += GYR_ACC_FILTER *(gyr_f[j][X +i] - gyr_f[j-1][X +i]);
			acc_f[j-1][X +i] += GYR_ACC_FILTER *(acc_f[j][X +i] - acc_f[j-1][X +i]);
		}
		
//		LPF_1_(100,dT_ms*1e-3f,sensor_val_ref[G_X + i],sensor.Gyro[X +i]);
//		LPF_1_(100,dT_ms*1e-3f,sensor_val_ref[A_X + i],sensor.Acc[X +i]);
				
	}
	
//======================================================================
	

	
//======================================================================
	/*赋值*/
	for(u8 i=0;i<3;i++)
	{	
		sensor.Gyro_deg[i] = gyr_f[0][i];//sensor_val_ref[G_X + i];
		sensor.Acc_cmss[i]  = acc_f[0][i];//sensor_val_ref[A_X+i];//
		sensor.Gyro_rad[i] = sensor.Gyro_deg[i] *0.01745f;
	}
	

}
