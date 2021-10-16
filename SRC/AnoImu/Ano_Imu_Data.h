#ifndef _ANO_IMU_DATA_H_
#define _ANO_IMU_DATA_H_


//==引用
#include "include.h"


//==定义
typedef struct
{
	//
	u8 data_sta;  //0不可用  1可用
	float IEM[3][3];
	//
	vec3_f f_gyrRaw;
	vec3_f f_accRaw;
	//
	vec3_f f_gyr_dps;
	vec3_f f_acc_cmpss;	
	//
	vec3_f f_gyr_dps_nb;
	vec3_f f_gyr_radps_nb;
	vec3_f f_acc_cmpss_nb;		
	//
	float f_temperature;
	vec3_f gyrSensitivity;
	vec3_f accSensitivity;
	//
	
}_imuData_st;

typedef struct
{
	u8 test_u8[4];
	float test[3];
}__attribute__ ((__packed__)) _test_st;
extern _test_st test_st;


//==数据声明
extern _imuData_st st_imuData;	

//==函数声明

//static


//public

/*IMU传感器灵敏度初始化*/
void ImuSensitivityInit(u8 ins_calibrated,vec3_f accRefValue);
/*IMU传感器数据获取*/
void ImuDataGet(vec3_s16 gyrRaw,vec3_s16 accRaw);
/*IMU温度获取*/
void ImuTemperatureGet(float f_temperature);
/*IMU数据计算处理*/
void ImuDataCalcu(u8 ins_calibrated,vec3_f gyrOffset,vec3_f accOffset,float IEM[3][3]);

	
#endif


