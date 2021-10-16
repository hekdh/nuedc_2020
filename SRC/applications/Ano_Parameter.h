#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "stm32f4xx.h"
#include "Ano_FcData.h"

enum
{
	//
	PAR_NULL    = 0,
	PAR_HW_TYPE ,
	PAR_HW_VER,
	PAR_SW_VER,
	PAR_BL_VER,//  = 4,
	//
	PAR_INFO5,
	PAR_INFO6,
	PAR_INFO7,
	PAR_INFO8,
	PAR_INFO9,
	PAR_INFO10,
	//PID
	PAR_PID_1_P,//	=	11,
	PAR_PID_1_I,
	PAR_PID_1_D,
	PAR_PID_2_P,
	PAR_PID_2_I,
	PAR_PID_2_D,
	PAR_PID_3_P,
	PAR_PID_3_I,
	PAR_PID_3_D,
	PAR_PID_4_P,
	PAR_PID_4_I,
	PAR_PID_4_D,
	PAR_PID_5_P,
	PAR_PID_5_I,
	PAR_PID_5_D,
	PAR_PID_6_P,
	PAR_PID_6_I,
	PAR_PID_6_D,
	PAR_PID_7_P,
	PAR_PID_7_I,
	PAR_PID_7_D,
	PAR_PID_8_P,
	PAR_PID_8_I,
	PAR_PID_8_D,
	PAR_PID_9_P,
	PAR_PID_9_I,
	PAR_PID_9_D,
	PAR_PID_10_P,
	PAR_PID_10_I,
	PAR_PID_10_D,
	//
	PAR_PID_11_P,//  = 41,
	PAR_PID_11_I,
	PAR_PID_11_D,
	PAR_PID_12_P,
	PAR_PID_12_I,
	PAR_PID_12_D,
	PAR_PID_13_P,
	PAR_PID_13_I,
	PAR_PID_13_D,
	PAR_PID_14_P,
	PAR_PID_14_I,
	PAR_PID_14_D,
	PAR_PID_15_P,
	PAR_PID_15_I,
	PAR_PID_15_D,
	PAR_PID_16_P,
	PAR_PID_16_I,
	PAR_PID_16_D,
	PAR_PID_17_P,
	PAR_PID_17_I,
	PAR_PID_17_D,
	PAR_PID_18_P,
	PAR_PID_18_I,
	PAR_PID_18_D,
	//
	PAR_PID_ONE_1,
	PAR_PID_ONE_2,
	PAR_PID_ONE_3,
	PAR_PID_MODE,
	//FC_SET
	PAR_RCINMODE     =	71,
	PAR_UNLOCKPWM,
	PAR_UNLOCK_OO,
	PAR_AUTO_GYR_CAL,
	PAR_BAT_CELLS,
	PAR_LV_WARN_100,
	PAR_LV_RETN_100,
	PAR_LV_LAND_100,
	PAR_CENPOS_X,
	PAR_CENPOS_Y,
	PAR_CENPOS_Z,
	PAR_TAKEOFFHIGH	 ,
	PAR_TAKEOFFSPEED ,
	PAR_LANDSPEED,
	PAR_LANDSPEED_MAX,
	PAR_AUTO_LANDING,
	PAR_HEATSWITCH	 ,	
	PAR_HEAT_TMPER   ,	
	//
	PAR_MAX_SPEED_HOR,
	PAR_MAX_SPEED_PRC,
	PAR_MAX_SPEED_UP,
	PAR_MAX_SPEED_DW,
//	PAR_MAX_SPEED_YAW,
	//
	PAR_SAFE_ATT = 94,
	PAR_MAGMODE,
	PAR_ACANGVELMAX,
	PAR_YCANGVELMAX,
	PAR_YCANGACCMAX,
	PAR_RH_ALT,
	PAR_RH_VEL,
	//LX_IMU
	PAR_GYR_FILTER = 151,
	PAR_ACC_FILTER,
	PAR_ATT_FUSION,
	PAR_MAG_FUSION,
	PAR_HOR_FUSION,
	PAR_VER_FUSION,
	PAR_FENCE_MODE,
	PAR_FENCE_RADIUS,
	PAR_FENCE_WIDTH_X,
	PAR_FENCE_WIDTH_Y,
	PAR_FENCE_HEIGHT,
	PAR_COM1_BAUD,
	PAR_COM2_BAUD,
	PAR_COM2_MODE,
	PAR_RGBOUT_ENA,
	//
	PAR_DataOutTime_01=300,
	PAR_DataOutTime_02,
	PAR_DataOutTime_03,
	PAR_DataOutTime_04,
	PAR_DataOutTime_05,
	PAR_DataOutTime_06,
	PAR_DataOutTime_07,
	PAR_DataOutTime_08,
	PAR_DataOutTime_09,
	PAR_DataOutTime_0A,
	PAR_DataOutTime_0B,
	PAR_DataOutTime_0C,
	PAR_DataOutTime_0D,
	PAR_DataOutTime_0E,
	PAR_DataOutTime_20,
	PAR_DataOutTime_21,
	PAR_DataOutTime_30,
	PAR_DataOutTime_32,
	PAR_DataOutTime_33,
	PAR_DataOutTime_34,
	PAR_DataOutTime_40,
	PAR_DataOutTime_41,
	//
	PAR_DATA_NUM,
	//
	CAL_ACC_OFFSET_X = 1000,
	CAL_ACC_OFFSET_Y,
	CAL_ACC_OFFSET_Z,
	CAL_ACC_SENSIV_X,
	CAL_ACC_SENSIV_Y,
	CAL_ACC_SENSIV_Z,
	CAL_ACC_IEM_00,
	CAL_ACC_IEM_01,
	CAL_ACC_IEM_02,
	CAL_ACC_IEM_10,
	CAL_ACC_IEM_11,
	CAL_ACC_IEM_12,
	CAL_ACC_IEM_20,
	CAL_ACC_IEM_21,
	CAL_ACC_IEM_22,
	CAL_MAG_OFFSET_X,
	CAL_MAG_OFFSET_Y,
	CAL_MAG_OFFSET_Z,
	CAL_MAG_SENSIV_X,
	CAL_MAG_SENSIV_Y,
	CAL_MAG_SENSIV_Z,
};

typedef struct
//__packed struct _Parameter_s
{
	//
	float IEM[3][3];                      //安装误差矩阵	
	//acc
	float acc_zero_offset[3];             //加速度计零偏
	float acc_sensitivity_ref[3];//1G
	//gyro
	float gyr_zero_offset[3];             //陀螺仪零偏
	//
	float body_central_pos_cm[VEC_XYZ];   //重心相对传感器位置偏移量
	//mag
	float mag_offset[VEC_XYZ];            //磁力计零偏
	float mag_gain[VEC_XYZ];              //磁力计校正比例	  	
	//
	float 	pid_att_1level[VEC_RPY][PID]; //姿态控制角速度环PID参数
	float 	pid_att_2level[VEC_RPY][PID]; //姿态控制角度环PID参数
	float 	pid_alt_1level[PID];          //高度控制高度速度环PID参数
	float 	pid_alt_2level[PID];           //高度控制高度环PID参数
	float 	pid_loc_1level[PID];          //位置控制位置速度环PID参数
	float 	pid_loc_2level[PID];           //位置控制位置环PID参数

	float 	pid_gps_loc_1level[PID];          //位置控制位置速度环PID参数
	float 	pid_gps_loc_2level[PID];           //位置控制位置环PID参数

	float   warn_power_voltage;
	s32	    bat_cell;
	float   lowest_power_voltage;
	
	float	auto_take_off_height;
	float auto_take_off_speed;
	float auto_landing_speed;
	float idle_speed_pwm;
	
	//	
	u8		pwmInMode;				//接收机模式，分别为PWM型PPM型
	u8		heatSwitch;				//
	//ins
	u8 acc_calibrated;
	u8 mag_calibrated;
	u8 reserve1;
	u8 reserve2;
	//
	u16 frist_init;	//飞控第一次初始化，需要做一些特殊工作，比如清空flash	
}_Parameter_s;//__attribute__ ((__packed__)) 

union Parameter
{
	//这里使用联合体，长度是4KByte，联合体内部是一个结构体，该结构体内是需要保存的参数
	_Parameter_s set;
	u8 byte[4096];
};
extern union Parameter Ano_Parame;

typedef struct
{
	u8 save_en;
	u8 save_trig;
	u16 time_delay;
}_parameter_state_st ;
extern _parameter_state_st para_sta;

void Ano_Parame_Read(void);
void Ano_Parame_Write_task(u16 dT_ms);
void PID_Rest(void);
void Parame_Reset(u8 mode);

s32 AnoParRead(u16 _id);
void AnoParWrite(u16 _id, s32 _val);

#endif 

