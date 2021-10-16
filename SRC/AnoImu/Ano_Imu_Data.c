//
//
#include "Ano_Imu_Data.h"
#include "ANO_Math.h"

#define G_1G_CMPSS 981
//
_imuData_st st_imuData;
_test_st test_st;
/*IMU�����������ȳ�ʼ��*/
void ImuSensitivityInit(u8 ins_calibrated,vec3_f accRefValue)
{
	//�����������Ȼ���
	for(u8 i=0;i<3;i++)
	{
		st_imuData.gyrSensitivity[i] = 2000.0f/32767.0f;
	}
	//���ٶȼ������Ȼ���
	if(ins_calibrated==1)
	{
		st_imuData.accSensitivity[0] = G_1G_CMPSS/accRefValue[0];
		st_imuData.accSensitivity[1] = G_1G_CMPSS/accRefValue[1];
		st_imuData.accSensitivity[2] = G_1G_CMPSS/accRefValue[2];
	}
	else
	{
		st_imuData.accSensitivity[0] = 
		st_imuData.accSensitivity[1] = 
		st_imuData.accSensitivity[2] = 0;
	}
}
/*IMU���������ݻ�ȡ*/
void ImuDataGet(vec3_s16 gyrRaw,vec3_s16 accRaw)
{
	for(u8 i=0;i<3;i++)
	{
		//
		st_imuData.f_gyrRaw[i] = gyrRaw[i];
		st_imuData.f_accRaw[i] = accRaw[i];
		//		
	}
}
/*IMU�¶Ȼ�ȡ*/
void ImuTemperatureGet(float f_temperature)
{
	st_imuData.f_temperature = f_temperature;
}
/*IMU���ݼ��㴦��*/
void ImuDataCalcu(u8 ins_calibrated,vec3_f gyrOffset,vec3_f accOffset,float IEM[3][3])
{
	if(ins_calibrated==1)
	{
		//
		st_imuData.data_sta = 1;
		//ת����λ
		for(u8 i=0;i<3;i++)
		{
			//
			st_imuData.f_gyr_dps[i]   = (st_imuData.f_gyrRaw[i] - gyrOffset[i])*st_imuData.gyrSensitivity[i];
			st_imuData.f_acc_cmpss[i] = (st_imuData.f_accRaw[i] - accOffset[i])*st_imuData.accSensitivity[i];
			//		
		}
		//תANO����ϵ
		Vec3f_Mul_MatrixT(st_imuData.f_gyr_dps,IEM,st_imuData.f_gyr_dps_nb);
		Vec3f_Mul_MatrixT(st_imuData.f_acc_cmpss,IEM,st_imuData.f_acc_cmpss_nb);
		//�Ƕ�ת����
		for(u8 i=0;i<3;i++)
		{
			st_imuData.f_gyr_radps_nb[i] = RAD_PER_DEG *st_imuData.f_gyr_dps_nb[i];
		}
	}
	else
	{
		//
		st_imuData.data_sta = 0;
		//
		for(u8 i=0;i<3;i++)
		{
			//δУ׼�����ԭʼֵ
			st_imuData.f_gyr_dps_nb[i]   = st_imuData.f_gyrRaw[i];
			st_imuData.f_acc_cmpss_nb[i] = st_imuData.f_accRaw[i];
		}	
	}
}




