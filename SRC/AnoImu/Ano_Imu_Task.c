#include "Ano_Imu_Data.h"
#include "Ano_Imu_Calibration.h"
#include "Ano_Imu_Task.h"
#include "Drv_Bmi088.h"
#include "Drv_spl06.h"
#include "Ano_Parameter.h"

/*IMUѭ������1����*/
void ImuServices_1ms_c()
{
	/*��ȡ�����Ǽ��ٶȼ�����*/
	DrvBmi088AccelerationRead();
	//Drv_Icm20602_Read();
	DrvBmi088ReadServices();
	//��������
	ImuDataGet((s16 *)st_bmi_data.un_ins.st_data.s16_gyr,(s16 *)st_bmi_data.un_ins.st_data.s16_acc);
	//���ݼ���
	ImuDataCalcu(Ano_Parame.set.acc_calibrated,(float *)Ano_Parame.set.gyr_zero_offset,(float *)Ano_Parame.set.acc_zero_offset,(float (*)[])Ano_Parame.set.IEM);
	//
	if(flag.unlock_sta==0)
	{
		//��ֹ���
		AccGyrStableCheck_Services(0.001f,st_imuData.f_acc_cmpss,st_imuData.f_gyr_dps);
		//
		if(st_imu_cali.gyr_cali_on)
		{
			//
			if(GetGyrAvValue(st_imu_cali.gyr_stable,st_imuData.f_gyrRaw,(float *)Ano_Parame.set.gyr_zero_offset))
			{
				if(st_imu_cali.gyr_cali_on==2)
				{
					st_imu_cali.gyr_cali_on = 0;
				}
				else if(st_imu_cali.gyr_cali_on==1)
				{
					st_imu_cali.gyr_cali_on = 0;
					//�洢����
					data_save();
				}
			}
		}
	}
}
/*IMUѭ������20����*/
void ImuServices_20ms_c()
{
	//��ȡIMU�¶�
	ImuTemperatureGet(spl_data.temperature);
	//
}
