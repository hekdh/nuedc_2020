
/*==========================================================================
 * ����    �����������(UP_OF)���ص����ݽ��д������������帩���������ת
             ����ɹ����������ϣ�Ҳ��������ת�������ת��������Ȼ����߶�
			���л���õ������ٶȣ�������ٶȼƲ������ݽ����ںϣ��õ����ȶ�
			�ĵ����ٶ������
 
 * ����ʱ�䣺2019-07-13 
 * ����	   �������ƴ�-Jyoun
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ��Ŀ������18084888982��18061373080
============================================================================
 * �����ƴ��ŶӸ�л��ҵ�֧�֣���ӭ��ҽ�Ⱥ���ཻ�������ۡ�ѧϰ��
 * �������������в��õĵط�����ӭ����ש�������
 * �������������ã�����������Ƽ���֧�����ǡ�
 * ������Դ������뻶ӭ�������á��������չ��������ϣ������ʹ��ʱ��ע��������
 * ����̹������С�˳����ݣ��������������ˮ���������ӣ�Ҳ��δ�й�Ĩ��ͬ�е���Ϊ��  
 * ��Դ���ף�����������ף�ϣ����һ������ء����ﻥ������ͬ������
 * ֻ������֧�֣������������ø��á� 
============================================================================
���£�

201908012354-Jyoun����������������ϵĹ����������
���汸ע�����������װ�����ǽ��߷����󣨸��ӷɻ�������Ҫ���趨������ĸ߶ȡ�




===========================================================================*/

//Ĭ������
#include "Ano_OF_DecoFusion.h"
#include "Ano_IMU.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"

//���ݽӿڶ��壺
//=========mapping===============
//��Ҫ���õ��ļ���
#include "Drv_UP_Flow.h"
#include "Ano_Sensor_Basic.h"
#include "Drv_laser.h"

//��Ҫ�������õ��ⲿ������
#define LASER_ONLINE              (LASER_LINKOK)
#define BUF_UPDATE_CNT            (of_buf_update_cnt)
#define OF_DATA_BUF               (OF_DATA)
#define RADPS_X                   (sensor.Gyro_rad[0])
#define RADPS_Y                   (sensor.Gyro_rad[1])
#define RELATIVE_HEIGHT_CM        (Laser_height_cm)  //��Ը߶�
//��Ҫ������ֵ���ⲿ������


//===============================
//ȫ�ֱ�����
u8 of_buf_update_flag;
_of_data_st of_data;
_of_rdf_st of_rdf;
float of_rot_d_degs[2];

float of_fus_err[2],of_fus_err_i[2];
//�����趨��
#define UPOF_PIXELPDEG_X    160.0f       //ÿ1�Ƕȶ�Ӧ�����ظ�������ֱ��ʺͽ����йأ���Ҫ���Ա궨��//
#define UPOF_PIXELPDEG_Y    160.0f       //ÿ1�Ƕȶ�Ӧ�����ظ�������ֱ��ʺͽ����йأ���Ҫ���Ա궨��
#define UPOF_CMPPIXEL_X     0.00012f     //ÿ���ض�Ӧ�ĵ�����룬�뽹��͸߶��йأ���Ҫ���Ա궨��//Ŀǰ���Ա궨
#define UPOF_CMPPIXEL_Y     0.00012f     //ÿ���ض�Ӧ�ĵ�����룬�뽹��͸߶��йأ���Ҫ���Ա궨��
#define FUS_KP              2.0f
#define FUS_KI              1.0f
//
#define UPOF_UP_DW          0             //0:���£�1������
#define OBJREF_HEIGHT_CM    280           //������߶ȣ�����;�������ϲ����á�
/**********************************************************************************************************
*�� �� ��: ANO_OF_Data_Check_Task
*����˵��: �����ƴ�����׼����������
*��    ��: ����ʱ��(s)
*�� �� ֵ: ��
**********************************************************************************************************/
void ANO_OF_Data_Prepare_Task(float dT_s)
{
	//
	ANO_OF_Data_Get(&dT_s,OF_DATA_BUF);
	//
	OF_INS_Get(&dT_s,RADPS_X,RADPS_Y,imu_data.w_acc[0],imu_data.w_acc[1]);
}

/**********************************************************************************************************
*�� �� ��: ANO_OFDF_Task
*����˵��: �����ƴ�������������ں�����
*��    ��: ����ʱ��(ms)
*�� �� ֵ: ��
**********************************************************************************************************/
void ANO_OFDF_Task(u8 dT_ms)
{
	//
	OF_State();
	//
	ANO_OF_Decouple(&dT_ms);
	//
	ANO_OF_Fusion(&dT_ms,(s32)RELATIVE_HEIGHT_CM);

	
}


/**********************************************************************************************************
*�� �� ��: OF_INS_Get
*����˵��: �����ƴ������������ݻ�ȡ
*��    ��: ����ʱ��(s���β�)��x��ÿ�뻡�ȣ�y��ÿ�뻡�ȣ�����x���ٶ�(cmpss)������y���ٶ�(cmpss)
*�� �� ֵ: ��
**********************************************************************************************************/
static void OF_INS_Get(float *dT_s,float rad_ps_x,float rad_ps_y,float acc_wx,float acc_wy)
{
	static float rad_ps_lpf[2];
	//====RD
	//��ͨ�˲�
	//�����ͺ�Ϊ�˶�����λ�����ﻻ����ױ���+FIFOЧ�����á�
	LPF_1_(5.0f,*dT_s,rad_ps_x,rad_ps_lpf[0]);
	LPF_1_(5.0f,*dT_s,rad_ps_y,rad_ps_lpf[1]);
//	rad_ps_lpf[0] += 0.2f *(rad_ps_x - rad_ps_lpf[0]);
//	rad_ps_lpf[1] += 0.2f *(rad_ps_y - rad_ps_lpf[1]);
	//
	of_rot_d_degs[0] = rad_ps_lpf[0] *DEG_PER_RAD ;
	of_rot_d_degs[1] = rad_ps_lpf[1] *DEG_PER_RAD ;
	//
//	test_calibration[0] += OF_ROT_KA *rad_ps_x *DEG_PER_RAD *dT_s;
	//====INS
	//��ͨ�˲�
	LPF_1_(5.0f,*dT_s,acc_wx,of_rdf.gnd_acc_est_w[X]);
	LPF_1_(5.0f,*dT_s,acc_wy,of_rdf.gnd_acc_est_w[Y]);
	//
	for(u8 i = 0;i<2;i++)
	{
		//�ںϹ��Ʋ���
		of_rdf.gnd_vel_est_w[i] += of_rdf.gnd_acc_est_w[i] *(*dT_s);


	}
	
}

/**********************************************************************************************************
*�� �� ��: ANO_OF_Data_Get
*����˵��: �����ƴ��������ݻ�ȡ
*��    ��: ����ʱ��(s���β�),�������ݻ��棨�βΣ�
*�� �� ֵ: ��
**********************************************************************************************************/
static void ANO_OF_Data_Get(float *dT_s,u8 *of_data_buf)
{
	static float offline_delay_time_s;
	u8 XOR = 0;
	if(of_buf_update_flag != BUF_UPDATE_CNT)
	{
		//
		of_buf_update_flag = BUF_UPDATE_CNT;
		//
		XOR = of_data_buf[2];
		for(u8 i = 3;i<12;i++)
		{
			XOR ^= of_data_buf[i];
		}
		//
		if(XOR == of_data_buf[12])
		{
			//
			of_data.updata ++;
			//
			of_data.valid = of_data_buf[10];
			//
			if(of_data.valid != 0xf5)
			{
				//
				of_data.flow_x_integral = of_data.flow_y_integral = 0;
			}
			else
			{	
				//
				of_data.flow_x_integral = (s16)(of_data_buf[4]|(of_data_buf[5]<<8));//org_y
				//
				of_data.flow_y_integral = (s16)(of_data_buf[2]|(of_data_buf[3]<<8));//org_x
			}				
			//
			of_data.it_ms = ((u16)(of_data_buf[6]|(of_data_buf[7]<<8)))/1000;

		}
		//
		offline_delay_time_s = 0;
		of_data.online = 1;
	}
	else
	{
		//null
		if(offline_delay_time_s<1.0f)
		{
			offline_delay_time_s += *dT_s;
		}
		else//����
		{
			of_data.online = 0;
		}
	}
}


/**********************************************************************************************************
*�� �� ��: ANO_OF_Decouple
*����˵��: �����ƴ����������
*��    ��: ����ʱ��(�β�ms)
*�� �� ֵ: ��
*��    ע: ����20ms����һ��
**********************************************************************************************************/
static void ANO_OF_Decouple(u8 *dT_ms)
{

	if(of_data.valid != 0xf5)
	{
		//
		of_rdf.of_vel[X] = of_rdf.of_vel[Y] = 0;
		//quality
		if(of_rdf.quality>=5)
		{
			of_rdf.quality -= 5;
		}
	}
	else
	{
		//
		if(UPOF_UP_DW==0)
		{
			of_rdf.of_vel[X] = (1000/of_data.it_ms *of_data.flow_x_integral + UPOF_PIXELPDEG_X *of_rot_d_degs[Y] );
			of_rdf.of_vel[Y] = (1000/of_data.it_ms *of_data.flow_y_integral - UPOF_PIXELPDEG_Y *of_rot_d_degs[X] );
		}
		else
		{
			of_rdf.of_vel[X] = -(1000/of_data.it_ms *of_data.flow_x_integral + UPOF_PIXELPDEG_X *of_rot_d_degs[Y] );
			of_rdf.of_vel[Y] =  (1000/of_data.it_ms *of_data.flow_y_integral + UPOF_PIXELPDEG_Y *of_rot_d_degs[X] );		
		}
		//quality
		if(of_rdf.quality<=250)
		{
			of_rdf.quality += 5;
		}			
	}
	
}

/**********************************************************************************************************
*�� �� ��: ANO_OF_Decoupling
*����˵��: �����ƴ����������
*��    ��: ����ʱ��(�β�ms),�ο��߶�(cm)
*�� �� ֵ: ��
**********************************************************************************************************/
static void ANO_OF_Fusion(u8 *dT_ms,s32 ref_height_cm)
{
	static float F_KP,F_KI;
	float dT_s = (*dT_ms)*1e-3f;

	//
	if(UPOF_UP_DW==0)
	{
		of_rdf.of_ref_height = LIMIT(ref_height_cm,20,500);//���Ƶ�20cm-500cm
	}
	else
	{
		of_rdf.of_ref_height = LIMIT((OBJREF_HEIGHT_CM - ref_height_cm),20,500);//���Ƶ�20cm-500cm
	}
	//
	of_rdf.gnd_vel_obs_h[X] = UPOF_CMPPIXEL_X *of_rdf.of_vel[X] *of_rdf.of_ref_height;
	of_rdf.gnd_vel_obs_h[Y] = UPOF_CMPPIXEL_Y *of_rdf.of_vel[Y] *of_rdf.of_ref_height;
	//
	h2w_2d_trans(of_rdf.gnd_vel_obs_h,imu_data.hx_vec,of_rdf.gnd_vel_obs_w);
	
	
	//
	switch(of_rdf.state)
	{
		case 0:
		{
			//
			of_rdf.state = 1;
			//
			F_KP = FUS_KP;
			//
			F_KI = FUS_KI;
			//
			OF_INS_Reset();	
			//			
		}
		break;
		case 1:
		{
			//�ں���������
			//(���￪Դ��򵥲��Һ��õ�PI�����ںϣ�ע������������ȡ��Ƶ��ȡ��Ƶ�Ĺ��ƵĲ��ֲ��ڴ˴�)

			//==
			//ԭʼֵ��Чʱ������
			if(of_data.valid == 0xf5)
			{
				//
				for(u8 i =0;i<2;i++)
				{				
					//
					of_fus_err[i] = of_rdf.gnd_vel_obs_w[i] - of_rdf.gnd_vel_est_w[i];
					//
					of_fus_err_i[i] += F_KI *of_fus_err[i] *dT_s;
					of_fus_err_i[i] = LIMIT(of_fus_err_i[i],-100,100);
					//
					of_rdf.gnd_vel_est_w[i] += (of_fus_err[i] *F_KP + of_fus_err_i[i]) *dT_s;

				}
			}
			//
			w2h_2d_trans(of_rdf.gnd_vel_est_w,imu_data.hx_vec,of_rdf.gnd_vel_est_h);

			//
		}
		break;
		default://case 2:
		{
			//
//			of_rdf.state = 1;
			OF_INS_Reset();
		}
		break;
//		default:break;
	}
}

/**********************************************************************************************************
*�� �� ��: OF_INS_Reset
*����˵��: �����ںϸ�λ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void OF_INS_Reset()
{
	for(u8 i = 0;i<2;i++)
	{
		//
		of_rdf.gnd_vel_est_w[i] = 0;
		//
		of_fus_err_i[i] = 0;
		
	}
}

/**********************************************************************************************************
*�� �� ��: OF_State
*����˵��: ����״̬����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
static void OF_State()
{
	//====sta
	if(imu_state.G_reset )//
	{
		//
		if(of_rdf.state == 1)
		{
			of_rdf.state = 0;
		}
	}
	else
	{
		if(of_rdf.quality<100)
		{
			of_rdf.state = 0;
		}
		else if(of_rdf.quality>200)
		{
			of_rdf.state = 1;
		}
	}
	//
	if(of_data.online && LASER_ONLINE)
	{
		sens_hd_check.of_df_ok = 1;
	}
	else
	{
		sens_hd_check.of_df_ok = 0;
	}
}

