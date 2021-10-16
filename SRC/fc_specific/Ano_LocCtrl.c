#include "Ano_LocCtrl.h"
#include "Drv_Gps.h"
#include "Ano_Imu.h"
#include "Ano_FlightCtrl.h"
#include "Ano_OF.h"
#include "Ano_OF_DecoFusion.h"
#include "Ano_Parameter.h"
#include "Ano_UWB.h"


//λ���ٶȻ����Ʋ���
_PID_arg_st loc_arg_1[2] ; 

//λ���ٶȻ���������
_PID_val_st loc_val_1[2] ; 

//λ���ٶȻ��������Ʋ���
_PID_arg_st loc_arg_1_fix[2] ; 

//λ���ٶȻ�������������
_PID_val_st loc_val_1_fix[2] ; 

static u8 mode_f[2];

/*�ǶȻ�PID������ʼ��*/
void Loc_1level_PID_Init()
{
	//GPS
	if(mode_f[1] == 2)
	{
		//normal
		loc_arg_1[X].kp = Ano_Parame.set.pid_gps_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = Ano_Parame.set.pid_gps_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
		//fix	
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = Ano_Parame.set.pid_gps_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];	
	}
	//OF
	else if(mode_f[1] == 1)
	{
		//normal
		loc_arg_1[X].kp = Ano_Parame.set.pid_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0.0f  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = Ano_Parame.set.pid_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
		//fix	
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = Ano_Parame.set.pid_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];	
	}
	//UWB ��UWB AND OF
	else if(mode_f[1] == 3 || mode_f[1] == 4)
	{
		//normal
		loc_arg_1[X].kp = Ano_Parame.set.pid_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0.0f  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = Ano_Parame.set.pid_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
		//fix	
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = Ano_Parame.set.pid_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];			
	}
	//
	else
	{
		//null	
	}
	

}

_loc_ctrl_st loc_ctrl_1;
static float fb_speed_fix[2];

float vel_fb_d_lpf[2];
float vel_fb_h[2],vel_fb_w[2];
float vel_fb_fix_w[2];
/*λ���ٶȻ�*/
void Loc_1level_Ctrl(u16 dT_ms,s16 *CH_N)
{
	static float loc_hand_exp_vel[2]={0};
	static unsigned short waite_gps_loc_cnt = 0;
	float ne_pos_control[2];
	unsigned char vel_diff = 5;
	float pos_ctrl_h_out[2];
	float pos_ctrl_w_out[2];

	
	//����UWB(����)
	if(switchs.uwb_on && (!switchs.of_flow_on) && (!switchs.gps_on))
	{
		mode_f[1] = 3;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}	
		//==
		////
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[X] ;
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[Y] ;
			
	}
	//���й�����UWB
	else if(switchs.uwb_on && switchs.of_flow_on && (!switchs.gps_on))
	{
		mode_f[1] = 4;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}	
		//==	
		//������ֵ
//		loc_ctrl_1.exp[X] = fs.speed_set_h[X];
//		loc_ctrl_1.exp[Y] = fs.speed_set_h[Y];
		h2w_2d_trans(fs.speed_set_h,imu_data.hx_vec,loc_ctrl_1.exp);
		//��ͨ�˲�
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.w_acc[X],vel_fb_d_lpf[X]);
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.w_acc[Y],vel_fb_d_lpf[Y]);		
		//������ֵHXYZ��ˮƽ�������꣩
		if(sens_hd_check.of_ok)
		{
			vel_fb_h[0] = ano_of.of2_dx_fix;//OF_DX2;
			vel_fb_h[1] = ano_of.of2_dy_fix;//OF_DY2;
		}
		else//sens_hd_check.of_df_ok
		{
			vel_fb_h[0] = of_rdf.gnd_vel_est_h[X];
			vel_fb_h[1] = of_rdf.gnd_vel_est_h[Y];	
		}
		//ת��NWU�������죩����
		h2w_2d_trans(vel_fb_h,imu_data.hx_vec,vel_fb_w);
		//������ֵ+���ٶȳ�ǰ
		loc_ctrl_1.fb[X] = vel_fb_w[0] + 0.03f *vel_fb_d_lpf[X];
		loc_ctrl_1.fb[Y] = vel_fb_w[1] + 0.03f *vel_fb_d_lpf[Y];
		//�ٶ�����ֵ��ֵ�����ڻ���
		fb_speed_fix[0] = uwb_data.w_vel_cmps[0];
		fb_speed_fix[1] = uwb_data.w_vel_cmps[1];
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										loc_ctrl_1.fb[i] ,			//����ֵ����
										&loc_arg_1[i], //PID�����ṹ��
										&loc_val_1[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;	
			
			//fix
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										fb_speed_fix[i] ,			//����ֵ����
										&loc_arg_1_fix[i], //PID�����ṹ��
										&loc_val_1_fix[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;	
			
			pos_ctrl_w_out[i] = loc_val_1[i].out + loc_val_1_fix[i].out;	//(PD)+(I)	
		}	
		//NWUתHXYZˮƽ��������
		w2h_2d_trans(pos_ctrl_w_out,imu_data.hx_vec,pos_ctrl_h_out); 
		//�����ֵ
		loc_ctrl_1.out[0] = pos_ctrl_h_out[0];
		loc_ctrl_1.out[1] = pos_ctrl_h_out[1];	
	}
	//���й���
	else if(switchs.of_flow_on && (!switchs.gps_on))
	{
		mode_f[1] = 1;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		loc_ctrl_1.exp[X] = fs.speed_set_h[X];
		loc_ctrl_1.exp[Y] = fs.speed_set_h[Y];
		//
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.h_acc[X],vel_fb_d_lpf[X]);
		LPF_1_(5.0f,dT_ms*1e-3f,imu_data.h_acc[Y],vel_fb_d_lpf[Y]);		
		
		if(sens_hd_check.of_ok)
		{
			loc_ctrl_1.fb[X] = ano_of.of2_dx_fix + 0.03f *vel_fb_d_lpf[X];
			loc_ctrl_1.fb[Y] = ano_of.of2_dy_fix + 0.03f *vel_fb_d_lpf[Y];
			
			fb_speed_fix[0] = ano_of.of2_dx_fix;//OF_DX2FIX;
			fb_speed_fix[1] = ano_of.of2_dy_fix;//OF_DY2FIX;
		}
		else//sens_hd_check.of_df_ok
		{
			loc_ctrl_1.fb[X] = of_rdf.gnd_vel_est_h[X] + 0.03f *vel_fb_d_lpf[X];
			loc_ctrl_1.fb[Y] = of_rdf.gnd_vel_est_h[Y] + 0.03f *vel_fb_d_lpf[Y];
			
			fb_speed_fix[0] = of_rdf.gnd_vel_est_h[X];
			fb_speed_fix[1] = of_rdf.gnd_vel_est_h[Y];		
		}
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										loc_ctrl_1.fb[i] ,			//����ֵ����
										&loc_arg_1[i], //PID�����ṹ��
										&loc_val_1[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;	
			
			//fix
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										fb_speed_fix[i] ,			//����ֵ����
										&loc_arg_1_fix[i], //PID�����ṹ��
										&loc_val_1_fix[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;	
			
			loc_ctrl_1.out[i] = loc_val_1[i].out + loc_val_1_fix[i].out;	//(PD)+(I)	
		}		


	}
	//����GPS
	else if (switchs.gps_on)
	{
		mode_f[1] = 2;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		for(u8 j = 0; j < 2; j++)
		{
			if (fs.speed_set_h[j] != 0)				//�ж��Ƿ񶯿���ҡ��
			{
				if (ABS(loc_hand_exp_vel[j]) < ABS(fs.speed_set_h[j]))	//�ж��ٶ��Ƿ�ﵽ�����ٶ�
				{
					if (loc_hand_exp_vel[j]*fs.speed_set_h[j] < 0)	//�ж�ҡ�˷�������������Ƿ���ͬ
					{
						if (fs.speed_set_h[j] > 0)					//�ж�ҡ�˷���
						{
							fs.speed_set_h[j] = MAX_SPEED;			//��������ٶ�
						}
						else 
						{
							fs.speed_set_h[j] = -MAX_SPEED;
						}
					}
					loc_hand_exp_vel[j] += 0.5f*dT_ms*fs.speed_set_h[j]/MAX_SPEED;		//���������ٶ�
				}
				else												
				{
					if (loc_hand_exp_vel[j] > 0)					//�ظ���Ӧ�������ٶ���Ӧ
					{
						loc_hand_exp_vel[j] -= vel_diff;
					}
					else
					{
						loc_hand_exp_vel[j] += vel_diff;
					}
				}
			}
			else
			{
				if (loc_hand_exp_vel[j] > vel_diff)					//�ظ���Ӧ�������ٶ���Ӧ
				{
					loc_hand_exp_vel[j] -= vel_diff;
				}
				else if (loc_hand_exp_vel[j] <= -vel_diff)
				{
					loc_hand_exp_vel[j] += vel_diff;
				}
				else
				{
					loc_hand_exp_vel[j] = 0;
				}
			}
		}
		if (loc_hand_exp_vel[X] || loc_hand_exp_vel[Y])				//�ж��Ƿ����ֶ������ٶ�
		{
			ne_pos_control[0] = 0;									//λ�ÿ���������
			ne_pos_control[1] = 0;
			waite_gps_loc_cnt = 50;									//�����ٶȹ���֮��ȴ�500ms
		}
		else
		{
			if (waite_gps_loc_cnt > 0)
			{
				ne_pos_control[0] = 0;					//λ�ÿ���������
				ne_pos_control[1] = 0;
				waite_gps_loc_cnt--;
				if (waite_gps_loc_cnt == 0)				//����λ���Ѿ��ȶ� ��¼����λ���Լ���λ�ý��п���
				{
					Gps_information.hope_latitude = Gps_information.latitude_offset;		//����λ�õ��ڵ�ǰλ��
					Gps_information.hope_longitude = Gps_information.longitude_offset;
				}
			}
			else
			{
				Gps_information.hope_latitude_err = Gps_information.hope_latitude - Gps_information.latitude_offset;		//γ�����
				Gps_information.hope_longitude_err = Gps_information.hope_longitude - Gps_information.longitude_offset;		//�������
				length_limit(&(Gps_information.hope_latitude_err), &(Gps_information.hope_longitude_err), MAX_SPEED*1.2f, ne_pos_control);	//����ģ������
			}
		}
		
		
		loc_ctrl_1.exp[X] =  ne_pos_control[0]*Ano_Parame.set.pid_gps_loc_2level[KP] + loc_hand_exp_vel[X]*imu_data.hx_vec[0] - loc_hand_exp_vel[Y]*imu_data.hx_vec[1];		//�����ٶȣ���������ת������������NED��
		loc_ctrl_1.exp[Y] = -ne_pos_control[1]*Ano_Parame.set.pid_gps_loc_2level[KP] + loc_hand_exp_vel[X]*imu_data.hx_vec[1] + loc_hand_exp_vel[Y]*imu_data.hx_vec[0];		

		loc_ctrl_1.fb[X] =  (Gps_information.last_N_vel) + (wcx_acc_use*0.2f);			//�ٶȷ���+���ٶ���ǰ
		loc_ctrl_1.fb[Y] = -(Gps_information.last_E_vel) + (wcy_acc_use*0.2f);
		
		fb_speed_fix[X] =  (Gps_information.last_N_vel);
		fb_speed_fix[Y] = -(Gps_information.last_E_vel);
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										loc_ctrl_1.fb[i] ,			//����ֵ����
										&loc_arg_1[i], //PID�����ṹ��
										&loc_val_1[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;		
			
			//fix
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										fb_speed_fix[i] ,			//����ֵ����
										&loc_arg_1_fix[i], //PID�����ṹ��
										&loc_val_1_fix[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;	
			
			if (!flag.taking_off)
			{
				loc_val_1_fix[i].err_i = 0;
			}				
		}
		//
		pos_ctrl_w_out[0] = loc_val_1[0].out + loc_val_1_fix[0].out;//(PD)+(I)
		pos_ctrl_w_out[1] = loc_val_1[1].out + loc_val_1_fix[1].out;//(PD)+(I)
		w2h_2d_trans(pos_ctrl_w_out, imu_data.hx_vec, pos_ctrl_h_out);	//�������꣨NWU�����ƽ��ת��������������
		loc_ctrl_1.out[X] = pos_ctrl_h_out[0];
		loc_ctrl_1.out[Y] = pos_ctrl_h_out[1];
	}
	//��̬ģʽ��ֱ���������ٶ�תΪ�Ƕȣ������Ƕȣ�
	else
	{
		mode_f[1] = 255;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[X] ;
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[Y] ;
	}
}

_loc_ctrl_st loc_ctrl_2;

