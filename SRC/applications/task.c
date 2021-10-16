//�Ƹ˼����ֲ���
#include "task.h"
#include "Drv_OpenMV.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_ultrasonic.h"
#include "math.h"
#include "Ano_Imu.h"
#include "Drv_time.h"
#include "Drv_usart.h"

//������������֡�΢��ϵ��
#define VrI_Kp                  (0.80f)  //������
#define VrI_Kd                  (0.05f)  //΢����
#define VrI_Ki                  (0.01f)  //������
void PID_init();
u8 ct_state_task1()//yaw�����
{
	if(opmv.offline==0)//���ӳ�������Ұ��
	{	
		if(opmv.cb.pos_x>10||opmv.cb.pos_x<-10)//���Ӳ�����Ұ����
		{
			Program_Ctrl_User_Set_YAWdps(opmv.cb.pos_x);
			return 1;
		}
		else
		{	
			Program_Ctrl_User_Set_YAWdps(0);
			//pc_user.vel_cmps_set_h[1]=5;     //�����ƶ�			
			return 2;
		}
	}
	else
	{
		Program_Ctrl_User_Set_YAWdps(10);//��ʧ��Ұ��ʱ��ԭ������
		Program_Ctrl_User_Set_HXYcmps(0,0);
		return 0;
	}
}
//�����ֺͱ���ֽ�ϵ�pid  �߶����üӡ�-�����������ò��ü�

float PID_realize(float ActualSpeed,float SetSpeed)//����ʵ��ֵ,�趨ֵ
 {
	float index=0;
    s16 err=0;                //����ƫ��ֵ
    static float err_last=0;            //������һ��ƫ��ֵ
	float voltage=0;            //�����ѹֵ������ִ�����ı�����
    s16 integral=0;			//�������ֵ			
	float VrI_i;
	err=SetSpeed-ActualSpeed;
	 
	if(ActualSpeed>50&&ActualSpeed<120) //���ַ���
	{
		if(ABS(err)>40)   //��40��120-80���Ͳ���i�� ���޾���120
		{
			index=0;
		}   
		else if(ABS(err)<10)
		{
			index=1.0;
			integral+=err;
		}
		else
		{
			index=(40-ABS(err))/4;
			integral+=err;
		}
	}
	else if(ActualSpeed<=50)//�����ֱ���
	{
		if(ABS(err)>40)       //����ֹ���
		{
			index=0.0;
		}
		else if(ABS(err)<=10)
		{
			index=1.0;
			if(ABS(err)>0)				
				integral+=err;
		}
		else
		{
			index=(40-ABS(err))/4;
			if(abs(err)>0)
				integral+=err;
		}
	}
   else//�����ֱ���
	{
		if(abs(err)>40)       //����ֹ���
		{
			index=0.0;
		}
		else if(abs(err)<=10)
		{
			index=1.0;
			if(abs(err)>0)				
				integral+=err;
		}
		else
		{
			index=(40-abs(err))/4;
			if(abs(err)>0)
				integral+=err;
		}
	}
	VrI_i=index*VrI_Ki*integral;
	VrI_i=LIMIT(VrI_i,-5,5);//i����
   voltage=VrI_Kp*err+VrI_i+VrI_Kd*(err-err_last);

   err_last=err;
   voltage*=1.0;
   return voltage;
}
u16 get_yaw()
{
	return (u16)(imu_data.yaw+180);

}

void task_1()
{
			if(time_data.start_turn_flag==0)
		{
			time_data.start_turn=SysTick_GetTick();
			time_data.start_turn_flag=1;//�Ƹ˿�ʼ��־λ��1
			yaw_ctrl.yaw_get=get_yaw();//��¼��ʼƫ��������
			yaw_ctrl.exp = (u16)(yaw_ctrl.yaw_get+10)%360;//�����Ƕ�����30�����ڿ˷����������
			
		}
		if((SysTick_GetTick()-time_data.start_turn)<=(10*1000))
		{
			ct_state_task1();
			ult_distance_hold(80,opmv.cb.sta);//�������
			pc_user.vel_cmps_set_h[1] = 10;//�����ƶ�
		}			
	
		else if(yaw_ctrl.exp>355)
		{	
			if(
					(
						(get_yaw()>yaw_ctrl.exp) 
						&&
						(get_yaw()<0)
				  )
						||
					(
					 (get_yaw()>0)
						&&
					 (get_yaw()<((yaw_ctrl.exp+5)%360))
					)
				)//����λ�ú�ͣ��[exp,0]||[0,(exp+5)%360]
			{
				Program_Ctrl_User_Set_YAWdps(0);
				Program_Ctrl_User_Set_HXYcmps(0,0);
				yaw_ctrl.flag++;
			}
			else
			{
				ct_state_task1();
				ult_distance_hold(80,opmv.cb.sta);//�������
				pc_user.vel_cmps_set_h[1] = 10;//�����ƶ�
			}
		}
		else if(yaw_ctrl.exp<=355)
		{	
			if((get_yaw()>yaw_ctrl.exp) &&(get_yaw()<(yaw_ctrl.exp+5)))//����λ�ú�ͣ��
			{
				Program_Ctrl_User_Set_YAWdps(0);
				Program_Ctrl_User_Set_HXYcmps(0,0);
				yaw_ctrl.flag++;
			}
			else
			{
				ct_state_task1();
				ult_distance_hold(80,opmv.cb.sta);//�������
				pc_user.vel_cmps_set_h[1] = 10;//�����ƶ�
			}
		}
		else
		{
			ct_state_task1();
			ult_distance_hold(60,opmv.cb.sta);//�������
			pc_user.vel_cmps_set_h[1] = 10;//�����ƶ�
		}

}
void task_2()
{
	if(time_data.start_move_flag==0)
	{
		time_data.start_move=SysTick_GetTick();
		time_data.start_move_flag=1;
	}
	if((SysTick_GetTick()-time_data.start_move)<=(10*1000))
	{
		pc_user.vel_cmps_set_h[1] = 10;//�����ƶ�1m
		pc_user.vel_cmps_set_h[0] = 0;
		Program_Ctrl_User_Set_YAWdps(0);
		u8 m[]= {2,2,2,2,2,2,2,2,2,2};
		Usart3_Send(m,7);
	}
	else if(opmv.offline==0&&opmv.cb.color_flag==2)
	{
		yaw_ctrl.flag++;
		//��ձ�־λ
		time_data.start_turn_flag=0;
		time_data.take_off_flag=0;
		yaw_ctrl.ct=0;
		time_data.start_move_flag=0;
	}
	else
	{
		u8 m[]= {2,2,2,2,2,2,2,2,2,2};
		Usart3_Send(m,7);
		pc_user.vel_cmps_set_h[1] = 0;//ͣ��
		pc_user.vel_cmps_set_h[0] = 0;
	}
}
//void ct_state_task2()
//{
//	if(ct_state_task1()==2)
//	{
//		pc_user.vel_cmps_set_h[1]=5;     //�����ƶ�
////		if(flag.distance<35)pc_user.vel_cmps_set_h[0]=LIMIT(opmv.cb.sta-30,0,20);//���Ƶ��˵ľ���
////		else if(flag.distance>50)pc_user.vel_cmps_set_h[0]=LIMIT(80-opmv.cb.sta,0,20);
////		else pc_user.vel_cmps_set_h[0]=0;
//	}
//	else
//	{
//		pc_user.vel_cmps_set_h[1]=0;
//	}
//}


//����ϵͳʱ������ʱ����ʱ����