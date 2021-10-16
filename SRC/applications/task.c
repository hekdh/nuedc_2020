//绕杆及各种测试
#include "task.h"
#include "Drv_OpenMV.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_ultrasonic.h"
#include "math.h"
#include "Ano_Imu.h"
#include "Drv_time.h"
#include "Drv_usart.h"

//定义比例、积分、微分系数
#define VrI_Kp                  (0.80f)  //比例项
#define VrI_Kd                  (0.05f)  //微分项
#define VrI_Ki                  (0.01f)  //积分项
void PID_init();
u8 ct_state_task1()//yaw轴控制
{
	if(opmv.offline==0)//杆子出现在视野中
	{	
		if(opmv.cb.pos_x>10||opmv.cb.pos_x<-10)//杆子不在视野中央
		{
			Program_Ctrl_User_Set_YAWdps(opmv.cb.pos_x);
			return 1;
		}
		else
		{	
			Program_Ctrl_User_Set_YAWdps(0);
			//pc_user.vel_cmps_set_h[1]=5;     //横向移动			
			return 2;
		}
	}
	else
	{
		Program_Ctrl_User_Set_YAWdps(10);//丢失视野的时候原地自旋
		Program_Ctrl_User_Set_HXYcmps(0,0);
		return 0;
	}
}
//抗积分和变积分结合的pid  高度引用加“-”，距离引用不用加

float PID_realize(float ActualSpeed,float SetSpeed)//定义实际值,设定值
 {
	float index=0;
    s16 err=0;                //定义偏差值
    static float err_last=0;            //定义上一个偏差值
	float voltage=0;            //定义电压值（控制执行器的变量）
    s16 integral=0;			//定义积分值			
	float VrI_i;
	err=SetSpeed-ActualSpeed;
	 
	if(ActualSpeed>50&&ActualSpeed<120) //积分分离
	{
		if(ABS(err)>40)   //到40（120-80）就不加i了 极限距离120
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
	else if(ActualSpeed<=50)//抗积分饱和
	{
		if(ABS(err)>40)       //变积分过程
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
   else//抗积分饱和
	{
		if(abs(err)>40)       //变积分过程
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
	VrI_i=LIMIT(VrI_i,-5,5);//i限制
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
			time_data.start_turn_flag=1;//绕杆开始标志位置1
			yaw_ctrl.yaw_get=get_yaw();//记录初始偏航角数据
			yaw_ctrl.exp = (u16)(yaw_ctrl.yaw_get+10)%360;//期望角度增加30，用于克服磁力计误差
			
		}
		if((SysTick_GetTick()-time_data.start_turn)<=(10*1000))
		{
			ct_state_task1();
			ult_distance_hold(80,opmv.cb.sta);//距离控制
			pc_user.vel_cmps_set_h[1] = 10;//横向移动
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
				)//到达位置后停下[exp,0]||[0,(exp+5)%360]
			{
				Program_Ctrl_User_Set_YAWdps(0);
				Program_Ctrl_User_Set_HXYcmps(0,0);
				yaw_ctrl.flag++;
			}
			else
			{
				ct_state_task1();
				ult_distance_hold(80,opmv.cb.sta);//距离控制
				pc_user.vel_cmps_set_h[1] = 10;//横向移动
			}
		}
		else if(yaw_ctrl.exp<=355)
		{	
			if((get_yaw()>yaw_ctrl.exp) &&(get_yaw()<(yaw_ctrl.exp+5)))//到达位置后停下
			{
				Program_Ctrl_User_Set_YAWdps(0);
				Program_Ctrl_User_Set_HXYcmps(0,0);
				yaw_ctrl.flag++;
			}
			else
			{
				ct_state_task1();
				ult_distance_hold(80,opmv.cb.sta);//距离控制
				pc_user.vel_cmps_set_h[1] = 10;//横向移动
			}
		}
		else
		{
			ct_state_task1();
			ult_distance_hold(60,opmv.cb.sta);//距离控制
			pc_user.vel_cmps_set_h[1] = 10;//横向移动
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
		pc_user.vel_cmps_set_h[1] = 10;//横向移动1m
		pc_user.vel_cmps_set_h[0] = 0;
		Program_Ctrl_User_Set_YAWdps(0);
		u8 m[]= {2,2,2,2,2,2,2,2,2,2};
		Usart3_Send(m,7);
	}
	else if(opmv.offline==0&&opmv.cb.color_flag==2)
	{
		yaw_ctrl.flag++;
		//清空标志位
		time_data.start_turn_flag=0;
		time_data.take_off_flag=0;
		yaw_ctrl.ct=0;
		time_data.start_move_flag=0;
	}
	else
	{
		u8 m[]= {2,2,2,2,2,2,2,2,2,2};
		Usart3_Send(m,7);
		pc_user.vel_cmps_set_h[1] = 0;//停下
		pc_user.vel_cmps_set_h[0] = 0;
	}
}
//void ct_state_task2()
//{
//	if(ct_state_task1()==2)
//	{
//		pc_user.vel_cmps_set_h[1]=5;     //横向移动
////		if(flag.distance<35)pc_user.vel_cmps_set_h[0]=LIMIT(opmv.cb.sta-30,0,20);//控制到杆的距离
////		else if(flag.distance>50)pc_user.vel_cmps_set_h[0]=LIMIT(80-opmv.cb.sta,0,20);
////		else pc_user.vel_cmps_set_h[0]=0;
//	}
//	else
//	{
//		pc_user.vel_cmps_set_h[1]=0;
//	}
//}


//利用系统时间来延时的延时函数