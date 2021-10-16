#include "Drv_ultrasonic.h"
#include "stm32f4xx.h" 
#include "Ano_FlightCtrl.h"
#include "ANO_FcData.h"
#include "Drv_OpenMV.h"
#include "Ano_ProgramCtrl_User.h"
#include "Ano_Math.h"
#include "Ano_AltCtrl.h"
#include "task.h"
#define Filter_Size 10	 //滑动滤波容量
#define filt_pere   0.01 //滤波系数
#define Threshold_1   20 //阈值1用于一阶带参滤波器，变化距离大于此值时计数增加
#define Threshold_2   10 //阈值1用于一阶带参滤波器，计数值大于此值时，增大参数，增强滤波跟随
#define N 25  //中值滤波数组数量
//ioesr05超声波数据接收函数
u8 UltraSonic_buf[10]; 
u16 raw_distance;
u8 filter_commom(u8 nowData,float a);
u8 Search(double a[]);//找数组中值的下标
u8 Median_sliding_filter(u8 value);
u8 Average_sliding_filter(u8 value);
static void UltraSonic_Data_Analysis(u8 *buf_data,u8 len)
{
if(buf_data[0]==0xff)
{
	if(buf_data[1]<=7)
	{
		flag.offline=0;
		raw_distance= buf_data[1]*256+buf_data[2];

	}
	else
	{
		flag.offline=1;
		//flag.distance= 0;
	}				
}

}



void UltraSonic_Byte_Get(u8 bytedata)
{
	flag.distance=Median_sliding_filter(bytedata);
//	flag.distance=Average_sliding_filter(flag.distance);
//	static u8 rec_sta;
//	u8 check_val=0;
//	
//	//
//	UltraSonic_buf[rec_sta] = bytedata;
//	//
//	if(rec_sta==0)
//	{
//		if(bytedata==0xff)
//		{
//			rec_sta++;
//		}
//		else
//		{
//			rec_sta=0;
//		}
//	}
//	else if(rec_sta==1)
//	{
//		if(bytedata >0x07)//(bytedata==0x29)未确定
//		{
//			rec_sta=0;
//						flag.offline=1;
//				//flag.distance= 0xffff;
//		}	
//		else 
//		{
//			rec_sta++;
//		}		
//	}
//	else if(rec_sta==2)
//	{
//			rec_sta++;
//		
//	}
//	else if(rec_sta==3)
//	{
//			for(u8 i=0;i<3;i++)
//		{
//			check_val += UltraSonic_buf[i];
//		}
//		//
//		if(check_val == bytedata)
//		{
//			//解析成功
//			UltraSonic_Data_Analysis(UltraSonic_buf,4);
//			rec_sta=0;
//		}
//		else
//		{

//			rec_sta=0;
//		}		
//	}
}


//距离保持
void ult_distance_hold(float ep_alt,float alt_data)
{
	if((flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)&&(flag.auto_take_off_land != AUTO_LAND)&&(flag.offline==0)) 
	{
		
		if((alt_data>=(ep_alt+10))||(alt_data<=(ep_alt-10)))//不加pitch程控的偏差范围  超声波是10
		{
			float ep_alt_temp;

			//ep_alt_temp= -pitch_hold_pid(ep_alt,alt_data,0.8,0.02,0.05);//p改小	
			ep_alt_temp=-PID_realize(alt_data,ep_alt);//变积分&分离抗饱和pid
			ep_alt_temp=LIMIT( ep_alt_temp,-20,20);//速度限制20cm/s
			pc_user.vel_cmps_set_h[0] = ep_alt_temp;
				
		}		
		else 
			pc_user.vel_cmps_set_h[0] = 0;
	}

}

//距离保持pid, 高度pid的改版（加形参方便调参）
float pitch_hold_pid(float ep_distance,float distance,float php_p,float php_i,float php_d)//0.8 0.02 0.05
{

	float pox_pid,pox_pid_p=0,pox_pid_i=0,pox_pid_d=0;
	static	float ep_pox_old=0,opmv_pox_old=0;
	float pid_setalt,pid_actualalt;
	pid_setalt=ep_distance;
	pid_actualalt=distance;
	
	if((pid_setalt-pid_actualalt)>0)
	{
		pox_pid_p=(pid_setalt-pid_actualalt);
		pox_pid_i+=php_i*pox_pid_p;
		pox_pid_d=(ep_pox_old-opmv_pox_old)-(pid_setalt-pox_pid_p);
		pox_pid_i=LIMIT( pox_pid_i,0,5);//i限制
		pox_pid=php_p*pox_pid_p+pox_pid_i+php_d*pox_pid_d;//p系数为0.8，i系数为0.02，d系数为0.05
	}
	else 
	{
		pox_pid_p=(pid_setalt-pid_actualalt);
		pox_pid_i+=php_i*pox_pid_p;
		pox_pid_d=(ep_pox_old-opmv_pox_old)-(pid_setalt-pox_pid_p);
		if((-pox_pid_i)>5) pox_pid_i=-5;//i限制
		pox_pid=php_p*pox_pid_p+pox_pid_i+php_d*pox_pid_d;//p系数为0.8，i系数为0.02，d系数为0.05
	}	
	
	ep_pox_old=pid_setalt;
	opmv_pox_old=pid_actualalt;
	
	return pox_pid;
}
//低通滤波普通版，为二次滤波作试验
u8 filter_commom(u8 nowData,float a)
{
	u8 nowOutData;
	static u16 oldOutData=0;
    nowOutData = a * nowData  + (1.0f - a) * oldOutData;
    oldOutData = nowOutData;
    return nowOutData;  
}


//低通滤波进阶版
float k_x=0;//滤波系数
u8 new_flag_x=0;//数据变化方向，1为同，0为不同
u8 num_x;//滤波计数器
/*
入口： NEW_DATA    新采样的距离值
      OLD_DATA    上次滤波获得的距离结果
      k           滤波系数(代表在滤波结果中的权重)
      flag        上次数据变化方向
出口： result      本次滤波距离结果
*/
u16 filter(u16 NEW_distance,float k)
{
	static u16 OLD_distance;
	static u8 flag=1;
	//角度变化方向
	if((NEW_distance-OLD_distance)>0)
	{
	new_flag_x=1;
	}
	else if((NEW_distance-OLD_distance)<0)
	{
	new_flag_x=0;
	}
	
	//计数分权
	if(new_flag_x==flag)
	{
		num_x++;
		//距离变化大于Threshold_1时，计数器快速增加，以快速增大k,提高跟随性
		if(ABS(NEW_distance-OLD_distance)>Threshold_1) num_x+=5;
		//距离变化递增或递减速度达一定速率时，增大k值
		if(num_x>Threshold_2)
		{
			k_x=k+0.2;   //0.2为k_x的增长值，看实际修改
			num_x=0;
		}
	}
	else
	{
		num_x=0;
		k_x=0.01; //变化稳定时的k_x值，依实际修改
	}
	OLD_distance=(1-k_x)*OLD_distance+k_x*NEW_distance;
	flag=new_flag_x;
	
	return OLD_distance;
}
//超声波赋值


//滤波 滑块滤波
void sliding_filter(u16 value)
{
//建数组储存
	static u16 ult_Filter_Size[Filter_Size];
	for(int i=0;i<Filter_Size;i++)
	{
	ult_Filter_Size[i]=ult_Filter_Size[i+1];				
	}
	ult_Filter_Size[Filter_Size-1]=value;
//限幅
	u16 distance_max=200,distance_min=0;
	for(int j=0;j<Filter_Size;j++)
	{
	if(ult_Filter_Size[j]>distance_max)
	{
		ult_Filter_Size[j]=distance_max;
	}
	if(ult_Filter_Size[j]<distance_min)
	{
		ult_Filter_Size[j]=distance_min;
	}
	
	}
	
}
//滑块的基础上加均值滤波
u8 Average_sliding_filter(u8 value)
{
	u16 sum=0;
//建数组储存
	static double ult_Filter_Size[N];
	for(int i=0;i<N;i++)
	{
	ult_Filter_Size[i]=ult_Filter_Size[i+1];				
	}
	ult_Filter_Size[N-1]=value;
		
	//排序求均值	
	for(u8 i=0;i<N;i++)
	{
		sum+=ult_Filter_Size[i];
	}
	return sum/N;
}


//滑块的基础上加中值滤波

u8 Median_sliding_filter(u8 value)
{
	u16 midn_sld_f;
//建数组储存
	static double ult_Filter_Size[N];
	for(int i=0;i<N;i++)
	{
	ult_Filter_Size[i]=ult_Filter_Size[i+1];				
	}
	ult_Filter_Size[N-1]=value;
		
	//排序找中值	
	u8 mid=Search(&ult_Filter_Size[N]);
	midn_sld_f=ult_Filter_Size[mid];
//	return midn_sld_f;
	
}
//求数组中值的下标
u8 Search(double a[])
{
	int i,j,flag;
	double tmp;
	for(i=N-1;i>=0;i--)
	{
		flag=0;
		for(j=0;j<i;j++)
		{
			if((a[j]-a[j+1])>0)
			{
				tmp=a[j];
				a[j]=a[j+1];
				a[j+1]=tmp;
				flag++;
			}
		}
		if(flag==0) break;
	}
	if(N%2)
	return N/2+1;
	else
	return N/2;
}



	/*ks103超声波发送*/ //0xD0,0x02,0xb4
//	
//		if(ult_send_re==ult_can_send)
//		{
//			for(u8 i=0;i<3;i++)
//			{
//				uint32_t tnow_temp= SysTick_GetTick();
//				if(tnow_temp==time_flag+0.1)
//				{
//				
//				Usart2_Send (&ult_commend[i],1);
//				time_flag= SysTick_GetTick();
//				ult_can_send=0;
//				}				
//			}
//	
//		}