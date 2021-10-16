#include "Drv_ultrasonic.h"
#include "stm32f4xx.h" 
#include "Ano_FlightCtrl.h"
#include "ANO_FcData.h"
#include "Drv_OpenMV.h"
#include "Ano_ProgramCtrl_User.h"
#include "Ano_Math.h"
#include "Ano_AltCtrl.h"
#include "task.h"
#define Filter_Size 10	 //�����˲�����
#define filt_pere   0.01 //�˲�ϵ��
#define Threshold_1   20 //��ֵ1����һ�״����˲������仯������ڴ�ֵʱ��������
#define Threshold_2   10 //��ֵ1����һ�״����˲���������ֵ���ڴ�ֵʱ�������������ǿ�˲�����
#define N 25  //��ֵ�˲���������
//ioesr05���������ݽ��պ���
u8 UltraSonic_buf[10]; 
u16 raw_distance;
u8 filter_commom(u8 nowData,float a);
u8 Search(double a[]);//��������ֵ���±�
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
//		if(bytedata >0x07)//(bytedata==0x29)δȷ��
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
//			//�����ɹ�
//			UltraSonic_Data_Analysis(UltraSonic_buf,4);
//			rec_sta=0;
//		}
//		else
//		{

//			rec_sta=0;
//		}		
//	}
}


//���뱣��
void ult_distance_hold(float ep_alt,float alt_data)
{
	if((flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)&&(flag.auto_take_off_land != AUTO_LAND)&&(flag.offline==0)) 
	{
		
		if((alt_data>=(ep_alt+10))||(alt_data<=(ep_alt-10)))//����pitch�̿ص�ƫ�Χ  ��������10
		{
			float ep_alt_temp;

			//ep_alt_temp= -pitch_hold_pid(ep_alt,alt_data,0.8,0.02,0.05);//p��С	
			ep_alt_temp=-PID_realize(alt_data,ep_alt);//�����&���뿹����pid
			ep_alt_temp=LIMIT( ep_alt_temp,-20,20);//�ٶ�����20cm/s
			pc_user.vel_cmps_set_h[0] = ep_alt_temp;
				
		}		
		else 
			pc_user.vel_cmps_set_h[0] = 0;
	}

}

//���뱣��pid, �߶�pid�ĸİ棨���βη�����Σ�
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
		pox_pid_i=LIMIT( pox_pid_i,0,5);//i����
		pox_pid=php_p*pox_pid_p+pox_pid_i+php_d*pox_pid_d;//pϵ��Ϊ0.8��iϵ��Ϊ0.02��dϵ��Ϊ0.05
	}
	else 
	{
		pox_pid_p=(pid_setalt-pid_actualalt);
		pox_pid_i+=php_i*pox_pid_p;
		pox_pid_d=(ep_pox_old-opmv_pox_old)-(pid_setalt-pox_pid_p);
		if((-pox_pid_i)>5) pox_pid_i=-5;//i����
		pox_pid=php_p*pox_pid_p+pox_pid_i+php_d*pox_pid_d;//pϵ��Ϊ0.8��iϵ��Ϊ0.02��dϵ��Ϊ0.05
	}	
	
	ep_pox_old=pid_setalt;
	opmv_pox_old=pid_actualalt;
	
	return pox_pid;
}
//��ͨ�˲���ͨ�棬Ϊ�����˲�������
u8 filter_commom(u8 nowData,float a)
{
	u8 nowOutData;
	static u16 oldOutData=0;
    nowOutData = a * nowData  + (1.0f - a) * oldOutData;
    oldOutData = nowOutData;
    return nowOutData;  
}


//��ͨ�˲����װ�
float k_x=0;//�˲�ϵ��
u8 new_flag_x=0;//���ݱ仯����1Ϊͬ��0Ϊ��ͬ
u8 num_x;//�˲�������
/*
��ڣ� NEW_DATA    �²����ľ���ֵ
      OLD_DATA    �ϴ��˲���õľ�����
      k           �˲�ϵ��(�������˲�����е�Ȩ��)
      flag        �ϴ����ݱ仯����
���ڣ� result      �����˲�������
*/
u16 filter(u16 NEW_distance,float k)
{
	static u16 OLD_distance;
	static u8 flag=1;
	//�Ƕȱ仯����
	if((NEW_distance-OLD_distance)>0)
	{
	new_flag_x=1;
	}
	else if((NEW_distance-OLD_distance)<0)
	{
	new_flag_x=0;
	}
	
	//������Ȩ
	if(new_flag_x==flag)
	{
		num_x++;
		//����仯����Threshold_1ʱ���������������ӣ��Կ�������k,��߸�����
		if(ABS(NEW_distance-OLD_distance)>Threshold_1) num_x+=5;
		//����仯������ݼ��ٶȴ�һ������ʱ������kֵ
		if(num_x>Threshold_2)
		{
			k_x=k+0.2;   //0.2Ϊk_x������ֵ����ʵ���޸�
			num_x=0;
		}
	}
	else
	{
		num_x=0;
		k_x=0.01; //�仯�ȶ�ʱ��k_xֵ����ʵ���޸�
	}
	OLD_distance=(1-k_x)*OLD_distance+k_x*NEW_distance;
	flag=new_flag_x;
	
	return OLD_distance;
}
//��������ֵ


//�˲� �����˲�
void sliding_filter(u16 value)
{
//�����鴢��
	static u16 ult_Filter_Size[Filter_Size];
	for(int i=0;i<Filter_Size;i++)
	{
	ult_Filter_Size[i]=ult_Filter_Size[i+1];				
	}
	ult_Filter_Size[Filter_Size-1]=value;
//�޷�
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
//����Ļ����ϼӾ�ֵ�˲�
u8 Average_sliding_filter(u8 value)
{
	u16 sum=0;
//�����鴢��
	static double ult_Filter_Size[N];
	for(int i=0;i<N;i++)
	{
	ult_Filter_Size[i]=ult_Filter_Size[i+1];				
	}
	ult_Filter_Size[N-1]=value;
		
	//�������ֵ	
	for(u8 i=0;i<N;i++)
	{
		sum+=ult_Filter_Size[i];
	}
	return sum/N;
}


//����Ļ����ϼ���ֵ�˲�

u8 Median_sliding_filter(u8 value)
{
	u16 midn_sld_f;
//�����鴢��
	static double ult_Filter_Size[N];
	for(int i=0;i<N;i++)
	{
	ult_Filter_Size[i]=ult_Filter_Size[i+1];				
	}
	ult_Filter_Size[N-1]=value;
		
	//��������ֵ	
	u8 mid=Search(&ult_Filter_Size[N]);
	midn_sld_f=ult_Filter_Size[mid];
//	return midn_sld_f;
	
}
//��������ֵ���±�
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



	/*ks103����������*/ //0xD0,0x02,0xb4
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