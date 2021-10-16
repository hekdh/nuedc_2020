/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：激光模块读取
**********************************************************************************/
#include "Drv_laser.h"
#include "Ano_FcData.h"
#include "Drv_time.h"
#define N 6  //滤波数组数量
u8 LASER_LINKOK = 0;	//0:无效，1：有效，2：测试中
u8 temp_distance;
u16 Laser_height_cm;
u8 Drv_Laser_Init(void)
{
	LASER_LINKOK = 2;
	for(u16 i=0; i<1000; i++)
	{
		if(LASER_LINKOK == 1)
			break;
		else
			Delay_ms(1);
	}
	if(LASER_LINKOK == 2)
		LASER_LINKOK = 0;
	sens_hd_check.tof_ok = LASER_LINKOK;
	return LASER_LINKOK;
}

void Drv_Laser_GetOneByte(u8 data)
{
	static u8 tmp[9];
	static u8 sta = 0;
	static u8 cnt = 0;
	
	if(sta == 0 && data == 0x59)
	{
		tmp[0] = 0x59;
		sta = 1;
	}
	else if(sta == 1)
	{
		if(data == 0x59)
		{
			tmp[1] = 0x59;
			sta = 2;
			cnt = 2;
		}
		else
			sta = 0;
	}
	else if(sta == 2)
	{
		tmp[cnt++] = data;
		if(cnt >= 9)
		{
			u8 sum = 0;
			for(u8 i=0; i<8; i++)
				sum += tmp[i];
			if(sum == tmp[8])	//校验通过
			{
				if(LASER_LINKOK == 2)
					LASER_LINKOK = 1;
				flag.distance= (tmp[2] + ((u16)tmp[3]<<8));
				
			}
			sta = 0;
			cnt = 0;
		}
	}
}

//激光加舵机滤波  滑块加最小值
//滑块的基础上加均值滤波
u8 Min_sliding_filter(u8 value)
{
	u8 min=0;
//建数组储存
	static double ult_Filter_Size[N];
	for(int i=0;i<N;i++)
	{
	ult_Filter_Size[i]=ult_Filter_Size[i+1];				
	}
	ult_Filter_Size[N-1]=value;
		
	//找最小值	
	for(u8 i=1;i<N;i++)
	{
		min=ult_Filter_Size[0];
		if(min>ult_Filter_Size[i])
		min=ult_Filter_Size[i];
	}
	return min;
}
