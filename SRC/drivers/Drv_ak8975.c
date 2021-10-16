/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：电子罗盘驱动
**********************************************************************************/
#include "drv_ak8975.h"
#include "include.h"
#include "Drv_spi.h"

void Drv_AK8975CSPin_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(AK8975_CS_RCC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = AK8975_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(AK8975_CS_GPIO, &GPIO_InitStructure);
	
	GPIO_SetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
}

static void ak8975_enable(u8 ena)
{
	if(ena)
		GPIO_ResetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
	else
		GPIO_SetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
}

u8 DrvAK8975Check(void)
{
	u8 _tmp;
	ak8975_enable(1);
	Drv_SPI2_RW(AK8975_WIA_REG|0x80);
	_tmp = Drv_SPI2_RW(0xFF);
	ak8975_enable(0);
	if(_tmp == 0x48)
		return 1;
	else
		return 0;
}

static void ak8975_Trig(void)
{
	ak8975_enable(1);
	Drv_SPI2_RW(AK8975_CNTL_REG);
	Drv_SPI2_RW(0x01);
	ak8975_enable(0);
}

static u8 ak8975_buf[6];
void DrvAk8975Read(void)
{	
	
	ak8975_enable(1);
	Drv_SPI2_RW(AK8975_HXL_REG|0x80);
	for(u8 i=0; i<6; i++)
		ak8975_buf[i] = Drv_SPI2_RW(0xff);
	ak8975_enable(0);
	
	ak8975_Trig();
	
}

//拓空者PRO-088
void Mag_Get(s16 mag_val[3])
{
	s16 tmp[3];
	
	tmp[0] = ((((int16_t)ak8975_buf[1]) << 8) | ak8975_buf[0]) ;
	tmp[1] = ((((int16_t)ak8975_buf[3]) << 8) | ak8975_buf[2]) ;
	tmp[2] = ((((int16_t)ak8975_buf[5]) << 8) | ak8975_buf[4]) ;
	
	/*转换坐标轴为ANO坐标*/
	mag_val[0] = -tmp[0];
	mag_val[1] = +tmp[1];
	mag_val[2] = -tmp[2];
}


/******************* (C) COPYRIGHT 2017 ANO TECH *****END OF FILE************/

