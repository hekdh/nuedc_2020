/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：电子罗盘驱动
**********************************************************************************/
#include "drv_ak09915.h"
#include "include.h"
#include "Drv_spi.h"

#define AK09915_WIA_REG          0X00 
#define AK09915_INFO_REG         0X01 
#define AK09915_ST1_REG			0x10
#define AK09915_HXL_REG			0x11
#define AK09915_CNTL2_REG		0x31

//


static void ak09915_enable(u8 ena)
{
	if(ena)
		GPIO_ResetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
	else
		GPIO_SetBits(AK8975_CS_GPIO, AK8975_CS_PIN);
}

static void ak09915WriteOneByte(u8 addr, u8 data)
{
	ak09915_enable(1);
	Drv_SPI2_RW(addr);
	Drv_SPI2_RW(data);
	ak09915_enable(0);
}

u8 DrvAK09915Check(void)
{
	u8 _tmp;
	ak09915_enable(1);
	Drv_SPI2_RW(AK09915_INFO_REG|0x80);
	_tmp = Drv_SPI2_RW(0xff);
	ak09915_enable(0);
	
	if(_tmp==0x10)
		return 1;
	else
		return 0;
}

void DrvAk09915Init(void)
{	
	ak09915WriteOneByte(AK09915_CNTL2_REG,0x4A);
}

_ak09915_data_un ak09915Buf;
void DrvAk09915Read(void)
{
	ak09915_enable(1);
	Drv_SPI2_RW(AK09915_ST1_REG|0x80);
	for(u8 i=0; i<9; i++)
		ak09915Buf.u8_buffer[i] = Drv_SPI2_RW(0xff);
	ak09915_enable(0);
}

