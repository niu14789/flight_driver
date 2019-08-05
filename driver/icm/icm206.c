/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : notify.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
	* BEEP TIM3 CHANNEL1 PWM Gerente
	* LED is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "f_shell.h"
#include "icm206.h"
#include "state.h"
#include "gd32f30x.h"
/* USER CODE BEGIN Includes */
FS_INODE_REGISTER("icm206.d",icm,icm206_heap_init,0);
/* LPF */
#if USED_LPF
/* bias gyro and accel */
static float bias_gyro[3];
static float bias_accel[3];
static float bias_ctrl = 0;
/* define */
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
/* end */
#endif
/* defined functions */
static int icm206_heap_init(void)
{
	/* init as default */
	icm.flip.f_inode = &icm;
	/* file interface  */
	icm.ops.read  = icm206_fread;
	icm.ops.open  = icm206_fopen;
	icm.ops.ioctl = icm206_ioctrl;
	/* return icm206 result */
	return icm206_init();
}
/* file & driver 's interface */
static struct file * icm206_fopen (FAR struct file *filp)
{
	/* return flip data */
	return &icm.flip;
}
/* data read */
static unsigned int icm206_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen)
{
	/* ignore the complier warring */
	(void)filp;
	/* read */
	if( buflen != sizeof(ICM206_INS_DEF) || buffer == NULL )
	{
		/* can not supply this format */
		return 0;
	}
	/* read data */
	icm206_read_sensor(buffer);
	/* use bias and low pass filter */
#if USED_LPF
	/* bias calibrate */
	if( bias_ctrl < 1000 )
	{
		/* gyro */
		bias_gyro[0] += ((ICM206_INS_DEF *)buffer)->gyro[0] / 1000;
		bias_gyro[1] += ((ICM206_INS_DEF *)buffer)->gyro[1] / 1000;
		bias_gyro[2] += ((ICM206_INS_DEF *)buffer)->gyro[2] / 1000;
		/* accel */
		bias_accel[0] += ((ICM206_INS_DEF *)buffer)->accel[0] / 1000;
		bias_accel[1] += ((ICM206_INS_DEF *)buffer)->accel[1] / 1000;
		bias_accel[2] += (1 - ((ICM206_INS_DEF *)buffer)->accel[2]) / 1000;
		/* next station */
		bias_ctrl ++;
		/* return 0 */
		return 0;
	}
	/* calibrate data */
	for( int i = 0 ; i < 3 ; i ++ )
	{
	  ((ICM206_INS_DEF  *)buffer)->gyro[i]  -= bias_gyro[i];
		((ICM206_INS_DEF *)buffer)->accel[i]  -= bias_accel[i];
	}
#endif
	/* return lens */
	return buflen;
}
/* icm206 ioctrl */
static int icm206_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* temp */
	unsigned char * io_tmp;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
			/* icm206_write_reg */
		  io_tmp = pri_data;
		  /* write */
		  icm206_write_reg(io_tmp[0],io_tmp[1]);
		  /* end of cmd */
			break;
		case 1:
			/* icm206_read_reg */
		  ret = icm206_read_reg((unsigned char)arg);
		  /* end of cmd */
			break;
		case 2:
			/* icm206_check_reg */
		  io_tmp = pri_data;
		  /* read and check */
			ret = icm206_check_reg(io_tmp[0],io_tmp[1]);
		  /* end of cmd */
			break;
		case 3:
			/* icm206_read_sensor */
		  if( arg != sizeof(ICM206_INS_DEF) )
			{
				return FS_ERR;//can not supply this format
			}
			/* read */
			icm206_read_sensor(pri_data);
			/* end of cmd */
		case 4:
			/* icm206_init */
		  ret = icm206_init();
		  /* end of cmd */
		default:
			break;
	}
	/* return */
	return ret;
}
/* lowlevel */
static void icm206_delay(unsigned int t)
{
	while(t--);
}
/* icm206_write_reg */
static void icm206_write_reg(unsigned char reg,unsigned char data)
{
	unsigned char d[2] = { reg , data };
	/* reset cs pin */
	gpio_bit_reset(GPIOA, GPIO_PIN_4);
	/* spi0 write one byte */
	spi0_wr_byte(d[0]);
	/* other byte */
	spi0_wr_byte(d[1]);
	/* ret cs pin */
	gpio_bit_set(GPIOA, GPIO_PIN_4);
}
/* icm206_read_reg */
static unsigned char icm206_read_reg(unsigned char reg)
{
	/* Xor read cmd */
	unsigned char d = reg | 0x80;
	/* reset cs pin */
	gpio_bit_reset(GPIOA, GPIO_PIN_4);
	/* spi0 write one byte */
	spi0_wr_byte(d);
	/* read */
	d = spi0_wr_byte(0xff);
	/* ret cs pin */
  gpio_bit_set(GPIOA, GPIO_PIN_4);
	/* return */
	return d;
}
/* write and check regs */
static int icm206_check_reg(unsigned char reg,unsigned char data)
{
	/* default */
	unsigned char tmp_reg;
	/* default try times */
	unsigned char timeout = 0x10;
	/* write and check */
	do
	{
		/* write */
		icm206_write_reg( reg , data );
		/* delay for a while */
    icm206_delay(0xffff);
		/* read */
		tmp_reg = icm206_read_reg(reg);
		/* delay for a while */
		icm206_delay(0xffff);
	}
	while( tmp_reg != data && timeout-- );//check
  /* check ok or not */
	if( timeout == 0xFF )
	{
		return FS_ERR; // timeout , oh no.we were failed
	}
  /* ok . great */
	return FS_OK;
}
/* spi read data */
static void icm206_read_sensor( ICM206_INS_DEF * ins )
{
	/* write reg first */
  unsigned char d = ICM_ACCEL_XOUT_H | 0x80;
	unsigned char buffer[14];
	/* chipselect */
	gpio_bit_reset(GPIOA, GPIO_PIN_4);
	/* write reg */
	spi0_wr_byte(d);
	/* read */
	spi0_multiple_read(buffer,sizeof(buffer));
	/* get */
  gpio_bit_set(GPIOA, GPIO_PIN_4);
  /* change */
  ins->accel[1] = (short)((buffer[0] << 8 ) + buffer[1]) * ACCEL_SENSITIVITY;	
	ins->accel[0] = (short)((buffer[2] << 8 ) + buffer[3]) * ACCEL_SENSITIVITY;
	ins->accel[2] = (short)((buffer[4] << 8 ) + buffer[5]) * ACCEL_SENSITIVITY;
	/* get gyro */
	ins->gyro[1] = (short)((buffer[8]   << 8 ) + buffer[9])  * GYRO_SENSITIVITY * DEG2RAD ; // transfer to rad
	ins->gyro[0] = (short)((buffer[10]  << 8 ) + buffer[11]) * GYRO_SENSITIVITY * DEG2RAD ; // transfer to rad
	ins->gyro[2] = (short)((buffer[12]  << 8 ) + buffer[13]) * GYRO_SENSITIVITY * DEG2RAD ; // transfer to rad
	/* get temperature */
	ins->icm206_temperature = (short)((buffer[6] << 8) + buffer[7]) * TEMPERATURE_SENSITIVITY + 25;
	/* ok */
#if USED_LPF
  for( int i = 0 ; i < 3 ; i ++ ) 
	{
		/* apply lpf2 */
    ins->accel[i] = lpf2pApply(&accLpf[i],ins->accel[i]);
	  ins->gyro[i]  = lpf2pApply(&gyroLpf[i] , ins->gyro[i]);
	}
#endif
}
/* disable mag sensor */
#if USED_MAG
/* icm 9250 get mag */
static void icm206_read_mag( icm206_MAG_DEF * mag )
{
/* write reg first */
  unsigned char d = ICM_MAG_XOUT_L | 0x80;
	unsigned char buffer[6];
	/* chipselect */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	/* write reg */
	HAL_SPI_Transmit(ICM_spi_handle,&d,1,0xffff);
	/* read */
	HAL_SPI_Receive(ICM_spi_handle,buffer,sizeof(buffer),0xffff);
	/* get */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);	
  /* change */	
	mag->mag[0] = (short)((buffer[0] << 8) + buffer[1]);	
	mag->mag[1] = (short)((buffer[2] << 8) + buffer[3]);	
	mag->mag[2] = (short)((buffer[4] << 8) + buffer[5]);	
	/* return */
}
#endif
/* spi base init and cs pin config */
/* USE SPI = SPI0 */
/* PA4 = CS PA5 = SCK PA6 = MISO PA7 = MOSI */
static void icm_spi_init(void)
{
	/* spi param handle */
	spi_parameter_struct Spiparam;
	/* enable clock first */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_SPI0);
	/* config cs pin PA4 */
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);     // CS setting
	/* spi interface init */
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);       // SCK
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6); // MISO
	gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);       // MOSI

	/* SPI0 parameter config */
	Spiparam.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
	Spiparam.device_mode = SPI_MASTER;
	Spiparam.frame_size = SPI_FRAMESIZE_8BIT;
	Spiparam.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	Spiparam.nss = SPI_NSS_SOFT;
	Spiparam.prescale = SPI_PSC_32; // 120MHz / 32 = 3.75Mhz
	Spiparam.endian = SPI_ENDIAN_MSB;
	/* load param */
	spi_init(SPI0, &Spiparam);
	/* enable spi0 */
	spi_enable(SPI0);
}
/* spi0 write and read a byte */
static unsigned char spi0_wr_byte( unsigned char WriteByte )
{
	/* wait until spi is idle */
	while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE))
	{ 
		/* nothing need to do , just waitting */
	}
	/* send a data */
	spi_i2s_data_transmit(SPI0, WriteByte);
	/* wait until spi is idle */
	while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE))
	{ 
		/* nothing need to do , just waitting */
	}
	/* read a data */
	unsigned char rd = spi_i2s_data_receive(SPI0);
	/* reutnr */
	return rd;
}
/* multiple read */
static void spi0_multiple_read(void * data,unsigned int len)
{
	/* transfer to unsigned char pointer */
	unsigned char * dst = (unsigned char *)data;
	/* read */
	for( int i = 0 ; i < len ; i ++ )
	{
		/* read data */
		dst[i] = spi0_wr_byte(0xff);
	}
}
/* icm206 init */
static int icm206_init(void)
{
	/* some default */
	unsigned char ctrl0 = 0x80;
	unsigned char ctrl1 = BIT_I2C_IF_DIS;
	unsigned char ctrl2 = OUT_DATA_RATE;
	unsigned char ctrl3 = GYRO_BADNWIDTH;
	unsigned char ctrl4 = (GYRO_SCALE<<3) | ENABLE_GDLPF;
	unsigned char ctrl5 = ACCEL_SCALE<<3;
	unsigned char ctrl6 = ACCEL_BADNWIDTH | (ENABLE_ADLPF<<3);
  /* decode */
	unsigned char ICM_REG[7] = {ICM_PWR_MGMT_1,ICM_USER_CTRL,ICM_SMPLRT_DIV,ICM_CONFIG,
	                            ICM_GYRO_CONFIG,ICM_ACCEL_CONFIG,ICM_ACCEL_CONFIG2};
	/* reg data that will write into */
	unsigned char CTRL_RE[7] = {0x01,ctrl1,ctrl2,ctrl3,ctrl4,ctrl5,ctrl6};
	/* init spi hardware first */
	icm_spi_init();
	/* reset first */
	icm206_write_reg(ICM_PWR_MGMT_1, ctrl0);
  /* delay for some time */
  icm206_delay(0xffff);
  /* for loop to config */
	for( int i = 0 ; i < 7 ; i++ )
	{
		/* start check */
		if( icm206_check_reg(ICM_REG[i],CTRL_RE[i]) != 0 )
		{
       return FS_ERR; //failed
		}
		/* delay for some time */
		icm206_delay(0xfff0);
		/* end file */
	}
	/* init  */
#if USED_LPF	
	for ( int i = 0 ; i < 3 ; i++ )
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}
#endif
	/* ok . great */
  return FS_OK;
}

/* USER CODE END */

































