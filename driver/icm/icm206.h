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

#ifndef __MPU5250_H__
#define __MPU5250_H__

#include "fs.h"
#include "state.h"

#define ICM_SELF_TEST_X_GYRO	0x00
#define ICM_SELF_TEST_Y_GYRO	0x01
#define ICM_SELF_TEST_Z_GYRO	0x02
#define ICM_SELF_TEST_X_ACCEL	0x0D
#define ICM_SELF_TEST_Y_ACCEL	0x0E
#define ICM_SELF_TEST_Z_ACCEL	0x0F
#define ICM_XG_OFFS_USRH		0x13
#define ICM_XG_OFFS_USRL		0x14
#define ICM_YG_OFFS_USRH		0x15
#define ICM_YG_OFFS_USRL		0x16
#define ICM_ZG_OFFS_USRH		0x17
#define ICM_ZG_OFFS_USRL		0x18
#define ICM_SMPLRT_DIV			0x19
#define ICM_CONFIG				0x1A
#define ICM_GYRO_CONFIG			0x1B
#define ICM_ACCEL_CONFIG		0x1C
#define ICM_ACCEL_CONFIG2		0x1D
#define ICM_LP_MODE_CFG			0x1E
#define ICM_ACCEL_WOM_THR		0x1F
#define ICM_FIFO_EN				0x23
#define ICM_FSYNC_INT			0x36
#define ICM_INT_PIN_CFG			0x37
#define ICM_INT_ENABLE			0x38
#define ICM_INT_STATUS			0x3A
#define ICM_ACCEL_XOUT_H		0x3B
#define ICM_ACCEL_XOUT_L		0x3C
#define ICM_ACCEL_YOUT_H		0x3D
#define ICM_ACCEL_YOUT_L		0x3E
#define ICM_ACCEL_ZOUT_H		0x3F
#define ICM_ACCEL_ZOUT_L		0x40
#define ICM_TEMP_OUT_H			0x41
#define ICM_TEMP_OUT_L			0x42
#define ICM_GYRO_XOUT_H			0x43
#define ICM_GYRO_XOUT_L			0x44
#define ICM_GYRO_YOUT_H			0x45
#define ICM_GYRO_YOUT_L			0x46
#define ICM_GYRO_ZOUT_H			0x47
#define ICM_GYRO_ZOUT_L			0x48
#define ICM_SIGNAL_PATH_RESET	0x68
#define ICM_ACCEL_INTEL_CTRL	0x69
#define ICM_USER_CTRL			0x6A
#define ICM_PWR_MGMT_1			0x6B
#define ICM_PWR_MGMT_2			0x6C
#define ICM_FIFO_COUNTH			0x72
#define ICM_FIFO_COUNTL			0x73
#define ICM_FIFO_R_W			0x74
#define ICM_WHO_AM_I			0x75
#define ICM_XA_OFFSET_H			0x77
#define ICM_XA_OFFSET_L			0x78
#define ICM_YA_OFFSET_H			0x7A
#define ICM_YA_OFFSET_L			0x7B
#define ICM_ZA_OFFSET_H			0x7D
#define ICM_ZA_OFFSET_L			0x7E

/* Variables -----------------------------------------------------------------*/
enum Odr { // gyro Output Data Rate
  ODR_1kHz = 0,
  ODR_500Hz,	// default
  ODR_333Hz,
  ODR_250Hz,
  ODR_200Hz,
  ODR_167Hz,
  ODR_142Hz,
  ODR_125Hz,
  ODR_111Hz		//0x08
};

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Abw { // accel bandwidth
  ABW_218_1 = 1,
  ABW_99_0,
  ABW_44_8,
  ABW_21_2,
  ABW_10_2,
  ABW_5_1,
  ABW_420 = 7 //0x07
};

enum Gscale {  // gyro full scale
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,		// default
  GFS_2000DPS         // 0x03
};

enum Gbw { // gyro bandwidth
  GBW_250Hz = 0,
  GBW_176Hz,
  GBW_92Hz,
  GBW_41Hz,
  GBW_20Hz,
  GBW_10Hz,
  GBW_5Hz,
  GBW_3281Hz	// 0x07
};

#define BIT_H_RESET					  0x80
#define BIT_RAW_RDY_EN				0x01
#define BIT_I2C_IF_DIS				0x10
#define BIT_RAW_RDY_EN				0x01
#define BIT_INT_ANYRD_2CLEAR	0x10

#define OUT_DATA_RATE				ODR_1kHz
#define ACCEL_SCALE					AFS_8G
#define ACCEL_BADNWIDTH			ABW_44_8
#define GYRO_SCALE					GFS_1000DPS
#define GYRO_BADNWIDTH			GBW_92Hz
#define ENABLE_ADLPF				0
#define ENABLE_GDLPF				0
#define DISABLE_ADLPF				1
#define DISABLE_GDLPF				3

/* sensitivity */
#define GYRO_SENSITIVITY               ((float)( 2000.f / 65536.f ))
#define ACCEL_SENSITIVITY              ((float)( 16.f / 65536.f   ))
#define TEMPERATURE_SENSITIVITY        ((float)( 1 / 326.8f    ))
	
/* config */

#define USED_LPF              0  /* enable / disable LPF ( low pass filter ) */
#define USED_MAG              0  /* as default . disable the mag sensor */

/* some decleare */

static int icm206_init(void);
int icm206_heap_init(void);
static void icm206_read_sensor( ICM206_INS_DEF * ins );
static unsigned char icm206_read_reg(unsigned char reg);
static void icm206_write_reg(unsigned char reg,unsigned char data);
static int icm206_check_reg(unsigned char reg,unsigned char data);
static int icm206_init(void);
static unsigned char spi0_wr_byte( unsigned char WriteByte );
static void spi0_multiple_read(void * data,unsigned int len);
/* Private includes ----------------------------------------------------------*/
static unsigned int icm206_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
static struct file * icm206_fopen (FAR struct file *filp);
static int icm206_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);
/*---------------*/
#endif




















