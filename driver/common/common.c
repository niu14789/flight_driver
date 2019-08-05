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
	* common is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "f_shell.h"
#include "gd32f30x.h"
#include "runtime.h"
#include "string.h"
#include "fs_config.h"
#include "f_ops.h"
#include "state.h"
#include "common.h"
/* fs inode system register */
FS_INODE_REGISTER("/common.o",common,common_heap_init,0);
/* system time define */
static unsigned int system_run_ms = 0;
/* regrister a system task */
FS_SHELL_STATIC(system_run_thread,system_run_thread,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD0_ID));
/* define some data */
static struct file * imu , * gps , * led , * pwm , * log;
/* heap init */
static int common_heap_init(void)
{
  /* full of zero */
	memset(&common,0,sizeof(common));
	/* shell base */
	common.shell_i = shell_sched_getfiles();
	/* driver config */
	common.config = common_default_config;
	/* file interface */
	common.flip.f_inode = &common;
	common.flip.f_path = "/common.o";
	/* heap */
	
	/* add your own code here */
  common.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* common_default_config led init */
static int common_default_config(void)
{
	/* open imu */
	imu = open("icm206.d",__FS_OPEN_ALWAYS);
	/* open gps */
	gps = open("gps.o",__FS_OPEN_ALWAYS);
	/* open led */
	led = open("led.o",__FS_OPEN_ALWAYS);
	/* open pwm */
	pwm = open("pwm.o",__FS_OPEN_ALWAYS);
	/* open log */
	log = open("log.o",__FS_OPEN_ALWAYS);
	/*----------*/
	return FS_OK;
}
/* static get system time */
static void system_run_thread(void)
{
	system_run_ms++;
}
/* common function */
/* get system time */
unsigned long long read_sys_time_us(void)
{
	return ( system_run_ms * 1000 + timer_counter_read(TIMER2) ) ;
}
/* user read gps data */
int user_read_gps(GPS_User_t * state)
{
	/* open gps ok ? */
	if( gps == NULL )
	{
		return FS_OK;// oh on . we got nothing
	}
	/* ok . let's roll it */
	int ret = fs_read(gps,state,sizeof(GPS_User_t));
	/* return ret */
	return ret;
}
/* int user read imu data */
int user_read_imu(ICM206_INS_DEF * icm)
{
	/* open gps ok ? */
	if( imu == NULL )
	{
		return FS_OK;// oh on . we got nothing
	}	
	/* ok . let's roll it */
	int ret = fs_read(imu,icm,sizeof(ICM206_INS_DEF));
	/* return */
	return ret;
}
/* user set pwm */
int user_set_pwm(unsigned short * pwm_t,unsigned short len)
{
  /* open gps ok ? */
	if( pwm == NULL )
	{
		return FS_OK;// oh on . we got nothing
	}		
	/* ok . let's roll it */
	// rev
	return FS_OK;
}
/* user set led mode */
int user_set_led(unsigned short mode )
{
  /* open gps ok ? */
	if( led == NULL )
	{
		return FS_OK;// oh on . we got nothing
	}	
	/* ok . let's roll it */
	// rev
	return FS_OK;	
}
/* user save log */
int user_update_log( const void * edata ,unsigned short len )
{
  /* open gps ok ? */
	if( log == NULL )
	{
		return FS_OK;// oh on . we got nothing
	}	
	/* ok . let's roll it */	
	fs_write(log,edata,len);
	/* return ok */
	return FS_OK;
}
/* end of file */











