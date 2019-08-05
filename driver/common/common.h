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

#ifndef __COMMON_H__
#define __COMMON_H__

#include "state.h"

/* declares */
static int common_default_config(void);
static int common_heap_init(void);
static void system_run_thread(void);
/* all above functions are for export */
extern unsigned long long read_sys_time_us(void);
int user_read_gps(GPS_User_t * state);
int user_read_imu(ICM206_INS_DEF * icm);
int user_set_pwm(unsigned short * pwm_t,unsigned short len);
int user_set_led(unsigned short mode );
int user_update_log( const void * edata ,unsigned short len );
/* end of files */
#endif


