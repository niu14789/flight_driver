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

#ifndef __ALGO_H__
#define __ALGO_H__
/* some data */
static int algo_heap_init(void);
static int algo_default_config(void);
static void algo_task_1ms(void);
static void algo_task_4ms(void);
static void algo_task_10ms(void);
static void algo_task_20ms(void);
static void algo_task_100ms(void);
static void algo_task_idle(void);
static void algo_task_500ms(void);

#endif



/* end of file */
