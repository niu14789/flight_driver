
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
	* usart is TIM4 CH3 and CH4
  */
/* USER CODE END Header */
#ifndef __USART_H__
#define __USART_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "f_shell.h"
#include "fs_config.h"
/* functions declare */

#define USART_NUM    (4) /* supply amount of usart */

static int usart_heap_init(void);
static int usart_default_config(void);
static int usart_dma_write( unsigned int index , unsigned short txrx_m,const void * data , unsigned int len );
static int usart_init_global( const uart_config_msg * msg );
static unsigned int usart_dma_read(unsigned int index,unsigned int dma_buf_addr,unsigned int dma_deepth,void * rdata,unsigned int rlen);
struct file * usart_fopen(FAR struct file *filp);
static int usart_fwrite(FAR struct file *filp, FAR const void *buffer, unsigned int buflen);
unsigned int usart_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
static int usart_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);

#endif
