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
#include "fs_config.h"
#include "string.h"
#include "f_ops.h"
/* some functions */
static int log_heap_init(void);
static int log_default_config(void);
/* file & driver 's interface */
static struct file * log_fopen (FAR struct file *filp);
/* file interface to read log data */
static int log_fwrite(FAR struct file *filp, FAR const void * buffer, unsigned int buflen);
/* fs inode system register */
FS_INODE_REGISTER("log.o",log,log_heap_init,0);
/* some test define */
static struct file * usart3_p;
/* heap init */
static int log_heap_init(void)
{
  /* full of zero */
	memset(&log,0,sizeof(log));
	/* shell base */
	log.shell_i = shell_sched_getfiles();
	/* driver config */
	log.config = log_default_config;
	/* file interface */
	log.flip.f_inode = &log;
	log.flip.f_path = "log.o";
	/* file interface */
	log.ops.open  = log_fopen;
	log.ops.write = log_fwrite;
	/* heap */
	
	/* add your own code here */
  log.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* log_default_config log init */
static int log_default_config(void)
{
	/* open usart 3 */
	usart3_p = open("/UART/3",__FS_OPEN_ALWAYS);
	/* config */
	if( usart3_p == NULL )
	{
		/* can not find some uart */
		return FS_ERR;
	}
	/* config msg */
	uart_config_msg msg;
	/* init param */
	msg.index = 3;
	msg.baudrate = 460800;
	msg.tx_mode = _UART_TX_DMA;
	msg.rx_mode = _UART_RX_DISABLE;
	/* init */
	fs_ioctl(usart3_p,0,sizeof(msg),&msg);
	/* end of file */
	return FS_OK;
}
/* file & driver 's interface */
static struct file * log_fopen (FAR struct file *filp)
{
	/* return flip data */
	return &log.flip;
}
/* file write */
static int log_fwrite(FAR struct file *filp, FAR const void * buffer, unsigned int buflen)
{
	/* ignore the compiler warning */
	(void)filp;
	/* send usart data */
  fs_write(usart3_p,buffer,buflen);
	/* return */
	return FS_OK;
}
/* end of data */













