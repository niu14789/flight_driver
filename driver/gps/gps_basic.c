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
#include "f_ops.h"
#include "string.h"
#include "gps.h"
#include "state.h"
/* file & driver 's interface */
static struct file * gps_fopen (FAR struct file *filp);
/* file interface to read gps data */
static unsigned int gps_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
/* fs inode system register */
FS_INODE_REGISTER("gps.o",gps,gps_heap_init,0);
/* some test define */
struct file * usart0_p,* usart1_p,* usart2_p,* usart3_p;
/* gps define */
static unsigned char gps_dma_d0[256];
/* gps_date_flox*/
gps_data ublox_gps;
/* export this */
FS_SHELL_STATIC(ublox_gps,ublox_gps,sizeof(ublox_gps),_CB_RT_);
/* heap init */
static int gps_heap_init(void)
{
  /* full of zero */
	memset(&gps,0,sizeof(gps));
	/* shell base */
	gps.shell_i = shell_sched_getfiles();
	/* driver config */
	gps.config = gps_default_config;
	/* file interface */
	gps.flip.f_inode = &gps;
	gps.flip.f_path = "gps.o";
	/* file interface */
	gps.ops.open = gps_fopen;
	gps.ops.read = gps_fread;
	/* heap */
	
	/* add your own code here */
  gps.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* gps_default_config gps init */
static int gps_default_config(void)
{
	/* open usart 0 */
	usart0_p = open("/UART/0",__FS_OPEN_ALWAYS);
	/* config */
	if( usart0_p == NULL )
	{
		/* can not find some uart */
		return FS_ERR;
	}
	/* config msg */
	uart_config_msg msg;
	/* init param */
	msg.index = 0;
	msg.rx_dma_buffer = (unsigned int)gps_dma_d0;
	msg.rx_dma_deepth = sizeof(gps_dma_d0);
	msg.baudrate = 9600;
	msg.tx_mode = _UART_TX_NARMOL;
	msg.rx_mode = _UART_RX_DMA;
	/* init */
	fs_ioctl(usart0_p,0,sizeof(msg),&msg);
	/* create a init thread */
	shell_create_dynamic("gps_init_thread",gps_init_thread,0);
	/* end of file */
	return FS_OK;
}
/* cread a read data thread */
static void gps_init_thread(void)
{
	/* define some data  */
  static unsigned char init_cnt = 0;
	/* state of gps as a template data */
	if( gps_configure() )
	{
		/* fail */
		init_cnt++;
		/* -------- */
		if( init_cnt > 3 )
		{
			/* delete init thread */
			shell_delete_dynamic("gps_init_thread",0xff);
			/* end of function */
		}
	}
	else
	{
		/* delete init thread and create a read data thread */
		shell_delete_dynamic("gps_init_thread",0xff);
		/* delete init thread and create a read data thread */
		shell_create_dynamic("gps_task_thread",gps_task_thread,1);		
		/* end of function */
	}
}
/* ublox gps thread */
static void gps_task_thread( void )
{
	/* define the buffer */
	unsigned char buf[ 2 * UBX_UART_MAX_DMA_SIZE ];
  /* define count */
	unsigned int 	count = 0;
  /* memset the buf */
	memset(buf, 0, sizeof(buf));
	/* read uart */
	count = fs_read(usart0_p, (char *)buf, sizeof(buf));
	/* get data? */
	if( count > 0)
	{
		/* pass received bytes to the packet decoder */
		for (int i = 0; i < count; i++)
		{
			gps_parse_char(buf[i]);
		}
	}
}
/* file & driver 's interface */
static struct file * gps_fopen (FAR struct file *filp)
{
	return &gps.flip;
}
/* file interface to read gps data */
static unsigned int gps_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen)
{
	/* ignore the complier warring */
	(void)filp;
	/* read */
	if( buflen != sizeof(GPS_User_t) || buffer == NULL )
	{
		/* can not supply this format */
		return FS_OK;
	}	
	/* transfer to gps data formate */
	GPS_User_t * gps_user = ( GPS_User_t * )buffer;
	/* get data and buffer . valibable ? */
  if( ublox_gps.reserved1 )
	{
		/*
		   GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 
		   2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix
		*/
		gps_user->fixType = ublox_gps.fixType;
		/**&lt; Position DOP [0.01] */
		gps_user->pDOP = ublox_gps.pDOP;
		/**&lt; Number of SVs used in Nav Solution */
		gps_user->numSV = ublox_gps.numSV;
		/**&lt; Longitude [deg] */
		gps_user->lon = ublox_gps.lon;
		/**&lt; Latitude [deg] */
		gps_user->lat = ublox_gps.lat;
		/**&lt; Height above mean sea level [m] */
		gps_user->height = ublox_gps.height;
		/**&lt; NED north velocity [m/s]*/
		gps_user->velN = ublox_gps.velN;
		/**&lt; NED east velocity [m/s]*/
		gps_user->velE = ublox_gps.velE;
		/**&lt; NED up velocity [m/s]*/
		gps_user->velU = -ublox_gps.velD;
		/* GPS time */
		gps_user->position_timestamp = ublox_gps.iTOW;
		/* disale gps data */
		ublox_gps.reserved1 = 0;
		/* look good ! return the leng we've read */
		return buflen;
	}
	/* oh no ! we got nothing */
	return FS_OK;
  /* end of data */	
}






















