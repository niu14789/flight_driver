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

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "usart.h"
#include "f_shell.h"
#include "fs_config.h"
#include "string.h"
#include "gd32f30x.h"
/* fs inode system register */
FS_INODE_REGISTER("/UART/",usart,usart_heap_init,0);
/* static usart file dev */
static struct file filp_u[USART_NUM];
static unsigned short dma_pos[USART_NUM];/* dma detector position */
/* usart code */
const unsigned int usart_msg[USART_NUM][9] = {
{ USART0,GPIOA,GPIO_PIN_9/* tx pin */,GPIOA,GPIO_PIN_10/* rx pin */,
  DMA0,DMA_CH3/* TX DMA */,DMA0,DMA_CH4/* RX DMA */},/* USART0 config message */
{ USART1,GPIOA,GPIO_PIN_2/* tx pin */,GPIOA,GPIO_PIN_3/* rx pin */,
  DMA0,DMA_CH6/* TX DMA */,DMA0,DMA_CH5/* RX DMA */},/* USART1 config message */
{ USART2,GPIOB,GPIO_PIN_10/* tx pin */,GPIOB,GPIO_PIN_11/* rx pin */,
  DMA0,DMA_CH1/* TX DMA */,DMA0,DMA_CH2/* RX DMA */},/* USART1 config message */
{ UART3,GPIOC,GPIO_PIN_10/* tx pin */,GPIOC,GPIO_PIN_11/* rx pin */,
  DMA1,DMA_CH4/* TX DMA */,DMA1,DMA_CH2/* RX DMA */},/* USART1 config message */
};
/* heap init */
static int usart_heap_init(void)
{
  /* full of zero */
	memset(&usart,0,sizeof(usart));
	/* shell base */
	usart.shell_i = shell_sched_getfiles();
	/* driver config */
	usart.config = usart_default_config;
	/* file interface */
	usart.ops.open  = usart_fopen;
	usart.ops.write = usart_fwrite;
	usart.ops.read  = usart_fread;
	usart.ops.ioctl = usart_ioctrl;
	/* file interface */
	usart.flip.f_inode = &usart;
	usart.flip.f_path = "/UART/";
	/* heap */
	
	/* add your own code here */
  usart.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;	
}
/* usart_default_config */
static int usart_default_config(void)
{
	/* return */
	return FS_OK;
}
/* usart dev open and init */
struct file * usart_fopen(FAR struct file *filp)
{
   int length;
	 char index;
	 /* get length of path */
   length = strlen(	filp->f_path);
	 /* is length more than 7? */
	 if(length == 0x7)
	 {
		 /* get index */
		 index = filp->f_path[6] - '0';
		 /* check index */
		 if( index < USART_NUM)
		 {
			 /* open always */
			 filp_u[index].f_oflags = __FS_IS_INODE_OK;
			 filp_u[index].f_inode = &usart;
			 filp_u[index].f_multi = index;
			 filp_u[index].f_path = "/UART/0-7";
			 /* allow open */
			 return &filp_u[index];//open succeed	
		 }
		 else
		 {
			 	/* can not find this device*/
		    return NULL;//open fail
		 }
	 }
	 else
	 {
		 /* can not supply this format */
		 return NULL;//open fail
	 }	
}
/* file interface write */
static int usart_fwrite(FAR struct file *filp, FAR const void *buffer, unsigned int buflen)
{
	return usart_dma_write(filp->f_multi, filp->f_user1 >> 16 ,buffer,buflen);
}
/* read data */
unsigned int usart_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen)
{
	return usart_dma_read(filp->f_multi,filp->f_user0,filp->f_user1 & 0xffff,buffer,buflen);
}
/* ioctrl */
/* usart ioctrl */
static int usart_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		/* select a cmd */
		case 0:
			/* get the lengtch */
			if(sizeof(uart_config_msg) != arg )
			{
				return FS_ERR;// not supply this format
			}			
			/* config usart ang uart */
		  ret = usart_init_global(pri_data);
      /* end of function */		
		  break;		
		  /* end of data */
		case 1:
			/* change baudrate */
		  usart_baudrate_set(usart_msg[filp->f_multi][0],arg);
      /* end of data */
		  break;
		  /* end of data */
		default:
			break;
	}
	/* return */
	return ret;
}
/* static usart init global */
static int usart_init_global( const uart_config_msg * msg )
{
	/* check param */
	if( msg->index >= USART_NUM )
	{
		return FS_ERR;/* bad index */
	}
	/* save dma buffer and length */
	filp_u[msg->index].f_user0 = msg->rx_dma_buffer;
	filp_u[msg->index].f_user1 = msg->rx_dma_deepth & 0xffff;
	/* save dma mode or normal mode */
	filp_u[msg->index].f_user1 |= ( msg->tx_mode << 24 ) | ( msg->rx_mode << 16 );
	/* enable all gpio clock */
	rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
  rcu_periph_clock_enable(RCU_GPIOD);	
	/* enable all of usart ' s clock */
	rcu_periph_clock_enable(RCU_USART0);
  rcu_periph_clock_enable(RCU_USART1);
	rcu_periph_clock_enable(RCU_USART2);
	rcu_periph_clock_enable(RCU_UART3);
	rcu_periph_clock_enable(RCU_UART4);
	
	/* connect port to USARTx_Tx */
	gpio_init(usart_msg[msg->index][1], GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, usart_msg[msg->index][2]);

	/* connect port to USARTx_Rx */
	gpio_init(usart_msg[msg->index][3], GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, usart_msg[msg->index][4]);

	/* UART configure */
	/* reset uart/usart */
	usart_deinit(usart_msg[msg->index][0]); 
  /* configration */
	usart_baudrate_set(usart_msg[msg->index][0], msg->baudrate);
	usart_word_length_set(usart_msg[msg->index][0], USART_WL_8BIT);
	usart_parity_config(usart_msg[msg->index][0], USART_PM_NONE);
	usart_stop_bit_set(usart_msg[msg->index][0], USART_STB_1BIT);
  /* usart mode settings */
	usart_receive_config(usart_msg[msg->index][0], USART_RECEIVE_ENABLE);     // enable receive
	usart_transmit_config(usart_msg[msg->index][0], USART_TRANSMIT_ENABLE);   // enable send 	
	/* enable usart 0 */
	usart_enable(usart_msg[msg->index][0]);
	
  /* dma config blove */
  dma_parameter_struct DmaInitParam;
  /* enable clock */
  rcu_periph_clock_enable(RCU_DMA0);
	rcu_periph_clock_enable(RCU_DMA1);
	/* DMA TX MODE config . param default */
	dma_deinit(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6]);
	/* init param */
	DmaInitParam.direction = DMA_MEMORY_TO_PERIPHERAL;
	DmaInitParam.memory_addr = 0;
	DmaInitParam.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	DmaInitParam.memory_width = DMA_MEMORY_WIDTH_8BIT;
	DmaInitParam.number = 0;
	DmaInitParam.periph_addr = usart_msg[msg->index][0] + 4;
	DmaInitParam.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	DmaInitParam.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	DmaInitParam.priority = DMA_PRIORITY_ULTRA_HIGH;	
	/* dma mode or not */
	if( msg->tx_mode == _UART_TX_DMA )
	{
		/* init */
		dma_init(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6], &DmaInitParam);
		/* configure DMA mode */
		dma_circulation_disable(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6]);
		dma_memory_to_memory_disable(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6]);
		/* USART DMA enable for transmission */
		usart_dma_transmit_config(usart_msg[msg->index][0], USART_DENT_ENABLE);
		/* enable DMA channel1 */
		dma_channel_disable(usart_msg[msg->index][5], (dma_channel_enum)usart_msg[msg->index][6]);
  }
	/* dma mode or not */
	if( msg->rx_mode == _UART_RX_DMA )	
	{
		/* DMA RX MODE CONFIG */
		dma_deinit(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8]);
		/* set param */
		DmaInitParam.direction = DMA_PERIPHERAL_TO_MEMORY;
		/* set receive buffer */
		DmaInitParam.memory_addr = msg->rx_dma_buffer;
		DmaInitParam.number = msg->rx_dma_deepth;
		/* others */
		DmaInitParam.priority = DMA_PRIORITY_ULTRA_HIGH;
		/* init */	
		dma_init(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8], &DmaInitParam);
		/* configure DMA mode */	
		dma_circulation_enable(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8]);
		dma_memory_to_memory_disable(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8]);
		/* enable usart0 dma receive mode */
		usart_dma_receive_config(usart_msg[msg->index][0], USART_DENR_ENABLE);
		/* enable dma */
		dma_channel_enable(usart_msg[msg->index][7], (dma_channel_enum)usart_msg[msg->index][8]);		
	}		
	/* ok . we've finished . now return */
	return FS_OK;
}
/* int usart0 dma tx */
static int usart_dma_write( unsigned int index , unsigned short txrx_m,const void * data , unsigned int len )
{
	/* get tx and rx mode */
	if( txrx_m >> 8 == _UART_TX_DMA )
	{
		/* disable first */
		dma_channel_disable(usart_msg[index][5], (dma_channel_enum)usart_msg[index][6]);
		/* reset dma */
		DMA_CHMADDR(usart_msg[index][5], (dma_channel_enum)usart_msg[index][6]) = (unsigned int)data;
		DMA_CHCNT(usart_msg[index][5], (dma_channel_enum)usart_msg[index][6])   = ( len & 0xffff );
		/* enable dma again */
		dma_channel_enable(usart_msg[index][5], (dma_channel_enum)usart_msg[index][6]);
	}		
	else
	{
		/* transfer data to pdata */
		unsigned char * pdata = (unsigned char *)data;
		/* write to usart one by one */
		for( int i = 0 ; i < len ; i ++ )
		{
			/* first wait until the usart is idle */
			while( RESET == usart_flag_get( usart_msg[index][0] , USART_FLAG_TC ) )
			{ /* nothing need to do */ }
      /* send data */
		  usart_data_transmit(usart_msg[index][0],pdata[i]);
		}
	}
	/* return ok as uaural */
	return FS_OK;
}
/* static usart read vio dma */
static unsigned int usart_dma_read(unsigned int index,unsigned int dma_buf_addr,unsigned int dma_deepth,void * rdata,unsigned int rlen)
{
	/* get and send */
	unsigned int dma_cnt = dma_deepth - dma_transfer_number_get(usart_msg[index][7], (dma_channel_enum)usart_msg[index][8]);
	/* get data */
	if( dma_cnt != dma_pos[index] )
	{
		/* ok ! we got some data */
		unsigned short dma_counter = ( dma_cnt > dma_pos[index] ) ? ( dma_cnt - dma_pos[index] ) : 
			                           ( dma_deepth - dma_pos[index] + dma_cnt );
		/* get read transfer length */
		dma_counter = ( rlen > dma_counter ) ? dma_counter : rlen;
		/* copy data */
		const unsigned char * src = ( const unsigned char *)dma_buf_addr;
		unsigned char * dst = (unsigned char *)rdata;
		/* real copy data */
		for( unsigned int i = 0 ; i < dma_counter ; i ++ )
		{
			dst[i] = src[(dma_pos[index]+i) %  dma_deepth];
		}
		/* reset pos */
		dma_pos[index] = ( dma_pos[index] + dma_counter ) % dma_deepth ;
		/* return data that had read */
		return dma_counter;
	}
	/* got nothing */
	return FS_OK;
}	
/* end of file */










