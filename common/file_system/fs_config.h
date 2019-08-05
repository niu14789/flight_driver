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

#ifndef __FS_CONFIG_H__
#define __FS_CONFIG_H__

/* uart and usart config */

#define _UART_MODE_NARMOL    (0)

#define _UART_PAR_NO         (0)
#define _UART_PAR_EVEN       (1)
#define _UART_PAR_OLD        (2)

#define _UART_BIT_8          (8)

#define _UART_STOP_1         (0)
#define _UART_STOP_1_5       (1)
#define _UART_STOP_2         (2)

#define _UART_TX_DISABLE     (0x0000)
#define _UART_TX_NARMOL      (0x0001)
#define _UART_TX_DMA         (0x0002)

#define _UART_RX_DISABLE     (0x0000)
#define _UART_RX_IT          (0x0001)
#define _UART_RX_DMA         (0x0002)
#define _UART_RX_RXNE        (0x0003)


typedef struct uart_config_msg_s
{
	unsigned int   mode;//0,normal;
	unsigned int   baudrate;
	unsigned int   rx_dma_deepth;
	unsigned short tx_mode;//TX:0x0000 0000 (tx,rx),0x0001:byte send,0x0002:dma  
	unsigned short rx_mode;//Rx:0x0001,IT timeout,0x0002 dma LLI receive 0x0003 byte interrupt without dma
	unsigned int   index;
	unsigned int   rx_dma_buffer;//rx buffer that 
}uart_config_msg;
/* end uart */

/* pwm config */
#define DEFAULT_DIV  (2)
/* struct */
typedef struct pwm_config_msg_s
{
	unsigned int         div;
	unsigned short    period;
	unsigned short dutycycle;
}pwm_config_msg;

/* spi config */
typedef struct spi_config_msg_s
{
	unsigned int freq;
}spi_config_msg;

/* task config */
typedef struct task_config_msg_s
{
	unsigned int period;
	unsigned int delay;
	unsigned int time;
	unsigned int enable;
}task_config_msg;

#endif























