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
#include "gd32f30x.h"
#include "runtime.h"
#include "string.h"
#include "fs_config.h"
/* declares */
int runtime_dev_ioctl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);
int system_shell_init(struct shell_cmd * p_shell_cmd,unsigned int max);
/* task init */
const isr_handler_def __FS_t0[4] = { 0xADCF3568 ,45,task2_handler, 0x887F651D };
const isr_handler_def __FS_t1[4] = { 0xADCF3568 ,46,task3_handler, 0x887F651D };
const isr_handler_def __FS_t2[4] = { 0xADCF3568 ,66,task4_handler, 0x887F651D };
const isr_handler_def __FS_t3[4] = { 0xADCF3568 ,70,task5_handler, 0x887F651D };
const isr_handler_def __FS_t4[4] = { 0xADCF3568 ,71,task6_handler, 0x887F651D };
/* fs inode system register */
FS_INODE_REGISTER("/runtime.o",runtime,task_heap_init,202);
/* fs shell register for start system */
FS_SHELL_REGISTER(system_start);
/* disable all it */
FS_SHELL_REGISTER(disable_all_it);
/* task control */
static unsigned int task_ctrl = 0;
/* heap init */
int task_heap_init(void)
{
	 /* full of zero */
	memset(&runtime,0,sizeof(runtime));
	/* fs runs init */
	fs_runs_init();
	/* shell base */
	runtime.shell_i = shell_sched_getfiles();
	/* driver config */
	runtime.config = task_default_config;
	runtime.ops.ioctl = runtime_dev_ioctl;
	/* file interface */
	runtime.flip.f_inode = &runtime;
	runtime.flip.f_path = "/runtime.o";
	/* shell init */
	FS_SHELL_INIT(system_start,system_start,4,_CB_EXE_);
	FS_SHELL_INIT(disable_all_it,disable_all_it,4,_CB_EXE_);
	/* heap */
	task_ctrl = 0;
	/* add your own code here */
  runtime.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT;
	/* ------end of file------ */
	return FS_OK;
}
///* task default config */
int task_default_config(void)
{
	/* psr and period */
	unsigned short psr[5] = {119,119,119,119,1199};/* 0.5ms 4ms 10ms 20ms 100ms */
	unsigned short ped[5] = {499,3999,9999,19999,9999};	
	unsigned int  timer_base[5] = { TIMER2 , TIMER3 , TIMER4 ,TIMER5 ,TIMER6 };
	/* timer init */
	timer_parameter_struct timer_initpara;
  /* enable timer's clock */
	rcu_periph_clock_enable(RCU_TIMER2);
  rcu_periph_clock_enable(RCU_TIMER3);
	rcu_periph_clock_enable(RCU_TIMER4);
  rcu_periph_clock_enable(RCU_TIMER5);
	rcu_periph_clock_enable(RCU_TIMER6);

	/* TIMERS configuration */
	timer_initpara.prescaler         = 0;
	timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.period            = 0;
	timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;
	/* config */
	for( unsigned int i = 0 ; i < 5 ; i ++ )
	{
		/* default timers settings */
		timer_deinit(timer_base[i]);		
		/* set psr */
		timer_initpara.prescaler = psr[i];
		/* set period */
		timer_initpara.period    = ped[i];
		/* init timers */
		timer_init(timer_base[i],&timer_initpara);
		/* it init */
		timer_interrupt_enable(timer_base[i], TIMER_INT_UP);	
		/* auto-reload preload enable */
		timer_auto_reload_shadow_enable(timer_base[i]);		
		/* disable timer first */
		/* auto-reload preload enable */
		timer_enable(timer_base[i]);			
	}
  /* return */
	return FS_OK;
}
/* start system */
int system_start(int argc)
{
	/* table for IRQn CMSIS */
	unsigned char irqn[5] = { TIMER2_IRQn , TIMER3_IRQn , 
		                         TIMER4_IRQn , TIMER5_IRQn , TIMER6_IRQn};
	/* typedef */
	unsigned short argc_wide = argc & 0x7fff;
	/*-----------*/
	if( argc & 0x8000 )
	{
		if( (argc&0x7f) >= 10 )
		{
			 return FS_ERR;//not supply this format
		}
    /*----------------*/
    if( argc & 0x80 )
		{
			/* enable */
			NVIC_EnableIRQ((IRQn_Type)irqn[ argc & 0x7f ]);
		}else
    {
			/* disable */
			NVIC_DisableIRQ((IRQn_Type)irqn[ argc & 0x7f ]);
		}			
		/* return for */
		return FS_OK;
	}
	/* each day pl */
	for( int i = 0 ; i < 5 ; i ++ )
  { 
		 if( argc_wide & ( 1 << i ) )
		 {
			  /* enable all init */
				nvic_irq_enable(irqn[i], i, 0);
			  /* enable the irq */
			  NVIC_EnableIRQ((IRQn_Type)irqn[i]);
		 }
		 else
		 {
        NVIC_DisableIRQ((IRQn_Type)irqn[i]);
		 }
  }	
	/* return */
	return FS_OK;
}
/* disable all it */
int disable_all_it(int argc)
{
	for( int i = 0 ; i < 60 ; i ++ )
	{
		NVIC_DisableIRQ((IRQn_Type)i);
	}
	/* return */
	return FS_OK;
}
/* ints */
void task2_handler(void)
{
	/* clear flag */
  timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
	/* about a ms */
	if( task_ctrl ++ % 2 )
	{
		shell_timer_thread(0);
	}
	/* run 0.5ms task */
	shell_timer_thread(6);
	/* end of data */
}
void task3_handler(void)
{
	/* clear flag */
  timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);
	/* about a ms */
	shell_timer_thread(1);
	/* end of data */
}
void task4_handler(void)
{
	/* clear flag */
  timer_interrupt_flag_clear(TIMER4, TIMER_INT_FLAG_UP);
	/* about a ms */
	shell_timer_thread(2);
  /* end of data */
}
void task5_handler(void)
{
	/* clear flag */
  timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
	/* about a ms */
	shell_timer_thread(3);
	/* end of data */
}
void task6_handler(void)
{
	/* clear flag */
  timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);
	/* about a ms */
	shell_timer_thread(4);
	/* end of data */
}


























