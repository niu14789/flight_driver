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
#include "string.h"
#include "gd32f30x.h"
/* functions declare */
static int led_heap_init(void);
static int led_default_config(void);
static void led_thread(void);
static struct file * led_fopen (FAR struct file *filp);
static int led_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);
/* fs inode system register */
FS_INODE_REGISTER("led.o",led,led_heap_init,0);
/* as default , define a led task in 100ms */
FS_SHELL_STATIC(led_thread,led_thread,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD3_ID));
/* led hardware default */
const unsigned int leds_hd[4][3] = 
{
   { RCU_GPIOB , GPIOB , GPIO_PIN_14 },
   { RCU_GPIOB , GPIOB , GPIO_PIN_15 },
   { RCU_GPIOC , GPIOC , GPIO_PIN_1  },
   { RCU_GPIOC , GPIOC , GPIO_PIN_0  }
};
/* heap init */
static int led_heap_init(void)
{
  /* full of zero */
	memset(&led,0,sizeof(led));
	/* shell base */
	led.shell_i = shell_sched_getfiles();
	/* driver config */
	led.config = led_default_config;
	/* file interface */
	led.flip.f_inode = &led;
	led.flip.f_path = "led.o";
	/* file interface */
	led.ops.open  = led_fopen;
	led.ops.ioctl = led_ioctrl;
	/* heap */
	
	/* add your own code here */
  led.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* led_default_config led init */
static int led_default_config(void)
{
	/* config GPIOS */
	for( unsigned int i = 0 ; i < sizeof(leds_hd) / sizeof(leds_hd[0]) ; i ++ )
	{
		/* enable gpio 's clock */
		rcu_periph_clock_enable((rcu_periph_enum)leds_hd[i][0]);
		/* configration gpios */
		gpio_init(leds_hd[i][1], GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, leds_hd[i][2]);
		/* end of file */
	}
	/* return a ok */
	return FS_OK;
}
/* file interface */
/* file & driver 's interface */
static struct file * led_fopen (FAR struct file *filp)
{
	return &led.flip;
}
/* ioctrl for control led mode */
static int led_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
/* nothing diffrent */
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
		  /* write */
		  
		  /* end of cmd */
			break;	
		case 1:
			/* set led mode */
		  break;
		default:
			break;
	}
	/* return */
	return ret;
}
/* for a task test */
static void led_thread(void)
{
	static unsigned int led_ctrl = 0;
	
	if( (led_ctrl++ % 4) < 2 )
	{
		gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);
		gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
		gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
	}
	else
	{
		gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
		gpio_bit_reset(leds_hd[2][1], leds_hd[2][2]);
		gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);
	}
}

#if 0
led_function_s m_led_func;



/* 设置某个LED的状态 */
static void led_set(led_e led, bool value)
{
    //    if (led >= LED_NUM)
    //        return;    

    if (value)
        gpio_bit_set(leds[led].gpio_periph, leds[led].pin);
    else
        gpio_bit_reset(leds[led].gpio_periph, leds[led].pin);
}

/*关闭所有LED*/
static void led_clear_all(void)
{
    uint8_t i;
    for (i = 0; i < LED_NUM; i++)
    {
        led_set((led_e)i, 0);
    }
}

/*打开所有LED*/
static void led_set_all(void)
{
    uint8_t i;
    for (i = 0; i < LED_NUM; i++)
    {
        led_set((led_e)i, 1);
    }
}

void led_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);
    //LED_L_H
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    //LED_L_T
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    //LED_R_H
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    //LED_R_T
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    led_clear_all();
    set_led_all_off();
}
//设置所有LED为常亮
void set_led_all_on(void) { m_led_func.flicker_function = LED_ALL_ON; }
//设置所有LED为常灭
void set_led_all_off(void) { m_led_func.flicker_function = LED_ALL_OFF; }
//设置LED灯为失控闪烁
void set_led_lose_control_flicker(void) { m_led_func.flicker_function = LED_LOSE_CONTROL_FLICKER; }
//设置LED灯为地磁校准第一步闪烁
void set_led_compass_calibrate_step1_flicker(void) { m_led_func.flicker_function = LED_COMPASS_CALIBRATE_STEP1_FLICKER; }
//设置LED灯为地磁校准第二步闪烁
void set_led_compass_calibrate_step2_flicker(void) { m_led_func.flicker_function = LED_COMPASS_CALIBRATE_STEP2_FLICKER; }
//设置LED灯为水平校准闪烁
void set_led_level_calibrate_flicker(void) { m_led_func.flicker_function = LED_LEVEL_CALIBRATE_FLICKER; }
//设置LED灯为对码闪烁
void set_led_binding_flicker(void) { m_led_func.flicker_function = LED_BINDING_FLICKER; }
//设置LED灯为低电闪烁
void set_led_low_voltage_flicker(void) { m_led_func.flicker_function = LED_LOW_VOLTAGE_FLICKER; }
//设置LED灯为姿态模式闪烁
void set_led_imu_mode_flicker(void) { m_led_func.flicker_function = LED_IMU_MODE_FLICKER; }
//设置LED灯为快速闪烁
void set_led_all_fast_flicker(void) { m_led_func.flicker_function = LED_ALL_FAST_FLICKER; }
//设置LED灯为慢速闪烁
void set_led_all_slow_flicker(void) { m_led_func.flicker_function = LED_ALL_SLOW_FLICKER; }
//设置LED灯为急停闪烁
void set_led_scram_flicker(void) { m_led_func.flicker_function = LED_SCRAM_FLICKER; }

void led_all_on(void)
{
    led_set_all();
}

void led_all_off(void)
{
    led_clear_all();
}

//led  失控闪烁功能 四个脚灯长灭/短亮循环，直到降落到地面 访问频率100hz
void led_lose_control_flicker(void)
{
    if ((m_led_func.timer % 200) == 0)
    {
        led_all_on();
    }
    else if ((m_led_func.timer % 200) == 10)
    {
        led_all_off();
    }
}

//led 闪烁功能 访问频率100hz 
void led_all_flicker(uint8_t by_flicker_freq)
{
    if ((m_led_func.timer % (50 / by_flicker_freq)) == 0)
    {
        if (m_led_func.flicker_on_off_flag == true)
        {
            led_all_on();
            m_led_func.flicker_on_off_flag = false;
        }
        else
        {
            led_all_off();
            m_led_func.flicker_on_off_flag = true;
        }
    }
}

//led 快闪烁功能 访问频率100hz 闪烁频率2Hz
static void led_all_flicker_fast(void)
{
    led_all_flicker(2);
}

//led 快闪烁功能 访问频率100hz 闪烁频率1Hz
static void led_all_flicker_slow(void)
{
    led_all_flicker(1);
}

//led 四灯交替快闪烁功能 访问频率100hz 
static void led_all_flicker_alternately(uint8_t by_flicker_freq)
{
    static uint8_t alternately_step = 0;
    if ((m_led_func.timer % (50 / by_flicker_freq)) == 0)
    {
        alternately_step++;
        if (alternately_step >= LED_NUM)
        {
            alternately_step = 0;
        }
        switch (alternately_step)
        {
        case    3:
            led_all_off();
            led_set(LED_L_H, 1);
            break;
        case    2:
            led_all_off();
            led_set(LED_L_T, 1);
            break;
        case    1:
            led_all_off();
            led_set(LED_R_T, 1);
            break;
        case    0:
            led_all_off();
            led_set(LED_R_H, 1);
            break;
        default:
            break;
        }
    }
}

//地磁校准第一步 灯闪 左侧机头绿灯和机尾红灯慢闪
static void led_compass_calibrate_step1_flicker(void)
{
    led_set(LED_R_H, 0);
    led_set(LED_R_T, 0);
    if ((m_led_func.timer % 50) == 0)
    {
        if (m_led_func.flicker_on_off_flag == true)
        {
            led_set(LED_L_H, 1);
            led_set(LED_L_T, 1);
            m_led_func.flicker_on_off_flag = false;
        }
        else
        {
            led_set(LED_L_H, 0);
            led_set(LED_L_T, 0);
            m_led_func.flicker_on_off_flag = true;
        }
    }
}
//地磁校准第二步 灯闪 右侧机头绿灯和机尾红灯慢闪
static void led_compass_calibrate_step2_flicker(void)
{
    led_set(LED_L_H, 0);
    led_set(LED_L_T, 0);
    if ((m_led_func.timer % 50) == 0)
    {
        if (m_led_func.flicker_on_off_flag == true)
        {
            led_set(LED_R_H, 1);
            led_set(LED_R_T, 1);
            m_led_func.flicker_on_off_flag = false;
        }
        else
        {
            led_set(LED_R_H, 0);
            led_set(LED_R_T, 0);
            m_led_func.flicker_on_off_flag = true;
        }
    }
}
//姿态模式闪灯 姿态模式：机头绿灯常亮，机尾两个红灯慢闪
static void led_imu_mode_flicker(void)
{
    led_set(LED_L_H, 1);
    led_set(LED_R_H, 1);
    if ((m_led_func.timer % 50) == 0)
    {
        if (m_led_func.flicker_on_off_flag == true)
        {
            led_set(LED_L_T, 1);
            led_set(LED_R_T, 1);
            m_led_func.flicker_on_off_flag = false;
        }
        else
        {
            led_set(LED_L_T, 0);
            led_set(LED_R_T, 0);
            m_led_func.flicker_on_off_flag = true;
        }
    }

}

//led 四灯交替快闪烁功能 访问频率100hz 闪烁频率2Hz
//static void led_all_flicker_alternately_fast(void)
//{        
//    led_all_flicker_alternately(2);
//}

//led 四灯交替慢闪功能 访问频率100hz 闪烁频率1Hz
static void led_all_flicker_alternately_slow(void)
{
    led_all_flicker_alternately(1);
}

//led 闪烁功能 访问频率100hz
void led_flicker(void)
{
    m_led_func.timer++;
    switch (m_led_func.flicker_function)
    {
    case    LED_ALL_ON:
        led_all_on();
        break;
    case    LED_ALL_OFF:
        led_all_off();
        break;
    case    LED_LOSE_CONTROL_FLICKER:
        led_lose_control_flicker();
        break;
    case    LED_COMPASS_CALIBRATE_STEP1_FLICKER:
        led_compass_calibrate_step1_flicker();
        break;
    case    LED_COMPASS_CALIBRATE_STEP2_FLICKER:
        led_compass_calibrate_step2_flicker();
        break;
    case    LED_ALL_SLOW_FLICKER:
    case    LED_LEVEL_CALIBRATE_FLICKER:
        led_all_flicker_slow();
        break;
    case    LED_ALL_FAST_FLICKER:
    case    LED_BINDING_FLICKER:
        led_all_flicker_fast();
        break;
    case    LED_LOW_VOLTAGE_FLICKER:
        led_all_flicker_slow();
        break;
    case    LED_IMU_MODE_FLICKER:
        led_imu_mode_flicker();
        break;
    case    LED_SCRAM_FLICKER:
        led_all_flicker_alternately_slow();
        break;
    default:
        break;
    }
}
#endif





