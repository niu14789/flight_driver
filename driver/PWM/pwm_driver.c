#include <stdio.h>
#include "gd32f30x.h"
//#include "gd32f307c_eval.h"
#include "string.h"
//#include "bsp.h"
#include "pwm_driver.h"

void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PA0 PA1 PA2(TIMER1 CH0 CH1 CH2) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_1);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_2);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3);
}

void timer_config(void)
{
    /* -----------------------------------------------------------------------
    TIMER1 configuration: generate 4 PWM signals with 4 different duty cycles:
    TIMER1CLK = SystemCoreClock / 120 = 1MHz

    TIMER1 channel0 duty cycle = (4000/ 16000)* 100  = 25%
    TIMER1 channel1 duty cycle = (8000/ 16000)* 100  = 50%
    TIMER1 channel2 duty cycle = (12000/ 16000)* 100 = 75%
    TIMER1 channel3 duty cycle = (12000/ 16000)* 100 = 75%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 59;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 4999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* CH0,CH1,CH2 and CH3 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER1,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_2,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_3,&timer_ocintpara);

    /* CH0 configuration in PWM mode0,duty cycle 40% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,2000);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode0,duty cycle 40% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,2000);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode0,duty cycle 40% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_2,2000);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
    
    /* CH3 configuration in PWM mode0,duty cycle 40% */
    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_3,2000);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* auto-reload preload enable */
    timer_enable(TIMER1);
}

void motor_init(void)
{
    gpio_config();
    timer_config();
}

/*设置电机PWM占空比 电机频率400HZ pulse 范围0 ~ 2000     */
void motors_set_ratio(motor_type_enum motor,uint32_t pulse)
{
    uint32_t radio;
    
    radio = (float)pulse;
    
    if(radio > MOTOR_RADIO_MAX)
    {
        radio = MOTOR_RADIO_MAX;
    }    
    //radio = 2000 + radio;          //2000 表示1000高电平 电平在1000~2000间变化
    radio *= 2;
    
    switch (motor)
    {
        case    MOTOR_M1:
            timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,radio);
            break;
        case    MOTOR_M2:
            timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_3,radio);
            break;
        case    MOTOR_M3:
            timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_2,radio);
            break;            
        case    MOTOR_M4:
            timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,radio);
            break;
        case    SERVO:
            timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_3,radio);
        default:
            break;             
    }
}

void servo_init(void)
{    
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PB9 (TIMER3 CH3) as alternate function*/
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);  

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);

    /* TIMER3 configuration */
    timer_initpara.prescaler         = 59;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 4999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3,&timer_initpara);

    /* CH3 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    
    timer_channel_output_config(TIMER3,TIMER_CH_3,&timer_ocintpara);

    /* CH3 configuration in PWM mode0,duty cycle 40% */
    timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_3,2000);
    timer_channel_output_mode_config(TIMER3,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);
    
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
    /* auto-reload preload enable */
    timer_enable(TIMER3);
}


