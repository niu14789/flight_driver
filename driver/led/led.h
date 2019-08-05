#ifndef LED_H
#define LED_H

#include "stdint.h"
#include "stdbool.h"

#define LED_NUM     4

typedef enum 
{
	LED_L_H = 0, 
	LED_L_T = 1, 
	LED_R_T = 2, 
	LED_R_H = 3, 
} led_e;

typedef enum 
{
    LED_ALL_ON                              = 0,
    LED_ALL_OFF,
	LED_LOSE_CONTROL_FLICKER, 
	LED_COMPASS_CALIBRATE_STEP1_FLICKER, 
	LED_COMPASS_CALIBRATE_STEP2_FLICKER, 
    LED_LEVEL_CALIBRATE_FLICKER,
    LED_BINDING_FLICKER,
    LED_LOW_VOLTAGE_FLICKER,
    LED_IMU_MODE_FLICKER, 
    LED_ALL_FAST_FLICKER,
    LED_ALL_SLOW_FLICKER,
    LED_SCRAM_FLICKER,
} led_function_e;

typedef struct
{
    bool        flicker_on_off_flag;        
    uint8_t     flicker_function;    
    uint32_t    timer;    
}led_function_s;
extern led_function_s m_led_func;

//设置所有LED为常亮
void set_led_all_on(void);
//设置所有LED为常灭
void set_led_all_off(void);
//设置LED灯为失控闪烁
void set_led_lose_control_flicker(void);
//设置LED灯为地磁校准第一步闪烁
void set_led_compass_calibrate_step1_flicker(void);
//设置LED灯为地磁校准第二步闪烁
void set_led_compass_calibrate_step2_flicker(void);
//设置LED灯为水平校准闪烁
void set_led_level_calibrate_flicker(void);
//设置LED灯为对码闪烁
void set_led_binding_flicker(void);
//设置LED灯为低电闪烁
void set_led_low_voltage_flicker(void);
//设置LED灯为姿态模式闪烁
void set_led_imu_mode_flicker(void);
//设置LED灯为快速闪烁
void set_led_all_fast_flicker(void);
//设置LED灯为慢速闪烁
void set_led_all_slow_flicker(void);
//设置LED灯为急停闪烁
void set_led_scram_flicker(void);

//led 初始化
void led_init(void);
//led 闪烁功能 访问频率100hz
void led_flicker(void);



#endif

