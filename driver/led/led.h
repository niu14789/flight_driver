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

//��������LEDΪ����
void set_led_all_on(void);
//��������LEDΪ����
void set_led_all_off(void);
//����LED��Ϊʧ����˸
void set_led_lose_control_flicker(void);
//����LED��Ϊ�ش�У׼��һ����˸
void set_led_compass_calibrate_step1_flicker(void);
//����LED��Ϊ�ش�У׼�ڶ�����˸
void set_led_compass_calibrate_step2_flicker(void);
//����LED��ΪˮƽУ׼��˸
void set_led_level_calibrate_flicker(void);
//����LED��Ϊ������˸
void set_led_binding_flicker(void);
//����LED��Ϊ�͵���˸
void set_led_low_voltage_flicker(void);
//����LED��Ϊ��̬ģʽ��˸
void set_led_imu_mode_flicker(void);
//����LED��Ϊ������˸
void set_led_all_fast_flicker(void);
//����LED��Ϊ������˸
void set_led_all_slow_flicker(void);
//����LED��Ϊ��ͣ��˸
void set_led_scram_flicker(void);

//led ��ʼ��
void led_init(void);
//led ��˸���� ����Ƶ��100hz
void led_flicker(void);



#endif

