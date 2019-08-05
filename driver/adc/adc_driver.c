#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>
#include "adc_driver.h"

#define BOARD_ADC_CHANNEL   ADC_CHANNEL_8
#define ADC_GPIO_PORT_RCU   RCU_GPIOB
#define ADC_GPIO_PORT       GPIOB
#define ADC_GPIO_PIN        GPIO_PIN_0

typedef struct
{
    uint32_t adc_value_sum;
    uint16_t adc_convert_counter;
    float bat_voltage;
}power_s;

power_s m_power;

void adc_init(void)
{
    /* enable GPIOA clock */
    rcu_periph_clock_enable(ADC_GPIO_PORT_RCU);
    /* enable ADC1 clock */
    rcu_periph_clock_enable(RCU_ADC1);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    /* config the GPIO as analog mode */
    gpio_init(ADC_GPIO_PORT, GPIO_MODE_AIN, GPIO_OSPEED_MAX, ADC_GPIO_PIN);
    
    /* ADC continuous function enable */
    adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, ENABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, DISABLE);    
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); 
    /* ADC data alignment config */
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE); 
    /* ADC channel length config */
    adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1);

    /* ADC regular channel config */
    adc_regular_channel_config(ADC1, 0, BOARD_ADC_CHANNEL, ADC_SAMPLETIME_55POINT5);
    adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);
  
    /* ADC resolusion 12B */
    adc_resolution_config(ADC1, ADC_RESOLUTION_12B);  
    
    /* enable ADC interface */
    adc_enable(ADC1);
    DelayMs(2);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);
    //start adc convert
    adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);
    //clear adc flag end of conversion
    adc_flag_clear(ADC1, ADC_FLAG_EOC);
    
    memset(&m_power,0,sizeof(m_power));
}

//获得电池电压 访问频率100Hz
float get_battery_voltage(void)
{
//    uint16_t temp;
    if (SET == adc_flag_get(ADC1, ADC_FLAG_EOC))
    {        
//        temp = ADC_RDATA(ADC1);
        m_power.adc_value_sum += ADC_RDATA(ADC1);
        m_power.adc_convert_counter ++;
        if ((m_power.adc_convert_counter >= 300) || (m_power.bat_voltage == 0))
        {
            m_power.bat_voltage = ((float)m_power.adc_value_sum/(float)m_power.adc_convert_counter)*10.3213f/4096.0f;
            m_power.adc_value_sum = 0;
            m_power.adc_convert_counter = 0;
        }     
        //clear adc flag end of conversion
        adc_flag_clear(ADC1, ADC_FLAG_EOC);
    }    
    return m_power.bat_voltage;
}


