#ifndef ADC_H
#define ADC_H

#include "stdint.h"
#include "stdbool.h"

void adc_init(void);
//获得电池电压 访问频率100Hz
float get_battery_voltage(void);


#endif


