#ifndef   PWM_DRIVER_H
#define   PWM_DRIVER_H
#include "stdint.h"
#include "stdbool.h"

#define MOTOR_RADIO_MAX     2000.0f

typedef enum
{
    MOTOR_M1    = 0,
    MOTOR_M2    = 1,
    MOTOR_M3    = 2,
    MOTOR_M4    = 3, 
    SERVO       = 4,     
}motor_type_enum;

void motor_init(void);
void servo_init(void);
void motors_set_ratio(motor_type_enum motor,uint32_t pulse);


#endif

