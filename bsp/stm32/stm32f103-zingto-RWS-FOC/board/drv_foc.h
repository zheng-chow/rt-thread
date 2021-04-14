#ifndef __DRV_FOC_H__
#define __DRV_FOC_H__
#include <rtthread.h>
/*
typedef struct _st_pwm_adjust{
    uint32_t cn1;  
    uint32_t cn2;  
    uint32_t cn3;  
}pwm_adjust_t;*/

typedef struct _st_pwm_update{
    float           detAngle;
    float           coefM;
}pwm_update_t;


typedef struct _st_AdcItem{
    uint16_t adc[4];
    uint8_t idx;
}AdcItem_t;


#define PWM3_CONTROL_SET_FREQ   0x01
#define PWM3_CONTROL_GET_PERIOD 0x02
//#define PWM3_CONTROL_ADJUST     0x03
#define PWM3_CONTROL_UPDATE     0x04

float foc_fast_get_power(void);
#endif
