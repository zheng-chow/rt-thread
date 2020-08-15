#ifndef __APP_ADC_H__
#define __APP_ADC_H__
#include <rtdef.h>
typedef struct {
    uint32_t freq;
    uint16_t cnls;
    uint16_t bytes;
    uint32_t stamp;
    //uint32_t precision;
}adc_timestamp_t;

typedef struct{
    float freq;
    float phase;
    float amp;
    float ref;
    rt_bool_t valid;
}adc_fit_t;

typedef void*  adc_callback_parameter_t;
typedef void   adc_callback_function_t(adc_callback_parameter_t h, rt_uint8_t* adc_data, rt_uint32_t adc_data_len, rt_uint32_t index, adc_timestamp_t timeMask);

rt_bool_t app_create_adc(void);
rt_bool_t app_delete_adc(void);
rt_bool_t app_start_adc(void);
rt_bool_t set_adc_callback_function(adc_callback_parameter_t h, adc_callback_function_t f);
rt_bool_t app_adc_is_exist(void);

#endif
