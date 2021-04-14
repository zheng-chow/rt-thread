#ifndef __APP_ABS_ENCODER_H__
#define __APP_ABS_ENCODER_H__
#include <rtdef.h>
rt_bool_t get_angle_from_encoder(float* angle);
rt_bool_t get_speed_from_encoder(float* speed, rt_int16_t* encoder);
#endif
