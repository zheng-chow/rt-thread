#ifndef __APP_ABS_ENCODER_H__
#define __APP_ABS_ENCODER_H__
#include <rtdef.h>
float get_speed_from_encoder(void);
rt_bool_t encoder_read(int16_t* encoder);
rt_bool_t encoder_reset(void);
#endif
