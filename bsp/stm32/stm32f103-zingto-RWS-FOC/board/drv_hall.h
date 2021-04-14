#ifndef __DRV_HALL_H__
#define __DRV_HALL_H__
#include <rtdef.h>

#define OFFS_ANGLE (-178)

/*
#if defined(AXIS_PITCH)
#define OFFS_ANGLE (-178 - 120)
#define OFFS_ANGLE (-178)
#elif defined(AXIS_YAW)
#define xOFFS_ANGLE (-178 - 120)
#define OFFS_ANGLE (-178)
#else
#error "unkown offset angle"
#endif 
*/

float hall_fast_get_angle(void);
float hall_fast_get_speed(void);
void hall_fast_set_speed(float speed);

//float hall_fast_clear_speed(void);

#endif

