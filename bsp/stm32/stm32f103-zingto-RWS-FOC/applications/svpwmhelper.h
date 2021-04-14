#ifndef __SVPWM_HELPER_H__
#define __SVPWM_HELPER_H__
#include <rtdef.h>
#if 0
float Svpwm_PhaseA(rt_uint16_t elec_ratio, float elec_angle);
float Svpwm_PhaseB(rt_uint16_t elec_ratio, float elec_angle);
float Svpwm_PhaseC(rt_uint16_t elec_ratio, float elec_angle);
#else
typedef struct{
	float a;
	float b;
	float c;
}SvpwmResult_t;
float Svpwm_PhaseA(float elec_ratio, float elec_angle);
float Svpwm_PhaseB(float elec_ratio, float elec_angle);
float Svpwm_PhaseC(float elec_ratio, float elec_angle);
void Svpwm_PhaseABC(float elec_ratio, float elec_angle, SvpwmResult_t* res);



#endif
#endif

