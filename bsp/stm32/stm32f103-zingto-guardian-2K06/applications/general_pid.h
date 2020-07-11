#ifndef __GENERAL_PID_H__
#define __GENERAL_PID_H__

#include <rtdef.h>

typedef struct __PID_Structure{
	float kP;
	float kI;
	float kD;

	float setpoint;
	float present;

	float errorLast;
	float errorLaster;

	float errorICS;

	float errorICS_TH;
	float outputMAX_TH;
	float errorIgnore_TH;

	rt_bool_t enable;
}PID_t;

void pid_init(PID_t* pid, float kP, float kI, float kD);
void pid_setThreshold(PID_t* pid, float errorICS_TH, float outputMAX_TH, float errorIgnore_TH);
void pid_setICS(PID_t* pid, float errorICS);
void pid_setSetpoint(PID_t* pid, float setpoint);
void pid_enable(PID_t* pid, rt_bool_t flag);
float pid_update(PID_t* pid, float present);

#endif
