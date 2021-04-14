#ifndef __APP_PID_H__
#define __APP_PID_H__
#include <rtdef.h>

typedef struct PID{
	float Kp;
	float Ki;
	float Kd;
	
	float accErr;
	float lastErr;
	float lastLastErr;
	
	float targValue;
	float measValue;
	
	float assisValue;
	rt_bool_t bEnableAssis;

	float accErrThrd;
	float resThrd;
	float errThrd;
	float resValue;
    
	rt_bool_t bEnable;
	rt_bool_t	bAutoClearAcc;
	rt_bool_t bStable;
}PID_t;

void pid_init(PID_t* pid, float Kp, float Ki, float Kd);
void pid_threshold(PID_t* pid, float accThrd, float restThrd, float errThrd);
void pid_update(PID_t* pid);
void pid_update1(PID_t* pid);
void pid_update2(PID_t* pid);
void pid_update3(PID_t* pid);
void pid_reset(PID_t* pid, float accErr);
void pid_set(PID_t* pid, float accErr);
void pid_enable(PID_t* pid, rt_bool_t bEnable);
void pid_enable_assis(PID_t* pid, rt_bool_t bEnable);
void pid_auto_clear_acc(PID_t* pid, rt_bool_t bEnable);

#endif
