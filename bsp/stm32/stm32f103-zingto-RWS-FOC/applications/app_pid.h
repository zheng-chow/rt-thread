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

	float maxErrThrd;
	float accErrThrd;
	float resThrd;
	float errThrd;
	float resValue;
	float resLastValue;
    
	rt_bool_t bEnable;
	rt_bool_t	bAutoClearAcc;
	rt_bool_t bStable;
}PID_t;

void pid_init(PID_t* pid, float Kp, float Ki, float Kd);
void pid_change_param(PID_t* pid, float Kp, float Ki, float Kd);
void pid_threshold(PID_t* pid, float accThrd, float restThrd, float errThrd, float maxErrThrd);
void pid_update(PID_t* pid);
void pid_update1(PID_t* pid);
void pid_update2(PID_t* pid);
void pid_update3(PID_t* pid);
void pid_reset(PID_t* pid, float accErr);
void pid_set(PID_t* pid, float accErr);
void pid_enable(PID_t* pid, rt_bool_t bEnable);
void pid_enable_assis(PID_t* pid, rt_bool_t bEnable);
void pid_auto_clear_acc(PID_t* pid, rt_bool_t bEnable);


typedef struct PidParameter{
	float Kp;//PID-P
	float Ki;//PID-I
	float Kd;//PID-D
    
    
    float accErr;//积分误差 sum(err*deltaT)
    float accErrThrd;//积分限幅(限幅为0不积分)
    
    float filter;//滤波参数
	float lastErr;//上次误差
    float dLastErr;//上次误差差
    float dLastLastErr;//上上次误差差
    
    float targValue;//目标值
	float measValue;//测量值
    float resValue;//输出结果
    float resLastValue;//上次输出结果
    float deltaT;//PID-ADJUST-DeltaT
	

	float resErrThrd;//输出增量上限
	float errThrd;//最大误差限值
    
	rt_bool_t bEnable;
}PidParameter_t;

void pidInit(PidParameter_t* pid, float Kp, float Ki, float Kd);
void pidSetThreshold(PidParameter_t* pid, float accThrd//积分限幅
        , float resErrThrd     // 输出增量限值
        , float errThrd);   //误差限值
void pidSetEnable(PidParameter_t* pid, rt_bool_t enable);
void pidReset(PidParameter_t* pid);
float pidUpdate(PidParameter_t* pid, float target, float measure, float dT);
        
#endif
