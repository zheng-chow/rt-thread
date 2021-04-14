#include <rtthread.h>
#include <math.h>
#include "app_pid.h"

void pid_init(PID_t* pid, float Kp, float Ki, float Kd)
{
	rt_memset(pid,0,sizeof(PID_t));
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;    
}
void pid_change_param(PID_t* pid, float Kp, float Ki, float Kd){
    rt_memset(pid,0,sizeof(PID_t));
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;    
}

void pid_threshold(PID_t* pid, float accThrd, float restThrd, float errThrd, float maxErrThrd)
{
	pid->accErrThrd = accThrd;
	pid->resThrd = restThrd;
	pid->errThrd = errThrd;
	pid->maxErrThrd = maxErrThrd;
}
void pid_reset(PID_t* pid, float accErr){
	 pid->accErr = accErr;
}
void pid_set(PID_t* pid, float accErr){
	if (((accErr > 0) && (pid->accErr < accErr))
		|| ((accErr < 0) && (pid->accErr > accErr))){
		pid->accErr = accErr;
		if (pid->accErrThrd != 0) {//积分限幅
			if(pid->accErr > pid->accErrThrd)///scale
				pid->accErr = pid->accErrThrd;///scale
			else if (pid->accErr < -pid->accErrThrd)///scale
				pid->accErr = -pid->accErrThrd;///scale
		}
	}
}

void pid_enable(PID_t* pid, rt_bool_t bEnable){
	pid->bEnable = bEnable;
}
void pid_enable_assis(PID_t* pid, rt_bool_t bEnable){
		pid->bEnableAssis = bEnable;
}
void pid_auto_clear_acc(PID_t* pid, rt_bool_t bEnable){
	pid->bAutoClearAcc = bEnable;
}

void pid_update(PID_t* pid)
{
	if (pid->bEnable){
//		float scale = 1.f;
		float err = pid->targValue - pid->measValue;
		float abserr = fabsf(err);
		if (pid->errThrd>0){
			if (pid->bStable){
				if(abserr < (pid->errThrd * 5)){
					err = 0;
					if(pid->bAutoClearAcc){
						pid->accErr *= 0.8f;
					}
				}
				else
					pid->bStable = RT_FALSE;
			}
			else{
				if(abserr < pid->errThrd){
					err = 0;
					pid->bStable = RT_TRUE;
					if(pid->bAutoClearAcc){
						pid->accErr *= 0.8f;
					}
				}
			}
			
		}
//		if(abserr < pid->errThrd){
//			 err = 0;
//			if(pid->bAutoClearAcc){
//				pid->accErr *= 0.8f;
//			}
//		}
		
		pid->accErr += err;
		if (pid->accErrThrd != 0) {//积分限幅
			if(pid->accErr > pid->accErrThrd)///scale
				pid->accErr = pid->accErrThrd;///scale
			else if (pid->accErr < -pid->accErrThrd)///scale
				pid->accErr = -pid->accErrThrd;///scale
		}
		pid->resValue = pid->Kp * err + pid->Ki * pid->accErr + pid->Kd * (err - pid->lastErr);
		pid->lastErr = err;
//		pid->resValue *= scale;
		if (pid->resThrd != 0) {//输出限幅
			if(pid->resValue > pid->resThrd)
				pid->resValue = pid->resThrd;
			else if (pid->resValue < -pid->resThrd)
				pid->resValue = -pid->resThrd;
		}	
	}
}
void pid_update1(PID_t* pid)
{
	if (pid->bEnable){
		float scale = 1.f;
		float err = pid->targValue - pid->measValue;
		 if(fabs(err) < 0.05f)
			 err = 0;
		 //+测试
		 if(fabs(err) < 5)
			 scale = 1.3f;
		 //-测试
		pid->accErr += err;
		if (pid->accErrThrd != 0) {//积分限幅
			if(pid->accErr > pid->accErrThrd/scale)
				pid->accErr = pid->accErrThrd/scale;
			else if (pid->accErr < -pid->accErrThrd/scale)
				pid->accErr = -pid->accErrThrd/scale;
		}
		pid->resValue = pid->Kp * err + pid->Ki * pid->accErr + pid->Kd * (err - pid->lastErr);
		pid->lastErr = err;
		pid->resValue *= scale;
		if (pid->resThrd != 0) {//输出限幅
			if(pid->resValue > pid->resThrd)
				pid->resValue = pid->resThrd;
			else if (pid->resValue < -pid->resThrd)
				pid->resValue = -pid->resThrd;
		}	
	}
}
void pid_update3(PID_t* pid)
{
	if (pid->bEnable){
		float err = pid->targValue - pid->measValue;
		if(fabs(err) < pid->errThrd)
			 err = 0;		
		else if ((err > pid->maxErrThrd) && (0 != pid->maxErrThrd))
			err = pid->maxErrThrd;
		else if (err < -pid->maxErrThrd && (0 != pid->maxErrThrd))
			err = -pid->maxErrThrd;				
		
		pid->accErr += err;
		pid->resValue = pid->Kp * err;
		if (pid->resThrd != 0){
			//存在输出限幅
			if ((pid->resValue >= pid->resThrd) || (pid->resValue <= -pid->resThrd))
				pid->accErr = 0;	
		}		 
		if (pid->accErrThrd != 0) {//积分限幅
			if(pid->accErr > pid->accErrThrd)
				pid->accErr = pid->accErrThrd;
			else if (pid->accErr < -pid->accErrThrd)
				pid->accErr = -pid->accErrThrd;
		}		
		pid->resValue += pid->Ki * pid->accErr + pid->Kd * (err - pid->lastErr);
		pid->lastErr = err;
		if (pid->resThrd != 0) {//输出限幅
			if(pid->resValue > pid->resThrd)
				pid->resValue = pid->resThrd;
			else if (pid->resValue < -pid->resThrd)
				pid->resValue = -pid->resThrd;
		}	
	}
}
void pid_update3_bak(PID_t* pid)
{
	if (pid->bEnable){
        pid->resLastValue = pid->resValue;
		float err = pid->targValue - pid->measValue;
		 if(fabs(err) < 0.05f)
			 err = 0;		 
		pid->accErr += err;
		pid->resValue = pid->Kp * err;
		if (pid->resThrd != 0){
			//存在输出限幅
			if ((pid->resValue >= pid->resThrd) || (pid->resValue <= -pid->resThrd))
				pid->accErr = 0;	
		}		 
		if (pid->accErrThrd != 0) {//积分限幅
			if(pid->accErr > pid->accErrThrd)
				pid->accErr = pid->accErrThrd;
			else if (pid->accErr < -pid->accErrThrd)
				pid->accErr = -pid->accErrThrd;
		}		
		pid->resValue += pid->Ki * pid->accErr + pid->Kd * (err - pid->lastErr);
		pid->lastErr = err;
		if (pid->resThrd != 0) {//输出限幅
			if(pid->resValue > pid->resThrd)
				pid->resValue = pid->resThrd;
			else if (pid->resValue < -pid->resThrd)
				pid->resValue = -pid->resThrd;
		}	
	}
}
void pid_update2(PID_t* pid)
{
	if (pid->bEnable){
		float err = pid->targValue - pid->measValue;
		pid->resValue = pid->Kp * (err - pid->lastErr) + pid->Ki * err + pid->Kd * (err - 2* pid->lastErr + pid->lastLastErr);
		pid->lastLastErr = pid->lastErr;
		pid->lastErr = err;
		
		if (pid->resThrd != 0) {//输出限幅
			if(pid->resValue > pid->resThrd)
				pid->resValue = pid->resThrd;
			else if (pid->resValue < -pid->resThrd)
				pid->resValue = -pid->resThrd;
		}
	}
}



void pidInit(PidParameter_t* pid, float Kp, float Ki, float Kd){
    rt_memset(pid,0,sizeof(PID_t));
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd; 
    pid->filter = 0.008f;
}
void pidSetThreshold(PidParameter_t* pid, float accThrd//积分限幅
        , float resErrThrd     // 输出增量限值
        , float errThrd){   //误差限值
    if ((accThrd != 0) && (pid->Ki != 0)){   
        pid->accErrThrd = accThrd / pid->Ki;
    }
    else pid->accErrThrd = 0;
    pid->resErrThrd = resErrThrd;
    pid->errThrd = errThrd;
    if (pid->accErrThrd < 0) pid->accErrThrd = -pid->accErrThrd;
    if (pid->resErrThrd < 0) pid->resErrThrd = -pid->resErrThrd;
    if (pid->errThrd < 0) pid->errThrd = -pid->errThrd;
}
void pidSetEnable(PidParameter_t* pid, rt_bool_t enable){
    pid->bEnable = enable;
}   
void pidReset(PidParameter_t* pid){
    pid->accErr = 0;
    pid->lastErr = 0;
    pid->dLastErr = 0;
    pid->dLastLastErr = 0;
    pid->resLastValue = 0;
}

static float pidLimit(float value, float thrd){
    if (thrd <= 0) return value;
    if (value > thrd) value = thrd;
    else if (value < -thrd) value = -thrd;
    return value;
}

  
static void pidDoUpdate(PidParameter_t* pid){
    float dAvr = 0;
    float dErr = 0;
    float err =  pidLimit(pid->targValue - pid->measValue, pid->errThrd);
    pid->accErr = pidLimit(pid->accErr + err * pid->deltaT, pid->accErrThrd);
    dErr = (err - pid->lastErr)/pid->deltaT;
    dErr = pid->dLastErr + pid->deltaT / (pid->filter + pid->deltaT) * (dErr - pid->dLastErr);
	dAvr = (dErr + pid->dLastErr + pid->dLastLastErr) * 0.333333f;
    pid->lastErr = err;
    pid->dLastLastErr = pid->dLastErr;
    pid->lastErr = dErr;
    pid->resValue = pid->Kp * err + pid->Ki * pid->accErr + pid->Kd * dAvr;
    float resErr = pidLimit(pid->resValue - pid->resLastValue, pid->resErrThrd);
    pid->resValue = pid->resLastValue + resErr;
    pid->resLastValue = pid->resValue;  
}
float pidUpdate(PidParameter_t* pid, float target, float measure, float dT){
    if (!pid->bEnable) return 0;
    pid->targValue = target;
    pid->measValue = measure;
    pid->deltaT = dT;
    pidDoUpdate(pid);
    return pid->resValue;    
}

