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

void pid_threshold(PID_t* pid, float accThrd, float restThrd, float errThrd)
{
	pid->accErrThrd = accThrd;
	pid->resThrd = restThrd;
	pid->errThrd = errThrd;
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
		if (pid->errThrd > 0){
			if (pid->bStable){
				if(abserr < (pid->errThrd * 5)){
					err = 0;
					if(pid->bAutoClearAcc){
						pid->accErr = 0.f;
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
						pid->accErr = 0.f;
					}
				}
			}
		}
        
		pid->accErr += err;
		if (pid->accErrThrd != 0) {//积分限幅
			if(pid->accErr > pid->accErrThrd)           ///scale
				pid->accErr = pid->accErrThrd;          ///scale
			else if (pid->accErr < -pid->accErrThrd)    ///scale
				pid->accErr = -pid->accErrThrd;         ///scale
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
    if (pid->bEnable != RT_TRUE)
        return ;
    
    float err = pid->targValue - pid->measValue;
    if(fabs(err) < pid->errThrd) {
        err = 0;
        pid->accErr = 0;
    }
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

void pid_update2(PID_t* pid)
{
	if (pid->bEnable){
		float err = pid->targValue - pid->measValue;
	//  if(fabs(err) < 0.1f)
	//		 err = 0;

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
