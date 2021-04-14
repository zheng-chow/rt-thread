#include "general_pid.h"
#include <rtthread.h>
#include <math.h>

#define PID_DEFAULT_KP          (1.0f)
#define PID_DEFAULT_KI          (0.0f)
#define PID_DEFAULT_KD		    (0.0f)
#define PID_DEFAULT_ERRIGNORETH (0.02f)

void pid_init(PID_t* pid, float kP, float kI, float kD)
{
	rt_memset(pid, 0, sizeof(PID_t));

	if (kP != 0) {
        pid->kP = kP;
        pid->kI = kI;
        pid->kD = kD;
    }
    else {
        pid->kP = PID_DEFAULT_KP;
        pid->kI = PID_DEFAULT_KI;
        pid->kD = PID_DEFAULT_KD;
    }
    
    pid->errorIgnore_TH = PID_DEFAULT_ERRIGNORETH;
    
    return;
}

void pid_setThreshold(PID_t* pid, float errorICS_TH, float outputMAX_TH, float errorIgnore_TH)
{
    pid->errorICS_TH = errorICS_TH;
    pid->errorIgnore_TH = errorIgnore_TH;
    pid->outputMAX_TH = outputMAX_TH;
    
    return;
}

void pid_setICS(PID_t* pid, float errorICS)
{
    pid->errorICS = errorICS;
    
    if (pid->errorICS_TH != 0) {                    // error Integral Calc Sum limiting work.
        if(pid->errorICS > pid->errorICS_TH)
            pid->errorICS = pid->errorICS_TH;
        else if (pid->errorICS < -pid->errorICS_TH)
            pid->errorICS = -pid->errorICS_TH;
    } 
    return;
}

void pid_setSetpoint(PID_t* pid, float setpoint)
{
    pid->setpoint = setpoint;   
    return;
}

void pid_enable(PID_t* pid, rt_bool_t flag)
{
	pid->enable = flag;
    return;
}

float pid_update(PID_t* pid, float present)
{
    if (pid->enable != RT_TRUE)
        return 0.0f;
    
    pid->present = present;
    
    float err = pid->setpoint - pid->present;
    float abserr = fabsf(err);
    
    if(abserr < pid->errorIgnore_TH)
         err = 0;
    
    pid->errorICS += err;
    
    if (pid->errorICS_TH != 0) {                 // error Integral Calc Sum limiting work.
        if(pid->errorICS > pid->errorICS_TH)
            pid->errorICS = pid->errorICS_TH;
        else if (pid->errorICS < -pid->errorICS_TH)
            pid->errorICS = -pid->errorICS_TH;
    }
    
    float output = pid->kP * err + pid->kI * pid->errorICS + pid->kD * (err - pid->errorLast);
    pid->errorLast = err;
    
    if (pid->outputMAX_TH != 0) {               // output limiting work.
        if(output > pid->outputMAX_TH)
            output = pid->outputMAX_TH;
        else if (output < -pid->outputMAX_TH)
            output = -pid->outputMAX_TH;
    }
    
    return output;
}
    
    /*
void pid_update1(PID_t* pid)
{
	if (pid->bEnable){
		float scale = 1.f;
		float err = pid->targValue - pid->measValue;
		 if(fabs(err) < 0.05f)
			 err = 0;
		 //+����
		 if(fabs(err) < 5)
			 scale = 1.3f;
		 //-����
		pid->accErr += err;
		if (pid->accErrThrd != 0) {//�����޷�
			if(pid->accErr > pid->accErrThrd/scale)
				pid->accErr = pid->accErrThrd/scale;
			else if (pid->accErr < -pid->accErrThrd/scale)
				pid->accErr = -pid->accErrThrd/scale;
		}
		pid->resValue = pid->Kp * err + pid->Ki * pid->accErr + pid->Kd * (err - pid->lastErr);
		pid->lastErr = err;
		pid->resValue *= scale;
		if (pid->resThrd != 0) {//����޷�
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
		 if(fabs(err) < 0.05f)
			 err = 0;		 
		pid->accErr += err;
		pid->resValue = pid->Kp * err;
		if (pid->resThrd != 0){
			//��������޷�
			if ((pid->resValue >= pid->resThrd) || (pid->resValue <= -pid->resThrd))
				pid->accErr = 0;	
		}		 
		if (pid->accErrThrd != 0) {//�����޷�
			if(pid->accErr > pid->accErrThrd)
				pid->accErr = pid->accErrThrd;
			else if (pid->accErr < -pid->accErrThrd)
				pid->accErr = -pid->accErrThrd;
		}		
		pid->resValue += pid->Ki * pid->accErr + pid->Kd * (err - pid->lastErr);
		pid->lastErr = err;
		if (pid->resThrd != 0) {//����޷�
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
	//  if(fabs(err) < 0.1f)
	//		 err = 0;

		pid->resValue = pid->Kp * (err - pid->lastErr) + pid->Ki * err + pid->Kd * (err - 2* pid->lastErr + pid->lastLastErr);
		pid->lastLastErr = pid->lastErr;
		pid->lastErr = err;
		
		if (pid->resThrd != 0) {//����޷�
			if(pid->resValue > pid->resThrd)
				pid->resValue = pid->resThrd;
			else if (pid->resValue < -pid->resThrd)
				pid->resValue = -pid->resThrd;
		}
	}
}
*/
