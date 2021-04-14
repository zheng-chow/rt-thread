#include <rtthread.h>
#include "app_env.h"
#include "app_motor.h"
#include "app_pid.h"
#include "AHRSHelper.h"
//#define Md 	    (0.15f)
//#define Maxq    (sqrt(1-Md*Md))
//#define MdTotal (0.25f)
//#define Md 	    (0.1f)
//#define Maxq    (sqrt(MdTotal*MdTotal-Md*Md))

#define PITCH_MAX_ANGLE     (20)
#define PITCH_MIN_ANGLE     (-20)
#define YAW_MAX_ANGLE       (25)
#define YAW_MIN_ANGLE       (-25)

#define  PITCH_PID_P        (8.0f)//(5.0)//(50.f)
#define  PITCH_PID_I        (0.02f)
#define  PITCH_PID_D        (0.f)//(PITCH_PID_P / 100.f)

#define  YAW_PID_P          (8.0f)//(50.f)
#define  YAW_PID_I          (0.02f)//(YAW_PID_P/100.f)
#define  YAW_PID_D          (0.f)//(YAW_PID_P/400.f)

#define  ROLL_PID_P         (8.0)//(50.f)
#define  ROLL_PID_I         (0.02f)//(ROLL_PID_P/200.f)
#define  ROLL_PID_D         (0.f)//(ROLL_PID_P/400.f)

/*
#define  PITCH_SPD_P        (0.005f)//(0.0025f)
#define  PITCH_SPD_I        (PITCH_SPD_P/50.f)
#define  PITCH_SPD_D        (0.f)

#define  ROLL_SPD_P         (0.0030f)
#define  ROLL_SPD_I         (ROLL_SPD_P/49.f)
#define  ROLL_SPD_D         (0.f)

#define  YAW_SPD_P          (0.0035f)
#define  YAW_SPD_I          (YAW_SPD_P/49.f)
#define  YAW_SPD_D          (0.f)
*/
#define MIN_MOTOR_SPEED 1050
#pragma anon_unions
typedef enum{
    WS_Init,
    WS_GoMiddleInit,
    WS_GoMiddle,
    WS_KeepSpeed,
    WS_KeepPosition,
    WS_KeepPositionEular,
    WS_KeepPositionCurrent,
    WS_KeepPositionEularCurrent,
    WS_DoCommand,
}WorkState_t;
typedef enum{
    Mode_ByPass,
    Mode_KeepSpeed,
    Mode_KeepPosition,
}AxisKeepMode_t;
typedef union{
    //WS_Init
    struct{
        Eular_t lastEular;
        rt_tick_t lastTick;
    };
    struct{
        uint8_t modePitch;
        uint8_t modeRoll;
        uint8_t modeYaw;
        axis_t  target;
    };
    //WS_GoMiddle
    //WS_KeepSpeed
    //WS_KeepPosition    
}WorkStateParam_t;
Eular_t prtEular;
static float GetEncoderYaw(int encoder){
	float fenc = encoder * 360.0/1024.0;
	if ((fenc > 180)|| (fenc < -180)){
		rt_kprintf("enc %d\n", encoder);
	}
	return fenc;
    //return encoder * 360.0/4096;
}
static uint8_t GetModeFromCommand(uint8_t command, uint8_t lastMode){
    switch(command){
        case Cmd_ByPass:
            return lastMode;
        case Cmd_GoMiddle:
            return WS_GoMiddle;
        case Cmd_KeepSpeed:
            return WS_KeepSpeed;
        case Cmd_KeepPosition:
            return WS_KeepPosition;
        case Cmd_KeepPositionEular:
            return WS_KeepPositionEular;     
        case   Cmd_KeepPositionMove:
            return WS_DoCommand;   
        case   Cmd_KeepPositionCurrent:
            return WS_KeepPositionCurrent;   
        case   Cmd_KeepPositionEularCurrent:
            return WS_KeepPositionEularCurrent;   
    }
    return WS_DoCommand;
}
void printf_entry(void* parameter){
    while(1){
        rt_kprintf("%d %d %d\n", (int)(prtEular.roll*1), (int)(prtEular.pitch*1), (int)(prtEular.yaw*1));
        rt_thread_mdelay(100);
    }
}
int app_printf(void){
    rt_thread_t tid = rt_thread_create("cancmd",printf_entry, RT_NULL, 2048, 20, RT_TICK_PER_SECOND/10);
    if (tid) rt_thread_startup(tid);
    return 0;
}
//INIT_APP_EXPORT(app_printf);
void highlayer_proc_entry(void* param){
    rt_mp_t      sensor_mp =    (rt_mp_t)rt_object_find(MEMPOOL_SENSOR_NAME, RT_Object_Class_MemPool);
    rt_mp_t      motor_mp =     (rt_mp_t)rt_object_find(MEMPOOL_MOTOR_NAME, RT_Object_Class_MemPool);
    rt_mailbox_t motor_mb =     (rt_mailbox_t)rt_object_find(MAILBOX_MOTOR_NAME, RT_Object_Class_MailBox);
    rt_mailbox_t sensor_mb =    (rt_mailbox_t)rt_object_find(MAILBOX_SENSOR_NAME, RT_Object_Class_MailBox);
    uint8_t workState = WS_Init;//WS_KeepSpeed;//WS_Init
    uint8_t workStateNew = WS_Init;
    WorkStateParam_t wsParam;
    drvcomm_t       lastDrv;
    PID_t pitch_position;
    PID_t yaw_position;  
    //int16_t encoderLock = 0;
    //Eular_t  eularLock = {0,0,0};
    //float    yawOffset = 0;

    RT_ASSERT(sensor_mb);
    RT_ASSERT(motor_mb);
    RT_ASSERT(sensor_mp);
    RT_ASSERT(motor_mp);
    rt_memset(&wsParam, 0, sizeof(WorkStateParam_t));
    rt_memset(&lastDrv, 0, sizeof(drvcomm_t));
    pid_init(&pitch_position,    PITCH_PID_P,    PITCH_PID_I,    PITCH_PID_D);
    pid_init(&yaw_position,      YAW_PID_P,      YAW_PID_I,      YAW_PID_D);


    // 初始化PID限值
    if (pitch_position.Ki != 0) 
        pid_threshold(&pitch_position,  10/pitch_position.Ki,   30.f,   0.1f);
    else
        pid_threshold(&pitch_position,  0.f,                    30.f,   0.1f);
    if (yaw_position.Ki != 0)
        pid_threshold(&yaw_position,    10/yaw_position.Ki,     30.f,   0.3f);
    else
        pid_threshold(&yaw_position,    0.f,                    30.f,   0.3f);	
    
    
       
    pid_enable(&pitch_position,RT_TRUE);
    pid_enable(&yaw_position,RT_TRUE);
    

    //g_speed.yaw = 5;
    //workState = WS_KeepSpeed;
    //rt_bool_t bInit = RT_FALSE;
    while (1){
        Eular_t eularReal;
        Eular_t eular;
				rt_bool_t exMinPitch = RT_FALSE, exMaxPitch = RT_FALSE, exMinYaw = RT_FALSE, exMaxYaw = RT_FALSE;
        
        lowcomm_t* psen;
        drvcomm_t* drv;
        rt_mb_recv(sensor_mb, (rt_ubase_t*)&psen, RT_WAITING_FOREVER);
        if (!psen)continue;
        drv = (drvcomm_t*)rt_mp_alloc(motor_mp, RT_WAITING_FOREVER);
        rt_memcpy(drv, &lastDrv,  sizeof(drvcomm_t));     
        
        Q_ToEularGeneral(&psen->q, &eularReal);
        eular.pitch = eularReal.pitch = ToDeg(eularReal.pitch);
        eular.roll = eularReal.roll = ToDeg(eularReal.roll);
        eular.yaw = GetEncoderYaw(psen->encoder);
        eularReal.yaw = ToDeg(eularReal.yaw);     

				exMinPitch = (eular.pitch <= PITCH_MIN_ANGLE);
				exMaxPitch = (eular.pitch >= PITCH_MAX_ANGLE);
				exMinYaw = (eular.yaw <= YAW_MIN_ANGLE);
				exMaxYaw = (eular.yaw >= YAW_MAX_ANGLE);
			

        prtEular.pitch = eularReal.yaw;//eular.pitch;
        prtEular.yaw = eular.yaw;
			

        
        if ((WS_Init == workState)||(WS_GoMiddleInit == workState))
            workStateNew = workState;
        else 
            workStateNew = GetModeFromCommand(psen->command, workState);

        switch(workStateNew){
        case WS_Init:
            if (!wsParam.lastTick){
                wsParam.lastTick = rt_tick_get();
                rt_memcpy(&wsParam.lastEular, &eular, sizeof(Eular_t));
                rt_kprintf("stable\n");
            }
            else{
                rt_tick_t curr = rt_tick_get();
                //test
                 wsParam.modePitch = Mode_KeepPosition;
                 wsParam.modeRoll = Mode_KeepPosition;
                 wsParam.modeYaw = Mode_KeepPosition;
                 wsParam.target.pitch = 0;
                 wsParam.target.roll = 0;
                 wsParam.target.yaw = 0;
                 workStateNew = WS_GoMiddleInit;
                 break;
                //test end                   
                /*
                if (fabsf(wsParam.lastEular.pitch - eular.pitch)>= 0.5f) wsParam.lastTick = curr;
                //else if (fabsf(wsParam.lastEular.roll - eular.roll)>= 0.5f) wsParam.lastTick = curr;
                else if (fabsf(wsParam.lastEular.yaw - eular.yaw)>= 0.5f) wsParam.lastTick = curr;
                else if ((curr - wsParam.lastTick) > RT_TICK_PER_SECOND){
                    workStateNew = WS_GoMiddleInit;
                    wsParam.modePitch = Mode_KeepPosition;
                    wsParam.modeRoll = Mode_KeepPosition;
                    wsParam.modeYaw = Mode_KeepPosition;
                    wsParam.target.pitch = 0;
                    wsParam.target.roll = 0;
                    wsParam.target.yaw = 0;
                    rt_kprintf("go middle\n");
                }*/
            }
            break;//continue;
        case WS_GoMiddle:
            wsParam.modePitch = Mode_KeepPosition;
            wsParam.modeRoll = Mode_KeepPosition;
            wsParam.modeYaw = Mode_KeepPosition;
            wsParam.target.pitch = 0;
            wsParam.target.roll = 0;
            wsParam.target.yaw = 0;
        case WS_GoMiddleInit:
            if (fabsf(wsParam.target.pitch  - eular.pitch)> 0.1f) break;
            //else if (fabsf(wsParam.target.roll - eular.roll)> 0.1f) break;
            //else if (fabsf(wsParam.target.yaw - eular.yaw)> 0.1f) break;
            else if ((psen->encoder>3) || (psen->encoder < -3))break;
            else {
                    rt_kprintf("reach middle\n");
                    wsParam.modePitch = Mode_ByPass;
                    wsParam.modeRoll = Mode_ByPass;
                    wsParam.modeYaw = Mode_ByPass;
                    wsParam.target.pitch = 0;
                    wsParam.target.roll = 0;
                    wsParam.target.yaw = 0;
                    workStateNew = WS_KeepPosition;
                    drv->pitch.speed = eular.pitch;
                    drv->roll.speed = eular.roll;
                    drv->yaw.speed = eular.yaw;
             }                 
            break;
        case WS_KeepSpeed:
            wsParam.modePitch = Mode_KeepSpeed;
            wsParam.modeRoll = Mode_KeepSpeed;
            wsParam.modeYaw = Mode_KeepSpeed;
            wsParam.target.pitch = psen->speed.pitch;
            wsParam.target.roll = psen->speed.roll;
            wsParam.target.yaw = psen->speed.yaw;    
			
						if (exMinPitch && (wsParam.target.pitch<0)) wsParam.target.pitch = 0;
						if (exMaxPitch && (wsParam.target.pitch>0)) wsParam.target.pitch = 0;
						if (exMinYaw && (wsParam.target.yaw<0)) wsParam.target.yaw = 0;
						if (exMaxYaw && (wsParam.target.yaw>0)) wsParam.target.yaw = 0;	
				
            break;
        case WS_KeepPosition:            
            wsParam.modePitch = Mode_KeepPosition;
            wsParam.modeRoll = Mode_KeepPosition;
            wsParam.modeYaw = Mode_KeepPosition;
            wsParam.target.pitch = psen->position.pitch;
            wsParam.target.roll = psen->position.roll;
            wsParam.target.yaw = psen->position.yaw; 

						if (exMinPitch && (wsParam.target.pitch<PITCH_MIN_ANGLE)) wsParam.target.pitch = PITCH_MIN_ANGLE;
						if (exMaxPitch && (wsParam.target.pitch>PITCH_MAX_ANGLE)) wsParam.target.pitch = PITCH_MAX_ANGLE;
						if (exMinYaw && (wsParam.target.yaw<YAW_MIN_ANGLE)) wsParam.target.yaw = YAW_MIN_ANGLE;
						if (exMaxYaw && (wsParam.target.yaw>YAW_MAX_ANGLE)) wsParam.target.yaw = YAW_MAX_ANGLE;	
				
            break;
        case WS_KeepPositionEular:            
            wsParam.modePitch = Mode_KeepPosition;
            wsParam.modeRoll = Mode_KeepPosition;
            wsParam.modeYaw = Mode_KeepPosition;
            wsParam.target.pitch = psen->position.pitch;
            wsParam.target.roll = psen->position.roll;
            wsParam.target.yaw = psen->position.yaw;  
				
						if (exMinPitch && (wsParam.target.pitch < PITCH_MIN_ANGLE)) wsParam.target.pitch = PITCH_MIN_ANGLE;
						if (exMaxPitch && (wsParam.target.pitch > PITCH_MAX_ANGLE)) wsParam.target.pitch = PITCH_MAX_ANGLE;
						if (exMinYaw && ((wsParam.target.yaw + eular.yaw - eularReal.yaw) < YAW_MIN_ANGLE)) wsParam.target.yaw = YAW_MIN_ANGLE + eularReal.yaw - eular.yaw;
						if (exMaxYaw && ((wsParam.target.yaw + eular.yaw - eularReal.yaw) > YAW_MAX_ANGLE)) wsParam.target.yaw = YAW_MAX_ANGLE + eularReal.yaw - eular.yaw;	
				
            eular.yaw = eularReal.yaw;    
				
            break;
         case WS_KeepPositionCurrent:            
            wsParam.modePitch = Mode_KeepPosition;
            wsParam.modeRoll = Mode_KeepPosition;
            wsParam.modeYaw = Mode_KeepPosition;
            if (workState != Cmd_KeepPositionCurrent){
                wsParam.target.pitch = eular.pitch;
                wsParam.target.roll = eular.roll;
                wsParam.target.yaw = eular.yaw;
            }        
						if (exMinPitch && (wsParam.target.pitch<PITCH_MIN_ANGLE)) wsParam.target.pitch = PITCH_MIN_ANGLE;
						if (exMaxPitch && (wsParam.target.pitch>PITCH_MAX_ANGLE)) wsParam.target.pitch = PITCH_MAX_ANGLE;
						if (exMinYaw && (wsParam.target.yaw<YAW_MIN_ANGLE)) wsParam.target.yaw = YAW_MIN_ANGLE;
						if (exMaxYaw && (wsParam.target.yaw>YAW_MAX_ANGLE)) wsParam.target.yaw = YAW_MAX_ANGLE;	
						
            break;
        case WS_KeepPositionEularCurrent:            
            wsParam.modePitch = Mode_KeepPosition;
            wsParam.modeRoll = Mode_KeepPosition;
            wsParam.modeYaw = Mode_KeepPosition;
            if (workState != WS_KeepPositionEularCurrent){
                wsParam.target.pitch = eularReal.pitch;
                wsParam.target.roll = eularReal.roll;
                wsParam.target.yaw = eularReal.yaw;
            }       
						if (exMinPitch && (wsParam.target.pitch < PITCH_MIN_ANGLE)) wsParam.target.pitch = PITCH_MIN_ANGLE;
						if (exMaxPitch && (wsParam.target.pitch > PITCH_MAX_ANGLE)) wsParam.target.pitch = PITCH_MAX_ANGLE;
						if (exMinYaw && ((wsParam.target.yaw + eular.yaw - eularReal.yaw) < YAW_MIN_ANGLE)) wsParam.target.yaw = YAW_MIN_ANGLE + eularReal.yaw - eular.yaw;
						if (exMaxYaw && ((wsParam.target.yaw + eular.yaw - eularReal.yaw) > YAW_MAX_ANGLE)) wsParam.target.yaw = YAW_MAX_ANGLE + eularReal.yaw - eular.yaw;	
            eular.yaw = eularReal.yaw;    
						
            break;       
        case WS_DoCommand:
            switch(psen->command){               
                case Cmd_KeepPositionMove:
                    wsParam.modePitch = Mode_KeepPosition;
                    wsParam.modeRoll = Mode_KeepPosition;
                    wsParam.modeYaw = Mode_KeepPosition;
                    wsParam.target.pitch = eular.pitch + psen->position.pitch;
                    wsParam.target.roll = psen->position.roll;
                    wsParam.target.yaw = eular.yaw + psen->position.yaw;  
								
										if (exMinPitch && (wsParam.target.pitch<PITCH_MIN_ANGLE)) wsParam.target.pitch = PITCH_MIN_ANGLE;
										if (exMaxPitch && (wsParam.target.pitch>PITCH_MAX_ANGLE)) wsParam.target.pitch = PITCH_MAX_ANGLE;
										if (exMinYaw && (wsParam.target.yaw<YAW_MIN_ANGLE)) wsParam.target.yaw = YAW_MIN_ANGLE;
										if (exMaxYaw && (wsParam.target.yaw>YAW_MAX_ANGLE)) wsParam.target.yaw = YAW_MAX_ANGLE;							
								
                    break;   
                default:
                    break;
            }
            break;
        }
        workState = workStateNew;
        WorkStateParam_t wsParamCtrl = wsParam;
        if ((workState == WS_DoCommand) && (workStateNew != WS_DoCommand)){
            drv->reset = 2;
            drv->yaw.speed = eular.yaw;
        }
        else if ((wsParamCtrl.modePitch == Mode_ByPass) 
            && (wsParamCtrl.modeRoll == Mode_ByPass)
        && (wsParamCtrl.modeYaw == Mode_ByPass))
            drv->reset = 1;
        else {
            switch(wsParamCtrl.modePitch){
                case Mode_KeepPosition:
                {//处理限位
                    float dAngle = eular.pitch - wsParamCtrl.target.pitch;
                    //int r = round(dAngle/360+0.5);                    
                    if (dAngle < -180) wsParamCtrl.target.pitch -= 360;
                    else if (dAngle > 180) wsParamCtrl.target.pitch += 360;
                }
                    pitch_position.targValue = wsParamCtrl.target.pitch;
                    pitch_position.measValue = eular.pitch;
                    pid_update3(&pitch_position);
                    wsParamCtrl.target.pitch = pitch_position.resValue;
                    //break;  
                case Mode_KeepSpeed:
                    drv->pitch.speed = wsParamCtrl.target.pitch*2100;	

                    if ((drv->pitch.speed > -MIN_MOTOR_SPEED) && (drv->pitch.speed < MIN_MOTOR_SPEED))
                        drv->pitch.speed = 0;
                    else if (lastDrv.pitch.speed == 0){
                        if ((drv->pitch.speed > -MIN_MOTOR_SPEED) && (drv->pitch.speed < MIN_MOTOR_SPEED))
                        drv->pitch.speed = 0;
                    }
																				
                    break;
                case Mode_ByPass:
                    break;
            }   
            switch(wsParamCtrl.modeRoll){
                case Mode_KeepPosition:
                    //break;  
                case Mode_KeepSpeed:
                    //drv->roll.speed = wsParamCtrl.target.roll;	
                    break;
                case Mode_ByPass:
                    break;
            }  
            switch(wsParamCtrl.modeYaw){
                case Mode_KeepPosition:
                {//处理限位
                    float dAngle = eular.yaw - wsParamCtrl.target.yaw;
                    if (dAngle < -180) wsParamCtrl.target.yaw -= 360;
                    else if (dAngle > 180) wsParamCtrl.target.yaw += 360;
                }
                    yaw_position.targValue = wsParamCtrl.target.yaw;
                    yaw_position.measValue = eular.yaw;
                    pid_update3(&yaw_position);
                    wsParamCtrl.target.yaw = yaw_position.resValue;
                    //break;  
                case Mode_KeepSpeed:
                    drv->yaw.speed = wsParamCtrl.target.yaw*2100;	
                    if ((drv->yaw.speed > -MIN_MOTOR_SPEED) && (drv->yaw.speed < MIN_MOTOR_SPEED))
                        drv->yaw.speed = 0;
                     else if (lastDrv.yaw.speed == 0){
                        if ((drv->yaw.speed > -MIN_MOTOR_SPEED) && (drv->yaw.speed < MIN_MOTOR_SPEED))
                        drv->yaw.speed = 0;
                    }			
                    break;
                case Mode_ByPass:
                    break;
            }              
            rt_memcpy(&lastDrv, drv, sizeof(drvcomm_t));
        }       
        //lastDrv.reset = 0;
        prtEular.roll = drv->pitch.speed;
				//drv->yaw.speed = -drv->yaw.speed;
        rt_mb_send(motor_mb,(rt_ubase_t)drv);        
        rt_mp_free(psen);
    }
}
