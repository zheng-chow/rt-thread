#ifndef __APP_ENV_H__
#define __APP_ENV_H__
#include <rtdef.h>
#include "app_can.h"
#include "AHRSHelper.h"



#define TIMER_EVENT_NAME "etimer1"
#define MEMPOOL_SENSOR_NAME "mp_sen"
#define MAILBOX_SENSOR_NAME "mb_sen"
#define MEMPOOL_MOTOR_NAME "mp_mot"
#define MAILBOX_MOTOR_NAME "mb_mot"

typedef struct{
    float roll;
    float pitch;
    float yaw;
}axis_t;
typedef struct{
    Gyro_t          gyro;
    Quaternion_t    q;
    int16_t         encoder; 
    axis_t    	    position;
    axis_t    	    speed;
    uint8_t         command;
    float 			parameter;
}lowcomm_t;
typedef struct{
    float speed;
}drvmotor_t;
typedef struct{
    rt_bool_t  reset;
    drvmotor_t pitch;
    drvmotor_t roll;
    drvmotor_t yaw;
}drvcomm_t;

typedef enum{
    Cmd_ByPass,
    Cmd_GoMiddle,
    Cmd_KeepSpeed,//保持速度
    Cmd_KeepPosition,//保持Eular
    //Cmd_KeepPositionEncoder,//保持Eular ，yaw以编码器为准
    Cmd_KeepPositionEular,
    Cmd_KeepPositionMove,//保持Eular增量
    Cmd_KeepPositionCurrent,
    Cmd_KeepPositionEularCurrent,    

    //shoot
    Cmd_SetTriggerLocker,
    Cmd_TriggerShoot,
    Cmd_TriggerLockerState,
}CtrlCommand_t;

typedef enum{
    Trigger_LockerInactive,
    Trigger_Shoot,
    Trigger_LockerActive,
	
	
		Trigger_CmdMiss,
    Trigger_Idle,
}TriggerCammand_t;
rt_bool_t app_env_init(void);
rt_bool_t app_env_start(void);

extern axis_t    	    g_position;
extern axis_t    	    g_speed;
extern uint8_t          g_command;
extern float 						g_parameter;
extern rt_bool_t        g_reset_encoder;
extern uint8_t          g_trigger_command;
extern int8_t         	g_trigger_position;
extern rt_bool_t        g_trigger_locker;
#endif
