#include <rtthread.h>
#include "app_env.h"
#include "app_sensor.h"
#include "app_can.h"
#include "app_abs_encoder.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME    "app.sensor"
#define DBG_LEVEL           DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

//#define NOSENSOR

void alowlayer_comm_entry(void* param){
    //const uint32_t adjTick = RT_TICK_PER_SECOND/CONTROL_FS;
    rt_event_t timer_evt =      (rt_event_t)rt_object_find(TIMER_EVENT_NAME, RT_Object_Class_Event);
    rt_mp_t      sensor_mp =    (rt_mp_t)rt_object_find(MEMPOOL_SENSOR_NAME, RT_Object_Class_MemPool);
    rt_mp_t      motor_mp =    (rt_mp_t)rt_object_find(MEMPOOL_MOTOR_NAME, RT_Object_Class_MemPool);
    rt_mailbox_t motor_mb =     (rt_mailbox_t)rt_object_find(MAILBOX_MOTOR_NAME, RT_Object_Class_MailBox);
    rt_mailbox_t sensor_mb =    (rt_mailbox_t)rt_object_find(MAILBOX_SENSOR_NAME, RT_Object_Class_MailBox);
    RT_ASSERT(timer_evt);
    RT_ASSERT(sensor_mb);
    RT_ASSERT(motor_mb);
    RT_ASSERT(sensor_mp);
    RT_ASSERT(motor_mp);
    
#ifndef NOSENSOR    
    can_motor_enable(1,RT_TRUE);
    can_motor_enable(3, RT_TRUE);
    if (!can_reset_q()){
        LOG_E("imu reset q failed\n");
    }

    //g_reset_encoder = RT_TRUE;
    do{
        float tmp[8];
        int16_t encoder;
        if (!can_read_gyro((float*)tmp)){
            LOG_E("imu first read w failed\n");
        }
        else{
            LOG_I("imu first read w success\n");
       }
       /*if (!can_encoder_read(&encoder)) {
           rt_kprintf("encoder first read failed\n");
       } */
			 if (!encoder_read(&encoder)) {
           LOG_E("encoder first read failed\n");
       } 
       else{
           LOG_I("encoder first read success\n");
       }
    }while (0);
#endif    
    rt_tick_t tick = 0;// rt_tick_get();
    rt_event_control(timer_evt, RT_IPC_CMD_RESET, RT_NULL);
    while (1){
        rt_bool_t can_faild = RT_FALSE;
        rt_uint32_t ercv;
        if (RT_EOK != rt_event_recv(timer_evt, 0x01, RT_EVENT_FLAG_CLEAR | RT_EVENT_FLAG_OR, RT_TICK_PER_SECOND, &ercv)) 
            continue;
        rt_tick_t curr = rt_tick_get();
        if (tick ==0) {
            tick = curr;
            continue;
        }
        tick = curr;    
#ifndef NOSENSOR  				
        if (g_reset_encoder){
            g_reset_encoder = RT_FALSE;
            //can_encoder_reset();
						//encoder_reset();
        }
        lowcomm_t* psen = (lowcomm_t*)rt_mp_alloc(sensor_mp, RT_WAITING_FOREVER);
        drvcomm_t* drv = RT_NULL;
        if (!psen) continue;
        /*if (!can_encoder_read(&psen->encoder)) {
           rt_kprintf("encoder read failed\n");
           can_faild = RT_TRUE;
           //goto process_mp;
        } */
				if (!encoder_read(&psen->encoder)) {
           LOG_E("encoder read failed\n");
           can_faild = RT_TRUE;
           //goto process_mp;
        }			
				else{
					if ((psen->encoder > 512) || (psen->encoder < -512)){
						LOG_I("read_encoder: %d\n", psen->encoder);
					}
				}
        if (!can_read_gyro((float*)&psen->gyro)){
            LOG_E("imu read w failed\n");
            can_faild = RT_TRUE;
            //goto process_mp;
        }
        if (!can_read_q((float*)&psen->q)) {
            LOG_E("imu read q failed\n");
            can_faild = RT_TRUE;
           //goto process_mp;
        }
        if (can_faild) {
					if (g_trigger_command != Trigger_Idle) g_trigger_command = Trigger_CmdMiss;
					goto process_mp;
				}
        rt_memcpy(&psen->position, &g_position, sizeof(axis_t));
        rt_memcpy(&psen->speed, &g_speed, sizeof(axis_t));
        psen->gyro.X = ToDeg(psen->gyro.X);
        psen->gyro.Y = ToDeg(psen->gyro.Y);
        psen->gyro.Z = ToDeg(psen->gyro.Z);
        psen->command = g_command;
        psen->parameter = g_parameter;
        psen->encoder = -psen->encoder;
        if (g_command != Cmd_ByPass) LOG_I("\n%d\n", g_command);
        g_command = Cmd_ByPass;
        rt_mb_send(sensor_mb, (rt_ubase_t) psen);
        psen = RT_NULL;
#endif
        if (g_trigger_command < Trigger_Idle){
            switch(g_trigger_command){
                case Trigger_LockerInactive:
                    if (can_trigger_set_locker(RT_FALSE)){
                        g_trigger_command = Trigger_Idle;
                        g_trigger_locker = RT_FALSE;
                    }
										else g_trigger_command = Trigger_CmdMiss;
                    break;
                case Trigger_Shoot:
                     if (!g_trigger_locker){
                         if (can_trigger_shoot(g_trigger_position)){
                            g_trigger_command = Trigger_Idle;
                         }
												 else g_trigger_command = Trigger_CmdMiss;
                    }
										else g_trigger_command = Trigger_Idle;
                   break;
                case Trigger_LockerActive:
                    if (can_trigger_set_locker(RT_TRUE)){
                        g_trigger_command = Trigger_Idle;
                        g_trigger_locker = RT_TRUE;
                    }
										else g_trigger_command = Trigger_CmdMiss;
                    break;
								case Trigger_CmdMiss:
									g_trigger_command = Trigger_Idle;
										break;
            default:
                 g_trigger_command++;
                break;            
            }
        }
#ifndef NOSENSOR 					
        if (RT_EOK != rt_mb_recv(motor_mb, (rt_ubase_t*)&drv,RT_TICK_PER_SECOND/100)) continue;
        if (!drv) continue;
        if (drv->reset){
            if (drv->reset == 1){
                can_reset_q();
            }
            else if(drv->reset == 2){
                can_set_yaw(drv->yaw.speed);     
            }
            //if ((drv->yaw.speed >0.01) || (drv->yaw.speed <-0.01))
                //can_set_yaw(drv->yaw.speed);            
                can_set_motor(1,0);
                can_set_motor(3,0);

        }            
        else{
            can_set_motor(1,drv->pitch.speed);
            //can_set_motor(3,drv->pitch.speed);
            can_set_motor(3,drv->yaw.speed);
        }
        rt_mp_free(drv);
#endif        
process_mp:
#ifndef NOSENSOR 		
				if (psen) rt_mp_free(psen);
        if (can_faild){
            can_set_motor(1,0);
            can_set_motor(3,0);
            LOG_E("read can failed, close moter 1 & 3\n");
       }
#endif 
		}
        		
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void speed(int argc, char** argv){
    if (argc == 1) return ;
    g_speed.pitch = atof(argv[1]);
    if (argc > 1) g_speed.yaw = atof(argv[2]);
    g_command = Cmd_KeepSpeed;
}
MSH_CMD_EXPORT_ALIAS(speed, speed, set speed pitch yaw);

void position(int argc, char** argv){
    if (argc == 1) return ;
    g_position.pitch = atof(argv[1]);
    if (argc > 1) g_position.yaw = atof(argv[2]);
    g_command = Cmd_KeepPosition;
}
MSH_CMD_EXPORT_ALIAS(position, position, set position pitch yaw);

void eular(int argc, char** argv){
    if (argc == 1) return ;
    g_position.pitch = atof(argv[1]);
    if (argc > 1) g_position.yaw = atof(argv[2]);
    g_command = Cmd_KeepPositionEular;
}
MSH_CMD_EXPORT_ALIAS(eular, eular, set eular pitch yaw);

void pitch_speed(int argc, char** argv)
{
    if (argc == 1) return ;
    g_speed.pitch = atof(argv[1]);
    g_command = Cmd_KeepSpeed;
}
MSH_CMD_EXPORT_ALIAS(pitch_speed, pitch_speed, set pitch speed);

void pitch_position(int argc, char** argv)
{
    if (argc == 1) return ;
    g_position.pitch = atof(argv[1]);
    g_command = Cmd_KeepPosition;

}
MSH_CMD_EXPORT_ALIAS(pitch_position, pitch_position, set pitch position);

void yaw_speed(int argc, char** argv)
{
    if (argc == 1) return ;
    g_speed.yaw = atof(argv[1]);
    g_command = Cmd_KeepSpeed;
}
MSH_CMD_EXPORT_ALIAS(yaw_speed, yaw_speed, set pitch speed);

void yaw_position(int argc, char** argv)
{
    if (argc == 1) return ;
    g_position.yaw = atof(argv[1]);
    g_command = Cmd_KeepPosition;

}
MSH_CMD_EXPORT_ALIAS(yaw_position, yaw_position, set pitch position);

void follow_encorder(int argc, char** argv)
{  
    g_command = Cmd_KeepPositionCurrent;
}
MSH_CMD_EXPORT_ALIAS(follow_encorder, follow_encorder, follow encoder position);

void follow_eular(int argc, char** argv)
{ 
    g_command = Cmd_KeepPositionEularCurrent;
}
MSH_CMD_EXPORT_ALIAS(follow_eular, follow_eular, follow eular position);

void go_middle(int argc, char** argv){ 
    g_position.pitch = 0;
    g_position.roll = 0;
    g_position.yaw = 0;
    g_speed.pitch = 0;
    g_speed.roll = 0;
    g_speed.yaw = 0;
    g_command = Cmd_GoMiddle;  
}
MSH_CMD_EXPORT_ALIAS(go_middle, go_middle, go to middle of plane);

void reset_enc(int argc, char** argv){ 
    g_reset_encoder = RT_TRUE;
}
MSH_CMD_EXPORT_ALIAS(reset_enc, reset_enc, reset encoder to 0);

void servo_lock(int argc, char** argv){ 
	if (argc < 2) return;
	if (g_trigger_command !=  Trigger_Idle)rt_kprintf("not idle\n");
	if (atoi(argv[1]) != 0)
		 g_trigger_command = Trigger_LockerActive;
	else
		 g_trigger_command = Trigger_LockerInactive;
}
MSH_CMD_EXPORT_ALIAS(servo_lock, servo_lock, set servo lock state);

void servo_state(int argc, char** argv){ 
	rt_kprintf("current servo is %s\n", g_trigger_locker?"Lock":"Unlock");
}
MSH_CMD_EXPORT_ALIAS(servo_state, servo_state, get servo lock state);

void servo_position(int argc, char** argv){ 
	if (argc < 2) return;
	if (g_trigger_command !=  Trigger_Idle)rt_kprintf("not idle\n");
	int8_t position = atoi(argv[1]);
	if (position < 0) position = 0;
	else if (position > 100) position = 100;
	g_trigger_position = position;
	g_trigger_command = Trigger_Shoot;
}
MSH_CMD_EXPORT_ALIAS(servo_position, servo_position, set servo position);

#endif
