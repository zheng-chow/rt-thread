#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <drv_foc.h>
#include <drv_hall.h>
#include <app_speed.h>
#include <app_abs_encoder.h>
#include <app_can.h>
#include <app_pid.h>
#include <global_define.h>

#define xDRV_DEBUG
#define LOG_TAG             "app.speed"
#include <drv_log.h>

#define SPEED_PRINT
/**************
PID调速处理线程
************/
#define PID_FREQ    1000
#define DeltaT      (1.0f / 1000);

#define PID_KP      0.00000045
#define PID_KI      (PID_KP/4)
#define PID_KD      (PID_KP*0)
#define MAX_ERR_THRD	0//(2100*5)
#define MIN_ERR_THRD	(1)
#define ACC_ERR 0.3
#define MAX_PID_RES 0.7
#define MIN_PID_RES 0.02

/*
#if defined(AXIS_PITCH)
#define PID_KP       0.00000045
#define PID_KI      (PID_KP/10)
#define ACC_ERR 0.3
#define MAX_PID_RES 0.7
#define MIN_PID_RES 0.04

#elif defined(AXIS_YAW)

#define PID_KP       0.00000045
#define PID_KI      (PID_KP/10)
#define ACC_ERR 0.3
#define MAX_PID_RES 0.7
#define MIN_PID_RES 0.04

#else
#error "no pid parameter"
#endif
*/





///////////////////////////////////////////////////////////////////////
static struct rt_event 			evt_speed;
static struct rt_mailbox    mb_speed;
ALIGN(RT_ALIGN_SIZE)
static float speed_buffer;
///////////////////////////////////////////////////////////////////////
#ifdef SPEED_PRINT
static rt_bool_t bPrint = RT_FALSE;
static float     fPrintSpeed[3] = {0,0, 0};
void printf_entry(void* parameter){
    while(1){
        if (bPrint) rt_kprintf("%d  %d  %d\n", (int)(fPrintSpeed[0]) , (int)(fPrintSpeed[1]) , (int)(fPrintSpeed[2]));
        rt_thread_mdelay(10);
    }
}
int app_printf(void){
    rt_thread_t tid = rt_thread_create("spdprt",printf_entry, RT_NULL, 2048, 20, RT_TICK_PER_SECOND/10);
    if (tid) rt_thread_startup(tid);
    return 0;
}
//INIT_APP_EXPORT(app_printf);
#endif
///////////////////////////////////////////////////////////////////////
static rt_err_t speed_timeout_call(rt_device_t dev, rt_size_t size){
    rt_event_send(&evt_speed, SPEED_CONTROL_ADJUST_MOTOR);
    return RT_EOK;
}
///////////////////////////////////////////////////////////////////////
static void speed_system_init(void){
	static rt_bool_t init = RT_FALSE;
	if (!init){
		init = RT_TRUE;
		rt_device_t tim_dev = rt_device_find("timer5");    
    rt_device_t hall_dev = rt_device_find("hall");
    rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD; 
    rt_hwtimerval_t timeout_s = {
			.sec = 0,
			.usec = 1000000/PID_FREQ
		};
		rt_event_init(&evt_speed, SPEED_EVT_NAME, RT_IPC_FLAG_FIFO);
    rt_mb_init(&mb_speed, "mbspeed", &speed_buffer, sizeof(speed_buffer)/4, RT_IPC_FLAG_FIFO);
		RT_ASSERT(tim_dev && hall_dev);  
    rt_device_open(hall_dev, 0);
		rt_device_open(tim_dev, RT_DEVICE_OFLAG_RDWR);
    rt_device_set_rx_indicate(tim_dev, speed_timeout_call);
    rt_device_control(tim_dev, HWTIMER_CTRL_MODE_SET, &mode);
    rt_device_write(tim_dev, 0, &timeout_s, sizeof(timeout_s));		
		
		
	}
}
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
static void speed_pid(void* parameter){
    rt_device_t pwm_dev = rt_device_find("pwmcom");
    PID_t pid;
    rt_bool_t bPwmDevOpened = RT_FALSE;
    rt_tick_t tickBreath = 0;
    float targetSpeed = 0.0;
		
		extern float g_eangle;
    pwm_update_t pwm_param = {.detAngle=90};
	
		uint32_t evt;
	
    RT_ASSERT(pwm_dev);	
	
    pid_init(&pid, PID_KP, PID_KI, 0);  
    if (PID_KI != 0) 
        pid_threshold(&pid, ACC_ERR / PID_KI, MAX_PID_RES, MIN_ERR_THRD,MAX_ERR_THRD);
    else  pid_threshold(&pid, 0, MAX_PID_RES, MIN_ERR_THRD,MAX_ERR_THRD);
	
		speed_system_init();
		rt_event_recv(&evt_speed, SPEED_CONTROL_ADJUST_MOTOR, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
    
    while (1){
        rt_event_recv(&evt_speed, SPEED_CONTROL_EVENT, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        if (RT_EOK == rt_mb_recv(&mb_speed, (rt_ubase_t*)&targetSpeed, 0)){
            LOG_D("target speed: %d.%03d", (int)targetSpeed, (int)((targetSpeed-(int)targetSpeed)*1000));
            tickBreath = rt_tick_get();
#ifdef SPEED_PRINT
					bPrint = bPwmDevOpened && (targetSpeed != 0);
#endif
        }
        if ((evt & SPEED_CONTROL_OPEN_MOTOR) && !bPwmDevOpened){   
					rt_device_open(pwm_dev, 0);
					bPwmDevOpened = RT_TRUE;
					targetSpeed = 0;
					tickBreath = rt_tick_get();
#ifdef SPEED_PRINT
					bPrint = bPwmDevOpened && (targetSpeed != 0);
#endif
				}
        else if ((evt & SPEED_CONTROL_CLOSE_MOTOR) && bPwmDevOpened){
					rt_device_close(pwm_dev);
					bPwmDevOpened = RT_FALSE;
					pid_reset(&pid, 0);
					targetSpeed = 0;					
#ifdef SPEED_PRINT
					bPrint = bPwmDevOpened;
#endif
					pid_enable(&pid, RT_FALSE);   
				} 
				if ((evt & SPEED_CONTROL_ADJUST_MOTOR) && bPwmDevOpened){  
           if (!pid.bEnable){
                float power = foc_fast_get_power();
                if ((power > 1) && (power < 56)){
                    LOG_I("power detect error, %d V", (int)(power+0.5f));                    
                    power = power / 24;
                    pid_init(&pid, PID_KP/power, PID_KI/power, PID_KD/power);  
                    if (PID_KI != 0) 
                    pid_threshold(&pid, ACC_ERR * power / PID_KI, MAX_PID_RES, MIN_ERR_THRD, MAX_ERR_THRD);
                    else  pid_threshold(&pid, 0, MAX_PID_RES, MIN_ERR_THRD,MAX_ERR_THRD);            
                    pid_enable(&pid, RT_TRUE);   
                }
                else{
                    LOG_E("power detect error, %d V", (int)(power+0.5f));
                }           
           }
           else{ 
						 float speed = 0;//hall_fast_get_speed();
						 rt_int16_t encoder = 0x8000;
						if (!get_speed_from_encoder(&speed, &encoder)){
							 targetSpeed = 0;
							 speed = hall_fast_get_speed();
							 LOG_W("encoder failed");
						 }
						else{
							hall_fast_set_speed(speed);
						}				 
						 pid.measValue = speed;//hall_fast_get_speed();
						 pid.targValue = targetSpeed;	
						 pid_update3(&pid);

						 if (pid.resValue< 0){
							 pwm_param.detAngle = -90;
							 pid.resValue = -pid.resValue;
						 }
						 else pwm_param.detAngle =90;
						 pwm_param.coefM = pid.resValue;
						 rt_device_control(pwm_dev, PWM3_CONTROL_UPDATE, &pwm_param);
						 //tickBreath = rt_tick_get();						 
#ifdef SPEED_PRINT
						 fPrintSpeed[0] = pid.targValue;
						 fPrintSpeed[1] = pid.measValue;
						 fPrintSpeed[2] = pid.resValue*100000;
#endif
					 }
        }  
        if (bPwmDevOpened){
					rt_tick_t currTick = rt_tick_get();
					if ((currTick - tickBreath) > (RT_TICK_PER_SECOND/2)) {
						if (targetSpeed != 0){
							targetSpeed = 0;
							rt_mb_send(&mb_speed, *(rt_ubase_t*)&targetSpeed); 
						}   
					}
				}
		}     
}

int app_speed(void){
	if (g_init){
    rt_thread_t tid = rt_thread_create("speedpid",speed_pid, RT_NULL, 4096, 16, RT_TICK_PER_SECOND/10);
    if (tid) return rt_thread_startup(tid);
	}
	return -RT_ERROR;
}
INIT_APP_EXPORT(app_speed);


#ifdef RT_USING_FINSH
#include <finsh.h>
static void speed(uint8_t argc, char **argv){
    if (argc != 2) {
        LOG_E("command format: speed 40");
        return;
    }
    float speed = atof(argv[1]);
    speed *= 2100;
		LOG_I("target speed %d", (int) speed);
    rt_mailbox_t mb =     (rt_mailbox_t)rt_object_find("mbspeed", RT_Object_Class_MailBox);
    rt_mb_send(mb, *(rt_ubase_t*)&speed);    
}
FINSH_FUNCTION_EXPORT_ALIAS(speed, __cmd_speed, set speed);

static void pwmenable(uint8_t argc, char **argv){
    if (argc != 2) {
        LOG_E("command format: speed 40");
        return;
    }
    rt_event_t evt = (rt_event_t) rt_object_find(SPEED_EVT_NAME, RT_Object_Class_Event);
    if (evt){
        int enable = atoi(argv[1]);
        if (enable != 0){           
            rt_event_send(evt, SPEED_CONTROL_OPEN_MOTOR);
        }
        else{
            rt_event_send(evt, SPEED_CONTROL_CLOSE_MOTOR);
        }
    }
}
FINSH_FUNCTION_EXPORT_ALIAS(pwmenable, __cmd_pwmenable, enable or disable control speed);
static void pwmtest(uint8_t argc, char **argv){
    rt_event_t evt = (rt_event_t) rt_object_find(SPEED_EVT_NAME, RT_Object_Class_Event);
    rt_mailbox_t mb =     (rt_mailbox_t)rt_object_find("mbspeed", RT_Object_Class_MailBox);
		float speed_set = 1;
		float speed = 0;
		rt_tick_t current_tick = 0;
		rt_tick_t tick_5s = rt_tick_from_millisecond(5000);
		rt_tick_t tick_10s = 0;
		if (argc > 1){
			speed_set = atof(argv[1]);
			if (speed_set < 0) speed_set = -speed_set;
		}
		if (speed_set > 2) {
			tick_5s *= 2/ speed_set;
		}
			
		tick_10s = tick_5s * 2;
		speed_set*=2100;
    if (evt && mb){
			rt_event_send(evt, SPEED_CONTROL_OPEN_MOTOR);
			speed = speed_set; rt_mb_send(mb, *(rt_ubase_t*)&speed);  
			current_tick = rt_tick_get();	
			while ((rt_tick_get() - current_tick) < tick_5s){
        if (bPrint) rt_kprintf("%d  %d  %d\n", (int)(fPrintSpeed[0]) , (int)(fPrintSpeed[1]) , (int)(fPrintSpeed[2]));
				rt_thread_mdelay(10);
			}
			speed = -speed_set; rt_mb_send(mb, *(rt_ubase_t*)&speed);    
			current_tick = rt_tick_get();	
			while ((rt_tick_get() - current_tick) < tick_10s){
        if (bPrint) rt_kprintf("%d  %d  %d\n", (int)(fPrintSpeed[0]) , (int)(fPrintSpeed[1]) , (int)(fPrintSpeed[2]));
				rt_thread_mdelay(10);
			}
			speed = speed_set; rt_mb_send(mb, *(rt_ubase_t*)&speed);    
			current_tick = rt_tick_get();	
			while ((rt_tick_get() - current_tick) < tick_5s){
        if (bPrint) rt_kprintf("%d  %d  %d\n", (int)(fPrintSpeed[0]) , (int)(fPrintSpeed[1]) , (int)(fPrintSpeed[2]));
				rt_thread_mdelay(10);
			}
			speed = 0; rt_mb_send(mb, *(rt_ubase_t*)&speed);   
			rt_event_send(evt, SPEED_CONTROL_CLOSE_MOTOR);
        
    }
}
FINSH_FUNCTION_EXPORT_ALIAS(pwmtest, __cmd_pwmtest, turn down 5s then turn up 10s and finally turn down 5s);

#endif /* RT_USING_FINSH */
