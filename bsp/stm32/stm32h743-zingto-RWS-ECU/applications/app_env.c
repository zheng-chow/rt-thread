#include <rtthread.h>
#include <rtdevice.h>
#include "app_env.h"
#include "app_configure.h"
#include "app_sensor.h"
#include "app_motor.h"

#define DBG_SECTION_NAME               "env"
#define LOG_TAG                        "app.env"
#define DBG_LEVEL                      DBG_INFO
#include <drv_log.h>


axis_t    	    g_position = {0,0,0};
axis_t    	    g_speed = {0,0,0};
uint8_t         g_command = Cmd_ByPass;
float 					g_parameter = 0;
rt_bool_t       g_reset_encoder = RT_FALSE;
uint8_t         g_trigger_command = Trigger_Idle;
int8_t         	g_trigger_position = 0;
rt_bool_t       g_trigger_locker = RT_TRUE;

static struct rt_event hwt_evt;
static struct rt_mempool mp_sensor;
static struct rt_mempool mp_motor;
static struct rt_mailbox mb_sensor;
static struct rt_mailbox mb_motor;
static struct rt_thread sensor_tid;
static struct rt_thread motor_tid;

ALIGN(RT_ALIGN_SIZE)
static void*  mb_sensor_buffer[4];
ALIGN(RT_ALIGN_SIZE)
static void*  mb_motor_buffer[4];

ALIGN(RT_ALIGN_SIZE)
static uint8_t mp_sensor_buffer[(sizeof(lowcomm_t) + sizeof(uint8_t*)*4)*4];
//static lowcomm_t mp_sensor_buffer[4];
ALIGN(RT_ALIGN_SIZE)
static uint8_t mp_motor_buffer[(sizeof(drvcomm_t) + sizeof(uint8_t*)*4)*4];
//static drvcomm_t mp_motor_buffer[4];

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t sensor_thread_stack[4096];
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t motor_thread_stack[8192];

static rt_err_t hwt_timeout(rt_device_t dev, rt_size_t size){
    return rt_event_send(&hwt_evt, 0x01);
}
rt_bool_t app_env_init(void){    
		rt_err_t ret = RT_EOK;
    if (!app_cfg_check()){
        LOG_E("Check configure file failed");
        return RT_FALSE;
    }
    if (!app_can_init()){
        LOG_E("Initialize can interface falled");
        return RT_FALSE;
    }
    //init static variable
    ret = rt_event_init(&hwt_evt, TIMER_EVENT_NAME, RT_IPC_FLAG_FIFO); if (ret != RT_EOK){LOG_E("%s failed", TIMER_EVENT_NAME); return RT_FALSE;}
    ret = rt_mb_init(&mb_sensor, MAILBOX_SENSOR_NAME, mb_sensor_buffer, sizeof(mb_motor_buffer)/4, RT_IPC_FLAG_FIFO);if (ret != RT_EOK){LOG_E("%s failed", TIMER_EVENT_NAME); return RT_FALSE;}
    ret = rt_mb_init(&mb_motor, MAILBOX_MOTOR_NAME, mb_motor_buffer, sizeof(mb_motor_buffer)/4, RT_IPC_FLAG_FIFO);if (ret != RT_EOK){LOG_E("%s failed", MAILBOX_SENSOR_NAME); return RT_FALSE;}
    ret = rt_mp_init(&mp_sensor, MEMPOOL_SENSOR_NAME, mp_sensor_buffer, sizeof(mp_sensor_buffer),  sizeof(lowcomm_t));if (ret != RT_EOK){LOG_E("%s failed", MEMPOOL_SENSOR_NAME); return RT_FALSE;}
    ret = rt_mp_init(&mp_motor, MEMPOOL_MOTOR_NAME, mp_motor_buffer, sizeof(mp_motor_buffer), sizeof(drvcomm_t));if (ret != RT_EOK){LOG_E("%s failed", MEMPOOL_MOTOR_NAME); return RT_FALSE;}
    ret = rt_thread_init(&sensor_tid, "lowcom", alowlayer_comm_entry, RT_NULL, &sensor_thread_stack[0], 4096, 18, RT_TICK_PER_SECOND/20);if (ret != RT_EOK){LOG_E("%s failed", "lowcom"); return RT_FALSE;}
    ret = rt_thread_init(&motor_tid, "driver", highlayer_proc_entry, RT_NULL, &motor_thread_stack[0], 8192, 17, RT_TICK_PER_SECOND/20);if (ret != RT_EOK){LOG_E("%s failed", "driver"); return RT_FALSE;}
    
    //find device
    rt_device_t hwt_dev = rt_device_find("timer2");
    if (!hwt_dev) {
        LOG_E("Cannot find hardware timer");
        return RT_FALSE;
    }   
    return RT_TRUE;
}

rt_bool_t app_env_start(void){
    //find device
    rt_device_t hwt_dev = rt_device_find("timer2");
    //start 
    rt_device_open(hwt_dev, RT_DEVICE_OFLAG_RDWR); 
    rt_device_set_rx_indicate(hwt_dev, hwt_timeout);
    rt_hwtimerval_t timeout_s = {0, 0};  
    rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD; 
    rt_device_control(hwt_dev, HWTIMER_CTRL_MODE_SET, &mode);
    timeout_s.sec = 0;    
    timeout_s.usec = 1000000/CONTROL_FS; 
    rt_device_write(hwt_dev, 0, &timeout_s, sizeof(timeout_s));

    rt_thread_startup(&sensor_tid);
    rt_thread_startup(&motor_tid);

    return RT_TRUE;
}

