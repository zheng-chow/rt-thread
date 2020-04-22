/*
 * Copyright (c) 2006-2018, ZINGTO UAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-04     serni        first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "guardian.h"
#include "general_pid.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME "ELAI"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#define TRACK_UARTPORT_NAME             "uart1"

#define TRACK_SEMAPHORE_NAME            "shELAI"
#define TRACK_SEMAPHORE_RX_NAME         "shELAIrx"

#define TRACK_EVENT_NAME                "evELAI"

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

#define ELAI_BUFFER_SIZE                (32)
#define ELAI_RX_TIMEOUT                 (RT_TICK_PER_SECOND / 20)

#define ELAI_STREAM_WIDTH               (1280)
#define ELAI_STREAM_HEIGHT              (720)

#define ELAI_CONTROL_NAIVE_MODE         (0x80)
#define ELAI_CONTROL_DEFAULT_ROI        (0x40)
#define ELAI_CONTROL_IS_PERSON          (0x20)
#define ELAI_CONTROL_MANUAL_RECOVER     (0x10)
#define ELAI_CONTROL_CENTER_SELECT      (0x08)
#define ELAI_CONTROL_TRACE_RESET        (0x04)
#define ELAI_CONTROL_TRIGGER            (0x02)
#define ELAI_CONTROL_POWER_OFF          (0x01)

#define ELAI_EVENT_MASK                 (0x00000003)
#define ELAI_EVENT_START                (0x00000001)
#define ELAI_EVENT_STOP                 (0x00000002)

#pragma pack(1)
typedef struct __ELAI_ControlPacket
{
    rt_uint8_t      HEADER;     // @
    rt_uint16_t     config;
    rt_uint16_t     roi_x;
    rt_uint16_t     roi_y;
    rt_uint16_t     roi_width;
    rt_uint16_t     roi_height;
    rt_uint8_t      TAIL;       // #
}ELAI_ControlPacket;

typedef struct __ELAI_ResponsePacket
{
    rt_uint16_t     roi_x;
    rt_uint16_t     roi_y;
    rt_uint16_t     roi_width;
    rt_uint16_t     roi_height;
    rt_uint16_t     status;
    float           confidence;
    rt_uint8_t      TAIL;       // #
}ELAI_ResponsePacket;
#pragma pack()

#define ELAI_PACKET_HEADER              ('@')
#define ELAI_PACKET_TAIL                ('#')

static rt_sem_t         __semaphore = RT_NULL;
static rt_event_t       __event = RT_NULL;

rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(__semaphore);
    
    return RT_EOK;
}

static rt_err_t uart_send_with_block(rt_device_t dev, void * buffer, rt_size_t size)
{
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    RT_ASSERT(size > 0);
    
    return rt_device_write(dev, 0, buffer, size);
}

static rt_err_t uart_clean_recv_buff(rt_device_t dev, void * buffer)
{
    rt_size_t sz_recv = 0;
    
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    
    while (1)
    {
        sz_recv = rt_device_read(dev, 0, buffer, ELAI_BUFFER_SIZE);
        
        if (sz_recv == 0)
            break;
    }
    
    return RT_EOK;
}

static rt_err_t uart_recv_with_timeout(rt_device_t dev, void * buffer, const rt_size_t size)
{
    rt_size_t sz_recv = 0;
    rt_err_t  result;
    
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    RT_ASSERT(size > 0);
    
    while (1)
    {
        result = rt_sem_take(__semaphore, ELAI_RX_TIMEOUT);
        
        if (result == -RT_ETIMEOUT)
            break;
        
        sz_recv += rt_device_read(dev, 0, (rt_uint8_t*)buffer + sz_recv, size - sz_recv);
        
        if (sz_recv == size)
            break;
    }
    
    return result;
}

static void trace_control_entry(void* parameter)
{
    struct guardian_environment *env = RT_NULL;
    ELAI_ControlPacket  control;
    ELAI_ResponsePacket response;

    rt_uint8_t*     pbuf;
    rt_err_t        result;
    rt_device_t     dev = RT_NULL;

    rt_uint32_t     e;
    rt_int32_t      flag_timeout = 0;
    rt_bool_t       on_tracing = RT_FALSE;
    
    PID_t           pid_x, pid_y;
    
    env = (struct guardian_environment*)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(ELAI_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    pid_init(&pid_x, 1.4f, 0.02f, 0.5f);
    pid_init(&pid_y, 1.4f, 0.02f, 0.5f);
    
    pid_setThreshold(&pid_x, 500.0f, 100.0f, 0.02f);
    pid_setThreshold(&pid_y, 500.0f, 100.0f, 0.02f);
    
    pid_setSetpoint(&pid_x, 0.0f);
    pid_setSetpoint(&pid_y, 0.0f);
    
    pid_enable(&pid_x, RT_TRUE);
    pid_enable(&pid_y, RT_TRUE);
    
    LOG_I("tracing control sub-thread, start!");
    
    while (1)
    {
        flag_timeout = 0;
        if (on_tracing == RT_FALSE)
            flag_timeout = RT_WAITING_FOREVER;
            
        result = rt_event_recv(__event, ELAI_EVENT_MASK, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, flag_timeout, &e);
        
        if (result == RT_EOK)
        {
            if (e == ELAI_EVENT_STOP)
            {
                LOG_D("stop ELAI tracing");
                
                on_tracing = RT_FALSE;
                
                env->trck_incharge = RT_TRUE;
                env->trck_lost = RT_TRUE;
                env->trck_err_x = 0;
                env->trck_err_y = 0;

                LOG_D("stop pantilt move");

                rt_sem_release(env->sh_ptz);
                
                continue;
            }
            else if (e == ELAI_EVENT_START)
            {
                LOG_D("start ELAI tracing");
                on_tracing = RT_TRUE;
            }
            else
            {
                LOG_D("unknown event, %08X", e);
                rt_thread_delay(1);
                continue;
            }
        }
        
        uart_clean_recv_buff(dev, pbuf);
        
        rt_memset(&control, 0x00, sizeof(ELAI_ControlPacket));
        control.HEADER = ELAI_PACKET_HEADER;
        control.TAIL = ELAI_PACKET_TAIL;
        uart_send_with_block(dev, &control, sizeof(ELAI_ControlPacket));
        
        result = uart_recv_with_timeout(dev, pbuf, sizeof(ELAI_ResponsePacket));
        
        if (result == -RT_ETIMEOUT) {
            LOG_W("no response");
            rt_thread_delay(RT_TICK_PER_SECOND);
            continue;
        }
        
        rt_memcpy(&response, pbuf, sizeof(ELAI_ResponsePacket));
        
        if (response.TAIL != ELAI_PACKET_TAIL) {
            LOG_W("invaild response in wrong format");
            ulog_hexdump("ELAI", 16, pbuf, sizeof(ELAI_ResponsePacket));
        }
        else {
            if (response.status == 0x01) {
                env->trck_lost = RT_FALSE;
                env->trck_incharge = RT_FALSE;
                
                rt_int16_t  roi_cx, roi_cy = 0;

                roi_cx = response.roi_x + response.roi_width / 2;
                roi_cy = response.roi_y + response.roi_height / 2;
                
                float deffer_x, deffer_y;
                
                deffer_x = ELAI_STREAM_WIDTH / 2 - roi_cx;
                deffer_y = ELAI_STREAM_HEIGHT / 2 - roi_cy;
                
                /* Enhance dynamic performance by PID control */
                
                env->trck_err_x = pid_update(&pid_x, deffer_x);
                env->trck_err_y = pid_update(&pid_y, deffer_y);                
                
                LOG_D("searching %d %d 0x%02X, 0.%02d", env->trck_err_x, env->trck_err_y, response.status, response.confidence*100);
            }
            else if (response.status == 0x02) {
                env->trck_lost = RT_FALSE;
                env->trck_incharge = RT_TRUE;
                
                rt_int16_t  roi_cx, roi_cy = 0;
                
                roi_cx = response.roi_x + response.roi_width / 2;
                roi_cy = response.roi_y + response.roi_height / 2;
                
                float deffer_x, deffer_y;
                
                deffer_x = ELAI_STREAM_WIDTH / 2 - roi_cx;
                deffer_y = ELAI_STREAM_HEIGHT / 2 - roi_cy;
                
                /* Enhance dynamic performance by PID control */
                
                env->trck_err_x = pid_update(&pid_x, deffer_x);
                env->trck_err_y = pid_update(&pid_y, deffer_y);
                
                rt_sem_release(env->sh_ptz);
                LOG_D("tracing %d %d 0x%02X, 0.%02d", env->trck_err_x, env->trck_err_y, response.status, response.confidence*100);
            }
            else if (response.status == 0x03) {
                env->trck_lost = RT_FALSE;
                env->trck_incharge = RT_TRUE;

                env->trck_err_x = 0;
                env->trck_err_y = 0;
                
                rt_sem_release(env->sh_ptz);
                LOG_D("try to regain target, 0x%02X",response.status);
            }
            else {

                /* update enviroment variables */
                env->trck_lost = RT_TRUE;
                env->trck_incharge = RT_TRUE;

                env->trck_err_x = 0;
                env->trck_err_y = 0;

                rt_sem_release(env->sh_ptz);
                LOG_W("lost target");
                LOG_D("tracing %d %d 0x%02X, 0.%02d", env->trck_err_x, env->trck_err_y, response.status, response.confidence*100);
                
                on_tracing = RT_FALSE;
            }
        }
        
        rt_thread_delay(RT_TICK_PER_SECOND / 5);
    }
    
    // should never happened.
}

void track_resolving_entry(void* parameter)
{
    static rt_device_t dev = RT_NULL;
    static struct guardian_environment *env = RT_NULL;
    ELAI_ControlPacket control; 
    rt_size_t pktsz;
    rt_thread_t pthread = RT_NULL;
    static rt_err_t result = RT_EOK;
    rt_uint32_t e;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    __semaphore = rt_sem_create(TRACK_SEMAPHORE_RX_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(__semaphore != RT_NULL);
    env->sh_track = rt_sem_create(TRACK_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(env->sh_track != RT_NULL);
    
    __event = rt_event_create(TRACK_EVENT_NAME, RT_IPC_FLAG_FIFO);
    RT_ASSERT(__event != RT_NULL);
    
    // set uart in 115200, 8E1.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.data_bits = DATA_BITS_8;
    config.parity = PARITY_NONE;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
    
    pthread = rt_thread_create("tELAIbg", trace_control_entry, env, 2048, 9, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    LOG_I("initialization finish, start!");
    
    env->trck_incharge = RT_FALSE;

    while (1)
    {
        result = rt_sem_take(env->sh_track, RT_WAITING_FOREVER);
        
        pktsz = sizeof(ELAI_ControlPacket);
        
        rt_memset(&control, 0x00, pktsz);
        
        control.HEADER = ELAI_PACKET_HEADER;
        control.TAIL = ELAI_PACKET_TAIL;
        
        if (env->trck_action == TRACK_ACTION_PREPARE)
        {
            control.config = ELAI_CONTROL_TRACE_RESET | \
                             ELAI_CONTROL_CENTER_SELECT | \
                             ELAI_CONTROL_NAIVE_MODE;
            
            control.roi_height = 0;
            control.roi_width = 0;
            control.roi_x = 0;
            control.roi_y = 0;
            
            LOG_D("TRACK_ACTION_TRACE_PREPARE");
            //ulog_hexdump("PREPARE", 16, (rt_uint8_t *)&control, sizeof(ELAI_ControlPacket));
            
            e = 0;
            
            env->trck_prepare = RT_TRUE;            
        }        
        else if (env->trck_action == TRACK_ACTION_TRACE_START)
        {
            control.config = ELAI_CONTROL_CENTER_SELECT | \
                             ELAI_CONTROL_MANUAL_RECOVER | \
                             ELAI_CONTROL_TRIGGER | \
                             ELAI_CONTROL_NAIVE_MODE;
            
            control.roi_height = 0;
            control.roi_width = 0;
            control.roi_x = 0;
            control.roi_y = 0;
            
            e = ELAI_EVENT_START;
            
            LOG_D("TRACK_ACTION_TRACE_START");
            //ulog_hexdump("START", 16, (rt_uint8_t *)&control, sizeof(ELAI_ControlPacket));
            
            env->trck_incharge = RT_FALSE;
            env->trck_prepare = RT_FALSE;
        }
        else if (env->trck_action == TRACK_ACTION_TRACE_STOP)
        {
            control.config = 0xFF;          // Loiter Mode
            
            control.roi_height = 0;
            control.roi_width = 0;
            control.roi_x = 0;
            control.roi_y = 0;
            
            LOG_D("TRACK_ACTION_TRACE_STOP");
            //ulog_hexdump("START", 16, (rt_uint8_t *)&control, sizeof(ELAI_ControlPacket));
            
            e = ELAI_EVENT_STOP;
            
            env->trck_incharge = RT_FALSE;
            env->trck_prepare = RT_FALSE;
        }
        else
            continue;
        
        uart_send_with_block(dev, &control, pktsz);
        env->trck_action = TRACK_ACTION_NULL;
        
        if (e != 0)
            rt_event_send(__event, e);
    }
    
    // never be here.
}

