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
#define DBG_SECTION_NAME "Track"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#define TRACK_UARTPORT_NAME "uart3"
#define TRACK_SEMAPHORE_NAME "shTRCK"
#define TRACK_SEMAPHORE_RX_NAME "shTRCKrx"

#define TRACK_SEND_MP_NAME "mpPTZtx"
#define TRACK_SEND_MB_NAME "mbPTZtx"

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

#define TRACK_BUFFER_SIZE       (64)

#define TRACK_ACK_PKT_SIZE      (8)
#define TRACK_ACK_PKT_HEADER    (0xBB)
#define TRACK_ACK_PKT_ADDR      (0x01)

#pragma pack(1)
typedef struct __SHB30X_SendPacket
{
	rt_uint16_t		HEADER;				//b1 Always be 0x7E7E.
	rt_uint8_t		ADDR;				//b3 Always be 0x44.
	rt_uint8_t		__reserved1;		//b4 reserved.
	rt_uint8_t		__reserved2;		//b5 reserved.
	rt_uint8_t		set_mode;			//b6 0x71,
	rt_uint8_t		set_fuction;		//b7 0xfe,
	rt_uint16_t		set_offset_x;       //b8
	rt_uint16_t		set_offset_y;       //b10
	rt_uint8_t		start_trace;        //b12 0x01,
	rt_uint8_t		__reserved3;		//b13 0x01,
	rt_uint8_t		set_trace_mode;		//b14 0x3c,
	rt_uint8_t		set_video_source;   //b15 0x00 - 0x03
	rt_uint8_t		__reserved5;		//b16
	rt_uint8_t		set_zoom;			//b17
	rt_uint8_t		__reserved6;		//b18
	rt_uint8_t		__reserved7;		//b19
	rt_uint8_t		__reserved8[28];	//b20
	rt_uint8_t		checksum;			//b48
}shb_serialctrlpkt;
#pragma pack()

static rt_sem_t semaph = RT_NULL;

static rt_mailbox_t mailbox = RT_NULL;
static rt_mp_t mempool = RT_NULL;

rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

static void track_data_send_entry(void* parameter)
{
    struct guardian_environment *env = RT_NULL;
    rt_ubase_t mail;
    rt_uint8_t* pbuf;
    rt_device_t dev = RT_NULL;
    
    env = (struct guardian_environment*)parameter;
    RT_ASSERT(env != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    LOG_I("send sub-thread, start!");
    
    while (1)
    {
        rt_mb_recv(mailbox, &mail, RT_WAITING_FOREVER);
        LOG_D("mb recv %0X", mail);
        if (mail == RT_NULL)
            continue;
        pbuf = (rt_uint8_t*)mail;
        
        LOG_D("send to shb30x");
        rt_device_write(dev, 0, pbuf, sizeof(shb_serialctrlpkt));
        
        rt_mp_free(pbuf);
        
        rt_thread_delay(1);
    }
}

static void track_data_recv_entry(void* parameter)
{
    struct guardian_environment *env = RT_NULL;
    rt_uint8_t* pbuf;
    rt_size_t szbuf = 0;;
    rt_err_t result;
    rt_device_t dev = RT_NULL;
    static shb_serialctrlpkt ctrlpkt;
    rt_size_t pktsz;
    rt_bool_t on_tracing = RT_FALSE;
    rt_uint8_t lost_count = 0;
    rt_uint8_t ctrl_count = 0;
    
    PID_t pid_x, pid_y;
    
    env = (struct guardian_environment*)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(TRACK_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);

    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    pid_init(&pid_x, 1.0f, 0.02f, 0.5f);    // 1.4f
    pid_init(&pid_y, 1.0f, 0.02f, 0.5f);
    
    pid_setThreshold(&pid_x, 500.0f, 200.0f, 0.02f);
    pid_setThreshold(&pid_y, 500.0f, 200.0f, 0.02f);
    
    pid_setSetpoint(&pid_x, 0.0f);
    pid_setSetpoint(&pid_y, 0.0f);
    
    pid_enable(&pid_x, RT_TRUE);
    pid_enable(&pid_y, RT_TRUE);
    
    LOG_I("recv sub-thread, start!");
    
    lost_count = 0;
    ctrl_count = 0;
    
    while (1)
    {
        result = rt_sem_take(semaph, RT_WAITING_FOREVER);
        
        if(result == -RT_ETIMEOUT)
            continue; 
        
        switch (szbuf){
        case 0:
        case 1:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (pbuf[0] != TRACK_ACK_PKT_HEADER)
                szbuf = 0;
            break;
        default:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, TRACK_ACK_PKT_SIZE - szbuf);
            break;
        }
        // should never happened.
        if (szbuf != TRACK_ACK_PKT_SIZE)
            continue;
        
        szbuf = 0;
        // check packet addr byte, shouled 0x01.
        if (pbuf[1] != TRACK_ACK_PKT_ADDR)
            continue;
        
        if ((pbuf[6] & 0x02) != 0x00)
        {
            on_tracing = RT_TRUE;
            lost_count = 0;
           
            
            float deffer_x, deffer_y;
            
            deffer_x = -(float)*(rt_int16_t*)&pbuf[2];
            deffer_y = -(float)*(rt_int16_t*)&pbuf[4];
            
            /* Enhance dynamic performance by PID control */
            
            env->trck_err_x = pid_update(&pid_x, deffer_x);
            env->trck_err_y = pid_update(&pid_y, deffer_y);
            
            LOG_D("tracing %d %d", env->trck_err_x, env->trck_err_y);
            
            if ( ctrl_count < 11)
                ctrl_count++;
            else {
                rt_sem_release(env->sh_ptz);
                ctrl_count = 0;
            }
        }
        else
        {       // cancel tracing.
            if (on_tracing == RT_FALSE)
                continue;
            
            if (lost_count < 90) {      // lost > 90 then stop tracing.
                lost_count++;
                continue;
            }
            
            on_tracing = RT_FALSE;
            
            if (env->trck_incharge == RT_TRUE)
            {
                env->trck_lost = RT_TRUE;
                
                /* stop Tracing module. */
                pktsz = sizeof(shb_serialctrlpkt);
                rt_memset(&ctrlpkt, 0x00, pktsz);

                ctrlpkt.HEADER = 0x7E7E;
                ctrlpkt.ADDR = 0x44;
                ctrlpkt.set_mode = 0x26;
                
                rt_uint8_t *ptr = (rt_uint8_t*)&ctrlpkt;
                for(int i = 0; i < pktsz - 1; i++)
                    ctrlpkt.checksum += *(ptr + i);
                
                ptr = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
                rt_memcpy(ptr, &ctrlpkt, pktsz);
                rt_mb_send(mailbox, (rt_ubase_t)ptr);
            }
            
            /* stop PanTiltZoom */
            env->trck_err_x = 0;
            env->trck_err_y = 0;
            rt_sem_release(env->sh_ptz);

            LOG_D("tracing target lost");
        }
        
    }
}

void track_resolving_entry(void* parameter)
{
    static rt_device_t dev = RT_NULL;
    static rt_uint8_t *pbuf = RT_NULL;
    static struct guardian_environment *env = RT_NULL;
    static shb_serialctrlpkt ctrlpkt; 
    rt_size_t pktsz;
    rt_thread_t pthread = RT_NULL;
    static rt_err_t result = RT_EOK;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    dev = rt_device_find(TRACK_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);

    semaph = rt_sem_create(TRACK_SEMAPHORE_RX_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);
    env->sh_track = rt_sem_create(TRACK_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(env->sh_track != RT_NULL);
    
    mempool = rt_mp_create(TRACK_SEND_MP_NAME, 16, TRACK_BUFFER_SIZE);
    RT_ASSERT(mempool != RT_NULL);
    mailbox = rt_mb_create(TRACK_SEND_MB_NAME, 16, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mailbox != RT_NULL);
    
    // set uart3 in 115200, 8E1.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.data_bits = DATA_BITS_8;
    config.parity = PARITY_NONE;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
    
    pthread = rt_thread_create("tTRCKtx", track_data_send_entry, env, 2048, 7, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);

    pthread = rt_thread_create("tTRCKrx", track_data_recv_entry, env, 2048, 8, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    LOG_I("initialization finish, start!");
    
    env->trck_incharge = RT_FALSE;

    while (1)
    {
        result = rt_sem_take(env->sh_track, RT_WAITING_FOREVER);
        
        pktsz = sizeof(shb_serialctrlpkt);
        
        rt_memset(&ctrlpkt, 0x00, pktsz);
        
        ctrlpkt.HEADER = 0x7E7E;
        ctrlpkt.ADDR = 0x44;
        
        if (env->trck_action == TRACK_ACTION_ZOOM_SHOW)
        {
			ctrlpkt.__reserved1 = 0x10;	
			ctrlpkt.set_mode = 0x83;
			ctrlpkt.set_fuction = 0x31;
            
            float  zoomf32 = 0.f;
            if (env->cam_zoom_pos < 30)
                zoomf32 = env->cam_zoom_pos + 1;                    // Optical ZOOM.
            else             
                zoomf32 = 30.0f * (env->cam_zoom_pos + 1 - 29);     // Optical ZOOM 30X combine Digital ZOOM.
            
			rt_memcpy(ctrlpkt.__reserved8 + 4, &zoomf32, 4);
            rt_memcpy(&ctrlpkt.__reserved5, &env->ptz_yaw, sizeof(float));
			rt_memcpy(ctrlpkt.__reserved8, &env->ptz_pitch, sizeof(float));           
        }
        else if (env->trck_action == TRACK_ACTION_PREPARE)
        {
            ctrlpkt.set_mode = 0x71;
            ctrlpkt.set_fuction = 0xFF;
            
            LOG_D("TRACK_ACTION_PREPAREHAL");
            
            env->trck_prepare = RT_TRUE;
        }
        else if (env->trck_action == TRACK_ACTION_TRACE_START)
        {
			ctrlpkt.set_mode = 0x71;			// 0x71 Trace Mode.
			ctrlpkt.set_fuction = 0xFE;
			ctrlpkt.start_trace = 0x01;			// 0: OFF; 1: ON.
			ctrlpkt.__reserved3 = 0x01;
			ctrlpkt.set_trace_mode = 0x38;		// medium size trace window.    // 0x3C: S,M,L     0x38: M,L    0x2C: S,M
            
            LOG_D("TRACK_ACTION_TRACE_START");
            
            env->trck_prepare = RT_FALSE;
            env->trck_incharge = RT_TRUE;
        }
        else if (env->trck_action == TRACK_ACTION_TRACE_STOP)
        {
            ctrlpkt.set_mode = 0x26;			// 0x71 Trace Mode.
            
            LOG_D("TRACK_ACTION_TRACE_STOP");
            
            env->trck_prepare = RT_FALSE;
            
            if (env->trck_incharge == RT_TRUE) {
                
                // stop moving.
                env->trck_lost = RT_TRUE;
                
                env->trck_err_x = 0;
                env->trck_err_y = 0;
                rt_sem_release(env->sh_ptz);
            }
        }
        else if (env->trck_action == TRACK_ACTION_CAPTURE)
        {
            ctrlpkt.set_mode = 0x7C;
            ctrlpkt.set_fuction = 0x02;
            
            LOG_D("TRACK_ACTION_CAPTURE");
        }
        else if (env->trck_action == TRACK_ACTION_RECORD_ON)
        {
            ctrlpkt.set_mode = 0x7C;
            ctrlpkt.set_fuction = 0x01;
            ctrlpkt.set_offset_x = 0x0258;
            
            LOG_D("TRACK_ACTION_RECORD_ON");
        }
        else if (env->trck_action == TRACK_ACTION_RECORD_OFF)
        {
            ctrlpkt.set_mode = 0x7C;
            
            LOG_D("TRACK_ACTION_RECORD_OFF");
        }
        else
            continue;
        
        pbuf = (rt_uint8_t*)&ctrlpkt;
        for(int i = 0; i < pktsz - 1; i++)
            ctrlpkt.checksum += *(pbuf + i);
        
        pbuf = rt_mp_alloc(mempool, RT_WAITING_FOREVER);
        rt_memcpy(pbuf, &ctrlpkt, pktsz);
        rt_mb_send(mailbox, (rt_ubase_t)pbuf);
        
        env->trck_action = TRACK_ACTION_NULL;
    }
    
    // never be here.
}

