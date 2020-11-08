/*
 * Copyright (c) 2006-2018, ZINGTO UAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-03     serni        first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>

#include "guardian.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME "SBUS"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#define SBUS_UARTPORT_NAME "uart4"
#define SBUS_SEMAPHORE_NAME "shSBus"

/* defined the LED pin: PB0 */
#define LED_PIN    GET_PIN(B, 0)

#define BIT_PER_CHANNEL     (11)
#define PKT_HEADER0         (0x0F)
#define PKT_HEADER1         (0x8F)
#define PKT_TAIL            (0x00)
#define PKT_TAIL0           (0x04)
#define PKT_TAIL1           (0x14)
#define PKT_TAIL2           (0x24)
#define PKT_TAIL3           (0x34)

#define HEADER_OFFSET       (00)
#define CHANNEL_OFFSET      (01)
#define FLAG_OFFSET         (23)
#define TAIL_OFFSET         (24)

#define SBUS_PKT_SIZE       (25)

#define SBUS_CH(x)         (x - 1)

static rt_sem_t semaph = RT_NULL;

static rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

void sbus_reform_value(rt_uint8_t *pbuf, rt_uint16_t *pval)
{
    rt_int8_t m = 0, n = 0;
    rt_int8_t currbit = 0, currbyte = 0;
    
    for (n = 0; n < SBUS_CHANNEL_NUMBER; n++)
    {
        pval[n] = 0;
        for (m = 0; m < BIT_PER_CHANNEL; m++)
        {
            pval[n] |= (((rt_uint16_t)((pbuf[CHANNEL_OFFSET + currbyte]) >> currbit))& 0x01)<< m;
            currbit++;
            if (currbit > 7)
            {
                currbit = 0;
                currbyte++;
            }
        }
    }
}

void sbus_resolving_entry(void* parameter)
{
    rt_device_t dev = RT_NULL;
    rt_uint8_t *pbuf = RT_NULL;
    rt_uint16_t pval[SBUS_CHANNEL_NUMBER];
    rt_size_t szbuf = 0;
    struct guardian_environment *env = RT_NULL;
    rt_err_t result = RT_EOK;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(SBUS_PKT_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(SBUS_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    semaph = rt_sem_create(SBUS_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);
    
    // set uart4 in 100000, 9E2. compatible with SBUS protocol.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = 100000;
    config.data_bits = DATA_BITS_9;
    config.stop_bits = STOP_BITS_2;
    config.parity = PARITY_EVEN;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
    
    LOG_I("initialization finish, start!");

    env->sbus_incharge = RT_FALSE;

    while (1)
    {
        
        result = rt_sem_take(semaph, RT_TICK_PER_SECOND);
        
        if(result == -RT_ETIMEOUT) {
            rt_pin_write(LED_PIN, PIN_LOW);
            continue;            
        }

        switch (szbuf){
        case 0:
        case 1:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (!((pbuf[HEADER_OFFSET] == PKT_HEADER0) || (pbuf[HEADER_OFFSET] == PKT_HEADER1)))
                szbuf = 0;
            break;
        default:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, SBUS_PKT_SIZE - szbuf);
            break;
        }
        // should nerver happened.
        if (szbuf != SBUS_PKT_SIZE)
            continue;
        
        szbuf = 0;
        // check packet tail, vaild packet.
        if (!((pbuf[TAIL_OFFSET] == PKT_TAIL) || (pbuf[TAIL_OFFSET] == PKT_TAIL0) || (pbuf[TAIL_OFFSET] == PKT_TAIL1) || (pbuf[TAIL_OFFSET] == PKT_TAIL2) || (pbuf[TAIL_OFFSET] == PKT_TAIL3)))
            continue;
        
        if (rt_pin_read(LED_PIN))
            rt_pin_write(LED_PIN, PIN_LOW);
        else
            rt_pin_write(LED_PIN, PIN_HIGH);
        
        sbus_reform_value(pbuf, pval);
        
        sbus_status tmp_status = SBUS_INVAILD;
        
        rt_bool_t ptz_request = RT_FALSE;
        rt_bool_t cam_request = RT_FALSE;
        rt_bool_t trck_request = RT_FALSE;
        
        rt_uint32_t cam_eval;
        
        for (int i = 0; i < SBUS_CHANNEL_NUMBER; i++)
        {
            switch(i) {

            case SBUS_CH(9): // calibrate
                if (pval[i] < SBUS_THRESHOLD_INVAILED)
                    tmp_status = SBUS_INVAILD;
                else if (pval[i] < SBUS_THRESHOLD_LOW)
                    tmp_status = SBUS_LOW;
                else if (pval[i] < SBUS_THRESHOLD_HIGH)
                    tmp_status = SBUS_IDLE;
                else if (pval[i] < SBUS_THRESHOLD_EXHIGH)
                    tmp_status = SBUS_HIGH;
                else
                    tmp_status = SBUS_EXHIGH;
                
                if (env->ch_status[i] != tmp_status)
                {
                    env->ch_status[i] = tmp_status;
                    
                    if (env->ch_status[i] == SBUS_EXHIGH)
                        env->sbus_cali_tick = rt_tick_get();
                }
                else
                {
                    if (env->ch_status[i] == SBUS_EXHIGH)
                    {
                        if (rt_tick_get() - env->sbus_cali_tick > RT_TICK_PER_SECOND * 5)
                        {
                            env->sbus_cali_tick = rt_tick_get();
                            
                            env->ptz_action = PANTILT_ACTION_CALIBRATE;
                            ptz_request = RT_TRUE;
                            i = SBUS_CHANNEL_NUMBER;
                        }
                    }
                }
                
                break;
            case SBUS_CH(10): // pitch
            case SBUS_CH(12): // yaw
                if (pval[i] != env->ch_value[i])
                {                    
                    env->ch_change[i] = RT_TRUE;
                    env->ch_value[i] = pval[i];
                    
                    env->ptz_action = PANTILT_ACTION_NULL;
                    ptz_request = RT_TRUE;
                }
                break;
            case SBUS_CH(11): // zoom
            {
                rt_int8_t tmpuc = 0;
                rt_int16_t differ = 0;
                
                differ = pval[i] - SBUS_VALUE_MEDIAN;

                if (abs(differ) < 100)
                    tmpuc = 0;
                else if (abs(differ) < 200)
                    tmpuc = 2;
                else if (abs(differ) < 300)
                    tmpuc = 3;
                else if (abs(differ) < 400)
                    tmpuc = 4;
                else if (abs(differ) < 500)
                    tmpuc = 5;
                else if (abs(differ) < 600)
                    tmpuc = 6;
                else 
                    tmpuc = 7;
                
                if (env->cam_zoom_speed != tmpuc)
                {
                    env->cam_zoom_speed = tmpuc;
                    
                    if (differ < -100)
                        cam_eval = CAMERA_CMD_ZOOM_OUT;
                    else if (differ > 100)
                        cam_eval = CAMERA_CMD_ZOOM_IN;
                    else
                        cam_eval = CAMERA_CMD_ZOOM_STOP;
                    
                    cam_eval |= CAMERA_CMD_ZOOM_GETPOS;
                    
                    cam_request = RT_TRUE;
                }
                else
                {
                    if ( env->cam_zoom_speed != 0)
                    {
                        if (rt_tick_get() - env->cam_getpos_tick > RT_TICK_PER_SECOND / 2)
                        {
                            cam_eval = CAMERA_CMD_ZOOM_GETPOS;
                            cam_request = RT_TRUE;
                        }
                    }
                }
            }
                break;
            
            case SBUS_CH(13): // ptz mode
                if (pval[i] < SBUS_THRESHOLD_INVAILED)
                    tmp_status = SBUS_INVAILD;
                else if (pval[i] < SBUS_THRESHOLD_LOW)
                    tmp_status = SBUS_LOW;
                else if (pval[i] < SBUS_THRESHOLD_HIGH)
                    tmp_status = SBUS_IDLE;
                else
                    tmp_status = SBUS_HIGH;
                
                if (env->ch_status[i] != tmp_status)
                {
                    env->ch_status[i] = tmp_status;
                    
                    ptz_request = RT_TRUE;
                    env->ptz_action = PANTILT_ACTION_NULL;
                    
                    if (env->ch_status[i] == SBUS_HIGH)
                        env->ptz_mode = PANTILT_MODE_HEADDOWN;
                    else if (env->ch_status[i] == SBUS_IDLE)
                        env->ptz_mode = PANTILT_MODE_HEADFREE;
                    else if (env->ch_status[i] == SBUS_LOW)
                        env->ptz_mode = PANTILT_MODE_HEADLOCK;
                    else
                        ptz_request = RT_FALSE;
                }
                break;          
            case SBUS_CH(14): // capture & record
                if (pval[i] < SBUS_THRESHOLD_INVAILED)
                    tmp_status = SBUS_INVAILD;
                else if (pval[i] < SBUS_THRESHOLD_LOW)
                    tmp_status = SBUS_LOW;
                else if (pval[i] < SBUS_THRESHOLD_HIGH)
                    tmp_status = SBUS_IDLE;
                else
                    tmp_status = SBUS_HIGH;
                
                if (env->ch_status[i] != tmp_status)
                {
                    env->ch_status[i] = tmp_status;
                    
                    if (env->ch_status[i] == SBUS_HIGH)
                    {
                        cam_eval = CAMERA_CMD_CAPTURE;
                        env->trck_action = TRACK_ACTION_CAPTURE;
                        
                        cam_request = RT_TRUE;
                        trck_request = RT_TRUE;
                    }
                    else if (env->ch_status[i] == SBUS_LOW)
                    {
                        if (env->cam_recording)
                        {
                            cam_eval = CAMERA_CMD_RECORD_OFF;
                            env->cam_recording = RT_FALSE;
                            env->trck_action = TRACK_ACTION_RECORD_OFF;
                        }
                        else
                        {
                            cam_eval = CAMERA_CMD_RECORD_ON;
                            env->cam_recording = RT_TRUE;
                            env->trck_action = TRACK_ACTION_RECORD_ON;
                            
                        }
                        
                        cam_request = RT_TRUE;
                        trck_request = RT_TRUE;
                    }
                    
                }
                break;
            case SBUS_CH(15): // tracker 
                if (pval[i] < SBUS_THRESHOLD_INVAILED)
                    tmp_status = SBUS_INVAILD;
                else if (pval[i] < SBUS_THRESHOLD_LOW)
                    tmp_status = SBUS_LOW;
                else if (pval[i] < SBUS_THRESHOLD_HIGH)
                    tmp_status = SBUS_IDLE;
                else
                    tmp_status = SBUS_HIGH;
                
                if (env->ch_status[i] != tmp_status)
                {
                    env->ch_status[i] = tmp_status;
                    
                    if (env->ch_status[i] == SBUS_HIGH)
                    {
                        if (env->trck_incharge == RT_FALSE)
                        {
                            if (env->trck_prepare == RT_FALSE)
                                env->trck_action = TRACK_ACTION_PREPARE;
                            else
                                env->trck_action = TRACK_ACTION_TRACE_START;
                            
                            trck_request = RT_TRUE;
                        }
                    }
                    else if (env->ch_status[i] == SBUS_LOW)
                    {
                        if ((env->trck_incharge == RT_TRUE) || (env->trck_prepare == RT_TRUE))
                        {
                            env->trck_action = TRACK_ACTION_TRACE_STOP;
                            trck_request = RT_TRUE;
                        }
                    }
                }
                break;
            case SBUS_CH(16): // return home pos
                if (pval[i] < SBUS_THRESHOLD_INVAILED)
                    tmp_status = SBUS_INVAILD;
                else if (pval[i] < SBUS_THRESHOLD_LOW)
                    tmp_status = SBUS_LOW;
                else if (pval[i] < SBUS_THRESHOLD_HIGH)
                    tmp_status = SBUS_IDLE;
                else
                    tmp_status = SBUS_HIGH;
                
                if (env->ch_status[i] != tmp_status)
                {
                    env->ch_status[i] = tmp_status;
                    
                    if (env->ch_status[i] == SBUS_HIGH)
                    {
                        env->ptz_action = PANTILT_ACTION_HOMING;
                        ptz_request = RT_TRUE;
                    }
                }
                break;
            case SBUS_CH(8): // pip mode & ir color
                if (pval[i] < SBUS_THRESHOLD_INVAILED)
                    tmp_status = SBUS_INVAILD;
                else if (pval[i] < SBUS_THRESHOLD_LOW)
                    tmp_status = SBUS_LOW;
                else if (pval[i] < SBUS_THRESHOLD_HIGH)
                    tmp_status = SBUS_IDLE;
                else
                    tmp_status = SBUS_HIGH;
                
                if (env->ch_status[i] != tmp_status)
                {
                    env->ch_status[i] = tmp_status;
                    
                    if (env->ch_status[i] == SBUS_LOW)
                    {
                        if (env->cam_blankvideo == RT_FALSE){
                                                    
                            env->cam_pip_mode++;
                            if (env->cam_pip_mode > 4)
                                env->cam_pip_mode = 1;
                            
                            if (env->cam_pip_mode == 1)
                                cam_eval = CAMERA_CMD_PIP_MODE1;
                            else if (env->cam_pip_mode == 2)
                                cam_eval = CAMERA_CMD_PIP_MODE2;
                            else if (env->cam_pip_mode == 3)
                                cam_eval = CAMERA_CMD_PIP_MODE3;
                            else
                                cam_eval = CAMERA_CMD_PIP_MODE4;

                            cam_request = RT_TRUE;
                        }
                    }
                    else if (env->ch_status[i] == SBUS_HIGH)
                    {
                        env->irs_color++;
                        if (env->irs_color > 3)
                            env->irs_color = 0;
                        
                        env->ptz_action = PANTILT_ACTION_IRCOLOR;

                        ptz_request = RT_TRUE;
                    }
                }
                break;
            case SBUS_CH(7): // IR sensor gain level
                if (pval[i] < SBUS_THRESHOLD_INVAILED)
                    tmp_status = SBUS_INVAILD;
                else if (pval[i] < SBUS_THRESHOLD_LOW)
                    tmp_status = SBUS_LOW;
                else if (pval[i] < SBUS_THRESHOLD_HIGH)
                    tmp_status = SBUS_IDLE;
                else
                    tmp_status = SBUS_HIGH;
                
                if (env->ch_status[i] != tmp_status)
                {
                    env->ch_status[i] = tmp_status;
                    
                    if (env->ch_status[i] == SBUS_HIGH)
                    {
                        env->ptz_action = PANTILT_ACTION_SET_HIGHGAIN;
                        ptz_request = RT_TRUE;

                    }
                    else if (env->ch_status[i] == SBUS_LOW)
                    {
                        env->ptz_action = PANTILT_ACTION_SET_LOWGAIN;
                        ptz_request = RT_TRUE;
                    }
                }
                break;
            case SBUS_CH(6): // IR zoom
                if (pval[i] < SBUS_THRESHOLD_INVAILED)
                    tmp_status = SBUS_INVAILD;
                else if (pval[i] < SBUS_THRESHOLD_LOW)
                    tmp_status = SBUS_LOW;
                else if (pval[i] < SBUS_THRESHOLD_HIGH)
                    tmp_status = SBUS_IDLE;
                else
                    tmp_status = SBUS_HIGH;
                
                if (env->ch_status[i] != tmp_status)
                {
                    env->ch_status[i] = tmp_status;
                    
                    if (env->ch_status[i] == SBUS_HIGH)
                    {
                        env->irs_zoom++;
                        if (env->irs_zoom > 4)
                            env->irs_zoom = 0;
                        
                        env->ptz_action = PANTILT_ACTION_IRZOOM;
                        ptz_request = RT_TRUE;
                    }
                }
                break;

            default:
                break;
            }
        }
        
        if (ptz_request == RT_TRUE)
        {
            ptz_request = RT_FALSE;
            if (env->sh_ptz != RT_NULL)
            {
                env->sbus_incharge = RT_TRUE;
                rt_sem_release(env->sh_ptz); // notify the PanTiltZoom.
            }
        }
        else {
            env->sbus_incharge = RT_FALSE;
        }
        
        if (trck_request == RT_TRUE)
        {
            trck_request = RT_FALSE;
            if (env->sh_track != RT_NULL)
                rt_sem_release(env->sh_track);
        }
        
        if (cam_request == RT_TRUE)
        {
            cam_request = RT_FALSE;
            if (env->ev_camera != RT_NULL)
                rt_event_send(env->ev_camera, cam_eval); // notify the Camera.
        }
            
        
        // next sbus packet.
    }
    // never be here.
}

