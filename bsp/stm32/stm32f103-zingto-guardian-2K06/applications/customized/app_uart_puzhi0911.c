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

#define DBG_ENABLE
#define DBG_SECTION_NAME "UART"
#define DBG_LEVEL DBG_LOG               // DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#define ZINGTO_UARTPORT_NAME   "uart5"
#define ZINGTO_SEMAPHORE_NAME  "shUart"


#define ZINGTO_BUFFER_SIZE     (16)
#define ZINGTO_RX_TIMEOUT      (10)

#define ZINGTO_PKT_HEADER0     (0xE1)
#define ZINGTO_PKT_HEADER1     (0x1E)
#define ZINGTO_PKT_TAIL0       (0xF1)
#define ZINGTO_PKT_TAIL1       (0x1F)

#define ZINGTO_PKT_SIZE   (5)

/* defined the LED pin: PB0 */
#define LED_PIN    GET_PIN(B, 0)

static rt_sem_t semaph = RT_NULL;

static rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

void zingto_resolving_entry(void* parameter)
{
    rt_device_t dev = RT_NULL;
    struct guardian_environment *env = RT_NULL;
    rt_err_t result = RT_EOK;
    rt_uint8_t *pbuf = RT_NULL;
    rt_size_t szbuf = 0;
    rt_uint8_t opcode, speedlv;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(ZINGTO_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(ZINGTO_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    semaph = rt_sem_create(ZINGTO_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);
    
    // set uart in 115200, 8N1.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
    
    LOG_I("initialization finish, start!");

    while (1)
    {
        result = rt_sem_take(semaph, RT_TICK_PER_SECOND / 100);
        
        if(result == -RT_ETIMEOUT) {
            rt_pin_write(LED_PIN, PIN_LOW);
            continue;
        }
        
        rt_pin_write(LED_PIN, PIN_HIGH);
            
        switch (szbuf){
        case 0:
        case 1:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (pbuf[0] != ZINGTO_PKT_HEADER0)
                szbuf = 0;
            break;
        default:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, ZINGTO_PKT_SIZE - szbuf);
            break;
        }
        // should nerver happened
        if (szbuf != ZINGTO_PKT_SIZE)
            continue;
        
        szbuf = 0;
        
        // check packet tail, vaild packet.
        if ((pbuf[1] != ZINGTO_PKT_HEADER1) || (pbuf[3] != ZINGTO_PKT_TAIL0) || (pbuf[4] != ZINGTO_PKT_TAIL1))
            continue;
        
        opcode = pbuf[2];
        LOG_D("opcode %02X", opcode);
        
        speedlv = (opcode & 0xE0) >> 5;     // get speed.
        opcode = opcode & 0x1F;             // clear speed bits.
        
        rt_bool_t ptz_request = RT_FALSE;
        rt_bool_t cam_request = RT_FALSE;
        rt_bool_t trck_request = RT_FALSE;
        rt_uint32_t cam_eval;
        
        switch(opcode) {
        case 0x00:  // stop
            //env->ch_value[0] = SBUS_VALUE_MEDIAN;
            env->ch_value[1] = SBUS_VALUE_MEDIAN;
            env->ch_value[3] = SBUS_VALUE_MEDIAN;
            env->ptz_action = PANTILT_ACTION_NULL;
            ptz_request = RT_TRUE;
            break;
//        case 0x0F:  // roll -
//            env->ch_value[0] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MAXIMUM - SBUS_VALUE_MEDIAN) * speedlv / 10.f;
//            env->ptz_action = PANTILT_ACTION_NULL;
//            ptz_request = RT_TRUE;
//            break;
//        case 0x10:  // roll +
//            env->ch_value[0] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MININUM - SBUS_VALUE_MEDIAN) * speedlv / 10.f;
//            env->ptz_action = PANTILT_ACTION_NULL;
//            ptz_request = RT_TRUE;
//            break;
        case 0x01:  // pitch -
            env->ch_value[1] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MAXIMUM - SBUS_VALUE_MEDIAN) * speedlv / 10.f;
            env->ptz_action = PANTILT_ACTION_NULL;
            ptz_request = RT_TRUE;
            break;
        case 0x02:  // pitch +
            env->ch_value[1] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MININUM - SBUS_VALUE_MEDIAN) * speedlv / 10.f;
            env->ptz_action = PANTILT_ACTION_NULL;
            ptz_request = RT_TRUE;
            break;
        case 0x03:  // yaw -
            env->ch_value[3] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MAXIMUM - SBUS_VALUE_MEDIAN) * speedlv / 10.f;
            env->ptz_action = PANTILT_ACTION_NULL;
            ptz_request = RT_TRUE;
            break;
        case 0x04:  // yaw +
            env->ch_value[3] = SBUS_VALUE_MEDIAN + (SBUS_VALUE_MININUM - SBUS_VALUE_MEDIAN) * speedlv / 10.f;
            env->ptz_action = PANTILT_ACTION_NULL;
            ptz_request = RT_TRUE;
            break;
        case 0x05:  // zoom out
            if (speedlv < 2) 
                env->cam_zoom_speed = 2;
            else
                env->cam_zoom_speed = speedlv;
            
            cam_eval = CAMERA_CMD_ZOOM_OUT;
            cam_eval |= CAMERA_CMD_ZOOM_GETPOS;
            cam_request = RT_TRUE;
            break;
        case 0x06:  // zoom in
            if (speedlv < 2) 
                env->cam_zoom_speed = 2;
            else
                env->cam_zoom_speed = speedlv;
            
            cam_eval = CAMERA_CMD_ZOOM_IN;
            cam_eval |= CAMERA_CMD_ZOOM_GETPOS;
            cam_request = RT_TRUE;
            break;
        case 0x07:  // zoom stop
            env->cam_zoom_speed = 0;
            cam_eval = CAMERA_CMD_ZOOM_STOP;
            cam_eval |= CAMERA_CMD_ZOOM_GETPOS;
            cam_request = RT_TRUE;
            break;
        case 0x08:  // head free.
            env->ptz_action = PANTILT_ACTION_NULL;
            env->ptz_mode = PANTILT_MODE_HEADFREE;
            ptz_request = RT_TRUE;
            break;
        case 0x09:  // head lock.
            env->ptz_action = PANTILT_ACTION_NULL;
            env->ptz_mode = PANTILT_MODE_HEADLOCK;
            ptz_request = RT_TRUE;
            break;
        case 0x0A:  // head down.
            env->ptz_action = PANTILT_ACTION_NULL;
            env->ptz_mode = PANTILT_MODE_HEADDOWN;
            ptz_request = RT_TRUE;
            break;
        case 0x0B:  // homing.
            env->ptz_action = PANTILT_ACTION_HOMING;
            ptz_request = RT_TRUE;
            break;
        case 0x0C:  // record on
            cam_eval = CAMERA_CMD_RECORD_ON;
            cam_request = RT_TRUE;
            env->trck_action = TRACK_ACTION_RECORD_ON;
            trck_request = RT_TRUE;
        
            env->cam_recording = RT_TRUE;
            break;
        case 0x0D:  // record off
            cam_eval = CAMERA_CMD_RECORD_OFF;
            cam_request = RT_TRUE;
            env->trck_action = TRACK_ACTION_RECORD_OFF;
            trck_request = RT_TRUE;
        
            env->cam_recording = RT_FALSE;
            break;
        case 0x0E:  // capture
            cam_eval = CAMERA_CMD_CAPTURE;
            cam_request = RT_TRUE;
            env->trck_action = TRACK_ACTION_CAPTURE;
            trck_request = RT_TRUE;
        
            break;
        case 0x11:
            LOG_W("calibrate gyro temp");
            env->ptz_action = PANTILT_ACTION_CALIBRATE;
            ptz_request = RT_TRUE;  
            break;
        case 0x13:  // pip mode 
            if (speedlv == 0) {
                cam_eval = CAMERA_CMD_PIP_MODE3;
                env->cam_pip_mode = 3;
            }
            else if (speedlv == 1) {
                cam_eval = CAMERA_CMD_PIP_MODE4;
                env->cam_pip_mode = 4;
            }
            else if (speedlv == 2) {
                cam_eval = CAMERA_CMD_PIP_MODE1;
                env->cam_pip_mode = 1;
            }
            else {
                cam_eval = CAMERA_CMD_PIP_MODE2;
                env->cam_pip_mode = 2;
            }
            
            cam_request = RT_TRUE;
            break;
        case 0x14:  // track prepare.
            env->trck_action = TRACK_ACTION_PREPARE;
            trck_request = RT_TRUE;
            break;
        case 0x15:  // track start.
            env->trck_action = TRACK_ACTION_TRACE_START;
            trck_request = RT_TRUE;
            break;
        case 0x16:  // track stop.
            env->trck_action = TRACK_ACTION_TRACE_STOP;
            trck_request = RT_TRUE;
            break;
        case 0x17:  // color mode 
            env->ptz_action = PANTILT_ACTION_IRCOLOR;
            env->irs_color = speedlv;
            ptz_request = RT_TRUE;
            break;
        case 0x18:  // ir zoom 
            env->ptz_action = PANTILT_ACTION_IRZOOM;
            env->irs_zoom = speedlv;
            ptz_request = RT_TRUE;
            break;
        case 0x19:  // video blank control
            if (speedlv == 0) {
                if (env->cam_blankvideo == RT_FALSE) {
                    cam_eval = CAMERA_CMD_PIP_MODE5;
                    env->cam_blankvideo = RT_TRUE;
                    cam_request = RT_TRUE;
                }
            }
            else
            {
                if (env->cam_blankvideo == RT_TRUE) {
                    if (env->cam_pip_mode == 1)
                        cam_eval = CAMERA_CMD_PIP_MODE1;
                    else if (env->cam_pip_mode == 2)
                        cam_eval = CAMERA_CMD_PIP_MODE2;
                    else if (env->cam_pip_mode == 3)
                        cam_eval = CAMERA_CMD_PIP_MODE3;
                    else
                        cam_eval = CAMERA_CMD_PIP_MODE4;
                    
                    env->cam_blankvideo = RT_FALSE;
                    
                    cam_request = RT_TRUE;
                }           
            }
            break;
        case 0x1A:  // tf logo control
            if (speedlv == 0) {
                if (env->cam_tflogo == RT_FALSE) {
                    cam_eval = CAMERA_CMD_TFLOGO_ON;
                    env->cam_tflogo = RT_TRUE;
                    cam_request = RT_TRUE;
                }
            }
            else
            {
                if (env->cam_tflogo == RT_TRUE) {
                    cam_eval = CAMERA_CMD_TFLOGO_OFF;
                    env->cam_tflogo = RT_FALSE;
                    cam_request = RT_TRUE;
                }
            }            
            break;
        default:
            LOG_W("unknown opcode, %02X", opcode);
            break;
        }
        
        //
        if (ptz_request == RT_TRUE)
        {
            ptz_request = RT_FALSE;
            if (env->sh_ptz != RT_NULL)
            {
                env->user_incharge = RT_TRUE;
                rt_sem_release(env->sh_ptz);    // notify the PanTiltZoom.
            }
        }
        else {
            env->user_incharge = RT_FALSE;
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
        
    }
    
    // never be here.
}

