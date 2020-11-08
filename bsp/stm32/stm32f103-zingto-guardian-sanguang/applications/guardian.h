/*
 * Copyright (c) 2006-2018, ZINGTO UAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-04     serni        first version
 */

#ifndef __GUARDIAN_H__
#define __GUARDIAN_H__

#include <rtthread.h>

/* SBUS kernel function thread */
typedef enum
{
    SBUS_INVAILD = 0,
    SBUS_IDLE,
    SBUS_LOW,
    SBUS_HIGH,
    SBUS_EXHIGH,
}sbus_status;

void sbus_resolving_entry(void* parameter);

#define SBUS_CHANNEL_NUMBER         (16)

#define SBUS_THRESHOLD_EXHIGH       (1600)
#define SBUS_THRESHOLD_HIGH         (1200)
#define SBUS_THRESHOLD_LOW          (800)
#define SBUS_THRESHOLD_INVAILED     (50)

#define SBUS_VALUE_MEDIAN           (1000)
#define SBUS_VALUE_IGNORE           (50)
#define SBUS_VALUE_MAXIMUM          (1700)
#define SBUS_VALUE_MININUM          (300)

/* camera kernel function thread */
void camera_resolving_entry(void* parameter);

#define CAMERA_CMD_MASK             ( 0x0000FFFF )
#define CAMERA_CMD_CAPTURE          ( 0x00000001 )
#define CAMERA_CMD_RECORD_ON        ( 0x00000002 )
#define CAMERA_CMD_RECORD_OFF       ( 0x00000004 )
#define CAMERA_CMD_PIP_MODE1        ( 0x00000008 )
#define CAMERA_CMD_PIP_MODE2        ( 0x00000010 )
#define CAMERA_CMD_PIP_MODE3        ( 0x00000020 )
#define CAMERA_CMD_PIP_MODE4        ( 0x00000040 )

#define CAMERA_CMD_ZOOM_IN          ( 0x00000080 )
#define CAMERA_CMD_ZOOM_OUT         ( 0x00000100 )
#define CAMERA_CMD_ZOOM_STOP        ( 0x00000200 )
#define CAMERA_CMD_ZOOM_GETPOS      ( 0x00000400 )

/* pantilt kernel function thread */
void pantilt_resolving_entry(void* parameter);

#define PANTILT_VALUE_IGNORE            (20)

#define PANTILT_ACTION_NULL             (0x00)
#define PANTILT_ACTION_HOMING           (0x01)
#define PANTILT_ACTION_HEADDOWN         (0x02)
#define PANTILT_ACTION_HEADLOCK         (0x03)
#define PANTILT_ACTION_HEADFREE         (0x04)
#define PANTILT_ACTION_IRCOLOR          (0x05)
#define PANTILT_ACTION_IRZOOM           (0x06)
#define PANTILT_ACTION_CALIBRATE        (0x07)
#define PANTILT_ACTION_ASK     			    (0x08)

#define PANTILT_MODE_HEADDOWN      	    (0x02)
#define PANTILT_MODE_HEADLOCK      	    (0x03)
#define PANTILT_MODE_HEADFREE      	    (0x04)

/* track kernel function thread */
void track_resolving_entry(void* parameter);

#define TRACK_ACTION_NULL               (0x00)
#define TRACK_ACTION_PREPARE            (0x01)
#define TRACK_ACTION_TRACE_START        (0x03)
#define TRACK_ACTION_TRACE_STOP         (0x04)
#define TRACK_ACTION_RECORD_ON          (0x05)
#define TRACK_ACTION_RECORD_OFF         (0x06)
#define TRACK_ACTION_SNAP               (0x07)

/* zingto kernel function thread */
void zingto_resolving_entry(void* parameter);

/* environment struct */
struct guardian_environment
{
    // SBUS Radio
    rt_bool_t       sbus_incharge;
    rt_bool_t       ch_change[SBUS_CHANNEL_NUMBER];
    sbus_status     ch_status[SBUS_CHANNEL_NUMBER];
    rt_uint16_t     ch_value[SBUS_CHANNEL_NUMBER];
    rt_tick_t       sbus_cali_tick;
    
    // Camera
    rt_uint8_t      cam_zoom_speed;
    rt_uint16_t     cam_zoom_pos;
    rt_tick_t       cam_getpos_tick;
    rt_uint8_t      cam_pip_mode;
    rt_bool_t       cam_recording;
    rt_event_t      ev_camera;
    
    // PanTilt
    rt_uint8_t      ptz_action;
    rt_uint8_t      irs_color;
    rt_uint8_t      irs_zoom;
		rt_uint8_t      ptz_mode;
    rt_sem_t        sh_ptz;
    
    // User
    rt_bool_t       user_incharge;
    
    // Track
    rt_bool_t       trck_incharge;
    rt_bool_t       trck_lost;
    rt_bool_t       trck_prepare;
    rt_int16_t      trck_err_x;
    rt_int16_t      trck_err_y;
    rt_uint8_t      trck_action;
    rt_sem_t        sh_track;
};


#endif
