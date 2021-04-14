/*
 * Copyright (c) 2006-2021, ZINGTO UAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-22     zhouzheng    first version
 */

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG   //DBG_INFO
#define DBG_SECTION_NAME  "SBGC"
#define DBG_COLOR
#include <rtdbg.h>

#include <math.h>
#include <dynacall.h>
#include <drv_sbus.h>
#include "SBGC32.h"

#define ROTATION_SPEED_MAX  35.0f
#define ANGLE_LIMIT_ROLL    60.0f

#define ROTATION_SPEED_LSB  0.1220740379f
#define POSITION_ANGLE_LSB  0.02197265625f
#define SBGC_TXRX_HEADER    0x3E
#define SBGC_REALTIME_RSPLEN 129

#define SBGC_DEVICE_UART    "uart2"
#define SBGC_MUTEX_TOKEN    "mtx_SBGC_Token"
#define SBGC_MUTEX_UDEV     "mtx_SBGC_Udev"

#define SBGC_TXBUFF_SIZE    64
static rt_uint8_t txbuf[SBGC_TXBUFF_SIZE];

#define SBGC_RXBUFF_SIZE    256
static rt_uint8_t rxbuf[SBGC_RXBUFF_SIZE];

#define FLOAT2INT16(value, unit)    ((rt_int16_t)(value/unit))      

static rt_mutex_t mtx_token = RT_NULL;
static rt_mutex_t mtx_udev = RT_NULL;
static rt_device_t dev_uart = RT_NULL;

static float rotation_speed_coeff = 1.0f;
static float rotation_value[3] = {0.0f, 0.0f, 0.0f};
static SBGC32_CtrlMode_t rotation_mode[3] = {CTRLMODE_ANGLE_REF_FRAME, CTRLMODE_SPEED, CTRLMODE_SPEED};

static int SBGC_Init(void)
{
    mtx_token = rt_mutex_create(SBGC_MUTEX_TOKEN, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mtx_token != RT_NULL);
    
    mtx_udev = rt_mutex_create(SBGC_MUTEX_UDEV, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mtx_udev != RT_NULL);
    
    dev_uart = rt_device_find(SBGC_DEVICE_UART);
    RT_ASSERT(dev_uart != RT_NULL);
    rt_device_open(dev_uart, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
    
    LOG_I("SBGC Runtime Initial Done.");
    
    return 0;
}
INIT_ENV_EXPORT(SBGC_Init);

void SBGC_SetRotationCoeff(float new_coeff) {
    rotation_speed_coeff = new_coeff;
}

static rt_size_t SBGC_Malloc(SBGC32_CmdID_t cmd)
{
    rt_size_t actual_size, payload_size;
    
    switch(cmd) {
        case CMD_CONTROL:       payload_size = 15; break;
        case CMD_PROFILE_SET:   payload_size = 10; break;
        case CMD_EXECUTE_MENU:  payload_size = 1; break;
        case CMD_CALIB_ORIENT_CORR: payload_size = 16; break;
        case CMD_REALTIME_DATA_4: payload_size = 0; break;
        default:
            LOG_W("CMD %d not support yet!", cmd); return RT_NULL;
    }
    
    actual_size = payload_size + 5;  // bufsz = payload + header_sz(5)
    rt_memset(txbuf, 0, actual_size);
    txbuf[0] = SBGC_TXRX_HEADER;
    txbuf[1] = cmd;
    txbuf[2] = payload_size;
    txbuf[3] = txbuf[1] + txbuf[2];
    
    return actual_size;
}

static inline void SBGC_CalculateCheckSum(rt_size_t bufsz)
{
    for(int i = 4; i < bufsz - 1; i++)
        txbuf[bufsz - 1] += txbuf[i];
}

static inline void SBGC_ClearRxBuffer(void)
{
    rt_size_t count;
    
    do {
        count = rt_device_read(dev_uart, 0, rxbuf, SBGC_RXBUFF_SIZE);
    }while(count != 0);
}

static inline int SBGC_ImmediateWrite(rt_size_t size) {
    return rt_device_write(dev_uart, 0, txbuf, size);
}

static inline int SBGC_BlockRead(rt_size_t expect_size, rt_tick_t timeout)
{
    rt_size_t count;
    rt_tick_t start;
    
    if (expect_size > SBGC_RXBUFF_SIZE) return -RT_ENOMEM;
    
    count = 0;
    start = rt_tick_get();
    while(RT_TRUE){
        rt_thread_delay(1);
        count += rt_device_read(dev_uart, 0, rxbuf + count, expect_size - count);
        if (count == expect_size) break;
        if ((rt_tick_get() - start) > timeout) return -RT_ETIMEOUT;
    }
    
    return RT_EOK;
}

int SBGC_SetMoveValue(SBGC32_Axis_t axis, float val)
{
    if ( !((axis == Axis_All) || (axis == Axis_Yaw) || \
            (axis == Axis_Pitch) || (axis == Axis_Roll)) )
        return -RT_EINVAL;

    if (axis == Axis_All) {
        rotation_value[Axis_Roll] = val;
        rotation_value[Axis_Pitch] = val;
        rotation_value[Axis_Yaw] = val;
    }
    else {
        rotation_value[axis] = val;
    }
    
    return RT_EOK;
}

int SBGC_SetMoveCtrlMode(SBGC32_Axis_t axis, SBGC32_CtrlMode_t mode)
{
    if ( !((axis == Axis_All) || (axis == Axis_Yaw) || \
            (axis == Axis_Pitch) || (axis == Axis_Roll)) )
        return -RT_EINVAL;
    
    if (axis == Axis_All) {
        rotation_mode[Axis_Roll] = mode;
        rotation_mode[Axis_Pitch] = mode;
        rotation_mode[Axis_Yaw] = mode;
    }
    else {
        rotation_mode[axis] = mode;
    }
    
    return RT_EOK;
}

int SBGC_MoveGimbal(int cooldown)
{
    static rt_tick_t prev_tick, cd_period;
    
    cd_period = rt_tick_get() - prev_tick;
    
    if (cd_period < cooldown) return -RT_EBUSY;
    
    prev_tick += cd_period;   // update the tick record.
    
    rt_mutex_take(mtx_udev, RT_WAITING_FOREVER);    
    
    rt_size_t bufsz = SBGC_Malloc(CMD_CONTROL);
    
    txbuf[4] = rotation_mode[Axis_Roll];
    txbuf[5] = rotation_mode[Axis_Pitch];
    txbuf[6] = rotation_mode[Axis_Yaw];
    
    if (rotation_mode[Axis_Roll] == CTRLMODE_SPEED) {
        *(rt_int16_t*)&txbuf[7] = FLOAT2INT16(rotation_value[Axis_Roll], ROTATION_SPEED_LSB) * rotation_speed_coeff;
        *(rt_int16_t*)&txbuf[9] = 0;
    }
    else if (rotation_mode[Axis_Roll] == CTRLMODE_ANGLE_REF_FRAME) {
        *(rt_int16_t*)&txbuf[7] = 0;
        *(rt_int16_t*)&txbuf[9] = FLOAT2INT16(rotation_value[Axis_Roll], POSITION_ANGLE_LSB);
    }
    
    if (rotation_mode[Axis_Pitch] == CTRLMODE_SPEED) {
        *(rt_int16_t*)&txbuf[11] = FLOAT2INT16(rotation_value[Axis_Pitch], ROTATION_SPEED_LSB) * rotation_speed_coeff;
        *(rt_int16_t*)&txbuf[13] = 0;
    }
    else if (rotation_mode[Axis_Pitch] == CTRLMODE_ANGLE_REF_FRAME) {
        *(rt_int16_t*)&txbuf[11] = 0;
        *(rt_int16_t*)&txbuf[13] = FLOAT2INT16(rotation_value[Axis_Pitch], POSITION_ANGLE_LSB);
    }

    if (rotation_mode[Axis_Yaw] == CTRLMODE_SPEED) {
        *(rt_int16_t*)&txbuf[15] = FLOAT2INT16(rotation_value[Axis_Yaw], ROTATION_SPEED_LSB) * rotation_speed_coeff;
        *(rt_int16_t*)&txbuf[17] = 0;
    }
    else if (rotation_mode[Axis_Yaw] == CTRLMODE_ANGLE_REF_FRAME) {
        *(rt_int16_t*)&txbuf[15] = 0;
        *(rt_int16_t*)&txbuf[17] = FLOAT2INT16(rotation_value[Axis_Yaw], POSITION_ANGLE_LSB);
    }

    SBGC_CalculateCheckSum(bufsz);
    SBGC_ImmediateWrite(bufsz);
    
    if ( (fabsf(rotation_value[Axis_Pitch]) < 0.05f) && (fabsf(rotation_value[Axis_Yaw]) < 0.05f) ) {
        if ((rotation_mode[Axis_Pitch] == CTRLMODE_SPEED) && (rotation_mode[Axis_Yaw] == CTRLMODE_SPEED)) {
            
            rt_thread_delay(5);
              
            bufsz = SBGC_Malloc(CMD_CONTROL);   // CTRLMODE_NO_CONTROL = 0
            
            if (rotation_mode[Axis_Roll] == CTRLMODE_ANGLE_REF_FRAME) {
                txbuf[4] = CTRLMODE_ANGLE_REF_FRAME;
                *(rt_int16_t*)&txbuf[9] = FLOAT2INT16(rotation_value[Axis_Roll], POSITION_ANGLE_LSB);
            }
            if (rotation_mode[Axis_Pitch] == CTRLMODE_ANGLE_REF_FRAME) {
                txbuf[5] = CTRLMODE_ANGLE_REF_FRAME;
                *(rt_int16_t*)&txbuf[13] = FLOAT2INT16(rotation_value[Axis_Pitch], POSITION_ANGLE_LSB);
            }
            if (rotation_mode[Axis_Yaw] == CTRLMODE_ANGLE_REF_FRAME) {
                txbuf[6] = CTRLMODE_ANGLE_REF_FRAME;
                *(rt_int16_t*)&txbuf[17] = FLOAT2INT16(rotation_value[Axis_Yaw], POSITION_ANGLE_LSB);
            }
            
            SBGC_CalculateCheckSum(bufsz);
            SBGC_ImmediateWrite(bufsz);
        }
    }
    
    rt_mutex_release(mtx_udev);

    return RT_EOK;
}

int SBGC_QueryRefAngle(float *ref_angle, int timeout)
{
    int retval;
    
    rt_mutex_take(mtx_udev, RT_WAITING_FOREVER);
    
    rt_size_t bufsz = SBGC_Malloc(CMD_REALTIME_DATA_4);
    SBGC_CalculateCheckSum(bufsz);
    SBGC_ClearRxBuffer();
    SBGC_ImmediateWrite(bufsz);

    retval = SBGC_BlockRead(SBGC_REALTIME_RSPLEN, timeout);
    if (retval == RT_EOK) {
        // 67 roll 69 pitch 71 yaw
        rt_int16_t tmp = 0;
        float tmpf = 0.f;
        
        tmp = *(int16_t *)&rxbuf[71];
		tmpf = tmp * POSITION_ANGLE_LSB;
		tmpf = tmpf - floor(tmpf / 360.0f) * 360.0f;
		if ( tmpf > 180.0f ) tmpf = tmpf - 360.0f; // range -180 and 180 degree.
        ref_angle[Axis_Yaw] = tmpf;

        tmp = *(int16_t *)&rxbuf[69];
		tmpf = tmp * POSITION_ANGLE_LSB;
        tmpf = tmpf - floor(tmpf / 360.0f) * 360.0f;
		if ( tmpf > 180.0f ) tmpf = tmpf - 360.0f;
        tmpf = tmpf * -1.0f;
        ref_angle[Axis_Pitch] = tmpf;

        tmp = *(int16_t *)&rxbuf[67];
		tmpf = tmp * POSITION_ANGLE_LSB;
        tmpf = tmpf - floor(tmpf / 360.0f) * 360.0f;
		if ( tmpf > 180.0f ) tmpf = tmpf - 360.0f;
        tmpf = tmpf * -1.0f;
        ref_angle[Axis_Roll] = tmpf;
    }
    
    rt_mutex_release(mtx_udev);
    
    return retval;
}

#include <stdio.h>
static void gimbal_pos(int argc, char **argv)
{
    float angle[3];
    char cbuf[128];
    rt_err_t retval;
    
    retval = SBGC_QueryRefAngle(angle, 50);
    if (retval == RT_EOK) {
        rt_memset(cbuf, 0, 128);
        snprintf(cbuf, 128, "%3.2f %3.2f %3.2f", angle[Axis_Roll], angle[Axis_Pitch], angle[Axis_Yaw]);
        rt_kprintf("%s\n", cbuf);
    }
    else {
        rt_kprintf("Read Gimbal-angle Failed!\n");
    }
}
MSH_CMD_EXPORT(gimbal_pos, read the 3-axis Gimbal-angle.)

static int SBGC_ExecMenuCommand(SBGC32_MenuCmd_t menu)
{
    rt_mutex_take(mtx_udev, RT_WAITING_FOREVER);
    
    rt_size_t bufsz = SBGC_Malloc(CMD_EXECUTE_MENU);
    
    txbuf[4] = menu;
    SBGC_CalculateCheckSum(bufsz);
    
    SBGC_ImmediateWrite(bufsz);
    
    rt_mutex_release(mtx_udev);
    return RT_EOK;
}

// dynacall functions.
static int SBGC_setRollSpeed(void *parameter)
{
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    
    //LOG_D("SBGC_setRollSpeed(%d)", val);
    SBGC_SetMoveCtrlMode(Axis_Roll, CTRLMODE_SPEED);
    SBGC_SetMoveValue(Axis_Roll, sbus_normalize_value(val) * ROTATION_SPEED_MAX);
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_setRollSpeed)

static int SBGC_setRollRefAngle(void *parameter)
{
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    
    //LOG_D("SBGC_setRollPosition(%d)", val);
    SBGC_SetMoveCtrlMode(Axis_Roll, CTRLMODE_ANGLE_REF_FRAME);
    SBGC_SetMoveValue(Axis_Roll, sbus_normalize_value(val) * ANGLE_LIMIT_ROLL);
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_setRollRefAngle)

static int SBGC_setPitchSpeed(void *parameter)
{
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    
    //LOG_D("SBGC_setPitchSpeed(%d)", val);
    SBGC_SetMoveCtrlMode(Axis_Pitch, CTRLMODE_SPEED);
    SBGC_SetMoveValue(Axis_Pitch, sbus_normalize_value(val) * ROTATION_SPEED_MAX);
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_setPitchSpeed)

static int SBGC_setYawSpeed(void *parameter)
{
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    
    //LOG_D("SBGC_setYawSpeed(%d)", val);
    SBGC_SetMoveCtrlMode(Axis_Yaw, CTRLMODE_SPEED);
    SBGC_SetMoveValue(Axis_Yaw, sbus_normalize_value(val) * ROTATION_SPEED_MAX);
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_setYawSpeed)

static int SBGC_execMenuLookdown(void *parameter) {
    return SBGC_ExecMenuCommand(MENU_CMD_LOOK_DOWN);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_execMenuLookdown)

static int SBGC_execMenuHeadfree(void *parameter) {
    return SBGC_ExecMenuCommand(MENU_CMD_PROFILE2);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_execMenuHeadfree)

static int SBGC_execMenuHeadlock(void *parameter) {
    return SBGC_ExecMenuCommand(MENU_CMD_PROFILE1);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_execMenuHeadlock)

static int SBGC_execMenuHomepos(void *parameter) {
    return SBGC_ExecMenuCommand(MENU_HOME_POSITION_SHORTEST);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_execMenuHomepos)

static int SBGC_dumpChannelValue(void *parameter) {
    LOG_D("SBGC_dumpChannelValue: %d", *(rt_uint16_t*)parameter);
    return RT_EOK;
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_dumpChannelValue)

static int SBGC_cmdStopAll(void *parameter){
    //LOG_D("SBGC_cmdStopAll(%d)", val);
    SBGC_SetMoveCtrlMode(Axis_All, CTRLMODE_SPEED);
    SBGC_SetMoveValue(Axis_All, 0.0f);
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_cmdStopAll)

static int SBGC_cmdPitchUp(void *parameter){
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    rt_uint8_t param2 = val & 0x00FF;
    //LOG_D("SBGC_cmdStopAll(%d)", val);
    
    SBGC_SetMoveCtrlMode(Axis_Pitch, CTRLMODE_SPEED);
    SBGC_SetMoveValue(Axis_Pitch, -1.0f * param2 * ROTATION_SPEED_MAX / 0xFF);
    
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_cmdPitchUp)

static int SBGC_cmdPitchDown(void *parameter){
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    rt_uint8_t param2 = val & 0x00FF;
    //LOG_D("SBGC_cmdStopAll(%d)", val);
    
    SBGC_SetMoveCtrlMode(Axis_Pitch, CTRLMODE_SPEED);
    SBGC_SetMoveValue(Axis_Pitch, 1.0f * param2 * ROTATION_SPEED_MAX / 0xFF);
    
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_cmdPitchDown)

static int SBGC_cmdYawLeft(void *parameter)
{
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    rt_uint8_t param1 = (val >> 8) & 0x00FF;
    //LOG_D("SBGC_cmdStopAll(%d)", val);
    
    SBGC_SetMoveCtrlMode(Axis_Yaw, CTRLMODE_SPEED);
    SBGC_SetMoveValue(Axis_Yaw, -1.0f * param1 * ROTATION_SPEED_MAX / 0xFF);
    
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_cmdYawLeft)

static int SBGC_cmdYawRight(void *parameter)
{
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    rt_uint8_t param1 = (val >> 8) & 0x00FF;
    //LOG_D("SBGC_cmdStopAll(%d)", val);
    
    SBGC_SetMoveCtrlMode(Axis_Yaw, CTRLMODE_SPEED);
    SBGC_SetMoveValue(Axis_Yaw, 1.0f * param1 * ROTATION_SPEED_MAX / 0xFF);
    
    return SBGC_MoveGimbal(RT_TICK_PER_SECOND / 50);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SBGC_cmdYawRight)
