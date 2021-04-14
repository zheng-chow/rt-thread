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
#define DBG_LEVEL DBG_INFO   //DBG_INFO DBG_LOG
#define DBG_SECTION_NAME  "VISCA"
#define DBG_COLOR
#include <rtdbg.h>

#include <math.h>
#include <dynacall.h>
#include <drv_sbus.h>
#include "VISCA.h"
#include "SBGC32.h"

#define VISCA_MUTEX_UDEV    "mtx_VISCA_Udev"
#define VISCA_DEVICE_UART   "uart1"

static rt_mutex_t mtx_udev = RT_NULL;
static rt_device_t dev_uart = RT_NULL;

#define VISCA_TXBUFF_SIZE    32
static rt_uint8_t txbuf[VISCA_TXBUFF_SIZE];

#define VISCA_RXBUFF_SIZE    32
static rt_uint8_t rxbuf[VISCA_RXBUFF_SIZE];

const rt_uint16_t ZoomTable_30X_12X[] = {
                      0x0000, 0x16A1, 0x2063, 0x2628, 0x2A1D, 0x2D13    //  1X ~ 6X
                    , 0x2F6D, 0x3161, 0x330D, 0x3486, 0x35D7, 0x3709    //  7X ~ 12X
                    , 0x3820, 0x3920, 0x3A0A, 0x3ADD, 0x3B9C, 0x3C46    // 13X ~ 18X
                    , 0x3CDC, 0x3D60, 0x3DD4, 0x3E39, 0x3E90, 0x3EDC    // 19X ~ 24X
                    , 0x3F1E, 0x3F57, 0x3F8A, 0x3FB6, 0x3FDC, 0x4000    // 25X ~ 30X
                    , 0x4000, 0x6000, 0x6A80, 0x7000, 0x7300, 0x7540    // digital 1X ~ 6X
                    , 0x76C0, 0x7800, 0x78C0, 0x7980, 0x7A00, 0x7AC0};  // digital 7X ~ 12X

const rt_uint16_t ZoomTable_20X[] = {
                      0x0000, 0x177F, 0x222A, 0x28AB, 0x2D46, 0x30D9    // 1X ~ 6X
                    , 0x33C9, 0x3644, 0x3863, 0x3A28, 0x3B9F, 0x3CCA    // 7X ~ 12X
                    , 0x3DC3, 0x3E70, 0x3F0F, 0x3F81, 0x3FA3, 0x3FC0    // 13X ~ 18X
                    , 0x3FE8, 0x4000};                                  // 19X ~ 20X

const float CoeffTable_30X_12X[] = {
                    1.00f, 0.50f, 0.45f, 0.40f, 0.35f, 0.30f, 0.30f, 0.30f, 0.30f, 0.25f
                  , 0.25f, 0.25f, 0.20f, 0.20f, 0.20f, 0.15f, 0.15f, 0.15f, 0.10f, 0.10f
                  , 0.10f, 0.10f, 0.10f, 0.10f, 0.10f, 0.10f, 0.10f, 0.10f, 0.10f, 0.10f
                  , 0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f
                  , 0.05f, 0.05f};
                    
const float CoeffTable_20X[] = {
                    1.0f, 0.5f, 0.45f, 0.4f, 0.35f, 0.3f, 0.3f, 0.3f, 0.3f, 0.25f
                  , 0.25f, 0.25f, 0.2f, 0.2f, 0.2f, 0.15f, 0.15f, 0.15f, 0.1f, 0.1f};


                    
static float ZoomPosGlob = 1.0f;
                    
static int VISCA_Init(void)
{
    mtx_udev = rt_mutex_create(VISCA_MUTEX_UDEV, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mtx_udev != RT_NULL);
    
    dev_uart = rt_device_find(VISCA_DEVICE_UART);
    RT_ASSERT(dev_uart != RT_NULL);
    rt_device_open(dev_uart, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX);
    
    LOG_I("VISCA Runtime Initial Done.");
    
    return RT_EOK;
}
INIT_ENV_EXPORT(VISCA_Init);

static inline void VISCA_ClearRxBuffer(void)
{
    rt_size_t count;
    
    do {
        count = rt_device_read(dev_uart, 0, rxbuf, VISCA_RXBUFF_SIZE);
    }while(count != 0);
}

static inline int VISCA_ImmediateWrite(rt_size_t size) {
    return rt_device_write(dev_uart, 0, txbuf, size);
}

static inline int HI3521_ImmediatePrint(char *buff) {
    return rt_device_write(dev_uart, 0, buff, sizeof(buff));
}

static inline int VISCA_BlockRead(int expect_size, rt_tick_t timeout)
{
    rt_size_t count;
    rt_tick_t start;
    
    if (expect_size > VISCA_RXBUFF_SIZE) return -RT_ENOMEM;
    
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

static VISCA_CategoryCode_t VISCA_GetCategoryCode(VISCA_CMD_t cmd)
{
    if ((cmd == VISCA_CMD_ZoomOut) || (cmd == VISCA_CMD_ZoomOutSpeed) \
        || (cmd == VISCA_CMD_ZoomIn) || (cmd == VISCA_CMD_ZoomInSpeed) \
        || (cmd == VISCA_CMD_ZoomStop) || (cmd == VISCA_CMD_ZoomSet)) {
        return VISCA_Cat_Basic;
    }
    else if ((cmd == VISCA_CMD_PhotoRecord) || (cmd == VISCA_CMD_VideoRecord)) {
        return VISCA_Cat_HYK;
    }
    
    return VISCA_Cat_Reserve;
}

static int VISCA_MakeBasicCommand(VISCA_CMD_t cmd, rt_uint8_t param)
{
    rt_memset(txbuf, 0, VISCA_TXBUFF_SIZE);
    
    txbuf[0] = 0x80 | VISCA_DEV_ADDR;
    txbuf[1] = 0x01;
    txbuf[2] = VISCA_Cat_Basic;
    txbuf[3] = cmd >> 8;
    if (cmd & 0xF0){
        txbuf[4] = (cmd & 0xF0) | (param & 0x0F);
    }
    else {
        txbuf[4] = cmd;
    }
    txbuf[5] = 0xFF;
    
    return 6;
}

static int VISCA_MakeHYKCommand(VISCA_CMD_t cmd, rt_uint8_t param)
{
    rt_memset(txbuf, 0, VISCA_TXBUFF_SIZE);
    
    txbuf[0] = 0x80 | VISCA_DEV_ADDR;
    txbuf[1] = 0x01;
    txbuf[2] = VISCA_Cat_HYK;
    txbuf[3] = cmd >> 8;
    txbuf[4] = (cmd & 0xF0) | (param & 0x0F);
    txbuf[5] = 0xFF;
    
    return 6;
}

static int VISCA_ExecCommand(VISCA_CMD_t cmd, rt_uint8_t param)
{
    int bufsz = 0;
    rt_err_t retval;
    VISCA_CategoryCode_t category = VISCA_GetCategoryCode(cmd);

    rt_mutex_take(mtx_udev, RT_WAITING_FOREVER);
    
    switch(category) {
        case VISCA_Cat_Basic:
            bufsz = VISCA_MakeBasicCommand(cmd, param);
            break;
        case VISCA_Cat_HYK:
            bufsz = VISCA_MakeHYKCommand(cmd, param);
            break;
        case VISCA_Cat_Gimbal:
        case VISCA_Cat_Regs:
        default:
            return -RT_EINVAL;
    }

    VISCA_ClearRxBuffer();
    VISCA_ImmediateWrite(bufsz);

    LOG_D("cmd %d", bufsz);
    retval = VISCA_BlockRead(3, 50);
    if (retval == RT_EOK) {
        LOG_D("==>ack");
        retval = VISCA_BlockRead(3, 500);
        if (retval == RT_EOK) {
            LOG_D("+++==>done");
        }
    }
    
    rt_mutex_release(mtx_udev);
    
    return retval;
}

// public functions.
int VISCA_GetZoomPos(float *zoom_pos, ZOOM_Ratio_t ratio, int timeout)
{
    rt_err_t retval;
    float zoom = 1.f, dzoom = 1.f;
    const rt_uint16_t *zoom_table = RT_NULL;
    rt_size_t zoom_ratio = 0;
    
    switch(ratio){
        case ZOOM_Ratio_Optical_20X:
            zoom_table = ZoomTable_20X; break;
        case ZOOM_Ratio_Mix_30X_12X:
        case ZOOM_Ratio_Optical_30X:
            zoom_table = ZoomTable_30X_12X; break;
        default:
            return -RT_EINVAL;
    }
    zoom_ratio = ratio;
    
    rt_mutex_take(mtx_udev, RT_WAITING_FOREVER);
    
    rt_memset(txbuf, 0, VISCA_TXBUFF_SIZE);
    
    txbuf[0] = 0x80 | VISCA_DEV_ADDR;
    txbuf[1] = 0x09;
    txbuf[2] = VISCA_Cat_Basic;
    txbuf[3] = VISCA_CMD_ZoomInquiry;
    txbuf[4] = 0xFF;
    
    VISCA_ClearRxBuffer();
    VISCA_ImmediateWrite(5);
    
    LOG_D("cmd %d", bufsz);
    retval = VISCA_BlockRead(7, timeout);   // 90 50 0p 0q 0r 0s FF, pqrs = Zoom Position.
    if (retval == RT_EOK) {
        LOG_D("==>zoom pos");
    }
    
    rt_mutex_release(mtx_udev);
    
    if (retval == RT_EOK) {
        rt_uint16_t raw_pos = 0;
        raw_pos = ((rxbuf[2] & 0x0F) << 12) | \
                   ((rxbuf[3] & 0x0F) << 8) | \
                   ((rxbuf[4] & 0x0F) << 4) | \
                   ((rxbuf[5] & 0x0F) << 0);
        
        for (int i = 1; i < zoom_ratio; i++) {
            if (raw_pos < zoom_table[i] ) {
                zoom = (float)i;
                zoom += (float)(raw_pos - zoom_table[i - 1]) / (float)(zoom_table[i] - zoom_table[i - 1]);
                dzoom = 1.0f;
                break;
            }
            else if (raw_pos == zoom_table[i] ) {
                if (i > 29) {
                    zoom = 30.0f;
                    dzoom = (float)( (i + 1) - 29);
                }
                else {
                    zoom = (float)(i + 1);
                    dzoom = 1.0f;
                }
                break;
            }
        }
        
        *zoom_pos = zoom * dzoom;
    }
    
    return retval;
}

static float VISCA_GetCoeff_20X(float zoom) {
    rt_uint8_t offset = (rt_uint8_t)(zoom - 1);
    
    if ( offset < sizeof(CoeffTable_20X))
        return CoeffTable_20X[offset];
    
    return 1.0f;
}

static float VISCA_GetCoeff_30X_12X(float zoom) {
    rt_uint8_t offset = (rt_uint8_t)(zoom - 1);
    
    if ( offset < sizeof(CoeffTable_30X_12X))
        return CoeffTable_30X_12X[offset];
    
    return 1.0f;
}

#include <stdio.h>
static void zoom_pos(int argc, char **argv)
{
    char cbuf[128];
    rt_err_t retval;
    
    retval = VISCA_GetZoomPos(&ZoomPosGlob, ZOOM_Ratio_Optical_20X, 200);

    if (retval == RT_EOK) {
        rt_memset(cbuf, 0, 128);
        snprintf(cbuf, 128, "zoom: %3.1fX", ZoomPosGlob);
        rt_kprintf("%s\n", cbuf);
    }
    else {
        rt_kprintf("Read VISCA Zoom position Failed!\n");
    }
}
MSH_CMD_EXPORT(zoom_pos, read the VISCA zoom-position.)

// dynacall functions.
static int VISCA_execCameraZoom(void *parameter) {
    static rt_int8_t prev_level;
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    rt_int8_t new_level = (rt_int8_t)(sbus_normalize_value(val) * 7.0f);
    
    if (prev_level == new_level) return -RT_EBUSY;
    
    prev_level = new_level;
    LOG_D("VISCA_execCameraZoom, level:%d", prev_level);
    
    if (new_level > 0) {
        VISCA_ExecCommand(VISCA_CMD_ZoomInSpeed, new_level);
    }
    else if (new_level < 0) {
        VISCA_ExecCommand(VISCA_CMD_ZoomOutSpeed, -new_level);
    }
    else {
        VISCA_ExecCommand(VISCA_CMD_ZoomStop, 0);
    }
    
    return RT_EOK;
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(VISCA_execCameraZoom)

static int VISCA_cmdZoomIn(void *parameter) {
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    rt_uint8_t zoom_speed = (val >> 8) & 0x00FF;
    
    if (zoom_speed < 2) zoom_speed = 2;
    else if (zoom_speed > 7) zoom_speed = 7;
    
    LOG_D("VISCA_cmdZoomIn, speed:%d", zoom_speed);
   
    return VISCA_ExecCommand(VISCA_CMD_ZoomInSpeed, zoom_speed);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(VISCA_cmdZoomIn)

static int VISCA_cmdZoomOut(void *parameter) {
    rt_uint16_t val = *(rt_uint16_t *)parameter;
    rt_uint8_t zoom_speed = (val >> 8) & 0x00FF;
    
    if (zoom_speed < 2) zoom_speed = 2;
    else if (zoom_speed > 7) zoom_speed = 7;
    
    LOG_D("VISCA_cmdZoomOut, speed:%d", zoom_speed);
   
    return VISCA_ExecCommand(VISCA_CMD_ZoomOutSpeed, zoom_speed);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(VISCA_cmdZoomOut)

static int VISCA_cmdZoomStop(void *parameter) {
    LOG_D("VISCA_cmdZoomStop, speed:%d", 0);
    return VISCA_ExecCommand(VISCA_CMD_ZoomStop, 0);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(VISCA_cmdZoomStop)

static int HYK_execPhotoRecord(void *parameter) {
    return VISCA_ExecCommand(VISCA_CMD_PhotoRecord, 0);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(HYK_execPhotoRecord)

static int HYK_trigVideoRecordStatus(void * parameter) {
    static rt_bool_t iswork = RT_FALSE;
    
    iswork = ~iswork;
    return VISCA_ExecCommand(VISCA_CMD_PhotoRecord, iswork);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(HYK_trigVideoRecordStatus)

static int HYK_servAdjustRotateSpeed(void *parameter) {   
    VISCA_GetZoomPos(&ZoomPosGlob, ZOOM_Ratio_Optical_20X, 200);
    
    float coeff = VISCA_GetCoeff_20X(ZoomPosGlob);
    SBGC_SetRotationCoeff(coeff);

    return RT_EOK;
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(HYK_servAdjustRotateSpeed)

static int EV7500_servAdjustRotateSpeed(void *parameter) {   
    VISCA_GetZoomPos(&ZoomPosGlob, ZOOM_Ratio_Mix_30X_12X, 200);
    
    float coeff = VISCA_GetCoeff_30X_12X(ZoomPosGlob);
    SBGC_SetRotationCoeff(coeff);

    return RT_EOK;
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(EV7500_servAdjustRotateSpeed)

#define HI3521_CMD_CAPTURE     "CAPTURE:ON\n"
#define HI3521_CMD_RECORD_ON   "RECORD:ON\n"
#define HI3521_CMD_RECORD_OFF  "RECORD:OFF\n"

static int HI3521_execPhotoRecord(void *parameter) {
    return HI3521_ImmediatePrint(HI3521_CMD_CAPTURE);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(HI3521_execPhotoRecord)

static int HI3521_trigVideoRecordStatus(void * parameter) {
    static rt_bool_t iswork = RT_FALSE;
    int retval;
    
    iswork = ~iswork;
    if (iswork)
        retval = HI3521_ImmediatePrint(HI3521_CMD_RECORD_ON);
    else
        retval = HI3521_ImmediatePrint(HI3521_CMD_RECORD_OFF);
    
    return retval;
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(HI3521_trigVideoRecordStatus)

static int HI3521_execVideoRecordStart(void * parameter) {
    return HI3521_ImmediatePrint(HI3521_CMD_RECORD_ON);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(HI3521_execVideoRecordStart)

static int HI3521_execVideoRecordStop(void * parameter) {
    return HI3521_ImmediatePrint(HI3521_CMD_RECORD_OFF);
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(HI3521_execVideoRecordStop)
