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
#define DBG_SECTION_NAME "Camera"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

const char XUANZHAN_CMD_SNAP[] = "<snap quality=\"10\" ></snap>\r\n";
const char XUANZHAN_CMD_RECORD_ON[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><?xml version=\"1.0\" encoding=\"GB2312\" ?><record cmd=\"start\"></record>\r\n";
const char XUANZHAN_CMD_RECORD_OFF[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><?xml version=\"1.0\" encoding=\"GB2312\" ?><record cmd=\"stop\"></record>\r\n";

const char XUANZHAN_LOGO_RECORD_ON[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><icon name=\"record_l\"  posx=\"10\" posy=\"10\" cmd=\"show\" ></icon>\r\n";
const char XUANZHAN_LOGO_RECORD_OFF[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><icon name=\"record_l\"  posx=\"10\" posy=\"10\" cmd=\"hide\" ></icon>\r\n";
const char XUANZHAN_LOGO_SNAP_ON[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><icon name=\"photo_l\"  posx=\"10\" posy=\"10\" cmd=\"show\" ></icon>\r\n";
const char XUANZHAN_LOGO_SNAP_OFF[] = "<?xml version=\"1.0\" encoding=\"GB2312\" ?><icon name=\"photo_l\"  posx=\"10\" posy=\"10\" cmd=\"hide\" ></icon>\r\n";

#define CAMERA_UARTPORT_NAME "uart5"
#define CAMERA_SEMAPHORE_NAME "shCAM"
#define CAMERA_EVENT_NAME "evCAM"

#define CAMERA_BUFFER_SIZE              (32)
#define CAMERA_RX_TIMEOUT               (RT_TICK_PER_SECOND)  // almost 0.3s

#define VISCA_ZOOM_SPEED                (2)
#define VISCA_SET_ZOOMPOS_CMD_SIZE      (9)
#define VISCA_GET_ZOOMPOS_ACK_SIZE      (7)


rt_uint8_t VISCA_ZOOM_IN[]        = {0x81, 0x01, 0x04, 0x07, 0x20, 0xFF};
rt_uint8_t VISCA_ZOOM_OUT[]       = {0x81, 0x01, 0x04, 0x07, 0x30, 0xFF};
const rt_uint8_t VISCA_ZOOM_STOP[]      = {0x81, 0x01, 0x04, 0x07, 0x00, 0xFF};
const rt_uint8_t VISCA_GET_ZOOMPOS[]    = {0x81, 0x09, 0x04, 0x47, 0xFF};
const rt_uint8_t VISCA_SET_1080P50HZ[]  = {0x81, 0x01, 0x04, 0x24, 0x72, 0x01, 0x04, 0xFF, 0x90, 0x41, 0xFF, 0x90, 0x51, 0xFF};
const rt_uint8_t VISCA_SET_1080P60HZ[]  = {0x81, 0x01, 0x04, 0x24, 0x72, 0x01, 0x05, 0xFF, 0x90, 0x41, 0xFF, 0x90, 0x51, 0xFF};

const rt_uint8_t VISCA_ZOOM[30][VISCA_SET_ZOOMPOS_CMD_SIZE] = 
{
	{0x81, 0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00, 0xFF},	//1X
	{0x81, 0x01, 0x04, 0x47, 0x01, 0x06, 0x0a, 0x01, 0xFF},	//2X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x00, 0x06, 0x03, 0xFF},	//3X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x06, 0x02, 0x08, 0xFF},	//4X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x0a, 0x01, 0x0d, 0xFF},	//5X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x0d, 0x01, 0x03, 0xFF},	//6X
	{0x81, 0x01, 0x04, 0x47, 0x02, 0x0f, 0x06, 0x0d, 0xFF},	//7X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x01, 0x06, 0x01, 0xFF},	//8X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x03, 0x00, 0x0d, 0xFF},	//9X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x04, 0x08, 0x06, 0xFF},	//10X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x05, 0x0d, 0x07, 0xFF},	//11X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x07, 0x00, 0x09, 0xFF},	//12X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x08, 0x02, 0x00, 0xFF},	//13X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x09, 0x02, 0x00, 0xFF},	//14X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0a, 0x00, 0x0a, 0xFF},	//15X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0a, 0x0d, 0x0d, 0xFF},	//16X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0b, 0x09, 0x0c, 0xFF},	//17X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0c, 0x04, 0x06, 0xFF},	//18X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0c, 0x0d, 0x0c, 0xFF},	//19X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0d, 0x06, 0x00, 0xFF},	//20X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0d, 0x0d, 0x04, 0xFF},	//21X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x03, 0x09, 0xFF},	//22X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x09, 0x00, 0xFF},	//23X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0e, 0x0d, 0x0c, 0xFF},	//24X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x01, 0x0e, 0xFF},	//25X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x05, 0x07, 0xFF},	//26X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x08, 0x0a, 0xFF},	//27X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x0b, 0x06, 0xFF},	//28X
	{0x81, 0x01, 0x04, 0x47, 0x03, 0x0f, 0x0d, 0x0c, 0xFF},	//29X
	{0x81, 0x01, 0x04, 0x47, 0x04, 0x00, 0x00, 0x00, 0xFF},	//30X
};

#define HI3521D_CMD_CAPTURE     "CAPTURE:ON\n"
#define HI3521D_CMD_RECORD_ON   "RECORD:ON\n"
#define HI3521D_CMD_RECORD_OFF  "RECORD:OFF\n"
#define HI3521D_CMD_PIP_MODE1   "SHOWCHANNEL:S1\n"
#define HI3521D_CMD_PIP_MODE2   "SHOWCHANNEL:S2\n"
#define HI3521D_CMD_PIP_MODE3   "SHOWCHANNEL:S3\n"
#define HI3521D_CMD_PIP_MODE4   "SHOWCHANNEL:S4\n"

#define HI3521D_ACK_CAPTURE     "CAPTURE:OK\n"
#define HI3521D_ACK_RECORD      "RECORD:OK\n"
#define HI3521D_ACK_PIP_MODE    "SHOWCHANNEL:OK\n"

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

static rt_sem_t semaph = RT_NULL;

static rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
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
        sz_recv = rt_device_read(dev, 0, buffer, CAMERA_BUFFER_SIZE);
        
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
        result = rt_sem_take(semaph, CAMERA_RX_TIMEOUT);
        
        if (result == -RT_ETIMEOUT)
            break;
        
        sz_recv += rt_device_read(dev, 0, (rt_uint8_t*)buffer + sz_recv, size - sz_recv);
        
        if (sz_recv == size)
            break;
    }
    
    return result;
}
static rt_tick_t tm_snap;

void snap_logo_manager_entry(void* parameter)
{
    rt_device_t udev = (rt_device_t)parameter;
    
    tm_snap = RT_TICK_MAX;
    
    while(1)
    {
        if (tm_snap != RT_TICK_MAX) {
            if (rt_tick_get() - tm_snap > RT_TICK_PER_SECOND) {
                uart_send_with_block(udev, (void *)XUANZHAN_LOGO_SNAP_OFF, sizeof(XUANZHAN_LOGO_SNAP_OFF));
                tm_snap = RT_TICK_MAX;
            }
        }
        
        rt_thread_delay(RT_TICK_PER_SECOND / 100);
    }
}

void camera_resolving_entry(void* parameter)
{
    rt_device_t dev = RT_NULL;
    rt_uint8_t *pbuf = RT_NULL;
    struct guardian_environment *env = RT_NULL;
    rt_err_t result = RT_EOK;
    rt_uint32_t opcode;
    rt_thread_t pthread;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(CAMERA_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(CAMERA_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    semaph = rt_sem_create(CAMERA_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);
    
    env->ev_camera = rt_event_create(CAMERA_EVENT_NAME, RT_IPC_FLAG_FIFO);
    RT_ASSERT(env->ev_camera != RT_NULL);
    
    // set uart in 115200     , 8N1.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
    
    pthread = rt_thread_create("tXZlogo", snap_logo_manager_entry, dev, 2048, 11, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    LOG_I("initialization finish, start!");
    
    env->cam_pip_mode = 3;   // defined S3 mode.
    env->cam_recording = RT_FALSE;

    while (1)
    {
        result = rt_event_recv(env->ev_camera, CAMERA_CMD_MASK, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &opcode);
        
        if ( result != RT_EOK)
        {
            LOG_E("event receive failed");
            continue;
        }
        
        if (opcode & CAMERA_CMD_ZOOM_STOP)
        {
            opcode = CAMERA_CMD_ZOOM_STOP;
        }
        
        switch(opcode & ~CAMERA_CMD_ZOOM_GETPOS) {
            case CAMERA_CMD_CAPTURE:
                #ifdef USE_HU_CAMERA
                    uart_send_with_block(dev, (void *)CAMERA_VISCA_CAPTURE, sizeof(CAMERA_VISCA_CAPTURE));
                    LOG_D("VISCA_CMD_CAPTURE");
                #else
                    uart_clean_recv_buff(dev, pbuf);     
                    uart_send_with_block(dev, HI3521D_CMD_CAPTURE, sizeof(HI3521D_CMD_CAPTURE));
                    LOG_D("HI3521D_CMD_CAPTURE");
                    uart_send_with_block(dev, HI3521D_CMD_CAPTURE, sizeof(HI3521D_CMD_CAPTURE));
                    LOG_D("HI3521D_CMD_CAPTURE");
                
                    rt_memset(pbuf, 0x00, CAMERA_BUFFER_SIZE);
                    result = uart_recv_with_timeout(dev, pbuf, sizeof(HI3521D_ACK_CAPTURE));

                    if (result != RT_EOK)
                        LOG_W("timeout!");
                    else if (rt_strncmp(HI3521D_ACK_CAPTURE, (void*)pbuf, sizeof(HI3521D_ACK_CAPTURE) - 2))
                        LOG_W("invailed!, %-16s", pbuf);
                    else
                        LOG_D("OK");
                #endif
                break;
            case CAMERA_CMD_RECORD_ON:
                #ifdef USE_HU_CAMERA
                    uart_send_with_block(dev, (void *)CAMERA_VISCA_RECORD_ON, sizeof(CAMERA_VISCA_RECORD_ON));
                    LOG_D("VISCA_CMD_CAPTURE");
                #else                
                    uart_clean_recv_buff(dev, pbuf);
                    uart_send_with_block(dev, HI3521D_CMD_RECORD_ON, sizeof(HI3521D_CMD_RECORD_ON));
                    LOG_D("HI3521D_CMD_RECORD_ON");
            
                    rt_memset(pbuf, 0x00, CAMERA_BUFFER_SIZE);
                    result = uart_recv_with_timeout(dev, pbuf, sizeof(HI3521D_ACK_RECORD));
                
                    if (result != RT_EOK)
                        LOG_W("timeout!");
                    else if (rt_strncmp(HI3521D_ACK_RECORD, (void*)pbuf, sizeof(HI3521D_ACK_RECORD) - 2))
                        LOG_W("invailed!, %-16s", pbuf);
                    else
                        LOG_D("OK");
                #endif
                break;
            case CAMERA_CMD_RECORD_OFF:
                #ifdef USE_HU_CAMERA
                    uart_send_with_block(dev, (void *)CAMERA_VISCA_RECORD_OFF, sizeof(CAMERA_VISCA_RECORD_OFF));
                    LOG_D("VISCA_CMD_CAPTURE");
                #else 
                    uart_clean_recv_buff(dev, pbuf);
                    uart_send_with_block(dev, HI3521D_CMD_RECORD_OFF, sizeof(HI3521D_CMD_RECORD_OFF));
                    LOG_D("HI3521D_CMD_RECORD_OFF");
                
                    rt_memset(pbuf, 0x00, CAMERA_BUFFER_SIZE);
                    result = uart_recv_with_timeout(dev, pbuf, sizeof(HI3521D_ACK_RECORD));
                
                    if (result != RT_EOK)
                        LOG_W("timeout!");
                    else if (rt_strncmp(HI3521D_ACK_RECORD, (void*)pbuf, sizeof(HI3521D_ACK_RECORD) - 2))
                        LOG_W("invailed!, %-16s", pbuf);
                    else
                        LOG_D("OK");
                #endif
                break;
            case CAMERA_CMD_PIP_MODE1:
                uart_clean_recv_buff(dev, pbuf);
            
                uart_send_with_block(dev, HI3521D_CMD_PIP_MODE1, sizeof(HI3521D_CMD_PIP_MODE1));
                LOG_D("HI3521D_CMD_PIP1");
            
                rt_memset(pbuf, 0x00, CAMERA_BUFFER_SIZE);
                result = uart_recv_with_timeout(dev, pbuf, sizeof(HI3521D_ACK_PIP_MODE));

                if (result != RT_EOK)
                    LOG_W("timeout!");
                else if (rt_strncmp(HI3521D_ACK_PIP_MODE, (void*)pbuf, sizeof(HI3521D_ACK_PIP_MODE) - 2))
                    LOG_W("invailed!, %-16s", pbuf);
                else
                    LOG_D("OK");
                break;
            case CAMERA_CMD_PIP_MODE2:
                uart_clean_recv_buff(dev, pbuf);
            
                uart_send_with_block(dev, HI3521D_CMD_PIP_MODE2, sizeof(HI3521D_CMD_PIP_MODE2));
                LOG_D("HI3521D_CMD_PIP2");
            
                rt_memset(pbuf, 0x00, CAMERA_BUFFER_SIZE);
                result = uart_recv_with_timeout(dev, pbuf, sizeof(HI3521D_ACK_PIP_MODE));
            
                if (result != RT_EOK)
                    LOG_W("timeout!");
                else if (rt_strncmp(HI3521D_ACK_PIP_MODE, (void*)pbuf, sizeof(HI3521D_ACK_PIP_MODE) - 2))
                    LOG_W("invailed!, %-16s", pbuf);
                else
                    LOG_D("OK");
                break;
            case CAMERA_CMD_PIP_MODE3:
                uart_clean_recv_buff(dev, pbuf);
            
                uart_send_with_block(dev, HI3521D_CMD_PIP_MODE3, sizeof(HI3521D_CMD_PIP_MODE3));
                LOG_D("HI3521D_CMD_PIP3");
            
                rt_memset(pbuf, 0x00, CAMERA_BUFFER_SIZE);
                result = uart_recv_with_timeout(dev, pbuf, sizeof(HI3521D_ACK_PIP_MODE));
            
                if (result != RT_EOK)
                    LOG_W("timeout!");
                else if (rt_strncmp(HI3521D_ACK_PIP_MODE, (void*)pbuf, sizeof(HI3521D_ACK_PIP_MODE) - 2))
                    LOG_W("invailed!, %-16s", pbuf);
                else
                    LOG_D("OK");
                break;
            case CAMERA_CMD_PIP_MODE4:
                uart_clean_recv_buff(dev, pbuf);
            
                uart_send_with_block(dev, HI3521D_CMD_PIP_MODE4, sizeof(HI3521D_CMD_PIP_MODE4));
                LOG_D("HI3521D_CMD_PIP4");
            
                rt_memset(pbuf, 0x00, CAMERA_BUFFER_SIZE);
                result = uart_recv_with_timeout(dev, pbuf, sizeof(HI3521D_ACK_PIP_MODE));
            
                if (result != RT_EOK)
                    LOG_W("timeout!");
                else if (rt_strncmp(HI3521D_ACK_PIP_MODE, (void*)pbuf, sizeof(HI3521D_ACK_PIP_MODE) - 2))
                    LOG_W("invailed!, %-16s", pbuf);
                else
                    LOG_D("OK");
                break;
            
            // visca command
            case CAMERA_CMD_ZOOM_IN:
                uart_clean_recv_buff(dev, pbuf);
                VISCA_ZOOM_IN[4] &= 0xF0;
                VISCA_ZOOM_IN[4] |= env->cam_zoom_speed;
                uart_send_with_block(dev, (rt_uint8_t*)VISCA_ZOOM_IN, sizeof(VISCA_ZOOM_IN));
                LOG_D("VISCA_ZOOM_IN");
            
                result = uart_recv_with_timeout(dev, pbuf, 6);
                if (result != RT_EOK)
                    LOG_W("timeout!");
                else
                    LOG_D("OK");
                break;
            case CAMERA_CMD_ZOOM_OUT:
                uart_clean_recv_buff(dev, pbuf);
                VISCA_ZOOM_OUT[4] &= 0xF0;
                VISCA_ZOOM_OUT[4] |= env->cam_zoom_speed;
                uart_send_with_block(dev, (rt_uint8_t*)VISCA_ZOOM_OUT, sizeof(VISCA_ZOOM_OUT));
                LOG_D("VISCA_ZOOM_OUT");
            
                result = uart_recv_with_timeout(dev, pbuf, 6);
                if (result != RT_EOK)
                    LOG_W("timeout!");
                else
                    LOG_D("OK");
                break;
            case CAMERA_CMD_ZOOM_STOP:
                
                uart_clean_recv_buff(dev, pbuf);
                uart_send_with_block(dev, (rt_uint8_t*)VISCA_ZOOM_STOP, sizeof(VISCA_ZOOM_STOP));
                LOG_D("VISCA_ZOOM_STOP");
            
                result = uart_recv_with_timeout(dev, pbuf, 6);
                if (result != RT_EOK)
                    LOG_W("timeout!");
                else
                    LOG_D("OK");
                break;
            case 0:
                break;
            default:
                LOG_W("undefined opcode, %08X", opcode);
                break;
        }
        
        if (opcode & CAMERA_CMD_ZOOM_GETPOS)
        {
            uart_clean_recv_buff(dev, pbuf);
            uart_send_with_block(dev, (rt_uint8_t*)VISCA_GET_ZOOMPOS, sizeof(VISCA_GET_ZOOMPOS));
            LOG_D("VISCA_GET_ZOOM_POS");
        
            rt_memset(pbuf, 0x00, CAMERA_BUFFER_SIZE);
            result = uart_recv_with_timeout(dev, pbuf, VISCA_GET_ZOOMPOS_ACK_SIZE);
        
            if (result != RT_EOK)
                LOG_W("timeout!");
            else if (!(( pbuf[0] == 0x90 ) && ( pbuf[1] == 0x50 ) && ( pbuf[6] == 0xFF)))
                LOG_W("invaild data!");
            else
            {
                // calculate the zoom position.
                LOG_D("start calculate zoom position");
                LOG_D("%02X %02X %02X %02X %02X %02X %02X", \
                    pbuf[0], pbuf[1], pbuf[2], pbuf[3], pbuf[4], pbuf[5], pbuf[6]);
                
                rt_uint8_t nZoom = 0;
                rt_uint16_t nZoomNow, nZoomPrev, nZoomNext;

                nZoomNow = ((pbuf[2] & 0x0F) << 12) | \
                           ((pbuf[3] & 0x0F) << 8) | \
                           ((pbuf[4] & 0x0F) << 4) | \
                           ((pbuf[5] & 0x0F) << 0);

                for ( nZoom = 0; nZoom < 30; nZoom++)
                {
                    int res = rt_memcmp(&VISCA_ZOOM[nZoom][4], &pbuf[2], 4);
                    if (0 == res)
                        break;
                    if (res > 0)
                    {
                        if (nZoom == 29)
                            continue;
                        else
                        {
                            nZoomPrev = ((VISCA_ZOOM[nZoom - 1][4] & 0x0F) << 12) | \
                                        ((VISCA_ZOOM[nZoom - 1][5] & 0x0F) << 8) | \
                                        ((VISCA_ZOOM[nZoom - 1][6] & 0x0F) << 4) | \
                                        ((VISCA_ZOOM[nZoom - 1][7] & 0x0F) << 0);
                            
                            nZoomNext = ((VISCA_ZOOM[nZoom][4] & 0x0F) << 12) | \
                                        ((VISCA_ZOOM[nZoom][5] & 0x0F) << 8) | \
                                        ((VISCA_ZOOM[nZoom][6] & 0x0F) << 4) | \
                                        ((VISCA_ZOOM[nZoom][7] & 0x0F) << 0);
                            
                            if ((nZoomNext - nZoomNow) > (nZoomNow - nZoomPrev))
                            {
                                nZoom--;
                                break;
                            }		
                            else
                                break;
                        }
                    }
                }
                
                if (nZoom == 30)
                    nZoom = 0xFF;   // invailed zoom position.
                
                env->cam_zoom_pos = nZoom;
                LOG_D("zoom, %d", nZoom);
                
                env->cam_getpos_tick = rt_tick_get();

            }
        }
        
        // wait the next.
    }
    
    // never be here.
}

