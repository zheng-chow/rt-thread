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
#define DBG_SECTION_NAME "ZINGTO"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#define ZINGTO_UARTPORT_NAME "uart1"
#define ZINGTO_SEMAPHORE_NAME "shZingto"


#define ZINGTO_BUFFER_SIZE     (8)
#define ZINGTO_RX_TIMEOUT      (10)

#define ZINGTO_PKT_HEADER0     (0xE1)
#define ZINGTO_PKT_HEADER1     (0x1E)
#define ZINGTO_PKT_TAIL0       (0xF1)
#define ZINGTO_PKT_TAIL1       (0x1F)

#define ZINGTO_PKT_SIZE   (5)

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

static rt_sem_t semaph = RT_NULL;

static rt_err_t uart_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

static void zingto_data_recv_entry(void* parameter)
{
    struct guardian_environment *env = RT_NULL;
    rt_uint8_t* pbuf;
    rt_size_t szbuf;
    rt_device_t dev = RT_NULL;
    rt_err_t result;
    
    env = (struct guardian_environment*)parameter;
    RT_ASSERT(env != RT_NULL);
    
    pbuf = rt_malloc(ZINGTO_BUFFER_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(ZINGTO_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    
    LOG_I("recv sub-thread, start!");
    
    while (1)
    {
        result = rt_sem_take(semaph, RT_WAITING_FOREVER);
        
        if(result == -RT_ETIMEOUT)
            continue;
        
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
    }
}

void zingto_resolving_entry(void* parameter)
{
    rt_device_t dev = RT_NULL;
    struct guardian_environment *env = RT_NULL;
    rt_err_t result = RT_EOK;
    rt_thread_t pthread = RT_NULL;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    dev = rt_device_find(ZINGTO_UARTPORT_NAME);
    RT_ASSERT(dev != RT_NULL);
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    semaph = rt_sem_create(ZINGTO_SEMAPHORE_NAME, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semaph != RT_NULL);
    
    // set uart in 115200, 8N1.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(dev, uart_hook_callback);
    
    pthread = rt_thread_create("tUSRrx", zingto_data_recv_entry, &env, 2048, 12, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    LOG_I("initialization finish, start!");

    while (1)
    {
        rt_thread_delay(RT_TICK_PER_SECOND);
    }
    
    // never be here.
}

