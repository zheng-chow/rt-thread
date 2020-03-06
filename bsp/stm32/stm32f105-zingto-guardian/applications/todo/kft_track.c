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
#define DBG_SECTION_NAME "visca"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#define SBUS_UARTPORT_NAME "uart4"
#define SBUS_SEMAPHORE_NAME "sbus"

/* defined the LED pin: PA0 */
#define LED_PIN    GET_PIN(A, 0)

#define CHANNEL_NUMBER  (16)
#define BIT_PER_CHANNEL (11)
#define PKT_HEADER0     (0x0F)
#define PKT_HEADER1     (0x8F)
#define PKT_TAIL        (0x00)
#define PKT_TAIL0       (0x04)
#define PKT_TAIL1       (0x14)
#define PKT_TAIL2       (0x24)
#define PKT_TAIL3       (0x34)

#define HEADER_OFFSET   (00)
#define CHANNEL_OFFSET  (01)
#define FLAG_OFFSET     (23)
#define TAIL_OFFSET     (24)

#define SBUS_PKT_SIZE   (25)

static rt_sem_t semaph = RT_NULL;

rt_err_t sbus_hook_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

void sbus_reform_value(rt_uint8_t *pbuf, rt_uint16_t *pval)
{
    rt_int8_t m = 0, n = 0;
    rt_int8_t currbit = 0, currbyte = 0;
    
    for (n = 0; n < CHANNEL_NUMBER; n++)
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
    static rt_device_t dev = RT_NULL;
    static rt_uint8_t *pbuf = RT_NULL;
    static struct guardian_environment *env = RT_NULL;
    static rt_err_t result = RT_EOK;
    
    env = (struct guardian_environment *)parameter;
    RT_ASSERT(env != RT_NULL);
    
    env->channel_number = CHANNEL_NUMBER;
    env->channel = rt_malloc(env->channel_number * sizeof(rt_uint16_t));
    RT_ASSERT(env->channel != RT_NULL);
    rt_memset(env->channel, 0x00, env->channel_number * sizeof(rt_uint16_t));
    
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
    
    rt_device_set_rx_indicate(dev, sbus_hook_callback);
    
    LOG_I("initialization finish, process start!");

    while (1)
    {
        static rt_size_t szbuf = 0;
        
        result = rt_sem_take(semaph, RT_WAITING_FOREVER);
        
        if(result == -RT_ETIMEOUT)
            continue;
        
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
        
        if (szbuf != SBUS_PKT_SIZE)
            continue;
        
        szbuf = 0;
        // check packet tail, check vaild.
        if (!((pbuf[TAIL_OFFSET] == PKT_TAIL) || (pbuf[TAIL_OFFSET] == PKT_TAIL0) || (pbuf[TAIL_OFFSET] == PKT_TAIL1) || (pbuf[TAIL_OFFSET] == PKT_TAIL2) || (pbuf[TAIL_OFFSET] == PKT_TAIL3)))
            continue;
        
        // got a complete packet, decode now.
        if (rt_pin_read(LED_PIN))
            rt_pin_write(LED_PIN, PIN_LOW);
        else
            rt_pin_write(LED_PIN, PIN_HIGH);
        
        sbus_reform_value(pbuf, env->channel);
        
    }
    
    // never be here.
}

