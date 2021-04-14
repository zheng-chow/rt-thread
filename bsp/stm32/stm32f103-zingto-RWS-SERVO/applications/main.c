/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-08     obito0   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG //DBG_INFO
#define DBG_SECTION_NAME  "app.main"
#define DBG_COLOR
#include <rtdbg.h>

#define LED_BLINK       GET_PIN(C, 10)

void can_rx_thread_handler(void *args);


int main(void)
{
    rt_thread_t ptr = RT_NULL;
    rt_bool_t inWorking = RT_TRUE;
   
    rt_pin_mode(LED_BLINK, PIN_MODE_OUTPUT);
    /* start thread for CAN client */
    ptr = rt_thread_create("app_CANclient", can_rx_thread_handler, RT_NULL, 4096, 6, 20);
    if (ptr == RT_NULL) {
        LOG_E("create thread \"app_CANclient\" failed!");
        return -RT_ERROR;
    }
    
    rt_thread_startup(ptr);

    while(inWorking)
    {
        rt_pin_write(LED_BLINK, PIN_HIGH);
        rt_thread_delay(RT_TICK_PER_SECOND / 2);
        rt_pin_write(LED_BLINK, PIN_LOW);
        rt_thread_delay(RT_TICK_PER_SECOND / 2);
    }

    return -RT_ERROR;
}
