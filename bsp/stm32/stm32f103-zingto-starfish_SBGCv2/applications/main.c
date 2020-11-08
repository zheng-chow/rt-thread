/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "zingto.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME "main"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#include <cmux.h>

static rt_device_t pWDT = RT_NULL;

static void idle_hook(void)
{
    rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
//    rt_kprintf("feed the dog!\n ");
}

int main(void)
{   
    rt_size_t timeout = 1;
    
    pWDT = rt_device_find("wdt");
    RT_ASSERT(pWDT != RT_NULL);
    
    rt_device_init(pWDT);
    rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    rt_thread_idle_sethook(idle_hook);
    LOG_I("Set WatchDog Complate!");
    
    /* set PWR_SBGC pin mode to Push-Pull */
    rt_pin_write(PWSW_PIN, PIN_HIGH);
    rt_pin_mode (PWSW_PIN, PIN_MODE_OUTPUT);
    /* set LED pin mode to Push-Pull */
    rt_pin_mode (LED_PIN, PIN_MODE_OUTPUT);
    
    while (1)
    {
        if (rt_pin_read(LED_PIN) == PIN_HIGH) {
            rt_pin_write(LED_PIN, PIN_LOW);
        }
        else {
            rt_pin_write(LED_PIN, PIN_HIGH);
        }
        
        rt_thread_mdelay(RT_TICK_PER_SECOND / 5);
    }

    // Never reach here.
}
