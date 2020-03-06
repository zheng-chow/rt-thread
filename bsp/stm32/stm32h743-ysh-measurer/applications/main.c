/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-09-17     tyustli   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(GET_PIN(G, 10), PIN_MODE_OUTPUT);
    rt_pin_mode(GET_PIN(G, 11), PIN_MODE_OUTPUT);
    rt_pin_mode(GET_PIN(G, 12), PIN_MODE_OUTPUT);
    rt_pin_mode(GET_PIN(G, 13), PIN_MODE_OUTPUT);
   
    while (count++)
    {
        rt_pin_write(GET_PIN(G, 10), PIN_HIGH);
        rt_pin_write(GET_PIN(G, 11), PIN_LOW);
        rt_pin_write(GET_PIN(G, 12), PIN_LOW);
        rt_pin_write(GET_PIN(G, 13), PIN_LOW);
        rt_thread_mdelay(50);
        rt_pin_write(GET_PIN(G, 10), PIN_LOW);
        rt_pin_write(GET_PIN(G, 11), PIN_HIGH);
        rt_pin_write(GET_PIN(G, 12), PIN_LOW);
        rt_pin_write(GET_PIN(G, 13), PIN_LOW);
        rt_thread_mdelay(50);        
        rt_pin_write(GET_PIN(G, 10), PIN_LOW);
        rt_pin_write(GET_PIN(G, 11), PIN_LOW);
        rt_pin_write(GET_PIN(G, 12), PIN_HIGH);
        rt_pin_write(GET_PIN(G, 13), PIN_LOW);
        rt_thread_mdelay(50);
        rt_pin_write(GET_PIN(G, 10), PIN_LOW);
        rt_pin_write(GET_PIN(G, 11), PIN_LOW);
        rt_pin_write(GET_PIN(G, 12), PIN_LOW);
        rt_pin_write(GET_PIN(G, 13), PIN_HIGH);
        rt_thread_mdelay(50);                
    }
    return RT_EOK;
}
