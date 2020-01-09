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

/* defined the LED0 pin: PA0 */
#define LED0_PIN        GET_PIN(A, 0)
/* defined the LED1 pin: PA1 */
#define LED1_PIN        GET_PIN(A, 1)
/* defined the LANRST pin: PB13 */
#define LANRST_PIN      GET_PIN(B, 13)
/* defined the CUC_RST pin: PB14 */
#define CUCRST_PIN      GET_PIN(B, 14)

int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    
    /* set RST pin mode to output */    
    rt_pin_mode(LANRST_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(CUCRST_PIN, PIN_MODE_OUTPUT);
    
    /* reset all chips and units on PCB */
    rt_pin_write(LANRST_PIN, PIN_LOW);
    rt_pin_write(CUCRST_PIN, PIN_LOW);   
    rt_thread_mdelay(200);    
    rt_pin_write(LANRST_PIN, PIN_HIGH);
    rt_pin_write(CUCRST_PIN, PIN_HIGH);
    
    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
