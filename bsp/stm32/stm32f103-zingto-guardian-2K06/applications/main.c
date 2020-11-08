/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include "guardian.h"

/* defined the LED pin: PB0 PB1*/
#define LED_SBUS_PIN   GET_PIN(B, 0)
#define LED_RUN_PIN    GET_PIN(B, 1)

#define MODE_PIN       GET_PIN(B, 3)

#define APP_VERSION "2.01.PuZhi"
#define RT_APP_PART_ADDR    0x08020000

static struct guardian_environment env;

/**
 * Function    ota_app_vtor_reconfig
 * Description Set Vector Table base location to the start addr of app(RT_APP_PART_ADDR).
*/
static int ota_app_vtor_reconfig(void)
{
    #define NVIC_VTOR_MASK   0x3FFFFF80
    /* Set the Vector Table base location by user application firmware definition */
    SCB->VTOR = RT_APP_PART_ADDR & NVIC_VTOR_MASK;

    return 0;
}
INIT_BOARD_EXPORT(ota_app_vtor_reconfig);

int main(void)
{
    rt_thread_t pthread = RT_NULL;
    rt_err_t result = RT_EOK;
    
    rt_kprintf("The current version of firmware: %s\n", APP_VERSION);
    
    rt_memset(&env, 0x00, sizeof(struct guardian_environment));
    
    for (int i = 0; i < SBUS_CHANNEL_NUMBER; i++)
    {
        env.ch_value[i] = SBUS_VALUE_MEDIAN;
        env.ch_status[i] = SBUS_IDLE;
    }
    
    /* set LED0 pin mode to output */
    rt_pin_mode(LED_RUN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_SBUS_PIN, PIN_MODE_OUTPUT);
    
    rt_pin_mode(MODE_PIN, PIN_MODE_INPUT);

    pthread = rt_thread_create("tCamera", camera_resolving_entry, &env, 2048, 9, 50);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    pthread = rt_thread_create("tPTZ", pantilt_resolving_entry, &env, 2048, 6, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    pthread = rt_thread_create("tTrack", track_resolving_entry, &env, 2048, 9, 20);
    RT_ASSERT(pthread != RT_NULL);
    result = rt_thread_startup(pthread);
    RT_ASSERT(result == RT_EOK);
    
    if (rt_pin_read(MODE_PIN) == 1)
    {
        rt_kprintf("Mode Select: SBUS\n");
        pthread = rt_thread_create("tSBus", sbus_resolving_entry, &env, 2048, 3, 20);
        RT_ASSERT(pthread != RT_NULL);
        result = rt_thread_startup(pthread);
        RT_ASSERT(result == RT_EOK);
    }
    else
    {
        rt_kprintf("Mode Select: UART\n");
        pthread = rt_thread_create("tZINGTO", zingto_resolving_entry, &env, 2048, 3, 20);
        RT_ASSERT(pthread != RT_NULL);
        result = rt_thread_startup(pthread);
        RT_ASSERT(result == RT_EOK);
    }
    
    rt_thread_delay(RT_TICK_PER_SECOND);
    
    env.cam_tflogo = RT_TRUE;
    env.cam_blankvideo = RT_FALSE;
    env.sbus_incharge = RT_FALSE;
    
    env.ptz_mode = PANTILT_MODE_HEADFREE;
    
    rt_kprintf("[%d] Waiting 15S... \n");
    
    rt_thread_delay(RT_TICK_PER_SECOND * 15);
    
    rt_kprintf("[%d] Start Comunication with Sumboy \n");
    
    rt_event_send(env.ev_camera, CAMERA_CMD_ZOOM_GETPOS); // notify the Camera.
    
    while(RT_TRUE)
    {
        
        if (env.trck_incharge == RT_TRUE)
            rt_thread_delay(RT_TICK_PER_SECOND / 5);
        else
            rt_thread_delay(RT_TICK_PER_SECOND / 5);

//        rt_kprintf("%4d %4d %4d %4d %4d %4d %4d %4d %4d\r", \
//                    env.ch_value[0], env.ch_value[1], env.ch_value[2], env.ch_value[3], env.ch_value[4], \
//                    env.ch_value[5], env.ch_value[6], env.ch_value[7], env.ch_value[8]);
        
        // automatic refresh the ptz angle.
        env.ptz_action = PANTILT_ACTION_SHOWANGLE;
        rt_sem_release(env.sh_ptz);
        
        if (rt_pin_read(LED_RUN_PIN)) {
            rt_pin_write(LED_RUN_PIN, PIN_LOW);
        }
        else {
            rt_pin_write(LED_RUN_PIN, PIN_HIGH);
        }            
        
    }
    
    // never be here.
}
