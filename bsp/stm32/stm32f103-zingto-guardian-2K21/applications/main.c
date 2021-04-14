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

#include "drv_sbus.h"
#include "launcher.h"

#ifdef PKG_USING_AGILE_LED
#include <agile_led.h>
#endif

/* defined the LED pin: PB0 PB1*/
#define LED_BLINK_PIN    GET_PIN(B, 1)

#define MODE_PIN         GET_PIN(B, 3)

#define PRODUCT_VERSION  "GP090"

#define RT_APP_PART_ADDR    0x08020000

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
//INIT_BOARD_EXPORT(ota_app_vtor_reconfig);

int main(void)
{
    agile_led_t *blink = RT_NULL;
    
    rt_kprintf("PRODUCT_VERSION: %s\n", PRODUCT_VERSION);
    
    blink = agile_led_create(LED_BLINK_PIN, PIN_HIGH, "100,900", -1);
    RT_ASSERT(blink != RT_NULL);
    agile_led_start(blink);
    
    while(RT_TRUE) {
        rt_thread_delay(RT_TICK_PER_SECOND);
    }
    
    // never be here.
}

static int SYS_cmdInquryVersion(void *parameter){
    struct cmd_ack *ack = (struct cmd_ack*)parameter;
    
    if (ack == RT_NULL) return -RT_EINVAL;
    if (ack->buff == RT_NULL) return -RT_ENOMEM;
    
    ack->bufsz = sizeof(PRODUCT_VERSION) - 1;
    rt_memcpy(ack->buff, PRODUCT_VERSION, ack->bufsz);
    return RT_EOK;
}
LAUNCHER_DYNACALL_EXPORT_CMD_FAST(SYS_cmdInquryVersion)
