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

#include <adis16334.h>
#include "AHRSHelper.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG //DBG_INFO
#define DBG_SECTION_NAME  "app.main"
#define DBG_COLOR
#include <rtdbg.h>

void sensor_rx_thread_entry(void * args);
void can_rx_thread_handler(void *args);

int main(void)
{
    static ImuAHRS_t iAHRS;
    rt_thread_t ptr = RT_NULL;
    
    rt_memset(&iAHRS, 0, sizeof(ImuAHRS_t));
    iAHRS.halfT = (1.0f / ADIS_SAMPLE_RATE) / 2.f;
    iAHRS.mutex = rt_mutex_create("mux_AHRSupdate", RT_IPC_FLAG_FIFO);
    if (iAHRS.mutex == RT_NULL) {
        LOG_E("create mutex \"mux_AHRSupdate\" failed!");
        return -RT_ERROR;
    }
    
    /* start AHRS thread */
    ptr = rt_thread_create("app_AHRScalc", sensor_rx_thread_entry, &iAHRS, 4096, 5, 20);
    if (ptr == RT_NULL) {
        LOG_E("create thread \"app_AHRScalc\" failed!");
        return -RT_ERROR;
    }
    
    rt_thread_startup(ptr);
    
    /* start AHRS thread */
    ptr = rt_thread_create("app_CANclient", can_rx_thread_handler, &iAHRS, 4096, 6, 20);
    if (ptr == RT_NULL) {
        LOG_E("create thread \"app_CANclient\" failed!");
        return -RT_ERROR;
    }
    
    rt_thread_startup(ptr);

    while(1)
    {
        rt_thread_delay(RT_TICK_PER_SECOND / 10);
        
//        rt_mutex_take(iAHRS.mutex, RT_WAITING_FOREVER);
//        Q_ToEularGeneral(&iAHRS.QS, &iAHRS.eular);
//        rt_mutex_release(iAHRS.mutex);
//        
//        rt_kprintf("%c[2K", 27);
//        //rt_kprintf("%+08d\t%+08d\t%+08d\r", (rt_int32_t)(iAHRS.gyro.X * 1000), (rt_int32_t)(iAHRS.gyro.Y * 1000), (rt_int32_t)(iAHRS.gyro.Z * 1000));
//        rt_kprintf("%+08d\t%+08d\t%+08d\r", (rt_int32_t)ToDeg(iAHRS.eular.pitch*100), (rt_int32_t)ToDeg(iAHRS.eular.roll*100), (rt_int32_t)ToDeg(iAHRS.eular.yaw*100));
    }

    return -RT_ERROR;
}
