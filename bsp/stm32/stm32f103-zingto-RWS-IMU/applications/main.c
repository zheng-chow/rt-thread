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
#include <rm3100.h>
#include "AHRSHelper.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG //DBG_INFO
#define DBG_SECTION_NAME  "app.main"
#define DBG_COLOR
#include <rtdbg.h>

void accel_rx_thread_entry(void *args);
void can_rx_thread_handler(void *args);
void mag_rx_thread_entry(void *args);

int main(void)
{
    static ImuAHRS_t ahrs_st;
    rt_thread_t thread = RT_NULL;
    
    rt_memset(&ahrs_st, 0, sizeof(ImuAHRS_t));
    ahrs_st.halfT = (1.0f / ADIS_SAMPLE_RATE) / 2.f;
    ahrs_st.mutex = rt_mutex_create("mtx_AhrsFlush", RT_IPC_FLAG_FIFO);
    if (ahrs_st.mutex == RT_NULL) {
        LOG_E("create mutex \"muxAHRS\" failed!");
        return -RT_ERROR;
    }
    
    /* start thread for AHRS service */
    thread = rt_thread_create("bgserv_GeoMag", mag_rx_thread_entry, &ahrs_st, 4096, 5, 20);
    if (thread == RT_NULL) {
        LOG_E("create thread \"bgserv_GeoMag\" failed!");
        return -RT_ERROR;
    }
    rt_thread_startup(thread);
    /* start thread for AHRS service */
    thread = rt_thread_create("bgserv_AHRS", accel_rx_thread_entry, &ahrs_st, 4096, 5, 20);
    if (thread == RT_NULL) {
        LOG_E("create thread \"bgserv_AHRS\" failed!");
        return -RT_ERROR;
    }
    rt_thread_startup(thread);
    
    /* start thread for CAN client */
    thread = rt_thread_create("bgserv_CAN", can_rx_thread_handler, &ahrs_st, 4096, 6, 20);
    if (thread == RT_NULL) {
        LOG_E("create thread \"bgserv_CAN\" failed!");
        return -RT_ERROR;
    }
    rt_thread_startup(thread);

    while(1) {
        rt_thread_delay(RT_TICK_PER_SECOND);
    }

    return -RT_ERROR;
}

#define IMU_SPI_BUS    "spi1"
#define IMU_SPI_DEV    "spi10"

int adis16334_port(void)
{
    struct rt_sensor_config cfg;
    
    rt_hw_spi_device_attach(IMU_SPI_BUS, IMU_SPI_DEV, GPIOA, GPIO_PIN_4);
    
    cfg.intf.dev_name = IMU_SPI_DEV;
    cfg.intf.type = RT_SENSOR_INTF_SPI;
    cfg.intf.user_data = RT_NULL;

    cfg.irq_pin.pin = GET_PIN(B, 2);
    cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;
    
    cfg.mode = RT_SENSOR_MODE_INT;
    cfg.power = RT_SENSOR_POWER_NORMAL;
    
    rt_hw_adis16334_init("ADiS", &cfg);

    return 0;
}
INIT_APP_EXPORT(adis16334_port);

#define MAG_SPI_BUS    "spi2"
#define MAG_SPI_DEV    "spi20"

int rm3100_port(void)
{
    struct rt_sensor_config cfg;
    
    rt_hw_spi_device_attach(MAG_SPI_BUS, MAG_SPI_DEV, GPIOB, GPIO_PIN_12);
    
    cfg.intf.dev_name = MAG_SPI_DEV;
    cfg.intf.type = RT_SENSOR_INTF_SPI;
    cfg.intf.user_data = RT_NULL;

    cfg.irq_pin.pin = GET_PIN(C, 6);
    cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;

    cfg.mode = RT_SENSOR_MODE_INT;
    cfg.power = RT_SENSOR_POWER_NORMAL;
    
    rt_hw_rm3100_init("PNI", &cfg);

    return 0;
}
INIT_APP_EXPORT(rm3100_port);
