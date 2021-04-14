/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 */

#ifndef __DRV_SBUS_H__
#define __DRV_SBUS_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>

#define SBUS_CHANNEL_COUNT  (16)

#define SBUS_MID_VALUE  1024
#define SBUS_TOLERANCE  50
#define SBUS_MAX_RANGE  700.f

typedef struct sbus_threshold
{
    rt_uint16_t min;
    rt_uint16_t low;
    rt_uint16_t high;
    rt_uint16_t max;
}sbus_thr_t;

typedef enum sbus_status
{
    SBUS_INITIAL = 0,
    SBUS_LOW,
    SBUS_MID,
    SBUS_HIGH,
    SBUS_INVAILD,
}sbus_status_t;

typedef void (*sbus_indicator)(rt_uint16_t *val, rt_uint32_t flag);

void sbus_set_update_callback(sbus_indicator callback);
sbus_indicator sbus_get_update_callback(void);

float sbus_normalize_value(rt_uint16_t value);

#endif /* __BOARD_H__ */
