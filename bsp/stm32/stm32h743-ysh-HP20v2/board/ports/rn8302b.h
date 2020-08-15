/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-04     zylx         The first version for STM32F4xx
 */

#ifndef __RN8302B_H__
#define __RN8302B_H__

#include <board.h>

struct rn8302b_reg_t
{
    rt_off_t  addr;
    char *desc;
    rt_size_t size;
    rt_bool_t writeable;
};

#define RN8302B_ADC_SET_FREQ

#endif
