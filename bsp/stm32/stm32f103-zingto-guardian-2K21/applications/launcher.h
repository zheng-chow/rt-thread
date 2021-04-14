/*
 * Copyright (c) 2006-2018, ZINGTO UAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-08     zhouzheng    first version
 */

#ifndef __LAUNCHER_H__
#define __LAUNCHER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG   //DBG_INFO
#define DBG_SECTION_NAME  "launcher"
#define DBG_COLOR
#include <rtdbg.h>

#define JSON_STRING_NULL        "null"

#include "dynacall.h"

struct json_ctrl_method
{
    char *type;
    char *port;
    char *spec;
};

struct json_ch_config
{
    char *name_desc;
    
    char *always_call_desc;
    char *value_vary_desc;
    
    char *status_high_desc;
    char *status_mid_desc;
    char *status_low_desc;
};

struct ch_config
{
    rt_off_t offset;
    
    rt_mailbox_t mb;
    rt_thread_t pid;
    
    rt_uint16_t thr_low;
    rt_uint16_t thr_high;

    dynamic_func always_call;
    dynamic_func value_vary;

    dynamic_func status_high;
    dynamic_func status_mid;
    dynamic_func status_low;
};

struct json_cmd_config
{
    rt_uint8_t cmd_code1;
    rt_uint8_t cmd_code2;
    rt_bool_t  ack_flag;
    char *exec_call_desc;
};

struct cmd_ack
{
    void *param;
    char *buff;
    int bufsz;
};

struct cmd_config
{
    rt_uint8_t cmd_code1;
    rt_uint8_t cmd_code2;
    rt_bool_t  ack_flag;
    dynamic_func exec_call;
};

#endif
