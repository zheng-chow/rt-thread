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

#define RT_FOTA_SW_VERSION  "1.0.1"

#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
#include <finsh.h>
#include <shell.h>
#endif

#define DBG_ENABLE
#define DBG_SECTION_NAME                    "main"

#ifdef RT_MAIN_DEBUG
#define DBG_LEVEL                           DBG_LOG
#else
#define DBG_LEVEL                           DBG_INFO
#endif

#define DBG_COLOR
#include <rtdbg.h>

const char zingto_logo_buf[] = "\
  ___________ _   _  _____ _______ ____  \n\
 |___  /_   _| \\ | |/ ____|__   __/ __ \\ \n\
    / /  | | |  \\| | |  __   | | | |  | |\n\
   / /   | | | . ` | | |_ |  | | | |  | |\n\
  / /__ _| |_| |\\  | |__| |  | | | |__| |\n\
 /_____|_____|_| \\_|\\_____|  |_|  \\____/ \n";
                                                            

const char boot_log_buf[] = 
" ____ _____      _____ ___  _____  _        \r\n\
|  _ \\_   _|	|  ___/ _ \\ _   _|/ \\      \r\n\
| |_) || |_____ | |_  || ||  | | / _ \\      \r\n\
|  _ < | |_____ |  _| ||_||  | |/ ___ \\     \r\n\
|_| \\_\\|_|	|_|   \\___/  |_/_/   \\_\\  \r\n";

void rt_fota_print_log(void)
{   
    LOG_RAW("%s\n", zingto_logo_buf);    
	LOG_RAW("2016 - 2019 Copyright by Radiation @ warfalcon\n");
	LOG_RAW("Version: %s build %s\n\n", RT_FOTA_SW_VERSION, __DATE__);
}

int main(void)
{    
#if defined(RT_USING_FINSH) && defined(FINSH_USING_MSH)
	finsh_set_prompt("zingto />");
#endif
    
    rt_fota_print_log();
    
    extern void rt_fota_init(void);
    rt_fota_init();

    return RT_EOK;
}
