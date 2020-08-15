/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_SECTION_NAME        "main"
#define DBG_LEVEL               DBG_INFO
//#define DBG_COLOR
#include <rtdbg.h>

#ifdef PKG_USING_FAL
#include <fal.h>
#endif

#ifdef RT_USING_DFS
#include <dfs_fs.h>
#define FS_PARTITION_NAME       "filesystem"
#endif

/* defined the LED0 pin: PE4 */
#define LED0_PIN    GET_PIN(E, 4)

int main(void)
{
    int count = 1;
    
    /* set LED0 pin mode to output */	
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);	

    #ifdef PKG_USING_FAL
    fal_init();
	#endif
    
    #ifdef RT_USING_DFS
    struct rt_device *mtd_dev = RT_NULL;
    
    mtd_dev = fal_mtd_nor_device_create(FS_PARTITION_NAME);
    if (!mtd_dev)
    {
        LOG_E("Can't create a mtd device on '%s' partition.", FS_PARTITION_NAME);
    }
    else
    {
        /* mount littlefs */
        if (dfs_mount(FS_PARTITION_NAME, "/", "lfs", 0, 0) == 0)
        {
            LOG_I("Filesystem initialized!");
        }
        else
        {
            /* format */
            dfs_mkfs("lfs", FS_PARTITION_NAME);
            /* mount littlefs */
            if (dfs_mount("filesystem", "/", "lfs", 0, 0) == 0)
            {
                LOG_I("Filesystem initialized!");
            }
            else
            {
                LOG_E("Failed to initialize filesystem!");
            }
        }
    }
    #endif
    
    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
    return RT_EOK;
}
