/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-27     zylx         first version
 */
 
#include <board.h>
#include <drv_qspi.h>
#include <rtdevice.h>
#include <rthw.h>
#include <finsh.h>

#ifdef BSP_USING_QSPI_FLASH

#include "spi_flash.h"
#include "spi_flash_sfud.h"
#include <dfs_fs.h>
#include <fal.h>

char w25qxx_read_status_register2(struct rt_qspi_device *device)
{
    /* 0x35 read status register2 */
    char instruction = 0x35, status;

    rt_qspi_send_then_recv(device, &instruction, 1, &status, 1);

    return status;
}

void w25qxx_write_enable(struct rt_qspi_device *device)
{
    /* 0x06 write enable */
    char instruction = 0x06;

    rt_qspi_send(device, &instruction, 1);
}

void w25qxx_enter_qspi_mode(struct rt_qspi_device *device)
{
    char status = 0;
    /* 0x38 enter qspi mode */
    char instruction = 0x38;
    char write_status2_buf[2] = {0};

    /* 0x31 write status register2 */
    write_status2_buf[0] = 0x31;

    status = w25qxx_read_status_register2(device);
    if (!(status & 0x02))
    {
        status |= 1 << 1;
        w25qxx_write_enable(device);
        write_status2_buf[1] = status;
        rt_qspi_send(device, &write_status2_buf, 2);
        rt_qspi_send(device, &instruction, 1);
        rt_kprintf("flash already enter qspi mode\n");
        rt_thread_mdelay(10);
    }
}

static int rt_hw_qspi_flash_with_sfud_init(void)
{
    stm32_qspi_bus_attach_device("qspi1", "qspi10", RT_NULL, 4, w25qxx_enter_qspi_mode, RT_NULL);
    
    /* init W25Q128 */
    if (RT_NULL == rt_sfud_flash_probe("W25Q128BV", "qspi10"))//W25Q128BV
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_qspi_flash_with_sfud_init);

#if defined(PKG_USING_FAL)
static int fal_load_init(void)
{
    return fal_init();
}
INIT_COMPONENT_EXPORT(fal_load_init);
#endif

#if defined(RT_USING_DFS_ELMFAT) && !defined(BSP_USING_SDCARD)
#include <dfs_fs.h>

#define BLK_DEV_NAME        "W25Q128BV"
#define FS_PARTITION_NAME   "rootfs"
int mnt_init(void)
{
    rt_device_t blk_nor = fal_blk_device_create(FS_PARTITION_NAME);
    if (!blk_nor) 
    {
        LOG_E("Can't create a blk device on '%s' partition.", FS_PARTITION_NAME);
    }

    if (dfs_mount(FS_PARTITION_NAME, "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("file system initialization done!\n");
    }
    else
    {
        if(dfs_mkfs("elm", FS_PARTITION_NAME) == 0)
        {
            if (dfs_mount(FS_PARTITION_NAME, "/", "elm", 0, 0) == 0)
            {
                rt_kprintf("file system initialization done!\n");
            }
            else
            {
                rt_kprintf("file system initialization failed!\n");
            }
        }
    }

    return 0;
}
INIT_ENV_EXPORT(mnt_init);

#endif /* defined(RT_USING_DFS_ELMFAT) && !defined(BSP_USING_SDCARD) */
#endif/* BSP_USING_QSPI_FLASH */
