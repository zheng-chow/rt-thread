/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-5      SummerGift   first version
 * 2020-5-17      yufanyufan77 support H7
 */

#ifndef __FAL_CFG_H__
#define __FAL_CFG_H__

#include <rtthread.h>
#include <board.h>

#define FAL_FLASH_PORT_DRIVER_SFUD

extern struct fal_flash_dev nor_flash0;

/* flash device table */
#define FAL_FLASH_DEV_TABLE                                         \
{                                                                   \
    &nor_flash0,                                                    \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG

/* partition table */
#define FAL_PART_TABLE                                                                                     \
{                                                                                                          \
    {FAL_PART_MAGIC_WROD,     "backup",    FAL_USING_NOR_FLASH_DEV_NAME,                  0,  1 * 1024 * 1024, 0}, \
    {FAL_PART_MAGIC_WROD, "filesystem",    FAL_USING_NOR_FLASH_DEV_NAME,    1 * 1024 * 1024, 15 * 1024 * 1024, 0}, \
}

#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */
