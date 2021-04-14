/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-5      SummerGift   first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtthread.h>
#include <board.h>

extern const struct fal_flash_dev stm32_onchip_flash_128k;
extern struct fal_flash_dev nor_flash0;


#define FLASH_SIZE_GRANULARITY_128K      (16 * 128 * 1024)
#define STM32_FLASH_START_ADDRESS_128K   (0x08000000)

/* flash device table */
#define FAL_FLASH_DEV_TABLE     \
{                               \
    &stm32_onchip_flash_128k,   \
    &nor_flash0,                \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG

/* partition table */
#define FAL_PART_TABLE      \
{                           \
    {FAL_PART_MAGIC_WORD,   "boot",              "onchip_flash_128k",   0, 1024 * 1024, 0}, \
    {FAL_PART_MAGIC_WORD,   "app",               "onchip_flash_128k",   1024 * 1024, 1024 * 1024, 0}, \
    {FAL_PART_MAGIC_WORD,   "rootfs",   FAL_USING_NOR_FLASH_DEV_NAME,   0,  16 * 1024 * 1024, 0}, \
}
#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */
