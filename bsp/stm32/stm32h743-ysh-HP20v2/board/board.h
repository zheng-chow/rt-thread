/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-5      SummerGift   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#define PHY_USING_IP101G

#include <rtthread.h>
#include <stm32h7xx.h>
#include "drv_common.h"
#include "drv_gpio.h"
#include "drv_eth_h7.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STM32_FLASH_START_ADRESS     ((uint32_t)0x08000000)
#define STM32_FLASH_SIZE             (2048 * 1024)
#define STM32_FLASH_END_ADDRESS      ((uint32_t)(STM32_FLASH_START_ADRESS + STM32_FLASH_SIZE))

#define STM32_SRAM_BEGIN					(0x24000000)
#define STM32_SRAM_SIZE           (512)
#define STM32_SRAM_END            (STM32_SRAM_BEGIN + STM32_SRAM_SIZE * 1024)

#if defined(__CC_ARM) || defined(__CLANG_ARM)
extern int Image$$RW_IRAM1$$ZI$$Limit;
//#define HEAP_BEGIN      (&Image$$RW_IRAM1$$ZI$$Limit)
#define HEAP_BEGIN				STM32_SRAM_BEGIN
#elif __ICCARM__
#pragma section="CSTACK"
#define HEAP_BEGIN      (__segment_end("CSTACK"))
#else
extern int __bss_end;
#define HEAP_BEGIN      (&__bss_end)
#endif

#define HEAP_END        STM32_SRAM_END

void SystemClock_Config(void);

//wickkid++
//#define __HAL_RCC_ETH_CLK_ENABLE()  do{__HAL_RCC_ETH1MAC_CLK_ENABLE();__HAL_RCC_ETH1TX_CLK_ENABLE();__HAL_RCC_ETH1RX_CLK_ENABLE();}while(0)
#define ETH_MEDIA_INTERFACE_RMII 		HAL_ETH_RMII_MODE
#define ETH_MEDIA_INTERFACE_MII 		HAL_ETH_MII_MODE
//#define ETH_RXBUFNB									ETH_RX_DESC_CNT
//#define ETH_TXBUFNB									ETH_TX_DESC_CNT
#define ETH_MODE_FULLDUPLEX					ETH_FULLDUPLEX_MODE
#define ETH_MODE_HALFDUPLEX					ETH_HALFDUPLEX_MODE


#define LED0_PIN    GET_PIN(D, 3)
#define LED1_PIN    GET_PIN(G, 12)
#define LED2_PIN    GET_PIN(G, 13)
#define LED3_PIN    GET_PIN(G, 14)

#define BLINK_PIN   LED0_PIN
#define CONV_PIN    LED1_PIN
#define MQTT_PIN    LED2_PIN
#define WARN_PIN    LED3_PIN

//wickkid++ end
#ifdef __cplusplus
}
#endif

#endif
