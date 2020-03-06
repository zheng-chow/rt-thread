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

/* defined the LED0 pin: PA0 */
#define LED0_PIN        GET_PIN(A, 0)
/* defined the LED1 pin: PA1 */
#define LED1_PIN        GET_PIN(A, 1)
/* defined the LANRST pin: PB13 */
#define LANRST_PIN      GET_PIN(B, 13)
/* defined the CUC_RST pin: PB14 */
#define CUCRST_PIN      GET_PIN(B, 14)
/* defined the EEPROM_WP pin: PB5 */
#define EE_WP_PIN		GET_PIN(B, 5)

extern const unsigned int IP1819_FIRMWARE_BIN;
const unsigned int *pfwbin = &IP1819_FIRMWARE_BIN;

extern void smi_write_reg(rt_uint8_t addr, rt_uint16_t value);

int main(void)
{
    int offset = 0;
    
    rt_uint32_t cmdcode;
    rt_uint8_t  page, regid;
    rt_uint16_t stval;
    
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    
    /* set RST pin mode to output */
    rt_pin_mode(LANRST_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(CUCRST_PIN, PIN_HIGH);
    rt_pin_mode(CUCRST_PIN, PIN_MODE_OUTPUT);
    
    rt_pin_mode(GET_PIN(A, 6), PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(A, 6), PIN_HIGH);
    
	/* set EEPROM WP disable */
    rt_pin_mode(EE_WP_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_write(EE_WP_PIN, PIN_LOW);
	
    /* reset all chips and units on PCB */
    rt_pin_write(LANRST_PIN, PIN_LOW);
    //rt_pin_write(CUCRST_PIN, PIN_LOW);
    rt_thread_mdelay(500);    
    rt_pin_write(LANRST_PIN, PIN_HIGH);
    
    rt_kprintf("\nNO.\tPAGE\tADDR\tVALUE\n");
    
    offset = 1;
    while(1)
    {
        cmdcode = *(pfwbin+ offset);
        page  = ( cmdcode & 0x000000FF );
        regid = ( cmdcode & 0x0000FF00 ) >> 8;
        stval = ( cmdcode & 0xFFFF0000 ) >> 16;
        
        if ((page == 0xFF) && (regid == 0xFF))
            break;
        
        rt_kprintf("#%02d\t0x%02X\t0x%02X\t0x%04X\n", offset, page, regid, stval);
        
        smi_write_reg(0xFF, page);
        smi_write_reg(regid, stval);
        
        offset++;
    }
    
    rt_kprintf("IP1819 Initialized\n");
    
    while(1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_mdelay(5);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_pin_write(LED1_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}
