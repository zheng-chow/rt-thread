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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define     NSS     GET_PIN(A, 4)
#define     SCK     GET_PIN(A, 5)
#define     MIO     GET_PIN(A, 7)

#define DBG_ENABLE
#define DBG_SECTION_NAME "icplus"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

static rt_uint8_t icplus_smi_init_flag = RT_FALSE;
static const char *MIB_desc[] = {
    "64B",
    "65_127B",
    "128_255B",
    "256_511B",
    "512_1023B",
    "1024_1518B",
    "Oversize",
    "Bcst",
    "Mcst",
    "Ucst",
    "Pause",
    "Pkt",
    "Byte_L",
    "Byte_H",
    "Drop(buff)",
    "Drop(other)",
    "CRC",
    "Alignment",
    "Runt",
    "Frag",
    "Jabber",
    "Symbol_Error",
    "ACL",
    "ACL"
};

/**
 * This function initializes the smi pin.
 *
 * @param void.
 */
static void icplus_smi_init(void)
{
    if (icplus_smi_init_flag)
        return;
    
    LOG_I("icplus smi init for 2-wire mode");
    
    rt_pin_mode(NSS, PIN_MODE_OUTPUT);
    rt_pin_write(NSS, PIN_HIGH);
    
    rt_pin_mode(SCK, PIN_MODE_OUTPUT);
    rt_pin_mode(MIO, PIN_MODE_OUTPUT_OD);
       
    rt_pin_write(SCK, PIN_LOW);
    rt_pin_write(MIO, PIN_HIGH);
    
    icplus_smi_init_flag = RT_TRUE;
}

/**
 * This function sets the SCK pin.
 *
 * @param The SCK pin state.
 */
static void smi_set_sck(rt_int32_t state)
{
    if (state)
    {
        rt_pin_write(SCK, PIN_HIGH);
    }
    else
    {
        rt_pin_write(SCK, PIN_LOW);
    }
}

/**
 * This function sets the MIO pin.
 *
 * @param The MIO pin state.
 */
static void smi_set_mio(rt_int32_t state)
{
    if (state)
    {
        rt_pin_write(MIO, PIN_HIGH);
    }
    else
    {
        rt_pin_write(MIO, PIN_LOW);
    }
}

/**
 * This function gets the MIO pin state.
 *
 * @param The MIO pin state.
 */

static rt_int32_t smi_get_mio(void)
{
    return rt_pin_read(MIO);
}

/**
 * The time delay function.
 *
 * @param microseconds.
 */
static void stm32_udelay(rt_uint32_t us)
{
    rt_uint32_t ticks;
    rt_uint32_t told, tnow, tcnt = 0;
    rt_uint32_t reload = SysTick->LOAD;

    ticks = us * reload / (1000000 / RT_TICK_PER_SECOND);
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

#define SMI_START       0x05    // 0b101
#define SMI_OPREAD      0x02    // 0b10
#define SMI_OPWRITE     0x01    // 0b01
#define SMI_CHIPID      0x00    // 0b00

/**
 * This function gets the register value.
 *
 * @param The register address.
 */
rt_uint16_t smi_read_reg(rt_uint8_t addr)
{
    rt_uint16_t opcfg = 0;
    rt_uint16_t value = 0;
    
    opcfg = (SMI_START << 13) + (SMI_OPREAD << 11) + (SMI_CHIPID << 9) + (addr << 1);
    
    /* write parameter code to chip */
    for (int i = 0; i < 16; i++)
    {
        rt_uint16_t bval = opcfg & (0x0001 << (15 - i));
        if (bval)
            smi_set_mio(PIN_HIGH);
        else
            smi_set_mio(PIN_LOW);
        
        stm32_udelay(1);
        smi_set_sck(PIN_HIGH);
        
        stm32_udelay(1);
        smi_set_sck(PIN_LOW);
        
    }
    
    /* mio input mode */
    smi_set_mio(PIN_HIGH);
    
    /* read register value from chip */
    for (int i = 0; i < 16; i++)
    {
        stm32_udelay(1);  
        smi_set_sck(PIN_HIGH);
        stm32_udelay(1);
        
        if (smi_get_mio())
            value += 0x01 << (15 - i);
        
        smi_set_sck(PIN_LOW);
    }
    
    smi_set_mio(PIN_LOW);
    stm32_udelay(1);
    smi_set_sck(PIN_HIGH);
    stm32_udelay(1);
    smi_set_sck(PIN_LOW);    
    
    return value;
}

/**
 * This function write the register value.
 *
 * @param The register address.
 */
void smi_write_reg(rt_uint8_t addr, rt_uint16_t value)
{
    rt_uint16_t opcfg = 0;
    
    opcfg = (SMI_START << 13) + (SMI_OPWRITE << 11) + (SMI_CHIPID << 9) + (addr << 1) + 1;
    
    /* write parameter code to chip */
    for (int i = 0; i < 16; i++)
    {
        rt_uint16_t bval = opcfg & (0x0001 << (15 - i));
        if (bval)
            smi_set_mio(PIN_HIGH);
        else
            smi_set_mio(PIN_LOW);
        
        stm32_udelay(1);
        smi_set_sck(PIN_HIGH);
        
        stm32_udelay(1);
        smi_set_sck(PIN_LOW);
        
    }
    
    /* mio input mode */
    smi_set_mio(PIN_LOW);
    stm32_udelay(1);
    smi_set_sck(PIN_HIGH);
    stm32_udelay(1);
    smi_set_sck(PIN_LOW);   
    
    /* read register value from chip */
    for (int i = 0; i < 16; i++)
    {
        rt_uint16_t bval = value & (0x0001 << (15 - i));
        if (bval)
            smi_set_mio(PIN_HIGH);
        else
            smi_set_mio(PIN_LOW);
        
        stm32_udelay(1);  
        smi_set_sck(PIN_HIGH);
        
        stm32_udelay(1);        
        smi_set_sck(PIN_LOW);
    } 
    
    return;
}

static void icplus(int argc, char *argv[])
{
    static rt_uint8_t page = 0;

    icplus_smi_init();
    
    if (argc > 1)
    {
        if (!rt_strcmp(argv[1], "read"))
		{
			if (argc > 2)
			{
                
                rt_uint32_t regid, reval; 
                sscanf(argv[2], "%2x", &regid);
                reval = smi_read_reg(regid);
				
				rt_kprintf("read 0x%04X from reg 0x%02X, page 0x%02X.\n", reval, regid, page);
			}
			else
			{
				rt_kprintf("icplus read <reg>           - read register value\n");
			}
        }
        else if (!rt_strcmp(argv[1], "write"))
        {
            if (argc > 3)
            {
                rt_uint32_t regid, stval;
                sscanf(argv[2], "%2x", &regid);
                sscanf(argv[3], "%4x", &stval);
                
                smi_write_reg(regid, stval);
                
                rt_kprintf("write 0x%04X to reg 0x%02X, page 0x%02X.\n", stval, regid, page);
                if (regid == 0xFF)
                    page = stval;
            }
            else
            {
                rt_kprintf("icplus write <reg> <val>    - write register value\n");
            }
        }
        else if (!rt_strcmp(argv[1], "MIB"))
        {
            if (argc > 2)
            {
                if (!rt_strcmp(argv[2], "on"))
                {
                    smi_write_reg(0xFF, 0x0000);
                    rt_uint16_t stval = 0x7E57;
                    smi_write_reg(0x01, stval);
                    rt_kprintf("write 0x%04X to reg 0x%02X, page 0x%02X.\n", stval, 0x01, 0x00);
                    if (smi_read_reg(0x01) & (1 << 9))
                        LOG_I("MIB function turn on!");
                    else
                        LOG_E("Failed!\n");
                    smi_write_reg(0xFF, page);
                }
                else if (!rt_strcmp(argv[2], "off"))
                {
                    smi_write_reg(0xFF, 0x0000);
                    rt_uint16_t stval = 0x7E57 & ~(1 << 9);
                    smi_write_reg(0x01, stval);
                    rt_kprintf("write 0x%04X to reg 0x%02X, page 0x%02X.\n", stval, 0x01, 0x00);
                    if (~(smi_read_reg(0x01) & (1 << 9)))
                        LOG_I("MIB function turn off!");
                    else
                        LOG_E("Failed!\n");
                    smi_write_reg(0xFF, page);
                }
                else if (!rt_strcmp(argv[2], "dump"))
                {
                    rt_uint32_t *rxcntr = RT_NULL;
                    rt_uint32_t *txcntr = RT_NULL;
                    rt_uint32_t port = 18;
                    rt_uint32_t portaddr = 0;
                    rt_uint16_t stval = 0;
                    
                    if (argc > 3)
                    {
                        port = atoi(argv[3]);
                        
                        if (port > 18)
                            portaddr = 28;
                        else if (port > 12)
                            portaddr = port + 7;
                        else
                            portaddr = port - 1;
                    }
                    
                    rxcntr = rt_malloc(0x18 * sizeof(rt_uint32_t));
                    txcntr = rt_malloc(0x18 * sizeof(rt_uint32_t));
                    RT_ASSERT(rxcntr != RT_NULL);
                    RT_ASSERT(txcntr != RT_NULL);
                    
                    rt_memset(rxcntr, 0xCC, 0x18 * sizeof(rt_uint32_t));
                    rt_memset(txcntr, 0xCC, 0x18 * sizeof(rt_uint32_t));
                    
                    smi_write_reg(0xFF, 0x0000);
                    rt_kprintf("Port %d(%d) MIB counter\n", port, portaddr);
                    
                    for (int i = 0; i < 0x18; i++)
                    {
                        
                        stval = (1 << 15) | (portaddr << 5) | i;
                        smi_write_reg(0x64, stval); // read RX;
                        
                        *(rxcntr + i) = smi_read_reg(0x66);
                        *(rxcntr + i) = *(rxcntr + i) << 16;
                        *(rxcntr + i) = *(rxcntr + i) + smi_read_reg(0x65);
                        
                        rt_kprintf("#%-12s | %10d(RX) | ", MIB_desc[i], *(rxcntr + i));
                        
                        if (i < 0x13 + 1)
                        {
                            stval = stval | (1 << 10);
                            smi_write_reg(0x64, stval); // read TX;
                            
                            *(txcntr + i) = smi_read_reg(0x66);
                            *(txcntr + i) = *(txcntr + i) << 16;
                            *(txcntr + i) = *(txcntr + i) + smi_read_reg(0x65);
                            rt_kprintf("%10d(TX)\n", *(txcntr + i));
                        }
                        else
                            rt_kprintf("\n");
                        
                        
                    }
                    
                    rt_free(rxcntr);
                    rt_free(txcntr);
                    
                    smi_write_reg(0xFF, page);
                    rt_kprintf("Dump finished\n");
                }
                else
                {
                    rt_kprintf("icplus MIB <on/off/dump>    - MIB is used to packet diagnosis\n");
                }
            }
            else
            {
                rt_kprintf("icplus MIB <on/off/dump>    - MIB is used to packet diagnosis\n");
            }
        }
        else if (!rt_strcmp(argv[1], "reset"))
        {
            rt_pin_write(GET_PIN(B, 13), PIN_LOW);
            rt_thread_delay(10);
            rt_pin_write(GET_PIN(B, 13), PIN_HIGH);
        }
        else
        {
            rt_kprintf("Unknown command. Please enter 'icplus' for help\n");
        }
	}
	else
	{
		rt_kprintf("Usage:\n");
        rt_kprintf("icplus read <reg>           - read register value\n");
        rt_kprintf("icplus write <reg> <val>    - write register value\n");
        rt_kprintf("icplus MIB <on/off/dump>    - MIB is used to packet diagnosis\n");
	}
}
MSH_CMD_EXPORT(icplus, utils for manage icplus switcher);
