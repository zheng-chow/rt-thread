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
#include <stdlib.h>

#include "at24cxx.h"

#define EEFW_I2C_BUS_NAME           "i2c1"
#define EEFW_I2C_SPECADDR           0
#define EEFW_MEM_SIZE               256

extern const unsigned char IP1819_FIRMWARE_BIN;
const unsigned char *ip1819fw = &IP1819_FIRMWARE_BIN;

void MemDmp(const void *adr, int len) 
{
    const char *hex_ch = "0123456789ABCDEF";
    const char *fmt    = "%08X:  %.8s %.8s %.8s %.8s  * %.16s *\n";
    const char *base_adr, *curr_adr;
    unsigned char  hex[16<<4], asc[16];

    base_adr = curr_adr = (const char *)adr;
    for (unsigned int ix=0, num=len/16, rem=len%16; ix <= num; ix++){
        unsigned int iy;
        rt_memset(hex,' ',16<<1);
        rt_memset(asc,' ',sizeof(asc));
        for (iy=0; iy < ((ix<num) ? 16 : rem); iy++) {
            hex[(iy<<1)+0] = hex_ch[0x0F&(base_adr[(ix<<4)+iy]>>4)];
            hex[(iy<<1)+1] = hex_ch[0x0F&(base_adr[(ix<<4)+iy])   ];
            asc[iy]=((unsigned char)base_adr[(ix<<4)+iy]<=' ') ? '.' : base_adr[(ix<<4)+iy];
        }
        if ((ix<num) || (rem>0)){
            hex[iy<<1]=' ';
            rt_kprintf(fmt, curr_adr - base_adr, hex + 0, hex + 8, hex + 16, hex + 24, asc);
            curr_adr += 16;
        }
    }
}

static void eefw(int argc, char *argv[])
{
    static at24cxx_device_t dev = RT_NULL;
    rt_uint8_t *pbuf = RT_NULL;

	if (!dev)
		dev = at24cxx_init(EEFW_I2C_BUS_NAME, EEFW_I2C_SPECADDR);
    
	if (argc > 1)
	{
		if (!strcmp(argv[1], "erase"))
		{
			if (argc > 3)
			{			
				rt_size_t offset = atoi(argv[2]);
				rt_size_t bufsz = atoi(argv[3]);
				
				pbuf = rt_malloc(bufsz);
				rt_memset(pbuf, 0xFF, bufsz);
				
				at24cxx_write(dev, offset, pbuf, bufsz);
				
				rt_free(pbuf);
				rt_kprintf("erase %d(%x) at %08X.\n", bufsz, bufsz, offset);
			}
			else
			{
				rt_kprintf("eefw erase <offset> <length> - erase data\n");
			}
		}
		else if (!strcmp(argv[1], "dump"))
		{	
			pbuf = rt_malloc(EEFW_MEM_SIZE);
			rt_memset(pbuf, 0x00, EEFW_MEM_SIZE);
			at24cxx_read(dev, 0, pbuf, EEFW_MEM_SIZE);
			
			MemDmp(pbuf, EEFW_MEM_SIZE);
			
			rt_free(pbuf);			
		}
        else if (!strcmp(argv[1], "fill"))
        {
            at24cxx_write(dev, 0, (rt_uint8_t *)ip1819fw, EEFW_MEM_SIZE);
        }
		else
        {
            rt_kprintf("Unknown command. Please enter 'eefw' for help\n");
        }
	}
	else
	{
		rt_kprintf("Usage:\n");
        rt_kprintf("eefw dump   							- dump eeprom firmware\n");
        rt_kprintf("eefw erase <offset(d)> <length(d)>		- erase eeprom firmware\n");
	}
}
MSH_CMD_EXPORT(eefw, utils for manage eeprom firmware);
