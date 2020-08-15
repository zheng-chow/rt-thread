/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-05     serni        first version
 */
 
#include <board.h>
#include <drv_spi.h>
#include <rtdevice.h>
#include <rthw.h>
#include <finsh.h>

#include "rn8302b.h"

#define METER_CHIP_NAME "rn8302b"

#define RN83_PM_PIN     GET_PIN(E, 2)
#define RN83_RST_PIN    GET_PIN(E, 4)

extern const struct rn8302b_reg_t RN8302B_REGMAP[];
extern const rt_size_t RN8302B_REGCOUNT;

static struct rt_spi_device * eem_spi_dev = RT_NULL;

static rt_err_t rn8302b_reg_read(rt_uint16_t addr, rt_uint8_t *pbuf, rt_uint8_t size)
{
    struct rt_spi_message msg1, msg2;
    rt_uint8_t cmd[2] = {0};
    
    RT_ASSERT(eem_spi_dev != RT_NULL);
    RT_ASSERT(pbuf != RT_NULL);
    
    cmd[0] = (addr & 0x00FF);               // reg
    cmd[1] = (addr & 0x0F00) >> 4 | 0 << 8; // bank + R'0
    
    msg1.send_buf = cmd;
    msg1.recv_buf = RT_NULL;
    msg1.length = 2;
    msg1.cs_take = 1;
    msg1.cs_release = 0;
    msg1.next = &msg2;
    
    msg2.send_buf = RT_NULL;
    msg2.recv_buf = pbuf;
    msg2.length = size;
    msg2.cs_take = 0;
    msg2.cs_release = 1;
    msg2.next = RT_NULL;

    rt_spi_transfer_message(eem_spi_dev, &msg1);
    
    return RT_EOK;
}

static rt_err_t rn8302b_reg_write(rt_uint16_t addr, rt_uint8_t *pbuf, rt_uint8_t size)
{
    struct rt_spi_message msg1;
    rt_uint8_t cmd[2] = {0};
    
    RT_ASSERT(eem_spi_dev != RT_NULL);
    RT_ASSERT(pbuf != RT_NULL);
    
    cmd[0] = (addr & 0x00FF);               // reg
    cmd[1] = (addr & 0x0F00) >> 4 | 1 << 8; // bank + W'1
    
    msg1.send_buf = cmd;
    msg1.recv_buf = RT_NULL;
    msg1.length = 2 + size;
    msg1.cs_take = 1;
    msg1.cs_release = 1;
    msg1.next = RT_NULL;
    
    rt_spi_transfer_message(eem_spi_dev, &msg1);
    
    return RT_EOK;
}

rt_err_t rn8302b_device_init(rt_device_t dev)
{
    return RT_EOK;
}

rt_err_t rn8302b_device_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

rt_err_t rn8302b_device_close(rt_device_t dev)
{
    return RT_EOK;
}

rt_size_t rn8302b_device_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_off_t addr;
    rt_size_t r_sz;
    
    if (buffer == RT_NULL)
        return 0;
    
    if (pos > RN8302B_REGCOUNT)
        return 0;
    
    addr = RN8302B_REGMAP[pos].addr;
    r_sz = RN8302B_REGMAP[pos].size;
    
    rn8302b_reg_read(addr, buffer, r_sz);
    
    return r_sz;
}

rt_size_t rn8302b_device_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    return 0;
}

rt_err_t rn8302b_device_control(rt_device_t dev, int cmd, void *args)
{
    
    
    return RT_EOK;
}

static int rt_hw_spi_rn8302b_init(void)
{
    rt_hw_spi_device_attach("spi1", "spi10", GPIOG, GPIO_PIN_10);
    
    eem_spi_dev = (struct rt_spi_device *)rt_device_find("spi10");
    
    eem_spi_dev->config.data_width = 8;
    eem_spi_dev->config.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;
    eem_spi_dev->config.max_hz = 4 * 1000 *1000;       /* 2M */

    rt_pin_mode(RN83_PM_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(RN83_RST_PIN, PIN_MODE_OUTPUT);
    
    rt_pin_write(RN83_PM_PIN, PIN_LOW);
    
    rt_thread_delay(10);
    rt_pin_write(RN83_RST_PIN, PIN_LOW);
    rt_thread_delay(10);
    rt_pin_write(RN83_RST_PIN, PIN_HIGH);
    
    rt_device_t dev = rt_device_create(RT_Device_Class_Char, 0);
    
    RT_ASSERT(dev != RT_NULL);
    
    dev->init = rn8302b_device_init;
    dev->open = rn8302b_device_open;
    dev->close = rn8302b_device_close;
    dev->read = rn8302b_device_read;
    dev->write = rn8302b_device_write;
    dev->control = rn8302b_device_control;
    
    return rt_device_register(dev, METER_CHIP_NAME, RT_DEVICE_FLAG_RDWR);
}
INIT_DEVICE_EXPORT(rt_hw_spi_rn8302b_init);

int rn83_test(int argc, char *argv[])
{
    rt_kprintf("rn8302b_test\n");

    rt_uint8_t id[4] = {0};

    rn8302b_reg_read(0x018F, id, 4);

    rt_kprintf("RN8302B 0x8F : %02X %02X %02X %02X\n", id[0], id[1], id[2], id[3]);

    return 0;
}
MSH_CMD_EXPORT(rn83_test, test the rn8302b on spi1);
