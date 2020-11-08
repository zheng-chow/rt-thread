/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <math.h>

/* defined the LED0 pin: PE4 */
#define LED0_PIN    GET_PIN(E, 4)

#define MIN_RX_INTERVAL     (5)         // 5ms timeout
#define RXBUFF_SIZE         (128)

static rt_device_t  pUart2 = RT_NULL;
static rt_sem_t     sUart2 = RT_NULL;
static rt_device_t  pUart3 = RT_NULL;
static rt_sem_t     sUart3 = RT_NULL;
static rt_device_t  pUart6 = RT_NULL;
static rt_sem_t     sUart6 = RT_NULL;

static rt_uint8_t uart2_rx_stack[ 1024 ];
static struct rt_thread uart2_rx_thread;
static rt_uint8_t uart3_rx_stack[ 1024 ];
static struct rt_thread uart3_rx_thread;
static rt_uint8_t uart6_rx_stack[ 1024 ];
static struct rt_thread uart6_rx_thread;

static rt_err_t rx2_hook(rt_device_t dev, rt_size_t sz)
{
	rt_sem_release(sUart2);
	return RT_EOK;
}
static rt_err_t rx3_hook(rt_device_t dev, rt_size_t sz)
{
	rt_sem_release(sUart3);
	return RT_EOK;
}
static rt_err_t rx6_hook(rt_device_t dev, rt_size_t sz)
{
	rt_sem_release(sUart6);
	return RT_EOK;
}

static void uart2_rx_entry(void* parameter)
{
	rt_err_t retval=RT_EOK;
	rt_size_t szbuf=0;
	rt_uint8_t *pbuf=RT_NULL;
	
	pUart2 = rt_device_find("uart2");
  RT_ASSERT(pUart2 != RT_NULL);
  rt_device_open(pUart2, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
	
	sUart2 = rt_sem_create("u2rx", 0, RT_IPC_FLAG_FIFO);
  rt_device_set_rx_indicate(pUart2, rx2_hook);
	
	pbuf = rt_malloc(RXBUFF_SIZE);
	rt_memset(pbuf, 0x00, RXBUFF_SIZE);
	
	while(RT_TRUE)
	{
		retval=rt_sem_take(sUart2,MIN_RX_INTERVAL);
		if(retval != -RT_ETIMEOUT) {
			szbuf += rt_device_read(pUart2, 0, pbuf + szbuf, 1);
			continue;
		}
		else {
			if (szbuf == 0)
				continue;
		}
		if(pbuf[0]==0xAA)
		{
			rt_kprintf("uart2 to uart6! \n");
			rt_device_write(pUart6, 0, pbuf, szbuf);
		}
		else{
			rt_kprintf("uart2 to uart3! \n");
			rt_device_write(pUart3, 0, pbuf, szbuf);
		}

		szbuf = 0;
	}
}

static void uart3_rx_entry(void* parameter)
{
	rt_err_t retval = RT_EOK;
	rt_size_t szbuf = 0;
	rt_uint8_t *pbuf = RT_NULL;

	pUart3 = rt_device_find("uart3");
	RT_ASSERT(pUart3 != RT_NULL);
	rt_device_open(pUart3, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

	sUart3 = rt_sem_create("u3rx", 0, RT_IPC_FLAG_FIFO);
	rt_device_set_rx_indicate(pUart3, rx3_hook);

	pbuf = rt_malloc(RXBUFF_SIZE);
	rt_memset(pbuf, 0x00, RXBUFF_SIZE);

	while(RT_TRUE)
	{
		retval=rt_sem_take(sUart3,MIN_RX_INTERVAL);
		if(retval != -RT_ETIMEOUT) {
			szbuf += rt_device_read(pUart3, 0, pbuf + szbuf, 1);
			continue;
		}
		else {
			if (szbuf == 0)
				continue;
		}
		rt_kprintf("uart3 to uart2! \n");
		rt_device_write(pUart2, 0, pbuf, szbuf);

		szbuf = 0;
	}
}

static void uart6_rx_entry(void* parameter)
{
	rt_err_t retval = RT_EOK;
	rt_size_t szbuf = 0;
	rt_uint8_t *pbuf = RT_NULL;

	pUart6 = rt_device_find("uart6");
	RT_ASSERT(pUart6 != RT_NULL);
	rt_device_open(pUart6, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

	sUart6 = rt_sem_create("u6rx", 0, RT_IPC_FLAG_FIFO);
	rt_device_set_rx_indicate(pUart6, rx6_hook);

	pbuf = rt_malloc(RXBUFF_SIZE);
	rt_memset(pbuf, 0x00, RXBUFF_SIZE);

	while(RT_TRUE)
	{
		retval=rt_sem_take(sUart6,MIN_RX_INTERVAL);
		if(retval != -RT_ETIMEOUT) {
			szbuf += rt_device_read(pUart6, 0, pbuf + szbuf, 1);
			continue;
		}
		else {
			if (szbuf == 0)
				continue;
		}
		rt_kprintf("uart6 to uart2! \n");
		rt_device_write(pUart2, 0, pbuf, szbuf);

		szbuf = 0;
	}
}

int main(void)
{
	rt_err_t retval = RT_EOK;

	/* init uart2 rx thread */
	retval = rt_thread_init(&uart2_rx_thread,
													"tRX2",
													uart2_rx_entry,
													RT_NULL,
													(rt_uint8_t*)&uart2_rx_stack[0],
													sizeof(uart2_rx_stack), 12, 5);
	if (retval == RT_EOK)
	{
		rt_kprintf("thread \"uart2_rx\"start.\n");
		rt_thread_startup(&uart2_rx_thread);
	}
	/* init uart3 rx thread */
	retval = rt_thread_init(&uart3_rx_thread,
													"tRX3",
													uart3_rx_entry,
													RT_NULL,
													(rt_uint8_t*)&uart3_rx_stack[0],
													sizeof(uart3_rx_stack), 12, 5);
	if (retval == RT_EOK)
	{
		rt_kprintf("thread \"uart3_rx\"start.\n");
		rt_thread_startup(&uart3_rx_thread);
	}
	/* init uart6 rx thread */
	retval = rt_thread_init(&uart6_rx_thread,
													"tRX6",
													uart6_rx_entry,
													RT_NULL,
													(rt_uint8_t*)&uart6_rx_stack[0],
													sizeof(uart6_rx_stack), 12, 5);
	if (retval == RT_EOK)
	{
		rt_kprintf("thread \"uart6_rx\"start.\n");
		rt_thread_startup(&uart6_rx_thread);
	}
	/* set LED0 pin mode to output */
	rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	
	while (RT_TRUE)
	{
		rt_pin_write(LED0_PIN, PIN_HIGH);
		rt_thread_mdelay(500);
		rt_pin_write(LED0_PIN, PIN_LOW);
		rt_thread_mdelay(500);
	}

	// Never reach here.
}
