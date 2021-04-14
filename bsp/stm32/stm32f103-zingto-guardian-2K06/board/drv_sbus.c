/*
 * Copyright (c) 2006-2018, ZINGTO UAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-03     serni        first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>

#define DBG_ENABLE
#define DBG_SECTION_NAME    "drv.sbus"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#define SBUS_IPC_SEMAPHORE  "shSBus"
#define SBUS_CHANNEL_COUNT  (16)
#define BIT_PER_CHANNEL     (11)

#define PKT_HEADER0         (0x0F)
#define PKT_HEADER1         (0x8F)
#define PKT_TAIL            (0x00)
#define PKT_TAIL0           (0x04)
#define PKT_TAIL1           (0x14)
#define PKT_TAIL2           (0x24)
#define PKT_TAIL3           (0x34)

#define HEADER_OFFSET       (00)
#define CHANNEL_OFFSET      (01)
#define FLAG_OFFSET         (23)
#define TAIL_OFFSET         (24)

#define SBUS_PKT_SIZE       (25)

#define SBUS_CH(x)          (x - 1)

/* defined the LED pin: PB0 */
#define LED_PIN             GET_PIN(B, 0)

static rt_sem_t semaph = RT_NULL;
static void (*sbus_update_callback)(rt_uint8_t size, rt_uint16_t *val) = RT_NULL;
static rt_uint16_t chval[SBUS_CHANNEL_COUNT];

static rt_err_t uart_pending_callback(rt_device_t dev, rt_size_t sz)
{
    rt_sem_release(semaph);
    
    return RT_EOK;
}

void _decode_packed_data(rt_uint8_t *rxbuf, rt_uint16_t *chval)
{
    rt_int8_t m = 0, n = 0;
    rt_int8_t currbit = 0, currbyte = 0;
    
    for (n = 0; n < SBUS_CHANNEL_COUNT; n++)
    {
        chval[n] = 0;
        for (m = 0; m < BIT_PER_CHANNEL; m++)
        {
            chval[n] |= (((rt_uint16_t)((rxbuf[CHANNEL_OFFSET + currbyte]) >> currbit))& 0x01)<< m;
            currbit++;
            if (currbit > 7)
            {
                currbit = 0;
                currbyte++;
            }
        }
    }
}

void sbus_background_worker(void* parameter)
{
    rt_device_t     dev = RT_NULL;
    rt_uint8_t      *pbuf = RT_NULL;
    rt_size_t       szbuf = 0;
    rt_uint16_t     rtval[SBUS_CHANNEL_COUNT];
    rt_err_t        result = RT_EOK;
    char            *device_name = parameter;
    
    RT_ASSERT(device_name != RT_NULL);
    // malloc 
    pbuf = rt_malloc(SBUS_PKT_SIZE); RT_ASSERT(pbuf != RT_NULL);
    
    dev = rt_device_find(device_name); RT_ASSERT(dev != RT_NULL);
    
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    
    semaph = rt_sem_create(SBUS_IPC_SEMAPHORE, 0, RT_IPC_FLAG_FIFO); RT_ASSERT(semaph != RT_NULL);
    
    // set uart4 in 100000, 9E2. compatible with SBUS protocol.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = 100000;
    config.data_bits = DATA_BITS_9;
    config.stop_bits = STOP_BITS_2;
    config.parity = PARITY_EVEN;
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &config);

    rt_device_set_rx_indicate(dev, uart_pending_callback);

    LOG_D("initialization finish, start!");
    
    while (1)
    {
        result = rt_sem_take(semaph, RT_TICK_PER_SECOND);
        
        if(result == -RT_ETIMEOUT) {
            rt_pin_write(LED_PIN, PIN_LOW);
            continue;
        }

        switch (szbuf){
        case 0:
        case 1:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, 1);
            if (!((pbuf[HEADER_OFFSET] == PKT_HEADER0) || (pbuf[HEADER_OFFSET] == PKT_HEADER1)))
                szbuf = 0;
            break;
        default:
            szbuf += rt_device_read(dev, 0, pbuf + szbuf, SBUS_PKT_SIZE - szbuf);
            break;
        }
        // should nerver happened.
        if (szbuf != SBUS_PKT_SIZE)
            continue;
        
        szbuf = 0;
        // check packet tail, vaild packet.
        if (!((pbuf[TAIL_OFFSET] == PKT_TAIL)  || (pbuf[TAIL_OFFSET] == PKT_TAIL0) ||      \
              (pbuf[TAIL_OFFSET] == PKT_TAIL1) || (pbuf[TAIL_OFFSET] == PKT_TAIL2) ||     \
              (pbuf[TAIL_OFFSET] == PKT_TAIL3))) {
            continue;
        }
        
        if (rt_pin_read(LED_PIN))
            rt_pin_write(LED_PIN, PIN_LOW);
        else
            rt_pin_write(LED_PIN, PIN_HIGH);

        _decode_packed_data(pbuf, rtval);
        
        for(rt_uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
            if(rtval[i] != chval[i]) {
                rt_memcpy(chval, rtval, SBUS_CHANNEL_COUNT * sizeof(rt_uint16_t));
                if(sbus_update_callback != RT_NULL) 
                    sbus_update_callback(SBUS_CHANNEL_COUNT, chval);
                break;
            }
        }
        
        // next sbus packet.
    }
    // never be here.
}

void sbus_set_update_callback(void (*ext_ind)(rt_uint8_t size, rt_uint16_t *val))
{
    sbus_update_callback = ext_ind;
}

#define SBUS_PORT_UART "uart4"
static int sbus_init(void)
{
    rt_thread_t thread = rt_thread_create("bgwSBus",
                      sbus_background_worker,
                      SBUS_PORT_UART,
                      2048, 10, 10);
    
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    else
        return -RT_ERROR;
    
    return RT_EOK;
}
INIT_ENV_EXPORT(sbus_init);
