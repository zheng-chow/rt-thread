/*
 * Copyright (c) 2006-2018, ZINGTO UAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-03     serni        first version
 */

#include "drv_sbus.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME    "drv.sbus"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#define SBUS_IPC_EVENT      "evt_SBUS_Recv"
#define SBUS_IPC_MUTEX      "mtx_SBUS_Val"

#define BIT_PER_CHANNEL     (11)

#define PKT_HEADER0         (0x0F)
#define PKT_HEADER1         (0x8F)

#define PKT_TAIL_INIT       (0x00)
#define PKT_TAIL0           (0x04)
#define PKT_TAIL1           (0x14)
#define PKT_TAIL2           (0x24)
#define PKT_TAIL3           (0x34)
#define PKT_TAIL_LOST       (0x80)

#define HEADER_OFFSET       (00)
#define CHANNEL_OFFSET      (01)
#define FLAG_OFFSET         (23)
#define TAIL_OFFSET         (24)

#define SBUS_PKT_SIZE       (25)

#define SBUS_EVENT_RX_NEW_DATA  (1 << 0)
#define SBUS_EVENT_RX_NOT_EMPTY (1 << 1)

/* defined the LED pin: PB0 */
#define LED_PIN             GET_PIN(B, 0)

struct drv_sbus
{
    rt_event_t  event;
    rt_mutex_t  mutex;
    rt_device_t dev;
    sbus_indicator callback;
    rt_uint16_t buffer[SBUS_CHANNEL_COUNT];
}sbus_object;

static struct drv_sbus *sbus = &sbus_object;

static void decode_channel_value(rt_uint8_t *rxbuf, rt_uint16_t *chval)
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

static rt_err_t sbus_rx_indicator(rt_device_t dev, rt_size_t sz)
{
    rt_event_send(sbus->event, SBUS_EVENT_RX_NEW_DATA);
    return RT_EOK;
}

void sbus_background_worker(void* parameter)
{
    rt_uint8_t      *pbuf = RT_NULL;
    rt_size_t       szbuf = 0;
    rt_uint16_t     chval[SBUS_CHANNEL_COUNT];
    rt_err_t        result = RT_EOK;
    const char      *DEVICE_NAME = parameter;
    rt_uint32_t     flag;
    
    RT_ASSERT(DEVICE_NAME != RT_NULL);
    // malloc buffer
    pbuf = rt_malloc(SBUS_PKT_SIZE); RT_ASSERT(pbuf != RT_NULL);
    // find the UART device.
    sbus->dev = rt_device_find(DEVICE_NAME); RT_ASSERT(sbus->dev != RT_NULL);
    // open
    rt_device_open(sbus->dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_DMA_RX);
    // init event for UART callback.
    sbus->event = rt_event_create(SBUS_IPC_EVENT, RT_IPC_FLAG_FIFO); RT_ASSERT(sbus->event != RT_NULL);
    // init mutex.
    sbus->mutex = rt_mutex_create(SBUS_IPC_MUTEX, RT_IPC_FLAG_FIFO); RT_ASSERT(sbus->mutex != RT_NULL);
    
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    
    // set uart4 in 100000, 9E2. compatible with SBUS protocol.
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = 100000;
    config.data_bits = DATA_BITS_9;
    config.stop_bits = STOP_BITS_2;
    config.parity = PARITY_EVEN;
    rt_device_control(sbus->dev, RT_DEVICE_CTRL_CONFIG, &config);

    rt_device_set_rx_indicate(sbus->dev, sbus_rx_indicator);

    LOG_D("initialization finish, start!");
    
    while (1)
    {
        result = rt_event_recv(sbus->event, \
                                SBUS_EVENT_RX_NEW_DATA | SBUS_EVENT_RX_NOT_EMPTY, \
                                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, \
                                RT_TICK_PER_SECOND, RT_NULL);
        
        if(result == -RT_ETIMEOUT) {
            if (rt_pin_read(LED_PIN))
                rt_pin_write(LED_PIN, PIN_LOW);
            else
                rt_pin_write(LED_PIN, PIN_HIGH);
            continue;   // no data recv.
        }
       
        while (!szbuf) {    // check header.
            szbuf = rt_device_read(sbus->dev, 0, pbuf, 1);

            if ( (pbuf[HEADER_OFFSET] == PKT_HEADER0) ||\
                 (pbuf[HEADER_OFFSET] == PKT_HEADER1) || \
                 (szbuf == 0) )
                break;
            
            szbuf = 0;
        }

        if (szbuf == 0) {
            continue;   // can not find valid header.
        }
        else {
            rt_size_t count = rt_device_read(sbus->dev, 0, pbuf + szbuf, SBUS_PKT_SIZE - szbuf); // read the next bytes.
            
            if (count == SBUS_PKT_SIZE - szbuf)
                rt_event_send(sbus->event, SBUS_EVENT_RX_NOT_EMPTY);
            
            szbuf += count;
        }

        // check the packet receiving be complete.
        if (szbuf != SBUS_PKT_SIZE)
            continue;
        
        // check packet tail, valid packet.
        if (!((pbuf[TAIL_OFFSET] == PKT_TAIL_INIT)  || (pbuf[TAIL_OFFSET] == PKT_TAIL0) ||      \
              (pbuf[TAIL_OFFSET] == PKT_TAIL1) || (pbuf[TAIL_OFFSET] == PKT_TAIL2) ||     \
              (pbuf[TAIL_OFFSET] == PKT_TAIL3) || (pbuf[TAIL_OFFSET] == PKT_TAIL_LOST))) {
            LOG_E("data error");
            LOG_HEX("pbuf", 8, pbuf, szbuf);
            szbuf = 0;
            continue;
        }
        
        szbuf = 0;
        
        if (rt_pin_read(LED_PIN))
            rt_pin_write(LED_PIN, PIN_LOW);
        else
            rt_pin_write(LED_PIN, PIN_HIGH);

        decode_channel_value(pbuf, chval);
        
        flag = 0;
        for(rt_uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
            if(sbus->buffer[i] != chval[i]) {
                sbus->buffer[i] = chval[i];
                flag |= 1 << i;
            }
        }
        // channel value varying, call the handler.
        if (flag != 0) {
            if (sbus->callback) {
                sbus->callback(sbus->buffer, flag);
            }
        }
        // next sbus packet.
    }
    // never be here.
}

static int drv_sbus_init(void)
{
    rt_thread_t thread = rt_thread_create("bgserv_SBUS", sbus_background_worker,
                      "uart4",
                      1024, 10, 10);
    
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    else
        return -RT_ERROR;
    
    return RT_EOK;
}
INIT_ENV_EXPORT(drv_sbus_init);

// public functions
void sbus_set_update_callback(sbus_indicator indicator)
{
    sbus->callback = indicator;
}

sbus_indicator sbus_get_update_callback(void)
{
    return sbus->callback;
}

float sbus_normalize_value(rt_uint16_t value)
{
    if (abs(value - SBUS_MID_VALUE) < SBUS_TOLERANCE) return 0.f;
    
    return (float)(value - SBUS_MID_VALUE) / SBUS_MAX_RANGE;
}
