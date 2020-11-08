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

/* defined the PoWer SiWtch pin: PC4 */
#define PWSW_PIN        GET_PIN(C, 4)

#define LED_PIN         GET_PIN(A, 15)

#define RXBUFF_SIZE         (128 + 32)
#define MAIL_SIZE           (128 + 32)
#define THREAD_STACK_SIZE   (1024)
#define THREAD_STACK_SIZE2  (1024)
#define MIN_RX_INTERVAL     (5)         // 5ms timeout

#define PKT_HEADER          (0x3E)

#define HEADER_OFFSET       (00)
#define COMMAND_OFFSET      (01)
#define PAYLOAD_SIZE_OFFSET (02)
#define HEADER_CKSUM_OFFSET (03)
#define PAYLOAD_OFFSET      (04)

/* 
** UART1: mixed serial data port.
** UART2: SBGC serial data port.(version#1 & version#2)
** UART3: other serial data port.
*/

/* NEW CONFIG
** UART1: SBGC serial data port.(version#1 & version#2)
** UART2: mixed serial data port.
** UART3: other serial data port.
*/

static rt_mp_t      mpUart1Tx = RT_NULL;
static rt_mp_t      mpUart2Tx = RT_NULL;
static rt_mp_t      mpUart3Tx = RT_NULL;

static rt_mailbox_t mbUart1Tx = RT_NULL;
static rt_mailbox_t mbUart2Tx = RT_NULL;
static rt_mailbox_t mbUart3Tx = RT_NULL;

static rt_sem_t     sUart1 = RT_NULL;
static rt_sem_t     sUart2 = RT_NULL;
static rt_sem_t     sUart3 = RT_NULL;

static rt_device_t  pUart1 = RT_NULL;
static rt_device_t  pUart2 = RT_NULL;
static rt_device_t  pUart3 = RT_NULL;

struct PKT_MAIL
{
    rt_uint8_t size;
    rt_uint8_t payload[MAIL_SIZE - sizeof(rt_uint8_t)];
};

static rt_err_t rx1_hook(rt_device_t dev, rt_size_t sz)
{
	rt_sem_release(sUart1);
    
	return RT_EOK;
}

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

void crc16_update(uint16_t length, uint8_t *data, uint8_t crc[2]) {
    uint16_t counter;
    uint16_t polynom = 0x8005;
    uint16_t crc_register = (uint16_t)crc[0] | ((uint16_t)crc[1] << 8);
    uint8_t shift_register;
    uint8_t data_bit, crc_bit;
    
    for (counter = 0; counter < length; counter++) 
    {
        for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1) {
            data_bit = (data[counter] & shift_register) ? 1 : 0;
            crc_bit = crc_register >> 15;
            crc_register <<= 1;
            if (data_bit != crc_bit) crc_register ^= polynom;
        }
    }
    crc[0] = crc_register;
    crc[1] = (crc_register >> 8);
}

void crc16_calculate(uint16_t length, uint8_t *data, uint8_t crc[2]) {
    crc[0] = 0; crc[1] = 0;
    crc16_update(length, data, crc);
}

static void uart1_send_entry(void* parameter){

    struct PKT_MAIL* pMail = RT_NULL;
    
    rt_device_t devUart = rt_device_find("uart1");
    RT_ASSERT(devUart);
    
    while (1){
        rt_mb_recv(mbUart1Tx, (rt_ubase_t*)&pMail, RT_WAITING_FOREVER);
        if (pMail == RT_NULL) continue;

        rt_device_write(devUart, 0, &pMail->payload[0], pMail->size);
        rt_mp_free(pMail);
    }
}

static void uart2_send_entry(void* parameter){

    struct PKT_MAIL* pMail = RT_NULL;
    
    rt_device_t devUart = rt_device_find("uart2");
    RT_ASSERT(devUart);
    
    while (1){
        rt_mb_recv(mbUart2Tx, (rt_ubase_t *)&pMail, RT_WAITING_FOREVER);
        if (pMail == RT_NULL) continue;

        rt_device_write(devUart, 0, &pMail->payload[0], pMail->size);
        rt_mp_free(pMail);
    }
}

static void uart3_send_entry(void* parameter){

    struct PKT_MAIL* pMail = RT_NULL;
    
    rt_device_t devUart = rt_device_find("uart3");
    RT_ASSERT(devUart);
    
    while (1){
        rt_mb_recv(mbUart3Tx, (rt_ubase_t *)&pMail, RT_WAITING_FOREVER);
        if (pMail == RT_NULL) continue;
        
        rt_device_write(devUart, 0, &pMail->payload[0], pMail->size);
        rt_mp_free(pMail);
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t uart2_rx_stack[THREAD_STACK_SIZE2];
static struct rt_thread uart2_rx_thread;
static void uart2_rx_entry(void* parameter)
{
    rt_uint8_t *pBuf = RT_NULL;
    rt_uint8_t *pBGC = RT_NULL;
    rt_size_t  szBuf;
    rt_size_t  szBGC;
    rt_err_t result = RT_EOK; 
    rt_bool_t isSBGC = RT_FALSE;
    struct PKT_MAIL *pMail;
    
    pUart2 = rt_device_find("uart2");
    RT_ASSERT(pUart2 != RT_NULL);
    
    rt_device_open(pUart2, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX); // | RT_DEVICE_FLAG_DMA_TX 
    
    // set uart1's baudrate to 115200
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    rt_device_control(pUart2, RT_DEVICE_CTRL_CONFIG, &config);

    rt_device_set_rx_indicate(pUart2, rx2_hook);
    
    pBuf = rt_malloc(RXBUFF_SIZE);
    RT_ASSERT(pBuf != RT_NULL);
    rt_memset(pBuf, 0x00, RXBUFF_SIZE);
    
    pBGC = rt_malloc(RXBUFF_SIZE);
    RT_ASSERT(pBGC != RT_NULL);
    rt_memset(pBGC, 0x00, RXBUFF_SIZE);
   
    szBuf = 0;
    szBGC = 0;
    
    while (RT_TRUE){
        
        result = rt_sem_take(sUart2, MIN_RX_INTERVAL);
        
        if (result != -RT_ETIMEOUT) {
            
            szBGC += rt_device_read(pUart2, 0, pBGC + szBGC, 1);
        
            if (isSBGC == RT_FALSE) {
                RT_ASSERT(szBuf < RXBUFF_SIZE);
                pBuf[szBuf++] = pBGC[szBGC - 1];
            }
            
            switch (szBGC - 1){
            case HEADER_OFFSET:
                if ( pBGC[HEADER_OFFSET] != PKT_HEADER ){
                    szBGC = 0;
                }
                break;
            case COMMAND_OFFSET:
            case PAYLOAD_SIZE_OFFSET:
                continue;
                //break
            case HEADER_CKSUM_OFFSET:

                if ( pBGC[HEADER_CKSUM_OFFSET] == pBGC[COMMAND_OFFSET] + pBGC[PAYLOAD_SIZE_OFFSET] ) {
                    isSBGC = RT_TRUE;
                    // clean the cache BGC data in buffer.
                    rt_memset(&pBuf[szBuf-4], 0x00, 4);
                    szBuf = szBuf - 4;
                }
                else
                    szBGC = 0;

               break;
            default:
                break;
            }
        }
        else {
            // semaphore timeout.
            // if buffer isn't empty, then send the data and clear the buffer.
            if (szBuf == 0)
                continue;
            
            if (szBuf == 5) {
                if ( ( pBuf[0] == 0xE1) && (pBuf[1] == 0x1E) && (pBuf[2] == 0x12) && (pBuf[3] == 0xF1) && (pBuf[4] == 0x1F) ) {

                    pBuf[0] = 0x3E;
                    pBuf[1] = 0x19;
                    pBuf[2] = 0x00;
                    pBuf[3] = 0x19;
                    pBuf[4] = 0x00;
                
                    pMail = (struct PKT_MAIL *)rt_mp_alloc(mpUart1Tx, RT_WAITING_NO);
                    if (pMail != RT_NULL) {
                        pMail->size = szBuf;
                        rt_memcpy(&pMail->payload[0], pBuf, szBuf);
                        result = rt_mb_send(mbUart1Tx, (rt_ubase_t)pMail);

                        if (RT_EOK != result) {
                            rt_kprintf("%d, tRX2: mb send TX1 wrong\n", rt_tick_get());
                            rt_mp_free(pMail);
                            rt_thread_delay(1);
                        }
                    }
                    else {
                        rt_kprintf("%d, timeout to tx3, loop\n", rt_tick_get());
                        while(1);
                    }
                    
                    rt_memset(pBuf, 0x00, szBuf);
                    szBuf = 0;
                    
                    continue;
                }
            }

            pMail = (struct PKT_MAIL *)rt_mp_alloc(mpUart3Tx, RT_WAITING_NO);
            if (pMail != RT_NULL) {
                pMail->size = szBuf;
                rt_memcpy(&pMail->payload[0], pBuf, szBuf);
                //rt_kprintf("%d: TIMOUT, send %d\r\n", rt_tick_get(), szBuf);
                result = rt_mb_send(mbUart3Tx, (rt_ubase_t)pMail);
                if (RT_EOK != result) {
                    rt_kprintf("%d, tRX2: mb send TX3 wrong\n", rt_tick_get());
                    rt_mp_free(pMail);
                    while(1);
                }                
            }
            else {
                rt_kprintf("%d, timeout to tx3, loop\n", rt_tick_get());
                while(1);
            }
            
            rt_memset(pBuf, 0x00, szBuf);
            szBuf = 0;
        }
        
        if (isSBGC == RT_TRUE) {
            // if recvive unfinish, then contiune.
            if (szBGC != pBGC[PAYLOAD_SIZE_OFFSET] + 5 ) {
                if (szBGC > RXBUFF_SIZE)
                    rt_kprintf("PAYLOAD SIZE OVERSIZE, %d\n", szBGC);
                continue;
            }

            // clear the flag.
            isSBGC = RT_FALSE;
            // send BGC packet to uart1.
            pMail = (struct PKT_MAIL *)rt_mp_alloc(mpUart1Tx, RT_WAITING_NO);
            if (pMail != RT_NULL) {
                pMail->size = szBGC;
                rt_memcpy(&pMail->payload[0], pBGC, szBGC);

                result = rt_mb_send(mbUart1Tx, (rt_ubase_t)pMail);
                
                if (RT_EOK != result) {
                    rt_kprintf("%d, tRX2: mb send TX1 wrong\n", rt_tick_get());
                    rt_mp_free(pMail);
                    while(1);
                }
            }
            else {
                rt_kprintf("%d, E11E12F11F, 1 loop\n", rt_tick_get());
                while(1);
            }

            rt_memset(pBGC, 0x00, szBGC);
            szBGC = 0;
                
            // send other data to uart3.
            if (szBuf == 0) continue;
                
            if (szBuf == 5) {
                if ( ( pBuf[0] == 0xE1) && (pBuf[1] == 0x1E) && (pBuf[2] == 0x12) && (pBuf[3] == 0xF1) && (pBuf[4] == 0x1F) ) {
                    pBuf[0] = 0x3E;
                    pBuf[1] = 0x19;
                    pBuf[2] = 0x00;
                    pBuf[3] = 0x19;
                    pBuf[4] = 0x00;

                    pMail = (struct PKT_MAIL *)rt_mp_alloc(mpUart1Tx, RT_WAITING_NO);
                    if (pMail != RT_NULL) {
                        pMail->size = szBuf;
                        rt_memcpy(&pMail->payload[0], pBuf, szBuf);
                        result = rt_mb_send(mbUart1Tx, (rt_ubase_t)pMail);

                        if (mpUart1Tx->block_free_count < 5)
                            rt_kprintf("%d, 2 mempool free: %d\n", rt_tick_get(), mpUart1Tx->block_free_count);
                        
                        if (RT_EOK != result) {
                            rt_kprintf("%d, tRX2: mb send TX1 wrong\n", rt_tick_get());
                            rt_mp_free(pMail);
                            while(1);
                        }
                    }
                    else {
                        rt_kprintf("%d, E11E12F11F, 2 loop\n", rt_tick_get());
                        while(1);
                    }
                            
                    rt_memset(pBuf, 0x00, szBuf);
                    szBuf = 0;
                            
                    continue;
                }
            }

            pMail = (struct PKT_MAIL *)rt_mp_alloc(mpUart3Tx, RT_WAITING_NO);

            if (pMail != RT_NULL) {
                pMail->size = szBuf;
                rt_memcpy(&pMail->payload[0], pBuf, szBuf);

                result = rt_mb_send(mbUart3Tx, (rt_ubase_t)pMail);
                if (RT_EOK != result) {
                    rt_kprintf("%d, tRX2: mb send TX3 wrong\n", rt_tick_get());
                    rt_mp_free(pMail);
                    while(1);
                }
            }
            else {
                rt_kprintf("%d: other to tx3, loop\n", rt_tick_get());
                while(1);                
            }

            rt_memset(pBuf, 0x00, szBuf);
            szBuf = 0;
        }
        else {
            // if buffer is full, send data to uart3 and clear it.
            if ( szBuf == MAIL_SIZE - sizeof(rt_uint8_t)) {

                pMail = (struct PKT_MAIL *)rt_mp_alloc(mpUart3Tx, RT_WAITING_NO);
                if (pMail != RT_NULL) {
                    pMail->size = szBuf;
                    rt_memcpy(&pMail->payload[0], pBuf, szBuf);
                    rt_kprintf("%d: OVERSZ, send %d\r\n", rt_tick_get(), szBuf);
                    result = rt_mb_send(mbUart3Tx, (rt_ubase_t)pMail);
                    if (RT_EOK != result) {
                        rt_kprintf("%d, tRX1: mb send TX3 wrong\n", rt_tick_get());
                        rt_mp_free(pMail);
                        while(1);
                    }
                }
                else {
                    rt_kprintf("%d: oversize to tx3, loop\n", rt_tick_get());
                    while(1);
                }

                rt_memset(pBuf, 0x00, szBuf);
                
                if (szBGC < 4)
                    szBuf = szBGC;
                else
                    szBuf = 0;
            }
        }

        // loop end.
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t uart1_rx_stack[ THREAD_STACK_SIZE ];
static struct rt_thread uart1_rx_thread;
static void uart1_rx_entry(void* parameter)
{
    rt_uint8_t *pbuf = RT_NULL;
    rt_size_t  szbuf;   
    rt_err_t result = RT_EOK;
    struct PKT_MAIL *pMail;
    
    pUart1 = rt_device_find("uart1");
    RT_ASSERT(pUart1 != RT_NULL);
    
    rt_device_open(pUart1, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX); // | RT_DEVICE_FLAG_DMA_TX

    // set uart1's baudrate to 115200
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    rt_device_control(pUart1, RT_DEVICE_CTRL_CONFIG, &config);

    rt_device_set_rx_indicate(pUart1, rx1_hook);
    
    pbuf = rt_malloc(RXBUFF_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    rt_memset(pbuf, 0x00, RXBUFF_SIZE);

    szbuf = 0;

    while (RT_TRUE){
        
        result = rt_sem_take(sUart1, MIN_RX_INTERVAL);
        
        if(result != -RT_ETIMEOUT) {
            /* read 1 byte form uart fifo ringbuffer */
            szbuf += rt_device_read(pUart1, 0, pbuf + szbuf, 1);
            
            if (szbuf < RXBUFF_SIZE - sizeof(rt_uint8_t))
                continue;
        }
        else {
            if (szbuf == 0)
                continue;       // ignore idle frame.
        }
        
        pMail = (struct PKT_MAIL *)rt_mp_alloc(mpUart2Tx, RT_WAITING_NO);
        if (pMail == RT_NULL) while(1);
        
        rt_memset(pMail, 0x00, MAIL_SIZE);
            
        pMail->size = szbuf;
        rt_memcpy(&pMail->payload[0], pbuf, szbuf);
        //rt_kprintf("%d: Form BGC, recv %d\r\n", rt_tick_get(), szbuf);
        result = rt_mb_send(mbUart2Tx, (rt_ubase_t)pMail);
        
        if (RT_EOK != result)
        {
            rt_kprintf("tRX1: mb send wrong\n");
            rt_mp_free(pMail);
            rt_thread_delay(1);
        }
        
        szbuf = 0;
        // loop end.
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t uart3_rx_stack[ THREAD_STACK_SIZE ];
static struct rt_thread uart3_rx_thread;
static void uart3_rx_entry(void* parameter)
{
    rt_uint8_t *pbuf = RT_NULL;
    rt_size_t  szbuf;   
    rt_err_t result = RT_EOK;
    struct PKT_MAIL *pMail;
    
    pUart3 = rt_device_find("uart3");
    RT_ASSERT(pUart3 != RT_NULL);
    
    rt_device_open(pUart3, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX ); //| RT_DEVICE_FLAG_DMA_TX
    
    // set uart3's baudrate to 115200
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    rt_device_control(pUart3, RT_DEVICE_CTRL_CONFIG, &config);
    
    rt_device_set_rx_indicate(pUart3, rx3_hook);
    
    pbuf = rt_malloc(RXBUFF_SIZE);
    RT_ASSERT(pbuf != RT_NULL);
    rt_memset(pbuf, 0x00, RXBUFF_SIZE);
   
    rt_thread_delay(100);
    
    szbuf = 0;

    while (RT_TRUE){
        
        result = rt_sem_take(sUart3, MIN_RX_INTERVAL);
        
        if(result != -RT_ETIMEOUT) {
            /* read 1 byte form uart fifo ringbuffer */
            szbuf += rt_device_read(pUart3, 0, pbuf + szbuf, 1);
            
            if (szbuf < RXBUFF_SIZE - sizeof(rt_uint8_t))
                continue;
        }
        else {
            if (szbuf == 0)
                continue;       // ignore idle frame.
        }
        
        pMail = (struct PKT_MAIL *)rt_mp_alloc(mpUart2Tx, RT_WAITING_NO);
        if (pMail == RT_NULL) while(1);
        
        rt_memset(pMail, 0x00, MAIL_SIZE);
            
        pMail->size = szbuf;
        rt_memcpy(&pMail->payload[0], pbuf, szbuf);

        result = rt_mb_send(mbUart2Tx, (rt_ubase_t)pMail);
        
        if (RT_EOK != result)
        {
            rt_kprintf("tRX3: mb send wrong\n");
            rt_mp_free(pMail);
            rt_thread_delay(1);
        }
        
        szbuf = 0;
        
        // loop end.
    }
}

static  rt_device_t pWDT = RT_NULL;

static void idle_hook(void)
{
    rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
//    rt_kprintf("feed the dog!\n ");
}

int main(void)
{   
    rt_err_t result = RT_EOK;
    rt_thread_t     tid = RT_NULL;
    rt_size_t       timeout = 1;
    
    pWDT = rt_device_find("wdt");
    RT_ASSERT(pWDT != RT_NULL);
    
    result = rt_device_init(pWDT);
    result = rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
    result = rt_device_control(pWDT, RT_DEVICE_CTRL_WDT_START, RT_NULL);
    rt_thread_idle_sethook(idle_hook);
    rt_kprintf("Set WatchDog Complate!\n");
    
    /* initialization */
    
    sUart1 = rt_sem_create("sRX1", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(sUart1 != RT_NULL);
    sUart2 = rt_sem_create("sRX2", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(sUart2 != RT_NULL);
    sUart3 = rt_sem_create("sRX3", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(sUart3 != RT_NULL);
    
    mpUart1Tx = rt_mp_create("mpTX1", 6, MAIL_SIZE);
    RT_ASSERT(mpUart1Tx != RT_NULL);
    mpUart2Tx = rt_mp_create("mpTX2", 3, MAIL_SIZE);
    RT_ASSERT(mpUart2Tx != RT_NULL);    
    mpUart3Tx = rt_mp_create("mpTX3", 3, MAIL_SIZE);
    RT_ASSERT(mpUart3Tx != RT_NULL);
    
    mbUart1Tx = rt_mb_create("mbTX1", 5, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mbUart1Tx != RT_NULL);
    mbUart2Tx = rt_mb_create("mbTX2", 3, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mbUart2Tx != RT_NULL);
    mbUart3Tx = rt_mb_create("mbTX3", 3, RT_IPC_FLAG_FIFO);
    RT_ASSERT(mbUart3Tx != RT_NULL);
    
    tid = rt_thread_create("tTX1", uart1_send_entry, NULL, THREAD_STACK_SIZE, 7, 50);
    if (tid != RT_NULL) rt_thread_startup(tid);    
    
    tid = rt_thread_create("tTX2", uart2_send_entry, NULL, THREAD_STACK_SIZE2, 5, 50);
    if (tid != RT_NULL) rt_thread_startup(tid);    
    
    tid = rt_thread_create("tTX3", uart3_send_entry, NULL, THREAD_STACK_SIZE, 6, 50);
    if (tid != RT_NULL) rt_thread_startup(tid);

	/* init uart1 rx thread */
	result = rt_thread_init(&uart1_rx_thread, 
                            "tRX1",
							uart1_rx_entry,
							RT_NULL,
                            (rt_uint8_t*)&uart1_rx_stack[0],
                            sizeof(uart1_rx_stack), 9, 50);
	if (result == RT_EOK)
	{
		rt_kprintf("thread \"uart1_rx\"start.\n");
		rt_thread_startup(&uart1_rx_thread);
	}
    
	/* init uart2 rx thread */
	result = rt_thread_init(&uart2_rx_thread, 
                            "tRX2",
							uart2_rx_entry,
							RT_NULL,
                            (rt_uint8_t*)&uart2_rx_stack[0],
                            sizeof(uart2_rx_stack), 9, 50);
	if (result == RT_EOK)
	{
		rt_kprintf("thread \"uart2_rx\"start.\n");
		rt_thread_startup(&uart2_rx_thread);
	}
    
	/* init uart3 rx thread */
	result = rt_thread_init(&uart3_rx_thread, 
                            "tRX3",
							uart3_rx_entry,
							RT_NULL,
                            (rt_uint8_t*)&uart3_rx_stack[0],
                            sizeof(uart3_rx_stack), 9, 50);
	if (result == RT_EOK)
	{
		rt_kprintf("thread \"uart3_rx\"start.\n");
		rt_thread_startup(&uart3_rx_thread);
	}
    
    /* set LED0 pin mode to output */
    rt_pin_write(PWSW_PIN, PIN_HIGH);
    rt_pin_mode (PWSW_PIN, PIN_MODE_OUTPUT);
    
    rt_pin_mode (LED_PIN, PIN_MODE_OUTPUT);
    
    
    while (1)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(RT_TICK_PER_SECOND / 5);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(RT_TICK_PER_SECOND * 4 / 5);
    }

    // Never reach here.
}
