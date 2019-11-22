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
#define UART3_USE_SEM

/* defined the LED0 pin: PB0 */
#define LED0_PIN    GET_PIN(B, 0)
#define RXBUFF_SIZE (1024)
#define MVBUFF_SIZE (512)


static rt_device_t  pUart2 = RT_NULL;
static rt_sem_t     sUart2 = RT_NULL;
static rt_mailbox_t mbUart3Tx = RT_NULL;
static rt_mp_t      mpUart3Tx = RT_NULL;
static rt_mailbox_t mbUart4Tx = RT_NULL;
static rt_mp_t      mpUart4Tx = RT_NULL;

static rt_device_t  pUart3 = RT_NULL;
static rt_sem_t     sUart3 = RT_NULL;
static rt_mailbox_t mbUart2Tx = RT_NULL;
static rt_mp_t      mpUart2Tx = RT_NULL;

static rt_device_t  pUart4 = RT_NULL;
static rt_event_t   eUart4 = RT_NULL;
static rt_size_t    szRX4 = 0;


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

static rt_err_t rx4_hook(rt_device_t dev, rt_size_t sz)
{
	szRX4 = sz;
	rt_event_send(eUart4, (1 << 0));
	
	return RT_EOK;
}

/** 
 * Calculating CRC-16 in 'C' 
 * @para addr, start of data 
 * @para num, length of data 
 * @para crc, incoming CRC 
 */
#define MAVLINK_MESSAGE_LENGTHS  {9, 31, 12, 0, 14, 28, 3, 32, 36, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 28, 22, 22, 21, 6, 6, 37, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 37, 4, 7, 0, 27, 25, 0, 0, 0, 0, 0, 72, 26, 181, 225, 42, 6, 4, 0, 11, 18, 0, 0, 37, 20, 35, 33, 3, 0, 0, 0, 22, 39, 37, 53, 51, 53, 51, 0, 28, 56, 42, 33, 81, 0, 0, 0, 0, 0, 0, 26, 32, 32, 20, 32, 62, 44, 64, 84, 9, 254, 16, 12, 36, 44, 64, 22, 6, 14, 12, 97, 2, 2, 113, 35, 6, 79, 35, 35, 22, 13, 255, 14, 18, 43, 8, 22, 14, 36, 43, 41, 32, 243, 14, 93, 0, 100, 36, 60, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 40, 63, 182, 40, 42, 0, 0, 0, 0, 0, 32, 52, 53, 6, 2, 38, 19, 254, 36, 30, 18, 18, 51, 9, 0}
#define MAVLINK_MESSAGE_CRCS    {50, 124, 137, 0, 237, 217, 104, 119, 117, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 132, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 189, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 163, 105, 151, 35, 150, 179, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0}

#define X25_INIT_CRC 0xffff
    
static rt_uint8_t mav_msg_length[] = MAVLINK_MESSAGE_LENGTHS;
static rt_uint8_t mav_msg_crc[] = MAVLINK_MESSAGE_CRCS;

static inline void crc_accumulate(rt_uint8_t data, rt_uint16_t *crcAccum)
{
        /*Accumulate one byte of data into the CRC*/
        rt_uint8_t tmp;

        tmp = data ^ (rt_uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}

static inline void crc_init(rt_uint16_t* crcAccum)
{
    *crcAccum = X25_INIT_CRC;
}

static inline uint16_t crc_calculate(const rt_uint8_t* pBuffer, rt_uint16_t length)
{
    rt_uint16_t crcTmp;
    
    crc_init(&crcTmp);
    while (length--) {
        crc_accumulate(*pBuffer++, &crcTmp);
    }
    return crcTmp;
}

static inline void crc_accumulate_buffer(rt_uint16_t *crcAccum, const rt_uint8_t *pBuffer, rt_uint16_t length)
{
    const rt_uint8_t *p = (const rt_uint8_t *)pBuffer;
    
    while (length--) {
        crc_accumulate(*p++, crcAccum);
    }
}

__packed struct MAV_MSG87{
    rt_uint32_t     time_boot_ms;
    rt_int32_t      lat_int;
    rt_int32_t      lon_int;
    float           alt;
    float           vx;
    float           vy;
    float           vz;
    float           afx;
    float           afy;
    float           afz;
    float           yaw;
    float           yaw_rate;
    rt_uint16_t     type_mask;
    rt_uint8_t      coordinate_frame;
};

static void mavlink_build_MSG87(rt_uint8_t *buff, rt_uint8_t crc, int lat, int lon, float alt)
{
    rt_uint16_t checksum;
    struct MAV_MSG87    package;
    
    buff[0] = 0xFE;      // stx
    buff[1] = 51;        //Length of payload
    buff[2] = 0x00;      //Sequence of packet
    buff[3] = 0x01;      //sysid    ID of message sender system/aircraft
    buff[4] = 0x01;      //compid  ID of the message sender component
    buff[5] = 87;        //msgid   ID of message in payload
    // payload #87
    rt_memset(&package, 0x00, sizeof(package));
    package.lat_int = lat;
    package.lon_int = lon;
    package.alt = alt;
    package.coordinate_frame = 0x05;
    package.type_mask = 0xFFF8;
    rt_memcpy(&buff[6], &package, sizeof(package));
    // checksum CRC16
    checksum = crc_calculate((const rt_uint8_t*)&buff[1], 5);
    crc_accumulate_buffer(&checksum,  (const rt_uint8_t*)&buff[6], 51);
    crc_accumulate(crc, &checksum);
    buff[57] = (rt_uint8_t)(checksum & 0xFF);
    buff[58] = (rt_uint8_t)(checksum >> 8);
    
    return;
}

static void print_rawdata(rt_uint8_t *pbuf, rt_size_t sz, char *str)
{
    if (str == RT_NULL)
        str = "null";
    
    rt_kprintf("print_raw(%s): %d\n", str, sz);

    for(int i = 0; i < sz; ) {
        
        if( (i % 16) == 0)
            rt_kprintf("0x%04X: ", i);
        
        rt_kprintf("%02x ", pbuf[i]);
        
        if( (++i % 16) == 0)
            rt_kprintf("\n");
    }
    rt_kprintf("(END)\n");
}

static void uart3_send_entry(void* parameter){
    rt_ubase_t addr;
    rt_uint8_t* paddr;
    rt_device_t devUart3 = rt_device_find("uart3");
    RT_ASSERT(devUart3);
    
    while (1){
        rt_mb_recv(mbUart3Tx, &addr, RT_WAITING_FOREVER);
        if (addr == 0) continue;
        paddr = (rt_uint8_t*)addr;
        rt_device_write(devUart3, 0, paddr, paddr[1] + 8);        
        rt_mp_free(paddr);
    }
}

static void uart4_send_entry(void* parameter){
    rt_ubase_t addr;
    rt_uint8_t* paddr;
    rt_device_t devUart4 = rt_device_find("uart4");
    RT_ASSERT(devUart4);
    
    while (1){
        rt_mb_recv(mbUart4Tx, &addr, RT_WAITING_FOREVER);
        if (addr == 0) continue;
        paddr = (rt_uint8_t*)addr;
        rt_device_write(devUart4, 0, paddr, 5);        
        rt_mp_free(paddr);
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t uart2_rx_stack[ 1024 ];
static struct rt_thread uart2_rx_thread;
static void uart2_rx_entry(void* parameter)
{
    rt_uint8_t *pbuf = RT_NULL;
    rt_uint8_t *ppkg = RT_NULL;
    rt_size_t bufsz, mavsz, offset;
    rt_err_t retval = RT_EOK;
    rt_thread_t tidU3Tx = RT_NULL, tidU4Tx = RT_NULL;
    
    pUart2 = rt_device_find("uart2");
    RT_ASSERT(pUart2 != RT_NULL);
    
    rt_device_open(pUart2, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    
    sUart2 = rt_sem_create("sRX2", 0, RT_IPC_FLAG_FIFO);

    rt_device_set_rx_indicate(pUart2, rx2_hook);
    
    pbuf = rt_malloc(RXBUFF_SIZE);
    rt_memset(pbuf, 0x00, RXBUFF_SIZE);
    
    ppkg = rt_malloc(MVBUFF_SIZE);
    rt_memset(ppkg, 0x00, MVBUFF_SIZE);
    
    mpUart3Tx = rt_mp_create("mpTX3", 4, 512);
    mbUart3Tx = rt_mb_create("mbTX3", 4, RT_IPC_FLAG_FIFO);
    
    tidU3Tx = rt_thread_create("tTX3", uart3_send_entry, NULL, 1024, 11, RT_TICK_PER_SECOND/20);
    if (tidU3Tx && mpUart3Tx && mbUart3Tx) rt_thread_startup(tidU3Tx);

    mpUart4Tx = rt_mp_create("mpTX4", 4, 512);
    mbUart4Tx = rt_mb_create("mbTX4", 4, RT_IPC_FLAG_FIFO);

    tidU4Tx = rt_thread_create("tTX4", uart4_send_entry, NULL, 1024, 11, RT_TICK_PER_SECOND/20);
    if (tidU4Tx && mpUart4Tx && mbUart4Tx) rt_thread_startup(tidU4Tx);
   
    rt_thread_delay(100);
    
    bufsz = 0; 
   
    while (RT_TRUE){
        size_t rxInBuf = 0;
        mavsz = 0;
        retval = rt_sem_take(sUart2, RT_WAITING_FOREVER); 
        if(retval == -RT_ETIMEOUT)  continue; 
        switch (bufsz){
        case 0:
        case 1:
            bufsz += rt_device_read(pUart2, 0, pbuf + bufsz, 1);
            if (0xFE != pbuf[0]){
                bufsz = 0;
                rt_kprintf("%02X ", pbuf[0]);
            }
            break;
        default: 
            mavsz = pbuf[1] + 6 + 2;
            bufsz += rt_device_read(pUart2, 0, pbuf + bufsz, mavsz-bufsz);
            break;
        }
        
        if ((bufsz != mavsz) || (mavsz == 0)) continue;
        
        // get a whole frame data.
        offset = 0;
        while (offset < bufsz){
            if (pbuf[offset] != 0xFE) { rt_kprintf ("RX2 miss2 %02X\n", pbuf[offset]); 
                continue;
            }
           
            mavsz = pbuf[1] + 6 + 2;
            rxInBuf = offset;
            offset += mavsz;
            if (offset <= bufsz){
               
                if (pbuf[5] == 0x4C) {
                    rt_uint8_t* pkg = (rt_uint8_t*) rt_mp_alloc(mpUart4Tx, 0);
                    if (!pkg) {rt_kprintf("tRX2: no mem for TX4\n"); continue;}
                    
                    rt_memcpy(ppkg, pbuf+rxInBuf, mavsz);
                    
                    float fVal = 0.0f;
                    rt_uint8_t   SGBC32[5] = {0xE1, 0x1E, 0x00, 0xF1, 0x1F};
                    fVal = *(float*)&ppkg[6];
                    rt_kprintf("tRX2: CTRL(0x%02X)\n", bufsz, (rt_uint8_t)fVal);
                    SGBC32[2] = (rt_uint8_t)fVal;
                    
                    rt_memcpy(pkg, SGBC32, sizeof(SGBC32));
                    
                    if (RT_EOK != rt_mb_send(mbUart4Tx, (rt_ubase_t)pkg)){
                        rt_kprintf("tRX2: mb wrong for TX4\n");
                        rt_mp_free(pkg);
                    }
                    //rt_kprintf("mb_send %d\n", mavsz);
                }
                else {
                    rt_uint8_t* pkg = (rt_uint8_t*) rt_mp_alloc(mpUart3Tx, 0);
                    if (!pkg) {rt_kprintf("tRX2: no mem for TX3\n"); continue;}

                    rt_memcpy(ppkg, pbuf+rxInBuf, mavsz);
                    //print_rawdata(ppkg, mavsz, "rx3");
                    //rt_kprintf("rx3(mavlink): id(%d) len(%d)\n", ppkg[5], mavsz);

                    rt_memcpy(pkg, ppkg, mavsz);
                    if (RT_EOK != rt_mb_send(mbUart3Tx, (rt_ubase_t)pkg)){
                        rt_kprintf("tRX2: mb wrong for TX3\n");
                        rt_mp_free(pkg);
                    }
                    //rt_kprintf("mb_send %d\n", mavsz);
                }
           }
        }
        
        if ((offset > bufsz) && (rxInBuf < bufsz)){
            rt_kprintf("tRX2:broken data\n");
            rt_memcpy(pbuf, pbuf + rxInBuf, bufsz - rxInBuf);
            bufsz = bufsz - rxInBuf;
        }
        else
            bufsz = 0;
    }
}

static void uart2_send_entry(void* parameter){
    rt_ubase_t addr;
    rt_uint8_t* paddr;
    rt_device_t devUart2 = rt_device_find("uart2");
    RT_ASSERT(devUart2);
    
    while (1){
        rt_mb_recv(mbUart2Tx, &addr, RT_WAITING_FOREVER);
        if (addr == 0) continue;
        paddr = (rt_uint8_t*)addr;
        rt_device_write(devUart2, 0, paddr, paddr[1] + 8);        
        rt_mp_free(paddr);
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t uart3_rx_stack[ 1024 ];
static struct rt_thread uart3_rx_thread;
static void uart3_rx_entry(void* parameter)
{
    rt_uint8_t *pbuf = RT_NULL;
    rt_uint8_t *ppkg = RT_NULL;
    rt_size_t bufsz, mavsz, offset;
    rt_err_t retval = RT_EOK;
    rt_thread_t tidU2Tx = RT_NULL;
    
    pUart3 = rt_device_find("uart3");
    RT_ASSERT(pUart3 != RT_NULL);
    
    rt_device_open(pUart3, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    
    sUart3 = rt_sem_create("u3rx", 0, RT_IPC_FLAG_FIFO);

    rt_device_set_rx_indicate(pUart3, rx3_hook);
    
    pbuf = rt_malloc(RXBUFF_SIZE);
    rt_memset(pbuf, 0x00, RXBUFF_SIZE);
    
    ppkg = rt_malloc(MVBUFF_SIZE);
    rt_memset(ppkg, 0x00, MVBUFF_SIZE);
    
    mpUart2Tx = rt_mp_create("mpTX2", 4, 512);
    mbUart2Tx = rt_mb_create("mbTX2", 4, RT_IPC_FLAG_FIFO);
    
    tidU2Tx = rt_thread_create("tTX2", uart2_send_entry, NULL, 1024, 11, RT_TICK_PER_SECOND/20);
    if (tidU2Tx && mpUart2Tx && mbUart2Tx) rt_thread_startup(tidU2Tx);
   
    rt_thread_delay(100);
    
    bufsz = 0; 
   
    while (RT_TRUE){
        size_t rxInBuf = 0;
        mavsz = 0;
        retval = rt_sem_take(sUart3, RT_WAITING_FOREVER); 
        if(retval == -RT_ETIMEOUT)  continue; 
        switch (bufsz){
        case 0:
        case 1:
            bufsz += rt_device_read(pUart3, 0, pbuf + bufsz, 1);
            if (0xFE != pbuf[0]){
                bufsz = 0;
                rt_kprintf("%02X ", pbuf[0]);
            }
            break;
        default: 
            mavsz = pbuf[1] + 6 + 2;
            bufsz += rt_device_read(pUart3, 0, pbuf + bufsz, mavsz-bufsz);
            break;
        }
        
        if ((bufsz != mavsz) || (mavsz == 0)) continue;
        
        // get a whole frame data.
        offset = 0;
        while (offset < bufsz){
           if (pbuf[offset] != 0xFE) { rt_kprintf ("miss2 %02X\n", pbuf[offset]); 
               continue;
           }
           
           mavsz = pbuf[1] + 6 + 2;
           rxInBuf = offset;
           offset += mavsz;
           if (offset <= bufsz){
               rt_uint8_t* pkg = (rt_uint8_t*) rt_mp_alloc(mpUart2Tx, 0);
               if (!pkg) {rt_kprintf("tRX3: no mem\n"); continue;}
               
                rt_memcpy(ppkg, pbuf+rxInBuf, mavsz);
               
                if (ppkg[5] == 87) {
                    // discard, MSG87 will send by tRX4.
                    //rt_kprintf("rx3(mavlink): position msg\n");
                    rt_mp_free(pkg);
                    continue;
                }
                
                rt_memcpy(pkg, ppkg, mavsz);
                if (RT_EOK != rt_mb_send(mbUart2Tx, (rt_ubase_t)pkg)){
                    rt_kprintf("tRX3: mb wrong\n");
                    rt_mp_free(pkg);                    
                }
            }
        }
        
        if ((offset > bufsz) && (rxInBuf < bufsz)){
            rt_kprintf("tRX3:broken data\n");
            rt_memcpy(pbuf, pbuf + rxInBuf, bufsz - rxInBuf);
            bufsz = bufsz - rxInBuf;
        }
        else
            bufsz = 0;
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t uart4_rx_stack[ 1024 ];
static struct rt_thread uart4_rx_thread;
static void uart4_rx_entry(void* parameter)
{
    rt_uint8_t *pbuf = RT_NULL;
    rt_size_t bufsz = 0;
    rt_uint32_t e;
    rt_err_t retval = RT_EOK;

    pUart4 = rt_device_find("uart4");
    RT_ASSERT(pUart4 != RT_NULL);
    rt_device_open(pUart4, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    eUart4 = rt_event_create("eRX4", RT_IPC_FLAG_FIFO);
    
    rt_device_set_rx_indicate(pUart4, rx4_hook);
    
    pbuf =  rt_malloc(RXBUFF_SIZE);
    
    rt_thread_delay(100);
    
    while(RT_TRUE) {
        
        rt_memset(pbuf, 0x00, RXBUFF_SIZE);
        bufsz = 0;
        
        do{
            retval = rt_event_recv(eUart4, (1 << 0), RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 10, &e);
            
            if(retval == -RT_ETIMEOUT) {
                break;
            }
            else {
                if (bufsz + szRX4 > RXBUFF_SIZE)
                    break;
                bufsz += rt_device_read(pUart4, 0, pbuf + bufsz, szRX4);
            }

        }while(1);
        
        // add some code to process the frame data.
        if (bufsz != 0) {
            
            print_rawdata(pbuf, 6, "RAW");
            
            int lat = *(rt_int16_t*)&pbuf[4];
            int lon = *(rt_int16_t*)&pbuf[2];
            float alt = (float)*(rt_int16_t*)&pbuf[0];
            
            rt_uint8_t* pkg = (rt_uint8_t*) rt_mp_alloc(mpUart2Tx, 0);
            if (!pkg) {rt_kprintf("tRX4: no mem\n"); continue;}
            
            rt_uint8_t msg_id = 87;
            rt_memset(pkg, 0x00, mav_msg_length[msg_id]);
            
            mavlink_build_MSG87(pkg, mav_msg_crc[msg_id], lat, lon, alt);
            print_rawdata(pkg, mav_msg_length[msg_id] + 8, "MSG87");
            
            if (RT_EOK != rt_mb_send(mbUart2Tx, (rt_ubase_t)pkg)){
                rt_kprintf("tRX4: mb wrong\n");
                rt_mp_free(pkg);                    
            }
        }
    }
}

int main(void)
{
    int count = 1;
    
    rt_err_t retval = RT_EOK;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    
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

	/* init uart4 rx thread */
	retval = rt_thread_init(&uart4_rx_thread, 
                            "tRX4",
							uart4_rx_entry,
							RT_NULL,
                            (rt_uint8_t*)&uart4_rx_stack[0],
                            sizeof(uart4_rx_stack), 12, 5);
	if (retval == RT_EOK)
	{
		rt_kprintf("thread \"uart4_rx\"start.\n");
		rt_thread_startup(&uart4_rx_thread);
	}
    
    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(RT_TICK_PER_SECOND);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(RT_TICK_PER_SECOND);
    }
    
    return RT_EOK;
}

int cmd_mavlink(int argc, char **argv)
{
    rt_uint8_t msg_id = 87;
    rt_uint8_t *buff = rt_malloc(mav_msg_length[msg_id] + 8);
    //rt_kprintf("MAVLINK:#%d, msg_length=%d, msg_crc=%d\n", msg_id, mav_msg_length[msg_id], mav_msg_crc[msg_id]);
    
    rt_memset(buff, 0x00, mav_msg_length[msg_id]);
    
    mavlink_build_MSG87(buff, mav_msg_crc[msg_id], 30, 60, 90);
    
    print_rawdata(buff, mav_msg_length[msg_id] + 8, "MSG87");

    rt_device_write(pUart2, 0, buff, sizeof(buff));

    return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_mavlink, __cmd_mavlink, Test the mavlink funcation.);

