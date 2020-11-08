/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-09-17     tyustli   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef RT_USING_LWIP
#include <netif/ethernetif.h>
#include <netdev.h>
#endif

#if defined(BSP_ADC_USING_DMA)
#include <drv_adc_dma.h>
#endif

#if defined(RT_USING_RUDP)
#include <udp_api.h>
#endif

#include "app_send.h"
#include "app_control.h"

#define DBG_ENABLE
#define DBG_SECTION_NAME    "app.main"
#define DBG_LEVEL           DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

extern void create_sin(void);

#include "cJSON.h"
#include <netif/ethernetif.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <netdev.h>

#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>

rt_err_t load_ethernet_parameter(void)
{
    const char* FILENAME = "netif.json";
    const rt_size_t JSON_BUFFER_SIZE = 1024;
    rt_size_t size = 0;
    char *interface = RT_NULL;
    char *address = RT_NULL;
    char *gateway = RT_NULL;
    char *netmask = RT_NULL;
    
    int fd = RT_NULL;
    cJSON *root = RT_NULL;
    char *json;
    
    fd = open(FILENAME, O_RDONLY, 0);
    if (fd < 0) {
        LOG_W("read config failed! creating a default config");
        
        fd = open(FILENAME, O_WRONLY | O_CREAT, 0);
        if (fd < 0) {
            LOG_E("create config file failed!");
            return -RT_EIO;
        }
        root = cJSON_CreateObject();
        cJSON_AddItemToObject(root, "Interface", cJSON_CreateString("e0"));
        cJSON_AddItemToObject(root, "Address", cJSON_CreateString(RT_LWIP_IPADDR));
        cJSON_AddItemToObject(root, "Gateway", cJSON_CreateString(RT_LWIP_GWADDR));
        cJSON_AddItemToObject(root, "Netmask", cJSON_CreateString(RT_LWIP_MSKADDR));

        json = cJSON_Print(root);
        write(fd, json, rt_strlen(json));
        rt_free(json);
        close(fd);
    }
    else {
        json = rt_malloc(JSON_BUFFER_SIZE);
        if (json == RT_NULL) {
            LOG_E("malloc buffer for json failed!");
            return -RT_ENOMEM;
        }
        
        size = read(fd, json, JSON_BUFFER_SIZE);
        if (size == JSON_BUFFER_SIZE)
            LOG_W("json buffer is full!");
        
        root = cJSON_Parse(json);
        rt_free(json);
        close(fd);
    }
    
    interface = cJSON_GetObjectItem(root, "Interface")->valuestring;
    address   = cJSON_GetObjectItem(root, "Address")->valuestring;
    gateway   = cJSON_GetObjectItem(root, "Gateway")->valuestring;
    netmask   = cJSON_GetObjectItem(root, "Netmask")->valuestring;
    
    if (interface && address && gateway && netmask){
        
        struct netdev * dev = netdev_get_by_name(interface);
        ip_addr_t a, gw, nm; 
        
        if (!dev){
            LOG_E("interface '%s' is not exist", interface);
        }
        else if (!inet_aton(address, &a.addr)){
            LOG_E("address '%s' is invalid", address);
        }    
        else if (!inet_aton(gateway, &gw.addr)){
            LOG_E("gateway '%s' is invalid", gateway);
        }   
        else if (!inet_aton(netmask, &nm.addr)){
            LOG_E("netmask '%s' is invalid", netmask);
        }            
        else{
            
            rt_uint8_t cnt = 15;                
            netdev_set_down(dev);
            netdev_set_ipaddr(dev,&a);
            netdev_set_gw(dev,&gw);                                
            netdev_set_netmask(dev,&nm);                                
            netdev_set_up(dev);                
            LOG_I("config address '%s' finished, wait link up...", address);
            while ((!(dev->flags & NETIF_FLAG_LINK_UP)) && (!cnt)){
                rt_thread_delay(RT_TICK_PER_SECOND/5); cnt--;
            }
            if (cnt > 0){
                LOG_I("netif link up");
            }
            else{
                LOG_E("netif link up failed");
            }
        }
    }
    
    cJSON_Delete(root);
    
    return RT_EOK;
}

/* defined the LED0 pin: PB7 */

#define EXT_SDRAM_BEGIN 0xC0000000
static void check_extern_sdram(void){//check sdram
	volatile rt_uint32_t i=0;  	  
	volatile rt_uint32_t temp=0;	   
	volatile rt_uint32_t deta=0;	   
	volatile rt_uint32_t sval=0;	//在地址0读到的数据	  
	rt_kprintf("Scan SDRAM Capacty\n");
	for(i=0;i<32*1024*1024;i+=16*1024)
	{
		*(volatile rt_uint32_t *)(EXT_SDRAM_BEGIN+i)=temp; 
		temp++;
	}
	//依次读出之前写入的数据,进行校验		  
	for(i=0;i<32*1024*1024;i+=16*1024) 
	{	
		temp=*(volatile rt_uint32_t*)(EXT_SDRAM_BEGIN+i);
		//rt_kprintf("S%u %08X\n", deta, temp);
		if(i==0){
			sval=temp;
			if (sval != 0){
				rt_kprintf("SDRAM read first 4B error\n");
				break;
			}			
		}
		else if(temp<=sval)break;//后面读出的数据一定要比第一次读到的数据大.	 
		else if (deta != (temp - sval)){
				rt_kprintf("SDRAM read S%u 16K error\n",deta);
				break;
		}
		deta++;
	}				
	temp = (rt_uint16_t)(temp-sval+1)*16;
    
    for(i=0;i<32*1024*1024;i++){
        *(volatile rt_uint8_t*)(EXT_SDRAM_BEGIN+i) = 0xAA;
    }        

	if (temp %1024)
		rt_kprintf("S%u SDRAM Capacity:%dKB\r\n",deta, temp);//打印SDRAM容量
	else
		rt_kprintf("SDRAM Capacity:%dMB\r\n", temp/1024);//打印SDRAM容量
}

#if defined(RT_USING_ADC) && defined(BSP_USING_ADC3)
#include <stdio.h>
void check_adc3(rt_uint32_t cnl, rt_uint32_t count, rt_tick_t mdelay)
{
    rt_adc_device_t dev_adc3 = (rt_adc_device_t)rt_device_find("adc3");
    RT_ASSERT(dev_adc3 != RT_NULL);
    
    rt_adc_enable(dev_adc3, cnl);
    
    for (rt_uint32_t n = 0; n < count; n++){
        static char tmp[256];
        rt_uint32_t value = rt_adc_read(dev_adc3, cnl);
        snprintf(tmp, 256, "adc3[ch%d]: %.4f\n", cnl, value*2.5f/4095);
        //rt_kprintf("adc3[ch%d]: %g\n", cnl, value*2.5f/65535);
        rt_kprintf("%s", tmp);
        rt_thread_mdelay(mdelay);
    }
    
    rt_adc_disable(dev_adc3, cnl);
    
}
#endif

static void assert_hook(const char *ex, const char *func, rt_size_t line)
{
    extern uint32_t SystemCoreClock;
    volatile rt_uint32_t tmp;
    rt_kprintf("(%s) assertion failed at function:%s, line number:%d \n", ex, func, line);
    while(1)
    {
        rt_pin_write(BLINK_PIN, PIN_HIGH);
        rt_pin_write(CONV_PIN, PIN_HIGH);
        rt_pin_write(WARN_PIN, PIN_HIGH);
        rt_pin_write(MQTT_PIN, PIN_HIGH);
        tmp = SystemCoreClock/3;while(tmp--);
        rt_pin_write(BLINK_PIN, PIN_LOW);
        rt_pin_write(CONV_PIN, PIN_LOW);
        rt_pin_write(WARN_PIN, PIN_LOW);
        rt_pin_write(MQTT_PIN, PIN_LOW);
        tmp = SystemCoreClock/3;while(tmp--);
    }
}

static rt_err_t exception_handle_hook(void *context)
{
    LOG_I("%s", context);
    return -RT_ERROR;
}

int main(void)
{
    int count = 1;
    /* set LED pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED3_PIN, PIN_MODE_OUTPUT);
    rt_assert_set_hook(assert_hook);
    rt_hw_exception_install(exception_handle_hook);
	check_extern_sdram();
    
#if defined(RT_USING_ADC) && defined(BSP_USING_ADC3)
    check_adc3(6, 10, 20);
#endif

//#define TEST_FAKE_DATA
#ifdef  TEST_FAKE_DATA
#if defined(BSP_ADC_USING_DMA)
    {
        rt_device_t adcdma = rt_device_find("adcdma");
 		static rt_tick_t tick, dtick = 0, ttick = 0;
        rt_uint32_t cnt = 0;
#if defined(RT_USING_RUDP)
        udp_api_handle_t udp_api_hd = udp_api_create(60000, "192.168.3.156", 60002, "udp");
        rt_thread_delay(RT_TICK_PER_SECOND*10);
        rt_kprintf("send start\n");
#endif
        //rt_uint32_t addrbase = 0xC0000000;
        //rt_uint32_t datalen = 0xC0000000;
        //RT_ASSERT(adcdma);
        if (adcdma){
            rt_device_open(adcdma, 0);
            rt_uint32_t offs = 0, offs_bak = 1;
            tick = rt_tick_get();
            while (1){//(cnt <= 10){
                rt_device_control(adcdma, RT_DEVICE_ADC_WAIT_OFFSET, &offs);
                dtick = rt_tick_get() - tick;
                tick += dtick;
                ttick += dtick;
                cnt++;
                if (offs == offs_bak)
                    rt_kprintf("cost %d %u\n", dtick, cnt);
                RT_ASSERT (offs != offs_bak);
                offs_bak = offs;
                
#if defined(RT_USING_RUDP)
                if(0 == cnt%10){
                    udp_api_send(udp_api_hd, (rt_uint8_t*)0xC0000000, 200000 * 6*2);//ADC_BUFF_SZ  1000000
                }
#endif
            }            
            //rt_thread_delay(RT_TICK_PER_SECOND);
            rt_device_close(adcdma);
            rt_kprintf("average: %d ms\n", (ttick * 1000)/cnt/RT_TICK_PER_SECOND);
#if defined(RT_USING_RUDP)
           udp_api_delete(&udp_api_hd);            
#endif
#endif
         /*   
            rt_uint32_t val[2][3]={{0,0,0},{0,0,0}};
            rt_uint16_t* v = (rt_uint16_t*)0xC0000000;
            rt_uint16_t err  = 0;
            for (rt_uint32_t idx = 0; idx < 2; idx++){
                v = (rt_uint16_t*)(0xC0000000 + (200000*6)*idx);
                for (rt_uint32_t n = 0; n < 200000; n++, v+=3){                    
                    val[idx][0] += v[0];
                    val[idx][1] += v[1];
                    val[idx][2] += v[2];
                    if (v[0] >= 0x800){
                        err = (err > (v[0] - 0x800))?err:(v[0] - 0x800);
                    }
                    else{
                        err = (err > (0x800 - v[0]))?err:(0x800 - v[0]);
                    }
                }                
            }
            val[0][0] = val[0][0] / 200000;
            val[0][1] = val[0][1] / 200000;
            val[0][2] = val[0][2] / 200000;
            val[1][0] = val[1][0] / 200000;
            val[1][1] = val[1][1] / 200000;
            val[1][2] = val[1][2] / 200000;
            rt_kprintf("max err: %u \n", err);
            rt_kprintf("signal1: %u %u %u\n", val[0][0], val[0][1], val[0][2]);
            rt_kprintf("signal2: %u %u %u\n", val[1][0], val[1][1], val[1][2]);*/
        }
    }
    
    create_sin();
#endif

#if defined(RT_USING_LWIP)
    load_ethernet_parameter();
#endif

    adc_control_server();

#ifdef PKG_NETUTILS_TELNET
    extern void telnet_server(void);
    telnet_server();
#endif

    while (count++)
    {
        rt_pin_write(BLINK_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(BLINK_PIN, PIN_LOW);
        rt_thread_mdelay(800);
    }
    
    return RT_EOK;
}
