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

#define OTA_APP_START_ADDRESS   0x08100000

/**
 * Function    ota_app_vtor_reconfig
 * Description Set Vector Table base location to the start addr of app(RT_APP_PART_ADDR).
*/
static int ota_app_vtor_reconfig(void)
{
    #define NVIC_VTOR_MASK   0x3FFFFF80
    /* Set the Vector Table base location by user application firmware definition */
    SCB->VTOR = OTA_APP_START_ADDRESS & NVIC_VTOR_MASK;

    return 0;
}
//INIT_BOARD_EXPORT(ota_app_vtor_reconfig);

static int lan_load_configSets(void)
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
        
        rt_thread_delay(RT_TICK_PER_SECOND * 2);
    }
    else {
        json = rt_malloc(JSON_BUFFER_SIZE);
        if (json == RT_NULL) {
            LOG_E("malloc buffer for json failed!");
            return -RT_ENOMEM;
        }
        
        rt_memset(json, 0, JSON_BUFFER_SIZE);
        
        size = read(fd, json, JSON_BUFFER_SIZE);
        if (size == JSON_BUFFER_SIZE)
            LOG_W("json buffer is full!");
        
        if (size == 0) {
            LOG_W("json file is empty");
            close(fd);
            return -RT_EIO;
        }
        
        root = cJSON_Parse(json);
        rt_free(json);
        close(fd);
        
        rt_thread_delay(RT_TICK_PER_SECOND / 2);
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
		else if(temp<=sval)break;   //后面读出的数据一定要比第一次读到的数据大.	 
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
    
    LOG_I("build %s %s", __DATE__, __TIME__);
    
#if defined(RT_USING_ADC) && defined(BSP_USING_ADC3)
    check_adc3(6, 10, 20);
#endif

#if defined(RT_USING_LWIP)
    lan_load_configSets();
#endif

    adc_control_server();

#ifdef PKG_NETUTILS_TELNET
    extern void telnet_server(void);
    telnet_server();
#endif
    
    extern int mqtt_server_init(void);
    rt_thread_delay(RT_TICK_PER_SECOND * 5);
    
    mqtt_server_init();
    
    while (count++)
    {
        rt_pin_write(BLINK_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(BLINK_PIN, PIN_LOW);
        rt_thread_mdelay(800);
    }
    
    return RT_EOK;
}

static int print_uname(int argc, char **argv)
{
    rt_kprintf("rt-thread, jc. build: %s %s\n", __DATE__, __TIME__);
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(print_uname, uname, "print info and build time");
