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

#define DBG_ENABLE
#define DBG_SECTION_NAME    "app.main"
#define DBG_LEVEL           DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#include "cJSON.h"
#include <netif/ethernetif.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <netdev.h>

#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>

#include "app_env.h"
#include "app_configure.h"

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
	LOG_I("Scan SDRAM Capacty");
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
				LOG_W("SDRAM read first 4B error");
				break;
			}			
		}
		else if(temp<=sval)break;//后面读出的数据一定要比第一次读到的数据大.	 
		else if (deta != (temp - sval)){
				LOG_W("SDRAM read S%u 16K error",deta);
				break;
		}
		deta++;
	}				
	temp = (rt_uint16_t)(temp-sval+1)*16;
    
    for(i=0;i<32*1024*1024;i++){
        *(volatile rt_uint8_t*)(EXT_SDRAM_BEGIN+i) = 0xAA;
    }        

	if (temp %1024)
		LOG_I("S%u SDRAM Capacity: %dKB", deta, temp);//打印SDRAM容量
	else
		LOG_I("SDRAM Capacity: %dMB", temp/1024);//打印SDRAM容量
}

static void assert_hook(const char *ex, const char *func, rt_size_t line)
{
    extern uint32_t SystemCoreClock;
    volatile rt_uint32_t tmp;
    rt_kprintf("(%s) assertion failed at function:%s, line number:%d \n", ex, func, line);
    while(1)
    {
        rt_pin_write(BLINK_PIN, PIN_HIGH);
        rt_pin_write(APP_PIN, PIN_HIGH);
        rt_pin_write(WARN_PIN, PIN_HIGH);
        rt_pin_write(FAULT_PIN, PIN_HIGH);
        tmp = SystemCoreClock/3;while(tmp--);
        rt_pin_write(BLINK_PIN, PIN_LOW);
        rt_pin_write(APP_PIN, PIN_LOW);
        rt_pin_write(WARN_PIN, PIN_LOW);
        rt_pin_write(FAULT_PIN, PIN_LOW);
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
 
#if defined(RT_USING_LWIP)
    lan_load_configSets();
#endif

#ifdef PKG_NETUTILS_TELNET
    extern void telnet_server(void);
    telnet_server();
#endif
    rt_thread_mdelay(1000);

    if (!app_env_init()){
        LOG_E("Initialize application environment failed");
        while(1) rt_thread_delay(RT_TICK_PER_SECOND);
    }
		/*
		while(1){
			Quaternion_t q;
			if (!can_read_q((float*)&q)) {
					rt_kprintf("imu read q failed\n");
			}
			else{
				Eular_t e;
				Q_ToEularGeneral(&q, &e);
				LOG_I("%d\t%d\t%d\n", (int)ToDeg(e.roll), (int)ToDeg(e.pitch), (int)ToDeg(e.yaw));
			}		
			rt_thread_mdelay(100);
		}*/
		/*can_motor_enable(1,RT_TRUE);
		while (1){
			if (can_motor_enable(1,RT_TRUE)){
				LOG_I("can tx&rx success");
			}
			else{
				LOG_W("can tx|rx failed");
			}
			rt_thread_delay(1000);
		}*/		
		
    if (!app_env_start()){
        LOG_E("Start application environment failed");
        while(1) rt_thread_delay(RT_TICK_PER_SECOND);
    }

    while (count++)
    {
        rt_pin_write(BLINK_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(BLINK_PIN, PIN_LOW);
        rt_thread_mdelay(800);
    }    
    return RT_EOK;
}

