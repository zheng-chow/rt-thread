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

#define DBG_ENABLE
#define DBG_SECTION_NAME    "app.main"
#define DBG_LEVEL           DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

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

/* defined the LED0 pin: PB7 */
#define EXT_SDRAM_BEGIN 0xC0000000
static void check_extern_sdram(void){//check sdram
	volatile rt_uint32_t i=0;
	volatile rt_uint32_t temp=0;
	volatile rt_uint32_t deta=0;
	volatile rt_uint32_t sval=0;	//在地址0读到的数据
	rt_kprintf("Scan SDRAM Capacty\n");
	for(i = 0; i < 32 * 1024 * 1024; i += 16 * 1024) {
		*(volatile rt_uint32_t *)(EXT_SDRAM_BEGIN+i)=temp; 
		temp++;
	}
	//依次读出之前写入的数据,进行校验		  
	for(i = 0;i < 32 * 1024 * 1024; i += 16 * 1024) {	
		temp=*(volatile rt_uint32_t*)(EXT_SDRAM_BEGIN+i);
		//rt_kprintf("S%u %08X\n", deta, temp);
		if(i==0){
			sval = temp;
			if (sval != 0){
				rt_kprintf("SDRAM read first 4B error\n");
				break;
			}			
		}
		else if(temp <= sval) break;   //后面读出的数据一定要比第一次读到的数据大.	 
		else if (deta != (temp - sval)){
				rt_kprintf("SDRAM read S%u 16K error\n", deta);
				break;
		}
		deta++;
	}				
	temp = (rt_uint16_t)(temp - sval + 1)*16;
    
    for(i=0;i<32*1024*1024;i++){
        *(volatile rt_uint8_t*)(EXT_SDRAM_BEGIN+i) = 0xAA;
    }        

	if (temp % 1024)
		rt_kprintf("S%u SDRAM Capacity:%dKB\r\n", deta, temp);  //打印SDRAM容量
	else
		rt_kprintf("SDRAM Capacity:%dMB\r\n", temp/1024);   //打印SDRAM容量
}

static void assert_hook(const char *ex, const char *func, rt_size_t line)
{
    extern uint32_t SystemCoreClock;
    volatile rt_uint32_t tmp;
    
    rt_kprintf("(%s) assertion failed at function:%s, line number:%d \n", ex, func, line);
    
    while(1){
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
    LOG_E("%s", context);
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

    while (count++)
    {
        rt_pin_write(BLINK_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(BLINK_PIN, PIN_LOW);
        rt_thread_mdelay(800);
    }
    
    return RT_EOK;
}
