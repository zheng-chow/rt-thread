/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-08     obito0   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <app_can.h>

//#define DRV_DEBUG
#define LOG_TAG             "app.main"
#include <drv_log.h>

/* defined the LED0 pin: PC10 */
#define LED0_PIN        GET_PIN(C, 10)

#ifdef TEST

#include "svpwmhelper.h"
#include <drv_foc.h>
#include <drv_hall.h>


#define TEST_GET_INDEX() ((GPIOC->IDR >> 6) & 0x07)
void test_hall(void){
    uint8_t state = 0, lstate = 0;
    while (1){
        state = TEST_GET_INDEX();
        if (state == lstate) continue;
        rt_kprintf("%d\n", state);
        lstate = state;
    }
}
void test_pwm(void){
    rt_device_t pwm_dev = rt_device_find("pwmcom");  
    rt_device_t hall_dev = rt_device_find("hall");  
    RT_ASSERT(pwm_dev && hall_dev);
    rt_uint32_t evt;
    float mcoeff = 0.1;
    float eangle = 0; 
    float dangle = 10;
    if (RT_EOK != rt_device_open(hall_dev, 0)){
        LOG_E("hall open failed");
        return;
    }
    rt_thread_mdelay(2000);    
    rt_device_open(pwm_dev, 0); 
    pwm_update_t upd;
    rt_device_read(hall_dev, 0, &eangle, 4);

    while (1){
        upd.coefM = mcoeff;
        upd.detAngle = eangle;
        rt_device_control(pwm_dev, PWM3_CONTROL_UPDATE, &upd);
        rt_thread_mdelay(10);
        rt_kprintf("%d\t%d\n",(int)eangle,(GPIOC->IDR >> 6) & 7);
        eangle+=dangle;
        if (eangle>=360) eangle-=360;
    } 
}
#define TEST_OFFS_ANGLE  (-178 - 120 +30)
void test_offs(void){
		static int16_t baseAngleTable[8] = {-1, 0 + TEST_OFFS_ANGLE, 120+TEST_OFFS_ANGLE, 60 + TEST_OFFS_ANGLE, 240 + TEST_OFFS_ANGLE, 300 + OFFS_ANGLE, 180 + TEST_OFFS_ANGLE, -1};
    rt_device_t pwm_dev = rt_device_find("pwmcom");  
    rt_device_t hall_dev = rt_device_find("hall");  
    RT_ASSERT(pwm_dev && hall_dev);
    rt_uint32_t evt;
    float mcoeff = 0.1;
    float dangle = 90;
    float eangle = 0; 
    if (RT_EOK != rt_device_open(hall_dev, 0)){
        LOG_E("hall open failed");
        return;
    }
    rt_thread_mdelay(2000);    
    rt_device_open(pwm_dev, 0); 
    pwm_update_t upd;
    while (1){
				int idx = TEST_GET_INDEX();
        eangle = baseAngleTable[idx]+hall_fast_get_angle()+dangle;
        if (eangle>=360) eangle-=360;
				else if (eangle < 360) eangle = 360;
        upd.coefM = mcoeff;
        upd.detAngle = eangle;
        rt_device_control(pwm_dev, PWM3_CONTROL_UPDATE, &upd);
        rt_thread_mdelay(10);
        rt_kprintf("hall:\t%d\ttarget:\t%d\thall_sec:\t%d\n",baseAngleTable[idx],(int)eangle, idx);
    } 
}
#endif

rt_bool_t g_init = RT_TRUE;

int main(void){
	rt_uint16_t on_tick, off_tick;
	rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
	can_addrpad_init();
	on_tick = can_addrpad_value();
	if (on_tick == 0){
		on_tick = 100;
		off_tick = 900;
	}
	else if (on_tick == 3){
			on_tick = 200;
			off_tick = 800;
	}
	else{
		on_tick = 500;
		off_tick = 500;
	}
	//test_pwm();
	//test_offs();
  //test_hall();
	while (1){
		rt_pin_write(LED0_PIN, PIN_HIGH);   
		rt_thread_mdelay(on_tick);
		rt_pin_write(LED0_PIN, PIN_LOW);  
		rt_thread_mdelay(off_tick);
	}
	return 0;   
}


















#ifdef BSP_USING_PWM1
#include "svpwmhelper.h"
#ifdef RT_USING_FINSH
#include <finsh.h>
static void set_wave(uint8_t argc, char **argv){
    uint32_t hz = 20000;//20KHz
    float angle = 0.f;
    float mcoeff = 0.3f;
    uint32_t period = 1000000000 / hz;
    if ((argc == 2) || (argc == 3)){
        angle = atof(argv[1]);
    }
    if (argc == 3){
        mcoeff = atof(argv[2]);
    }
    if (mcoeff > 0){
        if (mcoeff > 0.6) mcoeff=0.6f;
        mcoeff *= 65535;
        uint32_t sa = period * Svpwm_PhaseA(mcoeff, angle);
        uint32_t sb = period * Svpwm_PhaseB(mcoeff, angle);
        uint32_t sc = period * Svpwm_PhaseC(mcoeff, angle);
        rt_kprintf("pwm period: %u ns\n", period);
        rt_kprintf("\tpwm chA: %u ns\n", sa);
        rt_kprintf("\tpwm chB: %u ns\n", sb);
        rt_kprintf("\tpwm chC: %u ns\n", sc);
        
        struct rt_device_pwm* drvPwm = (struct rt_device_pwm *)rt_device_find("pwm1");
        
        rt_pwm_set(drvPwm, 1, period, sa);
        rt_pwm_set(drvPwm, 2, period, sb);
        rt_pwm_set(drvPwm, 3, period, sc);        
    }  
    else{
        rt_kprintf("");
    }
}
FINSH_FUNCTION_EXPORT_ALIAS(set_wave, __cmd_set_wave, set wave);
#endif /* RT_USING_FINSH */
#endif /* BSP_USING_PWM1 */


