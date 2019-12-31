/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   change to new framework
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define LOG_TAG             "main"
#include <drv_log.h>

/* defined the LED pin: PA5 */
#define LED0_PIN     GET_PIN(B, 4)
#define LED1_PIN     GET_PIN(B, 5)
#define PWRSEL_PIN	 GET_PIN(B, 6)

#define TIMESET_TIMEOUT     (15)

static rt_sem_t  semPVD = RT_NULL;

void prepare_enter_LPM4(void)
{

}

/**
  * @brief  Configures system clock after wake-up from STOP: enable HSI, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
static void SystemClockConfig_STOP(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Get the Oscillators configuration according to the internal RCC registers */
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

    /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

void PVD_IRQHandler(void)
{
    rt_interrupt_enter();
    
    HAL_PWR_PVD_IRQHandler();
    
    rt_interrupt_leave();
}

static rt_bool_t  PVDOflag = RT_FALSE;

void HAL_PWR_PVDCallback(void)
{
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO)) {
        PVDOflag = RT_TRUE;
        if (semPVD) rt_sem_release(semPVD);
    }    
    else {
        PVDOflag = RT_FALSE;
    }
}

void time_keeper(void *param)
{
    rt_tick_t  reftick;
    char dbuf[32];
    rt_device_t port = RT_NULL;
    
    port = rt_device_find("uart2");
    RT_ASSERT(port != RT_NULL);
    rt_device_open(port, RT_DEVICE_OFLAG_OPEN | RT_DEVICE_OFLAG_RDWR);
    
    while (RT_TRUE) {
        reftick = rt_tick_get();
        
        do {
            rt_pin_write(LED0_PIN, PIN_HIGH);
            rt_thread_mdelay(2);
            rt_pin_write(LED0_PIN, PIN_LOW);
            rt_thread_mdelay(200);
        }while (rt_tick_get() - reftick < RT_TICK_PER_SECOND * TIMESET_TIMEOUT);
        
        rt_pin_write(LED1_PIN, PIN_HIGH);
        
        time_t now;
        /* output current time */
        now = time(RT_NULL);
        struct tm *date = localtime(&now);
        int year = date->tm_year + 1900;
        int month = date->tm_mon + 1; 
        
        rt_memset(dbuf, 0x00, sizeof(dbuf));
        
        rt_sprintf(dbuf, "date -s \"%d-%02d-%02d %02d:%02d:%02d\"\n", year, month, date->tm_mday, 
                                                date->tm_hour, date->tm_min, date->tm_sec);
        
        rt_kprintf("%s", dbuf);
        /* send command to Hi3521D */
        rt_device_write(port, 0, dbuf, rt_strlen(dbuf));
        
        
        rt_thread_mdelay(200);
        rt_pin_write(LED1_PIN, PIN_LOW);
    }
    
    /* never run here */
}

int main(void)
{
    rt_thread_t tp = RT_NULL;
 
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(PWRSEL_PIN, PIN_MODE_OUTPUT);
    
    /* disable PMOS to avoid backup bat power the entire system */
    rt_pin_write(PWRSEL_PIN, PIN_HIGH);
    
    /* show the system reset */
    rt_pin_write(LED0_PIN, PIN_HIGH);
    rt_pin_write(LED1_PIN, PIN_HIGH);
    
    rt_thread_delay(RT_TICK_PER_SECOND/10);
    
    /* show the system reset */
    rt_pin_write(LED0_PIN, PIN_LOW);
    rt_pin_write(LED1_PIN, PIN_LOW);    
    
    semPVD = rt_sem_create("semPVD", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(semPVD != RT_NULL);
    
    tp = rt_thread_create("tidKPR", time_keeper, RT_NULL, 512, 10, 10);
    RT_ASSERT(tp != RT_NULL);
    rt_thread_startup(tp);
    
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO)) {
        LOG_I("Power up, in Battery mode");
        rt_sem_release(semPVD);
    }
    else {
        LOG_I("Power up, in Normal mode");
    }
    
    rt_kprintf("RCC->CSR:\t0x%08x\n", RCC->CSR);
    rt_kprintf("FLASH->OPTR:\t0x%08x\n", FLASH->OPTR);
	
    rt_sem_take(semPVD, RT_WAITING_FOREVER);
    
    LOG_I("Detect Power Off, System enter STOP mode in moment");
    
    /* config the uart GPIO to prevent leak current */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    
    /* deinit the GPIO for LEDs */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);

    /* Enter Stop Mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    /* Configures system clock after wake-up from STOP: enable HSI, PLL and select
        PLL as system clock source (HSI and PLL are disabled automatically in STOP mode) */            
    SystemClockConfig_STOP();
    
    if (PVDOflag)
        LOG_I("Detect Power On, System reset, PVDOflag = TRUE");
    else
        LOG_I("Detect Power On, System reset, PVDOflag = FALSE");
    
    rt_hw_cpu_reset();

    return RT_EOK;
}
