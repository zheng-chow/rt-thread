/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef RT_USING_ULOG
#define LOG_TAG     "app.main"
#define LOG_LVL     LOG_LVL_DBG
#include <ulog.h>
#endif

#include <shell.h>
#include <fal.h>
#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>

/* defined the LED pin*/
#define PIN_UPDATE_LED      GET_PIN(B, 0)
#define PIN_RUNNIG_LED      GET_PIN(B, 1)
#define PIN_MODE_SELECT     GET_PIN(B, 3)

static rt_sem_t sem_blrx;

int main(void)
{   
    /* set LED0 pin mode to output */
    rt_pin_mode(PIN_UPDATE_LED,  PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_RUNNIG_LED, PIN_MODE_OUTPUT);
    rt_pin_mode(PIN_MODE_SELECT, PIN_MODE_INPUT);
    
    while(RT_TRUE) {

        rt_pin_write(PIN_RUNNIG_LED, PIN_LOW);
        rt_thread_delay(RT_TICK_PER_SECOND);
        rt_pin_write(PIN_RUNNIG_LED, PIN_HIGH);
        rt_thread_delay(RT_TICK_PER_SECOND / 5);
    }
    
    // never be here.
}

rt_err_t bootloader_rx_indicate(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(sem_blrx);
    
    return RT_EOK;
}

int wait_key_until_autoboot(void)
{
    rt_device_t dev = RT_NULL;
    rt_err_t result;
    int timeout = 5;
    const char* DEVICE_NAME = RT_CONSOLE_DEVICE_NAME;
    
    dev = rt_device_find(DEVICE_NAME);
    if (dev == RT_NULL) {
        LOG_E("%s not found", DEVICE_NAME);
        return -RT_EIO;
    }
    sem_blrx = rt_sem_create("blrx", 0, RT_IPC_FLAG_FIFO);
    if (sem_blrx == RT_NULL) {
        LOG_E("create semaphore failed");
        return -RT_ENOMEM;
    }
    
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    rt_device_set_rx_indicate(dev, bootloader_rx_indicate);
    rt_kprintf("Hit any key to stop autoboot: %d sec", timeout);
    while(timeout) {
        rt_kprintf("\rHit any key to stop autoboot: %d sec", --timeout);
        result = rt_sem_take(sem_blrx, RT_TICK_PER_SECOND);
        if (result != -RT_ETIMEOUT) {
            break;
        }
    }
    
    rt_device_set_rx_indicate(dev, RT_NULL);
    rt_device_close(dev);
    rt_sem_delete(sem_blrx);
    
    if (!timeout) {
        LOG_RAW("\nJumping to 0x%08X to boot...\n", 0x08020000);
    }
    else {
        LOG_RAW("\nEnter shell...\n");
    }
    
    return RT_EOK;
}
INIT_APP_EXPORT(wait_key_until_autoboot);

#define BOOTLOADER_VERSION "0.2.0"
int check_update_bin(void)
{
    const struct fal_partition *kernel;
    int fd = RT_NULL;
    
    LOG_I("Welcome! bootloader version %s", BOOTLOADER_VERSION);
    
    const char *partition = "kernel";
    kernel = fal_partition_find(partition);
    if (kernel == RT_NULL) {
        LOG_E("%s, partition not found!", partition);
        return -RT_EIO;
    }
    
    const char *filename = "update.bin";
    fd = open(filename, O_RDONLY, 0);
    if (fd == RT_NULL) {
        LOG_W("%s, file not found!", filename);
    }
    
    off_t fsize = lseek(fd, 0, SEEK_END);
    LOG_I("file size %d", fsize);
    
    close(fd);
    
    return RT_EOK;
}
