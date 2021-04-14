 /*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-04     zhouzheng   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG   //DBG_INFO
#define DBG_SECTION_NAME  "msh.kdump"
#define DBG_COLOR
#include <rtdbg.h>

#include <finsh.h>
#include <shell.h>

#include "drv_sbus.h"

struct msh_kdump
{
    rt_device_t device;
    rt_err_t retval;
    rt_sem_t sem;
    rt_event_t event;
    char *outbuff;
    rt_bool_t flag;
    rt_uint8_t mode;
    rt_err_t (*old_rx_ind)(rt_device_t dev,  rt_size_t size);
};

static struct msh_kdump *kdump;

typedef enum kdump_mode
{
    KDUMP_MODE_NULL = 0,
    KDUMP_MODE_SBUS,
}kdump_mode_t;

static rt_err_t kdump_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(kdump->sem);
    
    return RT_EOK;
}

#define KDUMP_OUTBUFF_SIZE           256
#define KDUMP_EVENT_SBUS_UPDATE     (1 << 0)
static rt_uint16_t  valFormSBus[SBUS_CHANNEL_COUNT];
static void sbus_kdump_handler(rt_uint16_t *val,  rt_uint32_t flag)
{
    rt_event_send(kdump->event, KDUMP_EVENT_SBUS_UPDATE);
    rt_memcpy(valFormSBus, val, SBUS_CHANNEL_COUNT * sizeof(rt_uint16_t) );
}

int msh_kdump(int argc, char **argv)
{
    rt_err_t result = RT_EOK;
    rt_off_t offset;
    sbus_indicator sbus_ind_backup;
    
    if (argc != 2) {
        LOG_W("Usage: kdump sbus");
        return -RT_EIO;
    }
    
    kdump = rt_malloc(sizeof(struct msh_kdump));
    if (kdump == RT_NULL) {
        kdump->retval = -RT_ENOMEM;
        goto __exit;
    }
    rt_memset(kdump, 0x00, sizeof(struct msh_kdump));
    kdump->flag = RT_TRUE;
    kdump->retval = RT_EOK;
    
    if (rt_strcmp("sbus", argv[1]) == 0)
        kdump->mode = KDUMP_MODE_SBUS;
    else {
        LOG_W("Usage: kdump sbus");
        rt_free(kdump);
        return -RT_ENOMEM;
    }
    // take over the msh device control.
    kdump->device = rt_device_find(RT_CONSOLE_DEVICE_NAME);
    if (kdump->device == RT_NULL) {
        kdump->retval = -RT_ENOSYS;
        goto __exit;
    }
    kdump->old_rx_ind = kdump->device->rx_indicate;
    rt_device_set_rx_indicate(kdump->device, kdump_rx_ind);
    
    kdump->sem = rt_sem_create("semMshKDump", 0, RT_IPC_FLAG_FIFO);
    if (kdump->sem == RT_NULL) {
        kdump->retval = -RT_ENOMEM;
        goto __exit;
    }
    
    kdump->event = rt_event_create("evtMshKDump", RT_IPC_FLAG_FIFO); 
    if (kdump->event == RT_NULL) {
        kdump->retval = -RT_ENOMEM;
        goto __exit;
    }
    
    kdump->outbuff = rt_malloc(KDUMP_OUTBUFF_SIZE);
    if (kdump->outbuff == RT_NULL) {
        kdump->retval = -RT_ENOMEM;
        goto __exit;
    }
    
    if (kdump->mode == KDUMP_MODE_SBUS) {
        sbus_ind_backup = sbus_get_update_callback();
        sbus_set_update_callback(sbus_kdump_handler);
        LOG_RAW("KDump SBUS data in 30Hz\n");
    }
    
    while(kdump->flag) {
        result = rt_sem_take(kdump->sem, RT_TICK_PER_SECOND / 30);
        
        // got some key input, check the ESC code
        if (result == RT_EOK) {
            char input;
            
            while( rt_device_read(kdump->device, 0, &input, 1) ) {
                if (input == 0x03) // ^C = 0x03
                    kdump->flag = RT_FALSE;
            }
            if(kdump->flag == RT_FALSE) continue;
        }
        
        switch(kdump->mode)
        {
            case KDUMP_MODE_SBUS: {
                rt_memset(kdump->outbuff, 0, KDUMP_OUTBUFF_SIZE);
                offset = 0;
                
                for (rt_uint8_t i = 0; i < 8; i++)
                    offset += rt_snprintf(kdump->outbuff + offset, KDUMP_OUTBUFF_SIZE - offset, "%04d ", valFormSBus[i]);
                
                rt_kprintf("%s\r", kdump->outbuff);
                
                break;
            }
            default: {
                rt_kprintf("Message form kdump:NULL\n");
                break;
            }
        }
    }
    
    rt_kprintf("\n");
    
    rt_device_set_rx_indicate(kdump->device, kdump->old_rx_ind);
    
    if (kdump->mode == KDUMP_MODE_SBUS) {
        sbus_set_update_callback(sbus_ind_backup);
    }

    kdump->retval = RT_EOK;
    
__exit:
    
    if (kdump->outbuff) rt_free(kdump->outbuff);
    if (kdump->event) rt_event_delete(kdump->event);
    if (kdump->sem) rt_sem_delete(kdump->sem);
    if (kdump) rt_free(kdump);
    
    return result;
}
MSH_CMD_EXPORT_ALIAS(msh_kdump, kdump, dump miscellaneous data in msh.);
