#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>

#include <fal.h>

#define DBG_ENABLE
#define DBG_SECTION_NAME    "msh.update"
#define DBG_LEVEL           DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#define OTA_APP_START_ADDRESS   0x08100000
#define COPY_BUFFER_SIZE        512

typedef void (*rt_ota_app_func)(void);	
static rt_ota_app_func app_func = RT_NULL;

static int boot_app_manual(int argc, char **argv)
{
    const struct fal_partition *app_dev = fal_partition_find("app");
    
    if (app_dev == RT_NULL) {
        LOG_E("No [app] partition was probed");
        return -RT_EIO;
    }    

    rt_uint32_t addr_pc, addr_sp;
    fal_partition_read(app_dev, 0, (rt_uint8_t*)&addr_sp, 4);
    fal_partition_read(app_dev, 4, (rt_uint8_t*)&addr_pc, 4);

    LOG_D("PC: 0x%08X, SP: 0x%08X", addr_pc, addr_sp);
    
    if ((addr_pc & 0xfff00000) != 0x08100000) {
        LOG_E("Illegal Flash Code");
        return -RT_ERROR;
    }
    
     if ((addr_sp & 0x2ffe0000) != 0x20000000) {
        LOG_E("Illegal Stack Code");
        return -RT_ERROR;
    }
     
    LOG_I("Implement app @ 0x%08X now.", addr_pc);
    
    __disable_irq();

    app_func = (rt_ota_app_func)*(__IO rt_uint32_t*)(OTA_APP_START_ADDRESS + 4);
    __set_MSP(*(__IO rt_uint32_t*)OTA_APP_START_ADDRESS);
    
    app_func();
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(boot_app_manual, boot_app, "boot the app manual");

static int read_app_info(int argc, char **argv)
{
    const struct fal_partition *app_dev = fal_partition_find("app");
    
    if (app_dev == RT_NULL) {
        LOG_E("No [app] partition was probed");
        return -RT_EIO;
    }
    
    rt_uint32_t addr_pc, addr_sp;
    fal_partition_read(app_dev, 0, (rt_uint8_t*)&addr_pc, 4);
    fal_partition_read(app_dev, 4, (rt_uint8_t*)&addr_sp, 4);
    
    LOG_I("PC: 0x%08X, SP: 0x%08X", addr_pc, addr_sp);
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(read_app_info, read_info, "read the app image info");

static int read_register_raw(int argc, char **argv)
{
    if (argc != 2) {
        LOG_W("Usage: read_reg {addr} addr in 0x%08X");
        return -RT_EIO;
    }
    
    rt_uint32_t reg_addr = RT_NULL;
    sscanf(argv[1], "0x%08X", &reg_addr);
    
    rt_kprintf("[0x%08X]: 0x%08X\n", reg_addr, *(rt_uint32_t*)reg_addr);
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(read_register_raw, read_reg, "read_reg [address], read register value then print");

static int copy_app_binary(int argc, char **argv)
{
    const char * filename = "update.bin";
    rt_uint8_t *pbuf = 0;
    rt_size_t  size = 0, offset = 0;
    const struct fal_partition *app_dev = fal_partition_find("app");
    
    if (app_dev == RT_NULL) {
        LOG_E("No [app] partition was probed.");
        return -RT_EIO;
    }
    
    pbuf = rt_malloc(COPY_BUFFER_SIZE);
    if (pbuf == RT_NULL) {
        LOG_E("malloc %d buffer failed", COPY_BUFFER_SIZE);
        return -RT_ENOMEM;
    }
    
    int fd = open(filename, O_RDONLY, 0);
    if (fd < 0) {
        LOG_E("read %s failed, copy binary failed", filename);
        rt_free(pbuf);
        return -RT_EIO;
    }
    
    size = read(fd, pbuf, COPY_BUFFER_SIZE);
    if (size != COPY_BUFFER_SIZE) {
        LOG_E("binary too small, update abort");
        rt_free(pbuf);
        close(fd);
        return -RT_ERROR;
    }

    if ((*(rt_uint32_t*)&pbuf[4] & 0xff000000) != 0x08000000) {
        LOG_E("Illegal Flash Code");
        rt_free(pbuf);
        close(fd);
        return -RT_ERROR;
    }
    
     if ((*(rt_uint32_t*)&pbuf[0] & 0x2ffe0000) != 0x20000000) {
        LOG_E("Illegal Stack Code");
        rt_free(pbuf); 
        close(fd);
        return -RT_ERROR;
    }   
    
    LOG_I("Check update binary Passed, start update");
    LOG_D("erase the app partition...");
    fal_partition_erase_all(app_dev);
    LOG_D("Done");
    do {
        LOG_D("write %03d to 0x%08X...", size, offset);        
        fal_partition_write(app_dev, offset, pbuf, size);
        offset += size;
        size = read(fd, pbuf, COPY_BUFFER_SIZE);
        if (!size)  break;
        if (size % 32 != 0) {
            size = (size/32 + 1) * 32;
        }
    }while(RT_TRUE);

    LOG_I("update finish!");
    
    rt_free(pbuf);
    close(fd);
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(copy_app_binary, copy_bin, "copy update.bin to app partition.");
