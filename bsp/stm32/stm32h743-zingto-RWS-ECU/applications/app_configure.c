#include <rtthread.h>
#include <rtdevice.h>
#include "cJSON.h"
#define DBG_SECTION_NAME               "configure"
#define LOG_TAG                        "app.cfg"
#define DBG_LEVEL                      DBG_INFO
#include <drv_log.h>

#ifdef RT_USING_DFS
#include <dfs.h>
#include <dfs_fs.h>
#include <dfs_posix.h>
#endif
#include "app_configure.h"
#define CONGIGURE_FILE_NAME "/sdcard/configure.json"
rt_bool_t app_cfg_check(void){
    if (0 == access(CONGIGURE_FILE_NAME, 0)){
        LOG_I("Configure file exist");
        return RT_TRUE;
    }
    LOG_E("Configure file not exist");
    return app_cfg_write_default();
}

rt_bool_t app_cfg_write_default(void){
    char* str = 0;
    int fd = -1;
    cJSON *root = cJSON_CreateObject();
    if (!root) return RT_FALSE;
    cJSON_AddItemToObject(root, "Version", cJSON_CreateString("Ver1.0"));
    cJSON_AddItemToObject(root, "Manufacture", cJSON_CreateString("ZingTo"));
    str = cJSON_Print(root);
    cJSON_Delete(root);
    if (!str) return RT_FALSE;
    LOG_I("Write default configure file");
    fd = open(CONGIGURE_FILE_NAME, O_WRONLY|O_CREAT|O_TRUNC);
    if (fd>=0){
        write(fd, str, rt_strlen(str));
        close(fd);
    }
    else LOG_E("open file '%s' failed\n");
    rt_free(str);
    return (fd>=0)?RT_TRUE:RT_FALSE;
}
