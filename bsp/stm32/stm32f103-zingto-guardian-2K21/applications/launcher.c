 /*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-08     zhouzheng   first version
 */

#include "launcher.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <cjson.h>

#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>

#include <drv_sbus.h>

const struct json_ctrl_method support_method[] = 
{
    {"sbus", "uart4", "Futaba"},
    {"uart", "uart5", "ZINGTo"},
};

const struct json_cmd_config default_command_list[] =
{
    // basic cmds.
    {0x00, 0x00, RT_FALSE, "SBGC_cmdStopAll"},  //
    {0x00, 0x08, RT_FALSE, "SBGC_cmdPitchUp"},  //
    {0x00, 0x10, RT_FALSE, "SBGC_cmdPitchDown"}, //
    {0x00, 0x04, RT_FALSE, "SBGC_cmdYawLeft"}, //
    {0x00, 0x02, RT_FALSE, "SBGC_cmdYawRight"}, //
    {0x00, 0x20, RT_FALSE, "VISCA_cmdZoomIn"}, //
    {0x00, 0x40, RT_FALSE, "VISCA_cmdZoomOut"}, //
    {0x00, 0x60, RT_FALSE, "VISCA_cmdZoomStop"}, //
    // record cmds.
    {0x12, 0x00, RT_FALSE, "HI3521_execPhotoRecord"}, //
    {0x12, 0x01, RT_FALSE, "HI3521_execVideoRecordStart"}, //
    {0x12, 0x02, RT_FALSE, "HI3521_execVideoRecordStop"}, //
    {0x12, 0x03, RT_FALSE, "HYK_setModeDay"},
    {0x12, 0x04, RT_FALSE, "HYK_setModeNight"},
    // gimbal cmds.
    {0x13, 0x00, RT_FALSE, "SBGC_execMenuHeadfree"}, //
    {0x13, 0x01, RT_FALSE, "SBGC_execMenuHeadlock"}, //
    {0x13, 0x02, RT_FALSE, "SBGC_execMenuLookdown"}, //
    {0x13, 0x03, RT_FALSE, "SBGC_execMenuHomepos"}, //
    // gimbal angle cmds.
    {0x18, 0x01, RT_FALSE, "SBGC_cmdPitchAngle"},
    {0x18, 0x02, RT_FALSE, "SBGC_cmdYawAngle"},
    // inqury data cmds.
    {0xA0, 0x00, RT_TRUE,  "SBGC_cmdInquryAngle"},
    {0xA0, 0x01, RT_TRUE,  "SYS_cmdInquryVersion"}, //
    {0xA0, 0x02, RT_TRUE,  "EV7500_cmdInquryZoomPos"},
};

const struct json_ch_config default_channel_list[] = 
{
    {"ch1", RT_NULL, "SBGC_setRollRefAngle", RT_NULL, RT_NULL, RT_NULL},
    {"ch2", RT_NULL, "SBGC_setPitchSpeed", RT_NULL, RT_NULL, RT_NULL},
    {"ch3", RT_NULL, "VISCA_execCameraZoom", RT_NULL, RT_NULL, RT_NULL},
    {"ch4", RT_NULL, "SBGC_setYawSpeed", RT_NULL, RT_NULL, RT_NULL},
    {"ch5", RT_NULL, RT_NULL, "SBGC_execMenuLookdown", "SBGC_execMenuHeadfree", "SBGC_execMenuHeadlock"},
    {"ch6", RT_NULL, RT_NULL, "HI3521_execPhotoRecord", RT_NULL, "HI3521_trigVideoRecordStatus"},
    {"ch7", RT_NULL, RT_NULL, "SUMBOY_execTrackerStart", RT_NULL, "SUMBOY_execTrackerStop"},
    {"ch8", "EV7500_servAdjustRotateSpeed", RT_NULL, "SBGC_execMenuHomepos", RT_NULL, RT_NULL},
    {"ch9", RT_NULL, RT_NULL, "IRAY_varyIRCameraColorPalette", RT_NULL, "HI3521_varyVideoPIPMode"},
};

const sbus_thr_t default_thr = {100, 600, 1200, 2000};

rt_err_t make_uart_defconfig_jsonfile(const char *filename)
{
    int    fd = RT_NULL;
    cJSON *root = RT_NULL;
    cJSON *body = RT_NULL;
    cJSON *node = RT_NULL;
    
    fd = open(filename, O_WRONLY | O_CREAT, 0);
    if (fd < 0) {
        LOG_E("Create %s file failed.", filename);
        return -RT_EIO;
    }
    // create header domain
    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "project", cJSON_CreateString("zingto-guardian-2k21"));
    
    cJSON_AddItemToObject(root, "build", cJSON_CreateString(__DATE__\
                                                            " " \
                                                            __TIME__));
    
    cJSON_AddItemToObject(root, "token", cJSON_CreateString(UART_DEFAULT_TOKEN));
    // create method domain
    body = cJSON_CreateArray();
    node = cJSON_CreateObject();
    const struct json_ctrl_method *cmd_method = &support_method[1];
    cJSON_AddItemToObject(node, "type", cJSON_CreateString(cmd_method->type));
    cJSON_AddItemToObject(node, "port", cJSON_CreateString(cmd_method->port));
    cJSON_AddItemToObject(node, "spec", cJSON_CreateString(cmd_method->spec));
    cJSON_AddItemToArray(body, node);
    cJSON_AddItemToObject(root, "method", body);
    // create channel contrl domain
    body = cJSON_CreateArray();
    for(int i = 0; i < (sizeof(default_command_list) / sizeof(struct json_cmd_config)); i++) {
        const struct json_cmd_config *cfg_inst = &default_command_list[i];

        node = cJSON_CreateObject();
        cJSON_AddItemToObject(node, "cmd_code1", cJSON_CreateNumber(cfg_inst->cmd_code1));
        cJSON_AddItemToObject(node, "cmd_code2", cJSON_CreateNumber(cfg_inst->cmd_code2));
        cJSON_AddItemToObject(node, "ack_flag", cJSON_CreateBool(cfg_inst->ack_flag));
        
        // check default config, make up json struct node.
        if (cfg_inst->exec_call_desc != RT_NULL)
            cJSON_AddItemToObject(node, "exec_call", cJSON_CreateString(cfg_inst->exec_call_desc));
        else
            cJSON_AddItemToObject(node, "exec_call", cJSON_CreateString(JSON_STRING_NULL));
        
        cJSON_AddItemToArray(body, node);
    }
    cJSON_AddItemToObject(root, "command", body);
    
    char *json = cJSON_Print(root);
    
    write(fd, json, rt_strlen(json));
    close(fd);
    
    rt_free(json);
    cJSON_Delete(root);
    
    return RT_EOK;
}

rt_err_t make_sbus_defconfig_jsonfile(const char *filename)
{
    int    fd = RT_NULL;
    cJSON *root = RT_NULL;
    cJSON *body = RT_NULL;
    cJSON *node = RT_NULL;
    
    fd = open(filename, O_WRONLY | O_CREAT, 0);
    if (fd < 0) {
        LOG_E("Create %s file failed.", filename);
        return -RT_EIO;
    }
    // create header domain
    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "project", cJSON_CreateString("zingto-guardian-2k21"));
    
    cJSON_AddItemToObject(root, "build", cJSON_CreateString(__DATE__\
                                                            " " \
                                                            __TIME__));
    
    cJSON_AddItemToObject(root, "token", cJSON_CreateString(SBUS_DEFAULT_TOKEN));
    // create method domain
    body = cJSON_CreateArray();
    node = cJSON_CreateObject();
    const struct json_ctrl_method *rc_method = &support_method[0];
    cJSON_AddItemToObject(node, "type", cJSON_CreateString(rc_method->type));
    cJSON_AddItemToObject(node, "port", cJSON_CreateString(rc_method->port));
    cJSON_AddItemToObject(node, "spec", cJSON_CreateString(rc_method->spec));
    cJSON_AddItemToArray(body, node);
    cJSON_AddItemToObject(root, "method", body);
    // create channel contrl domain
    body = cJSON_CreateArray();
    for(int i = 0; i < (sizeof(default_channel_list) / sizeof(struct json_ch_config)); i++) {
        const struct json_ch_config *ch_config = &default_channel_list[i];
        if (ch_config->name_desc == RT_NULL) continue;

        node = cJSON_CreateObject();
        cJSON_AddItemToObject(node, "name", cJSON_CreateString(ch_config->name_desc));
        cJSON_AddItemToObject(node, "thr_low", cJSON_CreateNumber(default_thr.low));
        cJSON_AddItemToObject(node, "thr_high", cJSON_CreateNumber(default_thr.high));
        // check default config, make up json struct node.
        if (ch_config->always_call_desc != RT_NULL)
            cJSON_AddItemToObject(node, "always_call", cJSON_CreateString(ch_config->always_call_desc));
        else
            cJSON_AddItemToObject(node, "always_call", cJSON_CreateString(JSON_STRING_NULL));
        
        if (ch_config->value_vary_desc != RT_NULL)
            cJSON_AddItemToObject(node, "value_vary", cJSON_CreateString(ch_config->value_vary_desc));
        else
            cJSON_AddItemToObject(node, "value_vary", cJSON_CreateString(JSON_STRING_NULL));
        
        if (ch_config->status_high_desc != RT_NULL)
            cJSON_AddItemToObject(node, "status_high", cJSON_CreateString(ch_config->status_high_desc));
        else
            cJSON_AddItemToObject(node, "status_high", cJSON_CreateString(JSON_STRING_NULL));
        
        if (ch_config->status_mid_desc != RT_NULL)
            cJSON_AddItemToObject(node, "status_mid", cJSON_CreateString(ch_config->status_mid_desc));
        else
            cJSON_AddItemToObject(node, "status_mid", cJSON_CreateString(JSON_STRING_NULL));
        
        if (ch_config->status_low_desc != RT_NULL)
            cJSON_AddItemToObject(node, "status_low", cJSON_CreateString(ch_config->status_low_desc));
        else
            cJSON_AddItemToObject(node, "status_low", cJSON_CreateString(JSON_STRING_NULL));
        
        cJSON_AddItemToArray(body, node);
    }
    cJSON_AddItemToObject(root, "channel", body);
    
    char *json = cJSON_Print(root);
    
    write(fd, json, rt_strlen(json));
    close(fd);
    
    rt_free(json);
    cJSON_Delete(root);
    
    return RT_EOK;
}

static char iodev_name[RT_NAME_MAX] = {0};
static struct ch_config *sbus_cfglist = RT_NULL;
static struct cmd_config *uart_cmdlist = RT_NULL;
static int channel_count = 0;
static int command_count = 0;

static void sbus_update_scheduler(rt_uint16_t *val, rt_uint32_t flag)
{
    struct ch_config *config = RT_NULL;
    
    for (int i = 0; i < channel_count; i++) {
        config = &sbus_cfglist[i];
        
        if (flag & (1 << config->offset)) {
            rt_mb_send_wait(config->mb, (rt_ubase_t)val[i], 0);
        }
    }
}

static void sbus_channel_overseer(void * parameter)
{
    const struct ch_config *config = (struct ch_config*)parameter;
    sbus_status_t prev_status = SBUS_INITIAL, new_status;
    rt_uint32_t value;
    rt_err_t result;
    
    while(RT_TRUE) {
        result = rt_mb_recv(config->mb, (rt_ubase_t*)&value, RT_TICK_PER_SECOND / 50);
        
        if (result == -RT_ETIMEOUT) {
            if (config->always_call) config->always_call(&value); // exec when no change.
            continue;
        }
        
        if (config->value_vary) config->value_vary(&value); // exec when value varying.

        if ( value < config->thr_low)
            new_status = SBUS_LOW;
        else if (value < config->thr_high)
            new_status = SBUS_MID;
        else if (value < default_thr.max)
            new_status = SBUS_HIGH;
        else
            new_status = SBUS_INVAILD;
        
        if ( prev_status == new_status) {
            continue;
        }
        else if (prev_status == SBUS_INITIAL) {
            prev_status = new_status; continue;
        }
        
        if (new_status == SBUS_HIGH) {
            if (config->status_high) config->status_high(&value);
        }
        else if (new_status == SBUS_MID) {
            if (config->status_mid) config->status_mid(&value);
        }
        else if (new_status == SBUS_LOW) {
            if (config->status_low) config->status_low(&value);
        }
        
        prev_status = new_status;
        //LOG_I("%d prev %d next %d", config->offset, prev_status, new_status);
    }
}

static rt_sem_t uart_cb_sem = RT_NULL;
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size) {
    rt_sem_release(uart_cb_sem); return RT_EOK;
}

static void uart_command_overseer(void *parameter)
{
    RT_ASSERT(parameter != RT_NULL);
    rt_device_t devinst = rt_device_find((char *)parameter);
    if (devinst == RT_NULL) {
        LOG_E("can't find \"\", commnad control failed.", (char *)parameter); return;
    }
    
    
}

static int sbus_dynacall_launcher(void)
{
    int fd = RT_NULL;
    char objname[RT_NAME_MAX];
    cJSON *root = RT_NULL, *array = RT_NULL, *node = RT_NULL, *item = RT_NULL;
    char *cbuf = RT_NULL;
    struct ch_config *config = RT_NULL;
    
    LOG_I("Loading %s...", SBUS_DEFCONFIG_FILENAME);
    
    fd = open(SBUS_DEFCONFIG_FILENAME, O_RDONLY, 0);
    if (fd < 0) {
        LOG_W("Open failed, make a default config file instead...");
        make_sbus_defconfig_jsonfile(SBUS_DEFCONFIG_FILENAME);
        LOG_I("default config file has made.");
        
        fd = open(SBUS_DEFCONFIG_FILENAME, O_RDONLY, 0);
        RT_ASSERT(fd >= 0);
    }
    
    rt_size_t fsize = lseek(fd, 0, SEEK_END);
    
    cbuf = rt_malloc(fsize);
    RT_ASSERT(cbuf != RT_NULL);
    
    lseek(fd, 0, SEEK_SET);
    read(fd, cbuf, fsize);
    LOG_I("%s size %d.", SBUS_DEFCONFIG_FILENAME, fsize);
    close(fd);
    
    root = cJSON_Parse(cbuf);
    rt_free(cbuf); cbuf = RT_NULL;
    
    if (root == RT_NULL) {
        LOG_E("JSON Parse failed..."); goto _safe_exit;
    }
    
    item = cJSON_GetObjectItem(root, "token");
    if (item == RT_NULL) {
        LOG_E("No Item named \"token\". quit..."); goto _safe_exit;
    }
    
    cbuf = item->valuestring;
    if (rt_strcmp(cbuf, SBUS_DEFAULT_TOKEN) != 0) {
        LOG_E("Token is not matched. quit..."); goto _safe_exit;
    }
    
    array = cJSON_GetObjectItem(root, "channel");
    if (array == RT_NULL) {
        LOG_E("No Item named \"channel\", quit..."); goto _safe_exit;
    }
    channel_count = cJSON_GetArraySize(array);
    
    sbus_cfglist = rt_malloc( sizeof(struct ch_config)*channel_count );
    RT_ASSERT(sbus_cfglist != RT_NULL);
    rt_memset(sbus_cfglist, 0, sizeof(struct ch_config)*channel_count);

    for (int i = 0; i < channel_count; i++) {
        node = cJSON_GetArrayItem(array, i);
        config = &sbus_cfglist[i];
        item = cJSON_GetObjectItem(node, "name");
        if (item == RT_NULL) {
            LOG_W("No Item named \"name\". skip then continue."); continue;
        }
        cbuf = item->valuestring;
        if (rt_strncmp("ch", cbuf, 2) != 0) {
            LOG_W("%s can not match \"ch*\", skip then continue."); continue;
        }
        // set channel offset
        config->offset = atoi(&cbuf[2]) - 1;
        // create mailbox ipc.
        rt_memset(objname, 0, RT_NAME_MAX);
        rt_snprintf(objname, RT_NAME_MAX, "mb_%s", cbuf);
        config->mb = rt_mb_create(objname, 5, RT_IPC_FLAG_FIFO);
        // create thread ipc.
        rt_memset(objname, 0, RT_NAME_MAX);
        rt_snprintf(objname, RT_NAME_MAX, "overseer_%s", cbuf);
        config->pid = rt_thread_create(objname, sbus_channel_overseer, config, 1024, 15, 20);
        // set threshold. if no item, use default value.
        item = cJSON_GetObjectItem(node, "thr_low");
        if (item)
            config->thr_low = item->valueint;
        else
            config->thr_low = default_thr.low;
        
        item = cJSON_GetObjectItem(node, "thr_high");
        if (item)
            config->thr_high = item->valueint;
        else
            config->thr_high = default_thr.high;
        // lookup dynacall table and setup the correct call.
        item = cJSON_GetObjectItem(node, "always_call");
        if (item) {
            cbuf = item->valuestring;
            if (rt_strcmp(cbuf, "null") != 0) {
                config->always_call = support_dynacall_lookup(cbuf);
                if (!config->always_call)
                    LOG_W("%s not found, skip.", cbuf);
            }
        }
        item = cJSON_GetObjectItem(node, "value_vary");
        if (item) {
            cbuf = item->valuestring;
            if (rt_strcmp(cbuf, "null") != 0) {
                config->value_vary = support_dynacall_lookup(cbuf);
                if (!config->value_vary)
                    LOG_W("%s not found, skip.", cbuf);
            }
        }
        item = cJSON_GetObjectItem(node, "status_high");
        if (item) {
            cbuf = item->valuestring;
            if (rt_strcmp(cbuf, "null") != 0) {
                config->status_high = support_dynacall_lookup(cbuf);
                if (!config->status_high)
                    LOG_W("%s not found, skip.", cbuf);
            }
        }
        item = cJSON_GetObjectItem(node, "status_mid");
        if (item) {
            cbuf = item->valuestring;
            if (rt_strcmp(cbuf, "null") != 0) {
                config->status_mid = support_dynacall_lookup(cbuf);
                if (!config->status_mid)
                    LOG_W("%s not found, skip.", cbuf);
            }
        }
        item = cJSON_GetObjectItem(node, "status_low");
        if (item) {
            cbuf = item->valuestring;
            if (rt_strcmp(cbuf, "null") != 0) {
                config->status_low = support_dynacall_lookup(cbuf);
                if (!config->status_low)
                    LOG_W("%s not found, skip.", cbuf);
            }
        }
        RT_ASSERT(config->pid != RT_NULL);
        rt_thread_startup(config->pid);
        
        LOG_I("%s start, call tab: [%c] [%c] [%c] [%c] [%c]", config->pid->name
                        , config->always_call?'Y':'#'
                        , config->value_vary?'Y':'#'
                        , config->status_high?'Y':'#'
                        , config->status_mid?'Y':'#'
                        , config->status_low?'Y':'#' );
    }
    
    sbus_set_update_callback(sbus_update_scheduler);
    
_safe_exit:
    cJSON_Delete(root);
    return RT_EOK;
}

static int uart_dynacall_launcher(void)
{
    int fd = RT_NULL;
    cJSON *root = RT_NULL, *array = RT_NULL, *node = RT_NULL, *item = RT_NULL;
    char *cbuf = RT_NULL;
    struct cmd_config *config = RT_NULL;
    
    LOG_I("Loading %s...", UART_DEFCONFIG_FILENAME);

    fd = open(UART_DEFCONFIG_FILENAME, O_RDONLY, 0);
    if (fd < 0) {
        LOG_W("Open failed, make a default config file instead...");
        make_uart_defconfig_jsonfile(UART_DEFCONFIG_FILENAME);
        LOG_I("default config file has made.");
        
        fd = open(UART_DEFCONFIG_FILENAME, O_RDONLY, 0);
        RT_ASSERT(fd >= 0);
    }
    
    rt_size_t fsize = lseek(fd, 0, SEEK_END);
    
    cbuf = rt_malloc(fsize);
    RT_ASSERT(cbuf != RT_NULL);
    
    lseek(fd, 0, SEEK_SET);
    read(fd, cbuf, fsize);
    LOG_I("%s size %d.", UART_DEFCONFIG_FILENAME, fsize);
    close(fd);

    root = cJSON_Parse(cbuf);
    rt_free(cbuf); cbuf = RT_NULL;
    
    if (root == RT_NULL) {
        LOG_E("JSON Parse failed..."); goto _safe_exit;
    }

    item = cJSON_GetObjectItem(root, "token");
    if (item == RT_NULL) {
        LOG_E("No Item named \"token\". quit..."); goto _safe_exit;
    }
    
    cbuf = item->valuestring;
    if (rt_strcmp(cbuf, UART_DEFAULT_TOKEN) != 0) {
        LOG_E("Token is not matched. quit..."); goto _safe_exit;
    }
    // prase method.
    array = cJSON_GetObjectItem(root, "method");
    if (array == RT_NULL) {
        LOG_E("No Item named \"method\", quit..."); goto _safe_exit;
    }
    node = cJSON_GetArrayItem(array, 0);
    item = cJSON_GetObjectItem(node, "type");
    if (item == RT_NULL) {
        LOG_W("No Item named \"type\". skip then continue."); goto _safe_exit;
    }
    cbuf = item->valuestring;
    if (rt_strcmp(cbuf, "uart") != 0) {
        LOG_E("method type is not uart. quit..."); goto _safe_exit;
    }
    item = cJSON_GetObjectItem(node, "port");
    if (item == RT_NULL) {
        LOG_W("No Item named \"port\". skip then continue."); goto _safe_exit;
    }
    cbuf = item->valuestring;
    rt_memset(iodev_name, 0, sizeof(iodev_name));
    rt_memcpy(iodev_name, cbuf, rt_strlen(cbuf));
    LOG_I("method: \"uart\" \"%s\"", iodev_name);
    // prase command.
    array = cJSON_GetObjectItem(root, "command");
    if (array == RT_NULL) {
        LOG_E("No Item named \"command\", quit..."); goto _safe_exit;
    }
    command_count = cJSON_GetArraySize(array);
    uart_cmdlist = rt_malloc( sizeof(struct cmd_config)*command_count );
    RT_ASSERT(uart_cmdlist != RT_NULL);
    rt_memset(uart_cmdlist, 0, sizeof(struct cmd_config)*command_count);

    for (int i = 0; i < command_count; i++) {
        node = cJSON_GetArrayItem(array, i);
        config = &uart_cmdlist[i];
        // cmd code #1
        item = cJSON_GetObjectItem(node, "cmd_code1");
        if (item == RT_NULL) {
            LOG_W("No Item named \"cmd_code1\". skip then continue."); continue;
        }
        config->cmd_code1 = item->valueint;
        // cmd code #2
        item = cJSON_GetObjectItem(node, "cmd_code2");
        if (item == RT_NULL) {
            LOG_W("No Item named \"cmd_code1\". skip then continue."); continue;
        }
        config->cmd_code2 = item->valueint;
        // is ack flag
        item = cJSON_GetObjectItem(node, "ack_flag");
        if (item == RT_NULL) {
            LOG_W("No Item named \"ack_flag\". skip then continue."); continue;
        }
        config->ack_flag = item->valueint;
        // lookup dynacall table and setup the correct call.
        item = cJSON_GetObjectItem(node, "exec_call");
        if (item) {
            cbuf = item->valuestring;
            if (rt_strcmp(cbuf, "null") != 0) {
                config->exec_call = support_dynacall_lookup(cbuf);
                if (!config->exec_call)
                    LOG_W("%s not found, skip.", cbuf);
            }
        }

        LOG_I("filter: 0x%02X 0x%02X 0x%08X %s"
                        , config->cmd_code1
                        , config->cmd_code2
                        , config->exec_call
                        , config->ack_flag?"true":"false");
    }
    
_safe_exit:
    cJSON_Delete(root);
    return RT_EOK;
}

#define MODE_PIN       GET_PIN(B, 3)
static int dynacall_launcher(void)
{
    rt_err_t retval = RT_EOK;
    
    rt_pin_mode(MODE_PIN, PIN_MODE_INPUT);
    
    if (rt_pin_read(MODE_PIN) == 1) {
        LOG_I("mode select: SBUS");
        retval = sbus_dynacall_launcher();
    }
    else {
        LOG_I("mode select: UART");
        retval = uart_dynacall_launcher();
    }
    
    return retval;
}
INIT_APP_EXPORT(dynacall_launcher);
