#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <rtthread.h>

#define DBG_ENABLE
#define DBG_SECTION_NAME    "mqtt.jc"
#define DBG_LEVEL           DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

#include "mqtt_client.h"
#include "cJSON.h"
#include "rn8302b.h"
#include <arpa/inet.h>
/**
 * MQTT URI farmat:
 * domain mode
 * tcp://iot.eclipse.org:1883
 *
 * ipv4 mode
 * tcp://192.168.10.1:1883
 * ssl://192.168.10.1:1884
 *
 * ipv6 mode
 * tcp://[fe80::20c:29ff:fe9a:a07e]:1883
 * ssl://[fe80::20c:29ff:fe9a:a07e]:1884
 */

#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>
 
#define MQTT_URI                "tcp://192.168.5.48:1883"
#define MQTT_USERNAME           "admin"
#define MQTT_PASSWORD           "admin"
#define MQTT_SUBTOPIC           "x1000/jc/request"
#define MQTT_PUBTOPIC           "x1000/jc/response"
#define MQTT_WILLMSG            ""

struct MQTT_Configuration 
{
    char URL[128];
    char Username[32];
    char Password[32];
    char SubTopic[128];
    char PubTopic[128];
    char WillMessage[64];
};

static struct MQTT_Configuration MQTT_CONFIG;
/* define MQTT client context */
static mqtt_client client;
static int is_started = 0;

#define JC_MAILBOX_NAME         "jcMBox"
#define JC_MEMPOOL_NAME         "jcMemP"
#define JC_THREAD_NAME          "jcThrd"

#define JC_MAIL_SIZE            4096

static rt_mailbox_t jc_mbox = RT_NULL;
static rt_mp_t      jc_memp = RT_NULL;
static rt_thread_t  jc_thrd = RT_NULL;

struct sub_string
{
    rt_slist_t  node;
    char * str;
};

static rt_mailbox_t sub_mb = RT_NULL;

static void jc_mqtt_client_entry(void* parameter)
{
    char *pbuf, *pdch;
    char *token, *times;
    cJSON *root, *body, *node;
    rt_size_t total = 0;
    cJSON *re_root, *re_body, *re_node;
    rt_err_t retval = RT_EOK;
    rt_device_t jcdev = RT_NULL;
    struct jcdev_reg_t reg;
    rt_uint8_t regbuf[16];
    
    pbuf = RT_NULL; pdch = RT_NULL;
    root = RT_NULL; body = RT_NULL; node = RT_NULL;
    re_root = RT_NULL; re_body = RT_NULL; re_node = RT_NULL;
    
    jcdev = rt_device_find("rn8302b");
    RT_ASSERT( jcdev != RT_NULL);
    rt_device_open(jcdev, RT_DEVICE_OFLAG_RDWR);
    
    while(1)
    {
        retval = rt_mb_recv(jc_mbox, (rt_ubase_t *)&pbuf, RT_WAITING_FOREVER);
        if (retval != RT_EOK) {
            LOG_W("error in receiving mail!");
            continue;
        }
        
        if (pbuf == RT_NULL) {
            LOG_W("mail context is RT_NULL!");
            continue;
        }
        
        LOG_D("recv a mail, size %d", rt_strlen(pbuf));
        
        root = cJSON_Parse(pbuf);
        rt_mp_free(pbuf);
        
        pbuf = RT_NULL;
        
        if (root == RT_NULL) {
            cJSON_Delete(root);
            LOG_W("json parse failed!");            
            continue;
        }
        
        LOG_D("json parse success!");
        
        token = cJSON_GetObjectItem(root, "token")->valuestring;
        LOG_D("token: %s", token);
        times = cJSON_GetObjectItem(root, "timestamp")->valuestring;
        LOG_D("timestamp: %s", times);
        
        re_body = cJSON_CreateArray();
        body = cJSON_GetObjectItem(root, "body");
        total = cJSON_GetArraySize(body);
        
        for (int i = 0; i < total; i++) {
            node = cJSON_GetArrayItem(body, i);
            re_node = cJSON_CreateObject();
            // name
            pdch = cJSON_GetObjectItem(node, "name")->valuestring;
            LOG_D("name: %s", pdch);
            cJSON_AddItemToObject(re_node, "name", cJSON_CreateString(pdch));
            
            reg.desc = pdch;
            retval = rt_device_control(jcdev, JCDEV_SEARCH_BYNAME, &reg);
            LOG_D("search register by name done!");
            
            if (retval != RT_EOK) {
                
                LOG_W("no %s found", pdch);
                cJSON_AddItemToObject(re_node, "appendix", cJSON_CreateString("failure"));
                cJSON_AddItemToObject(re_node, "type", cJSON_CreateString("NULL"));
                cJSON_AddItemToObject(re_node, "value", cJSON_CreateString("NULL"));
                cJSON_AddItemToArray(re_body, re_node);
                continue;
            }
            // appendix
            pdch = cJSON_GetObjectItem(node, "appendix")->valuestring;
            LOG_D("appendix: %s", pdch);
            
            if (rt_strcmp("write", pdch) == 0) {
                if (reg.writeable == RT_TRUE) {
                    
                    cJSON_AddItemToObject(re_node, "appendix", cJSON_CreateString("ok"));
                    
                    // type
                    pdch = cJSON_GetObjectItem(node, "type")->valuestring;
                    cJSON_AddItemToObject(re_node, "type", cJSON_CreateString(pdch));
                    
                    if (rt_strcmp("hex", pdch) == 0) {
                        rt_uint32_t hex;
                        
                        pdch = cJSON_GetObjectItem(node, "value")->valuestring;
                        cJSON_AddItemToObject(re_node, "value", cJSON_CreateString(pdch));     
                        sscanf(pdch, "0x%08X", &hex);
                        LOG_I("jc write %x, %x, %d", reg.addr, hex, reg.size);

                        do{
                            rt_uint8_t movebit = 32 - reg.size*8;
                            hex = htonl(hex << movebit);
                        }while(0);      
                        rt_device_write(jcdev, reg.addr, &hex, reg.size);
                    }
                    else {
                        LOG_W("jc write is only support hex now!");
                        // value
                        pdch = cJSON_GetObjectItem(node, "value")->valuestring;
                        cJSON_AddItemToObject(re_node, "value", cJSON_CreateString(pdch));     
                    }
                }
                else {
                    cJSON_AddItemToObject(re_node, "appendix", cJSON_CreateString("failure"));
                    cJSON_AddItemToObject(re_node, "type", cJSON_CreateString("NULL"));
                    cJSON_AddItemToObject(re_node, "value", cJSON_CreateString("NULL"));
                }
            }
            else if (rt_strcmp("read", pdch) == 0) {
                
                cJSON_AddItemToObject(re_node, "appendix", cJSON_CreateString("ok"));
                
                rt_memset(regbuf, 0, sizeof(regbuf));
                rt_device_read(jcdev, (rt_off_t)&reg, regbuf, 1);
                
                // type
                pdch = cJSON_GetObjectItem(node, "type")->valuestring;
                cJSON_AddItemToObject(re_node, "type", cJSON_CreateString(pdch));
                
                // value
                if (rt_strcmp("float", pdch) == 0) {
                    char str[128] = {0};
                    
                    float sv = *(float*)regbuf;

                    rt_snprintf(str, sizeof(str), "%d.%03d", (rt_int32_t)floor(sv), (rt_int32_t)((sv - floor(sv)) * 1000));
                    
                    cJSON_AddItemToObject(re_node, "value", cJSON_CreateString(str));
                }
                else if (rt_strcmp("hex", pdch) == 0) {
                    char str[128] = {0};

                    LOG_HEX(reg.desc, 8, regbuf, 16);
                    
                    str[0] = '0';
                    str[1] = 'x';
                    for (int i = 0, offset = 2; i < reg.size; i++)
                        offset += rt_sprintf(str + offset, "%02x", regbuf[i]);

                    cJSON_AddItemToObject(re_node, "value", cJSON_CreateString(str));
                    
                }
                else {
                    cJSON_AddItemToObject(re_node, "value", cJSON_CreateString("syntax"));
                }
            }
            else {
                cJSON_AddItemToObject(re_node, "appendix", cJSON_CreateString("syntax"));
                cJSON_AddItemToObject(re_node, "type", cJSON_CreateString("NULL"));
                cJSON_AddItemToObject(re_node, "value", cJSON_CreateString("NULL"));
            }
            
            cJSON_AddItemToArray(re_body, re_node);
        }
        
        re_root = cJSON_CreateObject();
        cJSON_AddItemToObject(re_root, "token", cJSON_CreateString(token));
        cJSON_AddItemToObject(re_root, "timestamp", cJSON_CreateString(times));
        cJSON_AddItemToObject(re_root, "body", re_body);
        
        pdch = cJSON_PrintUnformatted(re_root);
        if (pdch != RT_NULL) {
            paho_mqtt_publish(&client, QOS0, MQTT_CONFIG.PubTopic, (void *)pdch, rt_strlen(pdch));
            rt_free(pdch);
        }
        else
            LOG_W("cJSON_Print failed!");

        cJSON_Delete(re_root);
        cJSON_Delete(root);
    }
}

static void mqtt_sub_callback(mqtt_client *c, message_data *msg_data)
{
    *((char *)msg_data->message->payload + msg_data->message->payloadlen) = '\0';
    LOG_D("mqtt sub callback: %.*s %.*s",
               msg_data->topic_name->lenstring.len,
               msg_data->topic_name->lenstring.data,
               msg_data->message->payloadlen,
               (char *)msg_data->message->payload);

    if (msg_data->message->payloadlen != 0 && msg_data->message->payloadlen < JC_MAIL_SIZE) {
        rt_uint8_t *pbuf = rt_mp_alloc(jc_memp, 0);
        if (pbuf == RT_NULL) {
            LOG_W("alloc mempool failed!");
            return;
        }
        
        rt_memset(pbuf, 0, JC_MAIL_SIZE);
        rt_memcpy(pbuf, msg_data->message->payload, msg_data->message->payloadlen);
        
        rt_err_t retval = rt_mb_send(jc_mbox, (rt_ubase_t)pbuf);
        if (retval != RT_EOK) {
            LOG_W("send mail failed, maybe mailbox is full!");
            rt_mp_free(pbuf);
            return;
        }
    }
    else {
        LOG_W("payload size wrong!");
    }
    
    return ;
}

static void mqtt_sub_default_callback(mqtt_client *c, message_data *msg_data)
{
    *((char *)msg_data->message->payload + msg_data->message->payloadlen) = '\0';
    LOG_D("mqtt sub default callback: %.*s %.*s",
               msg_data->topic_name->lenstring.len,
               msg_data->topic_name->lenstring.data,
               msg_data->message->payloadlen,
               (char *)msg_data->message->payload);
}

static void mqtt_connect_callback(mqtt_client *c)
{
    LOG_I("inter mqtt_connect_callback!");
}

static void mqtt_online_callback(mqtt_client *c)
{
    rt_pin_write(WARN_PIN, PIN_HIGH);
    
    LOG_I("inter mqtt_online_callback!");
}

static void mqtt_offline_callback(mqtt_client *c)
{
    rt_pin_write(WARN_PIN, PIN_LOW);
    
    LOG_W("inter mqtt_offline_callback!");
}

static int mqtt_start(int argc, char **argv)
{
    rt_thread_delay(RT_TICK_PER_SECOND * 5);
    
    /* init condata param by using MQTTPacket_connectData_initializer */
    MQTTPacket_connectData condata = MQTTPacket_connectData_initializer;
    static char cid[20] = { 0 };

    if (argc != 1)
    {
        rt_kprintf("mqtt_start    --start a mqtt worker thread.\n");
        return -1;
    }

    if (is_started)
    {
        LOG_E("mqtt client is already connected.");
        return -1;
    }
    
    sub_mb = rt_mb_create("mb_rn83", 8, RT_IPC_FLAG_FIFO);
    RT_ASSERT(sub_mb != RT_NULL);
    
    /* config MQTT context param */
    {
        client.isconnected = 0;
        client.uri = MQTT_CONFIG.URL;

        /* generate the random client ID */
        rt_snprintf(cid, sizeof(cid), "rtthread%d", rt_tick_get());
        /* config connect param */
        memcpy(&client.condata, &condata, sizeof(condata));
        client.condata.clientID.cstring = cid;
        client.condata.keepAliveInterval = 30;
        client.condata.cleansession = 1;
        client.condata.username.cstring = MQTT_CONFIG.Username;
        client.condata.password.cstring = MQTT_CONFIG.Password;

        /* config MQTT will param. */
        client.condata.willFlag = 1;
        client.condata.will.qos = 1;
        client.condata.will.retained = 0;
        client.condata.will.topicName.cstring = MQTT_CONFIG.PubTopic;
        client.condata.will.message.cstring = MQTT_CONFIG.WillMessage;

        /* malloc buffer. */
        client.buf_size = client.readbuf_size = 4096;
        client.buf = rt_calloc(1, client.buf_size);
        client.readbuf = rt_calloc(1, client.readbuf_size);
        if (!(client.buf && client.readbuf))
        {
            LOG_E("no memory for MQTT client buffer!");
            return -1;
        }

        /* set event callback function */
        client.connect_callback = mqtt_connect_callback;
        client.online_callback = mqtt_online_callback;
        client.offline_callback = mqtt_offline_callback;

        /* set subscribe table and event callback */
        client.message_handlers[0].topicFilter = rt_strdup(MQTT_CONFIG.SubTopic);
        client.message_handlers[0].callback = mqtt_sub_callback;
        client.message_handlers[0].qos = QOS1;

        /* set default subscribe event callback */
        client.default_message_handlers = mqtt_sub_default_callback;
    }
    
    {
        rt_uint32_t value;
        rt_uint16_t u16Value;

        value = 5;
        paho_mqtt_control(&client, MQTT_CTRL_SET_CONN_TIMEO, &value);
        
        value = 5;
        paho_mqtt_control(&client, MQTT_CTRL_SET_MSG_TIMEO, &value);
        
        value = 5;
        paho_mqtt_control(&client, MQTT_CTRL_SET_RECONN_INTERVAL, &value);
        
        value = 30;
        paho_mqtt_control(&client, MQTT_CTRL_SET_KEEPALIVE_INTERVAL, &value);
        
        u16Value = 3;
        paho_mqtt_control(&client, MQTT_CTRL_SET_KEEPALIVE_COUNT, &u16Value);
    }
    
    /* run mqtt client */
    paho_mqtt_start(&client, 8 * 1024, 14);
    is_started = 1;
    
    /* start jc service */
    {
        jc_memp = rt_mp_create(JC_MEMPOOL_NAME, 10, JC_MAIL_SIZE);
        RT_ASSERT(jc_memp != RT_NULL);
        
        jc_mbox = rt_mb_create(JC_MAILBOX_NAME, 10, RT_IPC_FLAG_FIFO);
        RT_ASSERT(jc_mbox != RT_NULL);
        
        jc_thrd = rt_thread_create(JC_THREAD_NAME, jc_mqtt_client_entry, 0, 4 * 1024, 13, 10);
        RT_ASSERT(jc_thrd != RT_NULL);
        rt_thread_startup(jc_thrd);
        
        LOG_I("jc mqtt client online");
    }

    return 0;
}

rt_err_t load_mqtt_config(void)
{
    const char* FILENAME = "mqtt.json"; 
    const rt_size_t JSON_BUFFER_SIZE = 1024;
    
    rt_size_t size = 0;

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
        cJSON_AddItemToObject(root, "URL", cJSON_CreateString(MQTT_URI));
        cJSON_AddItemToObject(root, "Username", cJSON_CreateString(MQTT_USERNAME));
        cJSON_AddItemToObject(root, "Password", cJSON_CreateString(MQTT_PASSWORD));
        cJSON_AddItemToObject(root, "SubTopic", cJSON_CreateString(MQTT_SUBTOPIC));
        cJSON_AddItemToObject(root, "PubTopic", cJSON_CreateString(MQTT_PUBTOPIC));
        cJSON_AddItemToObject(root, "WillMessage", cJSON_CreateString(MQTT_WILLMSG));

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
        
        root = cJSON_Parse(json);
        rt_free(json);
        close(fd);
        
        rt_thread_delay(RT_TICK_PER_SECOND / 2);
    }
    
    char * strp = RT_NULL;
    rt_memset(&MQTT_CONFIG, 0, sizeof(struct MQTT_Configuration));
    
    strp = cJSON_GetObjectItem(root, "URL")->valuestring;
    rt_memcpy(MQTT_CONFIG.URL, strp, rt_strlen(strp));
    strp = cJSON_GetObjectItem(root, "Username")->valuestring;
    rt_memcpy(MQTT_CONFIG.Username, strp, rt_strlen(strp));
    strp = cJSON_GetObjectItem(root, "Password")->valuestring;
    rt_memcpy(MQTT_CONFIG.Password, strp, rt_strlen(strp));
    strp = cJSON_GetObjectItem(root, "SubTopic")->valuestring;
    rt_memcpy(MQTT_CONFIG.SubTopic, strp, rt_strlen(strp));
    strp = cJSON_GetObjectItem(root, "PubTopic")->valuestring;
    rt_memcpy(MQTT_CONFIG.PubTopic, strp, rt_strlen(strp));
    strp = cJSON_GetObjectItem(root, "WillMessage")->valuestring;
    rt_memcpy(MQTT_CONFIG.WillMessage, strp, rt_strlen(strp));
     
    LOG_I("load mqtt configurations from file...");
    LOG_I("URL %s", MQTT_CONFIG.URL);
    LOG_I("PubTopic %s", MQTT_CONFIG.PubTopic);
    LOG_I("SubTopic %s", MQTT_CONFIG.SubTopic);
    
    cJSON_Delete(root);
    
    return RT_EOK;
}

int mqtt_server_init(void)
{
    if (load_mqtt_config() == RT_EOK) {
        mqtt_start(1, RT_NULL);
    }

    return 0;
}

static int mqtt_stop(int argc, char **argv)
{
    if (argc != 1)
    {
        rt_kprintf("mqtt_stop    --stop mqtt worker thread and free mqtt client object.\n");
    }

    is_started = 0;

    return paho_mqtt_stop(&client);
}

static int mqtt_pub(int argc, char **argv)
{
    char *str = "0123456789012345678901234567890";
    
    paho_mqtt_publish(&client, QOS0, MQTT_PUBTOPIC, (void *)str, rt_strlen(str));

    return 0;
}

#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(mqtt_start, startup mqtt client);
MSH_CMD_EXPORT(mqtt_stop, stop mqtt client);
MSH_CMD_EXPORT(mqtt_pub, publish mqtt message);
#endif /* FINSH_USING_MSH */
