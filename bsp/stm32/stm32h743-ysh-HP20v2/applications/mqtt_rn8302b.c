#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <rtthread.h>

#define DBG_ENABLE
#define DBG_SECTION_NAME    "mqtt.rn8302b"
#define DBG_LEVEL           DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

#include "paho_mqtt.h"

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
#define MQTT_URI                "tcp://192.168.5.224:1883"
#define MQTT_USERNAME           "admin"
#define MQTT_PASSWORD           "admin"
#define MQTT_SUBTOPIC           "/meter/rn8302b/request"
#define MQTT_PUBTOPIC           "/meter/rn8302b/response"
#define MQTT_WILLMSG            "offline,rn8302b"

#define MQTT_STRING_SIZE         32

/* define MQTT client context */
static MQTTClient client;
static int is_started = 0;

struct sub_string
{
    rt_slist_t  node;
    char * str;
};

static rt_mailbox_t sub_mb = RT_NULL;

static void mqtt_sub_callback(MQTTClient *c, MessageData *msg_data)
{
    *((char *)msg_data->message->payload + msg_data->message->payloadlen) = '\0';
    
    LOG_D("mqtt sub callback: %.*s %.*s",
               msg_data->topicName->lenstring.len,
               msg_data->topicName->lenstring.data,
               msg_data->message->payloadlen,
               (char *)msg_data->message->payload);
    
    if (msg_data->message->payloadlen == 0) {
        LOG_D("payload size = 0");
        return;
    }
    
    if (rt_strncmp(MQTT_SUBTOPIC, msg_data->topicName->lenstring.data, msg_data->topicName->lenstring.len) != 0) {
        LOG_D("topicname mismatch with %s", MQTT_SUBTOPIC);
        return;
    }
    
    LOG_D("start process message");
    
    char *strbuf = rt_malloc(MQTT_STRING_SIZE);
    rt_memset(strbuf, 0, MQTT_STRING_SIZE);
    rt_size_t sbufsz;
    char *ptr = (char *)msg_data->message->payload;
    
    rt_bool_t isdone, iscmd, isget;
    
    isdone = RT_FALSE;
    iscmd = RT_TRUE;
    
    while(isdone != RT_TRUE) {
        
        char * newptr = strchr(ptr, ',');
        
        if (newptr != RT_NULL) {
            sbufsz = newptr - ptr;
            rt_strncpy(strbuf, ptr, sbufsz);
            
            ptr = newptr + 1;
        }
        else {
            sbufsz = rt_strlen(ptr); 
            rt_strncpy(strbuf, ptr, sbufsz);
                
            isdone = RT_TRUE;
        }

        if (iscmd == RT_TRUE) {
            if( rt_strncmp("SET", strbuf, sbufsz) == 0)
                isget = RT_FALSE;
            else
                isget = RT_TRUE;
            
            LOG_D("%s", strbuf);
            
            iscmd = RT_FALSE;
        }
        else {
            if (isget == RT_TRUE) {
                rt_strncpy(strbuf + sbufsz, ", GET response working", rt_strlen(", GET response working"));
                LOG_D("%d, %s", rt_tick_get(), strbuf);
                paho_mqtt_publish(&client, QOS1, MQTT_PUBTOPIC, strbuf);
                LOG_D("%d, publish finsh", rt_tick_get());
            }  
            else {
                rt_strncpy(strbuf + sbufsz, ", SET response working", rt_strlen(", SET response working"));
                LOG_D("%d, %s", rt_tick_get(), strbuf);
                paho_mqtt_publish(&client, QOS1, MQTT_PUBTOPIC, strbuf);
                LOG_D("%d, publish finsh", rt_tick_get());
            }
            
            rt_thread_delay(10);
        }
        
        rt_memset(strbuf, 0, MQTT_STRING_SIZE);
    }
    
    rt_free(strbuf);
    
    return ;
}

static void mqtt_sub_default_callback(MQTTClient *c, MessageData *msg_data)
{
    *((char *)msg_data->message->payload + msg_data->message->payloadlen) = '\0';
    LOG_D("mqtt sub default callback: %.*s %.*s",
               msg_data->topicName->lenstring.len,
               msg_data->topicName->lenstring.data,
               msg_data->message->payloadlen,
               (char *)msg_data->message->payload);
}

static void mqtt_connect_callback(MQTTClient *c)
{
    LOG_D("inter mqtt_connect_callback!");
}

static void mqtt_online_callback(MQTTClient *c)
{
    LOG_D("inter mqtt_online_callback!");
}

static void mqtt_offline_callback(MQTTClient *c)
{
    LOG_D("inter mqtt_offline_callback!");
}

static int mqtt_start(int argc, char **argv)
{
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
        client.uri = MQTT_URI;

        /* generate the random client ID */
        rt_snprintf(cid, sizeof(cid), "rtthread%d", rt_tick_get());
        /* config connect param */
        memcpy(&client.condata, &condata, sizeof(condata));
        client.condata.clientID.cstring = cid;
        client.condata.keepAliveInterval = 30;
        client.condata.cleansession = 1;
        client.condata.username.cstring = MQTT_USERNAME;
        client.condata.password.cstring = MQTT_PASSWORD;

        /* config MQTT will param. */
        client.condata.willFlag = 1;
        client.condata.will.qos = 1;
        client.condata.will.retained = 0;
        client.condata.will.topicName.cstring = MQTT_PUBTOPIC;
        client.condata.will.message.cstring = MQTT_WILLMSG;

        /* malloc buffer. */
        client.buf_size = client.readbuf_size = 1024;
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
        client.messageHandlers[0].topicFilter = rt_strdup(MQTT_SUBTOPIC);
        client.messageHandlers[0].callback = mqtt_sub_callback;
        client.messageHandlers[0].qos = QOS1;

        /* set default subscribe event callback */
        client.defaultMessageHandler = mqtt_sub_default_callback;
    }

    /* run mqtt client */
    paho_mqtt_start(&client);
    is_started = 1;

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

#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(mqtt_start, startup mqtt client);
MSH_CMD_EXPORT(mqtt_stop, stop mqtt client);
#endif /* FINSH_USING_MSH */
