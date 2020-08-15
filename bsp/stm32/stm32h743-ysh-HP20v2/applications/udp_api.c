#include <udp_api.h>
#include <lwip/inet.h>
typedef struct{
	udp_eth_api_handler_t   eth_handle;
	udp_thread_handle_t     eth_send_handle;
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==1
	udp_thread_handle_t	    eth_recv_handle;
#endif
    int                     client_cnt;
    uint32_t                dstIp[UDP_MAX_CLINET];
}udp_api_handler_t;

#if !defined(UDP_USING_ACK) || UDP_USING_ACK==1
void udp_recv_loop(void* parameter){
	udp_eth_api_handler_t phandler = (udp_eth_api_handler_t)parameter;
	udp_r_loop(phandler);
}
#endif
void udp_send_loop(void* parameter){
	udp_eth_api_handler_t phandler = (udp_eth_api_handler_t)parameter;
	udp_s_loop(phandler);
}
rt_bool_t udp_api_is_in_use(udp_api_handle_t phandler, int idx){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler;
    return handler->dstIp[idx] != 0;
}
int udp_api_find_dspip(udp_api_handle_t phandler, const char* remoteip, uint16_t port){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler;
    return udp_eth_find_remote_addr(handler->eth_handle, remoteip, port);
}


udp_api_handle_t udp_api_create(uint16_t localport, const char* remoteip, uint16_t remoteport, const char* name){
	udp_api_handler_t* handler = (udp_api_handler_t*)udp_malloc(sizeof(udp_api_handler_t));
	char tmp[64];
	int tmplen = 0;
	assert(handler);
	udp_memset(handler, 0, sizeof(udp_api_handler_t));
	handler->eth_handle = udp_eth_create(localport, remoteip, remoteport, name);
	assert(handler->eth_handle);
    handler->client_cnt = 1;
	tmplen = udp_snprintf(tmp, 64, "%s", name);
 #if !defined(UDP_USING_ACK) || UDP_USING_ACK==1
	udp_snprintf(tmp+tmplen,16 - tmplen, "_r");
	handler->eth_recv_handle = udp_thread_create_r(tmp, udp_recv_loop, handler->eth_handle);
#endif
	udp_snprintf(tmp+tmplen,16 - tmplen, "_s");
	handler->eth_send_handle = udp_thread_create_t(tmp, udp_send_loop, handler->eth_handle);
	assert(
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==1
    handler->eth_recv_handle &&
#endif
    handler->eth_send_handle);
	udp_thread_start(handler->eth_send_handle);
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==1
	udp_thread_start(handler->eth_recv_handle);	
#endif
    handler->dstIp[0] = inet_addr(remoteip);
	return (udp_api_handle_t)handler;
}
void  udp_api_delete(udp_api_handle_t* phandler){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler[0];
	assert(phandler && phandler[0]);
    udp_eth_stop(&handler->eth_handle);
	udp_thread_stop(handler->eth_recv_handle);
	udp_thread_stop(handler->eth_send_handle);
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==1
	udp_thread_delete(handler->eth_recv_handle);
#endif
	udp_thread_delete(handler->eth_send_handle);
    rt_thread_delay(RT_TICK_PER_SECOND/10);
    if (handler->eth_handle)
        udp_eth_delete(&handler->eth_handle);	
	
	udp_free(handler);
	phandler[0] = NULL;
}
int  udp_api_add_client(udp_api_handle_t* phandler, uint16_t localport, const char* remoteip, uint16_t remoteport){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler[0];
	assert(phandler && phandler[0]);
    int idx = udp_eth_add_socket(&handler->eth_handle, localport, remoteip, remoteport);
    if (idx < UDP_MAX_CLINET){
        handler->client_cnt++;
        handler->dstIp[idx] = inet_addr(remoteip);
    }
    return idx;    
}
rt_bool_t  udp_api_del_client(udp_api_handle_t* phandler, int idx){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler[0];
	assert(phandler && phandler[0]);
    if (udp_eth_del_socket(&handler->eth_handle, idx)){
        handler->client_cnt--;
        handler->dstIp[idx] = 0;
        if (handler->client_cnt == 0)
            udp_api_delete(phandler);
        return RT_TRUE;
    }
    return  RT_FALSE;
}

uint32_t udp_api_get_client_count(udp_api_handle_t phandler){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler;
    return handler->client_cnt;
}
rt_bool_t udp_api_check_ip(udp_api_handle_t phandler, int idx, const char* remoteip){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler;
    if ((idx >= UDP_MAX_CLINET) || (idx < 0)) return RT_FALSE;
    rt_kprintf("dstIp[%d]: %08X\t checkIp:%08X\n", idx, handler->dstIp[idx], inet_addr(remoteip));
    return handler->dstIp[idx] == inet_addr(remoteip);    
}


void  udp_api_send(udp_api_handle_t phandler, uint8_t* data, uint32_t datalen, udp_timestamp_t timeMask){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler;
	udp_eth_send(handler->eth_handle, data, datalen, timeMask);
}


#if !defined(UDP_USING_ACK) || UDP_USING_ACK==1
void  udp_api_recv(udp_api_handle_t phandler, uint8_t* data, uint32_t datalen, udp_timestamp_t* timeMask){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler;
	udp_eth_recv(handler->eth_handle, data, datalen, timeMask);
}
#endif
