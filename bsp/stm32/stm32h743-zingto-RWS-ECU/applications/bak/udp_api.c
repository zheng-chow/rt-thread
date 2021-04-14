#include <udp_api.h>

typedef struct{
	udp_eth_api_handler_t eth_handle;
	udp_thread_handle_t   eth_send_handle;
	udp_thread_handle_t		eth_recv_handle;
}udp_api_handler_t;

void udp_recv_loop(void* parameter){
	udp_eth_api_handler_t phandler = (udp_eth_api_handler_t)parameter;
	udp_r_loop(phandler);
}
void udp_send_loop(void* parameter){
	udp_eth_api_handler_t phandler = (udp_eth_api_handler_t)parameter;
	udp_s_loop(phandler);
}


udp_api_handle_t udp_api_create(uint16_t localport, const char* remoteip, uint16_t remoteport, const char* name){
	udp_api_handler_t* handler = (udp_api_handler_t*)udp_malloc(sizeof(udp_api_handler_t));
	char tmp[64];
	int tmplen = 0;
	assert(handler);
	udp_memset(handler, 0, sizeof(udp_api_handler_t));
	handler->eth_handle = udp_eth_create(localport, remoteip, remoteport, name);
	assert(handler->eth_handle);
	tmplen = udp_snprintf(tmp, 64, "%s", name);
	udp_snprintf(tmp+tmplen,16 - tmplen, "_r");
	handler->eth_recv_handle = udp_thread_create_r(tmp, udp_recv_loop, handler->eth_handle);
	udp_snprintf(tmp+tmplen,16 - tmplen, "_s");
	handler->eth_send_handle = udp_thread_create_t(tmp, udp_send_loop, handler->eth_handle);
	assert(handler->eth_recv_handle && handler->eth_send_handle);
	udp_thread_start(handler->eth_send_handle);
	udp_thread_start(handler->eth_recv_handle);	
	return (udp_api_handle_t)handler;
}
void  udp_api_delete(udp_api_handle_t* phandler){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler[0];
	assert(phandler && phandler[0]);
    udp_eth_stop(&handler->eth_handle);
	udp_thread_stop(handler->eth_recv_handle);
	udp_thread_stop(handler->eth_send_handle);
	udp_thread_delete(handler->eth_recv_handle);
	udp_thread_delete(handler->eth_send_handle);
    rt_thread_delay(RT_TICK_PER_SECOND/10);
	udp_eth_delete(&handler->eth_handle);	
	
	udp_free(handler);
	phandler[0] = NULL;
}
void  udp_api_send(udp_api_handle_t phandler, uint8_t* data, uint32_t datalen, udp_timestamp_t timeMask){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler;
	udp_eth_send(handler->eth_handle, data, datalen, timeMask);
}
void  udp_api_recv(udp_api_handle_t phandler, uint8_t* data, uint32_t datalen, udp_timestamp_t* timeMask){
	udp_api_handler_t* handler = (udp_api_handler_t*)phandler;
	udp_eth_recv(handler->eth_handle, data, datalen, timeMask);
}
