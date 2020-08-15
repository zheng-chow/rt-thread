#ifndef __UDP_API_H__
#define __UDP_API_H__
#include <udp_ethernet.h>

typedef void* udp_api_handle_t;

udp_api_handle_t udp_api_create(uint16_t localport, const char* remoteip, uint16_t remoteport, const char* name);
void  udp_api_delete(udp_api_handle_t* phandler);

int  udp_api_add_client(udp_api_handle_t* phandler, uint16_t localport, const char* remoteip, uint16_t remoteport);
rt_bool_t  udp_api_del_client(udp_api_handle_t* phandler, int idx);

uint32_t udp_api_get_client_count(udp_api_handle_t phandler);
rt_bool_t udp_api_check_ip(udp_api_handle_t phandler, int idx, const char* remoteip);
rt_bool_t udp_api_is_in_use(udp_api_handle_t phandler, int idx);
int udp_api_find_dspip(udp_api_handle_t phandler, const char* remoteip, uint16_t port);

void  udp_api_send(udp_api_handle_t phandler, uint8_t* data, uint32_t datalen, udp_timestamp_t timeMask);
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==1
void  udp_api_recv(udp_api_handle_t phandler, uint8_t* data, uint32_t datalen, udp_timestamp_t* timeMask);
#endif

#endif

