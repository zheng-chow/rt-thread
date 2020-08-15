#ifndef __UDP_API_H__
#define __UDP_API_H__
#include <udp_ethernet.h>

typedef void* udp_api_handle_t;

udp_api_handle_t udp_api_create(uint16_t localport, const char* remoteip, uint16_t remoteport, const char* name);
void  udp_api_delete(udp_api_handle_t* phandler);
void  udp_api_send(udp_api_handle_t phandler, uint8_t* data, uint32_t datalen, udp_timestamp_t timeMask);
void  udp_api_recv(udp_api_handle_t phandler, uint8_t* data, uint32_t datalen, udp_timestamp_t* timeMask);

#endif

