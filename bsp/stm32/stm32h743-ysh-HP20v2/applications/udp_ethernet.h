#ifndef __UDP_ETHERNET_H__
#define __UDP_ETHERNET_H__
#include <platform_compatible.h>

#define UDP_MAX_CLINET      2

#define UDP_ETH_VERSION			1	/* Protocol version */
#define UDP_ETH_MAXPKTSIZE 	12000	/* Number of data bytes that can sent in a packet, RUDP header not included */
#define UPD_ETH_MAKPKTCOUNT	6

#define UDP_ETH_DATA	1
#define UDP_ETH_ACK	2
#define UDP_ETH_SYN	4
#define UDP_ETH_FIN	5

#define UDP_USING_ACK 0//1

#define ADC_FREQ 200000

typedef struct {
    uint32_t freq;
    uint16_t cnls;    
    uint16_t bytes;    
    uint32_t stamp;
    //uint32_t precision;
}udp_timestamp_t;

typedef void*  udp_eth_api_handler_t;

int udp_eth_find_remote_addr(udp_eth_api_handler_t phandler, const char* remoteip, uint16_t remoteport);

udp_eth_api_handler_t udp_eth_create(uint16_t localport, const char* remoteip, uint16_t remoteport, const char* name);
void udp_eth_delete(udp_eth_api_handler_t* phandler);

int udp_eth_add_socket(udp_eth_api_handler_t* phandler, uint16_t localport, const char* remoteip, uint16_t remoteport);
rt_bool_t udp_eth_del_socket(udp_eth_api_handler_t* phandler, int idx);

void udp_eth_stop(udp_eth_api_handler_t* phandler);
int udp_eth_send(udp_eth_api_handler_t phandler, uint8_t* payload, uint32_t payloadLength, udp_timestamp_t timeMask);
#if !defined(UDP_USING_ACK) || UDP_USING_ACK==1	
uint32_t udp_eth_isrecv(udp_eth_api_handler_t phandler);
int udp_eth_recv(udp_eth_api_handler_t phandler, uint8_t* payload, uint32_t payloadLength, udp_timestamp_t* timeMask);
void udp_r_loop(udp_eth_api_handler_t phandler);
#endif
void udp_s_loop(udp_eth_api_handler_t phandler);
#endif
