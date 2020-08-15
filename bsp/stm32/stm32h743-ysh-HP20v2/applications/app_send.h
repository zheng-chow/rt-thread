#ifndef __APP_SEND_H__
#define __APP_SEND_H__
#include <rtdef.h>
int app_create_send(const char* dstIp, rt_uint16_t dstPort[2]);
rt_bool_t app_delete_send(const char* dstIp, rt_uint16_t srcPort[2]);
void read_udp_port(void);
void write_default_control_ini_file(void);

rt_uint16_t get_local_port(int udp, int idx);
rt_bool_t set_remote_port(int udp, int idx, rt_uint16_t port);


int find_index(const char* dstIp, rt_uint16_t srcPort[2]);




#endif

