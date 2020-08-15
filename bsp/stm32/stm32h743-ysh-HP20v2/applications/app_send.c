#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "app_send.h"
#include "app_adc.h"



#if defined(RT_USING_RUDP)
#include <udp_api.h>
udp_api_handle_t udp_api_hd[2] = {RT_NULL};
#endif
static void   adc_callback(adc_callback_parameter_t h, rt_uint8_t* adc_data, rt_uint32_t adc_data_len, rt_uint32_t index, adc_timestamp_t stamp){
#if defined(RT_USING_RUDP)
    udp_timestamp_t timeMask;
    timeMask.freq = stamp.freq;
    timeMask.cnls = stamp.cnls;   
    timeMask.bytes = stamp.bytes;   
    timeMask.stamp = stamp.stamp;   
    udp_api_send(udp_api_hd[index], adc_data, adc_data_len,timeMask);    
   /*
    //static uint8_t tmp[2]={0,0};
    rt_uint32_t groups = adc_data_len/6;
    rt_uint16_t* data16 = (rt_uint16_t*)adc_data;

    //float fAngle = ANGLE_COEFF * stamp.stamp;
    udp_timestamp_t timeMask;
    timeMask.freq = stamp.freq;
    timeMask.stamp = stamp.stamp;
    float t = stamp.stamp / 1000.f;
    float N = SIG_FREQ  * t;
    N -= (int)N;
    float fAngle = SIN_POINTS * N;
    float fDetAngle = 1.f* SIN_POINTS * SIG_FREQ  / stamp.freq;
    //fAngle -= (int)fAngle;
    if (index == 0){
        //tmp[0]++;        tmp[0] &= 0x0F;
        //rt_memset(adc_data, tmp[0], adc_data_len);
        for (rt_uint32_t n = 0; n < groups; n++, data16+=3){
            float fa = fAngle;
            rt_uint32_t ua = ((rt_uint32_t)(fa))%SIN_POINTS;
            data16[0] = sinbuffer[ua];
            fa += A_120;
            ua = ((rt_uint32_t)(fa))%SIN_POINTS;      
            data16[1] = sinbuffer[ua];
            fa += A_120;
            ua = ((rt_uint32_t)(fa))%SIN_POINTS;   
            data16[2] = sinbuffer[ua];
            fAngle += fDetAngle;
        }
        udp_api_send(udp_api_hd[0], adc_data, adc_data_len,timeMask);
        / *{
          static rt_tick_t t1 = 0, t2 = 0, tm = 0;
          rt_uint32_t cc = 0;
          if (t1 == 0) t1  = rt_tick_get();
          t2 = rt_tick_get() - t1;
          t1 = t1 + t2;
          tm = (tm < t2)?(cc=adc_data_len,t2):tm;
        } * /  
    }
    else{
        //tmp[1]--;        tmp[1] &= 0x0F;
        //rt_memset(adc_data, tmp[1], adc_data_len);
        fAngle += A_60;
        for (rt_uint32_t n = 0; n < groups; n++, data16+=3){
            float fa = fAngle;
            rt_uint32_t ua = ((rt_uint32_t)(fa))%SIN_POINTS;
            data16[0] = sinbuffer[ua];
            fa += A_120;
            ua = ((rt_uint32_t)(fa))%SIN_POINTS;      
            data16[1] = sinbuffer[ua];
            fa += A_120;
            ua = ((rt_uint32_t)(fa))%SIN_POINTS;   
            data16[2] = sinbuffer[ua];
            fAngle += fDetAngle;
        }     
        udp_api_send(udp_api_hd[1], adc_data, adc_data_len,timeMask);    
    }     
   */
#endif
}
rt_uint16_t ch_local_port[2][UDP_MAX_CLINET] = {{60000, 60004}
                                                , {60001, 60005}};
/*rt_uint16_t ch_remote_port[2][UDP_MAX_CLINET] = {{60001, 60005}
                                                , {60002, 60006}};*/
/*
rt_uint16_t ch1_local_port[UDP_MAX_CLINET]  = {60000, 60004};
rt_uint16_t ch1_remote_port[UDP_MAX_CLINET] = {60002, 60006};
rt_uint16_t ch2_local_port[UDP_MAX_CLINET]  = {60001, 60005};
rt_uint16_t ch2_remote_port[UDP_MAX_CLINET] = {60003, 60007};
*/
rt_uint16_t get_local_port(int udp, int idx){
    if ((idx < UDP_MAX_CLINET) && (idx >= 0)
     && ((0 == udp) || (1 == udp))   
    )
        return ch_local_port[udp][idx];
    return 0;
}
/*
rt_bool_t set_remote_port(int udp, int idx, rt_uint16_t port){
    if ((idx < UDP_MAX_CLINET) && (idx >= 0)
     && ((0 == udp) || (1 == udp))   
    ){
        ch_remote_port[udp][idx] = port;
        return RT_TRUE;
    }
    return RT_FALSE;;
}*/
int find_index(const char* dstIp, rt_uint16_t srcPort[2]){
    int idx = -1;
    for (int n = 0; n < UDP_MAX_CLINET; n++){
        if ((ch_local_port[0][n] == srcPort[0])
        && (ch_local_port[1][n] == srcPort[1])){
            idx = n;
            break;
        }
    }
    if (idx >= 0){
        if (udp_api_check_ip(udp_api_hd[0], idx, dstIp) && udp_api_check_ip(udp_api_hd[1], idx, dstIp)){
            return idx;
        }
    }
    return -1;
}

#if defined(RT_USING_INI_HELPER)
#include "util_ini.h"
#endif
#ifdef RT_USING_INI_HELPER
void write_default_control_ini_file(void){
    const char* filename = "control.ini";
    char buffer[20];
    char tport[7];
    int err, line;
    struct ini_file*         inifile = RT_NULL;
    inifile = ini_read(filename,&err,&line);
    if (!inifile)
        inifile = ini_read(RT_NULL,&err,&line);   
    if (inifile){
        char* value = RT_NULL;
        for (int n = 0; n < UDP_MAX_CLINET; n++){
            rt_snprintf(buffer, 20, "LocalPort%d", n);
            value = (char*)ini_get(inifile,"UDP1", buffer, RT_NULL);
            if (!value){
                rt_snprintf(tport, 7, "%d", ch_local_port[0][n]);
                ini_put(inifile,"UDP1", buffer,tport);
            }
            value = (char*)ini_get(inifile,"UDP2", buffer, RT_NULL);
            if (!value){
                rt_snprintf(tport, 7, "%d", ch_local_port[1][n]);
                ini_put(inifile,"UDP2", buffer,tport);
            }        
            /*
            rt_snprintf(buffer, 20, "RemotePort%d", n);
            value = (char*)ini_get(inifile,"UDP1", buffer, RT_NULL);
            if (!value){
                rt_snprintf(tport, 7, "%d", ch_remote_port[0][n]);
                ini_put(inifile,"UDP1", buffer,tport);
            }    
            value = (char*)ini_get(inifile,"UDP2", buffer, RT_NULL);
            if (!value){
                rt_snprintf(tport, 7, "%d", ch_remote_port[1][n]);
                ini_put(inifile,"UDP2", buffer,tport);
            } */           
        } 
        ini_write(inifile, filename);	
        rt_thread_delay(RT_TICK_PER_SECOND/100);       
        ini_free(inifile);	  
    }
}
void read_udp_port(void){
    const char* filename = "control.ini";
    char buffer[20];
    struct ini_file* inifile = RT_NULL;
    int err, line;    
    inifile = ini_read(filename,&err,&line);
    if (inifile){ 
        char* value = RT_NULL;
        for (int n = 0; n < UDP_MAX_CLINET; n++){
            rt_snprintf(buffer, 20, "LocalPort%d", n);
            value = (char*)ini_get(inifile,"UDP1", buffer, RT_NULL);
            if (value)
                ch_local_port[0][n] = atoi(value);            
            value = (char*)ini_get(inifile,"UDP2", buffer, RT_NULL);
            if (value)
                ch_local_port[1][n] = atoi(value);
     /*
            
            value = (char*)ini_get(inifile,"UDP1", buffer, RT_NULL);
            if (value)
                ch_remote_port[0][n] = atoi(value);    
            value = (char*)ini_get(inifile,"UDP2", buffer, RT_NULL);
            if (value)
                ch2_remote_port[n] = atoi(value);*/
                      
        } 
        ini_free(inifile);
    }  
    return ;
}
#endif

int app_check_send(const char* dstIp, rt_uint16_t dstPort[2]){
    if ((!udp_api_hd[0] && udp_api_hd[1]) || (udp_api_hd[0] && !udp_api_hd[1])) return -1;
    int idx0 = udp_api_find_dspip(udp_api_hd[0], dstIp, dstPort[0]);
    int idx1 = udp_api_find_dspip(udp_api_hd[1], dstIp, dstPort[1]);
    if (idx0 != idx1) return -1;
    return idx0;
}
int app_create_send(const char* dstIp, rt_uint16_t dstPort[2]){
#if defined(RT_USING_RUDP)
    if ((!udp_api_hd[0] && udp_api_hd[1]) || (udp_api_hd[0] && !udp_api_hd[1])) return -1;
    int idx = -1;
    if (!udp_api_hd[0]){        
        udp_api_hd[0] = udp_api_create(ch_local_port[0][0], dstIp, dstPort[0], "udp0");
        udp_api_hd[1] = udp_api_create(ch_local_port[1][0], dstIp, dstPort[1], "udp1");
        idx = 0;
   }
   else{
       //查找
       idx = app_check_send(dstIp, dstPort);
       if (idx != -1) {
           rt_kprintf("connect '%s: %u & %u' connected\n", dstIp, dstPort[0], dstPort[1]);
           return idx;     
       }  
       else {
           rt_kprintf("another connect '%s'\n", dstIp);
           for (int n = 0; n < UDP_MAX_CLINET; n++){
               rt_bool_t valid1 = udp_api_is_in_use(udp_api_hd[0], n);
               rt_bool_t valid2 = udp_api_is_in_use(udp_api_hd[1], n);
               if ((!valid1 && valid1) || (valid1 && !valid1)) {
                   rt_kprintf("find a channel unbalance\n");
                   return -1;
               }
               
               if (!valid1 && !valid2){
                   idx = n; 
                   break;
               }
           }
           if (idx == -1) return -1; 
           rt_kprintf("check[%d]: %s %u %u\n", idx,dstIp,dstPort[0],dstPort[1]);
           int idx1 = udp_api_add_client(&udp_api_hd[0], ch_local_port[0][idx], dstIp, dstPort[0]);   
           int idx2 = udp_api_add_client(&udp_api_hd[1], ch_local_port[1][idx], dstIp, dstPort[1]);   
           if ((idx1 != idx) || (idx2 != idx)){
                rt_kprintf("new channel is unbalance, %d %d\n", idx1, idx2);
                udp_api_del_client(&udp_api_hd[0], idx1);
                udp_api_del_client(&udp_api_hd[1], idx2);
               return -1;
           }
        }
    }

    /*
    if (!udp_api_hd[0])
        udp_api_hd[0] = udp_api_create(ch_local_port[0][0], dstIp, dstPort, "udp0");
    else {
        idx = udp_api_add_client(&udp_api_hd[0], ch_local_port[0][0], dstIp, dstPort);
    }
    if (!udp_api_hd[1])
        udp_api_hd[1] = udp_api_create(ch_local_port[1][0], dstIp, dstPort, "udp1");
    else{
        idx = udp_api_add_client(&udp_api_hd[1], ch_local_port[1][0], dstIp, dstPort);
    }*/
#endif   
    //确保adc只被启动一次
    if (app_adc_is_exist()) return idx;
    
    if (!app_create_adc()){
        udp_api_delete(&udp_api_hd[0]);            
        udp_api_delete(&udp_api_hd[1]);    
        return -1;
    }
    set_adc_callback_function(RT_NULL, adc_callback);
    if (!app_start_adc())
        return -1;
    return idx;
}
rt_bool_t app_delete_send(const char* dstIp, rt_uint16_t srcPort[2]){
    int idx = find_index(dstIp, srcPort);       
    if ((idx < 0) || (idx >= UDP_MAX_CLINET)) return RT_FALSE;
    
    int count1 = udp_api_get_client_count(udp_api_hd[0]);
    int count2 = udp_api_get_client_count(udp_api_hd[1]);
    if (count1 != count2) return RT_FALSE;
    
    if (count1 ==1){
        app_delete_adc();        
#if defined(RT_USING_RUDP)
        udp_api_delete(&udp_api_hd[0]);            
        udp_api_delete(&udp_api_hd[1]);            
#endif          
    }
    else{
        udp_api_del_client(&udp_api_hd[0], idx);
        udp_api_del_client(&udp_api_hd[1], idx);
    }   
    return RT_TRUE;
}

/*
#ifdef RT_USING_LWIP
#include <netif/ethernetif.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <netdev.h>
//extern struct netdev *netdev_get_first_by_flags(uint16_t flags);
#endif
static int adcudp(int argc, char** argv){
    static rt_bool_t bInUse = RT_FALSE;
    char *dstip = RT_NULL,*srcip = RT_NULL;
    if (argc == 1){
        rt_kprintf("current application is %s\n", (bInUse?"in use":"not in use"));
    }
    else{//argc > 1
        if (0 == rt_strcmp(argv[1], "stop")){//stop
            if (argc == 2){
                if (bInUse){
                    app_delete_send();
                    rt_kprintf("application is stopped\n");
                    bInUse = RT_FALSE;
                    return 0;
                }
                else{
                    rt_kprintf("current application is not in use\n");
                }
            }
            else{//argc > 2
              rt_kprintf("parameter '%s' is too much\n", argv[1]);
           }
        }
        else if (0 != rt_strcmp(argv[1], "start")){//argv[1] != "start"
           rt_kprintf("parameter '%s' is unkown\n", argv[1]);
        }
        else{//argv[1] == "start"
            if (argc == 2){
                rt_kprintf("parameter is not enough\n");            
            }
            else if (argc > 4){
                 rt_kprintf("parameter '%s' is too much\n", argv[1]);
            }
            else{
                ip_addr_t r, l, gw; 
                dstip = argv[2];
                if (argc == 4)
                    srcip = argv[3];
                if(!inet_aton(dstip, &r.addr)){
                    rt_kprintf("remote address '%s' is invalid\n", dstip);
                }
                else{
                    if (srcip){
                        if(!inet_aton(srcip, &l.addr)){
                            rt_kprintf("local address '%s' is invalid\n", srcip);
                            return -1;
                        }
                        else{
                            gw = l;
                            gw.addr = (gw.addr & 0x00FFFFFF) | (0x01000000);
                        }
                    }
                    if (bInUse){
                        rt_kprintf("current application is not in use\n");
                    }
                    else{
                        if (srcip){//设置当前IP
                            struct netdev * dev = netdev_get_by_name("e0");
                            if (!dev){
                                rt_kprintf("config local ip '%s' failed, get netdev 'e0' failed\n", srcip);
                            }
                            else{
                                rt_uint8_t cnt = 15;
                                netdev_set_down(dev);
                                netdev_set_ipaddr(dev,&l);
                                netdev_set_gw(dev,&gw);                                
                                netdev_set_up(dev);
                                rt_kprintf("config local ip '%s' finished, wait link up...\n", srcip);
                                while ((!(dev->flags & NETIF_FLAG_LINK_UP)) && (!cnt)){
                                    rt_thread_delay(RT_TICK_PER_SECOND/5);cnt--;
                                }
                                if (cnt > 0){
                                    rt_kprintf("netif link up\n");                                
                                }
                                else{
                                    rt_kprintf("netif link up failed\n");
                                }
                            }
                        }
                        app_create_send(dstip);
                        bInUse = RT_TRUE;
                        rt_kprintf("application is started\n");
                        return 0;                        
                    }                    
                }              
            }
        }
    }
    return -1;
}
MSH_CMD_EXPORT(adcudp, adc data send)*/
