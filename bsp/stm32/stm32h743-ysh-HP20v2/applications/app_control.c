#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "app_control.h"
#include "app_send.h"

//#define DRV_DEBUG
#define LOG_TAG      "app.control"
#include <drv_log.h>

#ifdef RT_USING_LWIP
#include "lwip/init.h"

#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/netifapi.h"

#include "netif/ethernetif.h"

#include <lwip/api.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#endif

#ifdef RT_USING_LWIP

#define MAX_CLIENT_SZ   2

rt_thread_t tid_conn;
rt_thread_t tid_disconn;
rt_thread_t tid_recv;
rt_event_t  evt_disconn;
rt_sem_t    sem_conn;
rt_mutex_t  mt_receive;
rt_mutex_t  mt_send;
//char        destip[20] = "192.168.199.155";
int			nSockClient[MAX_CLIENT_SZ] = {-1, -1};

static int ConfigSocket(int socket){
    
	const rt_uint32_t chOpt=1;   
	const rt_uint32_t keepalive = SO_KEEPALIVE;         // 打开探测
	const rt_uint32_t keepidle = 3;                     // 开始探测前的空闲等待时间
	const rt_uint32_t keepintvl = 1;                    // 发送探测分节的时间间隔
	const rt_uint32_t keepcnt = 3;                      // 发送探测分节的次数
    const rt_uint32_t timeout = RT_TICK_PER_SECOND;
#if LWIP_VERSION_MAJOR >= 2U
    struct timeval recv_timeout = { timeout / RT_TICK_PER_SECOND, timeout % RT_TICK_PER_SECOND };
#else
    int recv_timeout = timeout * 1000UL / RT_TICK_PER_SECOND;
#endif
    const rt_uint32_t on = 1;

	if (socket < 0) return -1;

	if (0 > setsockopt( socket,   SOL_SOCKET,   SO_RCVTIMEO,   &recv_timeout,   sizeof(recv_timeout))){
		rt_kprintf("set socket opt LWIP_SO_RCVTIMEO failed!\n");
		return -1;
	}
	if (0 > setsockopt( socket,   IPPROTO_TCP,   TCP_NODELAY,   &chOpt,   sizeof(chOpt)))   {
		rt_kprintf("set socket opt TCP_NODELAY failed!\n");
		return -1;
	}

	if (0 > setsockopt(socket, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepalive, sizeof (keepalive))){
		rt_kprintf("fail to set SO_KEEPALIVE\n");
		return -1;
	}
	if (0 > setsockopt(socket, IPPROTO_TCP, TCP_KEEPIDLE, (void *) &keepidle, sizeof (keepidle))){
		rt_kprintf("fail to set TCP_KEEPIDLE\n");
		return -1;
	}
	if (0 > setsockopt(socket, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&keepintvl, sizeof (keepintvl))){
		rt_kprintf("fail to set TCP_KEEPINTVL\n");
		return -1;
	}
	if (0 > setsockopt(socket, IPPROTO_TCP, TCP_KEEPCNT, (void *)&keepcnt, sizeof (keepcnt))){
		rt_kprintf("fail to set TCP_KEEPCNT\n");
		return -1;
	}	
    if (0 >  setsockopt( socket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on) )){
		rt_kprintf("Reuse listener port failed\n");
		return -1;
	}

	return 0;	
}
static int MakeListener(const rt_uint16_t port){    
 	int nSockServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	struct sockaddr_in 	sin = {0};

	if (nSockServer < 0)
		goto ret;
    sin.sin_family = AF_INET;
	sin.sin_port = htons(port);
	sin.sin_addr.s_addr = htonl(INADDR_ANY);
    
	if ( 0 != ConfigSocket(nSockServer))	{
		rt_kprintf("Config listener failed!\n");
		goto ret;
	}	

	if(bind(nSockServer,(struct sockaddr*)&sin, sizeof(sin)) < 0){
		rt_kprintf("Bind failed!\n");
		goto ret;
	}

	if (listen(nSockServer,1) < 0)	{
		rt_kprintf("Listen failed!\n");
		goto ret;
	}
    return nSockServer;
ret:
	if (nSockServer >= 0){
		closesocket(nSockServer);
	}
	return -1;	
   
}
static int WaitForClient(int nSockServer, int timeout){
	struct timeval	 	tim={timeout/1000, (timeout%1000) * 1000};
	fd_set fdRead;
	if (nSockServer < 0)
		return -1;    
 	FD_ZERO(&fdRead);
	FD_SET(nSockServer,&fdRead);
	
	return select(FD_SETSIZE, &fdRead, NULL, NULL,&tim);
}
static int AcceptConnector(int nSockServer){
	int nSocket = -1;
	struct sockaddr_in 	din = {0};
    socklen_t           dlen = sizeof(struct sockaddr_in);
 	if (nSockServer < 0)
		return -1;   
    nSocket = accept(nSockServer, (struct sockaddr*)&din, &dlen);
    rt_kprintf("Accept client address : '%s'\tport: '%d'\n",inet_ntoa(din.sin_addr.s_addr),din.sin_port);
    if (0 != ConfigSocket(nSocket)){
        rt_kprintf("Config client failed!\n");
        closesocket(nSocket);
        return -1;
    }
    rt_kprintf("Accept %d\n", nSocket);
    return nSocket;
}

/*

static int AcceptClient(const rt_uint16_t port)
{
	int nSockServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	int nSocket = -1;
	int	nRet = -1;
	struct sockaddr_in 	sin = {0};
	struct sockaddr_in 	din = {0};
	struct timeval	 	tim={10,0};
    socklen_t           dlen = sizeof(din);
	fd_set fdRead;

	if (nSockServer < 0)
		goto ret;
    sin.sin_family = AF_INET;
	sin.sin_port = htons(port);
	sin.sin_addr.s_addr = htonl(INADDR_ANY);
    
	if ( 0 != ConfigSocket(nSockServer))	{
		rt_kprintf("Config listener failed!\n");
		goto ret;
	}	

	if(bind(nSockServer,(struct sockaddr*)&sin, sizeof(sin)) < 0){
		rt_kprintf("Bind failed!\n");
		goto ret;
	}

	if (listen(nSockServer,1) < 0)	{
		rt_kprintf("Listen failed!\n");
		goto ret;
	}
	FD_ZERO(&fdRead);
	FD_SET(nSockServer,&fdRead);
	
	nRet = select(FD_SETSIZE, &fdRead, NULL, NULL,&tim);
	if (nRet > 0) {
		nSocket = accept(nSockServer, (struct sockaddr*)&din, &dlen);
		rt_kprintf("Accept client address : '%s'\tport: '%d'\n",inet_ntoa(din.sin_addr.s_addr),din.sin_port);
		if (0 != ConfigSocket(nSocket)){
			rt_kprintf("Config client failed!\n");
			closesocket(nSocket);
			goto ret;
		}	
	}	
	closesocket(nSockServer);
	nSockServer = -1;
ret:
	if (nSockServer >= 0){
		closesocket(nSockServer);
		rt_kprintf("Network: Accept error\n");
	}
	return nSocket;	
}*/

static void accept_thread_entry(void* parameter){
    rt_uint16_t port = (rt_uint16_t) (rt_uint32_t)parameter;
    RT_ASSERT(RT_NULL != sem_conn);
    while(1){
        int sockIdx = -1;
        for (int n = 0; n < MAX_CLIENT_SZ; n++){
            if (-1 == nSockClient[n]){
                sockIdx = n;
                break;
            }
        }
        if (sockIdx != -1){
           int sockServer = MakeListener(port); 
           if (sockServer == -1){
               rt_thread_delay(RT_TICK_PER_SECOND/10);
               continue;
           }
           if (WaitForClient(sockServer, 10000)> 0){
               int sock = AcceptConnector(sockServer);
               if (sock >=0){
                   nSockClient[sockIdx] = sock;
                   rt_pin_write(CONV_PIN, PIN_HIGH);  
                   rt_kprintf("Accept client， socket = %d\n", sock);
               }
           }
           closesocket(sockServer);           
           continue;
           //nSockClient[sockIdx] = AcceptClient(port);
        }        
   		rt_sem_take(sem_conn, RT_WAITING_FOREVER);  
/*        
        rt_mutex_take(mt_receive, RT_WAITING_FOREVER);  
        rt_mutex_take(mt_send, RT_WAITING_FOREVER);  
 		nSockClient = -1;
		closesocket(sock);
        rt_mutex_release(mt_send);
		rt_mutex_release(mt_receive);
        rt_pin_write(CONV_PIN, PIN_LOW);        
       	rt_kprintf("Disconnect client\n");*/
        
   }
}
static void disconnect_thread_entry(void* parameter){
    rt_uint32_t recved;
    while(1){
        if (RT_EOK == rt_event_recv(evt_disconn, 0xFFFFFFFF
            , RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &recved)){     
            if (recved == 0) continue;
            rt_mutex_take(mt_receive, RT_WAITING_FOREVER);  
            rt_mutex_take(mt_send, RT_WAITING_FOREVER);  
            int cnt = 0;
            for (int n = 0; n < MAX_CLIENT_SZ; n++){
                if (recved & (0x00000001 << n)){
                    if (-1 != nSockClient[n]){
                        int sock = nSockClient[n];
                        nSockClient[n] = -1;
                        closesocket(sock);                        
                        rt_kprintf("Disconnect client, socket = %d\n", sock);             
                    }
                }
                if (-1 != nSockClient[n])
                    cnt++;
            }
            if (cnt == (MAX_CLIENT_SZ-1)) rt_sem_release(sem_conn);
            rt_mutex_release(mt_send);
            rt_mutex_release(mt_receive);
            rt_bool_t bSockExist = RT_FALSE;
            for (int n = 0; n < MAX_CLIENT_SZ; n++){
                if (-1 != nSockClient[n]){
                    bSockExist = RT_TRUE;
                    break;
                }
            }
            if (!bSockExist)            
                rt_pin_write(CONV_PIN, PIN_LOW);        
        }      
    }
}

/*
static void accept_thread_entry(void* parameter){
    rt_uint16_t port = (rt_uint16_t) (rt_uint32_t)parameter;
    RT_ASSERT(RT_NULL != sem_conn);
    while(1){
        int sock = nSockClient;
        if (-1 == sock){//需要连接
            nSockClient = AcceptClient(port);
			if (-1 == nSockClient)
				rt_thread_delay(RT_TICK_PER_SECOND/100);
			else
                rt_pin_write(CONV_PIN, PIN_HIGH);     
            continue;
        }
 		rt_kprintf("Accept client\n");
   		rt_sem_take(sem_conn, RT_WAITING_FOREVER);     
        rt_mutex_take(mt_receive, RT_WAITING_FOREVER);  
        rt_mutex_take(mt_send, RT_WAITING_FOREVER);  
 		nSockClient = -1;
		closesocket(sock);
        rt_mutex_release(mt_send);
		rt_mutex_release(mt_receive);
        rt_pin_write(CONV_PIN, PIN_LOW);        
       	rt_kprintf("Disconnect client\n");
   }
}
*/
static int get_readable_socket_index(void){
   	fd_set 					fdrd;
	struct timeval 	        tim;
	int 					nRet = -1;
    int                     cnt = 0;
	FD_ZERO(&fdrd);
    rt_mutex_take(mt_receive,RT_WAITING_FOREVER);
    for (int n = 0; n < MAX_CLIENT_SZ; n++){
        if (nSockClient[n] != -1){
            FD_SET(nSockClient[n],&fdrd);  
            cnt++;            
        }
    }
    if (cnt > 0){
        tim.tv_sec = 5;
        tim.tv_usec = 0;
		nRet = select(FD_SETSIZE,&fdrd,0,0,&tim);
        if (nRet > 0){
            nRet = -1;
            for (int n = 0; n < MAX_CLIENT_SZ; n++){
                if (nSockClient[n] != -1){
                    if (!FD_ISSET(nSockClient[n],&fdrd))  
                        continue;
                    nRet = n;
                    break;
                }
            } 
        }
        else
            nRet = -1;        
    }
    else
        nRet = -2;
 	rt_mutex_release(mt_receive);
    return nRet;
}

static int receive_bytes(int nSock, char* cmdbuffer, int cmdbytes){
   	fd_set 					fdrd;
	struct timeval 	        tim;
	int 					nRet;
	rt_int32_t rBytes = 0;
	rt_int32_t tBytes = 0;

	
	if (0 >= cmdbytes) return 0;
	if (-1 == nSock) return 0;
	rt_mutex_take(mt_receive,RT_WAITING_FOREVER);
	while(rBytes < cmdbytes){		
		FD_ZERO(&fdrd);
		FD_SET(nSock,&fdrd);
        tim.tv_sec = 0;
        tim.tv_usec = 200000;
		nRet = select(FD_SETSIZE,&fdrd,0,0,&tim);
		if (nRet < 0){
			rBytes = -1;
			break;
		}
		if (nRet == 0)
			break;
		tBytes = recv(nSock,cmdbuffer+rBytes,cmdbytes - rBytes,0);
		if (tBytes <= 0){
			rBytes = -1;
			break;
		}
		rBytes += tBytes;
      	tim.tv_sec = 0;
        tim.tv_usec = 200000;  
	}
	rt_mutex_release(mt_receive);
	return rBytes; 
}

static int send_bytes(int nSock, char* cmdbuffer, int cmdbytes)
{	
	fd_set 					fdwd;
	struct timeval 	tim;
	int 						nRet;
	rt_int32_t sBytes = 0;
	rt_int32_t tBytes = 0;

	tim.tv_sec = 0;
	tim.tv_usec = 10000;	

	
	if (0 == cmdbytes) return 0;
	if (-1 == nSock) return 0;
	rt_mutex_take(mt_send,RT_WAITING_FOREVER);
    
	while(sBytes < cmdbytes){
		FD_ZERO(&fdwd);
		FD_SET(nSock,&fdwd);
		nRet = select(FD_SETSIZE,0,&fdwd,0,&tim);
		if (nRet < 0) break;
		if (nRet == 0) continue;
		tBytes = send(nSock,cmdbuffer+sBytes,cmdbytes - sBytes,0);
		if (tBytes < 0) 	break;
		sBytes += tBytes;
	}
	rt_mutex_release(mt_send);
	return sBytes;
}

#define CMD_HEADER "CCB+"
#define CMD_TAILER "\r\n"
#define CMD_DOT     ","
static int receive_frame(int nSock, char* framebuffer, int framebytes){
	const char* cpheader = CMD_HEADER;
	const char* cptailer = CMD_TAILER;
	const int clheader = rt_strlen(cpheader);
	const int cltailer = rt_strlen(cptailer);
	char* pheader = framebuffer;
	char* ptailer = 0;
   	int tframe = 0;
	int rframe = 0;		
	rt_memset(framebuffer,0,framebytes); 
 	while(1)
	{
		tframe = receive_bytes(nSock, framebuffer + rframe,1);
		if (tframe < 0) return -1;
		if (tframe == 0) 	{
			if (0 == rframe) return 0;
			else return -1;
		}
		tframe = rframe;
		rframe ++;
		if (rframe <= clheader) {
			if (cpheader[tframe] != pheader[tframe])
				goto proc_err;
		}
		else if (RT_NULL != ptailer) {
			int offset = rframe - (ptailer - framebuffer) -1;
			if (ptailer[offset] != cptailer[offset]) goto proc_err;
			else if (offset == (cltailer - 1)) 	return rframe;
		}
		else if (framebuffer[tframe] == cptailer[0]) 	 ptailer = framebuffer + tframe;
		
		if (rframe >= framebytes) goto proc_err;
		continue;
proc_err:
		rframe = 0;
		ptailer = 0;
	}   
}

static int unpackage_frame(char* cmdstring,char** itemstring, int itemsz){
    const char* pheader  = CMD_HEADER;
	const char* psplit = CMD_DOT;
    const char* ptailer     = CMD_TAILER;
	const int   lheader = rt_strlen(pheader);
	const int 	lsplit = rt_strlen(psplit);
	const int 	ltailer     = rt_strlen(ptailer);
	int			paramcnt = 0;
    rt_memset(itemstring,0,sizeof(char*)*itemsz);
    cmdstring = rt_strstr(cmdstring,pheader);
	RT_ASSERT(cmdstring != RT_NULL);
	itemstring[0] = cmdstring + lheader;
	for (paramcnt = 1; paramcnt < itemsz; paramcnt++){
		cmdstring = rt_strstr(itemstring[paramcnt-1],psplit);
		if (RT_NULL == cmdstring){
            cmdstring = rt_strstr(itemstring[paramcnt-1],ptailer);
            if (cmdstring)
                rt_memset(cmdstring, 0, ltailer);
            
			break;
        }
        rt_memset(cmdstring, 0, lsplit);
		itemstring[paramcnt] = cmdstring + lsplit;
	}	
	return paramcnt;	
}
	
static int package_frame(char* cmdstring, int cmdlen,char** itemstring, int itemsz){
    const char* pheader     = CMD_HEADER;
	const char* psplit      = CMD_DOT;
    const char* ptailer     = CMD_TAILER;
	const int   lheader     = rt_strlen(pheader);
	const int 	lsplit      = rt_strlen(psplit);
	const int 	ltailer     = rt_strlen(ptailer);
	int			cmdbytes = 0;
    
    if ((cmdbytes + lheader) >= cmdlen) return -1;
    rt_memcpy(cmdstring + cmdbytes, pheader, lheader);
    cmdbytes += lheader;
    for (int n = 0; n < itemsz; n++){
      int itemlen = rt_strlen(itemstring[n]);
      if ((cmdbytes + itemlen) >= cmdlen) return -1;
      rt_memcpy(cmdstring + cmdbytes, itemstring[n], itemlen);
      cmdbytes += itemlen;
      if ((n+1)<itemsz){
          if ((cmdbytes + lsplit) >= cmdlen) return -1;
          rt_memcpy(cmdstring + cmdbytes, psplit, lsplit);
          cmdbytes += lsplit;  
       }        
    }
    if ((cmdbytes + ltailer) >= cmdlen) return -1;
    rt_memcpy(cmdstring + cmdbytes, ptailer, ltailer);
    cmdbytes += ltailer;       
    cmdstring[cmdbytes] = 0;    
	return cmdbytes;	
}

static char remote_ip_buffer[20];
static int get_peer_ip(int sock, const char* rip){
    struct sockaddr_storage sa;
    socklen_t salen = sizeof(sa);
    if (getpeername(sock, (struct sockaddr*)&sa, &salen) == -1) return -1;
	
    if (sa.ss_family == AF_INET) {
        struct sockaddr_in *s = (struct sockaddr_in*)&sa;
        rt_strncpy((char*)rip, inet_ntoa(s->sin_addr), 20);
        rt_kprintf("sock[%d] is %s\n", sock, rip);
		return 0;
	}
    return -1;
}

#define FRAME_CMD_START     "START"
#define FRAME_CMD_STOP      "STOP"
#define FRAME_CMD_REBOOT    "REBOOT"
#define FRAME_RES_OK        "OK"
#define FRAME_RES_FAILD     "ERR"

#define APP_CONTROL_BUFFFER_SIZE 1024
#define APP_CONTROL_BUFFFER_BASE rbuffer
#define MAX_FRAME_ITEM_CNT  13
static char rbuffer[APP_CONTROL_BUFFFER_SIZE];

static void receive_thread_entry(void * parameter){
    char* const cmdframe 	= (char*) APP_CONTROL_BUFFFER_BASE;
    const int	lcmdframe = APP_CONTROL_BUFFFER_SIZE - 1;
    char*   	items[MAX_FRAME_ITEM_CNT] = {0};

    int rbytes = 0;

    cmdframe[lcmdframe] = 0;

    while (1){
        int idx = get_readable_socket_index();
        if (idx < 0){
           	rt_thread_delay(RT_TICK_PER_SECOND/100);
			continue;
        }
        int sock = nSockClient[idx];
		if (-1 == sock) {rt_kprintf("receive select error, socket index is %d\n", idx);continue;}
		
  		rbytes = receive_frame(sock, cmdframe,lcmdframe);  
        if (rbytes <0) {
			if (nSockClient[idx] != -1)     rt_event_send(evt_disconn, 0x00000001<<idx);//rt_sem_release(sem_conn);//___________________________________________________________________________改成event
			rt_thread_delay(RT_TICK_PER_SECOND/100);
			continue;
		}
		if (rbytes == 0) continue;    
      	cmdframe[rbytes] = 0;  
        rbytes = unpackage_frame(cmdframe, items, MAX_FRAME_ITEM_CNT);
        if (0 == rt_strcmp(items[0], FRAME_CMD_START)){
            char lport[2][7];
            rt_kprintf("Start ADC & UDP received\n");
            if (rbytes < 3){
                items[0] = FRAME_RES_FAILD;
                rbytes = 2;
            }
            else if (get_peer_ip(sock, remote_ip_buffer) != 0){
                items[0] = FRAME_RES_FAILD;
                rbytes = 2;
            }
            else{
                uint16_t port[2];
                port[0] = atoi(items[1]);
                port[1] = atoi(items[2]);
                int idx = app_create_send(remote_ip_buffer, port);
                rt_kprintf("get index is %d\n", idx);
                if (idx < 0)  {
                    items[0] = FRAME_RES_FAILD;
                    rbytes = 2;
                }
                else{
                    items[0] = FRAME_RES_OK;
                    items[2] = lport[0];
                    items[3] = lport[1];
                    rbytes = 4;
                    rt_snprintf(lport[0], 7, "%u", get_local_port(0, idx));
                    rt_snprintf(lport[1], 7, "%u", get_local_port(1, idx));                    
                }
            }            
            items[1] = FRAME_CMD_START;
            rbytes = package_frame(cmdframe, lcmdframe, items, rbytes);
            if (rbytes != send_bytes(sock, cmdframe, rbytes)){
                rt_event_send(evt_disconn, 0x00000001<<idx);//rt_sem_release(sem_conn);//___________________________________________________________________________改成event
            }
        }
        else if (0 == rt_strcmp(items[0], FRAME_CMD_STOP)){
            rt_kprintf("Stop ADC & UDP received\n");
            if (rbytes < 3){
                items[0] = FRAME_RES_FAILD;
                rbytes = 2;
            }
            else if (get_peer_ip(sock, remote_ip_buffer) != 0){
                items[0] = FRAME_RES_FAILD;
                rbytes = 2;
            }
            else{
                uint16_t port[2];
                port[0] = atoi(items[1]);
                port[1] = atoi(items[2]);

                if (app_delete_send(remote_ip_buffer, port)){
                    items[0] = FRAME_RES_OK;
                    rbytes = 2;
                }
                else {
                    items[0] = FRAME_RES_FAILD;  
                    rbytes = 2;
                    
                }                    
            }
            items[1] = FRAME_CMD_STOP;

            rbytes = package_frame(cmdframe, lcmdframe, items, rbytes);
            if (rbytes != send_bytes(sock, cmdframe, rbytes)){
                rt_event_send(evt_disconn, 0x00000001<<idx);//rt_sem_release(sem_conn);//___________________________________________________________________________改成event
            }
        }
        else if (0 == rt_strcmp(items[0], FRAME_CMD_REBOOT)){
            rt_hw_cpu_reset();
        }
        
        /*else if (0 == rt_strcmp(items[0], FRAME_CMD_RESPOND)){
        }*/
    }   
}

#include "cJSON.h"
#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>

#define DEFAULT_SERVER_PORT         60606

#define DEFAULT_UDP1_LOCALPORT0     60000
#define DEFAULT_UDP1_LOCALPORT1     60004
#define DEFAULT_UDP2_LOCALPORT0     60001
#define DEFAULT_UDP2_LOCALPORT1     60005

rt_uint16_t load_server_port(void)
{
    const char * FILENAME = "netif_config.json";
    const rt_size_t JSON_BUFFER_SIZE = 1024;
    rt_size_t size;
    cJSON *root;
    char *json;
    int fd = RT_NULL;
    
    fd = open(FILENAME, O_RDONLY, 0);
    if (fd < 0) {
        LOG_W("read config failed! creating a default config");
        
        fd = open(FILENAME, O_WRONLY | O_CREAT, 0);
        if (fd < 0) {
            LOG_E("create config file failed!");
            return 0;
        }
        root = cJSON_CreateObject();
        
        cJSON_AddItemToObject(root, "SERVER_PORT", cJSON_CreateNumber(DEFAULT_SERVER_PORT));
        cJSON_AddItemToObject(root, "UDP1_LocalPort0", cJSON_CreateNumber(DEFAULT_UDP1_LOCALPORT0));
        cJSON_AddItemToObject(root, "UDP1_LocalPort1", cJSON_CreateNumber(DEFAULT_UDP1_LOCALPORT1));
        cJSON_AddItemToObject(root, "UDP2_LocalPort0", cJSON_CreateNumber(DEFAULT_UDP2_LOCALPORT0));
        cJSON_AddItemToObject(root, "UDP2_LocalPort1", cJSON_CreateNumber(DEFAULT_UDP2_LOCALPORT1));
        
        json = cJSON_Print(root);
        write(fd, json, rt_strlen(json));
        rt_free(json);
        close(fd);
    }
    else {
        json = rt_malloc(JSON_BUFFER_SIZE);
        
        if (json == RT_NULL) {
            LOG_E("malloc buffer for json failed!");
            return 0;
        }
        
        size = read(fd, json, JSON_BUFFER_SIZE);
        if (size == JSON_BUFFER_SIZE)
            LOG_W("json buffer is full!");
        
        root = cJSON_Parse(json);
        rt_free(json);
        close(fd);
    }
    
    rt_uint16_t port = cJSON_GetObjectItem(root, "SERVER_PORT")->valueint;

    LOG_I("server use %d to listening...", port);
    
    cJSON_Delete(root);
    
    return port;
}

/*
void update_remote_address(void){
#ifdef RT_USING_INI_HELPER
    const char* filename = "control.ini";
    int err, line;
    struct ini_file* inifile = ini_read(filename,&err,&line);
	if (inifile){
        char *sip = (char*)ini_get(inifile,"REMOTE", "Address", RT_NULL);
        if (sip)  rt_strncpy(destip, sip, 20);      
        else {
            ini_free(inifile);	
            inifile = RT_NULL;
        }
        
    }
    if (!inifile){
        char tport[7];
        rt_snprintf(tport, 7, "%d", DEFAULT_SERVER_PORT);
        inifile = ini_read(RT_NULL,&err,&line);
        if (!inifile) return ;
        while (1 != ini_put(inifile,"NETOWRK","Port",tport)) rt_thread_delay(1);
        while (1 != ini_put(inifile,"REMOTE","Address",destip)) rt_thread_delay(1);
        ini_write(inifile, filename);	rt_thread_delay(1);
    }
    ini_free(inifile);	
    read_udp_port();
#endif
}*/

void adc_control_server(void){
    rt_uint32_t port = load_server_port();
    //update_remote_address();
    
 	tid_disconn = rt_thread_create("ndconn", disconnect_thread_entry, (void*)port, 4096,15,5);   
 	tid_conn = rt_thread_create("nconn", accept_thread_entry, (void*)port, 4096,15,5);   
    tid_recv = rt_thread_create("nrecv", receive_thread_entry, RT_NULL, 4096,16,5);

    sem_conn = rt_sem_create("sConn",0,RT_IPC_FLAG_FIFO);
    evt_disconn = rt_event_create("eConn",RT_IPC_FLAG_FIFO);
	mt_receive = rt_mutex_create("mtRcv",RT_IPC_FLAG_FIFO);
	mt_send = rt_mutex_create("mtSnd",RT_IPC_FLAG_FIFO);

   	RT_ASSERT(tid_disconn && tid_conn && tid_recv && sem_conn && mt_receive && mt_send);
 	rt_thread_startup(tid_disconn);
 	rt_thread_startup(tid_conn);
	rt_thread_startup(tid_recv);
}
#endif
