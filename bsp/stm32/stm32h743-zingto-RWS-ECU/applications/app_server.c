#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "app_env.h"
#include "app_server.h"

#ifdef RT_USING_LWIP
#include <netif/ethernetif.h>
#include <netdev.h>
#include "lwip/opt.h"
#include "lwip/init.h"
#include <lwip/api.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <netdev.h>


static rt_bool_t ConfigSocket(int socket){
	const rt_uint32_t chOpt=1;   
	const rt_uint32_t keepalive = SO_KEEPALIVE;        // 打开探测
	const rt_uint32_t keepidle = 3;        // 开始探测前的空闲等待时间
	const rt_uint32_t keepintvl = 1;        // 发送探测分节的时间间隔
	const rt_uint32_t keepcnt = 3;        // 发送探测分节的次数
    const rt_uint32_t timeout = RT_TICK_PER_SECOND;
#if LWIP_VERSION_MAJOR >= 2U
    struct timeval recv_timeout = { timeout / RT_TICK_PER_SECOND, (timeout % RT_TICK_PER_SECOND) * 1000};
#else
    int recv_timeout = timeout * 1000UL / RT_TICK_PER_SECOND;
#endif
    const rt_uint32_t on = 1;

	if (socket < 0) return RT_FALSE;

	if (0 > setsockopt( socket,   SOL_SOCKET,   SO_RCVTIMEO,   &recv_timeout,   sizeof(recv_timeout))){
		rt_kprintf("set socket opt LWIP_SO_RCVTIMEO failed!\n");
		return RT_FALSE;
	}
	if (0 > setsockopt( socket,   IPPROTO_TCP,   TCP_NODELAY,   &chOpt,   sizeof(chOpt)))   {
		rt_kprintf("set socket opt TCP_NODELAY failed!\n");
		return RT_FALSE;
	}

	if (0 > setsockopt(socket, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepalive, sizeof (keepalive))){
		rt_kprintf("fail to set SO_KEEPALIVE\n");
		return RT_FALSE;
	}
	if (0 > setsockopt(socket, IPPROTO_TCP, TCP_KEEPIDLE, (void *) &keepidle, sizeof (keepidle))){
		rt_kprintf("fail to set TCP_KEEPIDLE\n");
		return RT_FALSE;
	}
	if (0 > setsockopt(socket, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&keepintvl, sizeof (keepintvl))){
		rt_kprintf("fail to set TCP_KEEPINTVL\n");
		return RT_FALSE;
	}
	if (0 > setsockopt(socket, IPPROTO_TCP, TCP_KEEPCNT, (void *)&keepcnt, sizeof (keepcnt))){
		rt_kprintf("fail to set TCP_KEEPCNT\n");
		return RT_FALSE;
	}	
    if (0 >  setsockopt( socket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on) )){
		rt_kprintf("Reuse listener port failed\n");
		return RT_FALSE;
	}
	return RT_TRUE;	
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
static int MakeListener(const rt_uint16_t port){    
 	int nSockServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	struct sockaddr_in 	sin = {0};

	if (nSockServer < 0)
		goto ret;
    sin.sin_family = AF_INET;
	sin.sin_port = htons(port);
	sin.sin_addr.s_addr = htonl(INADDR_ANY);
    
	if (!ConfigSocket(nSockServer))	{
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
static int AcceptConnector(int nSockServer){
	int nSocket = -1;
	struct sockaddr_in 	din = {0};
    socklen_t           dlen = sizeof(struct sockaddr_in);
 	if (nSockServer < 0)
		return -1;   
    nSocket = accept(nSockServer, (struct sockaddr*)&din, &dlen);
    rt_kprintf("Accept client address : '%s'\tport: '%d'\n",inet_ntoa(din.sin_addr.s_addr),din.sin_port);
    if (!ConfigSocket(nSocket)){
        rt_kprintf("Config client failed!\n");
        closesocket(nSocket);
        return -1;
    }
    rt_kprintf("Accept %d\n", nSocket);
    return nSocket;
}
static int ReceiveCommand(int nSockClient, char rBuf[CTRL_CMD_MAX]){
    if (nSockClient < 0) return -1;
    const char* HEADER = CTRL_CMD_HEADER;
    const char* TAILER = CTRL_CMD_TAILER;
    const int HEADER_LEN = strlen(HEADER);
    const int TAILER_LEN = strlen(TAILER);
    int rBytes = 0;
    int ret = 0;
    rt_memset(rBuf, 0, CTRL_CMD_MAX);
    
    ret = WaitForClient(nSockClient, 200);
    if (ret <= 0) return ret;
    while (rBytes < CTRL_CMD_MAX){
        ret = recv(nSockClient,rBuf+rBytes,1,0);
        if (ret <= 0) return -1;
        rBytes++;
        if (rBytes <= HEADER_LEN){
            if (rBuf[rBytes-1] != HEADER[rBytes-1]) {
                rBytes = 0;
            }
        }   
        else if (rBytes >= (HEADER_LEN+TAILER_LEN)){
            rt_bool_t bTailer = RT_TRUE;
            for (int n = 1; n <= TAILER_LEN; n++){
                if (rBuf[rBytes-n] != TAILER[TAILER_LEN-n]){
                    bTailer = RT_FALSE;
                    break;
                }
            }
            if (bTailer) break;
        }
        ret = WaitForClient(nSockClient, 50);
        if (ret <= 0) return ret;
    }
    return rBytes;    
}
static int AnalyseCommand(char cmd[CTRL_CMD_MAX], char* param[CTRL_CMD_PARAM_MAX]){
    const int HEADER_LEN = strlen(CTRL_CMD_HEADER);
    const int TAILER_LEN = strlen(CTRL_CMD_TAILER);
    const char* sDot = ",";
    int startPos = HEADER_LEN;
    int cnt = 0;
    rt_bool_t bLast = RT_FALSE;
    rt_memset(param, 0, sizeof(param));
    while (cnt < CTRL_CMD_PARAM_MAX){

        char* pEnd = strstr(cmd+startPos,sDot);
        if (!pEnd) {
            pEnd = strstr(cmd+startPos,CTRL_CMD_TAILER);
            bLast = RT_TRUE;
        }
        if (!pEnd) return 0;        
        param[cnt++] = cmd+startPos;
        pEnd[0] = 0;
        if (bLast) break;
        startPos =  pEnd - cmd + 1;
    }
    return cnt;   
}
static CtrlCommand_t GetCommand(const char* cmd, const char* param1){
    CtrlCommand_t cmdCode = Cmd_ByPass;
    if (rt_memcmp(CTRL_CMD_KEEP_POSITION, cmd, strlen(CTRL_CMD_KEEP_POSITION)) == 0){
        cmdCode = Cmd_KeepPosition;
    }   
    else if (rt_memcmp(CTRL_CMD_KEEP_EULAR, cmd, strlen(CTRL_CMD_KEEP_EULAR)) == 0){
        cmdCode = Cmd_KeepPositionEular;
    } 
		else if (rt_memcmp(CTRL_CMD_KEEP_SPEED, cmd, strlen(CTRL_CMD_KEEP_SPEED)) == 0){
        cmdCode = Cmd_KeepSpeed;
    } 
    else if (rt_memcmp(CTRL_CMD_SET_MODE, cmd, strlen(CTRL_CMD_SET_MODE)) == 0){
        if (!param1){
            cmdCode = Cmd_KeepPositionCurrent;
        }
        else if (rt_memcmp(CTRL_CMD_SET_MODE_CURRENT_POSITION, param1, strlen(CTRL_CMD_SET_MODE_CURRENT_POSITION)) == 0){
            cmdCode = Cmd_KeepPositionCurrent;
        } 
        else if (rt_memcmp(CTRL_CMD_SET_MODE_CURRENT_EULAR, param1, strlen(CTRL_CMD_SET_MODE_CURRENT_EULAR)) == 0){
            cmdCode = Cmd_KeepPositionEularCurrent;
        } 
    } 
    else if (rt_memcmp(CTRL_CMD_TRIGGER_LOKER, cmd, strlen(CTRL_CMD_TRIGGER_LOKER)) == 0){
        cmdCode = Cmd_SetTriggerLocker;
    }
    else if (rt_memcmp(CTRL_CMD_TRIGGER_SHOOT, cmd, strlen(CTRL_CMD_TRIGGER_SHOOT)) == 0){
        cmdCode = Cmd_TriggerShoot;
    }
    else if (rt_memcmp(CTRL_CMD_TRIGGER_LOCKER_STATE, cmd, strlen(CTRL_CMD_TRIGGER_LOCKER_STATE)) == 0){
        cmdCode = Cmd_TriggerLockerState;
    }
    
    /*else if (rt_memcmp(CTRL_CMD_KEEP_CURRENT_POSITION, cmd, strlen(CTRL_CMD_KEEP_EULAR)) == 0){
        cmdCode = Cmd_KeepPositionCurrent;
    } 
    else if (rt_memcmp(CTRL_CMD_KEEP_CURRENT_EULAR, cmd, strlen(CTRL_CMD_KEEP_EULAR)) == 0){
        cmdCode = Cmd_KeepPositionEularCurrent;
    } */   
    return cmdCode;
}
static void reset_motor(void){
    g_speed.pitch = 0;
    g_speed.roll = 0;
    g_speed.yaw = 0;    
    g_position.pitch = 0;
    g_position.roll = 0;
    g_position.yaw = 0;
    g_command = Cmd_KeepPositionCurrent;//Cmd_KeepSpeed;
    g_trigger_locker = RT_TRUE;
    g_trigger_command = Trigger_Idle;
}
void app_server_entry(void* parameter){
    int sockServer = -1, sockClient = -1;
    char cmd[CTRL_CMD_MAX];
    char* param[CTRL_CMD_PARAM_MAX];
    rt_tick_t tickReceive = 0;
    
    while (!netdev_get_first_by_flags(NETIF_FLAG_LINK_UP)) 
            rt_thread_mdelay(100);
    
    while (1){
				rt_pin_write(APP_PIN, PIN_LOW);
        if (sockClient == -1){
            //if (sockServer != -1) closesocket(sockServer);
            if (sockServer == -1)  sockServer = MakeListener(CTRL_SERVER_PORT);
            if (sockServer == -1) rt_thread_mdelay(1000);
            else{
                int ret = WaitForClient(sockServer, 3000);
                if (ret == 0) continue;
                else if (ret > 0) {
                    sockClient = AcceptConnector(sockServer);
                    tickReceive = 0;
                }            
                closesocket(sockServer);
                sockServer = -1;
            }
        }
        else {
           int rbytes = ReceiveCommand(sockClient, cmd);
					  rt_pin_write(APP_PIN, PIN_HIGH);

           if (rbytes < 0) {
               closesocket(sockClient);
               sockClient = -1;
               reset_motor();
           }
           else if (rbytes > 0){    
               rt_kprintf("cmd:[%s]\n", cmd);
               int cnt = AnalyseCommand(cmd, param);
               if (cnt == 0) continue;
               int cmdCode = GetCommand(param[0], param[1]);
               rt_bool_t bOK = RT_FALSE;
               rt_bool_t bReturn = RT_TRUE;
               tickReceive = rt_tick_get();
               switch(cmdCode){                   
                   case Cmd_KeepPosition:
                       if (cnt == 4){
                           g_position.pitch = atof(param[1]);
                           g_position.roll = atof(param[2]);
                           g_position.yaw = atof(param[3]);
                           g_command = Cmd_KeepPosition;
                       }
                       else  g_command = Cmd_KeepPositionCurrent;
                       bOK = RT_TRUE;
                       break;
                       
                   case Cmd_KeepPositionEular:
                       if (cnt == 4){
                           g_position.pitch = atof(param[1]);
                           g_position.roll = atof(param[2]);
                           g_position.yaw = atof(param[3]);
                           g_command = Cmd_KeepPosition;
                       }
                       else  g_command = Cmd_KeepPositionEularCurrent;
                       bOK = RT_TRUE;
                       break;
                   case Cmd_KeepSpeed:
                       if (cnt == 4){
                           g_speed.pitch = atof(param[1]);
                           g_speed.roll = atof(param[2]);
                           g_speed.yaw = atof(param[3]);
                           g_command = Cmd_KeepSpeed;
                       }
                       else  g_command = Cmd_KeepPositionCurrent;
                       bOK = RT_TRUE;
                       break;   
                   case Cmd_KeepPositionCurrent:     
                       g_command = Cmd_KeepPositionCurrent;
                       bOK = RT_TRUE; 
                       break;
                   case Cmd_KeepPositionEularCurrent:
                       g_command = Cmd_KeepPositionEularCurrent;
                       bOK = RT_TRUE; 
                       break;
                   case Cmd_SetTriggerLocker:
										 if ((cnt == 2) && (g_trigger_command == Trigger_Idle)){
												 if (atoi(param[1]) != 0) g_trigger_command = Trigger_LockerActive;
												 else g_trigger_command = Trigger_LockerInactive;
	                       bOK = RT_TRUE; 
											}
                       break;
                   case Cmd_TriggerShoot:
										 if ((cnt == 2) && (g_trigger_command == Trigger_Idle) &&(!g_trigger_locker)){
												int8_t trigger_position = atoi(param[1]);
												if (trigger_position < 0) trigger_position = 0;
												else if (trigger_position > 100)  trigger_position = 100;
												g_trigger_position = trigger_position;
												g_trigger_command = Trigger_Shoot;
	                      bOK = RT_TRUE; 
											}
                       break;
                   case Cmd_TriggerLockerState:
                       bReturn = RT_FALSE;
                       rt_snprintf(cmd, CTRL_CMD_MAX, "%s%s,%d\n",g_trigger_locker?1:0);
                       break;
                   default:
                       break;
               }
               if (bReturn)
                    rt_snprintf(cmd, CTRL_CMD_MAX, "%s%s,%s\n",CTRL_CMD_HEADER,param[0],(bOK?CTRL_RET_SUCCESS:CTRL_RET_FAILED));
               send(sockClient, cmd, rt_strlen(cmd), 0);
               rt_kprintf("ret:[%s]\n", cmd);

           } 
           if (tickReceive != 0){
                if ((rt_tick_get() -  tickReceive) > 300){
                    //reset_motor();
                }
           }
           
        }    
    }    
}

int app_server(void){
    rt_thread_t tid = rt_thread_create("sctrl",app_server_entry, RT_NULL, 2048, 12, RT_TICK_PER_SECOND/10);
    if (tid) rt_thread_startup(tid);
    return 0;
}
INIT_APP_EXPORT(app_server);
#endif
