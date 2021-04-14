#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <drv_foc.h>
#include <drv_hall.h>
#include <app_can.h>
#include <app_speed.h>
#include <global_define.h>

//#define DRV_DEBUG
#define LOG_TAG             "app.can"
#include <drv_log.h>

/**************
CAN接收处理线程
************/
#define MOTOR_CAN_PITCH_ID 'P'//'Y'//'P'//'R'
#define MOTOR_CAN_YAW_ID 'Y'//'Y'//'P'//'R'

#define TARGET_CAN_ID 'C'

#define MOTOR_ADDR1_PIN        GET_PIN(B, 15)
#define MOTOR_ADDR2_PIN        GET_PIN(C, 9)
///////////////////////////////////////////////////////////////////////
static struct rt_semaphore sem_can;
///////////////////////////////////////////////////////////////////////
void can_addrpad_init(void){
	static rt_bool_t init = RT_FALSE;
	if (!init){
		rt_pin_mode(MOTOR_ADDR1_PIN, PIN_MODE_INPUT_PULLUP);
		rt_pin_mode(MOTOR_ADDR2_PIN, PIN_MODE_INPUT_PULLUP);	
		init = RT_TRUE;
	}
}
rt_uint8_t  can_addrpad_value(void){
	rt_uint8_t value = 0;
	if (rt_pin_read(MOTOR_ADDR1_PIN)) value |=0x01;
	if (rt_pin_read(MOTOR_ADDR2_PIN)) value |=0x02;
	return value;	
}
///////////////////////////////////////////////////////////////////////
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size){
    rt_sem_release(&sem_can);
    return RT_EOK;
}
///////////////////////////////////////////////////////////////////////
static rt_bool_t PwmEnable(rt_bool_t enable){
    rt_err_t err = RT_EOK;
    rt_event_t evt = (rt_event_t) rt_object_find(SPEED_EVT_NAME, RT_Object_Class_Event);
    if (enable) err = rt_event_send(evt, SPEED_CONTROL_OPEN_MOTOR); 
    else err = rt_event_send(evt, SPEED_CONTROL_CLOSE_MOTOR); 
    return err == RT_EOK;
}
static rt_bool_t PwmDrive(float speed){
    rt_mailbox_t mb =     (rt_mailbox_t)rt_object_find("mbspeed", RT_Object_Class_MailBox);
    return rt_mb_send(mb, *(rt_ubase_t*)&speed);    
}
static uint8_t GetMoterCanId(void){
	rt_uint8_t pad;
	can_addrpad_init();
	pad = can_addrpad_value();	
	if (pad == 0) return MOTOR_CAN_YAW_ID; 
	if (pad == 3) return MOTOR_CAN_PITCH_ID;
	LOG_E("PB15 is %d and PC9 is %d", pad & 0x01, (pad >> 1) & 0x01);
	RT_ASSERT(0);
	return 0xFF;
}
///////////////////////////////////////////////////////////////////////
void can_system_init(void){
	static rt_bool_t init = RT_FALSE;
	can_addrpad_init();
	if (!init){
		init = RT_TRUE;
		rt_device_t can_dev = rt_device_find("can1");  
		uint8_t MOTOR_CAN_ID = GetMoterCanId();	
    struct rt_can_filter_item items[1] ={
        RT_CAN_FILTER_STD_INIT(MOTOR_CAN_ID, RT_NULL, RT_NULL),
    };    
    struct rt_can_filter_config cfg = {1, 1, items};		
    rt_kprintf("RWS Can Addr: %c\n", MOTOR_CAN_ID);
    RT_ASSERT(can_dev);
		
    rt_sem_init(&sem_can, "semcan", 0, RT_IPC_FLAG_FIFO);     
		
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud);//CAN1MBaud
    rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);	
		rt_device_set_rx_indicate(can_dev, can_rx_call);
	}	
}
void can_receive_frame(rt_device_t dev, struct rt_can_msg* rxmsg){
	rxmsg->hdr = -1;
	rt_sem_take(&sem_can, RT_WAITING_FOREVER);
	rt_device_read(dev, 0, rxmsg, sizeof(struct rt_can_msg));	
}

void can_rx_entry(void* param){	
	rt_device_t can_dev = rt_device_find("can1");  
  struct rt_can_msg rxmsg = {0};  
	
	rt_bool_t bPwmOpened = RT_FALSE;
	float speed = 0.f;
  float currAngle;	
	can_system_init();       
  while (1){ 
        struct rt_can_msg txmsg = {
        .id = TARGET_CAN_ID,           
        .ide = RT_CAN_STDID, 
        .rtr = RT_CAN_DTR,  
				};  
				can_receive_frame(can_dev, &rxmsg);
        txmsg.data[0] = rxmsg.data[0];
        txmsg.data[1] = rxmsg.data[1];
        switch(rxmsg.data[0]){
            case 'D':            
               rt_memcpy(&speed, rxmsg.data+2, 4);               
               txmsg.len = 3;
               if (bPwmOpened){                  
                    if (PwmDrive(speed))
                        txmsg.data[2] = 'S';   
                    else                
                        txmsg.data[2] = 'F';
               }
               else {
                   txmsg.data[2] = 'F';
               }
               break;
            case 'E':
                currAngle = hall_fast_get_angle();
                rt_memcpy(txmsg.data+2, &currAngle, 4);
                txmsg.len = 6;
                break;
            case 'P':
                if ((bPwmOpened && (rxmsg.data[2]!=0))
                    ||(!bPwmOpened && (rxmsg.data[2]==0)))
                    txmsg.data[2] = 'F';
                else{
                    if (!PwmEnable(rxmsg.data[2]!=0))  txmsg.data[3] = 'F';
                    else{
                    
                        bPwmOpened = !bPwmOpened;
                        txmsg.data[2] = 'S';                      
                    }
                }
                txmsg.len = 3;
                break;
            default:
                txmsg.len = 3;
                txmsg.data[2] = 'U';
                break;//receive unkown command
        }

        rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));     
        LOG_D("To[%02X] : %02X %02X %02X %02X %02X %02X %02X %02X"
            , rxmsg.id, rxmsg.data[0], rxmsg.data[1], rxmsg.data[2], rxmsg.data[3]
            , rxmsg.data[4], rxmsg.data[5], rxmsg.data[6], rxmsg.data[7]);
    }      
}
int app_can(void){
	if (g_init){
    rt_thread_t tid = rt_thread_create("cancmd",can_rx_entry, RT_NULL, 4096, 15, RT_TICK_PER_SECOND/10);
    if (tid) return rt_thread_startup(tid);
	}
	return -RT_ERROR;
}
INIT_APP_EXPORT(app_can);

