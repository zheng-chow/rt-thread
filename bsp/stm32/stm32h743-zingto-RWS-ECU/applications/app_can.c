#include <rtthread.h>
#include <rtdevice.h>
#include "app_can.h"
#include "app_abs_encoder.h"
#define DRV_DEBUG
#define LOG_TAG             "app.can"
#include <drv_log.h>
//#include <math.h>
#define MOTOR_PITCH_CAN_ID  'P'
#define MOTOR_ROLL_CAN_ID   'R'
#define MOTOR_YAW_CAN_ID    'Y'
#define CONTROL_CAN_ID      'C'
#define IMU_CAN_ID          'I'
#define TRIGGER_CAN_ID      'T'
#define ENCODER_YAW_CAN_ID  0x02

static rt_device_t can_dev = RT_NULL;
static struct rt_semaphore  can_sem;

static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size){
    rt_sem_release(&can_sem);
    return RT_EOK;
}
static rt_bool_t can_get_motor_addr(uint8_t motor_id, uint32_t* addr){
    switch(motor_id){
        case 1: *addr = MOTOR_PITCH_CAN_ID; break;
        case 2: return RT_FALSE;
        case 3: *addr = MOTOR_YAW_CAN_ID; break;
        default: return RT_FALSE;
    }   
    return RT_TRUE;
}
static rt_bool_t can_send_cmd(uint32_t id, uint8_t cmd){
    struct rt_can_msg txmsg = {0};  
    if (!can_dev) return RT_FALSE;
    txmsg.id = id;   
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 1;               
    txmsg.data[0] = cmd;
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));
    return RT_TRUE;
}
static rt_bool_t can_send_cmd_with_parameter(uint32_t id, uint8_t cmd, uint8_t idx, float parameter){
    struct rt_can_msg txmsg = {0};  
    if (!can_dev) return RT_FALSE;
    txmsg.id = id;   
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 6;               
    txmsg.data[0] = cmd;
    txmsg.data[1] = idx;
    rt_memcpy(txmsg.data+2, &parameter, 4);
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));
    return RT_TRUE;
}
static rt_bool_t can_recv_msg(struct rt_can_msg* rxmsg, int32_t timeout){
    int32_t to = timeout;
    rt_tick_t tickline = rt_tick_get() + timeout;
    if (!can_dev) return RT_FALSE;
    do{
        if (timeout >= 0){
            rt_tick_t tick= rt_tick_get();
            if (tickline <= tick) return RT_FALSE;
            to = tickline - tick;
            if (to <= 0) return RT_FALSE;
        }
        if (RT_EOK != rt_sem_take(&can_sem, to)) return RT_FALSE;
        break;
    }while (1);
    rxmsg->hdr = -1;
    rt_device_read(can_dev, 0, rxmsg, sizeof(*rxmsg));
    return RT_TRUE;
}
static rt_bool_t can_recv_cmd(struct rt_can_msg* rxmsg, uint8_t cmd, int32_t timeout){
    int32_t to = timeout;
    rt_tick_t tickline = rt_tick_get() + timeout;
    if (!can_dev) return RT_FALSE;
    do{

        do{
            if (timeout >= 0){
                rt_tick_t tick= rt_tick_get();
                if (tickline <= tick) return RT_FALSE;
                to = tickline - tick;
                if (to <= 0) return RT_FALSE;
            }
            if (RT_EOK != rt_sem_take(&can_sem, to)) return RT_FALSE;
            break;
        }while (1);
        rxmsg->hdr = -1;
        rt_device_read(can_dev, 0, rxmsg, sizeof(*rxmsg));

        
    }while (rxmsg->data[0] != cmd);
    return rxmsg->data[0] == cmd;
}
rt_bool_t can_test_send(void){
    struct rt_can_msg txmsg = {0};  
    if (!can_dev) return RT_FALSE;
    txmsg.id = 0x70;             
    txmsg.ide = RT_CAN_STDID;   
    txmsg.rtr = RT_CAN_DTR;      
    txmsg.len = 8;                
    txmsg.data[0] = 0x00;
    txmsg.data[1] = 0x11;
    txmsg.data[2] = 0x22;
    txmsg.data[3] = 0x33;
    txmsg.data[4] = 0x44;
    txmsg.data[5] = 0x55;
    txmsg.data[6] = 0x66;
    txmsg.data[7] = 0x77;
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));
    return RT_TRUE;
}
rt_bool_t can_test_receive(struct rt_can_msg* rxmsg, int32_t timeout){
    int32_t to = timeout;
    rt_tick_t tickline = rt_tick_get() + timeout;
    if (!can_dev) return RT_FALSE;
       
    if (timeout >= 0){
        rt_tick_t tick= rt_tick_get();
        if (tickline <= tick) return RT_FALSE;
        to = tickline - tick;
        if (to <= 0) return RT_FALSE;
    }
    if (RT_EOK != rt_sem_take(&can_sem, to)) return RT_FALSE;
   
    rxmsg->hdr = -1;
    rt_device_read(can_dev, 0, rxmsg, sizeof(*rxmsg));
    return RT_TRUE;
  
}
rt_bool_t can_encoder_test(void){
    
    return RT_TRUE;
}
rt_bool_t app_can_init(void){	
    struct rt_can_filter_item items[] ={
        //RT_CAN_FILTER_ITEM_INIT(CONTROL_CAN_ID, 0, 0, 1, 0x01, RT_NULL, RT_NULL),
        RT_CAN_FILTER_STD_INIT(CONTROL_CAN_ID, RT_NULL, RT_NULL),
        RT_CAN_FILTER_STD_INIT(0x01, RT_NULL, RT_NULL),
    };    
    struct rt_can_filter_config cfg = {2, 1, items}; 
    
    can_dev = rt_device_find("fdcan1");  
    rt_sem_init(&can_sem, "cansem", 0, RT_IPC_FLAG_FIFO);
    if (!can_dev) return RT_FALSE;
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud);//CAN1MBaud
    rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    rt_device_set_rx_indicate(can_dev, can_rx_call);
	return RT_TRUE;
}

rt_bool_t can_read_gyro(float* gyro){
    struct rt_can_msg rxmsg = {0};
    uint8_t cmd = 'W';    
    uint8_t flag = 0;
    if (!can_send_cmd(IMU_CAN_ID, cmd)) return RT_FALSE;
   
    while (flag != 0x7){
        uint8_t idx;
        if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)){ 
					LOG_E("read gyro failed, not enough %d", flag);
					return RT_FALSE;
				}
        if (rxmsg.len != 6) return RT_FALSE;
        idx = rxmsg.data[1];
        if ((idx < 1) || (idx > 3)) {
					LOG_E("read gyro failed, out range %d", idx);
					return RT_FALSE;
				}
        idx--;
        flag |= 1<<idx;
        rt_memcpy(gyro+idx, rxmsg.data+2, 4);        
    }
    return RT_TRUE;	
}

rt_bool_t can_read_accel(float* accel){
    struct rt_can_msg rxmsg = {0};
    uint8_t cmd = 'A';    
    uint8_t flag = 0;
    if (!can_send_cmd(IMU_CAN_ID, cmd)) return RT_FALSE;
   
    while (flag != 0x7){
        uint8_t idx;
        if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)){
					LOG_E("read accel failed, not enough %d", flag);
					return RT_FALSE;
				}
        if (rxmsg.len != 6) return RT_FALSE;
        idx = rxmsg.data[1];
        if ((idx < 1) || (idx > 3)){
					LOG_E("read accel failed, out range %d", idx);
					return RT_FALSE;
				}
        idx--;
        flag |= 1<<idx;
        rt_memcpy(accel+idx, rxmsg.data+2, 4);        
    }
    return RT_TRUE;	
}

rt_bool_t can_read_q(float* q){
    struct rt_can_msg rxmsg = {0};
    uint8_t cmd = 'Q';    
    uint8_t flag = 0;
    if (!can_send_cmd(IMU_CAN_ID, cmd)) return RT_FALSE;
   
    while (flag != 0xF){
        uint8_t idx;
        if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)){
					LOG_E("read q failed, not enough %d", flag);
					return RT_FALSE;
				}
        if (rxmsg.len != 6) return RT_FALSE;
        idx = rxmsg.data[1];
        if ((idx < 1) || (idx > 4)){
					LOG_E("read q failed, out range %d", idx);
					return RT_FALSE;
				}
        idx--;
        flag |= 1<<idx;
        rt_memcpy(q+idx, rxmsg.data+2, 4);        
    }
    //q[0] = sqrt(1- q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
    return RT_TRUE;	
}

rt_bool_t can_reset_q(void){
    struct rt_can_msg rxmsg = {0};
    uint8_t cmd = 'R';    
    if (!can_send_cmd(IMU_CAN_ID, cmd)) return RT_FALSE;
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;       
    return rxmsg.data[2] == (rt_uint8_t)('S');   
}

rt_bool_t can_set_pitch(float pitch){
    struct rt_can_msg rxmsg = {0};
    uint8_t cmd = 'E';    
    if (!can_send_cmd_with_parameter(IMU_CAN_ID, cmd, '1', pitch)) return RT_FALSE;
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;       
    return rxmsg.data[2] == (rt_uint8_t)('S');   
}
rt_bool_t can_set_roll(float roll){
    struct rt_can_msg rxmsg = {0};
    uint8_t cmd = 'E';    
    if (!can_send_cmd_with_parameter(IMU_CAN_ID, cmd, '2', roll)) return RT_FALSE;
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;       
    return rxmsg.data[2] == (rt_uint8_t)('S');   
}
rt_bool_t can_set_yaw(float yaw){
    struct rt_can_msg rxmsg = {0};
    uint8_t cmd = 'E';    
    if (!can_send_cmd_with_parameter(IMU_CAN_ID, cmd, '3', yaw)) return RT_FALSE;
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;       
    return rxmsg.data[2] == (rt_uint8_t)('S');   
}
rt_bool_t can_read_distance(float* distance){
    struct rt_can_msg rxmsg = {0};
    uint8_t cmd = 'D';    
    if (!can_send_cmd(IMU_CAN_ID, cmd)) return RT_FALSE;
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 6) return RT_FALSE;
    rt_memcpy(&distance, rxmsg.data+2, 4);        
    return RT_TRUE;	
}
/*
rt_bool_t can_set_motor(uint8_t motor_id,uint16_t mcoeff, float eangle){
    struct rt_can_msg rxmsg = {0};
    struct rt_can_msg txmsg = {0};  
    uint8_t cmd = 'D';    
    uint32_t addr;
    if (!can_dev) return RT_FALSE; 
    if (!can_get_motor_addr(motor_id, &addr)) return RT_FALSE;
    txmsg.id = addr;
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 8;               
    txmsg.data[0] = cmd;
    txmsg.data[1] = 1;
    rt_memcpy(txmsg.data+2,&mcoeff, 2);
    rt_memcpy(txmsg.data+4,&eangle, 4);
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));
    
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;     
    return rxmsg.data[2] == (rt_uint8_t)('S');   
}*/
rt_bool_t can_trigger_set_locker(rt_bool_t enable){
    struct rt_can_msg rxmsg = {0};
    struct rt_can_msg txmsg = {0};  
    uint8_t cmd = 'L';    
    //uint32_t addr;
    if (!can_dev) return RT_FALSE; 
    txmsg.id = TRIGGER_CAN_ID;
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 3;               
    txmsg.data[0] = cmd;
    txmsg.data[1] = 1;
    txmsg.data[2] = (enable?0xFF:0x00);    
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));    
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;     
    return rxmsg.data[2] == (rt_uint8_t)('S');   
}
rt_bool_t can_trigger_shoot(int8_t position){
    struct rt_can_msg rxmsg = {0};
    struct rt_can_msg txmsg = {0};  
    uint8_t cmd = 'P';    
    //uint32_t addr;
    if (!can_dev) return RT_FALSE; 
    txmsg.id = TRIGGER_CAN_ID;
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 2;               
    txmsg.data[0] = cmd;
    txmsg.data[1] = position;
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));    
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;     
    return rxmsg.data[2] == (rt_uint8_t)('S');
}
rt_bool_t can_set_motor(uint8_t motor_id,float speed){
    struct rt_can_msg rxmsg = {0};
    struct rt_can_msg txmsg = {0};  
    uint8_t cmd = 'D';    
    uint32_t addr;
    if (!can_dev) return RT_FALSE; 
    if (!can_get_motor_addr(motor_id, &addr)) return RT_FALSE;
    txmsg.id = addr;
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 6;               
    txmsg.data[0] = cmd;
    txmsg.data[1] = 1;
    rt_memcpy(txmsg.data+2,&speed, 4);
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));
    
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;     
    return rxmsg.data[2] == (rt_uint8_t)('S');   
}
rt_bool_t can_get_eangle(uint8_t motor_id, float* eangle){
    struct rt_can_msg rxmsg = {0};  
    uint32_t addr;
    uint8_t cmd = 'E';
    if (!can_get_motor_addr(motor_id, &addr)) return RT_FALSE;
    if (!can_send_cmd(addr, cmd)) return RT_FALSE;
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 6) return RT_FALSE;   
    rt_memcpy(eangle, rxmsg.data+2, 4);
    return RT_TRUE;
}
rt_bool_t can_motor_enable(uint8_t motor_id,  rt_bool_t enable){
    struct rt_can_msg rxmsg = {0};
    struct rt_can_msg txmsg = {0};  
    uint8_t cmd = 'P';    
    uint32_t addr;
    if (!can_dev) return RT_FALSE; 
    if (!can_get_motor_addr(motor_id, &addr)) return RT_FALSE;
    txmsg.id = addr;
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 3;               
    txmsg.data[0] = cmd;
    txmsg.data[1] = 1;
    txmsg.data[2] = (enable?0xFF:0x00);    
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));    
    if (!can_recv_cmd(&rxmsg, cmd, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if (rxmsg.len != 3) return RT_FALSE;     
    return rxmsg.data[2] == (rt_uint8_t)('S');   
}
rt_bool_t x_can_encoder_read(int16_t* encoder){
    struct rt_can_msg rxmsg = {0};
    struct rt_can_msg txmsg = {0};  
    //uint32_t addr;
    if (!can_dev) return RT_FALSE; 
    txmsg.id = ENCODER_YAW_CAN_ID;
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 4;               
    txmsg.data[0] = txmsg.len;
    txmsg.data[1] = ENCODER_YAW_CAN_ID;
    txmsg.data[2] = 0x01;    
    txmsg.data[3] = 0x00;    
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));    
    if (!can_recv_msg(&rxmsg, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if ((rxmsg.data[0] != 5)||(rxmsg.data[1] != 2)||(rxmsg.data[2] != 1)) return RT_FALSE;
    rt_memcpy(encoder, rxmsg.data+3,2);
    if (*encoder >= 2048)         *encoder = *encoder - 4096 ;
    return RT_TRUE;        
}
rt_bool_t x_can_encoder_reset(void){
    struct rt_can_msg rxmsg = {0};
    struct rt_can_msg txmsg = {0};  
    //uint32_t addr;
    if (!can_dev) return RT_FALSE; 
    txmsg.id = ENCODER_YAW_CAN_ID;
    txmsg.ide = RT_CAN_STDID;  
    txmsg.rtr = RT_CAN_DTR; 
    txmsg.len = 4;               
    txmsg.data[0] = txmsg.len;
    txmsg.data[1] = ENCODER_YAW_CAN_ID;
    txmsg.data[2] = 0x06;    
    txmsg.data[3] = 0x00;    
    rt_device_write(can_dev, 0, &txmsg, sizeof(txmsg));    
    if (!can_recv_msg(&rxmsg, RT_TICK_PER_SECOND/10)) return RT_FALSE;
    if ((rxmsg.data[0] != 4)||(rxmsg.data[1] != 2)||(rxmsg.data[2] != 6)||(rxmsg.data[3] != 0)) return RT_FALSE;
    return RT_TRUE;        
}


