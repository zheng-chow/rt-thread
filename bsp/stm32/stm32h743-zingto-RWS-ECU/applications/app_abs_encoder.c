#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>
#include <app_abs_encoder.h>
#define DRV_DEBUG
#define LOG_TAG             "app.encoder"
#include <drv_log.h>

#define DE_NRE_PIN                          GET_PIN(D, 4)
#define RE485_ENTER_RECEIVE_MODE()          rt_pin_write(DE_NRE_PIN, PIN_LOW);      
#define RE485_LEAVE_RECEIVE_MODE()          rt_pin_write(DE_NRE_PIN, PIN_HIGH);      
#define ENCODER_MODBUS_ADDR                 0x01
#define ABS_ENCODER_READ_FREQ               250
#define ABS_ENCODER_CIRCLE_POINTS           1024
#define RS485_READ_EVT_NAME                 "evt485"

static struct rt_semaphore 	sem_rs485;
static struct rt_mutex      mt_rs485;
static struct rt_event      evt_rs485;

static volatile int16_t rs485_encoder = 0x7FFF;
static rt_bool_t encrst = RT_FALSE;
static float rs485_speed = 0;
static rt_tick_t rs485_tick = 0;
static struct rt_mutex  rs485_loker;

float get_speed_from_encoder(void){
	return rs485_speed;
}
rt_bool_t encoder_read(int16_t* encoder){
	if (rs485_encoder == 0x7FFF) return RT_FALSE;
	rt_mutex_take(&rs485_loker,RT_WAITING_FOREVER);
	//rt_memcpy(encoder, (void*)&rs485_encoder, 2);
	*encoder = rs485_encoder;
	RT_ASSERT(*encoder ==  rs485_encoder);
	//LOG_I("%d <-> %d",rs485_encoder,*encoder);
	rt_mutex_release(&rs485_loker);
	return RT_TRUE;
}
rt_bool_t encoder_reset(void){
	encrst = RT_TRUE;
  rt_event_send(&evt_rs485, 0x02);
	return !encrst;
}	
//rt_bool_t can_encoder_reset(void);
static rt_err_t rs485_tx_done(rt_device_t dev,void *buffer){
	RE485_ENTER_RECEIVE_MODE();
	return RT_EOK;
}
static rt_err_t rs485_rx_call(rt_device_t dev, rt_size_t size){
    rt_sem_release(&sem_rs485);
    return RT_EOK;
}
static rt_err_t encoder_timeout_call(rt_device_t dev, rt_size_t size){
    rt_event_send(&evt_rs485, 0x01);
    return RT_EOK;
}



static rt_uint16_t modbus_crc(rt_uint8_t* buffer, rt_uint8_t buflen){
	rt_uint16_t crc = 0xFFFF;
	for(int n = 0; n < buflen; n++){
		crc = buffer[n] ^ crc;
		for(int i = 0;i < 8;i++){  
			if(crc & 0x01){
				crc = crc >> 1;
				crc = crc ^ 0xa001;
			}   
			else{
				crc = crc >> 1;
			}   
		}  
	} 
	return crc;
}

static rt_uint8_t encoder_make_read(rt_uint8_t* buffer, rt_uint16_t reg){
	rt_uint16_t crc;
	buffer[0] = ENCODER_MODBUS_ADDR;
	buffer[1] = 0x03;
	buffer[2] = reg >> 8;
	buffer[3] = reg;
	buffer[4] = 0;
	buffer[5] = 1;
	crc = modbus_crc(buffer, 6);
	buffer[6] = crc;
	buffer[7] = crc>>8;
	return 8;
}
static rt_uint8_t encoder_make_write(rt_uint8_t* buffer, rt_uint16_t reg, rt_uint16_t val){
	rt_uint16_t crc;
	buffer[0] = ENCODER_MODBUS_ADDR;
	buffer[1] = 0x06;
	buffer[2] = reg >> 8;
	buffer[3] = reg;
	buffer[4] = val >> 8;
	buffer[5] = val;
	crc = modbus_crc(buffer, 6);
	buffer[6] = crc;
	buffer[7] = crc>>8;
	return 8;
}
static rt_uint8_t encoder_make_multi_write(rt_uint8_t* buffer, rt_uint16_t reg, rt_uint16_t* val, rt_uint8_t valcnt){
	rt_uint16_t crc;
	buffer[0] = ENCODER_MODBUS_ADDR;
	buffer[1] = 0x10;
	buffer[2] = reg >> 8;
	buffer[3] = reg;	
	buffer[4] = valcnt >> 8;
	buffer[5] = valcnt;
	buffer[6] = valcnt*2;
	for (rt_uint8_t n = 0; n < valcnt; n++){
		buffer[7 + n * 2] = val[n] >> 8;
		buffer[7 + n * 2 + 1] = val[n];
	}
	crc = modbus_crc(buffer, 7 + valcnt*2);
	buffer[7 + valcnt*2] = crc;
	buffer[8 + valcnt*2] = crc>>8;
	return 9 + valcnt*2;
}

static rt_int8_t encoder_receive_frame(rt_device_t dev, rt_uint8_t* buffer, rt_uint8_t cmd, rt_int32_t timeout){
	rt_uint32_t offs = 0;
	rt_uint8_t  total = 0;
	while (1){
		rt_err_t err = RT_EOK;
		if (offs){
			err =  rt_sem_take(&sem_rs485, 3);
		}else{
			err =  rt_sem_take(&sem_rs485, timeout);
		}
		if (err != RT_EOK){
			rt_kprintf("encoder receive timeout, wait %d", timeout);
			return -1;
		}
		rt_device_read(dev,0,buffer+offs,1);
		if ((offs == 0) && (buffer[0] != ENCODER_MODBUS_ADDR)) goto process_err;	
		else if (offs == 3){//4B有效，可判断总字节数
			if (buffer[1] == 0x03){
				total = 5 + buffer[2];
			}
			else if ((buffer[1] == 0x06) || (buffer[1] == 0x10)){
				total = 8;
			}		
			else if (buffer[1] == 0x83){
				total = 5;
			}
			else{
				LOG_E("Unknown command response %02X", buffer[1]);
				goto process_err;
			}
		}
		offs++;
		if ((total > 0) && (total == offs)){
				rt_uint16_t crc = modbus_crc(buffer, total - 2);
				if ((buffer[total-1] != (crc>>8)) 
					|| (buffer[total-2] != (crc & 0xFF))){
						LOG_E("crc error, receive %02X%02X but calculate %04X", buffer[total-2],buffer[total-1], crc);
					goto process_err;
				}
				else{
					if (cmd != buffer[1]){
						LOG_D("command err, receive %02X but wait %02X", buffer[1], cmd);
					}		
					else break;
				}
		}
		continue;
process_err:
		LOG_E("error frame, offs = %d", offs);
		offs = 0;
		total = 0;
	}
	return total;

}
static rt_uint32_t encoder_read_current(rt_device_t dev){
	rt_uint8_t buffer[128];
	rt_int8_t len = encoder_make_read(buffer, 0);
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	rt_device_write(dev, 0, buffer, len);
	RE485_ENTER_RECEIVE_MODE();
	//LOG_HEX("read", 16, buffer, len);
	len = encoder_receive_frame(dev,buffer, 0x03,2000);
	rt_mutex_release(&mt_rs485);
	if (len != 7) {
		LOG_E("encoder read failed, return %d", len);
		return RT_UINT32_MAX;
	}
	return buffer[3] * 256UL + buffer[4];
}
static rt_bool_t encoder_read_current2(rt_device_t dev, uint32_t* value){
	rt_uint8_t buffer[128];
	rt_int8_t len = 0;
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	len = encoder_make_read(buffer, 0);
	rt_device_write(dev, 0, buffer, len);
	RE485_ENTER_RECEIVE_MODE();
	//LOG_HEX("read", 16, buffer, len);
	rt_memset(buffer, 0, 128);
	len = encoder_receive_frame(dev,buffer, 0x03,2000);
	rt_mutex_release(&mt_rs485);
	if (len != 7) {
		LOG_E("encoder read failed, return %d", len);
		return RT_FALSE;
	}
	*value = buffer[3] * 256UL + buffer[4];
	return RT_TRUE;
}
static rt_bool_t encoder_set_zero(rt_device_t dev){
	rt_uint8_t buffer[128];
	rt_int8_t len = encoder_make_write(buffer, 0x0D,1);
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	rt_device_write(dev, 0, buffer, len);
	RE485_ENTER_RECEIVE_MODE();
	len = encoder_receive_frame(dev,buffer, 0x06,200);
	rt_mutex_release(&mt_rs485);
	return len == 8;

}

static rt_bool_t encoder_set_baudrate(rt_device_t dev, rt_uint32_t baudrate){
	rt_uint8_t buffer[128];
	rt_uint16_t value[2];
	value[0] = baudrate>>16;
	value[1] = baudrate;
	rt_int8_t len = encoder_make_multi_write(buffer, 0x03, value, 2);
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	rt_device_write(dev, 0, buffer, len);
	RE485_ENTER_RECEIVE_MODE();
	len = encoder_receive_frame(dev,buffer, 0x10,200);
	rt_mutex_release(&mt_rs485);
	return len == 8;
}

static void encoder_init(void){
	static rt_bool_t init = RT_FALSE;
	rt_device_t rs485_dev = rt_device_find("uart2");  
	if(!init && rs485_dev){
		init = RT_TRUE;
		rt_sem_init(&sem_rs485, "sem485", 0, RT_IPC_FLAG_FIFO);     
		rt_mutex_init(&mt_rs485, "mt485", RT_IPC_FLAG_FIFO); 	
		rt_device_set_tx_complete(rs485_dev, rs485_tx_done);
		rt_device_set_rx_indicate(rs485_dev, rs485_rx_call);		
		rt_pin_mode(DE_NRE_PIN, PIN_MODE_OUTPUT);
		RE485_ENTER_RECEIVE_MODE();    
	}
}

void encoder_rx_entry(void* param){
	rt_device_t rs485_dev = rt_device_find("uart2");  
  rt_device_t tim_dev = rt_device_find("timer3");    
	struct serial_configure rs485_config = RT_SERIAL_CONFIG_DEFAULT;
	rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD; 
  rt_hwtimerval_t timeout_s;
  uint32_t evt;
	rt_uint32_t encoder = RT_UINT32_MAX;
	rt_tick_t		encoder_tick = 0;
	RT_ASSERT(rs485_dev);
	RT_ASSERT(tim_dev);
    
	encoder_init();
	rt_event_init(&evt_rs485, RS485_READ_EVT_NAME, RT_IPC_FLAG_FIFO);
	rt_mutex_init(&rs485_loker, "mt485v",RT_IPC_FLAG_PRIO); 	//RT_IPC_FLAG_FIFO
	
	rs485_config.baud_rate = BAUD_RATE_115200;
	rs485_config.data_bits = DATA_BITS_8;     
	rs485_config.stop_bits = STOP_BITS_2;        
	rs485_config.bufsz     = 128;              
	rs485_config.parity    = PARITY_NONE;   
	rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);	
	rt_device_open(rs485_dev, RT_DEVICE_FLAG_INT_RX);//|RT_DEVICE_FLAG_INT_TX
	
	
  rt_device_set_rx_indicate(tim_dev, encoder_timeout_call);
	rt_device_open(tim_dev, RT_DEVICE_OFLAG_RDWR);
  mode = HWTIMER_MODE_PERIOD;
  rt_device_control(tim_dev, HWTIMER_CTRL_MODE_SET, &mode);
  timeout_s.sec = 0;     
  timeout_s.usec = 1000000/ABS_ENCODER_READ_FREQ;    
  rt_device_write(tim_dev, 0, &timeout_s, sizeof(timeout_s));	
  rt_event_recv(&evt_rs485, 0x01, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
	while (RT_UINT32_MAX == encoder){
		rt_event_recv(&evt_rs485, 0x01, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
		rs485_tick = rt_tick_get(); 
		encoder = encoder_read_current(rs485_dev);
	}
	encoder_tick = 0;
	while (1){
		rt_uint32_t encoder_current;
		rt_tick_t tick_current;
		rt_bool_t read_ret;
			if (RT_EOK == rt_event_recv(&evt_rs485, 0xFF, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_TICK_PER_SECOND, &evt)){//RT_WAITING_FOREVER
			if (evt & 0x01){
				tick_current = rt_tick_get();
				read_ret = encoder_read_current2(rs485_dev,&encoder_current);
				encoder_tick += RT_TICK_PER_SECOND / ABS_ENCODER_READ_FREQ;
				if (read_ret){
					float deta = 0;
					if (encoder_current > encoder){
						deta = encoder_current - encoder;
					}
					else{
						deta = ABS_ENCODER_CIRCLE_POINTS + encoder - encoder_current;
					}
					deta = deta *360 / (ABS_ENCODER_CIRCLE_POINTS-1) * RT_TICK_PER_SECOND/encoder_tick;
					rs485_speed = deta;
					rs485_tick = tick_current;
					encoder = encoder_current;
					encoder_tick = 0;
					rt_mutex_take(&rs485_loker,RT_WAITING_FOREVER);
					if (encoder_current >= 512)   encoder_current = encoder_current - 1024 ;					
					rs485_encoder = encoder_current;
					if ((rs485_encoder > 512) || (rs485_encoder < -512))  LOG_W("encoder is %d %d", rs485_encoder, (int)(-rs485_encoder*360.0/1024.0));
					rt_mutex_release(&rs485_loker);			
				}
			}
			if (evt & 0x02){
				if (encrst){
					encrst = !encoder_set_zero(rs485_dev);
				}
			}
		}
		else{
			LOG_E("encoder timeout");
		}
	}
	
	
	
	
	
	
}

int app_abs_encoder(void){
    rt_thread_t tid = rt_thread_create("encoder",encoder_rx_entry, RT_NULL, 2048, 11, RT_TICK_PER_SECOND/10);
    if (tid) return rt_thread_startup(tid);
    return -RT_ERROR;
}
INIT_APP_EXPORT(app_abs_encoder);

static void enc_read(uint8_t argc, char **argv){
	rt_device_t rs485_dev = rt_device_find("uart2");  
	encoder_init();
	rt_device_open(rs485_dev, RT_DEVICE_FLAG_INT_RX);
	if (argc > 1){
		struct serial_configure rs485_config = RT_SERIAL_CONFIG_DEFAULT;
		rt_uint32_t current = atoi(argv[1]);
		encoder_init();
		rs485_config.baud_rate = current;
		rs485_config.data_bits = DATA_BITS_8;     
		rs485_config.stop_bits = STOP_BITS_2;        
		rs485_config.parity    = PARITY_NONE;   
		rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);		
	}
	
	rt_uint32_t encoder = encoder_read_current(rs485_dev);
	if (RT_UINT32_MAX != encoder){
		int enc_val = 1024 -  encoder;
		if (enc_val > 512){
			enc_val = enc_val - 1024;
		}
		LOG_I("encoder is %d", enc_val);
	}
	else LOG_E("encoder read failed");
	rt_device_close(rs485_dev);
}
FINSH_FUNCTION_EXPORT_ALIAS(enc_read, __cmd_enc_read, read encoder);
static void enc_szero(uint8_t argc, char **argv){
	rt_device_t rs485_dev = rt_device_find("uart2");  
	encoder_init();
	rt_device_open(rs485_dev, RT_DEVICE_FLAG_INT_RX);
	if (argc > 1){
		struct serial_configure rs485_config = RT_SERIAL_CONFIG_DEFAULT;
		rt_uint32_t current = atoi(argv[1]);
		encoder_init();
		rs485_config.baud_rate = current;
		rs485_config.data_bits = DATA_BITS_8;     
		rs485_config.stop_bits = STOP_BITS_2;        
		rs485_config.parity    = PARITY_NONE;   
		rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);		
	}
	if (encoder_set_zero(rs485_dev)) LOG_I("encoder set zero finished");
	else LOG_E("encoder set zero failed");
	rt_device_close(rs485_dev);
}
FINSH_FUNCTION_EXPORT_ALIAS(enc_szero, __cmd_enc_szero, set encoder position to zero);
static void enc_sbaudrate(uint8_t argc, char **argv){
	if (argc != 3){
		LOG_E("command formate: enc_sbaudrate [current] [target]");
	}
	else{
		rt_device_t rs485_dev = rt_device_find("uart2");  
		struct serial_configure rs485_config = RT_SERIAL_CONFIG_DEFAULT;
		rt_uint32_t current = atoi(argv[1]);
		rt_uint32_t target = atoi(argv[2]);
		encoder_init();
		rs485_config.baud_rate = current;
		rs485_config.data_bits = DATA_BITS_8;     
		rs485_config.stop_bits = STOP_BITS_2;        
		rs485_config.parity    = PARITY_NONE;   
		rt_device_open(rs485_dev, RT_DEVICE_FLAG_INT_RX);
		rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);		
		if (encoder_set_baudrate(rs485_dev, target)) LOG_I("encoder set baudrate finished");
		else LOG_E("encoder set baudrate failed");
		rt_device_close(rs485_dev);
	}
}
FINSH_FUNCTION_EXPORT_ALIAS(enc_sbaudrate, __cmd_enc_sbaudrate, enc_sbaudrate 9600 115200);




