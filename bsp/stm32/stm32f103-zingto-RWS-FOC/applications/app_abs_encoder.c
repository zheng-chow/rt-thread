#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdlib.h>
#include <app_encoder_modbus.h>
#include <app_abs_encoder.h>
//#include <drv_hall.h>
#include <global_define.h>




#define xDRV_DEBUG
#define LOG_TAG             "app.encoder"
#include <drv_log.h>
#define ENCODER_DEFAULT_BAUDRATE 	115200
#define ENCODER_DEFAULT_STOPBIT		STOP_BITS_2
#define ENCODER_DEFUALT_PARITY 		PARITY_NONE

///////////////////////////////////////////////////////////////////////
#define DE_NRE_PIN        GET_PIN(B, 14)
#define RE485_ENTER_RECEIVE_MODE()         do{rt_pin_write(DE_NRE_PIN, PIN_LOW);}while(0)     
#define RE485_LEAVE_RECEIVE_MODE()         do{rt_pin_write(DE_NRE_PIN, PIN_HIGH);} while(0)    

#define ABS_ENCODER_READ_FREQ 		200
#define ENCODER_SCALE	(21 * 360 /1.35 / ABS_ENCODER_CIRCLE_POINTS)
///////////////////////////////////////////////////////////////////////
ALIGN(RT_ALIGN_SIZE)
static rt_uint32_t 						mq_rs485_pool[16];
static struct rt_messagequeue mq_rs485;
static struct rt_event 				evt_rs485_tx_done;
static struct rt_event 			evt_rs485;
static struct rt_mutex 			mt_rs485;
static struct rt_mutex 			mt_rs485_locker;

static int16_t    rs485_encoder = 0;
static float 			rs485_speed = 0;
static rt_tick_t 	rs485_tick = 0;
///////////////////////////////////////////////////////////////////////
rt_bool_t get_angle_from_encoder(float* angle){
	rt_tick_t dTick = (rt_tick_get() - rs485_tick);
	rt_mutex_take(&mt_rs485_locker,RT_WAITING_FOREVER);
	*angle = rs485_encoder * 360.f *21 / ABS_ENCODER_CIRCLE_POINTS;
	rt_mutex_release(&mt_rs485_locker);
	return dTick <= (5*RT_TICK_PER_SECOND / ABS_ENCODER_READ_FREQ);
}
rt_bool_t get_speed_from_encoder(float* speed, int16_t * encoder){
	rt_tick_t dTick = (rt_tick_get() - rs485_tick);
	rt_mutex_take(&mt_rs485_locker,RT_WAITING_FOREVER);
	*speed = rs485_speed;
	*encoder = rs485_encoder;	
	rt_mutex_release(&mt_rs485_locker);
	return dTick <= (5*RT_TICK_PER_SECOND / ABS_ENCODER_READ_FREQ);
}
///////////////////////////////////////////////////////////////////////
static rt_err_t rs485_tx_done(rt_device_t dev,void *buffer){
	RE485_ENTER_RECEIVE_MODE();
	rt_event_send(&evt_rs485_tx_done,0x01);
	return RT_EOK;
}
static rt_err_t rs485_rx_call(rt_device_t dev, rt_size_t size){
	rt_err_t result;
  result = rt_mq_send(&mq_rs485, &size, sizeof(size));
  if ( result == -RT_EFULL) rt_kprintf("message queue full!\n");
	return result;
}
static rt_err_t encoder_timeout_call(rt_device_t dev, rt_size_t size){
    rt_event_send(&evt_rs485, 0x01);
    return RT_EOK;
}
static rt_err_t encoder_wait_for_tx_done(void){
	uint32_t evt;
	rt_err_t err =  rt_event_recv(&evt_rs485_tx_done, 0xFF, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_TICK_PER_SECOND, &evt);
	if (RT_EOK != err)	LOG_E("write timeout, %d", err);
	return err;
}
///////////////////////////////////////////////////////////////////////


static rt_int16_t encoder_receive_frame(rt_device_t dev, rt_uint8_t buffer[128], rt_uint8_t cmd, rt_int32_t timeout){
	rt_bool_t finished = RT_FALSE;
	rt_int16_t total = 0;		
	rt_int16_t offs = 0;
	while (!finished){
		rt_size_t size, rsize;
		int16_t total_sz;
		rt_err_t err = rt_mq_recv(&mq_rs485, &size, sizeof(size), timeout);
		if (err != RT_EOK) {
			LOG_E("receive timeout, error code %d", err);
			return -1;
		}
		if (size == 0) {
			LOG_E("no data to read");
			continue;
		}
		rsize = rt_device_read(dev,0,buffer+offs, size);
		if (rsize == 0){
			LOG_E("no data read, should have %d , offs is %d", size, offs);
			continue;
		}
		else {
			size = rsize;
			LOG_HEX("frm", 16, buffer+offs, size);

		}		

		offs += size;
		if (offs < 5) 
			continue;
		total_sz = encoder_check_cmd(buffer, cmd);
		if (total_sz < 0) {
			LOG_E("encoder_check_cmd return -1");
			offs = 0;
			continue;
		}		
		else if (offs < total_sz) {
			//LOG_D("offs < total_sz");
			continue;
		}
		if (!encoder_response_valid(buffer, offs)) {
			LOG_W("encoder response invalid");
			offs = 0;
			continue;
		}
		
		total = total_sz;
		//total = encoder_check_cmd(buffer, cmd);
		//if (total < 0) continue;
		break;		
	}	
	return total;
}

static rt_bool_t encoder_read_current(rt_device_t dev, rt_int16_t* encoder){
	rt_uint8_t buffer[128];
	rt_int8_t len = 0;
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	len = encoder_make_read(buffer, 0);
	rt_device_write(dev, 0, buffer, len);
	encoder_wait_for_tx_done();
	LOG_HEX("cmd", 16, buffer, len);
	rt_memset(buffer, 0, 128);
	len = encoder_receive_frame(dev, buffer, 0x03, 5000);
	rt_mutex_release(&mt_rs485);
	if (len > 0)	LOG_HEX("rsp", 16, buffer, len);
	if (len != 7) return RT_FALSE;
	*encoder = buffer[3] * 256UL + buffer[4];
	//if (*encoder > ABS_ENCODER_HALF_CIRCLE_POINTS) *encoder = ABS_ENCODER_CIRCLE_POINTS - *encoder;
	//else *encoder = -*encoder;
	*encoder = -*encoder;
	return RT_TRUE;
}
static rt_bool_t encoder_read_current_hw(rt_device_t dev, rt_int16_t* encoder){
	rt_uint8_t buffer[128];
	rt_int8_t len = 0;
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	len = encoder_make_read_single(buffer, 0, 2);
	rt_device_write(dev, 0, buffer, len);
	encoder_wait_for_tx_done();
	LOG_HEX("cmd", 16, buffer, len);
	rt_memset(buffer, 0, 128);
	len = encoder_receive_frame(dev, buffer, 0x03, 5000);
	rt_mutex_release(&mt_rs485);
	if (len > 0)LOG_HEX("rsp", 16, buffer, len);
	if (len != 9) return RT_FALSE;
	*encoder = buffer[3] * 256UL + buffer[4];
	if (*encoder > ABS_ENCODER_HALF_CIRCLE_POINTS) *encoder = ABS_ENCODER_CIRCLE_POINTS - *encoder;
	else *encoder = -*encoder;
	return RT_TRUE;
}
static rt_bool_t encoder_read_uid(rt_device_t dev){
	rt_uint8_t buffer[128];
	rt_int8_t len = 0;
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	len = encoder_make_read_single(buffer, 6, 3);
	rt_device_write(dev, 0, buffer, len);
	encoder_wait_for_tx_done();
	LOG_HEX("cmd", 16, buffer, len);
	rt_memset(buffer, 0, 128);
	len = encoder_receive_frame(dev,buffer, 0x03, 5000);
	rt_mutex_release(&mt_rs485);
	if (len > 0)LOG_HEX("rsp", 16, buffer, len);
	if (len != 11) return RT_FALSE;
	LOG_I("uid: %02X%02X%02X%02X%02X%02X", buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8]);
	return RT_TRUE;
}
static rt_bool_t encoder_set_zero(rt_device_t dev){
	rt_uint8_t buffer[128];
	rt_int8_t len = 0;
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	len = encoder_make_write(buffer, 0x0D,1);
	rt_device_write(dev, 0, buffer, len);
	encoder_wait_for_tx_done();
	rt_memset(buffer, 0, 128);
	len = encoder_receive_frame(dev,buffer, 0x06,200);
	rt_mutex_release(&mt_rs485);
	return len == 8;
}

static rt_bool_t encoder_set_baudrate(rt_device_t dev, rt_uint32_t baudrate){
	rt_uint8_t buffer[128];
	rt_uint16_t value[2];
	value[0] = baudrate>>16;
	value[1] = baudrate;
	rt_int8_t len = 0;
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	len = encoder_make_multi_write(buffer, 0x03, value, 2);
	rt_device_write(dev, 0, buffer, len);
	LOG_HEX("cmd", 16, buffer, len);
	encoder_wait_for_tx_done();
	rt_memset(buffer, 0, 128);
	len = encoder_receive_frame(dev,buffer, 0x10,5000);
	rt_mutex_release(&mt_rs485);
	if (len > 0) 	LOG_HEX("rsp", 16, buffer, len);
	if (len != 8) return RT_FALSE;
	return RT_TRUE;
}
static rt_bool_t encoder_set_baudrate_level(rt_device_t dev, rt_uint32_t baudrate){
	rt_uint8_t buffer[128];
	rt_uint16_t value;
	switch(baudrate){
		case 9600 : value = 0;break;
		case 19200 : value = 1;break;
		case 38400 : value = 2;break;
		case 57600 : value = 3;break;
		case 115200 : value = 4;break;
		default: return RT_FALSE;
	}
	rt_int8_t len = 0;
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	len = encoder_make_write(buffer, 0x05, value);
	rt_device_write(dev, 0, buffer, len);
	LOG_HEX("cmd", 16, buffer, len);
	encoder_wait_for_tx_done();
	rt_memset(buffer, 0, 128);
	len = encoder_receive_frame(dev,buffer, 0x6,5000);
	rt_mutex_release(&mt_rs485);
	if (len > 0) 	LOG_HEX("rsp", 16, buffer, len);
	if (len != 8) return RT_FALSE;
	return RT_TRUE;
}

static void encoder_init(void){
	static rt_bool_t init = RT_FALSE;
	rt_device_t rs485_dev = rt_device_find("uart3");  
	if(!init && rs485_dev){
		init = RT_TRUE;
		rt_mq_init(&mq_rs485, "mq485",
               mq_rs485_pool,            
               sizeof(rt_size_t), 
               sizeof(mq_rs485_pool),       
               RT_IPC_FLAG_FIFO);    
		rt_event_init(&evt_rs485_tx_done, "evt485tx", RT_IPC_FLAG_FIFO);		 
		rt_mutex_init(&mt_rs485, "mt485", RT_IPC_FLAG_FIFO); 		
		rt_mutex_init(&mt_rs485_locker, "mt485lo", RT_IPC_FLAG_PRIO); 		
		rt_device_set_tx_complete(rs485_dev, rs485_tx_done);
		rt_device_set_rx_indicate(rs485_dev, rs485_rx_call);		
		rt_pin_mode(DE_NRE_PIN, PIN_MODE_OUTPUT);
		RE485_ENTER_RECEIVE_MODE();    
	}
}
static void encoder_update_parameter(rt_device_t dev, int baudrate, int stopbit, int parity){
	if (stopbit > 3){ 
		LOG_E("mistake stopbit");
		return ;
	}
	if (parity > 2){ 
		LOG_E("mistake parity");
		return ;
	}	
	struct serial_configure rs485_config = RT_SERIAL_CONFIG_DEFAULT;
	rs485_config.baud_rate = baudrate;
	rs485_config.data_bits = DATA_BITS_8;     
	rs485_config.stop_bits = stopbit;        
	rs485_config.parity    = parity; 
	rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);		
}
static void encoder_init_uart(void){
	static rt_bool_t init = RT_FALSE;
	rt_device_t rs485_dev = rt_device_find("uart3");  
	if(!init && rs485_dev){
		init = RT_TRUE;
		struct serial_configure rs485_config = RT_SERIAL_CONFIG_DEFAULT;
		rs485_config.baud_rate = ENCODER_DEFAULT_BAUDRATE;
		rs485_config.data_bits = DATA_BITS_8;     
		rs485_config.stop_bits = ENCODER_DEFAULT_STOPBIT;        
		rs485_config.parity    = ENCODER_DEFUALT_PARITY; 
		rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);		
		rt_device_open(rs485_dev, RT_DEVICE_FLAG_DMA_RX|RT_DEVICE_FLAG_DMA_TX);
		/*
		if (ENCODER_DEFAULT_BAUDRATE != 115200){
			while (1){
				if (encoder_set_baudrate(rs485_dev, 115200)) break;
				LOG_E("set baudrate failed, retry 100ms later\n");
				rt_thread_mdelay(100);
			}
			rt_device_close(rs485_dev);
			rt_thread_mdelay(100);
			rs485_config.baud_rate = 115200;
			rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);		
			rt_device_open(rs485_dev, RT_DEVICE_FLAG_DMA_RX|RT_DEVICE_FLAG_DMA_TX);
		}*/
	}
}
static void encoder_init_timer(void){
	static rt_bool_t init = RT_FALSE;
  rt_device_t tim_dev = rt_device_find("timer6");    
	if(!init && tim_dev){
		init = RT_TRUE;
		rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD; 
		rt_hwtimerval_t timeout_s = {
			.sec = 0,
			.usec = 1000000/ABS_ENCODER_READ_FREQ,
		};
		rt_device_set_rx_indicate(tim_dev, encoder_timeout_call);
		rt_device_open(tim_dev, RT_DEVICE_OFLAG_RDWR);
		rt_device_control(tim_dev, HWTIMER_CTRL_MODE_SET, &mode);
		rt_device_write(tim_dev, 0, &timeout_s, sizeof(timeout_s));	
	}
}
static void encoder_init_timer_tick(void){
	static rt_bool_t init = RT_FALSE;
  rt_device_t tim_dev = rt_device_find("timer7");    
	if(!init && tim_dev){
		init = RT_TRUE;
		rt_hwtimer_mode_t mode = HWTIMER_MODE_PERIOD;//HWTIMER_MODE_ONESHOT; 
		rt_hwtimerval_t timeout_s = {
			.sec = 0,
			.usec = 1000000*10/ABS_ENCODER_READ_FREQ,
		};
		rt_device_open(tim_dev, RT_DEVICE_OFLAG_RDWR);
		rt_device_control(tim_dev, HWTIMER_CTRL_MODE_SET, &mode);
		rt_device_write(tim_dev, 0, &timeout_s, sizeof(timeout_s));	
	}
}
static float encoder_get_tick(rt_device_t tim_dev){
	rt_hwtimerval_t timeout_s;
	float value = 0;
	rt_device_read(tim_dev, 0, &timeout_s, sizeof(timeout_s));	
	return timeout_s.sec * 1.0f + timeout_s.usec / 1000000.f;
}


static void encoder_rx_entry(void* param){
	rt_device_t rs485_dev = rt_device_find("uart3");  
  rt_device_t tim_dev = rt_device_find("timer7");    
  uint32_t evt;
	rt_int16_t 	encoder_last = 0;
	float		encoder_tick_last = 0;
	
	RT_ASSERT(rs485_dev && tim_dev);
    
	rt_event_init(&evt_rs485, "evt485", RT_IPC_FLAG_FIFO);
	
	encoder_init();
	encoder_init_uart();		
	encoder_init_timer();
	encoder_init_timer_tick();
	
	rt_event_control(&evt_rs485, RT_IPC_CMD_RESET, 0);
	
	while (1){
		rt_event_recv(&evt_rs485, 0xFF, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
		encoder_tick_last = encoder_get_tick(tim_dev); 
		if (encoder_read_current(rs485_dev, &encoder_last)) break;
	}
	LOG_I("encoder on line");	
	while (1){
		rt_int16_t 	encoder_current = 0;
		float				encoder_tick_current = 0;
		rt_bool_t 	encoder_res = RT_FALSE;
		
		rt_event_recv(&evt_rs485, 0xFF, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);		
		encoder_tick_current = encoder_get_tick(tim_dev); 
		encoder_res = encoder_read_current(rs485_dev, &encoder_current);
		
		if (encoder_res){
			float deta = encoder_current - encoder_last;
			float deta_tick = encoder_tick_current - encoder_tick_last;
			if (deta_tick <= 0) deta_tick += 1.0*10/ABS_ENCODER_READ_FREQ;		
			
			if (deta > ABS_ENCODER_HALF_CIRCLE_POINTS) deta -= ABS_ENCODER_CIRCLE_POINTS;
			else if (deta < -ABS_ENCODER_HALF_CIRCLE_POINTS) deta += ABS_ENCODER_CIRCLE_POINTS;
			else if ((deta == ABS_ENCODER_HALF_CIRCLE_POINTS) && (rs485_speed<0)) deta -= ABS_ENCODER_CIRCLE_POINTS;
			else if ((deta == -ABS_ENCODER_HALF_CIRCLE_POINTS) && (rs485_speed>0)) deta += ABS_ENCODER_CIRCLE_POINTS;
			
			deta = deta * ENCODER_SCALE /deta_tick;
			rt_mutex_take(&mt_rs485_locker,RT_WAITING_FOREVER);
			rs485_encoder = encoder_current;
			rs485_speed = deta;
			rs485_tick = rt_tick_get();
			rt_mutex_release(&mt_rs485_locker);
			
			encoder_last = encoder_current;
			encoder_tick_last = encoder_tick_current;		
		}
		else{
			rs485_tick = 0;
			LOG_W("enc failed");
		}
	}
}

static void encoder_scan(rt_device_t rs485_dev){
	do{
		rt_int16_t encoder_value;
		int bd[5] = {9600, 19200, 38400, 57600, 115200}; 
		for (uint8_t b = 0; b < 5; b++){
			for(uint8_t s = 0 ; s < 3; s++){
				for(uint8_t p = 0 ; p < 2; p++){
					encoder_update_parameter(rs485_dev, bd[4-b], s, p);
					rt_thread_mdelay(100);
					if (encoder_read_current(rs485_dev, &encoder_value)){
						LOG_I("%d %d %d", b,s,p);
					}	
					else {
						LOG_E("%d %d %d", b,s,p);
					}
					rt_thread_mdelay(100);					
				}
			}			
		}
		
		return;
	}while(0);
}
static void encoder_rx_entry_test(void* param){
	rt_device_t rs485_dev = rt_device_find("uart3"); 
	rt_uint8_t cnt_max = 10;
	rt_uint8_t cnt_table[8];
	rt_int32_t encoder_table[8];
	rt_uint8_t idx_last = (GPIOC->IDR >> 6) & 0x07;
	rt_bool_t dir = RT_TRUE;
	rt_bool_t log_enable = RT_TRUE;
	RT_ASSERT(rs485_dev);
	encoder_init();
	encoder_init_uart();
	rt_memset(cnt_table, 0, 8);
	
	//encoder_read_uid(rs485_dev);
	LOG_I("encoder test start");
	
	rt_tick_t  last_tick;
	while (1){
			rt_int16_t encoder_value;
			rt_tick_t curr_tick = rt_tick_get();
			if (encoder_read_current(rs485_dev, &encoder_value)){
				LOG_I("%d spance %d ms, transmit %d ms", encoder_value,curr_tick - last_tick,  rt_tick_get() - curr_tick);
				last_tick = curr_tick;
			}
			else{
				LOG_E("read failed");
			}
			//rt_thread_mdelay(2);
	}
	
	
	while(1){
		if (log_enable){
			log_enable = RT_FALSE;
			LOG_I("move %s manual please", dir ?"positive":"negative");
		}
		rt_int16_t encoder_value;
		int idx = ((GPIOC->IDR >> 6) & 0x07);
		if (idx == idx_last) continue;//û�仯
		if ((idx == 0) || (idx == 7)){
			LOG_E("hall sector is invalid , got %d", idx);
			continue;
		}
		if (encoder_read_current(rs485_dev, &encoder_value)){
			rt_bool_t valid = RT_FALSE;
			LOG_I("got %s section %d encoder value %d\n", dir ?"positive":"negative", idx, encoder_value);
			//5->1->3->2->6->4
			switch (idx){
				case 5:
					if ((idx_last == 4) && dir) valid = RT_TRUE;
					else if ((idx_last == 1) && !dir) valid = RT_TRUE;			
					break;
				case 1:
					if ((idx_last == 5) && dir) valid = RT_TRUE;
					else if ((idx_last == 3) && !dir) valid = RT_TRUE;			
					break;
				case 3:
					if ((idx_last == 1) && dir) valid = RT_TRUE;
					else if ((idx_last == 2) && !dir) valid = RT_TRUE;			
					break;
				case 2:
					if ((idx_last == 3) && dir) valid = RT_TRUE;
					else if ((idx_last == 6) && !dir) valid = RT_TRUE;			
					break;
				case 6:
					if ((idx_last == 2) && dir) valid = RT_TRUE;
					else if ((idx_last == 4) && !dir) valid = RT_TRUE;			
					break;
				case 4:
					if ((idx_last == 6) && dir) valid = RT_TRUE;
					else if ((idx_last == 5) && !dir) valid = RT_TRUE;			
					break;
			}			
		
			if (valid){
				if ((cnt_table[idx] < cnt_max) && dir) {
					encoder_table[idx] += encoder_value;
					cnt_table[idx]++;
				}
				else if ((cnt_table[idx] < (cnt_max*2)) && !dir) {
					encoder_table[idx] += encoder_value;
					cnt_table[idx]++;
				}
				else valid = RT_FALSE;
				
				if (valid){
					if (dir){
							for( int n = 1; (n < 7) && valid; n++){
								if (cnt_table[n] < cnt_max) valid = RT_FALSE;
							}
					}
					else{
							for( int n = 1; (n < 7) && valid; n++){
								if (cnt_table[n] < (cnt_max*2)) valid = RT_FALSE;
							}
					}
					if (valid && dir) {
						dir = RT_FALSE;
						log_enable = RT_TRUE;
					}
					else if (valid && !dir){
						break;
					}
				}
				//LOG_I("got %s section %d encoder value %d\n", dir ?"positive":"negative", idx, encoder_value);
			}
		}
		idx_last = idx;		
	}
	for( int n = 1; (n < 7); n++){
		encoder_table[n] /= (cnt_max*2);
		LOG_I("sector %d, encoder average value %d", n, (int)(encoder_table[n]));
	}	
	LOG_I("encoder test finished");

}

int app_abs_encoder(void){
	if (g_init){
    rt_thread_t tid = rt_thread_create("encoder",encoder_rx_entry, RT_NULL, 4096, 14, RT_TICK_PER_SECOND/10);
    if (tid) return rt_thread_startup(tid);
	}
	/*else{
    rt_thread_t tid = rt_thread_create("encoder",encoder_rx_entry_test, RT_NULL, 4096, 18, RT_TICK_PER_SECOND/10);
    if (tid) return rt_thread_startup(tid);
	}*/
	return -RT_ERROR;
}
INIT_APP_EXPORT(app_abs_encoder);


static void enc_s_read(uint8_t argc, char **argv){
	rt_device_t rs485_dev = rt_device_find("uart3");  
	if (argc < 2){
		LOG_E("loss register address");
		return;
	}
	rt_uint16_t reg = atoi(argv[1]);
	rt_uint16_t cnt = 1;
	if (argc > 2) cnt = atoi(argv[2]);
	rt_uint8_t buffer[128];
	rt_int8_t len = 0;
	rt_mutex_take(&mt_rs485, RT_WAITING_FOREVER);
	RE485_LEAVE_RECEIVE_MODE();
	len = encoder_make_read_single(buffer, reg, cnt);
	rt_device_write(rs485_dev, 0, buffer, len);
	encoder_wait_for_tx_done();
	LOG_HEX("cmd", 16, buffer, len);
	rt_memset(buffer, 0, 128);
	len = encoder_receive_frame(rs485_dev,buffer, 0x03, 5000);
	rt_mutex_release(&mt_rs485);
	if (len > 0)LOG_HEX("rsp", 16, buffer, len);
}
FINSH_FUNCTION_EXPORT_ALIAS(enc_s_read, __cmd_enc_s_read, encoder single read );



static void enc_read(uint8_t argc, char **argv){
	rt_int16_t encoder;
	rt_device_t rs485_dev = rt_device_find("uart3");  
	encoder_init();
	encoder_init_uart();		
	if (encoder_read_current(rs485_dev, &encoder)) LOG_I("encoder is %d", encoder);
	else LOG_E("encoder read failed");
}
FINSH_FUNCTION_EXPORT_ALIAS(enc_read, __cmd_enc_read, read encoder);
static void enc_szero(uint8_t argc, char **argv){
	rt_device_t rs485_dev = rt_device_find("uart3");  
	encoder_init();
	encoder_init_uart();		
	if (encoder_set_zero(rs485_dev)) LOG_I("encoder set zero finished");
	else LOG_E("encoder set zero failed");
}
FINSH_FUNCTION_EXPORT_ALIAS(enc_szero, __cmd_enc_szero, set encoder position to zero);
static void enc_sbaudrate(uint8_t argc, char **argv){
	if (argc != 3){
		LOG_E("command formate: enc_sbaudrate [current] [target]");
	}
	else{
		struct serial_configure rs485_config = RT_SERIAL_CONFIG_DEFAULT;
		rt_device_t rs485_dev = rt_device_find("uart3");  
		encoder_init();
		encoder_init_uart();		
		rt_uint32_t current = atoi(argv[1]);
		rt_uint32_t target = atoi(argv[2]);
		rs485_config.baud_rate = current;
		rs485_config.data_bits = DATA_BITS_8;     
		rs485_config.stop_bits = ENCODER_DEFAULT_STOPBIT;        
		rs485_config.parity    = ENCODER_DEFUALT_PARITY;   
		rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);		
		if (encoder_set_baudrate_level(rs485_dev, target)){
		//if (encoder_set_baudrate(rs485_dev, target)){
			LOG_I("encoder set baudrate finished");
		}
		else LOG_E("encoder set baudrate failed");
			rs485_config.baud_rate = target;
			rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &rs485_config);		
	}
}
FINSH_FUNCTION_EXPORT_ALIAS(enc_sbaudrate, __cmd_enc_sbaudrate, enc_sbaudrate 9600 115200);




