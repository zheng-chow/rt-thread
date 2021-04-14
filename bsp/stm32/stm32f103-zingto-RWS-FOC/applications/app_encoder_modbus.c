#include <app_encoder_modbus.h>

#define xDRV_DEBUG
#define LOG_TAG             "app.encoder.modbus"
#include <drv_log.h>

rt_uint16_t modbus_crc(rt_uint8_t* buffer, rt_uint8_t buflen){
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



rt_uint8_t encoder_make_read(rt_uint8_t* buffer, rt_uint16_t reg){
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
rt_uint8_t encoder_make_read_single(rt_uint8_t* buffer, rt_uint16_t reg, rt_uint8_t regcnt){
	rt_uint16_t crc;
	buffer[0] = ENCODER_MODBUS_ADDR;
	buffer[1] = 0x03;
	buffer[2] = reg >> 8;
	buffer[3] = reg;
	buffer[4] = 0;
	buffer[5] = regcnt;
	crc = modbus_crc(buffer, 6);
	buffer[6] = crc;
	buffer[7] = crc>>8;
	return 8;
}
rt_uint8_t encoder_make_write(rt_uint8_t* buffer, rt_uint16_t reg, rt_uint16_t val){
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
rt_uint8_t encoder_make_multi_write(rt_uint8_t* buffer, rt_uint16_t reg, rt_uint16_t* val, rt_uint8_t valcnt){
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

rt_bool_t encoder_response_valid(rt_uint8_t* buffer, rt_int16_t buflen){
	if (buflen < 5) {
		LOG_E("receive response too less %d", buflen);
		return RT_FALSE;
	}
	if (buffer[0] != ENCODER_MODBUS_ADDR) {
		LOG_E("receive addr %02X but not %02X", buffer[0] , ENCODER_MODBUS_ADDR);
		return RT_FALSE;
	}
	rt_uint8_t total = 0;
	if (buffer[1] == 0x03)	total = 5 + buffer[2];
	else if ((buffer[1] == 0x06) || (buffer[1] == 0x10))	total = 8;
	else if (buffer[1] == 0x83)	total = 5;
	else{
		LOG_E("unknown command response %02X", buffer[1]);
		return RT_FALSE;
	}	
	if (total > buflen) {
		LOG_E("receive response miss bytes, receive %d but should be %d", buflen, total);
		return RT_FALSE;
	}
	if (total < buflen) {
		LOG_W("receive response with extra bytes, receive %d but should be %d", buflen, total);
		//LOG_HEX("extra", 16, buffer, buflen);
	}
	rt_uint16_t crc = modbus_crc(buffer, total - 2);
	if ((buffer[total-1] != (crc>>8))	|| (buffer[total-2] != (crc & 0xFF))){
		LOG_E("crc error, receive %02X%02X but calculate %04X", buffer[total-2],buffer[total-1], crc);
		return RT_FALSE;
	}
	return RT_TRUE;
}
int16_t encoder_check_cmd(rt_uint8_t* buffer, rt_uint8_t cmd){
	if (cmd != buffer[1]){
		LOG_I("command err, receive %02X but wait %02X", buffer[1], cmd);
		return -1;
	}		
	else {
		if (buffer[1] == 0x03)	return 5 + buffer[2];
		else if ((buffer[1] == 0x06) || (buffer[1] == 0x10))	return 8;
		else if (buffer[1] == 0x83)	return 5;
		else return -1;
	}
}
