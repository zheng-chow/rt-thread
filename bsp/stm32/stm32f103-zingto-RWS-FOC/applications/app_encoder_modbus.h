#ifndef __APP_MODBUS_H__
#define __APP_MODBUS_H__
#include <rtdef.h>
#define ENCODER_MODBUS_ADDR 0x01
#define ABS_ENCODER_CIRCLE_POINTS 4096L
#define ABS_ENCODER_HALF_CIRCLE_POINTS (2048L)

rt_uint16_t modbus_crc(rt_uint8_t* buffer, rt_uint8_t buflen);
rt_uint8_t encoder_make_read(rt_uint8_t* buffer, rt_uint16_t reg);
rt_uint8_t encoder_make_write(rt_uint8_t* buffer, rt_uint16_t reg, rt_uint16_t val);
rt_uint8_t encoder_make_multi_write(rt_uint8_t* buffer, rt_uint16_t reg, rt_uint16_t* val, rt_uint8_t valcnt);

rt_uint8_t encoder_make_read_single(rt_uint8_t* buffer, rt_uint16_t reg, rt_uint8_t regcnt);


rt_bool_t encoder_response_valid(rt_uint8_t* buffer, rt_int16_t buflen);
int16_t encoder_check_cmd(rt_uint8_t* buffer, rt_uint8_t cmd);

#endif

