#ifndef __APP_CAN_H__
#define __APP_CAN_H__
#include <rtdef.h>
#include <rtthread.h>
#include <rtdevice.h>
rt_bool_t can_test_send(void);
rt_bool_t can_test_receive(struct rt_can_msg* rxmsg, int32_t timeout);

rt_bool_t app_can_init(void);
rt_bool_t can_read_gyro(float* gyro);
rt_bool_t can_read_accel(float* accel);
rt_bool_t can_read_q(float* q);
rt_bool_t can_reset_q(void);
rt_bool_t can_set_pitch(float pitch);
rt_bool_t can_set_roll(float roll);
rt_bool_t can_set_yaw(float yaw);
rt_bool_t can_read_distance(float* distance);
//rt_bool_t can_set_motor(uint8_t motor_id,uint16_t mcoeff, float eangle);
rt_bool_t can_set_motor(uint8_t motor_id, float speed);
rt_bool_t can_get_eangle(uint8_t motor_id, float* eangle);
rt_bool_t can_motor_enable(uint8_t motor_id,  rt_bool_t enable);
//rt_bool_t can_encoder_read(int16_t* encoder);
//rt_bool_t can_encoder_reset(void);

rt_bool_t can_trigger_set_locker(rt_bool_t enable);
rt_bool_t can_trigger_shoot(int8_t position);

#if 0
#ifndef AXIS_SZ	
#define AXIS_SZ 3
#endif



#pragma pack(1)
#pragma anon_unions
struct zingto_can_frame
{
	rt_uint8_t reserved:3;
	rt_uint8_t def_err:1;
	rt_uint8_t def_read:1;
	rt_uint8_t def_write:1;
	rt_uint8_t def_cmd:1;
	rt_uint8_t def_ret:1;

	rt_uint8_t type;
    
    union
    {
        rt_uint16_t     raw_data16[3];
        
        struct {
            rt_uint16_t elec_M;
            float       elec_angle;
        };
        
        struct{
            rt_int16_t  enc_pos;
            float   enc_speed;
        };
        
        struct{
            rt_uint16_t vref_raw;
            rt_uint16_t vIa_raw;
            rt_uint16_t vIb_raw;
        };
        
        struct{
            rt_int16_t Ia_real;
            rt_int16_t currentB;
        };
				
				struct {
            rt_int16_t 	encoder_offs;
            float       angle_offs;
        };

    };
    
};
#pragma pack()

#define CAN_PACKET_ACCEL    		0xA1
#define CAN_PACKET_GYRO     		0xA2
#define CAN_PACKET_TEMP     		0xA3

#define CAN_PACKET_SETRPOS    		0xB1
#define CAN_PACKET_ENCODER			0xB2
#define CAN_PACKET_ENCODER_RAW	    0xB3
//#define CAN_PACKET_CURRENT		    0xB4
#define CAN_PACKET_CURRENT_RAW		0xB5
#define CAN_PACKET_SVPWMEN			0xB6
#define CAN_PACKET_GETRPOS			0xB7
#define CAN_PACKET_OFFS			0xB8

rt_bool_t app_can_init(void);// setfilter
rt_bool_t can_read_gyro(uint16_t* px, uint16_t* py, uint16_t* pz);
rt_bool_t can_read_accel(uint16_t* px, uint16_t* py, uint16_t* pz);
rt_bool_t can_read_temper(uint16_t* pt);
rt_bool_t can_read_sensor(int16_t* buffer);


rt_bool_t can_read_encoder_raw(uint8_t motorx, rt_int16_t * encoder, float* speed);
rt_bool_t can_read_encoder(uint8_t motorx, rt_int16_t * encoder, float* speed);
rt_bool_t can_read_rotor(uint8_t addr, float* obs_angle);
rt_bool_t can_write_motor(uint8_t addr, float m, float elecangle);


rt_bool_t can_set_motor_enable(uint8_t motorx, rt_bool_t enable);
rt_bool_t can_read_current(uint8_t motorx, rt_int16_t * current);
rt_bool_t can_read_adc_current(uint8_t motorx, rt_uint16_t * adc);
rt_bool_t can_read_offs(uint8_t addr, rt_int16_t* pencoder_offs, float* pangle_offs);

#endif
#endif

