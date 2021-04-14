 /*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-03-08     zhouzheng   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG //DBG_INFO
#define DBG_SECTION_NAME  "app.can"
#define DBG_COLOR
#include <rtdbg.h>

#define TRIGGER_CAN_ID      'T'
#define CONTROLLER_CAN_ID   'C'

#define PWM_DEV_NAME        "pwm2"
#define PWM_DEV_CHANNEL     1

#define SERVO_PWM_PERIOD    (20000 * 1000)      // 20ms,  unit ns
#define SERVO_PWM_PULSE_MAX (2200 * 1000)       // 2.2ms, unit ns
#define SERVO_PWM_PULSE_MIN (900 * 1000)        // 0.9ms, unit ns

static struct rt_semaphore rx_sem;
struct rt_device_pwm *pwm_dev = RT_NULL;

static rt_err_t can_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);

    return RT_EOK;
}
static void _can_send_result(rt_device_t can_dev, rt_uint8_t cmd, rt_uint8_t index, char txval)
{
    struct rt_can_msg txMsg = {0};
    
    txMsg.id = CONTROLLER_CAN_ID;
    txMsg.ide = RT_CAN_STDID;
    txMsg.rtr = RT_CAN_DTR;
    txMsg.len = 3;  // cmd(1B) + index(1B) + char(1B)
    txMsg.data[0] = cmd;
    txMsg.data[1] = index;
    txMsg.data[2] = txval;
    
    rt_device_write(can_dev, 0, &txMsg, sizeof(txMsg));
}

static void _servo_set_pos(struct rt_device_pwm *dev, rt_uint8_t value)
{
    rt_uint8_t pos = value;
    
    if (pos > 100) pos = 100;

    rt_uint32_t pulse = 
        (SERVO_PWM_PULSE_MAX - SERVO_PWM_PULSE_MIN) * pos / 100 + SERVO_PWM_PULSE_MIN;

    rt_pwm_set(dev, PWM_DEV_CHANNEL, SERVO_PWM_PERIOD, pulse);
}

static int servo_set(int argc, char **argv)
{
    int result = 0;
    
    if (argc != 2)
    {
        rt_kprintf("Usage: servo_set val[0-100]\n");
        result = -RT_ERROR;
        goto _exit;
    }
    
    _servo_set_pos(pwm_dev, atoi(argv[1]));
    
_exit:
    return result;
}
MSH_CMD_EXPORT(servo_set, servo_set val[0-100]);

void can_rx_thread_handler(void *args)
{
    struct rt_can_filter_item items[1] = {
        RT_CAN_FILTER_STD_INIT(TRIGGER_CAN_ID, RT_NULL, RT_NULL), };
    struct rt_can_filter_config cfg = {1, 1, items};
    
    struct rt_can_msg rxMsg = {0};
    
    rt_sem_init(&rx_sem, "sem_CANrx", 0, RT_IPC_FLAG_FIFO);
    
    rt_device_t can_dev = rt_device_find("can1");
    RT_ASSERT(can_dev != RT_NULL);
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud); //CAN1MBaud
    rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    rt_device_set_rx_indicate(can_dev, can_rx_callback);
    
    {
        rt_uint32_t pulse;

        pulse =  SERVO_PWM_PULSE_MIN;

        pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
        if (pwm_dev == RT_NULL) {
            LOG_E("servo not find the extern clock pwm");
            return;
        }

        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, SERVO_PWM_PERIOD, pulse);
        rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
    }
    
    while(1) {
        
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        rxMsg.hdr = -1;
        rt_device_read(can_dev, 0, &rxMsg, sizeof(rxMsg));
        
        LOG_D("%02X %02X %02X %02X", rxMsg.id, rxMsg.data[0], rxMsg.data[1], rxMsg.data[2]);
        switch(rxMsg.data[0]) {
            case 'L': {
                LOG_I("UNLOCK");
                _can_send_result(can_dev, 'L', 0x01, 'S');
                break;
            }
            case 'P': {
                rt_uint8_t pos = rxMsg.data[1];
                _servo_set_pos(pwm_dev, pos);
                LOG_I("POS: %d%", pos);
                _can_send_result(can_dev, 'P', 0x01, 'S');
                break;
            }
            default : {
                LOG_W("recvive message but unkown command in it. %02X", rxMsg.data[0]);
                break;
            }
        }
    }
}
