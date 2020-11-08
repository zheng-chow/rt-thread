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

#include "AHRSHelper.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG //DBG_INFO
#define DBG_SECTION_NAME  "app.can"
#define DBG_COLOR
#include <rtdbg.h>

#define SENSOR_IMU_CAN_ID   'I'
#define CONTROLLER_CAN_ID   'C'

static struct rt_semaphore rx_sem;

static rt_err_t can_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

static void _can_send_float(rt_device_t can_dev, rt_uint8_t cmd, rt_uint8_t index, float txval)
{
    struct rt_can_msg txMsg = {0};
    
    txMsg.id = CONTROLLER_CAN_ID;
    txMsg.ide = RT_CAN_STDID;
    txMsg.rtr = RT_CAN_DTR;
    txMsg.len = 6;  // cmd(1B) + index(1B) + float(4B)
    txMsg.data[0] = cmd;
    txMsg.data[1] = index;
    rt_memcpy(&txMsg.data[2], &txval, sizeof(float));
    
    rt_device_write(can_dev, 0, &txMsg, sizeof(txMsg));
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
void can_rx_thread_handler(void *args)
{
    ImuAHRS_t *pAHRS = (ImuAHRS_t *)args;
    
    struct rt_can_filter_item items[1] = {
        RT_CAN_FILTER_STD_INIT(SENSOR_IMU_CAN_ID, RT_NULL, RT_NULL), };
    struct rt_can_filter_config cfg = {1, 1, items};
    
    struct rt_can_msg rxMsg = {0};
    
    rt_sem_init(&rx_sem, "sem_CANrx", 0, RT_IPC_FLAG_FIFO);
    
    rt_device_t can_dev = rt_device_find("can1");
    RT_ASSERT(can_dev != RT_NULL);
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN500kBaud); //CAN1MBaud
    rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    rt_device_set_rx_indicate(can_dev, can_rx_callback);
    
    while(1) {
//        _can_send_float(can_dev, 'W', 0x01, 1.000f);
//        LOG_I("send W 1.000f");
//        rt_thread_delay(RT_TICK_PER_SECOND / 50);
//        continue;
        
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        rxMsg.hdr = -1;
        rt_device_read(can_dev, 0, &rxMsg, sizeof(rxMsg));
        
        //LOG_I("%02X %02X %02X %02X", rxMsg.id, rxMsg.data[0], rxMsg.data[1], rxMsg.data[2]);
        switch(rxMsg.data[0]) {
            case 'A': {
                float fbuf[3];

                //rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
                fbuf[0] = pAHRS->accel.X;
                fbuf[1] = pAHRS->accel.Y;
                fbuf[2] = pAHRS->accel.Z;
                //rt_mutex_release(pAHRS->mutex);
                
                _can_send_float(can_dev, 'A', 0x01, fbuf[0]);
                _can_send_float(can_dev, 'A', 0x02, fbuf[1]);
                _can_send_float(can_dev, 'A', 0x03, fbuf[2]);
                
                break;
            }
            case 'W': {
                float fbuf[3];
                
                //rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
                fbuf[0] = pAHRS->gyro.X;
                fbuf[1] = pAHRS->gyro.Y;
                fbuf[2] = pAHRS->gyro.Z;
                //rt_mutex_release(pAHRS->mutex);
                
                _can_send_float(can_dev, 'W', 0x01, fbuf[0]);
                _can_send_float(can_dev, 'W', 0x02, fbuf[1]);
                _can_send_float(can_dev, 'W', 0x03, fbuf[2]);
                
                break;
            }
            case 'Q': {
                Quaternion_t qbuf;
                
                rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
                rt_memcpy(&qbuf, &pAHRS->mirrorQS, sizeof(Quaternion_t));
                rt_mutex_release(pAHRS->mutex);

                _can_send_float(can_dev, 'Q', 0x01, qbuf.q0);
                _can_send_float(can_dev, 'Q', 0x02, qbuf.q1);
                _can_send_float(can_dev, 'Q', 0x03, qbuf.q2);
                _can_send_float(can_dev, 'Q', 0x04, qbuf.q3);
                
                break;
            }
            case 'R': {
                // reset the Quaternion.
                rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
                Q_Reset(&pAHRS->QS);
                rt_mutex_release(pAHRS->mutex);
                _can_send_result(can_dev, 'R', 0x01, 'S');
                LOG_W("reset the Quaternion.");
                break;
            }
            
            case 'E': {
                switch(rxMsg.data[0]) {
                    case '1': {
                        rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
                        
                        Q_ToEularGeneral(&pAHRS->QS, &pAHRS->eular);
                        
                        pAHRS->eular.pitch = 0.0f;
                        
                        Q_FromEularGeneral(&pAHRS->eular, &pAHRS->QS);
                       
                        rt_mutex_release(pAHRS->mutex);
                        
                        _can_send_result(can_dev, 'E', 0x01, 'S');
                        
                        break;
                    }
                    case '2': {
                        rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
                        
                        Q_ToEularGeneral(&pAHRS->QS, &pAHRS->eular);
                        
                        pAHRS->eular.roll = 0.0f;
                        
                        Q_FromEularGeneral(&pAHRS->eular, &pAHRS->QS);
                       
                        rt_mutex_release(pAHRS->mutex);
                        
                        _can_send_result(can_dev, 'E', 0x01, 'S');
                        
                        break;
                    }
                    case '3': {
                        rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
                        
                        Q_ToEularGeneral(&pAHRS->QS, &pAHRS->eular);
                        
                        pAHRS->eular.yaw = 0.0f;
                        
                        Q_FromEularGeneral(&pAHRS->eular, &pAHRS->QS);
                       
                        rt_mutex_release(pAHRS->mutex);
                        
                        _can_send_result(can_dev, 'E', 0x01, 'S');
                        
                        break;
                    }
                    default : {
                        LOG_W("reset the Eular. but channel error.");
                    }
                }
                
                break;
            }
            default : {
                LOG_W("recvive message but unkown command in it. %02X", rxMsg.data[0]);
                break;
            }
        }
    }
}
