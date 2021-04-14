 /*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-08     zhouzheng   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include <adis16334.h>
#include <rm3100.h>
#include "AHRSHelper.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG //DBG_INFO
#define DBG_SECTION_NAME  "app.ahrs"
#define DBG_COLOR
#include <rtdbg.h>

#define SEMAPHORE_IMU_RX    "sem_IMU_RX"
#define SEMAPHORE_MAG_RX    "sem_Mag_RX"
#define ACCE_SENSOR_NAME    "acce_ADiS"
#define GYRO_SENSOR_NAME    "gyro_ADiS"
#define MAG_SENSOR_NAME     "mag_PNI"

#define USING_KALMAN_FILTER
#ifdef USING_KALMAN_FILTER    
#include "kalman_filter.h"

#define KLM_ACCE_P          (50)
#define KLM_GYRO_P          (50)
#endif


/* defined the LED0 pin: PC10 */
#define LED0_PIN        GET_PIN(C, 10)

static struct rt_semaphore sem_accel, sem_mag;

static ImuAHRS_t *pAHRS = RT_NULL;

static rt_err_t accel_update_callback(rt_device_t dev, rt_size_t size) {
    return rt_sem_release(&sem_accel);
}

static rt_err_t mag_update_callback(rt_device_t dev, rt_size_t size) {
    return rt_sem_release(&sem_mag);
}

void mag_rx_thread_entry(void *args)
{
    pAHRS = (ImuAHRS_t *)args;
    rt_device_t mag = RT_NULL;
    struct rt_sensor_data mag_data;
    
    rt_sem_init(&sem_mag, SEMAPHORE_MAG_RX, 0, RT_IPC_FLAG_FIFO);

    mag = rt_device_find(MAG_SENSOR_NAME);
    if (mag == RT_NULL) {
        LOG_E("can't find %s!", MAG_SENSOR_NAME);
        return;
    }
    rt_device_set_rx_indicate(mag, mag_update_callback);
    rt_device_open(mag, RT_DEVICE_FLAG_INT_RX);
    rt_device_control(mag, RT_SENSOR_CTRL_SET_MODE, 0);
    while(1) {

        rt_sem_take(&sem_mag, RT_WAITING_FOREVER);
        rt_device_read(mag, 0, &mag_data, 1);
        pAHRS->mag.X = (float)mag_data.data.mag.x * RM3100_MCC_200_UNIT;
        pAHRS->mag.Y = (float)mag_data.data.mag.y * RM3100_MCC_200_UNIT;
        pAHRS->mag.Z = (float)mag_data.data.mag.z * RM3100_MCC_200_UNIT;
        pAHRS->magValid = RT_FALSE;
    }
}

void accel_rx_thread_entry(void *args)
{
    pAHRS = (ImuAHRS_t *)args;
    rt_device_t acce = RT_NULL, gyro = RT_NULL;
    struct rt_sensor_data acce_data, gyro_data;

#ifdef USING_KALMAN_FILTER
    kalman1_state klm_gyro[3], klm_acce[3];
#endif
    
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_sem_init(&sem_accel, SEMAPHORE_IMU_RX, 0, RT_IPC_FLAG_FIFO);

    acce = rt_device_find(ACCE_SENSOR_NAME);
    if (acce == RT_NULL) {
        LOG_E("can't find %s!", ACCE_SENSOR_NAME);
        return;
    }
    rt_device_set_rx_indicate(acce, accel_update_callback);
    rt_device_open(acce, RT_DEVICE_FLAG_INT_RX);

    gyro = rt_device_find(GYRO_SENSOR_NAME);
    if (gyro == RT_NULL) {
        LOG_E("can't find %s!", GYRO_SENSOR_NAME);
        return;
    }
    rt_device_open(gyro, RT_DEVICE_FLAG_RDONLY);
    
    
#ifdef USING_KALMAN_FILTER
    kalman1_init(&klm_acce[0], 0, KLM_ACCE_P);
    kalman1_init(&klm_acce[1], 0, KLM_ACCE_P);
    kalman1_init(&klm_acce[2], 0, KLM_ACCE_P);
    kalman1_init(&klm_gyro[0], 0, KLM_GYRO_P);
    kalman1_init(&klm_gyro[1], 0, KLM_GYRO_P);
    kalman1_init(&klm_gyro[2], 0, KLM_GYRO_P);
#endif

    Q_Reset(&pAHRS->QS);

    while(1) {
        rt_sem_take(&sem_accel, RT_WAITING_FOREVER);
        
        rt_pin_write(LED0_PIN, PIN_HIGH);
        
        rt_device_read(acce, 0, &acce_data, 1);
        rt_device_read(gyro, 0, &gyro_data, 1);

        pAHRS->accel.X = (float)acce_data.data.acce.x * ADIS_ACCE_UNIT;
        pAHRS->accel.Y = (float)acce_data.data.acce.y * ADIS_ACCE_UNIT;
        pAHRS->accel.Z = (float)acce_data.data.acce.z * ADIS_ACCE_UNIT;
        
        pAHRS->gyro.X = ToRad((float)gyro_data.data.gyro.x * ADIS_GYRO_UNIT);
        pAHRS->gyro.Y = ToRad((float)gyro_data.data.gyro.y * ADIS_GYRO_UNIT);
        pAHRS->gyro.Z = ToRad((float)gyro_data.data.gyro.z * ADIS_GYRO_UNIT);
        
#ifdef USING_KALMAN_FILTER
        pAHRS->accel.X = kalman1_filter(&klm_acce[0], pAHRS->accel.X);
        pAHRS->accel.Y = kalman1_filter(&klm_acce[1], pAHRS->accel.Y);
        pAHRS->accel.Z = kalman1_filter(&klm_acce[2], pAHRS->accel.Z);
        
        pAHRS->gyro.X  = kalman1_filter(&klm_gyro[0], pAHRS->gyro.X);
        pAHRS->gyro.Y  = kalman1_filter(&klm_gyro[1], pAHRS->gyro.Y);
        pAHRS->gyro.Z  = kalman1_filter(&klm_gyro[2], pAHRS->gyro.Z);
#endif
        if (pAHRS->magValid)
            IMUupdateEx(pAHRS); // 
        else
            IMUupdate(pAHRS);
        
        rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
        
        rt_memcpy(&pAHRS->mirrorQS, &pAHRS->QS, sizeof(Quaternion_t));
        
        rt_mutex_release(pAHRS->mutex);
        
        rt_pin_write(LED0_PIN, PIN_LOW);
    }
}

typedef enum {
    ARHS_DUMPMODE_INVAILD = 0,
    ARHS_DUMPMODE_ACCEL,
    ARHS_DUMPMODE_GYRO,
    ARHS_DUMPMODE_EULAR,
    ARHS_DUMPMODE_MAG,
}ARHS_DumpMode_t;

#include <finsh.h>
static rt_sem_t _sem_kdump = RT_NULL;
#define KDUMP_OUT_BUFFSIZE  128
#define KDUMP_OUT_RATEHZ    20

static rt_err_t arhs_kdump_callback(rt_device_t dev, rt_size_t size) {
    return rt_sem_release(_sem_kdump);
}

int kdump_IMUdata(int argc, char **argv)
{
    rt_bool_t rc_flag = RT_TRUE;
    rt_err_t (*rx_ind_backup)(rt_device_t dev,  rt_size_t size);
    rt_device_t ch_dev = RT_NULL;
    rt_err_t retval = RT_EOK;
    char cbuf[KDUMP_OUT_BUFFSIZE];
    ARHS_DumpMode_t rc_mode = ARHS_DUMPMODE_INVAILD;
    
    if (argc != 2) {
        LOG_RAW("Usage: kdump accel/gyro/eular\n");
        return -RT_EIO;
    }
    
    if (rt_strcmp("accel", argv[1]) == 0) {
        rc_mode = ARHS_DUMPMODE_ACCEL;
        LOG_RAW("Dump Accel Data in %dHz\n", KDUMP_OUT_RATEHZ);
    }
    else if (rt_strcmp("gyro", argv[1]) == 0) {
        rc_mode = ARHS_DUMPMODE_GYRO;
        LOG_RAW("Dump Gyro Data in %dHz\n", KDUMP_OUT_RATEHZ);
    }
    else if (rt_strcmp("mag", argv[1]) == 0) {
        rc_mode = ARHS_DUMPMODE_MAG;
        LOG_RAW("Dump Mag Data in %dHz\n", KDUMP_OUT_RATEHZ);
    }
    else if (rt_strcmp("eular", argv[1]) == 0) {
        rc_mode = ARHS_DUMPMODE_EULAR;
        LOG_RAW("Dump eular angle in %dHz\n", KDUMP_OUT_RATEHZ);
    }
    else {
        LOG_RAW("Usage: kdump accel/gyro/eular\n");
        return -RT_EIO;
    }
    
    ch_dev = rt_device_find(RT_CONSOLE_DEVICE_NAME);
    if( ch_dev == RT_NULL ) {
        LOG_RAW("%s not find\n", RT_CONSOLE_DEVICE_NAME);
        return -RT_ENOMEM;
    }
    
    LOG_RAW("\33[?25l");
    _sem_kdump = rt_sem_create("sem_kdump", 0, RT_IPC_FLAG_FIFO);
    rx_ind_backup = ch_dev->rx_indicate;
    rt_device_set_rx_indicate(ch_dev, arhs_kdump_callback);
    
    while(rc_flag) {
        retval = rt_sem_take(_sem_kdump, RT_TICK_PER_SECOND / KDUMP_OUT_RATEHZ);
        
        if (retval == RT_EOK) {
            char input;
            
            while( rt_device_read(ch_dev, 0, &input, 1) ) {
                if (input == 0x03) // ^C = 0x03
                    rc_flag = RT_FALSE;
            }
            if(!rc_flag) continue;
        }
        
        rt_memset(cbuf, 0, KDUMP_OUT_BUFFSIZE);
        switch(rc_mode) {
            case ARHS_DUMPMODE_ACCEL: {
                snprintf(cbuf, KDUMP_OUT_BUFFSIZE, "%8.3f %8.3f %8.3f", pAHRS->accel.X, pAHRS->accel.Y, pAHRS->accel.Z);
                break;
            }
            case ARHS_DUMPMODE_GYRO: {
                snprintf(cbuf, KDUMP_OUT_BUFFSIZE, "%8.3f %8.3f %8.3f", pAHRS->gyro.X, pAHRS->gyro.Y, pAHRS->gyro.Z);
                break;
            }
            case ARHS_DUMPMODE_MAG: {
                snprintf(cbuf, KDUMP_OUT_BUFFSIZE, "%8.3f %8.3f %8.3f", pAHRS->mag.X, pAHRS->mag.Y, pAHRS->mag.Z);
                break;
            }
            case ARHS_DUMPMODE_EULAR: {
                rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
                Q_ToEularGeneral(&pAHRS->QS, &pAHRS->eular);
                rt_mutex_release(pAHRS->mutex);
                snprintf(cbuf, KDUMP_OUT_BUFFSIZE, "%8.3f %8.3f %8.3f", ToDeg(pAHRS->eular.pitch)
                                                                         , ToDeg(pAHRS->eular.roll)
                                                                         , ToDeg(pAHRS->eular.yaw) );
                break;
            }
            default:
                return -RT_EINVAL;
        }
        LOG_RAW("%c[2K%s\r", 27, cbuf);
    }
    LOG_RAW("\n\33[?25h");
    rt_device_set_rx_indicate(ch_dev, rx_ind_backup);
    rt_sem_delete(_sem_kdump);
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(kdump_IMUdata, kdump, dump accel/gyro/mag/eular ARHS data.);
