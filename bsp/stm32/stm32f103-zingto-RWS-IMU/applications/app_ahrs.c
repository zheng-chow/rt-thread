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
#include "AHRSHelper.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG //DBG_INFO
#define DBG_SECTION_NAME  "app.ahrs"
#define DBG_COLOR
#include <rtdbg.h>

#define SEMAPHORE_RX_NAME   "sem_IMUrx"
#define ACCE_SENSOR_NAME    "acce_ADiS"
#define GYRO_SENSOR_NAME    "gyro_ADiS"

/* defined the LED0 pin: PC10 */
#define LED0_PIN        GET_PIN(C, 10)

static struct rt_semaphore rx_sem;

static rt_err_t sensor_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

const float DELTA = 0.05f, KAPPA = 0.2f, ALPHA= 0.8f;
static float fal_function(float error)
{
    float res;
    int sign_flag = 1;
    
    if(error < 0) {
        error = -error;
        sign_flag = -1;
    }
    if(error > DELTA)
        res =KAPPA*pow(error, ALPHA)*sign_flag;
    else
        res =KAPPA * error / pow(DELTA,(1 - ALPHA));
    
    return res;
}

static float fal_filter(float last, float input)
{
    float error;
    
    error = input - last;
    
    last += fal_function(error);
    
    return last;
}

void sensor_rx_thread_entry(void * args)
{
    ImuAHRS_t *pAHRS = args;
    
    rt_device_t acce = RT_NULL;
    rt_device_t gyro = RT_NULL;
    
    struct rt_sensor_data acce_data;
    struct rt_sensor_data gyro_data;
    
    float fdata, ferror;
    
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_sem_init(&rx_sem, SEMAPHORE_RX_NAME, 0, RT_IPC_FLAG_FIFO);   

    acce = rt_device_find(ACCE_SENSOR_NAME);
    if (acce == RT_NULL) {
        LOG_E("can't find %s!", ACCE_SENSOR_NAME);
        return;
    }
    rt_device_set_rx_indicate(acce, sensor_rx_callback);
    rt_device_open(acce, RT_DEVICE_FLAG_INT_RX);

    gyro = rt_device_find(GYRO_SENSOR_NAME);
    if (gyro == RT_NULL) {
        LOG_E("can't find %s!", GYRO_SENSOR_NAME);
        return;
    }
    rt_device_open(gyro, RT_DEVICE_FLAG_RDONLY);
    
    Q_Reset(&pAHRS->QS);
    
    while(1) {
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        
        rt_pin_write(LED0_PIN, PIN_HIGH);
        
        rt_device_read(acce, 0, &acce_data, 1);
        rt_device_read(gyro, 0, &gyro_data, 1);
        
#if 0
        pAHRS->accel.X = (float)acce_data.data.acce.x * ADIS_ACCE_RATIO;
        pAHRS->accel.Y = (float)acce_data.data.acce.y * ADIS_ACCE_RATIO;
        pAHRS->accel.Z = (float)acce_data.data.acce.z * ADIS_ACCE_RATIO;
        
        pAHRS->gyro.X = ToRad((float)gyro_data.data.gyro.x * ADIS_GYRO_RATIO);
        pAHRS->gyro.Y = ToRad((float)gyro_data.data.gyro.y * ADIS_GYRO_RATIO);
        pAHRS->gyro.Z = ToRad((float)gyro_data.data.gyro.z * ADIS_GYRO_RATIO);
#else
        pAHRS->accel.X = (float)acce_data.data.acce.y * ADIS_ACCE_RATIO;
        pAHRS->accel.Y = (float)acce_data.data.acce.z * ADIS_ACCE_RATIO;
        pAHRS->accel.Z = (float)acce_data.data.acce.x * ADIS_ACCE_RATIO;
        
        ferror = 
        
        fdata = ToRad((float)gyro_data.data.gyro.y * ADIS_GYRO_RATIO);
        ferror = fdata - pAHRS->gyro.X;
        if ( fabs(ferror) > ToRad(0.1) )
            pAHRS->gyro.X = fdata;//fal_filter(pAHRS->gyro.X, ftmp);
        
        fdata = ToRad((float)gyro_data.data.gyro.z * ADIS_GYRO_RATIO);
        ferror = fdata - pAHRS->gyro.Y;
        if ( fabs(ferror) > ToRad(0.1) )
            pAHRS->gyro.Y = fdata;//fal_filter(pAHRS->gyro.Y, ftmp);
        
        fdata = ToRad((float)gyro_data.data.gyro.x * ADIS_GYRO_RATIO);
        ferror = fdata - pAHRS->gyro.Z;
        if ( fabs(ferror) > ToRad(0.1) )
            pAHRS->gyro.Z = fdata;//fal_filter(pAHRS->gyro.Z, ftmp);

#endif

        IMUupdate(pAHRS);
        
        rt_mutex_take(pAHRS->mutex, RT_WAITING_FOREVER);
        
        rt_memcpy(&pAHRS->mirrorQS, &pAHRS->QS, sizeof(Quaternion_t));
        
        rt_mutex_release(pAHRS->mutex);
        
        rt_pin_write(LED0_PIN, PIN_LOW);
    }
}

#define SPI_BUS_NAME    "spi1"
#define SPI_DEV_NAME    "spi10"

int adis16334_port(void)
{
    struct rt_sensor_config cfg;
    
    rt_hw_spi_device_attach(SPI_BUS_NAME, SPI_DEV_NAME, GPIOA, GPIO_PIN_4);
    
    cfg.intf.dev_name = SPI_DEV_NAME;
    cfg.intf.type = RT_SENSOR_INTF_SPI;
    cfg.intf.user_data = RT_NULL;

    cfg.irq_pin.pin = GET_PIN(B, 2);
    cfg.irq_pin.mode = PIN_MODE_INPUT_PULLDOWN;
    
    cfg.mode = RT_SENSOR_MODE_INT;
    cfg.power = RT_SENSOR_POWER_NORMAL;
    
    rt_hw_adis16334_init("ADiS", &cfg);

    return 0;
}
INIT_APP_EXPORT(adis16334_port);
