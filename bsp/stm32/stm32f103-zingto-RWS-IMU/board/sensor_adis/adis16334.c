// SPDX-License-Identifier: GPL-2.0-only
/*
 * adis16400.c	support Analog Devices ADIS16400/5
 *		3d 2g Linear Accelerometers,
 *		3d Gyroscopes,
 *		3d Magnetometers via SPI
 *
 * Copyright (c) 2009 Manuel Stahl <manuel.stahl@iis.fraunhofer.de>
 * Copyright (c) 2007 Jonathan Cameron <jic23@kernel.org>
 * Copyright (c) 2011 Analog Devices Inc.
 */

#include "adis16334.h"
#include "adis.h"
#include <board.h>

#define DBG_ENABLE
#define DBG_LEVEL DBG_INFO
#define DBG_SECTION_NAME  "adis16334"
#define DBG_COLOR
#include <rtdbg.h>

#define IMU_RST     GET_PIN(A, 1)

#define IMU_READ_BRUST_SIZE     7

#define PWM_DEV_NAME        "pwm3"
#define PWM_DEV_CHANNEL     3

static struct rt_spi_device *spi_bus_dev;
static struct rt_device_pwm *pwm_dev;

static rt_uint16_t _adis_read_reg(struct rt_spi_device * spi, rt_uint8_t addr)
{
    rt_uint16_t cmd = 0, tmp;
    
    cmd = addr << 8;
    rt_spi_send_then_recv(spi, &cmd, 1, &tmp, 1);
    
    return tmp;
}

static rt_err_t _adis_read_brust(struct rt_spi_device * spi, rt_int16_t *buff)
{
    const rt_uint16_t cmd = 0x3E00;
    const rt_size_t   bufsz = IMU_READ_BRUST_SIZE;
    
    rt_spi_send_then_recv(spi, &cmd, 1, buff, bufsz);
    
    for (rt_uint8_t i = 0; i < bufsz; i++) {
        if( buff[i] & 0x2000 )
			buff[i] |= 0xC000;
		else
			buff[i] &= 0x2FFF;
    }

    return RT_EOK;
}

static rt_err_t _adis_write_reg(struct rt_spi_device * spi, rt_uint8_t addr, rt_uint16_t val)
{
    rt_uint16_t tmp = 0;
    
    
    tmp = 0x8000 | ((addr+1) << 8) | (0x00FF & (val >> 8));
    rt_spi_send(spi, &tmp, 1);    
    
    tmp = 0x8000 | (addr << 8) | (0x00FF & val);
    rt_spi_send(spi, &tmp, 1);

    return RT_EOK;
}

static rt_err_t _adis16334_init(struct rt_sensor_intf *intf)
{
    rt_uint16_t tmp;
    
    RT_ASSERT(intf != RT_NULL);
    
    if (spi_bus_dev == RT_NULL) {
        spi_bus_dev = (struct rt_spi_device *)rt_device_find(intf->dev_name);

        if (spi_bus_dev == RT_NULL) {
            return -RT_ERROR;
        }

        spi_bus_dev->config.data_width = 16;
        spi_bus_dev->config.mode = RT_SPI_MASTER | RT_SPI_MSB | RT_SPI_MODE_3;
        spi_bus_dev->config.max_hz = ADIS16400_SPI_BURST;

        LOG_I("config SPI parameter for ADiS16334");
    }
    
    rt_device_open((rt_device_t)spi_bus_dev->bus, RT_DEVICE_OFLAG_RDWR);
    rt_device_open((rt_device_t)spi_bus_dev, RT_DEVICE_OFLAG_RDWR);

    rt_pin_mode(IMU_RST, PIN_MODE_OUTPUT_OD);
    rt_pin_write(IMU_RST, PIN_LOW);
    rt_thread_delay(10);                        // assert a manual reset.
    rt_pin_write(IMU_RST, PIN_HIGH);
    rt_thread_delay(ADIS16400_STARTUP_DELAY);   // reset recovery time.

    tmp = _adis_read_reg(spi_bus_dev, ADIS16400_PRODUCT_ID);
    if (tmp == 0x3FCE) {
        rt_thread_delay(1);
        LOG_D("LOT_ID1: %04X", _adis_read_reg(spi_bus_dev, ADIS16334_LOT_ID1));
        rt_thread_delay(1);
        LOG_D("LOT_ID2: %04X", _adis_read_reg(spi_bus_dev, ADIS16334_LOT_ID2));
        rt_thread_delay(1);
        LOG_D("PROD_ID: %04X", _adis_read_reg(spi_bus_dev, ADIS16400_PRODUCT_ID));
        rt_thread_delay(1);
        LOG_D("SER_NUM: %04X", _adis_read_reg(spi_bus_dev, ADIS16334_SERIAL_NUMBER));
        rt_thread_delay(1);
        LOG_D("GPIOCTL: %04X", _adis_read_reg(spi_bus_dev, ADIS16400_GPIO_CTRL));
        rt_thread_delay(1);
        LOG_D("MSC_CTL: %04X", _adis_read_reg(spi_bus_dev, ADIS16400_MSC_CTRL));
        rt_thread_delay(1);
        LOG_D("SMPLPRD: %04X", _adis_read_reg(spi_bus_dev, ADIS16400_SMPL_PRD));
        rt_thread_delay(1);
        LOG_D("SENSAVG: %04X", _adis_read_reg(spi_bus_dev, ADIS16400_SENS_AVG));
    }
    else {
        LOG_E("wrong PROD_ID: %04X, init failed!", tmp);
        return -RT_ERROR;
    }

    {
        rt_uint32_t period, pulse;

        period = 2 *1000 * 1000;    /* 1ms,  unit ns */
        pulse =  50 * 1000;         /* 50us, unit ns */

        pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
        if (pwm_dev == RT_NULL) {
            LOG_E("can not find the extern clock pwm");
            return -RT_ERROR;
        }

        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
        rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
    }

    {
        LOG_D("set SMPL_PRD with 0x0000 to use external clock.");
        _adis_write_reg(spi_bus_dev, ADIS16400_SMPL_PRD, 0x0000);
        rt_thread_delay(1);
        LOG_D("SMPLPRD: %04X", _adis_read_reg(spi_bus_dev, ADIS16400_SMPL_PRD));
    }

    return RT_EOK;
}

static rt_err_t _adis16334_get_id(rt_sensor_t sensor, rt_uint16_t * buff)
{
    *buff = _adis_read_reg(spi_bus_dev, ADIS16400_PRODUCT_ID);
    
    return RT_EOK;
}

static rt_err_t _adis16334_set_range(rt_sensor_t sensor, rt_int32_t range)
{
    return RT_EOK;
}

static rt_err_t _adis16334_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    return RT_EOK;
}

static rt_err_t _adis16334_set_mode(rt_sensor_t sensor, rt_uint8_t mode)
{
    return RT_EOK;
}

static rt_err_t _adis16334_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    return RT_EOK;
}

static rt_err_t _adis16334_self_test(rt_sensor_t sensor, rt_uint8_t func)
{
    return RT_EOK;
}

static rt_size_t _adis16334_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    static rt_int16_t   out_data[IMU_READ_BRUST_SIZE];
    
    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        LOG_D("read ADiS out data");
        _adis_read_brust(spi_bus_dev, out_data);

        data->type = RT_SENSOR_CLASS_ACCE;
        data->data.acce.x = out_data[3];
        data->data.acce.y = out_data[4];
        data->data.acce.z = out_data[5];
        data->timestamp = rt_sensor_get_ts();

    }
    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
    {
        data->type = RT_SENSOR_CLASS_GYRO;
        data->data.gyro.x = out_data[0];
        data->data.gyro.y = out_data[1];
        data->data.gyro.z = out_data[2];
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_size_t _adis16334_acc_fifo_get_data(rt_sensor_t sensor, struct rt_sensor_data *data, rt_size_t len)
{
//    adis16334_Axes_t acce;
    rt_uint8_t i;

//    for (i = 0; i < len; i++)
//    {
//        if (adis16334_ACC_GetAxes(&adis16334, &acce) == 0)
//        {
//            data[i].type = RT_SENSOR_CLASS_ACCE;
//            data[i].data.acce.x = acce.x;
//            data[i].data.acce.y = acce.y;
//            data[i].data.acce.z = acce.z;
//            data[i].timestamp = rt_sensor_get_ts();
//        }
//        else
//            break;
//    }
    return i;
}

static rt_size_t _sensor_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _adis16334_polling_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_INT)
    {
        return _adis16334_polling_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_FIFO)
    {
        return _adis16334_acc_fifo_get_data(sensor, buf, len);
    }
    else
        return 0;
}

static rt_err_t _sensor_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd) {
        case RT_SENSOR_CTRL_GET_ID: {
            _adis16334_get_id(sensor, (rt_uint16_t*)args);
            break;        
        }
        case RT_SENSOR_CTRL_SET_RANGE: {
            _adis16334_set_range(sensor, *(rt_int32_t *)args);
            break;        
        }
        case RT_SENSOR_CTRL_SET_ODR: {
            _adis16334_set_odr(sensor, *(rt_uint16_t *)args);
            break;
        }
        case RT_SENSOR_CTRL_SET_MODE: {
            _adis16334_set_mode(sensor, *(rt_uint8_t *)args);
            break;
        }
        case RT_SENSOR_CTRL_SET_POWER:
            _adis16334_set_power(sensor, *(rt_uint8_t *)args);
            break;
        case RT_SENSOR_CTRL_SELF_TEST: {
            _adis16334_self_test(sensor, *(rt_uint8_t *)args);
            break;
        }
        default:
            return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    _sensor_fetch_data,
    _sensor_control
};

int rt_hw_adis16334_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_acce = RT_NULL;
    rt_sensor_t sensor_gyro = RT_NULL;

    if (cfg == RT_NULL)
        return -1;

#ifdef BSP_USING_ADIS16334_ACCE
    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -1;

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_acce->info.model      = "adis16334_acc";
        sensor_acce->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_SPI;
        sensor_acce->info.range_max  = ADIS_ACCE_RANGE_5G;
        sensor_acce->info.range_min  = -ADIS_ACCE_RANGE_5G;
        sensor_acce->info.period_min = 1000 / ADIS_SAMPLE_RATE;

        cfg->odr = ADIS_SAMPLE_RATE;            // 1kHz
        cfg->range = ADIS_ACCE_RANGE_5G;
        
        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
    
        sensor_acce->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_INT_RX, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
#endif
#ifdef BSP_USING_ADIS16334_GYRO
    /* gyroscope sensor register */
    {
        sensor_gyro = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_gyro == RT_NULL)
            goto __exit;

        sensor_gyro->info.type       = RT_SENSOR_CLASS_GYRO;
        sensor_gyro->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_gyro->info.model      = "adis16334_gyro";
        sensor_gyro->info.unit       = RT_SENSOR_UNIT_MDPS;
        sensor_gyro->info.intf_type  = RT_SENSOR_INTF_SPI;
        sensor_gyro->info.range_max  = ADIS_GYRO_RANGE_300DPS;
        sensor_gyro->info.range_min  = -ADIS_GYRO_RANGE_300DPS;
        sensor_gyro->info.period_min = 1000 / ADIS_SAMPLE_RATE;
        
        cfg->odr = ADIS_SAMPLE_RATE;            // 1kHz
        cfg->range = ADIS_GYRO_RANGE_300DPS;
        
        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));

        sensor_gyro->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_gyro, name, RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_INT_RX, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
#endif

    result = _adis16334_init(&cfg->intf);
    
    if (result != RT_EOK)
    {
        LOG_E("ADiS16334 init err code: %d", result);
        goto __exit;
    }

    LOG_I("ADiS16334 init success");
    return RT_EOK;

__exit:
    if (sensor_acce)
        rt_free(sensor_acce);
    if (sensor_gyro)
        rt_free(sensor_gyro);
    
    return -RT_ERROR;
}

int _adis_offset(int argc, char **argv)
{
   
    rt_thread_delay(1);    
    LOG_W("ADIS16400_XGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_XGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_YGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_YGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_ZGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_FLASH_CNT: %d", _adis_read_reg(spi_bus_dev, ADIS16400_FLASH_CNT));

    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(_adis_offset, adis_offset, Show Gyro offset);

int _adis_mbc(int argc, char **argv)
{
    rt_size_t flash_cnt = 0;
    rt_int16_t buff[IMU_READ_BRUST_SIZE];
    float fsum[3] = {0.f};
    rt_int16_t offset[3] = {0};
    
    rt_thread_delay(1);    
    LOG_W("ADIS16400_XGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_XGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_YGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_YGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_ZGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF));
    
    LOG_I("start calibration");

    flash_cnt = _adis_read_reg(spi_bus_dev, ADIS16400_FLASH_CNT);
    LOG_W("ADIS16400_FLASH_CNT: %d", flash_cnt);

    LOG_I("start calibration in 3 sec");
    rt_thread_delay(RT_TICK_PER_SECOND * 3);
    
    rt_uint16_t loop_cnt = 1024;
    while(loop_cnt--) {
        rt_thread_delay(RT_TICK_PER_SECOND / 20);    
        _adis_read_brust(spi_bus_dev, buff);
        
        fsum[0] += buff[0] * ADIS_GYRO_RATIO;
        fsum[1] += buff[1] * ADIS_GYRO_RATIO;
        fsum[2] += buff[2] * ADIS_GYRO_RATIO;
        
        LOG_W("%d %d %d", buff[0], buff[1], buff[2]);
    }
    
    offset[0] = (rt_int16_t)(-fsum[0] / 1024.f / 0.0125f);
    offset[1] = (rt_int16_t)(-fsum[1] / 1024.f / 0.0125f);
    offset[2] = (rt_int16_t)(-fsum[2] / 1024.f / 0.0125f);
    LOG_I("finish calibration, %04X %04X %04X", offset[0], offset[1], offset[2]);
    
    rt_thread_delay(1); 
    _adis_write_reg(spi_bus_dev, ADIS16400_XGYRO_OFF, offset[0] & 0x1FFF);
    rt_thread_delay(1); 
    _adis_write_reg(spi_bus_dev, ADIS16400_YGYRO_OFF, offset[1] & 0x1FFF);
    rt_thread_delay(1); 
    _adis_write_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF, offset[2] & 0x1FFF);
    
    rt_thread_delay(1);    
    LOG_W("ADIS16400_XGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_XGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_YGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_YGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_ZGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF));
    
    rt_thread_delay(1); 
    _adis_write_reg(spi_bus_dev, ADIS16400_GLOB_CMD, 0x0008);

    rt_thread_delay(100);
    LOG_W("ADIS16400_FLASH_CNT: %d", _adis_read_reg(spi_bus_dev, ADIS16400_FLASH_CNT));
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(_adis_mbc, adis_mbc, Execute Manual Bias Calibration);

int _adis_setgyro(int argc, char **argv)
{
    rt_thread_delay(1); 
    _adis_write_reg(spi_bus_dev, ADIS16400_XGYRO_OFF, 0x002E);
    rt_thread_delay(1); 
    _adis_write_reg(spi_bus_dev, ADIS16400_YGYRO_OFF, 0xFFD0);
    rt_thread_delay(1); 
    _adis_write_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF, 0x0000);
    
    rt_thread_delay(1);    
    LOG_W("ADIS16400_XGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_XGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_YGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_YGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_ZGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_FLASH_CNT: %d", _adis_read_reg(spi_bus_dev, ADIS16400_FLASH_CNT));
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(_adis_setgyro, adis_setgyro, Execute Manual Bias Calibration);

int _adis_save(int argc, char **argv)
{
    rt_thread_delay(1); 
    _adis_write_reg(spi_bus_dev, ADIS16400_GLOB_CMD, 0x0008);
    
    rt_thread_delay(100);
    LOG_W("ADIS16400_FLASH_CNT: %d", _adis_read_reg(spi_bus_dev, ADIS16400_FLASH_CNT));

    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(_adis_save, adis_save, Execute Manual Bias Calibration);

int _adis_abc(int argc, char **argv)
{
    rt_size_t flash_cnt = 0;
    
    rt_thread_delay(1);    
    LOG_W("ADIS16400_XGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_XGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_YGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_YGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_ZGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF));

    flash_cnt = _adis_read_reg(spi_bus_dev, ADIS16400_FLASH_CNT);
    LOG_W("ADIS16400_FLASH_CNT: %d", flash_cnt);

    LOG_I("start calibration in 3 sec");
    rt_thread_delay(RT_TICK_PER_SECOND * 3);
    LOG_I("calibration start");
    _adis_write_reg(spi_bus_dev, ADIS16400_GLOB_CMD, 0x0001);
    
    rt_tick_t time_cnt = 1;
    while(RT_TRUE) {
        rt_thread_delay(RT_TICK_PER_SECOND * 10);

        if (flash_cnt != _adis_read_reg(spi_bus_dev, ADIS16400_FLASH_CNT)) {
            LOG_I("finish calibration");
            break;
        }
        
        LOG_W("calibration cost %d sec", time_cnt++ * 10);
    }

    rt_thread_delay(1);    
    LOG_W("ADIS16400_XGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_XGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_YGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_YGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_ZGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF));
    
    rt_thread_delay(100);
    LOG_W("ADIS16400_FLASH_CNT: %d", _adis_read_reg(spi_bus_dev, ADIS16400_FLASH_CNT));

    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(_adis_abc, adis_abc, Execute Auto Bias Calibration);

int _adis_restore(int argc, char **argv)
{
    _adis_write_reg(spi_bus_dev, ADIS16400_GLOB_CMD, 0x0002);
    LOG_I("finish restore");
    
    rt_thread_delay(RT_TICK_PER_SECOND);
    
    LOG_W("ADIS16400_XGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_XGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_YGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_YGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_ZGYRO_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_ZGYRO_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_XACCL_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_XACCL_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_YACCL_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_YACCL_OFF));
    rt_thread_delay(1);
    LOG_W("ADIS16400_ZACCL_OFF: 0x%04X", _adis_read_reg(spi_bus_dev, ADIS16400_ZACCL_OFF));
    rt_thread_delay(1);
    
    return RT_EOK;
}
MSH_CMD_EXPORT_ALIAS(_adis_restore, adis_restore, Execute Restore Factory Setting);
