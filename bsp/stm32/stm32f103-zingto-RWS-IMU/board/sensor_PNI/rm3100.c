// SPDX-License-Identifier: GPL-2.0-only
/*
 * rm3100.c	support PNI sensor RM3100
 *		3d Magnetometers via SPI
 */
#include "rm3100.h"
#include <board.h>

#define DBG_ENABLE
#define DBG_LEVEL   DGB_INFO   //DBG_INFO      
#define DBG_SECTION_NAME  "rm3100"
#define DBG_COLOR
#include <rtdbg.h>

#define GEOMAG_DRDY         GET_PIN(C, 4)

static struct rt_spi_device *spi_bus_dev;

static rt_uint8_t _rm3100_read_reg(struct rt_spi_device * spi, rt_uint8_t addr)
{
    rt_uint8_t txbuf[2], rxbuf[2];
    
    txbuf[0] = addr | (1 << 7);
    txbuf[1] = 0x00;
    
    rt_spi_transfer(spi, txbuf, rxbuf, 2);
    
    return rxbuf[1];
}

static rt_err_t _rm3100_write_reg(struct rt_spi_device * spi, rt_uint8_t addr, rt_uint8_t val)
{
    rt_uint8_t txbuf[2];
    
    txbuf[0] = addr & ~(1 << 7);
    txbuf[1] = val;
    return rt_spi_send(spi, txbuf, 2);
}

static rt_err_t _rm3100_write_brust(struct rt_spi_device * spi, rt_uint8_t addr, rt_uint8_t *pbuf, rt_size_t bufsz)
{
    if (bufsz <= 0) return -RT_EINVAL;
    rt_uint8_t mask = addr & ~(1 << 7);   
    return rt_spi_send_then_send(spi, &mask, 1, pbuf, bufsz);
}

static rt_err_t _rm3100_read_brust(struct rt_spi_device * spi, rt_uint8_t addr, rt_uint8_t *pbuf, rt_size_t bufsz)
{
    if (bufsz <= 0) return -RT_EINVAL;
    rt_uint8_t mask = addr | (1 << 7);
    return rt_spi_send_then_recv(spi, &mask, 1, pbuf, bufsz);
}

static rt_err_t _rm3100_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t revID;
    
    RT_ASSERT(intf != RT_NULL);
    
    if (spi_bus_dev == RT_NULL) {
        spi_bus_dev = (struct rt_spi_device *)rt_device_find(intf->dev_name);

        if (spi_bus_dev == RT_NULL) {
            return -RT_ERROR;
        }

        spi_bus_dev->config.data_width = 8;
        spi_bus_dev->config.mode = RT_SPI_MASTER | RT_SPI_MSB | RT_SPI_MODE_0;
        spi_bus_dev->config.max_hz = 500 * 1000;

        LOG_D("config SPI parameter for RM3100");
    }
    
    rt_device_open((rt_device_t)spi_bus_dev->bus, RT_DEVICE_OFLAG_RDWR);
    rt_device_open((rt_device_t)spi_bus_dev, RT_DEVICE_OFLAG_RDWR);

    revID = _rm3100_read_reg(spi_bus_dev, RM3100_REVID);
    if (revID == RM3100_REVID_VALUE) {
        LOG_I("Found the GeoMag Sensor...");
        rt_uint8_t CC_DATA[6] = {0, 200, 0, 200, 0, 200};
        _rm3100_write_brust(spi_bus_dev, RM3100_CCX, CC_DATA, sizeof(CC_DATA));
        LOG_D("CCX: 0x%02X%02X"
                , _rm3100_read_reg(spi_bus_dev, RM3100_CCX)
                , _rm3100_read_reg(spi_bus_dev, RM3100_CCX + 1));
        LOG_D("CCY: 0x%02X%02X"
                , _rm3100_read_reg(spi_bus_dev, RM3100_CCY)
                , _rm3100_read_reg(spi_bus_dev, RM3100_CCY + 1));
        LOG_D("CCZ: 0x%02X%02X"
                , _rm3100_read_reg(spi_bus_dev, RM3100_CCZ)
                , _rm3100_read_reg(spi_bus_dev, RM3100_CCZ + 1));
        _rm3100_write_reg(spi_bus_dev, RM3100_TMRC, 0x96);
        LOG_D("TMRC: 0x%02X", _rm3100_read_reg(spi_bus_dev, RM3100_TMRC));
        _rm3100_write_reg(spi_bus_dev, RM3100_HSHAKE, 0x0B);
        LOG_D("HSHAKE: 0x%02X", _rm3100_read_reg(spi_bus_dev, RM3100_HSHAKE));
    }
    else {
        LOG_E("wrong RM3100_REVID: %02X, init failed!", revID);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t _rm3100_get_id(rt_sensor_t sensor, rt_uint8_t * buff) {
    *buff = _rm3100_read_reg(spi_bus_dev, RM3100_REVID);
    return RT_EOK;
}

static rt_err_t _rm3100_set_range(rt_sensor_t sensor, rt_int32_t range) {
    return RT_EOK;
}

static rt_err_t _rm3100_set_odr(rt_sensor_t sensor, rt_uint16_t odr) {
    return RT_EOK;
}

static rt_err_t _rm3100_set_mode(rt_sensor_t sensor, rt_uint8_t mode) {
    LOG_D("RM3100 start Continuous Measurement.");
    _rm3100_write_reg(spi_bus_dev, RM3100_CMM, 0x79);
    return RT_EOK;
}

static rt_err_t _rm3100_set_power(rt_sensor_t sensor, rt_uint8_t power) {
    return RT_EOK;
}

static rt_err_t _rm3100_self_test(rt_sensor_t sensor, rt_uint8_t func) {
    return RT_EOK;
}

static rt_size_t _rm3100_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    rt_uint8_t rxbuf[GEOMAG_RAWDATA_READ_SIZE];
    rt_uint32_t tmpval;
    
    LOG_D("read RM3100 realtime data");
    _rm3100_read_brust(spi_bus_dev, RM3100_MX, rxbuf, GEOMAG_RAWDATA_READ_SIZE);
    
    data->type = RT_SENSOR_CLASS_MAG;
    
    tmpval = (rt_uint32_t)rxbuf[0] << 16 | (rt_uint32_t)rxbuf[1] << 8 | (rt_uint32_t)rxbuf[2];
    data->data.mag.x = (tmpval >= 0x00800000)?(tmpval | 0xFF000000):tmpval;
    tmpval = (rt_uint32_t)rxbuf[3] << 16 | (rt_uint32_t)rxbuf[4] << 8 | (rt_uint32_t)rxbuf[5];
    data->data.mag.y = (tmpval >= 0x00800000)?(tmpval | 0xFF000000):tmpval;
    tmpval = (rt_uint32_t)rxbuf[6] << 16 | (rt_uint32_t)rxbuf[7] << 8 | (rt_uint32_t)rxbuf[8];
    data->data.mag.z = (tmpval >= 0x00800000)?(tmpval | 0xFF000000):tmpval;
    data->timestamp = rt_sensor_get_ts();
    
    return 1;
}

static rt_size_t _rm3100_fifo_get_data(rt_sensor_t sensor, struct rt_sensor_data *data, rt_size_t len) {
    return 0;
}

static rt_size_t _sensor_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _rm3100_polling_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_INT)
    {
        return _rm3100_polling_get_data(sensor, buf);
    }
    else if (sensor->config.mode == RT_SENSOR_MODE_FIFO)
    {
        return _rm3100_fifo_get_data(sensor, buf, len);
    }
    else
        return 0;
}

static rt_err_t _sensor_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd) {
        case RT_SENSOR_CTRL_GET_ID:
            _rm3100_get_id(sensor, (rt_uint8_t *)args); break;        
        case RT_SENSOR_CTRL_SET_RANGE:
            _rm3100_set_range(sensor, *(rt_int32_t *)args); break;        
        case RT_SENSOR_CTRL_SET_ODR:
            _rm3100_set_odr(sensor, *(rt_uint16_t *)args); break;
        case RT_SENSOR_CTRL_SET_MODE:
            _rm3100_set_mode(sensor, *(rt_uint8_t *)args); break;
        case RT_SENSOR_CTRL_SET_POWER:
            _rm3100_set_power(sensor, *(rt_uint8_t *)args); break;
        case RT_SENSOR_CTRL_SELF_TEST:
            _rm3100_self_test(sensor, *(rt_uint8_t *)args); break;
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

int rt_hw_rm3100_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_geom = RT_NULL;

    if (cfg == RT_NULL)
        return -1;

#ifdef BSP_USING_RM3100
    /* geomagnetic sensor register */
    {
        sensor_geom = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_geom == RT_NULL)
            return -1;

        sensor_geom->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_geom->info.vendor     = RT_SENSOR_VENDOR_UNKNOWN;
        sensor_geom->info.model      = "rm3100_geomag";
        sensor_geom->info.unit       = RT_SENSOR_UNIT_MGAUSS;
        sensor_geom->info.intf_type  = RT_SENSOR_INTF_SPI;
        sensor_geom->info.range_max  = RM3100_GEOMAG_DATA_MAX;
        sensor_geom->info.range_min  = RM3100_GEOMAG_DATA_MIN;
        sensor_geom->info.period_min = 1000 / BSP_RM3100_DATA_RATE;

        cfg->odr = BSP_RM3100_DATA_RATE;            // 1kHz
        cfg->range = RM3100_GEOMAG_DATA_MAX;
        
        rt_memcpy(&sensor_geom->config, cfg, sizeof(struct rt_sensor_config));
    
        sensor_geom->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_geom, name, RT_DEVICE_FLAG_RDONLY | RT_DEVICE_FLAG_INT_RX, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }
#endif

    result = _rm3100_init(&cfg->intf);
    
    if (result != RT_EOK)
    {
        LOG_E("RM3100 init err code: %d", result);
        goto __exit;
    }

    LOG_I("RM3100 init success");
    return RT_EOK;

__exit:
    if (sensor_geom)
        rt_free(sensor_geom);
    
    return -RT_ERROR;
}
