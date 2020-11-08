
#ifndef SENSOR_ADI_ADIS16334_H__
#define SENSOR_ADI_ADIS16334_H__

#include "sensor.h"
#include "adis.h"
#include <drv_spi.h>

#define ADIS16334_ADDR_DEFAULT (0xD6 >> 1)

#define ADIS_SAMPLE_RATE        (500.f)

#define ADIS_ACCE_RANGE_5G      (5000)  // -5000 to 5000, 0.001  G/LSB
#define ADIS_GYRO_RANGE_300DPS  (6000)  // -6000 to 6000, 0.05 DPS/LSB

#define ADIS_ACCE_RATIO         (0.001f)
#define ADIS_GYRO_RATIO         (0.05f)

int rt_hw_adis16334_init(const char *name, struct rt_sensor_config *cfg);

#endif
