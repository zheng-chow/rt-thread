
#ifndef __SENSOR_PNI_RM3100_H__
#define __SENSOR_PNI_RM3100_H__

#include "sensor.h"
#include <drv_spi.h>

#define GEOMAG_CHANNEL_COUNT        3
#define GEOMAG_RAWDATA_READ_SIZE    9

#define RM3100_GEOMAG_DATA_MAX     (8388607)   // 0x007FFFFF
#define RM3100_GEOMAG_DATA_MIN     (-8388608)  // 0xFF800000
/* 1 mGAUSS = 0.1 uTesla */
#define RM3100_MEASURE_CYCLE        (200)
#define RM3100_MCC_200_UNIT         (10.0f / 75) // 75 LSB/uT 

/* Register Map Table */
#define RM3100_POLL     (0x00) //RW, Polls for a Single Measurement
#define RM3100_CMM      (0x01) //RW, Initiates Continuous Measurement Mode
#define RM3100_CCX      (0x04) //RW, Cycle Count Register - X Axis
#define RM3100_CCY      (0x06) //RW, Cycle Count Register - Y Axis
#define RM3100_CCZ      (0x08) //RW, Cycle Count Register - Z Axis
#define RM3100_TMRC     (0x0B) //RW, Set Continuous Measurement Mode Data Rate
#define RM3100_ALLX     (0x0C) //RW, Alarm Lower Limit - X Axis
#define RM3100_AULX     (0x0F) //RW, Alarm Upper Limit - X Axis
#define RM3100_ALLY     (0x12) //RW, Alarm Lower Limit - Y Axis
#define RM3100_AULY     (0x15) //RW, Alarm Upper Limit - Y Axis
#define RM3100_ALLZ     (0x18) //RW, Alarm Lower Limit - Z Axis
#define RM3100_AULZ     (0x1B) //RW, Alarm Upper Limit - Z Axis
#define RM3100_ADLX     (0x1E) //RW, Alarm Hysteresis Value - X Axis
#define RM3100_ADLY     (0x20) //RW, Alarm Hysteresis Value - Y Axis
#define RM3100_ADLZ     (0x22) //RW, Alarm Hysteresis Value - Z Axis
#define RM3100_MX       (0x24) //RO, Measurement Results - X Axis
#define RM3100_MY       (0x27) //RO, Measurement Results - Y Axis
#define RM3100_MZ       (0x2A) //RO, Measurement Results - Z Axis
#define RM3100_BIST     (0x33) //RW, Built-In Self Test
#define RM3100_STATUS   (0x34) //RO, Status of DRDY
#define RM3100_HSHAKE   (0x35) //RW, HandShake Register
#define RM3100_REVID    (0x36) //RO, MagI2C Revision Identification
/* Enum Values */
#define RM3100_REVID_VALUE      (0x22)


int rt_hw_rm3100_init(const char *name, struct rt_sensor_config *cfg);

#endif
