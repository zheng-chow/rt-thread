/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 *
 */

#ifndef __RN8302B_H__
#define __RN8302B_H__

#include <board.h>

#define EConst  (3200)
#define Ust     (24000000L)      // 0x016E3600   120V
#define Ist     (8000000L)       // 0x007A1200   3A
#define Pst     (22888183)       // 120V * 3A    360W   (Ust * Ist) / 2^23

#define CONFIG_FILENAME     "jc_config.json"
#define CONFIG_JSON_SIZE    (2048)

#define WAVE_SAMPLE_COUNT               (128)
#define FFT_PI                          (3.1415926f)

#define HARMONIC_NUMBER                 (20)

#define JCDEV_SEARCH_BYNAME             (0)
#define JCDEV_SOFT_RESET                (1)

#define JCDEV_CALI_ENV_INITIAL          (2)
#define JCDEV_CALI_U_GAIN               (3)
#define JCDEV_CALI_I_GAIN               (4)
#define JCDEV_CALI_PHASE_0D5L           (5)
#define JCDEV_CALI_U_OFFSET             (6)
#define JCDEV_CALI_I_OFFSET             (7)
#define JCDEV_CALI_DC_OFFSET            (8)
#define JCDEV_EXEC_WAVE_FFT             (9)
#define JCDEV_SAVE_CONFIG               (10)
#define JCDEV_LOAD_CONFIG               (11)

#define RN8302B_HFCONST                 (0x3D0F)

struct jcdev_reg_t
{
    rt_off_t    addr;
    char        *desc;
    rt_size_t   size;
    rt_bool_t   writeable;
    float       ratio;
    char        *detail;
};

struct jcdev_harmonic_t
{
    rt_tick_t timestamp;
    
    float ThdPhV[3];
    float ThdA[3];
    float phsA_HphV[HARMONIC_NUMBER];
    float phsB_HphV[HARMONIC_NUMBER];
    float phsC_HphV[HARMONIC_NUMBER];
    float phsA_HA[HARMONIC_NUMBER];
    float phsB_HA[HARMONIC_NUMBER];
    float phsC_HA[HARMONIC_NUMBER];
};

struct jcdev_stdef_t
{
    rt_uint16_t HFConst;
    // UV gain.
    rt_uint16_t GSUA;
    rt_uint16_t GSUB;
    rt_uint16_t GSUC;
    // IA gain.
    rt_uint16_t GSIA;
    rt_uint16_t GSIB;
    rt_uint16_t GSIC;
    rt_uint16_t GSIN;
    // Phase offset
    rt_uint16_t PHSIA;
    rt_uint16_t PHSIB;
    rt_uint16_t PHSIC;
    
    rt_uint16_t PRTH1L;
    rt_uint16_t PRTH1H;
    rt_uint16_t PRTH2L;
    rt_uint16_t PRTH2H;
    
    rt_uint16_t GPA;
    rt_uint16_t GPB;
    rt_uint16_t GPC;
    
    rt_uint16_t PA_PHSL;
    rt_uint16_t PB_PHSL;
    rt_uint16_t PC_PHSL;
    
    rt_uint16_t QA_PHSL;
    rt_uint16_t QB_PHSL;
    rt_uint16_t QC_PHSL;

    rt_uint16_t GQA;
    rt_uint16_t GQB;
    rt_uint16_t GQC;

    rt_uint16_t GSA;
    rt_uint16_t GSB;
    rt_uint16_t GSC;
    
    rt_uint16_t PA_OS;
    rt_uint16_t PB_OS;
    rt_uint16_t PC_OS;
    
    rt_uint16_t IA_OS;
    rt_uint16_t IB_OS;
    rt_uint16_t IC_OS;
    
    rt_uint16_t UA_OS;
    rt_uint16_t UB_OS;
    rt_uint16_t UC_OS;
    
    rt_uint16_t DCOS_UA;
    rt_uint16_t DCOS_UB;
    rt_uint16_t DCOS_UC;
    
    rt_uint16_t DCOS_IA;
    rt_uint16_t DCOS_IB;
    rt_uint16_t DCOS_IC;
};

#endif
