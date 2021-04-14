/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-05     serni        first version
 */
 
#include <board.h>
#include <drv_spi.h>
#include <rtdevice.h>
#include <rthw.h>
#include <finsh.h>

#include "rn8302b.h"

//#define DRV_DEBUG
#define LOG_TAG      "drv.jcdev"
#include <drv_log.h>

#define JC_CHIP_TYPE "rn8302b"

#define JCDEV_PM_PIN     GET_PIN(E, 2)
#define JCDEV_RST_PIN    GET_PIN(E, 4)

extern const struct jcdev_reg_t RN8302B_REGLIST[];
extern const rt_size_t RN8302B_REGCOUNT;

static struct rt_spi_device     *jc_spi_dev = RT_NULL;
static struct jcdev_stdef_t     _jc_std_cfg = {0};
static struct jcdev_harmonic_t  _jc_harmonic = {0};
static struct jcdev_energy_t    _jc_energy = {0};

static float *sin_tab;
static float *cos_tab;

static void _FFT(float dataR[WAVE_SAMPLE_COUNT], float dataI[WAVE_SAMPLE_COUNT])
{
	int x0,x1,x2,x3,x4,x5,x6,xx;
	int i,j,k,b,p,L;
	float TR,TI,temp;
	
	/********** following code invert sequence ************/
	for ( i=0;i<WAVE_SAMPLE_COUNT;i++ )
	{
		x0=x1=x2=x3=x4=x5=x6=0;
		x0=i&0x01; x1=(i/2)&0x01; x2=(i/4)&0x01; x3=(i/8)&0x01;x4=(i/16)&0x01; x5=(i/32)&0x01; x6=(i/64)&0x01;
		xx=x0*64+x1*32+x2*16+x3*8+x4*4+x5*2+x6;
		dataI[xx]=dataR[i];
	}
	for ( i=0;i<WAVE_SAMPLE_COUNT;i++ )
	{
		dataR[i]=dataI[i]; dataI[i]=0; 
	}

	/************** following code FFT *******************/
	for ( L=1;L<=7;L++ )
	{ /* for(1) */
		b=1; i=L-1;
		while ( i>0 ) 
		{
			b=b*2; i--;
		} /* b= 2^(L-1) */
		for ( j=0;j<=b-1;j++ ) /* for (2) */
		{
			p=1; i=7-L;
			while ( i>0 ) /* p=pow(2,7-L)*j; */
			{
				p=p*2; i--;
			}
			p=p*j;
			for ( k=j;k<128;k=k+2*b ) /* for (3) */
			{
				TR=dataR[k]; TI=dataI[k]; temp=dataR[k+b];
				dataR[k]=dataR[k]+dataR[k+b]*cos_tab[p]+dataI[k+b]*sin_tab[p];
				dataI[k]=dataI[k]-dataR[k+b]*sin_tab[p]+dataI[k+b]*cos_tab[p];
				dataR[k+b]=TR-dataR[k+b]*cos_tab[p]-dataI[k+b]*sin_tab[p];
				dataI[k+b]=TI+temp*sin_tab[p]-dataI[k+b]*cos_tab[p];
			} /* END for (3) */
		} /* END for (2) */
	} /* END for (1) */
} /* END FFT */

static rt_err_t _jcdev_reg_read(rt_uint16_t addr, rt_uint8_t *pbuf, rt_uint8_t size)
{
    struct rt_spi_message msg1, msg2;
    rt_uint8_t rdata[8] = {0};
    
    RT_ASSERT(jc_spi_dev != RT_NULL);
    RT_ASSERT(pbuf != RT_NULL);
    
    rdata[0] = (addr & 0x00FF);               // reg
    rdata[1] = (addr & 0x0F00) >> 4;          // bank + R'0
    
    msg1.send_buf = rdata;
    msg1.recv_buf = RT_NULL;
    msg1.length = 2;
    msg1.cs_take = 1;
    msg1.cs_release = 0;
    msg1.next = &msg2;
    
    msg2.send_buf = RT_NULL;
    msg2.recv_buf = pbuf;
    msg2.length = size + 1;
    msg2.cs_take = 0;
    msg2.cs_release = 1;
    msg2.next = RT_NULL;
    
    rt_spi_transfer_message(jc_spi_dev, &msg1);
    
    rt_uint8_t checksum = 0;
    
    checksum += rdata[0];
    checksum += rdata[1];
    
    for(rt_uint8_t i = 0; i < size; i++)
        checksum += pbuf[i];

    checksum = ~checksum;
    
    rt_memcpy(&rdata[2], pbuf, size + 1);
    //LOG_HEX("rdata", 8, rdata, size + 2 + 1);
    
    if (checksum != pbuf[size]) {
        LOG_E("jcdev read checksum error!");
        return -RT_EIO;
    }
    
    return RT_EOK;
}

static rt_err_t _jcdev_brust_read(rt_uint16_t addr, rt_uint8_t *pbuf)
{
    const rt_uint8_t BRUST_READ_SIZE = 16;
    struct rt_spi_message msg1, msg2;
    rt_uint8_t rdata[8] = {0};
    rt_uint8_t brust[50] = {0};
    
    RT_ASSERT(jc_spi_dev != RT_NULL);
    RT_ASSERT(pbuf != RT_NULL);
    
    rdata[0] = (addr & 0x00FF);               // reg
    rdata[1] = (addr & 0x0F00) >> 4;          // bank + R'0
    rdata[1] |= 0x0C;   //brust read, size 16
    
    msg1.send_buf = rdata;
    msg1.recv_buf = RT_NULL;
    msg1.length = 2;
    msg1.cs_take = 1;
    msg1.cs_release = 0;
    msg1.next = &msg2;
    
    msg2.send_buf = RT_NULL;
    msg2.recv_buf = brust;
    msg2.length = BRUST_READ_SIZE*3 + 1;
    msg2.cs_take = 0;
    msg2.cs_release = 1;
    msg2.next = RT_NULL;
    
    rt_spi_transfer_message(jc_spi_dev, &msg1);
    
    rt_uint8_t checksum = 0;
    
    checksum += rdata[0];
    checksum += rdata[1];
    
    for(rt_uint8_t i = 0; i < BRUST_READ_SIZE * 3; i++)
        checksum += brust[i];

    checksum = ~checksum;
    
    if (checksum != brust[BRUST_READ_SIZE*3]) {
        LOG_E("jcdev brust_read checksum error!");
        return -RT_EIO;
    }

    rt_memcpy(pbuf, brust, BRUST_READ_SIZE*3);
    
    return RT_EOK;
}

static rt_err_t _jcdev_reg_write(rt_uint16_t addr, const rt_uint8_t *pbuf, rt_uint8_t size)
{
    struct rt_spi_message msg1;
    rt_uint8_t wdata[16] = {0};
    
    RT_ASSERT(jc_spi_dev != RT_NULL);
    RT_ASSERT(pbuf != RT_NULL);
    
    wdata[0] = (addr & 0x00FF);               // reg
    wdata[1] = (addr & 0x0F00) >> 4 | 0x80;   // bank + W'1
    
    rt_memcpy(&wdata[2], pbuf, size);
    
    rt_uint8_t checksum = 0;
    
    checksum += wdata[0];
    checksum += wdata[1];
    
    for (rt_uint8_t i = 2; i < size + 2; i++)
        checksum += wdata[i];
    
    wdata[size + 2] = ~checksum;
    
    //  LOG_HEX("wdata", 8, wdata, size + 2 + 1);
    
    msg1.send_buf = wdata;
    msg1.recv_buf = RT_NULL;
    msg1.length = size + 2 + 1;
    msg1.cs_take = 1;
    msg1.cs_release = 1;
    msg1.next = RT_NULL;
   
    rt_spi_transfer_message(jc_spi_dev, &msg1);
    
    return RT_EOK;
}

static rt_err_t _jcdev_search_reg(struct jcdev_reg_t *reg)
{
    if (reg->desc == RT_NULL) {
        LOG_E("search register need a desc!");
        return -RT_ENOMEM;
    }
    
    for (int i = 0; i < RN8302B_REGCOUNT; i++) {
        
        if (rt_strcmp(reg->desc, (RN8302B_REGLIST + i)->desc) == 0) {
            reg->addr =         (RN8302B_REGLIST + i)->addr;
            reg->size =         (RN8302B_REGLIST + i)->size;
            reg->writeable =    (RN8302B_REGLIST + i)->writeable;
            reg->ratio =        (RN8302B_REGLIST + i)->ratio;
            reg->detail =       (RN8302B_REGLIST + i)->detail;
            
            return RT_EOK;
        }
    }
    
    return -RT_EEMPTY;
}

static void _jcdev_config_reg(char *desc, rt_uint8_t *buff)
{
    struct jcdev_reg_t reg = {0};

    if (desc == RT_NULL) {
        LOG_E("config register need a desc!");
        return ;
    }
    
    if (buff == RT_NULL) {
        LOG_E("config register need a buff!");
        return ;
    }
    
    reg.desc = desc;
    
    if (_jcdev_search_reg(&reg) == RT_EOK) {
        _jcdev_reg_write(reg.addr, buff, reg.size);
        LOG_D("config %s \twith %d byte(s), 0x%02X 0x%02X", desc, reg.size, buff[0], buff[1]);
    }
    else {
        LOG_W("config %s \tfailed, desc not found!", desc);
    }
    
    return;
}

static rt_size_t _jcdev_access_reg(char *desc, rt_uint8_t *buff)
{
    struct jcdev_reg_t reg = {0};
    
    if (desc == RT_NULL) {
        LOG_E("request register need a desc!");
        return 0;
    }
    
    if (buff == RT_NULL) {
        LOG_E("request register need a buff!");
        return 0;
    }
    
    reg.desc = desc;
    
    if (_jcdev_search_reg(&reg) == RT_EOK) {
        _jcdev_reg_read(reg.addr, buff, reg.size);
        LOG_D("%s access %d byte(s).", desc, reg.size);
    }
    else {
        LOG_W("access %s failed, desc not found!", desc);
    }
    
    return reg.size;
}

rt_int32_t _raw_to_int(rt_uint8_t *pbuf, rt_uint8_t bit)
{
    if (pbuf == RT_NULL) {
        LOG_E("buffer to convert is NULL!");
        return 0;
    }
    
    rt_int32_t s32 = 0;
    rt_uint8_t *u8p = (rt_uint8_t *)&s32;
    rt_uint8_t byte = (bit + 7) / 8;
    for (rt_uint8_t n = 0; n < byte; n++){
      *(u8p + n) = *(pbuf + byte - 1 - n);
    }

    rt_uint32_t MASK = 0;
    bit = bit - 1;      // base 0;
    
    
    for (rt_uint8_t i = bit; i < 32; i++) {
        MASK |= (1 << i);
    }
    
    if ( s32 & (1 << bit)) {
        s32 |= MASK;
    }
    else {
        s32 &= ~MASK;
    }

    return s32;
}

rt_uint32_t _raw_to_uint(rt_uint8_t *pbuf, rt_uint8_t bit)
{
    if (pbuf == RT_NULL) {
        LOG_E("buffer to convert is NULL!");
        return 0;
    }
    
    rt_uint32_t u32 = 0;
    rt_uint8_t *u8p = (rt_uint8_t *)&u32;
    rt_uint8_t byte = (bit + 7) / 8;
    for (rt_uint8_t n = 0; n < byte; n++){
      *(u8p + n) = *(pbuf + byte - 1 - n);
    }

    return u32;
}

static void _jcdev_wave_sample(void)
{
    rt_uint8_t buff[4];
    
    rt_memset(buff, 0, sizeof(buff));
    buff[0] = 0xE5; _jcdev_config_reg("WREN", buff);  // write enable.
    buff[0] = 0xA0; _jcdev_config_reg("WSAVECON", buff); // start wave sample
    buff[0] = 0xDC; _jcdev_config_reg("WREN", buff);  // write disable.
    
    while (RT_TRUE) {
        rt_thread_delay( 5 );
        _jcdev_access_reg("WSAVECON", buff);
        if (buff[0] == 0x80) {
            LOG_D("wave sample finish!");
            break;
        }
        LOG_W("wave sampling... 0x%02X", buff[0]);
    }
}

static float _jcdev_raw_to_float(struct jcdev_reg_t *reg, rt_uint8_t *rawbuf)
{
    float ftv = 0.f;
    
    if (reg->detail[0] == 'S') {
        if (rt_strlen(reg->detail) > 2) {
            // calc the effective bit length
            rt_uint8_t ebsz = atoi(&reg->detail[1]);
            LOG_D("%s is S and %d", reg->detail, ebsz);
            
            ftv = reg->ratio * _raw_to_int(rawbuf, ebsz);
        }
    }
    else if (reg->detail[0] == 'U') {
        if (rt_strlen(reg->detail) > 2) {
            if (rt_strcmp("^-1", &reg->detail[3]) == 0) {
                // UFreq only.
                rt_uint32_t uTemp = (rawbuf[0] << 16) + (rawbuf[1] << 8) + rawbuf[2];
                ftv = reg->ratio / uTemp;
            }
            else {
                rt_uint8_t ebsz = atoi(&reg->detail[1]);
                LOG_D("%s is U and %d", reg->detail, ebsz);
                
                if (ebsz < 25) {
                    rt_uint32_t uTemp = (rawbuf[0] << 16) + (rawbuf[1] << 8) + rawbuf[2];
                    ftv = reg->ratio * uTemp;
                }
                else {
                    ftv = reg->ratio * _raw_to_uint(rawbuf, ebsz);
                }                
            }
        }
    }
    else {
        LOG_W("reg->desc with unrecognized char");
    }
    
    return ftv;
}

static float _jcdev_wave_fft(char * chdesc, float harmonic[HARMONIC_NUMBER])
{
    rt_uint8_t *rawbuff = RT_NULL;
    rt_uint16_t rawbuf_addr = RT_NULL;
    float *real_array = RT_NULL;
    float *mega_array = RT_NULL;
    struct jcdev_reg_t target;
    float rms = 0.f;
    float ftv = 0.f;
    float chv = 0.f;
    rt_uint8_t regbuf[8];
    
    RT_ASSERT(chdesc != RT_NULL);

    target.desc = chdesc;
    if (_jcdev_search_reg(&target) != RT_EOK) {
        LOG_E("fft search reg %s failed!", target.desc);
        return 0.0f;
    }
    
    _jcdev_reg_read(target.addr, regbuf, target.size);
    chv = _jcdev_raw_to_float(&target, regbuf);
    
    if (target.desc[0] == 'U') {
        if (fabs(chv) < 10.f) {  // if no UV input, exit.
            rt_memset(harmonic, 0x00, HARMONIC_NUMBER * sizeof(float));
            return 0.0f;
        }
        
        if (target.desc[1] == 'A')
            rawbuf_addr = 0x0200;
        else if (target.desc[1] == 'B')
            rawbuf_addr = 0x0280;
        else if (target.desc[1] == 'C')
            rawbuf_addr = 0x0300;
        else {
            goto __exit;
        }
    }
    else if (target.desc[0] == 'I') {
        if (fabs(chv) < 0.1f) {  // if no IA input, exit.
            rt_memset(harmonic, 0x00, HARMONIC_NUMBER * sizeof(float));
            return 0.0f;
        }
        
        if (target.desc[1] == 'A')
            rawbuf_addr = 0x0380;
        else if (target.desc[1] == 'B')
            rawbuf_addr = 0x0400;
        else if (target.desc[1] == 'C')
            rawbuf_addr = 0x0480;
        else {
            goto __exit;
        }
    }
    else {
        goto __exit;
    }
    
    // malloc buffer.
    rawbuff = rt_malloc( WAVE_SAMPLE_COUNT * 3 );    // S24
    real_array = rt_malloc( WAVE_SAMPLE_COUNT * sizeof(float));
    mega_array = rt_malloc( WAVE_SAMPLE_COUNT * sizeof(float));

    if (rawbuff == RT_NULL) {
        goto __exit;
    }
    if (real_array == RT_NULL) {
        goto __exit;
    }
    if (mega_array == RT_NULL) {
        goto __exit;
    }
    
    for (rt_uint8_t i = 0; i < WAVE_SAMPLE_COUNT / 16; i++) {
        _jcdev_brust_read((rawbuf_addr + i*16), &rawbuff[i*48]);
    }
    
    for (rt_uint8_t i = 0; i < WAVE_SAMPLE_COUNT; i++) {
        real_array[i] = target.ratio * _raw_to_int(&rawbuff[i*3], 24);
        mega_array[i] = 0.f;
    }
    
    _FFT(real_array, mega_array);
    
    ftv = real_array[1] * real_array[1] + mega_array[1] * mega_array[1];
    harmonic[0] = sqrt(ftv);  // base wave level;

    for (rt_uint8_t i = 2; i < HARMONIC_NUMBER; i++) {
        ftv = real_array[i] * real_array[i] + mega_array[i] * mega_array[i];
        harmonic[i - 1] = sqrt(ftv) / harmonic[0];
    }
    
    ftv = 0.f;
    for (rt_uint8_t i = 1; i < HARMONIC_NUMBER; i++) {
        ftv += harmonic[i] * harmonic[i];
    }
    rms = sqrt(ftv);

__exit:
    if (rawbuff) {
        rt_free(rawbuff);
        LOG_D("free rawbuff!");
    }
    if (real_array) {
        rt_free(real_array);
        LOG_D("free real_array!");
    }
    if (mega_array) {
        rt_free(mega_array);
        LOG_D("free mega_array!");
    }
 
    return rms;
}


#include "cJSON.h"

#include <dfs.h>
#include <dfs_file.h>
#include <dfs_posix.h>

static rt_err_t _jcdev_load_config(char *filename)
{
    int fd = -1;
    rt_size_t size;
    cJSON *root;
    char *json;
    char *item;
    rt_uint32_t hex;
    
    RT_ASSERT(filename != RT_NULL);
    
    fd = open(filename, O_RDONLY, 0);
    if (fd < 0) {
        LOG_E("read config failed! maybe there's no file?");
        close(fd);
        return -RT_EIO;
    }
    
    json = rt_malloc(CONFIG_JSON_MAXSIZE);
    if (json == RT_NULL) {
        LOG_E("malloc buffer for json failed!");
        close(fd);
        return -RT_ENOMEM;
    }
    
    size = read(fd, json, CONFIG_JSON_MAXSIZE);
    if (size == CONFIG_JSON_MAXSIZE)
        LOG_W("json buffer is full!");
    
    close(fd);
    
    root = cJSON_Parse(json);
    rt_free(json);
    
    if (root == RT_NULL) {
        LOG_E("parse json failed.");
        cJSON_Delete(root);
        return -RT_EIO;
    }

    if (cJSON_GetObjectItem(root, "HFConst") == RT_NULL) goto exit;    
    item = cJSON_GetObjectItem(root, "HFConst")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.HFConst = hex;
    LOG_I("load from config, HFConst = 0x%04X", _jc_std_cfg.HFConst);

    if (cJSON_GetObjectItem(root, "GSUA") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "GSUA")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.GSUA = hex;
    LOG_I("load from config, GSUA = 0x%04X", _jc_std_cfg.GSUA);

    if (cJSON_GetObjectItem(root, "GSUB") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "GSUB")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.GSUB = hex;
    
    if (cJSON_GetObjectItem(root, "GSUC") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "GSUC")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.GSUC = hex;
    
    if (cJSON_GetObjectItem(root, "GSIA") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "GSIA")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.GSIA = hex;
    
    if (cJSON_GetObjectItem(root, "GSIB") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "GSIB")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.GSIB = hex;
    
    if (cJSON_GetObjectItem(root, "GSIC") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "GSIC")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.GSIC = hex;
    
    if (cJSON_GetObjectItem(root, "GSIN") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "GSIN")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.GSIN = hex;
    
    if (cJSON_GetObjectItem(root, "PA_PHSL") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "PA_PHSL")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.PA_PHSL = hex;
    LOG_I("load from config, PA_PHSL = 0x%04X", _jc_std_cfg.PA_PHSL);

    if (cJSON_GetObjectItem(root, "QA_PHSL") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "QA_PHSL")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.QA_PHSL = hex;

    if (cJSON_GetObjectItem(root, "PB_PHSL") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "PB_PHSL")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.PB_PHSL = hex;

    if (cJSON_GetObjectItem(root, "QB_PHSL") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "QB_PHSL")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.QB_PHSL = hex;

    if (cJSON_GetObjectItem(root, "PC_PHSL") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "PC_PHSL")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.PC_PHSL = hex;
    
    if (cJSON_GetObjectItem(root, "QC_PHSL") == RT_NULL) goto exit;
    item = cJSON_GetObjectItem(root, "QC_PHSL")->valuestring;
    sscanf(item, "0x%04X", &hex);
    _jc_std_cfg.QC_PHSL = hex;
    
    LOG_I("load from %s", filename);

exit:    
    cJSON_Delete(root);
    
    return RT_EOK;
}

static rt_err_t _jcdev_save_config(char *filename)
{
    const rt_size_t BUFSZ = 32;
    cJSON *root;
    char strbuf[BUFSZ];
    char *json;
    int fd = -1;
    
    RT_ASSERT(filename != RT_NULL);
    
    root = cJSON_CreateObject();
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.HFConst);
    cJSON_AddItemToObject(root, "HFConst", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.GSUA);
    cJSON_AddItemToObject(root, "GSUA", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.GSUB);
    cJSON_AddItemToObject(root, "GSUB", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.GSUC);
    cJSON_AddItemToObject(root, "GSUC", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.GSIA);
    cJSON_AddItemToObject(root, "GSIA", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.GSIB);
    cJSON_AddItemToObject(root, "GSIB", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.GSIC);
    cJSON_AddItemToObject(root, "GSIC", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.GSIN);
    cJSON_AddItemToObject(root, "GSIN", cJSON_CreateString(strbuf));

    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.PA_PHSL);
    cJSON_AddItemToObject(root, "PA_PHSL", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.QA_PHSL);
    cJSON_AddItemToObject(root, "QA_PHSL", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.PB_PHSL);
    cJSON_AddItemToObject(root, "PB_PHSL", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.QB_PHSL);
    cJSON_AddItemToObject(root, "QB_PHSL", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.PC_PHSL);
    cJSON_AddItemToObject(root, "PC_PHSL", cJSON_CreateString(strbuf));
    
    rt_snprintf(strbuf, BUFSZ, "0x%04X", _jc_std_cfg.QC_PHSL);
    cJSON_AddItemToObject(root, "QC_PHSL", cJSON_CreateString(strbuf));
    
    json = cJSON_Print(root);
    
    //LOG_I("\n%s\n", json);
    
    fd = open(filename, O_WRONLY | O_CREAT, 0);
    if (fd < 0) {
        LOG_E("create config file failed!");
        return -RT_EIO;
    }
    
    write(fd, json, rt_strlen(json));
    close(fd);

    rt_free(json);
    cJSON_Delete(root);
    
    LOG_D("save to %s", filename);
    
    return RT_EOK;
}

/*
 *
 *  RT-Thread Device Driver
 *
*/

rt_err_t jcdev_device_init(rt_device_t dev)
{
    rt_uint8_t buff[8] = {0};
    
    rt_memset(&_jc_std_cfg, 0, sizeof(struct jcdev_stdef_t));
    
    if(_jcdev_load_config(CONFIG_FILENAME) != RT_EOK) {
        _jc_std_cfg.HFConst = RN8302B_HFCONST;   // 0x13FB
        
        _jc_std_cfg.GSUA = 0x0A99;
        _jc_std_cfg.GSUB = 0x0876;
        _jc_std_cfg.GSUC = 0x08A6;
        
        _jc_std_cfg.GSIA = 0x4B51;
        _jc_std_cfg.GSIB = 0x4B42;
        _jc_std_cfg.GSIC = 0x4A01;

        LOG_W("load config failed! using default config");
    }

    LOG_I("start configure RN8302B IC...");

    buff[0] = 0xE5; _jcdev_config_reg("WREN", buff);  // write enable.
    buff[0] = 0xA2; _jcdev_config_reg("WMSW", buff);  // change mode to EMM.
    buff[0] = 0xFA; _jcdev_config_reg("SOFTRST", buff);   // generate a soft reset.
    
    rt_thread_delay(RT_TICK_PER_SECOND / 2);

    buff[0] = 0xE5; _jcdev_config_reg("WREN", buff);  // write enable.
    buff[0] = 0xA2; _jcdev_config_reg("WMSW", buff);  // change mode to EMM.
    
    buff[0] = _jc_std_cfg.HFConst / 256;
    buff[1] = _jc_std_cfg.HFConst % 256;
    _jcdev_config_reg("HFConst1", buff);  // write HFConst.
    _jcdev_config_reg("HFConst2", buff);

    buff[0] = _jc_std_cfg.GSUA / 256;
    buff[1] = _jc_std_cfg.GSUA % 256;
    _jcdev_config_reg("GSUA", buff);  // write UA Gain value.

    buff[0] = _jc_std_cfg.GSUB / 256;
    buff[1] = _jc_std_cfg.GSUB % 256;
    _jcdev_config_reg("GSUB", buff);  // write UB Gain value.

    buff[0] = _jc_std_cfg.GSUC / 256;
    buff[1] = _jc_std_cfg.GSUC % 256;
    _jcdev_config_reg("GSUC", buff);  // write UC Gain value.
    
    buff[0] = _jc_std_cfg.GSIA / 256;
    buff[1] = _jc_std_cfg.GSIA % 256;
    _jcdev_config_reg("GSIA", buff);  // write IA Gain value.

    buff[0] = _jc_std_cfg.GSIB / 256;
    buff[1] = _jc_std_cfg.GSIB % 256;
    _jcdev_config_reg("GSIB", buff);  // write IB Gain value.

    buff[0] = _jc_std_cfg.GSIC / 256;
    buff[1] = _jc_std_cfg.GSIC % 256;
    _jcdev_config_reg("GSIC", buff);  // write IC Gain value.
    
    buff[0] = _jc_std_cfg.GSIN / 256;
    buff[1] = _jc_std_cfg.GSIN % 256;
    _jcdev_config_reg("GSIN", buff);  // write IN Gain value.
    
    // to be done. PRTH1L, PRTH1H, PRTH2L, PRTH2H

    buff[0] = _jc_std_cfg.GPA / 256;
    buff[1] = _jc_std_cfg.GPA % 256;
    _jcdev_config_reg("GPA", buff);  // write Active Power Gain value.

    buff[0] = _jc_std_cfg.GPB / 256;
    buff[1] = _jc_std_cfg.GPB % 256;
    _jcdev_config_reg("GPB", buff);  // write Active Power Gain value.
    
    buff[0] = _jc_std_cfg.GPC / 256;
    buff[1] = _jc_std_cfg.GPC % 256;
    _jcdev_config_reg("GPC", buff);  // write Active Power Gain value.
    
    buff[0] = _jc_std_cfg.PA_PHSL / 256;
    buff[1] = _jc_std_cfg.PA_PHSL % 256;
    _jcdev_config_reg("PA_PHSL", buff);  // write Active Power Phase Cali value.
    
    buff[0] = _jc_std_cfg.PB_PHSL / 256;
    buff[1] = _jc_std_cfg.PB_PHSL % 256;
    _jcdev_config_reg("PB_PHSL", buff);  // write Active Power Phase Cali value.

    buff[0] = _jc_std_cfg.PC_PHSL / 256;
    buff[1] = _jc_std_cfg.PC_PHSL % 256;
    _jcdev_config_reg("PC_PHSL", buff);  // write Active Power Phase Cali value.

    buff[0] = _jc_std_cfg.QA_PHSL / 256;
    buff[1] = _jc_std_cfg.QA_PHSL % 256;
    _jcdev_config_reg("QA_PHSL", buff);   // write Reactive Power Phase Cali value.

    buff[0] = _jc_std_cfg.QB_PHSL / 256;
    buff[1] = _jc_std_cfg.QB_PHSL % 256;
    _jcdev_config_reg("QB_PHSL", buff);   // write Reactive Power Phase Cali value.

    buff[0] = _jc_std_cfg.QC_PHSL / 256;
    buff[1] = _jc_std_cfg.QC_PHSL % 256;
    _jcdev_config_reg("QC_PHSL", buff);   // write Reactive Power Phase Cali value.
    
    // to be done: GQA, GQB, GQC, GSA, GSB, GSC, PA_OS, PB_OS, PC_OS
    
    buff[0] = _jc_std_cfg.IA_OS / 256;
    buff[1] = _jc_std_cfg.IA_OS % 256;
    _jcdev_config_reg("IA_OS", buff);   // write Current Offset Cali value.
    
    buff[0] = _jc_std_cfg.IB_OS / 256;
    buff[1] = _jc_std_cfg.IB_OS % 256;
    _jcdev_config_reg("IB_OS", buff);   // write Current Offset Cali value.
    
    buff[0] = _jc_std_cfg.IC_OS / 256;
    buff[1] = _jc_std_cfg.IC_OS % 256;
    _jcdev_config_reg("IC_OS", buff);   // write Current Offset Cali value.
    
    buff[0] = 0x00; buff[1] = 0x00;
    _jcdev_config_reg("DCOS_UA", buff);
    _jcdev_config_reg("DCOS_UB", buff);
    _jcdev_config_reg("DCOS_UC", buff);
    _jcdev_config_reg("DCOS_IA", buff);
    _jcdev_config_reg("DCOS_IB", buff);
    _jcdev_config_reg("DCOS_IC", buff);
    _jcdev_config_reg("DCOS_IN", buff);

    buff[0] = 0x02; buff[1] = 0x36; _jcdev_config_reg("IStart_PS", buff);
    buff[0] = 0x02; buff[1] = 0x36; _jcdev_config_reg("IStart_Q", buff);
    buff[0] = 0x04; buff[1] = 0x00; _jcdev_config_reg("LostVoltT", buff);
    buff[0] = 0x00; buff[1] = 0x2C; _jcdev_config_reg("ZXOT", buff);
    buff[0] = 0x04; buff[1] = 0x77; buff[2] = 0x10; _jcdev_config_reg("CFCFG", buff);
    /* EMUCFG: control the Energy Measure Unit
    *  0x780000, read and not clear.
    *  0x700000, read and clear.
    */
    buff[0] = 0x70; buff[1] = 0x00; buff[2] = 0x00; _jcdev_config_reg("EMUCFG", buff);
    buff[0] = 0x10; _jcdev_config_reg("WSAVECON", buff);
    buff[0] = 0x00; _jcdev_config_reg("MODSEL", buff);
    buff[0] = 0x77; buff[1] = 0x77; buff[2] = 0x77; _jcdev_config_reg("EMUCON", buff);
    buff[0] = 0xDC; _jcdev_config_reg("WREN", buff);
    
    LOG_I("finish configuration.");
    
    return RT_EOK;
}

rt_err_t jcdev_device_open(rt_device_t dev, rt_uint16_t oflag)
{
    LOG_I("device open.");
    
    return RT_EOK;
}

rt_err_t jcdev_device_close(rt_device_t dev)
{
    LOG_I("device close.");
    
    return RT_EOK;
}

rt_size_t jcdev_device_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_uint8_t regbuf[8] = {0};
    struct jcdev_reg_t *reg = (struct jcdev_reg_t*)pos;
    
    if (buffer == RT_NULL)
        return 0;
    
    if (reg == RT_NULL)
        return 0;
    
    if (reg->addr < 0xA000) {   // RN8302B Register Access
        _jcdev_reg_read(reg->addr, regbuf, reg->size);
        
        if (reg->detail == RT_NULL) {
            rt_memcpy(buffer, regbuf, reg->size);
            
            return reg->size;
        }
        else {
            *(float *)buffer = _jcdev_raw_to_float(reg, regbuf);
            
            return 1;
        }
    }
    else if (reg->addr > 0xB000) {
        if (reg->addr < 0xB010) {
            rt_uint8_t index = (reg->addr & 0x001F);
                
            if (index < (sizeof(struct jcdev_energy_t)/4)) {
                *(float*)buffer = (float)*((double*)&_jc_energy + index);
                return 1;
            }
            else
                LOG_W("EMU log out of index, 0x%04X", reg->addr);
        }
        else
            LOG_W("Non-violate Vars out of index, 0x%04X", reg->addr);
    }
    else {  // Virtual Register Access
        

        
        if (rt_tick_get() - _jc_harmonic.timestamp > RT_TICK_PER_SECOND *5) {
            // Data out of date, sample a new wave then calculate.
            _jcdev_wave_sample();

            _jc_harmonic.ThdPhV[0] = _jcdev_wave_fft("UA", _jc_harmonic.phsA_HphV);
            _jc_harmonic.ThdPhV[1] = _jcdev_wave_fft("UB", _jc_harmonic.phsB_HphV);
            _jc_harmonic.ThdPhV[2] = _jcdev_wave_fft("UC", _jc_harmonic.phsC_HphV);
            _jc_harmonic.ThdA[0]   = _jcdev_wave_fft("IA", _jc_harmonic.phsA_HA);
            _jc_harmonic.ThdA[1]   = _jcdev_wave_fft("IB", _jc_harmonic.phsB_HA);
            _jc_harmonic.ThdA[2]   = _jcdev_wave_fft("IC", _jc_harmonic.phsC_HA);
            
            _jc_harmonic.timestamp = rt_tick_get();
        }
        
        if (reg->addr < 0xA100) {           // 0xA000 - 0xA100
            rt_uint8_t index = reg->addr & 0x00FF;
            float fsum = 0.f;
            
            switch(index) {
                case 0:
                case 1:
                case 2: {
                    fsum = _jc_harmonic.ThdPhV[index];
                    break;
                }
                case 3:
                case 4:
                case 5: {
                    fsum = _jc_harmonic.ThdA[index - 3];
                    break;
                }
                default: {
                    LOG_W("reg->addr 0x%04X can not match any data", reg->addr);
                    break;
                }
            }
            *(float*)buffer = fsum * 100.f;
        }
        else if (reg->addr < 0xA200) {      // 0xA100 - 0xA200
            rt_uint8_t index = (reg->addr & 0x00FF) + 1;
            
            if (index < HARMONIC_NUMBER)
                *(float*)buffer = _jc_harmonic.phsA_HphV[index];
            else
                LOG_W("harmonic array out of index, 0x%04X", reg->addr);
        }
        else if (reg->addr < 0xA300) {      // 0xA200 - 0xA300
            rt_uint8_t index = (reg->addr & 0x00FF) + 1;
            
            if (index < HARMONIC_NUMBER)
                *(float*)buffer = _jc_harmonic.phsB_HphV[index];
            else
                LOG_W("harmonic array out of index, 0x%04X", reg->addr);
        }
        else if (reg->addr < 0xA400) {      // 0xA300 - 0xA400
            rt_uint8_t index = (reg->addr & 0x00FF) + 1;
            
            if (index < HARMONIC_NUMBER)
                *(float*)buffer = _jc_harmonic.phsC_HphV[index];
            else
                LOG_W("harmonic array out of index, 0x%04X", reg->addr);
        }
        else if (reg->addr < 0xA500) {      // 0xA400 - 0xA500
            rt_uint8_t index = (reg->addr & 0x00FF) + 1;
            
            if (index < HARMONIC_NUMBER)
                *(float*)buffer = _jc_harmonic.phsA_HA[index];
            else
                LOG_W("harmonic array out of index, 0x%04X", reg->addr);
        }
        else if (reg->addr < 0xA600) {      // 0xA500 - 0xA600
            rt_uint8_t index = (reg->addr & 0x00FF) + 1;
            
            if (index < HARMONIC_NUMBER)
                *(float*)buffer = _jc_harmonic.phsB_HA[index];
            else
                LOG_W("harmonic array out of index, 0x%04X", reg->addr);
        }
        else if (reg->addr < 0xA700) {      // 0xA500 - 0xA600
            rt_uint8_t index = (reg->addr & 0x00FF) + 1;
            
            if (index < HARMONIC_NUMBER)
                *(float*)buffer = _jc_harmonic.phsC_HA[index];
            else
                LOG_W("harmonic array out of index, 0x%04X", reg->addr);
        }
        else{
             LOG_W("harmonic array out of index, 0x%04X", reg->addr);
        }
    }
    
    return 1;
}

rt_size_t jcdev_device_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    if (buffer == RT_NULL)
        return 0;
    
    _jcdev_reg_write(pos, buffer, size);
    
    return 0;
}

rt_err_t jcdev_device_control(rt_device_t dev, int cmd, void *args)
{
    struct jcdev_reg_t *reg_t = (struct jcdev_reg_t*)args;
    rt_uint8_t buff[8] = {0};
    
    switch(cmd)
    {
        case JCDEV_SOFT_RESET: {
            buff[0] = 0xFA;
            _jcdev_config_reg("SOFTRST", buff);   // generate a soft reset.
        
            LOG_D("JCDEV_FORCE_RESET_CHIP execute");
            break;
        }
        case JCDEV_SEARCH_BYNAME: {
            if (reg_t == RT_NULL) {
                LOG_E("JCDEV_SEARCH_REGISTER_BYNAME need memory");
                return -RT_ENOMEM;
            }
            reg_t->addr = 0xFFFF;
            
            for (int i = 0; i < RN8302B_REGCOUNT; i++) {
                
                if (rt_strcmp(reg_t->desc, (RN8302B_REGLIST + i)->desc) == 0) {
                    reg_t->addr = (RN8302B_REGLIST + i)->addr;
                    reg_t->size = (RN8302B_REGLIST + i)->size;
                    reg_t->writeable = (RN8302B_REGLIST + i)->writeable;
                    reg_t->ratio = (RN8302B_REGLIST + i)->ratio;
                    reg_t->detail = (RN8302B_REGLIST + i)->detail;
                    
                    break;
                }
            }
            
            if (reg_t->addr == 0xFFFF)
                return -RT_EEMPTY;
        
            break;
        }
        case JCDEV_CALI_ENV_INITIAL: {
            rt_memset(buff, 0, sizeof(buff));
            
            LOG_W("JCDEV_CALI_ENV_INITIAL start!");
            buff[0] = 0xE5;
            _jcdev_config_reg("WREN", buff);
            buff[0] = 0xA2;
            _jcdev_config_reg("WMSW", buff);
            buff[0] = 0xFA;
            _jcdev_config_reg("SOFTRST", buff);   // generate a soft reset.
            
            LOG_I("Soft reset chip...");
            rt_thread_delay(RT_TICK_PER_SECOND / 2);
    
            buff[0] = 0xE5;
            _jcdev_config_reg("WREN", buff);  // write enable.
    
            buff[0] = 0xA2;
            _jcdev_config_reg("WMSW", buff);  // change mode to EMM.
            
            
            buff[0] = _jc_std_cfg.HFConst / 256;
            buff[1] = _jc_std_cfg.HFConst % 256;
            _jcdev_config_reg("HFConst1", buff);
            _jcdev_config_reg("HFConst2", buff);
            
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_config_reg("GSUA", buff);
            _jcdev_config_reg("GSUB", buff);
            _jcdev_config_reg("GSUC", buff);
            _jcdev_config_reg("GSIA", buff);
            _jcdev_config_reg("GSIB", buff);
            _jcdev_config_reg("GSIC", buff);
            _jcdev_config_reg("GSIN", buff);
            _jcdev_config_reg("GPA", buff);
            _jcdev_config_reg("GPB", buff);
            _jcdev_config_reg("GPC", buff);
            _jcdev_config_reg("PA_PHSL", buff);
            _jcdev_config_reg("PB_PHSL", buff);
            _jcdev_config_reg("PC_PHSL", buff);
            _jcdev_config_reg("QA_PHSL", buff);
            _jcdev_config_reg("QB_PHSL", buff);
            _jcdev_config_reg("QC_PHSL", buff);
            _jcdev_config_reg("UA_OS", buff);
            _jcdev_config_reg("UB_OS", buff);
            _jcdev_config_reg("UC_OS", buff);
            _jcdev_config_reg("IA_OS", buff);
            _jcdev_config_reg("IB_OS", buff);
            _jcdev_config_reg("IC_OS", buff);
            _jcdev_config_reg("IN_OS", buff);
            
            buff[0] = 0x02; buff[1] = 0x36;
            _jcdev_config_reg("IStart_PS", buff);
            _jcdev_config_reg("IStart_Q", buff);
            buff[0] = 0x04; buff[1] = 0x00;
            _jcdev_config_reg("LostVoltT", buff);
            buff[0] = 0x00; buff[1] = 0x2C;
            _jcdev_config_reg("ZXOT", buff);
            
            buff[0] = 0x04; buff[1] = 0x32; buff[2] = 0x10;
            _jcdev_config_reg("CFCFG", buff);
            buff[0] = 0x40; buff[1] = 0x00; buff[2] = 0x00;
            _jcdev_config_reg("EMUCFG", buff);
            buff[0] = 0x00; buff[1] = 0x00; buff[2] = 0x77;
            _jcdev_config_reg("EMUCON", buff);
            LOG_I("Initial finish!");
            
            break;
        }
        case JCDEV_CALI_U_GAIN: {
            float err = 0.0f;
            rt_int32_t SV = 0;
            
            LOG_W("JCDEV_CALI_U_GAIN start!");
            // calibrate UA
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("UA", buff);
            
            SV = _raw_to_int(buff, 28);
            
            err = ((float)(SV - Ust) * 1.0f) / (float)Ust;
            err = - (err / (1.0f + err));
            LOG_W("UA: %d, err: %d.%08d", SV, (rt_int32_t)floor(err), (rt_int32_t)((err - floor(err))*100000000));
            if (err >= 0.0f) {
                _jc_std_cfg.GSUA = (rt_uint16_t)round(err * 32768);
            }
            else {
                _jc_std_cfg.GSUA = (rt_uint16_t)round(err * 32768 + 65536);
            }
            buff[0] = _jc_std_cfg.GSUA / 256;
            buff[1] = _jc_std_cfg.GSUA % 256;
            _jcdev_config_reg("GSUA", buff);
            // calibrate UB
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("UB", buff);
            
            SV = _raw_to_int(buff, 28);
            err = (float)(SV - Ust) * 1.0f / (float)Ust;
            err = - (err / (1.0f + err));
            LOG_W("UB: %d, err: %d.%08d", SV, (rt_int32_t)floor(err), (rt_int32_t)((err - floor(err))*100000000));
            if (err > 0.0f) {
                _jc_std_cfg.GSUB = (rt_uint16_t)(err * 32768);
            }
            else {
                _jc_std_cfg.GSUB = (rt_uint16_t)(err * 32768 + 65536);
            }
            buff[0] = _jc_std_cfg.GSUB / 256;
            buff[1] = _jc_std_cfg.GSUB % 256;
            _jcdev_config_reg("GSUB", buff);
            // calibrate UC
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("UC", buff);
            
            SV = _raw_to_int(buff, 28);
            err = (float)(SV - Ust) * 1.0f / (float)Ust;
            err = - (err / (1.0f + err));
            LOG_W("UC: %d, err: %d.%08d", SV, (rt_int32_t)floor(err), (rt_int32_t)((err - floor(err))*100000000));
            if (err > 0.0f) {
                _jc_std_cfg.GSUC = (rt_uint16_t)(err * 32768);
            }
            else {
                _jc_std_cfg.GSUC = (rt_uint16_t)(err * 32768 + 65536);
            }
            buff[0] = _jc_std_cfg.GSUC / 256;
            buff[1] = _jc_std_cfg.GSUC % 256;
            _jcdev_config_reg("GSUC", buff);
            
            LOG_I("JCDEV_CALI_U_GAIN finish!");
            LOG_I("GSUA: 0x%04X", _jc_std_cfg.GSUA);
            LOG_I("GSUB: 0x%04X", _jc_std_cfg.GSUB);
            LOG_I("GSUC: 0x%04X", _jc_std_cfg.GSUC);
            
            break;
        }
        case JCDEV_CALI_I_GAIN: {
            float err = 0.0f;
            rt_int32_t SV = 0;
            LOG_W("JCDEV_CALI_I_GAIN start!");
            // calibrate IA
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("IA", buff);
            
            SV = _raw_to_int(buff, 28);
            err = (float)(SV - Ist) * 1.0f / (float)Ist;
            err = - (err / (1.0f + err));
            LOG_W("IA: %d, err: %d.%08d", SV, (rt_int32_t)floor(err), (rt_int32_t)((err - floor(err))*100000000));
            if (err > 0.0f) {
                _jc_std_cfg.GSIA = (rt_uint16_t)(err * 32768);
            }
            else {
                _jc_std_cfg.GSIA = (rt_uint16_t)(err * 32768 + 65536);
            }
            buff[0] = _jc_std_cfg.GSIA / 256;
            buff[1] = _jc_std_cfg.GSIA % 256;
            _jcdev_config_reg("GSIA", buff);
            // calibrate IB
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("IB", buff);
            
            SV = _raw_to_int(buff, 28);
            err = (float)(SV - Ist) * 1.0f / (float)Ist;
            err = - (err / (1.0f + err));
            LOG_W("IB: %d, err: %d.%08d", SV, (rt_int32_t)floor(err), (rt_int32_t)((err - floor(err))*100000000));
            if (err > 0.0f) {
                _jc_std_cfg.GSIB = (rt_uint16_t)(err * 32768);
            }
            else {
                _jc_std_cfg.GSIB = (rt_uint16_t)(err * 32768 + 65536);
            }
            buff[0] = _jc_std_cfg.GSIB / 256;
            buff[1] = _jc_std_cfg.GSIB % 256;
            _jcdev_config_reg("GSIB", buff);
            // calibrate IC
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("IC", buff);
            
            SV = _raw_to_int(buff, 28);
            err = (float)(SV - Ist) * 1.0f / (float)Ist;
            err = - (err / (1.0f + err));
            LOG_W("IC: %d, err: %d.%08d", SV, (rt_int32_t)floor(err), (rt_int32_t)((err - floor(err))*100000000));
            if (err > 0.0f) {
                _jc_std_cfg.GSIC = (rt_uint16_t)(err * 32768);
            }
            else {
                _jc_std_cfg.GSIC = (rt_uint16_t)(err * 32768 + 65536);
            }
            buff[0] = _jc_std_cfg.GSIC / 256;
            buff[1] = _jc_std_cfg.GSIC % 256;
            _jcdev_config_reg("GSIC", buff);
            
            LOG_I("JCDEV_CALI_I_GAIN finish!");
            LOG_I("GSIA: 0x%04X", _jc_std_cfg.GSIA);
            LOG_I("GSIB: 0x%04X", _jc_std_cfg.GSIB);
            LOG_I("GSIC: 0x%04X", _jc_std_cfg.GSIC);
            break;
        }
        case JCDEV_CALI_PHASE_0D5L: {
            float err = 0.0f;
            float lambda = 0.0f;
            rt_int32_t SV = 0;
            
            // calibrate Phase UA
            LOG_W("PHA Calibrate start!");
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("PA", buff);
            SV = _raw_to_int(buff, 32);
            
            err = (float)(SV - P05st) * 1.0f / (float)P05st;
            
            lambda = -1.0f * err / sqrt(3);
            
            if ( lambda < 0) {
                _jc_std_cfg.PA_PHSL = (rt_uint16_t)(lambda * 32768 + 65536);
                _jc_std_cfg.QA_PHSL = (rt_uint16_t)(lambda * 32768 + 65536);
            }
            else {
                _jc_std_cfg.PA_PHSL = (rt_uint16_t)(lambda * 32768);
                _jc_std_cfg.QA_PHSL = (rt_uint16_t)(lambda * 32768);
            }
            
            LOG_I("RESULT: 0x%04X, 0x%04X", _jc_std_cfg.PA_PHSL, _jc_std_cfg.QA_PHSL);

            // calibrate Phase UB
            LOG_W("PHB Calibrate start!");
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("PB", buff);
            SV = _raw_to_int(buff, 32);
            
            err = (float)(SV - P05st) * 1.0f / (float)P05st;
            lambda = -1.0f * (float)err / sqrt(3);
            
            if ( lambda < 0) {
                _jc_std_cfg.PB_PHSL = (rt_uint16_t)(lambda * 32768 + 65536);
                _jc_std_cfg.QB_PHSL = (rt_uint16_t)(lambda * 32768 + 65536);
            }
            else {
                _jc_std_cfg.PB_PHSL = (rt_uint16_t)(lambda * 32768);
                _jc_std_cfg.QB_PHSL = (rt_uint16_t)(lambda * 32768);
            }
            
            LOG_I("RESULT: 0x%04X, 0x%04X", _jc_std_cfg.PB_PHSL, _jc_std_cfg.QB_PHSL);
            
            // calibrate Phase UC
            LOG_W("PHC Calibrate start!");
            rt_memset(buff, 0, sizeof(buff));
            _jcdev_access_reg("PC", buff);
            SV = _raw_to_int(buff, 32);
            
            err = (float)(SV - P05st) * 1.0f / (float)P05st;
            lambda = -1.0f * (float)err / sqrt(3);
            
            if ( lambda < 0) {
                _jc_std_cfg.PC_PHSL = (rt_uint16_t)(lambda * 32768 + 65536);
                _jc_std_cfg.QC_PHSL = (rt_uint16_t)(lambda * 32768 + 65536);
            }
            else {
                _jc_std_cfg.PC_PHSL = (rt_uint16_t)(lambda * 32768);
                _jc_std_cfg.QC_PHSL = (rt_uint16_t)(lambda * 32768);
            }
            
            LOG_I("RESULT: 0x%04X, 0x%04X", _jc_std_cfg.PC_PHSL, _jc_std_cfg.QC_PHSL);

            break;
        }
        case JCDEV_CALI_U_OFFSET: {
            LOG_W("JCDEV_CALI_U_OFFSET start!");
            rt_int32_t SUM, SV;
            rt_uint16_t OFFSET;
            // correct UA offset.
            SUM = 0; SV = 0;
            for (rt_uint8_t i = 0; i < 8; i++) {
                rt_memset(buff, 0, sizeof(buff));
                _jcdev_access_reg("UA", buff);
                SUM += _raw_to_int(buff, 28);
                rt_thread_delay(300);
            }
            
            SV = SUM / 8;
            SV = SV * SV;
            SV = ~SV;
            OFFSET = (SV / 0x4000) & 0xFFFF;
            if (SV & 0x80000000) {
                OFFSET |= 0x8000;
            }
            _jc_std_cfg.UA_OS = OFFSET;
            buff[0] = _jc_std_cfg.UA_OS / 256;
            buff[1] = _jc_std_cfg.UA_OS % 256;
            _jcdev_config_reg("UA_OS", buff);
            
            LOG_I("JCDEV_CALI_U_OFFSET finish!");
            LOG_I("UA_OS: 0x%04X", _jc_std_cfg.UA_OS);
            
            break;
        }
        case JCDEV_CALI_DC_OFFSET: {
            LOG_W("JCDEV_CALI_DC_OFFSET start!");
            
            rt_memset(buff, 0, sizeof(buff));
            buff[0] = 0x7F;
            _jcdev_config_reg("AUTODC_EN", buff);
            
            while(1) {
                rt_thread_delay(RT_TICK_PER_SECOND);
                
                _jcdev_access_reg("AUTODC_EN", buff);
                
                if (buff[0] == 0x00) {
                    break;
                }
                else {
                    LOG_W("auto dc offset correct working..., %02X", buff[0]);
                }
            }
            LOG_I("JCDEV_CALI_DC_OFFSET finish!");
            
            _jcdev_access_reg("DCOS_UA", buff);
            _jc_std_cfg.DCOS_UA = *(rt_uint16_t*)buff;
            LOG_I("DCOS_UA: 0x%02X%02X", buff[0], buff[1]);
            
            _jcdev_access_reg("DCOS_UB", buff);
            _jc_std_cfg.DCOS_UB = *(rt_uint16_t*)buff;
            LOG_I("DCOS_UB: 0x%02X%02X", buff[0], buff[1]);
            
            _jcdev_access_reg("DCOS_UC", buff);
            _jc_std_cfg.DCOS_UC = *(rt_uint16_t*)buff;
            LOG_I("DCOS_UC: 0x%02X%02X", buff[0], buff[1]);
            
            _jcdev_access_reg("DCOS_IA", buff);
            _jc_std_cfg.DCOS_IA = *(rt_uint16_t*)buff;
            LOG_I("DCOS_IA: 0x%02X%02X", buff[0], buff[1]);
            
            _jcdev_access_reg("DCOS_IB", buff);
            _jc_std_cfg.DCOS_IB = *(rt_uint16_t*)buff;
            LOG_I("DCOS_IB: 0x%02X%02X", buff[0], buff[1]);
            
            _jcdev_access_reg("DCOS_IC", buff);
            _jc_std_cfg.DCOS_IC = *(rt_uint16_t*)buff;
            LOG_I("DCOS_IC: 0x%02X%02X", buff[0], buff[1]);
            
            break;
        }
        case JCDEV_EXEC_WAVE_FFT: {
            LOG_I("JCDEV_EXEC_WAVE_FFT start!");
            
            // sample a realtime wave data.
            _jcdev_wave_sample();

            _jc_harmonic.ThdPhV[0] = _jcdev_wave_fft("UA", _jc_harmonic.phsA_HphV);
            _jc_harmonic.ThdPhV[1] = _jcdev_wave_fft("UB", _jc_harmonic.phsB_HphV);
            _jc_harmonic.ThdPhV[2] = _jcdev_wave_fft("UC", _jc_harmonic.phsC_HphV);
            _jc_harmonic.ThdA[0]   = _jcdev_wave_fft("IA", _jc_harmonic.phsA_HA);
            _jc_harmonic.ThdA[1]   = _jcdev_wave_fft("IB", _jc_harmonic.phsB_HA);
            _jc_harmonic.ThdA[2]   = _jcdev_wave_fft("IC", _jc_harmonic.phsC_HA);
            
            _jc_harmonic.timestamp = rt_tick_get();

            break;
        }
        case JCDEV_SAVE_CONFIG: {
            
            _jcdev_save_config(args);
            
            break;
        }
        case JCDEV_LOAD_CONFIG: {
            
            _jcdev_load_config(args);
            
            break;
        }
        default: {
            LOG_W("jcdev control got unknown cmd");
            break;
        }
    }
    
    return RT_EOK;
}

rt_err_t jcdev_wave_sample(void)
{
    rt_uint8_t args;
    
    args = 0xE5;
    _jcdev_config_reg("WREN", &args);  // write enable.
    args = 0xA0;
    _jcdev_config_reg("WSAVECON", &args); // start wave sample
    args = 0xDC;
    _jcdev_config_reg("WREN", &args);  // write disable.

    while (RT_TRUE) {
        rt_thread_delay( 5 );
        _jcdev_access_reg("WSAVECON", &args);
        if (args == 0x80) {
            LOG_D("wave sample finish!");
            break;
        }
        LOG_W("wave sampling... 0x%02X", args);
    }
    
    return RT_EOK;
}

static rt_err_t _jcdev_save_EMUlog(struct jcdev_energy_t* dataset)
{
    const char *filename = EMULOG_FILENAME;
    const rt_size_t BUFSZ = 32;
    cJSON *root;
    char strbuf[BUFSZ];
    char *json;
    int fd = -1;
    
    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "WRCYCLE", cJSON_CreateNumber(dataset->WRCYCLE));

    snprintf(strbuf, BUFSZ, "%.6f", dataset->EPA);
    cJSON_AddItemToObject(root, "EPA", cJSON_CreateString(strbuf));

    snprintf(strbuf, BUFSZ, "%.6f", dataset->EPB);
    cJSON_AddItemToObject(root, "EPB", cJSON_CreateString(strbuf));

    snprintf(strbuf, BUFSZ, "%.6f", dataset->EPC);
    cJSON_AddItemToObject(root, "EPC", cJSON_CreateString(strbuf));

    snprintf(strbuf, BUFSZ, "%.6f", dataset->EPT);
    cJSON_AddItemToObject(root, "EPT", cJSON_CreateString(strbuf));
    
    snprintf(strbuf, BUFSZ, "%.6f", dataset->EQA);
    cJSON_AddItemToObject(root, "EQA", cJSON_CreateString(strbuf));
    
    snprintf(strbuf, BUFSZ, "%.6f", dataset->EQB);
    cJSON_AddItemToObject(root, "EQB", cJSON_CreateString(strbuf));

    snprintf(strbuf, BUFSZ, "%.6f", dataset->EQC);
    cJSON_AddItemToObject(root, "EQC", cJSON_CreateString(strbuf));

    snprintf(strbuf, BUFSZ, "%.6f", dataset->EQT);
    cJSON_AddItemToObject(root, "EQT", cJSON_CreateString(strbuf));
    
    snprintf(strbuf, BUFSZ, "%.6f", dataset->PosEPT);
    cJSON_AddItemToObject(root, "PosEPT", cJSON_CreateString(strbuf));
    
    snprintf(strbuf, BUFSZ, "%.6f", dataset->PosEQT);
    cJSON_AddItemToObject(root, "PosEQT", cJSON_CreateString(strbuf));

    snprintf(strbuf, BUFSZ, "%.6f", dataset->NegEPT);
    cJSON_AddItemToObject(root, "NegEPT", cJSON_CreateString(strbuf));

    snprintf(strbuf, BUFSZ, "%.6f", dataset->NegEQT);
    cJSON_AddItemToObject(root, "NegEQT", cJSON_CreateString(strbuf));

    json = cJSON_Print(root);

    fd = open(filename, O_WRONLY | O_CREAT, 0);
    if (fd < 0) {
        LOG_E("create %s failed!", filename);
        return -RT_EIO;
    }
    
    write(fd, json, rt_strlen(json));
    close(fd);

    rt_free(json);
    cJSON_Delete(root);
    
    LOG_D("save to %s", filename);

    return RT_EOK;
}

static rt_err_t _jcdev_load_EMUlog(struct jcdev_energy_t* dataset)
{
    const char *filename = EMULOG_FILENAME;
    int fd = -1;
    rt_size_t size;
    cJSON *root, *item;
    char *json = RT_NULL;
    double sv;
    
    fd = open(filename, O_RDONLY, 0);
    if (fd < 0) {
        LOG_E("read %s failed! maybe there's no file?", filename);
        close(fd);
        return -RT_EIO;
    }
    
    json = rt_malloc(EMULOG_JSON_MAXSIZE);
    if (json == RT_NULL) {
        LOG_E("malloc buffer for json failed!");
        close(fd);
        return -RT_ENOMEM;
    }
    
    size = read(fd, json, EMULOG_JSON_MAXSIZE);
    if (size == EMULOG_JSON_MAXSIZE)
        LOG_W("json buffer is full!");
    close(fd);
    
    root = cJSON_Parse(json);
    rt_free(json);
    
    if (root == RT_NULL) {
        LOG_E("parse json failed.");
        goto __exit;  
    }
    
    item = cJSON_GetObjectItem(root, "WRCYCLE");
    if (item != RT_NULL) { dataset->WRCYCLE = item->valueint;}
    else {LOG_W("%s parse failed!", "WRCYCLE");}
    
    item = cJSON_GetObjectItem(root, "EPA");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->EPA = sv;}
    else {LOG_W("%s parse failed!", "EPA");}

    item = cJSON_GetObjectItem(root, "EPB");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->EPB = sv;}
    else {LOG_W("%s parse failed!", "EPB");}
    
    item = cJSON_GetObjectItem(root, "EPC");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->EPC = sv;}
    else {LOG_W("%s parse failed!", "EPC");}
    
    item = cJSON_GetObjectItem(root, "EPT");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->EPT = sv;}
    else {LOG_W("%s parse failed!", "EPT");}
    
    item = cJSON_GetObjectItem(root, "EQA");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->EQA = sv;}
    else {LOG_W("%s parse failed!", "EQA");}

    item = cJSON_GetObjectItem(root, "EQB");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->EQB = sv;}
    else {LOG_W("%s parse failed!", "EQB");}
    
    item = cJSON_GetObjectItem(root, "EQC");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->EQC = sv;}
    else {LOG_W("%s parse failed!", "EQC");}
    
    item = cJSON_GetObjectItem(root, "EQT");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->EQT = sv;}
    else {LOG_W("%s parse failed!", "EQT");}
    
    item = cJSON_GetObjectItem(root, "PosEPT");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->PosEPT = sv;}
    else {LOG_W("%s parse failed!", "PosEPT");}

    item = cJSON_GetObjectItem(root, "PosEQT");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->PosEQT = sv;}
    else {LOG_W("%s parse failed!", "PosEQT");}
    
    item = cJSON_GetObjectItem(root, "NegEPT");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->NegEPT = sv;}
    else {LOG_W("%s parse failed!", "NegEPT");}
    
    item = cJSON_GetObjectItem(root, "NegEQT");
    if (item != RT_NULL) { sv = atof(item->valuestring); dataset->NegEQT = sv;}
    else {LOG_W("%s parse failed!", "NegEQT");}
    
__exit:
    cJSON_Delete(root);
    
    return RT_EOK;
}
static rt_err_t _jcdev_update_EMUlog(struct jcdev_energy_t* dataset)
{
    struct jcdev_reg_t reg;
    rt_uint8_t regbuf[8];
    rt_err_t retval;
    
    dataset->WRCYCLE++;
    
    reg.desc = "EPA"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->EPA += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}

    reg.desc = "EPB"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->EPB += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}

    reg.desc = "EPC"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->EPC += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}
    
    reg.desc = "EPT"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->EPT += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}

    reg.desc = "EQA"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->EQA += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}

    reg.desc = "EQB"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->EQB += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}

    reg.desc = "EQC"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->EQC += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}
    
    reg.desc = "EQT"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->EQT += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}
    
    reg.desc = "PosEPT"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->PosEPT += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}

    reg.desc = "PosEQT"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->PosEQT += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}

    reg.desc = "NegEPT"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->NegEPT += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}
    
    reg.desc = "NegEQT"; retval = jcdev_device_control(RT_NULL, JCDEV_SEARCH_BYNAME, &reg);
    if (retval == RT_EOK) {_jcdev_reg_read(reg.addr, regbuf, reg.size); dataset->NegEQT += _jcdev_raw_to_float(&reg, regbuf);}
    else {LOG_W("no %s found", reg.desc);}
    return RT_EOK;
}

static void jcdev_thread_EMUlog(void *parameter)
{
    const char* filename = EMULOG_FILENAME;
    int fd = -1;
    rt_err_t retval;
    
    fd = open(filename, O_RDONLY, 0);
    if (fd < 0) {
        LOG_E("read %s failed! creating a default.", filename);
        rt_memset(&_jc_energy, 0, sizeof(struct jcdev_energy_t));
        retval = _jcdev_save_EMUlog(&_jc_energy);
        if (retval != RT_EOK) {
            LOG_E("init %s failed! thread jcdev_thread_EMUlog quit!", filename); return;
        }
    }
    else {
        close(fd);
        retval = _jcdev_load_EMUlog(&_jc_energy);
        if (retval != RT_EOK) {
            LOG_E("load %s failed! thread jcdev_thread_EMUlog quit!", filename); return;
        }
    }
    
    while (RT_TRUE) {
        rt_thread_delay(RT_TICK_PER_SECOND * EMULOG_PREIOD);
        
        _jcdev_load_EMUlog(&_jc_energy);
        
        _jcdev_update_EMUlog(&_jc_energy);
        
        _jcdev_save_EMUlog(&_jc_energy);
    }
}

static int rt_hw_spi_jcdevice_init(void)
{
    rt_hw_spi_device_attach("spi1", "spi10", GPIOG, GPIO_PIN_10);
    
    jc_spi_dev = (struct rt_spi_device *)rt_device_find("spi10");
    
    jc_spi_dev->config.data_width = 8;
    jc_spi_dev->config.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;
    jc_spi_dev->config.max_hz = 4 * 1000 *1000;       /* 4M */

    rt_pin_mode(JCDEV_PM_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(JCDEV_RST_PIN, PIN_MODE_OUTPUT);
    
    rt_pin_write(JCDEV_PM_PIN, PIN_LOW);
    
    rt_thread_delay(50);
    rt_pin_write(JCDEV_RST_PIN, PIN_LOW);
    rt_thread_delay(50);
    rt_pin_write(JCDEV_RST_PIN, PIN_HIGH);
    
    sin_tab = rt_malloc(WAVE_SAMPLE_COUNT * sizeof(float));
    cos_tab = rt_malloc(WAVE_SAMPLE_COUNT * sizeof(float));
    
    // init sin & cos brust calc table.
	for (rt_uint8_t i = 0; i < WAVE_SAMPLE_COUNT; i++ ) {
		sin_tab[i]=sin(FFT_PI*2*i/WAVE_SAMPLE_COUNT);
		cos_tab[i]=cos(FFT_PI*2*i/WAVE_SAMPLE_COUNT);
	}
    
    rt_device_t dev = rt_device_create(RT_Device_Class_Char, 0);
    
    RT_ASSERT(dev != RT_NULL);

    dev->init = jcdev_device_init;
    dev->open = jcdev_device_open;
    dev->close = jcdev_device_close;
    dev->read = jcdev_device_read;
    dev->write = jcdev_device_write;
    dev->control = jcdev_device_control;
    
    dev->user_data = jc_spi_dev;
    
    LOG_I("jcdev init success");
    
    rt_thread_t nv_thrd = rt_thread_create("bgNVEMU", jcdev_thread_EMUlog, 0, 4096, 20, 10);
    RT_ASSERT(nv_thrd != RT_NULL);
    rt_thread_startup(nv_thrd);
    LOG_I("jc EMUlog thread startup.");
    
    return rt_device_register(dev, JC_CHIP_TYPE, RT_DEVICE_FLAG_RDWR);
}
INIT_DEVICE_EXPORT(rt_hw_spi_jcdevice_init);

static rt_device_t jcdev = RT_NULL;

int jc_init(int argc, char *argv[])
{
    if (jcdev == RT_NULL) {
        LOG_I("First call should execute initialization");
        
        jcdev  = rt_device_find(JC_CHIP_TYPE);
        RT_ASSERT(jcdev != RT_NULL);
        rt_device_open(jcdev, RT_DEVICE_OFLAG_RDWR);
    }
    
    LOG_W("Initial already finished!");
    
    return 0;
}
MSH_CMD_EXPORT(jc_init, init the jcdev(rn8302b) on spi1);

int jc_read(int argc, char * argv[])
{
    rt_uint8_t buff[16] = {0};
    char *desc = argv[1];
    
    if (argc != 2) {
        LOG_E("usage: jc_read REG_NAME");
        return -1;
    }
    
    if (_jcdev_access_reg(desc, buff) == 0) {
        return -1;
    }
    
    LOG_I("%02X %02X %02X %02X", buff[0], buff[1], buff[2], buff[3]);
    
    return 0;
}
MSH_CMD_EXPORT(jc_read, read regs of the jcdev(rn8302b) on spi1);

int jc_cali(int argc, char *argv[])
{
    if (jcdev == RT_NULL) {
        LOG_W("exec jc_init first!");
        
        return -1;
    }
    
    if (argc == 2) {
        if (rt_strcmp(argv[1], "setenv") == 0) {
            rt_device_control(jcdev, JCDEV_CALI_ENV_INITIAL, RT_NULL);
        }
        else if (rt_strcmp(argv[1], "offset") == 0) {
            rt_device_control(jcdev, JCDEV_CALI_DC_OFFSET, RT_NULL);
        }
        else if (rt_strcmp(argv[1], "phase") == 0) {
            rt_device_control(jcdev, JCDEV_CALI_PHASE_0D5L, RT_NULL);
        }
        else if (rt_strcmp(argv[1], "gain") == 0) { 
            rt_device_control(jcdev, JCDEV_CALI_U_GAIN, RT_NULL);
            rt_device_control(jcdev, JCDEV_CALI_I_GAIN, RT_NULL);
        }
        else if (rt_strcmp(argv[1], "save") == 0) {
            rt_device_control(jcdev, JCDEV_SAVE_CONFIG, CONFIG_FILENAME);
        }
        else if (rt_strcmp(argv[1], "load") == 0) {
            rt_device_control(jcdev, JCDEV_LOAD_CONFIG, CONFIG_FILENAME);
        }
        
        return 0;
    }
    
    LOG_W("usage: jc_cali setenv/gain/phase/offset/save");

    return -1;
}
MSH_CMD_EXPORT(jc_cali, cali params of the jcdev(rn8302b) on spi1);

static void printfloat(float sv)
{
    sv = sv * 100;
    
    if (sv < 0) {
        sv = -sv;
        rt_kprintf("%c%d.%02d% ", '-', (rt_int32_t)floor(sv), (rt_int32_t)((sv - floor(sv))*100));
    }
    else {
        rt_kprintf("%c%d.%02d% ", ' ', (rt_int32_t)floor(sv), (rt_int32_t)((sv - floor(sv))*100));
    }
}

int jc_fft(int argc, char *argv[])
{
    if (jcdev == RT_NULL) {
        LOG_W("exec jc_init first!");
        return -1;
    }
    
    if (argc != 1) {
        LOG_W("usage: jc_fft");
        return -1;
    }

    rt_device_control(jcdev, JCDEV_EXEC_WAVE_FFT, RT_NULL);
    
    LOG_I("IA harmonic analysis");
    for (rt_uint8_t i = 1; i < 6; i++)
        printfloat(_jc_harmonic.phsA_HA[i]);
    rt_kprintf("\r\n");

    LOG_I("IB harmonic analysis");
    for (rt_uint8_t i = 1; i < 6; i++)
        printfloat(_jc_harmonic.phsB_HA[i]);
    rt_kprintf("\r\n");

    return 0;
}
MSH_CMD_EXPORT(jc_fft, test fft on jcdev(rn8302b) on spi1);
