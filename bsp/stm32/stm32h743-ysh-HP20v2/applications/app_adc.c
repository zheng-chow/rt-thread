#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "app_adc.h"
#include <drv_adc_dma.h>

#define BASELINE_CNL            (6)

struct adc_app_channel{    
  //struct rt_mutex           mt_adc_channel;
  struct{
  volatile rt_uint32_t      channel_addr;
  volatile rt_uint32_t      channel_size;
  }cnl[2];
  volatile rt_uint32_t      i_sel;
  volatile rt_uint32_t      o_sel;  
  //volatile rt_uint32_t      channel_addr;
  //volatile rt_uint32_t      channel_size;
  //volatile rt_uint32_t      channel_addr_2;
  //volatile rt_uint32_t      channel_size_2;
};
struct adc_app_handler{
  adc_callback_parameter_t  p;  
  adc_callback_function_t*  func;
  rt_device_t               dev_adc_channel;
  rt_adc_device_t           dev_adc_baseline;
  rt_thread_t               tid_adc_channel;
  rt_thread_t               tid_adc_baseline;
  struct rt_event           evt_adc_channel;
  volatile float            baseline;
  volatile rt_uint32_t      baseline_valid;
    
  struct adc_app_channel    channel[2];
   
};

struct adc_app_handler* __adc_handler = RT_NULL;
static void app_adc_channel_entry(void* parameter);
static void app_adc_baseline_entry(void* parameter);
static rt_err_t adc_channel_rx_ind(rt_device_t dev, rt_size_t size);
static rt_bool_t adc_update_baseline(void);

rt_bool_t app_create_adc(void){
    if (__adc_handler) app_delete_adc();
    __adc_handler = (struct adc_app_handler*) rt_malloc(sizeof(struct adc_app_handler));
    rt_memset(__adc_handler, 0, sizeof(struct adc_app_handler));
    
    rt_event_init(&__adc_handler->evt_adc_channel, "eCnl", RT_IPC_FLAG_FIFO);    
   
    __adc_handler->dev_adc_channel = rt_device_find("adcdma");
    __adc_handler->dev_adc_baseline = (rt_adc_device_t)rt_device_find("adc3");
    __adc_handler->tid_adc_channel = rt_thread_create("channel", app_adc_channel_entry
        , __adc_handler, 4096, 13, RT_TICK_PER_SECOND / 20);
    __adc_handler->tid_adc_baseline = rt_thread_create("baseline", app_adc_baseline_entry
        , __adc_handler, 4096, 14, RT_TICK_PER_SECOND / 20);

    if (__adc_handler->dev_adc_channel
        && __adc_handler->dev_adc_baseline
        && __adc_handler->tid_adc_channel
        && __adc_handler->tid_adc_baseline){
        return RT_TRUE;
    }
    else{
        if (__adc_handler->tid_adc_channel)
            rt_thread_delete(__adc_handler->tid_adc_channel);
        if (__adc_handler->tid_adc_baseline)
            rt_thread_delete(__adc_handler->tid_adc_baseline);
        rt_free(__adc_handler);
        __adc_handler = RT_NULL;
        return RT_FALSE;
    }
}
rt_bool_t app_delete_adc(void){
    if (!__adc_handler) return RT_TRUE;    
    if (__adc_handler->tid_adc_channel)
        rt_thread_delete(__adc_handler->tid_adc_channel);    
    if (__adc_handler->tid_adc_baseline)
        rt_thread_delete(__adc_handler->tid_adc_baseline);
    if (__adc_handler->dev_adc_baseline)
        rt_adc_disable(__adc_handler->dev_adc_baseline, BASELINE_CNL);
    if (__adc_handler->dev_adc_channel){
        rt_device_close(__adc_handler->dev_adc_channel);
        rt_device_set_rx_indicate(__adc_handler->dev_adc_channel, RT_NULL); 
        rt_event_detach(&__adc_handler->evt_adc_channel);    
    }
    rt_free(__adc_handler);
    __adc_handler = RT_NULL;  
    return RT_TRUE;
}
rt_bool_t set_adc_callback_function(adc_callback_parameter_t h, adc_callback_function_t f){
    if (!__adc_handler) return RT_FALSE;
    __adc_handler->func = f;
    __adc_handler->p = h;
    return RT_TRUE;
}
rt_bool_t app_start_adc(void){
    if (!__adc_handler) return RT_FALSE;
    if (!__adc_handler->func) return RT_FALSE;
    rt_device_set_rx_indicate(__adc_handler->dev_adc_channel, adc_channel_rx_ind);
    rt_adc_enable(__adc_handler->dev_adc_baseline, BASELINE_CNL);
    adc_update_baseline();
    
    rt_thread_startup(__adc_handler->tid_adc_baseline);
    rt_thread_startup(__adc_handler->tid_adc_channel);
    return RT_TRUE;
}
rt_bool_t app_adc_is_exist(void){
    return __adc_handler && __adc_handler->func;   
}

#include <math.h>
#define M_PI		3.14159265358979323846
#define M_PI_120    (M_PI*2/3)
#define M_PI_60    (M_PI/3)

//强制精度小数点后2位
#define SIG_REAL_FREQ    60.0
#define SIG_SCALE       ((rt_uint64_t)100)
#define SIG_FREQ        (((rt_uint64_t)(SIG_REAL_FREQ * SIG_SCALE))*1.0/SIG_SCALE)
    
#define SINCOS_POINTS  360000
#define A_120  (SINCOS_POINTS/3)
#define A_60  (SINCOS_POINTS/6)

#define FIT_BUFFER_SZ 80000

#if defined ( __ICCARM__ )

#pragma location=0xC0400000
rt_uint16_t cosbuffer[SINCOS_POINTS];
//#pragma location=0xC0600000
//float sinValue[SINCOS_POINTS];
//#pragma location=0xC0600000
//float cosValue[SINCOS_POINTS];
//#pragma location=0xC0A00000
//float fitValue[FIT_BUFFER_SZ];

#elif defined ( __CC_ARM )

__attribute__((at(0xC0400000))) rt_uint16_t cosbuffer[SINCOS_POINTS];
//__attribute__((at(0xC0600000))) float sinValue[SINCOS_POINTS];
//__attribute__((at(0xC0800000))) float cosValue[SINCOS_POINTS];
//__attribute__((at(0xC0A00000))) float fitValue[FIT_BUFFER_SZ];

#endif

void create_sin(void){
    for (rt_int32_t n = 0; n < SINCOS_POINTS;n++){
      cosbuffer[n] = (cos(n*M_PI/(SINCOS_POINTS/2))+1)*0.5*0xFFF;
        //sinValue[n] = sin(n*M_PI/(SINCOS_POINTS/2));
        //cosValue[n] = cos(n*M_PI/(SINCOS_POINTS/2));
    }
}
/*
static float fit_mean(float* pdata, rt_uint32_t groups){
    float value = 0;
    for(rt_uint32_t g=0; g<groups; g++)
        value += pdata[g];
    value /= groups;
    return value;
}*/
/*
static rt_bool_t app_adc_fit(rt_uint8_t* adc_data, rt_uint32_t adc_data_len
    , adc_timestamp_t stamp, adc_fit_t fit){
    const float DELTA = 0.3f;
    const float KAPPA = 0.8f;
    const float ALPHA = 0.2f;
    const float         ERR = 0.0001;
    const rt_uint8_t    CNT = 10;
    rt_bool_t           bConvergence = RT_FALSE;
    

    rt_uint32_t groups = adc_data_len/6;
    rt_uint16_t* pdata = (rt_uint16_t*)adc_data;
    for (rt_uint8_t c = 0; c < stamp.cnls; c++){
        float Y[4]={0,0,0,0};
        float dX[4]={0,0,0,0};
        float X[4]={0,0,0,0};
        float A[4][4]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
        rt_int32_t n = 0, k =0 , i = 0 ,j = 0;
        float err = 100;
        rt_int32_t cnt = 10;
        fit.valid = RT_FALSE;
        fitValue[0] = adc_data[c]*1.25f / 0xFFF;
        for (rt_uint32_t g = 1; g < groups; g++){
            rt_uint32_t idx = g * stamp.cnls + c;
            fitValue[g] = adc_data[idx]*1.25f / 0xFFF;
            
            
        }
        X[3] = fit_mean(fitValue, groups);
        
        
        
        
        
    }
    
    
    
    
    return bConvergence;
}*/
static void app_adc_fake(rt_uint8_t* adc_data, rt_uint32_t adc_data_len, rt_uint32_t index, adc_timestamp_t stamp){
    rt_uint32_t groups = adc_data_len/6;
    rt_uint16_t* data16 = (rt_uint16_t*)adc_data;
    rt_uint64_t period = ((rt_uint64_t)(SIG_FREQ * SIG_SCALE)) * stamp.stamp; 
    period -= period/(SIG_SCALE * 1000)*(SIG_SCALE * 1000);
    float fAngle = SINCOS_POINTS / (SIG_SCALE * 1000.0)* period;
    float fDetAngle = SIG_FREQ  / stamp.freq * SINCOS_POINTS ;
    if (index == 0){
        for (rt_uint32_t n = 0; n < groups; n++, data16+=3){
            float fa = fAngle;
            rt_uint32_t ua = ((rt_uint32_t)(fa+0.5f))%SINCOS_POINTS;
            data16[0] = cosbuffer[ua];
            fa += A_120;
            ua = ((rt_uint32_t)(fa+0.5f))%SINCOS_POINTS;      
            data16[1] = cosbuffer[ua];
            fa += A_120;
            ua = ((rt_uint32_t)(fa+0.5f))%SINCOS_POINTS;   
            data16[2] = cosbuffer  [ua];
            fAngle += fDetAngle;
        }
 
    }
    else{
        fAngle += A_60;
        for (rt_uint32_t n = 0; n < groups; n++, data16+=3){
            float fa = fAngle;
            rt_uint32_t ua = ((rt_uint32_t)(fa+0.5f))%SINCOS_POINTS;
            data16[0] = cosbuffer[ua];
            fa += A_120;
            ua = ((rt_uint32_t)(fa+0.5f))%SINCOS_POINTS;      
            data16[1] = cosbuffer[ua];
            fa += A_120;
            ua = ((rt_uint32_t)(fa+0.5f))%SINCOS_POINTS;   
            data16[2] = cosbuffer [ua];
            fAngle += fDetAngle;
        }     
    }   
}
static void app_adc_channel_entry(void* parameter){
    struct adc_app_handler* handler = (struct adc_app_handler*)parameter;
    rt_err_t    err = RT_EOK;
    rt_uint32_t evt = 0;
    rt_uint32_t index = 0;
    rt_uint32_t bank = 0;
    rt_uint32_t length = 0;
    rt_uint32_t addr = 0;
    rt_base_t   level;
    adc_timestamp_t timeMask[2]={{ADC_FREQ, ADC_CNL_CNT, ADC_BYTES, 0},{ADC_FREQ, ADC_CNL_CNT, ADC_BYTES, 0}};
    rt_uint32_t timeInc = 0;
    struct adc_app_channel* cnl = RT_NULL;
    rt_device_open(handler->dev_adc_channel, 0);
    while(1){
        err = rt_event_recv(&handler->evt_adc_channel, 0x03, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR
        , RT_TICK_PER_SECOND/5, &evt);
        if (err == RT_EOK){

            index = (evt & 0x01)?0:1;
            cnl = &handler->channel[index];
            if (cnl->cnl[cnl->o_sel].channel_size == 0){
                bank = (cnl->o_sel+1) & 0x01;            
                if (cnl->cnl[bank].channel_size == 0)
                    continue;
                cnl->o_sel = bank;
            }
            bank = cnl->o_sel;            
            length = cnl->cnl[bank].channel_size;
            addr = cnl->cnl[bank].channel_addr;
            timeInc = (length/ADC_BYTES/ADC_CNL_CNT)*1000/ADC_FREQ;
            if (handler->func){
                //app_adc_fake((rt_uint8_t*)addr, length, index, timeMask[index]);
                handler->func(handler->p, (rt_uint8_t*)addr, length, index, timeMask[index]);//bank
            }      
            timeMask[index].stamp += timeInc;
            level = rt_hw_interrupt_disable();            
            cnl->cnl[bank].channel_size -= length;
            cnl->cnl[bank].channel_addr += length;
            if (cnl->cnl[bank].channel_size == 0){
                bank = (bank+1) & 0x01;
                if (cnl->cnl[bank].channel_size != 0)
                  cnl->o_sel = bank;  
            }           
            rt_hw_interrupt_enable(level);
        }
        else{
            static rt_bool_t bOn = RT_FALSE;
            if (bOn)
                rt_pin_write(WARN_PIN, PIN_HIGH);      
            else
                rt_pin_write(WARN_PIN, PIN_LOW);  
            bOn = !bOn;           
        }       
    }
    rt_device_close(handler->dev_adc_channel);

}
static void app_adc_baseline_entry(void* parameter){
    while (1){
        if (!adc_update_baseline())
            break;
        rt_thread_mdelay(5000);        
    }
}
static rt_err_t adc_channel_rx_ind(rt_device_t dev, rt_size_t size){
    rt_uint32_t index = stm32_rx_info_get_adc_index(size);
    rt_uint32_t length = stm32_rx_info_get_adc_length(size);
    rt_uint32_t addr = stm32_rx_info_get_adc_addr(size);
    struct adc_app_channel*  cnl = (index==0)?(index=0,&__adc_handler->channel[0]):(index=1,&__adc_handler->channel[1]);
    volatile rt_uint32_t bank = cnl->i_sel;
    if (cnl->cnl[bank].channel_addr == 0){
        cnl->cnl[bank].channel_size = length;
        cnl->cnl[bank].channel_addr = addr;
    }
    else if (addr >= cnl->cnl[bank].channel_addr){
        if (addr != (cnl->cnl[bank].channel_addr + cnl->cnl[bank].channel_size)){
            rt_pin_write(WARN_PIN, PIN_HIGH);            
        }
        else{
            cnl->cnl[bank].channel_size += length;
        }
    }
    else{
        bank = (bank+1) & 0x01;
        RT_ASSERT(cnl->cnl[bank].channel_size == 0);
        cnl->i_sel = bank;
        cnl->cnl[bank].channel_size = length;
        cnl->cnl[bank].channel_addr = addr;
    }    
    rt_event_send(&__adc_handler->evt_adc_channel, 1<<index); 

    return RT_EOK;
}
static rt_bool_t adc_update_baseline(void){
    if (!__adc_handler) return RT_FALSE;   
    rt_uint32_t value = rt_adc_read(__adc_handler->dev_adc_baseline, BASELINE_CNL);
    __adc_handler->baseline = value *2.5f/4095;
    if ((__adc_handler->baseline > 1.0f) && (__adc_handler->baseline < 1.5f))
        __adc_handler->baseline_valid = RT_TRUE;
    else
        __adc_handler->baseline_valid = RT_FALSE;
    return RT_TRUE;
}

