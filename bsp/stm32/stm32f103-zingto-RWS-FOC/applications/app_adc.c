#include <rtthread.h>
#include <rtdevice.h>
#include <drv_foc.h>
#include <drv_hall.h>
#include <app_adc.h>

//#define DRV_DEBUG
#define LOG_TAG             "app.adc"
#include <drv_log.h>

#define GetI(x)  (4.69*(x))
#define GetVa(x)      (((float)(x)-408.17)*3.3/4095)
#define GetVb(x)      (((float)(x)-395.8)*3.3/4095)
#define GetVc(x)      (((float)(x)-356.29)*3.3/4095)


/*********************
电流测试线程
初始化ADC通知
打开



*****************/
struct rt_event adc_finished_evt;
rt_err_t adc_finished_callback(rt_device_t dev, rt_size_t size){
    return rt_event_send(&adc_finished_evt, 0x01<<size);
}
void app_motor_entry(void* parameter){
    rt_device_t drvPwm = rt_device_find("pwmcom");    
    rt_uint32_t evt;
    
    uint8_t idxList[]={5,4,6,2,3,1};
    uint32_t maxBuffer = 600;
    
    uint16_t bufDataCnt = 0;
    uint8_t ckIdx = 0;
    float  adcData[4] = {0,0,0, 0};
    float  ia,ib,ic;
    AdcItem_t * adcBuffer = (AdcItem_t*)rt_malloc(sizeof(AdcItem_t)*maxBuffer);
    if (!adcBuffer){
        LOG_E("no more memory"); return;
    }  
    rt_event_init(&adc_finished_evt,"pwmadc", RT_IPC_FLAG_FIFO);
    rt_device_set_rx_indicate(drvPwm, adc_finished_callback);    

    rt_thread_mdelay(3000);  
   
    rt_event_recv(&adc_finished_evt, 0x01, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,&evt);
    LOG_I("Start ADC read");    

    while(1)
    {
        rt_bool_t bSectorEnd = RT_FALSE;
        rt_err_t err = rt_event_recv(&adc_finished_evt, 0x01, RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,&evt);
        if (err != RT_EOK) continue;
        rt_device_read(drvPwm, 0, (void*)(adcBuffer+bufDataCnt), sizeof(AdcItem_t)); 
        
        if (idxList[ckIdx] != adcBuffer[bufDataCnt].idx){
           if (bufDataCnt == 0) continue;
           bSectorEnd = RT_TRUE;           
        }
        else{
            bufDataCnt++;
            if (bufDataCnt == maxBuffer) bSectorEnd = RT_TRUE;
        }
        
        if (bSectorEnd){
            for (uint16_t n = 0; n < bufDataCnt;n++){
                adcData[0] += adcBuffer[n].adc[0];
                adcData[1] += adcBuffer[n].adc[1];
                adcData[2] += adcBuffer[n].adc[2];
                adcData[3] += adcBuffer[n].adc[3];
                LOG_D("[%03d]\t%d\t%d\t%d\t%d",n+1,adcBuffer[n].idx
                ,  adcBuffer[n].adc[0],  adcBuffer[n].adc[1]
                ,  adcBuffer[n].adc[2]);

                ia = GetVa(adcBuffer[n].adc[0]);
                ib = GetVb(adcBuffer[n].adc[1]);
                ic = GetVc(adcBuffer[n].adc[2]);
                ia = GetI(ia);
                ib = GetI(ib);
                ic = GetI(ic); 
                switch(adcBuffer[n].idx){
                    case 2:
                    case 3:                        
                    //case 1:
                    //case 6:                        
                        ic = -ia -ib;
                        break;
                    case 4:
                    case 6:
                    //case 2:
                    //case 3:                       
                        ib = -ia - ic;
                        break;
                    case 1:
                    case 5:
                    //case 4:
                    //case 5:
                        ia = -ib -ic;
                        break;  
                }
         
                 LOG_I("[%03d]\t%d\t%d\t%d\t%d",n+1,adcBuffer[n].idx
                ,  (int)(ia*100),  (int)(ib*100)
                ,  (int)(ic*100));       

            }
            bufDataCnt = 0;
            ckIdx++;
            if (ckIdx == 6) break;
        }     
    }
    LOG_I("Stop ADC read");   
    
    if (adcBuffer) rt_free(adcBuffer);
    while (1){
        rt_thread_mdelay(1000);
    } 
}
int app_motor(void){
    rt_thread_t tid = rt_thread_create("motor",app_motor_entry, RT_NULL, 2048, 14, RT_TICK_PER_SECOND/10);
    if (tid) return rt_thread_startup(tid);
    return -RT_ERROR;
}
//INIT_APP_EXPORT(app_motor);

