#include <board.h>
#ifndef RT_USING_PWM
#include "drv_config.h"

#include "drv_foc.h"
#include "drv_hall.h"
#include "svpwmhelper.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.pwmcom"
#include <drv_log.h>

#define MAX_PERIOD 65535
#define MIN_PERIOD 3
#define MIN_PULSE 1

struct stm32_pwm
{
    struct rt_device        parent;
    TIM_HandleTypeDef       tim_handle;
   // TIM_HandleTypeDef       hall_handle;
    ADC_HandleTypeDef       adc_handler;
    DMA_HandleTypeDef       adc_dma_handler;
    char *                  name;
    uint32_t                period;
    uint32_t                freq;
    uint8_t                 idx;
    uint32_t                adc_buffer[8];
    uint32_t                adc_data[4];
    float                   coefM;
    float                   eAngle;
    float                   eAngleBase;
    //rt_bool_t               bUpdate;
};
static struct stm32_pwm pwm_object={
      .tim_handle.Instance = TIM1,
      //.hall_handle.Instance = TIM3,
      .adc_handler.Instance = ADC1,
      .adc_dma_handler.Instance = DMA1_Channel1,
      .name = "pwmcom",
      .period = 1800,
      .freq = 20000,
};
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

static rt_uint64_t rt_stm32_get_prescaler(rt_uint64_t freq){
    rt_uint64_t tim_clock = HAL_RCC_GetPCLK1Freq() * 2;
    rt_uint64_t tim_clock_div = 0;
    rt_uint64_t prescaler = 0;
    rt_uint64_t period = 0;
    do{
       prescaler++;
       tim_clock_div = tim_clock / prescaler;
       period = tim_clock_div / freq;        
    }while(period>MAX_PERIOD);  
   
    LOG_I("prescaler: %lu", prescaler);

    return prescaler;
}
static rt_uint64_t rt_stm32_get_period(rt_uint64_t prescaler, rt_uint64_t freq){
    rt_uint64_t tim_clock = HAL_RCC_GetPCLK1Freq() * 2;
    rt_uint64_t period = tim_clock / prescaler/ freq;
    if (period < MIN_PERIOD) period = MIN_PERIOD;
    //LOG_I("period: %lu", period);
    return period;
}
static rt_err_t  rt_stm32_pwm3_timer_init(rt_device_t dev){
    struct stm32_pwm* pwm = (struct stm32_pwm*)dev;
    TIM_HandleTypeDef *htim = &pwm->tim_handle;
    rt_err_t result = RT_EOK;
    
    TIM_OC_InitTypeDef oc_config = {0};
    TIM_MasterConfigTypeDef master_config = {0};
    TIM_ClockConfigTypeDef clock_config = {0};
    rt_uint64_t prescaler = rt_stm32_get_prescaler(pwm->freq*2);
    rt_uint64_t period = rt_stm32_get_period(prescaler, pwm->freq*2);
    RT_ASSERT(&pwm_object == pwm);      
    pwm->period = period;
    htim->Init.Prescaler = prescaler-1;
    //wickkid
    htim->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    htim->Init.Period = period ;//pwm->period-1;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;//TIM_AUTORELOAD_PRELOAD_DISABLE;

    LOG_I("freq: %u\tperiod: %u", pwm->freq,  pwm->period);
  
    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        LOG_E("%s time base init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }

    clock_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(htim, &clock_config) != HAL_OK)
    {
        LOG_E("%s clock init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }

    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
        LOG_E("%s pwm init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }    
    
    master_config.MasterOutputTrigger = TIM_TRGO_OC4REF;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &master_config) != HAL_OK)
    {
        LOG_E("%s master config failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }

    oc_config.OCMode = TIM_OCMODE_PWM1;
    oc_config.Pulse = 0;
    
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    oc_config.OCFastMode = TIM_OCFAST_ENABLE;

    
    if (HAL_TIM_PWM_ConfigChannel(htim, &oc_config, TIM_CHANNEL_1) != HAL_OK)
    {
        LOG_E("%s channel1 config failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }   

   
    if (HAL_TIM_PWM_ConfigChannel(htim, &oc_config, TIM_CHANNEL_2) != HAL_OK)
    {
        LOG_E("%s channel2 config failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }  


    if (HAL_TIM_PWM_ConfigChannel(htim, &oc_config, TIM_CHANNEL_3) != HAL_OK)
    {
        LOG_E("%s channel3 config failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }   

    if (HAL_TIM_PWM_ConfigChannel(htim, &oc_config, TIM_CHANNEL_4) != HAL_OK)
    {
        LOG_E("%s channel4 config failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, period - 1);  
    
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0xF;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig);
    
    HAL_TIM_MspPostInit(htim);

    __HAL_TIM_URS_ENABLE(htim);


__exit:
    return result; 

}
static rt_err_t  rt_stm32_pwm3_adc_init(rt_device_t dev){
    struct stm32_pwm* pwm = (struct stm32_pwm*)dev;
    ADC_HandleTypeDef *hadc = &pwm->adc_handler;
    DMA_HandleTypeDef *hdma = &pwm->adc_dma_handler;
    //ADC_InjectionConfTypeDef cn_cfg = {0};
    ADC_ChannelConfTypeDef cn_cfg = {0};
    rt_err_t result = RT_EOK;

    hadc->Init.DataAlign             = ADC_DATAALIGN_RIGHT;     
    hadc->Init.ScanConvMode          = ADC_SCAN_ENABLE;//ADC_SCAN_DISABLE;//ADC_SCAN_ENABLE;            
    hadc->Init.ContinuousConvMode    = DISABLE;                    
    hadc->Init.NbrOfConversion       = 4;                           
    hadc->Init.DiscontinuousConvMode = DISABLE;                    
    hadc->Init.NbrOfDiscConversion   = 1;                            
    hadc->Init.ExternalTrigConv      = ADC_EXTERNALTRIGINJECCONV_T1_CC4;         
    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
        LOG_E("%s adc base init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }  
   
    cn_cfg.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    cn_cfg.Channel = ADC_CHANNEL_10;
    cn_cfg.Rank = ADC_REGULAR_RANK_1;
    if (HAL_ADC_ConfigChannel(hadc, &cn_cfg) != HAL_OK)
    {
        LOG_E("%s adc channel 10 failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }
    
    cn_cfg.Channel = ADC_CHANNEL_12;
    cn_cfg.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(hadc, &cn_cfg) != HAL_OK)
    {
        LOG_E("%s adc channel 12 failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }
    cn_cfg.Channel = ADC_CHANNEL_13;
    cn_cfg.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(hadc, &cn_cfg) != HAL_OK)
    {
        LOG_E("%s adc channel 13 failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }
    cn_cfg.Channel = ADC_CHANNEL_11;
    cn_cfg.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(hadc, &cn_cfg) != HAL_OK)
    {
        LOG_E("%s adc channel 11 failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }
     /**/
    /*
    cn_cfg.InjectedSamplingTime = ADC_SAMPLETIME_55CYCLES_5;//ADC_SAMPLETIME_7CYCLES_5;    
    cn_cfg.InjectedRank =ADC_INJECTED_RANK_1;
    cn_cfg.InjectedNbrOfConversion = 1;
    cn_cfg.InjectedDiscontinuousConvMode = DISABLE;//ENABLE;
    cn_cfg.AutoInjectedConv = DISABLE;
    cn_cfg.ExternalTrigInjecConv = ADC_SOFTWARE_START;//ADC_EXTERNALTRIGINJECCONV_T1_CC4;
    
    
    cn_cfg.InjectedOffset=0;
    cn_cfg.InjectedChannel = ADC_CHANNEL_10;
    if (HAL_ADCEx_InjectedConfigChannel(hadc, &cn_cfg) != HAL_OK)
    {
        LOG_E("%s adc channel 10 failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }  */
    /*
    //cn_cfg.InjectedRank =ADC_INJECTED_RANK_2;
    cn_cfg.InjectedOffset=1;
    cn_cfg.InjectedChannel = ADC_CHANNEL_12;
    if (HAL_ADCEx_InjectedConfigChannel(hadc, &cn_cfg) != HAL_OK)
    {
        LOG_E("%s adc channel 12 failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }      
    //cn_cfg.InjectedRank =ADC_INJECTED_RANK_3;
    cn_cfg.InjectedOffset=2;
    cn_cfg.InjectedChannel = ADC_CHANNEL_13;
    if (HAL_ADCEx_InjectedConfigChannel(hadc, &cn_cfg) != HAL_OK)
    {
        LOG_E("%s adc channel 13 failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }  */

    HAL_ADCEx_Calibration_Start(hadc);
 
    HAL_ADCEx_InjectedStart(hadc);  
    
   
   __HAL_LINKDMA(hadc, DMA_Handle, pwm->adc_dma_handler);
    
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma->Init.Mode = DMA_CIRCULAR;
    hdma->Init.Direction = DMA_PERIPH_TO_MEMORY; 
    hdma->Init.PeriphInc = DMA_PINC_DISABLE;   
    hdma->Init.MemInc = DMA_MINC_ENABLE;     
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;     
    hdma->Init.MemDataAlignment = DMA_MDATAALIGN_WORD; 
    hdma->Init.Priority = DMA_PRIORITY_HIGH;   
    if (HAL_DMA_Init(hdma) != HAL_OK)
    {
        LOG_E("%s adc dma init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;   
    }

    //HAL_ADC_Start_DMA(hadc, pwm->adc_buffer, 3);
    
    
__exit:
    return result; 
}
/*
static rt_err_t  rt_stm32_hall_timer_init(rt_device_t dev){
    struct stm32_pwm* pwm = (struct stm32_pwm*)dev;
    TIM_HandleTypeDef *hall = &pwm->hall_handle;
    rt_err_t result = RT_EOK;
    TIM_MasterConfigTypeDef   sMasterConfig; 
    TIM_HallSensor_InitTypeDef     sHallSensorConfig;
    
    hall->Init.Prescaler = 0;
    hall->Init.CounterMode = TIM_COUNTERMODE_UP;
    hall->Init.Period = 1800;
    hall->Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    hall->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    sHallSensorConfig.IC1Prescaler=TIM_ICPSC_DIV1;
    sHallSensorConfig.IC1Polarity=TIM_ICPOLARITY_BOTHEDGE;//TIM_ICPOLARITY_BOTHEDGE;
    sHallSensorConfig.IC1Filter=0;
    sHallSensorConfig.Commutation_Delay=1;  
    if (HAL_TIMEx_HallSensor_Init(hall,&sHallSensorConfig)!= HAL_OK)
    {
        LOG_E("%s hall sensor init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }
  
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;//TIM_TRGO_OC2REF;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;//TIM_MASTERSLAVEMODE_ENABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(hall, &sMasterConfig) != HAL_OK)
    {
        LOG_E("%s hall master init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }
  
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);   
__exit:    
    return result;

}*/
static rt_err_t  rt_stm32_pwm3_init(rt_device_t dev){
    struct stm32_pwm* pwm = (struct stm32_pwm*)dev;
    rt_err_t result = rt_stm32_pwm3_timer_init(dev);
    if (result != HAL_OK)    {
        LOG_E("%s timer init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    } 
    result = rt_stm32_pwm3_adc_init(dev);
    if (result != HAL_OK)    {
        LOG_E("%s adc init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    } 
    /*result = rt_stm32_hall_timer_init(dev);
    if (result != HAL_OK)    {
        LOG_E("%s hall init failed", pwm->name);
        result = -RT_ERROR;
        goto __exit;
    }*/
__exit:
    return result; 
}
static rt_err_t  rt_stm32_pwm3_open(rt_device_t dev, rt_uint16_t oflag){
    struct stm32_pwm* pwm = (struct stm32_pwm*)dev;
    TIM_HandleTypeDef *htim = &pwm->tim_handle;
    //TIM_HandleTypeDef *hall = &pwm->hall_handle;
    ADC_HandleTypeDef *hadc = &pwm->adc_handler;
    RT_ASSERT(&pwm_object == pwm);
    
    rt_memset(pwm->adc_buffer, 0, sizeof(pwm->adc_buffer));
	
		HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 1); 
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_ADC_Start_DMA(hadc, pwm->adc_buffer, sizeof(pwm->adc_buffer)/sizeof(sizeof(pwm->adc_buffer)[0]));
    //__HAL_ADC_ENABLE(hadc);
    
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Start(htim, TIM_CHANNEL_1); 
    
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Start(htim, TIM_CHANNEL_2); 
    
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Start(htim, TIM_CHANNEL_3);     
    
    //__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC4);//TIM_IT_UPDATE
    //NVIC_EnableIRQ(TIM1_CC_IRQn);//TIM1_UP_IRQn
    __HAL_TIM_SET_COUNTER(htim, 0);
    
    
    pwm->coefM = 0.02;
    pwm->eAngle = 0;     
    float  eangle = hall_fast_get_angle();   
		pwm->eAngleBase = eangle;
    uint16_t pulse1 = pwm_object.period * Svpwm_PhaseA(pwm->coefM, eangle);
    uint16_t pulse2 = pwm_object.period * Svpwm_PhaseB(pwm->coefM, eangle);
    uint16_t pulse3 = pwm_object.period * Svpwm_PhaseC(pwm->coefM, eangle);
    if (pulse1 < MIN_PULSE) pulse1 = MIN_PULSE;
    else if (pulse1 > (pwm->period-MIN_PULSE)) pulse1 = pwm->period-MIN_PULSE;
    if (pulse2 < MIN_PULSE) pulse2 = MIN_PULSE;
    else if (pulse2 > (pwm->period-MIN_PULSE)) pulse2 = pwm->period-MIN_PULSE;
    if (pulse3 < MIN_PULSE) pulse3 = MIN_PULSE;
    else if (pulse3 > (pwm->period-MIN_PULSE)) pulse3 = pwm->period-MIN_PULSE;
            
    __HAL_TIM_SET_COMPARE(&pwm->tim_handle, TIM_CHANNEL_1, pulse1);
    __HAL_TIM_SET_COMPARE(&pwm->tim_handle, TIM_CHANNEL_2, pulse2);
    __HAL_TIM_SET_COMPARE(&pwm->tim_handle, TIM_CHANNEL_3, pulse3);            
    __HAL_TIM_SET_COUNTER(&pwm->tim_handle, 0);
    //pwm_object.bUpdate = RT_FALSE;  
     
     
     

    return RT_EOK;
}
static rt_err_t  rt_stm32_pwm3_close(rt_device_t dev){
    struct stm32_pwm* pwm = (struct stm32_pwm*)dev;
    TIM_HandleTypeDef *htim = &pwm->tim_handle;
    //TIM_HandleTypeDef *hall = &pwm->hall_handle;
    ADC_HandleTypeDef *hadc = &pwm->adc_handler;
    RT_ASSERT(&pwm_object == pwm);
    LOG_I("pulse: %d\t%d\t%d", htim->Instance->CCR1, htim->Instance->CCR2, htim->Instance->CCR3);
    /*
    HAL_TIM_IC_Stop_IT(hall, TIM_CHANNEL_1|TIM_CHANNEL_2|TIM_CHANNEL_3);
    HAL_TIMEx_HallSensor_Stop(hall);
    */
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Stop(htim, TIM_CHANNEL_1);
    
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Stop(htim, TIM_CHANNEL_2);   
    
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Stop(htim, TIM_CHANNEL_3);
    
    //__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);//TIM_IT_UPDATE
    //NVIC_DisableIRQ(TIM1_CC_IRQn);//TIM1_UP_IRQn

    __HAL_TIM_SET_COUNTER(htim, 0);
    
    //__HAL_ADC_DISABLE(hadc);
    
    HAL_ADC_Stop_DMA(hadc);
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    return RT_EOK;
}
static rt_err_t  rt_stm32_pwm3_control(rt_device_t dev, int cmd, void *args){
    struct stm32_pwm* pwm = (struct stm32_pwm*)dev;
    TIM_HandleTypeDef *htim = &pwm->tim_handle;
    RT_ASSERT(&pwm_object == pwm);

    switch(cmd){
        case PWM3_CONTROL_SET_FREQ:
        {
            rt_uint64_t freq = *(rt_uint32_t*)args;
            rt_uint64_t prescaler = rt_stm32_get_prescaler(freq);
            rt_uint64_t period = rt_stm32_get_period(prescaler, freq);
            pwm->freq = freq;
            pwm->period = period;  
            __HAL_TIM_SET_PRESCALER(htim, prescaler - 1);            
            __HAL_TIM_SET_AUTORELOAD(htim, period);
            __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, period - 1);            

            //HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);    
            LOG_I("freq: %u -> period: %u\n", freq, period);
        }
            break;
        case PWM3_CONTROL_GET_PERIOD:
            *(rt_uint32_t*)args = pwm->period;
            break;

        case PWM3_CONTROL_UPDATE:{
            struct _st_pwm_update* upd = (struct _st_pwm_update*)args;  
            pwm->coefM = upd->coefM;
            pwm->eAngle = upd->detAngle; 
        }
            break;
        default:
            return -RT_ENOSYS;
    }
    return RT_EOK;
}
static rt_size_t rt_stm32_pwm3_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size){
   struct stm32_pwm* pwm = (struct stm32_pwm*)dev;
    AdcItem_t* item = (AdcItem_t*)buffer;
    RT_ASSERT(pwm && item);
    if (size != sizeof(AdcItem_t)) return 0;
    item->idx = pwm->idx;
    item->adc[0] = pwm->adc_data[0];
    item->adc[1] = pwm->adc_data[1];
    item->adc[2] = pwm->adc_data[2];
    item->adc[3] = pwm->adc_data[3];    
   return size;
    
}

void DMA1_Channel1_IRQHandler(void){
    DMA_HandleTypeDef* hdma = &pwm_object.adc_dma_handler;
    rt_interrupt_enter();   
    HAL_DMA_IRQHandler(hdma);    
    rt_interrupt_leave();
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
   //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); 
    pwm_object.idx = (GPIOC->IDR >> 6) & 0x7;
    pwm_object.adc_data[0] = pwm_object.adc_buffer[0] & 0xFFF;
    pwm_object.adc_data[1] = pwm_object.adc_buffer[1] & 0xFFF;
    pwm_object.adc_data[2] = pwm_object.adc_buffer[2] & 0xFFF;
    pwm_object.adc_data[3] = pwm_object.adc_buffer[3] & 0xFFF;
    if (pwm_object.parent.rx_indicate)
        pwm_object.parent.rx_indicate(&pwm_object.parent,0);
}

float g_eangle;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    pwm_object.idx = (GPIOC->IDR >> 6) & 0x7;
    pwm_object.adc_data[0] = pwm_object.adc_buffer[4];
    pwm_object.adc_data[1] = pwm_object.adc_buffer[5];
    pwm_object.adc_data[2] = pwm_object.adc_buffer[6];
    pwm_object.adc_data[3] = pwm_object.adc_buffer[7];
	
    //float  eangle = hall_fast_get_angle()+ pwm_object.eAngle;
		//pwm_object.eAngleBase += hall_fast_get_speed() /1000;
		pwm_object.eAngleBase = hall_fast_get_angle();
		if (pwm_object.eAngleBase > 360) pwm_object.eAngleBase -=360;
		else if (pwm_object.eAngleBase <= -360) pwm_object.eAngleBase +=360;
		//float  eangle = hall_fast_get_angle()+ pwm_object.eAngle;
		float  eangle = pwm_object.eAngleBase+ pwm_object.eAngle;
		g_eangle = eangle;
		SvpwmResult_t svpwm;
		Svpwm_PhaseABC(pwm_object.coefM, eangle, &svpwm);
		uint16_t pulse1 = pwm_object.period * svpwm.a;
    uint16_t pulse2 = pwm_object.period * svpwm.b;
    uint16_t pulse3 = pwm_object.period * svpwm.c;
	
    if (pulse1 < MIN_PULSE) pulse1 = MIN_PULSE;
    else if (pulse1 > (pwm_object.period-MIN_PULSE)) pulse1 = pwm_object.period-MIN_PULSE;
    if (pulse2 < MIN_PULSE) pulse2 = MIN_PULSE;
    else if (pulse2 > (pwm_object.period-MIN_PULSE)) pulse2 = pwm_object.period-MIN_PULSE;
    if (pulse3 < MIN_PULSE) pulse3 = MIN_PULSE;
    else if (pulse3 > (pwm_object.period-MIN_PULSE)) pulse3 = pwm_object.period-MIN_PULSE;
    

    __HAL_TIM_SET_COMPARE(&pwm_object.tim_handle, TIM_CHANNEL_1, pulse1);
    __HAL_TIM_SET_COMPARE(&pwm_object.tim_handle, TIM_CHANNEL_2, pulse2);
    __HAL_TIM_SET_COMPARE(&pwm_object.tim_handle, TIM_CHANNEL_3, pulse3);            

    if (pwm_object.parent.rx_indicate)
        pwm_object.parent.rx_indicate(&pwm_object.parent,3);
    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

}
float foc_fast_get_power(void){
   //return  pwm_object.adc_data[3] *3.3f / 4095 / 0.056f;
	return  pwm_object.adc_data[3] *3.3f / 4095 / 0.02629f;
}

/*
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    uint32_t val = 0;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
        val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
        val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    } 
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
        val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
   }     
}

void TIM3_IRQHandler(void){
    TIM_HandleTypeDef *hall = &pwm_object.hall_handle;
    rt_interrupt_enter();
    HAL_TIM_IRQHandler(hall);     
    rt_interrupt_leave();
}*/

#if 0
static void do_led_triggle(void){
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
}


void HAL_TIM_PWM_PulseFinishedCallback(void){
    do_led_triggle();
}
void HAL_TIM_PeriodElapsedCallback(void){
    do_led_triggle();
}

void TIM1_CC_IRQHandler(void){
    TIM_HandleTypeDef *htim = &pwm_object.tim_handle;
    rt_interrupt_enter();    
    HAL_TIM_IRQHandler(htim);     
    rt_interrupt_leave();

}


void TIM1_UP_IRQHandler(void){
    TIM_HandleTypeDef *htim = &pwm_object.tim_handle;
    rt_interrupt_enter();
    HAL_TIM_IRQHandler(htim);     
    rt_interrupt_leave();

}

#endif 
/* leave interrupt */
static int stm32_pwm3_init(void)
{
    int result = RT_EOK;
    //rt_memset(&pwm_object, 0 , sizeof(pwm_object));
    //pwm_object.name = "pwmcom";
    pwm_object.parent.type = RT_Device_Class_Miscellaneous;
    pwm_object.parent.init         = rt_stm32_pwm3_init;
    pwm_object.parent.open         = rt_stm32_pwm3_open;
    pwm_object.parent.close        = rt_stm32_pwm3_close;
    pwm_object.parent.read         = rt_stm32_pwm3_read;
    pwm_object.parent.write        = RT_NULL;
    pwm_object.parent.control      = rt_stm32_pwm3_control; 
    result =   rt_device_register(&pwm_object.parent, pwm_object.name, RT_DEVICE_FLAG_RDWR);

    return result;
}
INIT_DEVICE_EXPORT(stm32_pwm3_init);

#endif
