/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-05     zylx         first version
 * 2018-12-12     greedyhao    Porting for stm32f7xx
 * 2019-02-01     yuneizhilin   fix the stm32_adc_init function initialization issue
 */
#include <drv_adc_dma.h>
#include <board.h>

#if !defined(BSP_USING_ADC1) && !defined(BSP_USING_ADC2) && defined(RT_USING_ADC) && defined(BSP_ADC_USING_DMA)
#define RT_USING_ADC1
#define RT_USING_ADC2

#define TIMER_FREQUENCY_RANGE_MIN      ((uint32_t)    1)    /* Timer minimum frequency (unit: Hz), used to calculate frequency range. With a timer 16 bits, maximum frequency will be 32000 times this value. */
#define TIMER_PRESCALER_MAX_VALUE      (0xFFFF-1)           /* Timer prescaler maximum value (0xFFFF for a timer 16 bits) */

#ifndef ADC_BYTES
#define ADC_BYTES 2
#endif
#if defined(RT_USING_ADC1) && defined(RT_USING_ADC2)
#define ADC_CNT         2
#else
#define ADC_CNT         1
#endif

#ifndef ADC_CNL_CNT
#define ADC_CNL_CNT     3
#endif

#ifndef ADC_FREQ
#define ADC_FREQ            200000//200000
#endif
#ifndef ADC_SLICE_TIME
#define ADC_SLICE_TIME      40
#endif

//#define ADC_BUFF_SLICE_SZ	(ADC_SLICE_TIME * ADC_FREQ * ADC_CNL_CNT /1000)
//#define ADC_BUFFS_SZ		((((ADC_FREQ*ADC_CNL_CNT/ADC_BUFF_SLICE_SZ)+1) & 0xFFFFFFFE)*ADC_BUFF_SLICE_SZ*ADC_BYTES)//缓存1s以上数据
//#define ADC_BUFF_PHYS_SZ	ADC_BUFFS_SZ
//#define ADC_DATA_BASE	    (0xC0000000)





//#define DRV_DEBUG
#define LOG_TAG             "drv.adcdma"
#include <drv_log.h>
enum ADC_INDEX{
#if defined(RT_USING_ADC1)   
    ADC1_IDX,
#endif
#if defined(RT_USING_ADC2)   
    ADC2_IDX,
#endif
};
static ADC_HandleTypeDef adc_config[ADC_CNT] =
{
#if defined(RT_USING_ADC1)   
    {
       .Instance                        = ADC1,                          
       .Init.ClockPrescaler             = ADC_CLOCK_ASYNC_DIV4,   
#if ADC_BYTES==2        
       .Init.Resolution                 = ADC_RESOLUTION_12B,  
#else
       .Init.Resolution                 = ADC_RESOLUTION_8B,  
#endif        
       .Init.ScanConvMode               = ENABLE,                      
       .Init.EOCSelection               = ADC_EOC_SEQ_CONV,           
       .Init.LowPowerAutoWait           = DISABLE,                       
       .Init.ContinuousConvMode         = DISABLE,                       
       .Init.NbrOfConversion            = ADC_CNL_CNT,                                       
       .Init.DiscontinuousConvMode      = ENABLE,                       
       .Init.NbrOfDiscConversion        = ADC_CNL_CNT,                              
       .Init.ExternalTrigConv           = ADC_EXTERNALTRIG_T3_TRGO,      
       .Init.ExternalTrigConvEdge       = ADC_EXTERNALTRIGCONVEDGE_RISING, 
       .Init.ConversionDataManagement   = ADC_CONVERSIONDATA_DMA_CIRCULAR,       
       .Init.Overrun                    = ADC_OVR_DATA_OVERWRITTEN,          
       .Init.OversamplingMode           = DISABLE,           
    },
#endif
#if defined(RT_USING_ADC2)
    {
       .Instance                        = ADC2,                          
       .Init.ClockPrescaler             = ADC_CLOCK_ASYNC_DIV4,          
#if ADC_BYTES==2        
       .Init.Resolution                 = ADC_RESOLUTION_12B,  
#else
       .Init.Resolution                 = ADC_RESOLUTION_8B,  
#endif  
        .Init.ScanConvMode               = ENABLE,          
       .Init.EOCSelection               = ADC_EOC_SEQ_CONV,           
       .Init.LowPowerAutoWait           = DISABLE,                       
       .Init.ContinuousConvMode         = DISABLE,                       
       .Init.NbrOfConversion            = ADC_CNL_CNT,                             
       .Init.DiscontinuousConvMode      = ENABLE,                       
       .Init.NbrOfDiscConversion        = ADC_CNL_CNT,                                      
       .Init.ExternalTrigConv           = ADC_EXTERNALTRIG_T3_TRGO,       
       .Init.ExternalTrigConvEdge       = ADC_EXTERNALTRIGCONVEDGE_RISING,
       .Init.ConversionDataManagement   = ADC_CONVERSIONDATA_DMA_CIRCULAR,       
       .Init.Overrun                    = ADC_OVR_DATA_OVERWRITTEN,          
       .Init.OversamplingMode           = DISABLE, 
    },
#endif 
};
static rt_uint32_t adc_channel[ADC_CNT][ADC_CNL_CNT] = 
{
#if defined(RT_USING_ADC1)   
    {3,9,5},
#endif
#if defined(RT_USING_ADC2)   
    {15,18,19},
#endif
};
struct stm32_adc_handler{
	ADC_HandleTypeDef 	    ADC_Handler;
	DMA_HandleTypeDef       DMA_Handler;

	__IO rt_uint32_t 		XferAddrBase;
	__IO rt_uint32_t        XferTransferNumber;
    __IO rt_uint32_t    	XferSingleBytes;
    __IO rt_uint32_t    	XferHalfBlockBytes;
    __IO rt_uint32_t        XferCount;
    __IO rt_uint32_t        XferSize;
        
	//__IO rt_bool_t		    bFirstBlock;	
	//struct rt_completion    rx_comp;		
};

struct stm32_adc
{
    TIM_HandleTypeDef   			Tim_Handle;
    RCC_ClkInitTypeDef				rcc_clk;
    struct stm32_adc_handler 	    stm32_adc_cnl[ADC_CNT];
    struct rt_device 				stm32_adc_device;
};

static struct stm32_adc stm32_adc_obj;
static rt_uint32_t stm32_adc_get_rx_info(struct stm32_adc_handler* cnl, rt_uint32_t mem0){
    rt_uint32_t info = (cnl->XferSize) << 16 | ((cnl - &stm32_adc_obj.stm32_adc_cnl[0]) & 0xFF);
    if (cnl->XferCount)
        info |= ((cnl->XferTransferNumber-cnl->XferCount) & 0xFF) <<8;
    else if (mem0)
        info |= ((cnl->XferTransferNumber - 0) & 0xFF) <<8;
    else
        info |= ((cnl->XferTransferNumber + 1) & 0xFF) <<8;
    return info;
}
rt_uint32_t stm32_rx_info_get_adc_addr(rt_uint32_t info){
    rt_uint32_t index = info & 0xFF; 
    struct stm32_adc_handler* cnl = &stm32_adc_obj.stm32_adc_cnl[index];
    rt_uint32_t addr = (info >> 16) * ((info >> 8) & 0xFF) * cnl->XferSingleBytes;
    addr += cnl->XferAddrBase;
    return addr;    
}
rt_uint32_t stm32_rx_info_get_adc_length(rt_uint32_t info){
   struct stm32_adc_handler* cnl = &stm32_adc_obj.stm32_adc_cnl[info & 0xFF];
   return cnl->XferSize * cnl->XferSingleBytes;    
}
rt_uint32_t stm32_rx_info_get_adc_index(rt_uint32_t info){
   return info & 0xFF;    
}
static rt_uint32_t stm32_adc_get_channel(rt_uint32_t channel)
{
    rt_uint32_t stm32_channel = 0;

    switch (channel)
    {
    case  0:
        stm32_channel = ADC_CHANNEL_0;
        break;
    case  1:
        stm32_channel = ADC_CHANNEL_1;
        break;
    case  2:
        stm32_channel = ADC_CHANNEL_2;
        break;
    case  3:
        stm32_channel = ADC_CHANNEL_3;
        break;
    case  4:
        stm32_channel = ADC_CHANNEL_4;
        break;
    case  5:
        stm32_channel = ADC_CHANNEL_5;
        break;
    case  6:
        stm32_channel = ADC_CHANNEL_6;
        break;
    case  7:
        stm32_channel = ADC_CHANNEL_7;
        break;
    case  8:
        stm32_channel = ADC_CHANNEL_8;
        break;
    case  9:
        stm32_channel = ADC_CHANNEL_9;
        break;
    case 10:
        stm32_channel = ADC_CHANNEL_10;
        break;
    case 11:
        stm32_channel = ADC_CHANNEL_11;
        break;
    case 12:
        stm32_channel = ADC_CHANNEL_12;
        break;
    case 13:
        stm32_channel = ADC_CHANNEL_13;
        break;
    case 14:
        stm32_channel = ADC_CHANNEL_14;
        break;
    case 15:
        stm32_channel = ADC_CHANNEL_15;
        break;
    case 16:
        stm32_channel = ADC_CHANNEL_16;
        break;
    case 17:
        stm32_channel = ADC_CHANNEL_17;
        break;
    case 18:
        stm32_channel = ADC_CHANNEL_18;
        break;
	case 19:
        stm32_channel = ADC_CHANNEL_19;
        break;
    }

    return stm32_channel;
}
/*
static rt_err_t stm32_get_adc_value(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    ADC_ChannelConfTypeDef ADC_ChanConf;
    ADC_HandleTypeDef *stm32_adc_handler;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(value != RT_NULL);

    stm32_adc_handler = device->parent.user_data;

    rt_memset(&ADC_ChanConf, 0, sizeof(ADC_ChanConf));

#if defined(SOC_SERIES_STM32F1)
    if (channel <= 17)
#elif defined(SOC_SERIES_STM32F0) || defined(SOC_SERIES_STM32F2)  || defined(SOC_SERIES_STM32F4) || defined(SOC_SERIES_STM32F7) \
        || defined(SOC_SERIES_STM32L4) || defined(SOC_SERIES_STM32G0)
    if (channel <= 18)
#elif  defined(SOC_SERIES_STM32H7)
		  if (channel <= 19)
#endif
    {
        ADC_ChanConf.Channel =  stm32_adc_get_channel(channel);
    }
    else
    {
#if defined(SOC_SERIES_STM32F1)
        LOG_E("ADC channel must be between 0 and 17.");
#elif defined(SOC_SERIES_STM32F0) || defined(SOC_SERIES_STM32F2)  || defined(SOC_SERIES_STM32F4) || defined(SOC_SERIES_STM32F7) \
        || defined(SOC_SERIES_STM32L4) || defined(SOC_SERIES_STM32G0)
        LOG_E("ADC channel must be between 0 and 18.");
#elif  defined(SOC_SERIES_STM32H7)
        LOG_E("ADC channel must be between 0 and 19.");
#endif
        return -RT_ERROR;
    }
    ADC_ChanConf.Rank = 1;
#if defined(SOC_SERIES_STM32F0)
    ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
#elif defined(SOC_SERIES_STM32F1)
    ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
#elif defined(SOC_SERIES_STM32F2) || defined(SOC_SERIES_STM32F4) || defined(SOC_SERIES_STM32F7)
    ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_112CYCLES;
#elif defined(SOC_SERIES_STM32L4)
    ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
#elif defined(SOC_SERIES_STM32H7)
    ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
#endif
#if defined(SOC_SERIES_STM32F2) || defined(SOC_SERIES_STM32F4) || defined(SOC_SERIES_STM32F7) || defined(SOC_SERIES_STM32L4) || defined(SOC_SERIES_STM32H7)
    ADC_ChanConf.Offset = 0;
#endif
#if defined(SOC_SERIES_STM32L4) || defined(SOC_SERIES_STM32H7)
    ADC_ChanConf.OffsetNumber = ADC_OFFSET_NONE;
    ADC_ChanConf.SingleDiff = LL_ADC_SINGLE_ENDED;
#endif
    HAL_ADC_ConfigChannel(stm32_adc_handler, &ADC_ChanConf);

    HAL_ADC_Start(stm32_adc_handler);

    HAL_ADC_PollForConversion(stm32_adc_handler, 100);

    *value = (rt_uint32_t)HAL_ADC_GetValue(stm32_adc_handler);

    return RT_EOK;
}

static const struct rt_adc_ops stm_adc_ops =
{
    .enabled = stm32_adc_enabled,
    .convert = stm32_get_adc_value,
};*/
static void stm32_triggle_source(rt_uint32_t hz){
    static uint32_t latency;
    static uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
   
    
    static uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
    TIM_MasterConfigTypeDef masterCfg;
	__HAL_RCC_TIM3_CLK_ENABLE();

    HAL_RCC_GetClockConfig(&stm32_adc_obj.rcc_clk, &latency);

    if (stm32_adc_obj.rcc_clk.APB1CLKDivider == RCC_HCLK_DIV1)
        timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
    else
        timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;


    timer_prescaler = timer_clock_frequency/ (TIMER_PRESCALER_MAX_VALUE * (hz /TIMER_FREQUENCY_RANGE_MIN)) +1;//(timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;	

    stm32_adc_obj.Tim_Handle.Instance = TIM3;  

    stm32_adc_obj.Tim_Handle.Init.Period            = timer_clock_frequency / (timer_prescaler * (hz)) -1;
    stm32_adc_obj.Tim_Handle.Init.Prescaler         = (timer_prescaler - 1);
    stm32_adc_obj.Tim_Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    stm32_adc_obj.Tim_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    stm32_adc_obj.Tim_Handle.Init.RepetitionCounter = 0x0;	
    HAL_TIM_Base_Init(&stm32_adc_obj.Tim_Handle);	

    masterCfg.MasterOutputTrigger = TIM_TRGO_UPDATE;//TIM_TRGO_UPDATE;
    masterCfg.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    masterCfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;//TIM_MASTERSLAVEMODE_ENABLE;//TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&stm32_adc_obj.Tim_Handle, &masterCfg);	
}
static char tmp_char[1024];
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    struct stm32_adc_handler* cnl = (struct stm32_adc_handler*)hadc;
    static uint32_t tmp = 0;
    rt_uint32_t info = 0;
    tmp  = ((((DMA_Stream_TypeDef *)(cnl->DMA_Handler.Instance))->CR) & DMA_SxCR_CT);
    info = stm32_adc_get_rx_info(cnl, tmp);
    if(cnl->XferCount != 0)
    {
        uint32_t addr = 0;
        /* Update memory 0 address location */
        if (((cnl->XferCount & 1) == 0) && (tmp != 0))//(((cnl->XferCount % 2) == 0) && (tmp != 0))
        {
            addr = ((DMA_Stream_TypeDef *)(cnl->DMA_Handler.Instance))->M0AR;
            HAL_DMAEx_ChangeMemory(&cnl->DMA_Handler, (addr + (cnl->XferSingleBytes*cnl->XferSize * 2)), MEMORY0);
            cnl->XferCount--;	 
            rt_pin_write(CONV_PIN, PIN_HIGH);      
            if (stm32_adc_obj.stm32_adc_device.rx_indicate)
                stm32_adc_obj.stm32_adc_device.rx_indicate(&stm32_adc_obj.stm32_adc_device, info);
            

        }

        else if (((cnl->XferCount & 1) == 1) && (tmp == 0))/* Update memory 1 address location */
        {
            addr = ((DMA_Stream_TypeDef *)(cnl->DMA_Handler.Instance))->M1AR;
            HAL_DMAEx_ChangeMemory(&cnl->DMA_Handler, (addr + (cnl->XferSingleBytes*cnl->XferSize * 2)), MEMORY1);
            cnl->XferCount--;
            rt_pin_write(CONV_PIN, PIN_LOW);  
            if (stm32_adc_obj.stm32_adc_device.rx_indicate)
                stm32_adc_obj.stm32_adc_device.rx_indicate(&stm32_adc_obj.stm32_adc_device, info);
            
        }
        /*else{
            __nop();
        }*/
        /*if (addr == (cnl->XferAddrBase + cnl->XferHalfBlockBytes - cnl->XferSize * cnl->XferSingleBytes)){
            cnl->bFirstBlock = RT_TRUE;
            rt_completion_done(&(cnl->rx_comp));
        }*/
    }
    else if (tmp != 0)/* Update memory 0 address location */
    {
        HAL_DMAEx_ChangeMemory(&cnl->DMA_Handler, cnl->XferAddrBase, MEMORY0);
        rt_pin_write(CONV_PIN, PIN_HIGH);   
        if (stm32_adc_obj.stm32_adc_device.rx_indicate)
            stm32_adc_obj.stm32_adc_device.rx_indicate(&stm32_adc_obj.stm32_adc_device, info);
        
    }
    else if (tmp == 0) /* Update memory 1 address location */
    {   
        HAL_DMAEx_ChangeMemory(&cnl->DMA_Handler, cnl->XferAddrBase + (cnl->XferSingleBytes*cnl->XferSize), MEMORY1);
        cnl->XferCount = cnl->XferTransferNumber;
        //cnl->bFirstBlock = RT_FALSE;
        //rt_completion_done(&(cnl->rx_comp));
        rt_pin_write(CONV_PIN, PIN_LOW);    
        if (stm32_adc_obj.stm32_adc_device.rx_indicate)
            stm32_adc_obj.stm32_adc_device.rx_indicate(&stm32_adc_obj.stm32_adc_device, info);
    }   
}
static HAL_StatusTypeDef stm32_config_dma_addr(struct stm32_adc_handler* cnl, uint32_t* pData, uint32_t Length){
    rt_uint32_t	SecondMemAddress = 0;
    ADC_HandleTypeDef* hadc = &cnl->ADC_Handler;	

    RT_ASSERT(0 == (Length % ADC_BUFF_SLICE_SZ));//必须整倍数，不考虑多余情况	
    RT_ASSERT(2 <= (Length / ADC_BUFF_SLICE_SZ));//至少2个buffer

    cnl->XferCount = 0;
    cnl->XferTransferNumber = 0;

    cnl->XferSingleBytes			=  ADC_BYTES;
    cnl->XferCount = 1;
    cnl->XferSize = Length;
    cnl->XferAddrBase = (rt_uint32_t)pData;

    cnl->XferCount = Length / ADC_BUFF_SLICE_SZ;
    cnl->XferSize = ADC_BUFF_SLICE_SZ;
    if (cnl->XferCount & 1){  
        rt_uint32_t det = (0xFFFF<ADC_BUFF_SLICE_SZ)?0xFFFF:ADC_BUFF_SLICE_SZ;    
        cnl->XferCount = 1;
        cnl->XferSize = Length;
        
        while(cnl->XferSize > det){//0xFFFF
            cnl->XferSize = (cnl->XferSize/2);
            cnl->XferCount = cnl->XferCount*2;
        }
    }
    cnl->XferCount = (cnl->XferCount - 2);
    cnl->XferTransferNumber = cnl->XferCount;
    cnl->XferHalfBlockBytes = cnl->XferSingleBytes * Length / 2;

    SecondMemAddress = cnl->XferAddrBase + (cnl->XferSingleBytes*cnl->XferSize);

    rt_snprintf(tmp_char, 1024, "CNL[%d]%08X %d %d %d %d %d %08X"
    , cnl - &stm32_adc_obj.stm32_adc_cnl[0]
    , cnl->XferAddrBase
    , cnl->XferTransferNumber
    , cnl->XferSingleBytes
    , cnl->XferHalfBlockBytes
    , cnl->XferCount
    , cnl->XferSize
    , SecondMemAddress
    );
    LOG_I(tmp_char);
    return HAL_DMAEx_MultiBufferStart_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, SecondMemAddress, cnl->XferSize);
}
static HAL_StatusTypeDef stm32_start_dma(struct stm32_adc_handler* cnl, uint32_t* pData, uint32_t Length){
    ADC_HandleTypeDef* hadc = &cnl->ADC_Handler;
    HAL_StatusTypeDef tmp_hal_status = HAL_OK;

    assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

    if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET)
    {
        __HAL_LOCK(hadc);

        if (ADC_IS_DUAL_REGULAR_CONVERSION_ENABLE(hadc) == RESET)
        {
            tmp_hal_status = ADC_Enable(hadc);

            if (tmp_hal_status == HAL_OK)
            {
                if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
                {
                    CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
                }
                else
                {
                    ADC_CLEAR_ERRORCODE(hadc);
                }
                
                ADC_STATE_CLR_SET(hadc->State,
                  (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP),
                  HAL_ADC_STATE_REG_BUSY);

                if (ADC12_NONMULTIMODE_OR_MULTIMODEMASTER(hadc))
                {
                    CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
                }

                hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;
                hadc->DMA_Handle->XferM1CpltCallback = ADC_DMAConvCplt;

                hadc->DMA_Handle->XferHalfCpltCallback = 0;//ADC_DMAHalfConvCplt;

                hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;


                __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

                __HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

                //HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);
                stm32_config_dma_addr(cnl, pData, Length);
                __HAL_UNLOCK(hadc);
                SET_BIT(hadc->Instance->CR, ADC_CR_ADSTART);
            }
            else
            {
                __HAL_UNLOCK(hadc);
            } 
        }
        else
        {
            tmp_hal_status = HAL_ERROR;
            __HAL_UNLOCK(hadc);
        } 

    }
    else
    {   
        tmp_hal_status = HAL_BUSY;
    }

    return tmp_hal_status;

}
static rt_err_t stm32_adc_init(rt_device_t dev){
    //通道配置
    static rt_uint32_t rank[3] = {ADC_REGULAR_RANK_1,ADC_REGULAR_RANK_2,ADC_REGULAR_RANK_3};
    struct stm32_adc* padc = (struct stm32_adc*)dev->user_data;	
    ADC_ChannelConfTypeDef   sConfig;
    //ADC_MultiModeTypeDef     MultiModeInit;

    sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;    /* Minimum sampling time */
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
    sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
    sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
     
    //MultiModeInit.Mode = ADC_MODE_INDEPENDENT;
    //MultiModeInit.DualModeData = ADC_DUALMODEDATAFORMAT_DISABLED;  /* ADC and DMA configured in resolution 32 bits to match with both ADC master and slave resolution */
    //MultiModeInit.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;

    for (int cnt =0; cnt < ADC_CNT; cnt++){
        for (int cnl =0; cnl < ADC_CNL_CNT; cnl++){
            sConfig.Channel      = stm32_adc_get_channel(adc_channel[cnt][cnl]);                /* Sampled channel number */
            sConfig.Rank         = rank[cnl];          /* Rank of sampled channel number ADCx_CHANNEL */
            if (HAL_ADC_ConfigChannel(&padc->stm32_adc_cnl[cnt].ADC_Handler, &sConfig) != HAL_OK) {
                LOG_E("adcdma[%d][%d] config channel[%d] failed", cnt, cnl, adc_channel[cnt][cnl]);
                return -RT_ERROR;
            }
        } 

        if (HAL_ADCEx_Calibration_Start(&padc->stm32_adc_cnl[cnt].ADC_Handler, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
        {
            LOG_E("adcdma[%d] calibrate failed", cnt);
            return -RT_ERROR;
        }    
        /*if (HAL_ADCEx_MultiModeConfigChannel(&padc->stm32_adc_cnl[cnt].ADC_Handler, &MultiModeInit) != HAL_OK)
        {
            LOG_E("adcdma[%d] config multi mode failed", cnt);
            return -RT_ERROR;
        }*/   
    }  
    return RT_EOK;
}
static rt_err_t stm32_adc_open(rt_device_t dev, rt_uint16_t oflag){
	struct stm32_adc* padc = (struct stm32_adc*)dev->user_data;	
    for (int cnt =0; cnt < ADC_CNT; cnt++){
        rt_uint32_t base = ADC_DATA_BASE+ADC_BUFF_PHYS_SZ*cnt;
        rt_uint32_t length = ADC_BUFFS_SZ/ADC_BYTES;;        
		rt_memset((void*)base, 0xCC, ADC_BUFF_PHYS_SZ);       
        stm32_start_dma(&padc->stm32_adc_cnl[cnt], (uint32_t*)base, length);
	}
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    __HAL_TIM_CLEAR_IT(&padc->Tim_Handle, TIM_IT_UPDATE);
	//__HAL_TIM_ENABLE_IT(&padc->Tim_Handle, TIM_IT_UPDATE);
    //HAL_ADC_Start(&padc->stm32_adc_cnl[0].ADC_Handler);
	HAL_TIM_Base_Start(&padc->Tim_Handle);
	return RT_EOK;
}
static rt_err_t stm32_adc_close(rt_device_t dev){
	struct stm32_adc* padc = (struct stm32_adc*)dev->user_data;	    
	HAL_TIM_Base_Stop(&padc->Tim_Handle);	
	__HAL_TIM_SET_COUNTER(&padc->Tim_Handle, 0);
	__HAL_TIM_DISABLE_IT(&padc->Tim_Handle, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_IT(&padc->Tim_Handle, TIM_IT_UPDATE);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    rt_thread_delay(1);
   
    for (int cnt =0; cnt < ADC_CNT; cnt++){
        while (HAL_OK != HAL_ADC_PollForConversion(&padc->stm32_adc_cnl[cnt].ADC_Handler,100));
        HAL_ADC_Stop_IT(&padc->stm32_adc_cnl[cnt].ADC_Handler);
        //HAL_ADCEx_MultiModeStop_DMA(&padc->stm32_adc_cnl[cnt].ADC_Handler);
        ADC_Disable(&padc->stm32_adc_cnl[cnt].ADC_Handler);
		//HAL_ADC_Stop_DMA(&padc->stm32_adc_cnl[i].ADC_Handler);
		//rt_completion_wait(&(padc->stm32_adc_cnl[cnt].rx_comp), 0);
	}	
	return RT_EOK;
}
static rt_err_t stm32_adc_control(rt_device_t dev, int cmd, void *args){
	struct stm32_adc* padc = (struct stm32_adc*)dev->user_data;	
	switch (cmd){
		case RT_DEVICE_ADC_SET_FREQ :		{	
			  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
				uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
				uint32_t hz = *(uint32_t*)args;
				if (padc->rcc_clk.APB1CLKDivider == RCC_HCLK_DIV1)
				{
					timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
				}
				else
				{
					timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
				}
				timer_prescaler = timer_clock_frequency/ (TIMER_PRESCALER_MAX_VALUE * (hz /TIMER_FREQUENCY_RANGE_MIN)) +1;//(timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;	
				padc->Tim_Handle.Init.Period            = ((timer_clock_frequency / (timer_prescaler * hz)) - 1);
				padc->Tim_Handle.Init.Prescaler         = (timer_prescaler - 1);
				
		}break;
		/*case RT_DEVICE_ADC_WAIT_OFFSET:{
			int i = 0;
			rt_bool_t bFirstBlock = RT_TRUE;
			RT_ASSERT(args != RT_NULL);
			for (i = 0; i < sizeof(adc_config) / sizeof(adc_config[0]); i++){
				rt_completion_wait(&(padc->stm32_adc_cnl[i].rx_comp), RT_WAITING_FOREVER);
				if (i == 0)
					bFirstBlock = padc->stm32_adc_cnl[i].bFirstBlock;
				else if (bFirstBlock != padc->stm32_adc_cnl[i].bFirstBlock){
						rt_kprintf("not synchrous\n");
						return -RT_ERROR;
				}
			}
			if(bFirstBlock)
				*(rt_uint32_t*)args = 0;
			else
				*(rt_uint32_t*)args = 1;
		}
		break;*/
		default:
			return -RT_ENOSYS;
	}
	return RT_EOK;
}
static void stm32_adc_dma_config(struct stm32_adc_handler* cnl){
    
    if (cnl->ADC_Handler.Instance == ADC1){
        cnl->DMA_Handler.Instance = DMA1_Stream1;
        cnl->DMA_Handler.Init.Request             = DMA_REQUEST_ADC1;
        cnl->DMA_Handler.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        cnl->DMA_Handler.Init.PeriphInc           = DMA_PINC_DISABLE;
        cnl->DMA_Handler.Init.MemInc              = DMA_MINC_ENABLE;
#if ADC_BYTES==2        
        cnl->DMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  
        cnl->DMA_Handler.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;    
#else
        cnl->DMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  
        cnl->DMA_Handler.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;    
#endif  
        cnl->DMA_Handler.Init.Mode                = DMA_CIRCULAR;           
        cnl->DMA_Handler.Init.Priority            = DMA_PRIORITY_HIGH;
  
        HAL_DMA_DeInit(&cnl->DMA_Handler);
        HAL_DMA_Init(&cnl->DMA_Handler);

        __HAL_LINKDMA(&cnl->ADC_Handler, DMA_Handle, cnl->DMA_Handler);
  
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);        
    }
    else if (cnl->ADC_Handler.Instance == ADC2){
        cnl->DMA_Handler.Instance = DMA1_Stream2;
        cnl->DMA_Handler.Init.Request             = DMA_REQUEST_ADC2;
        cnl->DMA_Handler.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        cnl->DMA_Handler.Init.PeriphInc           = DMA_PINC_DISABLE;
        cnl->DMA_Handler.Init.MemInc              = DMA_MINC_ENABLE;
#if ADC_BYTES==2        
        cnl->DMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  
        cnl->DMA_Handler.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;    
#else
        cnl->DMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  
        cnl->DMA_Handler.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;    
#endif     
        cnl->DMA_Handler.Init.Mode                = DMA_CIRCULAR;           
        cnl->DMA_Handler.Init.Priority            = DMA_PRIORITY_HIGH;
  
        HAL_DMA_DeInit(&cnl->DMA_Handler);
        HAL_DMA_Init(&cnl->DMA_Handler);

        __HAL_LINKDMA(&cnl->ADC_Handler, DMA_Handle, cnl->DMA_Handler);
  
        HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn); 
    }
}
static int stm32_adc_dma_init(void)
{
    const char* name = "adcdma";
    int result = RT_EOK;
    rt_memset(&stm32_adc_obj, 0, sizeof(stm32_adc_obj));		
    stm32_triggle_source(ADC_FREQ);
    __HAL_RCC_DMA1_CLK_ENABLE();
    for (int i = 0; i < ADC_CNT; i++)
    {
        /* ADC init */
        stm32_adc_obj.stm32_adc_cnl[i].ADC_Handler = adc_config[i];
        //rt_completion_init(&(stm32_adc_obj.stm32_adc_cnl[i].rx_comp));
        __HAL_LINKDMA(&stm32_adc_obj.stm32_adc_cnl[i].ADC_Handler, DMA_Handle, stm32_adc_obj.stm32_adc_cnl[i].DMA_Handler);

        if (HAL_ADC_Init(&stm32_adc_obj.stm32_adc_cnl[i].ADC_Handler) != HAL_OK)
        {
            LOG_E("%s init failed", name);
            result = -RT_ERROR;
            return result;
        } 
        stm32_adc_dma_config(&stm32_adc_obj.stm32_adc_cnl[i]);
    }
    stm32_adc_obj.stm32_adc_device.init= stm32_adc_init;
    stm32_adc_obj.stm32_adc_device.open = stm32_adc_open;
    stm32_adc_obj.stm32_adc_device.close = stm32_adc_close;
    stm32_adc_obj.stm32_adc_device.control = stm32_adc_control;
    stm32_adc_obj.stm32_adc_device.user_data = &stm32_adc_obj;
    if (rt_device_register(&stm32_adc_obj.stm32_adc_device, name, RT_DEVICE_FLAG_RDWR) == RT_EOK)
    {
        LOG_D("%s init success", name);
    }
    else
    {
        LOG_E("%s register failed", name);
        result = -RT_ERROR;
    }
    return result;
}
INIT_BOARD_EXPORT(stm32_adc_dma_init);
volatile rt_uint32_t half_second = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
/*    if (htim->Instance == TIM3){
        static rt_tick_t t1,t2;
        if (half_second == 0){
            t1 = rt_tick_get();
        }
        else if (half_second == 100000){
            t2 = rt_tick_get() - t1;
            __nop();
       }
        half_second++;
    }
*/ 
}
void DMA1_Stream1_IRQHandler(void){
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&stm32_adc_obj.stm32_adc_cnl[ADC1_IDX].DMA_Handler);

    /* leave interrupt */
    rt_interrupt_leave();
}
void DMA1_Stream2_IRQHandler(void){
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_DMA_IRQHandler(&stm32_adc_obj.stm32_adc_cnl[ADC2_IDX].DMA_Handler);

    /* leave interrupt */
    rt_interrupt_leave();
}
void TIM3_IRQHandler(void){
    /* enter interrupt */
    rt_interrupt_enter();
    
	HAL_TIM_IRQHandler(&stm32_adc_obj.Tim_Handle);
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* BSP_USING_ADC */
