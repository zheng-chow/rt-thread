#ifndef __DRV_ADC_DMA_H__
#define __DRV_ADC_DMA_H__

#include <drv_common.h>
#include <board.h>

#define RT_DEVICE_ADC_SET_FREQ 0x01
//#define RT_DEVICE_ADC_WAIT_OFFSET 0x02
#define ADC_FREQ            200000//200000
#define ADC_SLICE_TIME      40
#define ADC_BYTES           2
#define ADC_BUFF_PHYS_SZ	ADC_BUFFS_SZ
#define ADC_DATA_BASE	    (0xC0000000)

#define ADC_CNL_CNT     3


#define ADC_BUFF_SLICE_SZ	(ADC_SLICE_TIME * ADC_FREQ * ADC_CNL_CNT /1000)
#define ADC_BUFFS_SZ		((((ADC_FREQ*ADC_CNL_CNT/ADC_BUFF_SLICE_SZ)+1) & 0xFFFFFFFE)*ADC_BUFF_SLICE_SZ*ADC_BYTES)//缓存1s以上数据


rt_uint32_t stm32_rx_info_get_adc_addr(rt_uint32_t info);
rt_uint32_t stm32_rx_info_get_adc_length(rt_uint32_t info);
rt_uint32_t stm32_rx_info_get_adc_index(rt_uint32_t info);
#endif

