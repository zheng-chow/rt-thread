#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_hall.h"

#define DRV_DEBUG
#define LOG_TAG             "drv.hall"
#include <drv_log.h>
#define MAX_TIME        0.05
#define TIME_ACCURACY 0.000001 //s
#define TIME_PERIOD 10000
#define TIME_ACCURACY2 (TIME_ACCURACY * TIME_PERIOD) //0.01
#define TIME_PERIOD2    (MAX_TIME/ TIME_ACCURACY2) //10000

#define SPEED_TIME_THRD	(0.001)
struct stm32_hall{
    struct rt_device        parent;
    char*                   name;
    TIM_HandleTypeDef       hall_handle;
    TIM_HandleTypeDef       tim10us_handle;
    TIM_HandleTypeDef       tim100ms_handle;
    uint8_t                 idxFlag;
    int16_t*                baseAngles;
    float                   encSpeed; 
    float                   expSpeed; 
		float										expAngle;
    //float                   expAcc;
    rt_bool_t               bTimeout;
};
/*
positive speed: 5->1->3->2->6->4

#if defined(AXIS_PITCH)
static int16_t baseAngleTable[8] = {-1, 0 + OFFS_ANGLE, 120+OFFS_ANGLE, 60 + OFFS_ANGLE, 240 + OFFS_ANGLE, 300 + OFFS_ANGLE, 180 + OFFS_ANGLE, -1};
#elif defined(AXIS_YAW)
static int16_t baseAngleTable[8] = {-1, 0 + OFFS_ANGLE, 120+OFFS_ANGLE, 60 + OFFS_ANGLE, 240 + OFFS_ANGLE, 300 + OFFS_ANGLE, 180 + OFFS_ANGLE, -1};
#else
#error "undefine hall AngleTable"
#endif */
//static int16_t baseAngleTable[8] = {-1, 0, 120, 60, 240, 300, 180, -1};
//-178
//static int16_t baseAngleTable[8] = {-1, 0 + OFFS_ANGLE, 120+OFFS_ANGLE, 60 + OFFS_ANGLE, 240 + OFFS_ANGLE, 300 + OFFS_ANGLE, 180 + OFFS_ANGLE, -1};
static int16_t baseAngleTable[8] = {-1, 360 + OFFS_ANGLE, 360+120+OFFS_ANGLE, 360+60 + OFFS_ANGLE, 240 + OFFS_ANGLE, 300 + OFFS_ANGLE, 180 + OFFS_ANGLE, -1};

static struct stm32_hall hall_object={
      .name = "hall",
      .hall_handle.Instance = TIM3,
      .tim10us_handle.Instance = TIM4,
      .tim100ms_handle.Instance = TIM2,
      .baseAngles = baseAngleTable,
};
static uint8_t GetQuadrantIndex(){
    //LOG_D("Idx : %u",(GPIOC->IDR >> 6) & 0x07);
    return (GPIOC->IDR >> 6) & 0x07;
}
static float GetQuadrantTime(void){
    return TIM2->CNT * TIME_ACCURACY2 + TIM4->CNT * TIME_ACCURACY;
}
static void ResetQuadrantTime(void){
    TIM2->CNT = 0;
    TIM4->CNT = 0;
    hall_object.bTimeout = RT_FALSE;
}
static float GetQuadrantSpeed(int idxLast, int idxCurr){
		return hall_object.encSpeed;	
	
    if (hall_object.bTimeout) return 0;
    float t = GetQuadrantTime();
		if ((t > SPEED_TIME_THRD) && (SPEED_TIME_THRD!=0)) return hall_object.encSpeed;	
    float a = hall_object.baseAngles[idxCurr] - hall_object.baseAngles[idxLast];
    if (a <= -180) a+=360;
    else if (a > 180) a-=360;
    return a / t;
}
static rt_err_t rt_stm32_hall_timer_init(rt_device_t dev){
    rt_err_t result = RT_EOK;
    
    struct stm32_hall* hall = (struct stm32_hall*)dev;
    TIM_HandleTypeDef *hhall = &hall->hall_handle;
    TIM_HallSensor_InitTypeDef      sHallSensorConfig;
    //TIM_ClockConfigTypeDef          sClockConfig = {0};
    rt_uint64_t tim_clock = HAL_RCC_GetPCLK1Freq() * 2;
    
    hhall->Init.Prescaler = 0;//((uint32_t)(tim_clock * TIME_ACCURACY)) - 1;
    hhall->Init.CounterMode = TIM_COUNTERMODE_UP;
    hhall->Init.Period = 0;//TIME_PERIOD-1;//pwm->period-1;
    hhall->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    hhall->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    

    sHallSensorConfig.IC1Prescaler=TIM_ICPSC_DIV1;
    sHallSensorConfig.IC1Polarity=TIM_ICPOLARITY_BOTHEDGE;
    sHallSensorConfig.IC1Filter=0;
    sHallSensorConfig.Commutation_Delay=0;  
    /*sClockConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(hhall, &sClockConfig) != HAL_OK)
    {
        LOG_E("%s clock init failed", hall->name);
        result = -RT_ERROR;
        goto __exit;
    } */ 
    
    if (HAL_TIMEx_HallSensor_Init(hhall, &sHallSensorConfig)!= HAL_OK)    {
        LOG_E("%s hall sensor init failed", hall->name);
        result = -RT_ERROR;
        goto __exit;
    }
		
   /*
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;//TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(hhall, &sMasterConfig) != HAL_OK)
    {
        LOG_E("%s master config failed", hall->name);
        result = -RT_ERROR;
        goto __exit;
    }  
*/
  
 __exit:
    return result;    
}
static rt_err_t rt_stm32_hall_timer_10us_init(rt_device_t dev){
    rt_err_t result = RT_EOK;       
    struct stm32_hall* hall = (struct stm32_hall*)dev;
    TIM_HandleTypeDef *htim = &hall->tim10us_handle;
    TIM_ClockConfigTypeDef          sClockConfig = {0};
    TIM_MasterConfigTypeDef         sMasterConfig;
    rt_uint64_t tim_clock = HAL_RCC_GetPCLK1Freq() * 2;
    
    htim->Init.Prescaler = ((uint32_t)(tim_clock * TIME_ACCURACY)) - 1;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = TIME_PERIOD-1;//pwm->period-1;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        LOG_E("%s time base init failed", hall->name);
        result = -RT_ERROR;
        goto __exit;
    }   
    
    sClockConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(htim, &sClockConfig) != HAL_OK)
    {
        LOG_E("%s clock init failed", hall->name);
        result = -RT_ERROR;
        goto __exit;
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;//TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
    {
        LOG_E("%s master config failed", hall->name);
        result = -RT_ERROR;
        goto __exit;
    } 
    
__exit:
    return result;    
}

static rt_err_t rt_stm32_hall_timer_100ms_init(rt_device_t dev){
    rt_err_t result = RT_EOK;       
    struct stm32_hall* hall = (struct stm32_hall*)dev;
    TIM_HandleTypeDef *htim = &hall->tim100ms_handle;
    TIM_ClockConfigTypeDef          sClockConfig = {0};
    rt_uint64_t tim_clock = HAL_RCC_GetPCLK1Freq() * 2;
    
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = TIME_PERIOD2-1;//pwm->period-1;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
        LOG_E("%s time base init failed", hall->name);
        result = -RT_ERROR;
        goto __exit;
    }   
    
    sClockConfig.ClockSource = TIM_CLOCKSOURCE_ITR3;
    if (HAL_TIM_ConfigClockSource(htim, &sClockConfig) != HAL_OK)
    {
        LOG_E("%s clock init failed", hall->name);
        result = -RT_ERROR;
        goto __exit;
    }
__exit:
    return result;    
}



static rt_err_t  rt_stm32_hall_init(rt_device_t dev){
    rt_err_t result = RT_EOK;
    result = rt_stm32_hall_timer_init(dev);
    if (result != RT_EOK){
        LOG_E("init hall timer failed");
       goto __exit;
    }
    result = rt_stm32_hall_timer_10us_init(dev);
    if (result != RT_EOK){
        LOG_E("init hall timer 10us failed");
       goto __exit;
    }   
    result = rt_stm32_hall_timer_100ms_init(dev);
    if (result != RT_EOK){
        LOG_E("init hall timer 100ms failed");
       goto __exit;
    }   
    
 __exit:
    return result;
}
static rt_err_t  rt_stm32_hall_open(rt_device_t dev, rt_uint16_t oflag){
    struct stm32_hall* hall = (struct stm32_hall*)dev;
    TIM_HandleTypeDef *hhall = &hall->hall_handle;
    TIM_HandleTypeDef *htim_10us = &hall->tim10us_handle;
    TIM_HandleTypeDef *htim_100ms = &hall->tim100ms_handle;
    RT_ASSERT(&hall_object == hall); 
    
    hall->bTimeout = RT_FALSE;
    hall->idxFlag = GetQuadrantIndex();
    if ((hall->idxFlag <1) || (hall->idxFlag > 7)) return -RT_EIO;
    LOG_I("hall state: %u", hall->idxFlag);
		hall->expAngle = hall_object.baseAngles[hall->idxFlag];
    __HAL_TIM_CLEAR_IT(htim_100ms, TIM_IT_UPDATE);

    
    //__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(htim_100ms);
    HAL_TIM_Base_Start(htim_10us);
    
    __HAL_TIM_CLEAR_IT(hhall, TIM_IT_CC1);
    __HAL_TIM_CLEAR_IT(hhall, TIM_IT_CC2);
    __HAL_TIM_CLEAR_IT(hhall, TIM_IT_CC3);
    HAL_TIMEx_HallSensor_Start(hhall);
    HAL_TIM_IC_Start_IT(hhall, TIM_CHANNEL_1); 
    HAL_TIM_IC_Start_IT(hhall, TIM_CHANNEL_2); 
    HAL_TIM_IC_Start_IT(hhall, TIM_CHANNEL_3); 
    
    __HAL_TIM_SET_COUNTER(htim_10us, 0);
    __HAL_TIM_SET_COUNTER(htim_100ms, 0);


		HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    return RT_EOK;
}
static rt_err_t  rt_stm32_hall_close(rt_device_t dev){
    struct stm32_hall* hall = (struct stm32_hall*)dev;
    TIM_HandleTypeDef *hhall = &hall->hall_handle;
    TIM_HandleTypeDef *htim_10us = &hall->tim10us_handle;
    TIM_HandleTypeDef *htim_100ms = &hall->tim100ms_handle;
    RT_ASSERT(&hall_object == hall);
    
    NVIC_DisableIRQ(TIM2_IRQn);
    NVIC_DisableIRQ(TIM3_IRQn);
    HAL_TIM_IC_Stop_IT(hhall, TIM_CHANNEL_1); 
    HAL_TIM_IC_Stop_IT(hhall, TIM_CHANNEL_2); 
    HAL_TIM_IC_Stop_IT(hhall, TIM_CHANNEL_3); 
    HAL_TIMEx_HallSensor_Stop(hhall);
    HAL_TIM_Base_Stop_IT(htim_100ms);
    //__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
     
    __HAL_TIM_SET_COUNTER(htim_10us, 0);
    __HAL_TIM_SET_COUNTER(htim_100ms, 0);
    hall->bTimeout = RT_TRUE; 
    return RT_EOK;
}
static rt_size_t rt_stm32_hall_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size){
    struct stm32_hall* hall = (struct stm32_hall*)dev;
    //TIM_HandleTypeDef *htim = &hall->tim_handle;
    //TIM_HandleTypeDef *hhall = &hall->hall_handle;
    RT_ASSERT(&hall_object == hall);
    if (size != 4) return 0;
    switch(pos){
        case 0://½Ç¶È
        {
            float base = hall->expAngle;//hall->baseAngles[hall->idxFlag];
            float t = GetQuadrantTime();
            float dAngle = hall->expSpeed*t;// + 0.5*hall->expAcc*t*t;
            if (dAngle>60) dAngle = 60;
            else if (dAngle < -60) dAngle = -60;
            *(float*)buffer = base + dAngle;
        }
            break;
        case 1://ËÙ¶È
        {
            float t = GetQuadrantTime();
            *(float*)buffer = hall->expSpeed;//+hall->expAcc*t;
        }
            break;
        default:
            return 0;
    }
    return 4;
}
void hall_fast_set_speed(float speed){
  //float t = GetQuadrantTime();
	hall_object.encSpeed = speed;
	//if (t > SPEED_TIME_THRD) 
		hall_object.expSpeed = speed;
}
float hall_fast_get_angle(void){
#ifdef TEST
    float base = 0;
#else
    float base = hall_object.expAngle;  ////hall_object.baseAngles[hall_object.idxFlag];    
#endif  
		float t = GetQuadrantTime();
    float dAngle = hall_object.expSpeed*t ;//+ 0.5 * hall_object.expAcc*t*t;
    if (dAngle>60) dAngle = 60;
    else if (dAngle < -60) dAngle = -60;         
    return base + dAngle;
}
float hall_fast_get_speed(void){
   return hall_object.expSpeed;//+hall_object.expAcc*t;
}
/*float hall_fast_clear_speed(void){
    hall_object.bTimeout = RT_TRUE;
    hall_object.expSpeed= 0;
}*/
void TIM3_IRQHandler(void){
    rt_interrupt_enter();
    HAL_TIM_IRQHandler(&hall_object.hall_handle);     
    rt_interrupt_leave();
}
void TIM2_IRQHandler(void){
    TIM_HandleTypeDef *htim = &hall_object.tim100ms_handle;
    rt_interrupt_enter();
    //HAL_TIM_IRQHandler(&hall_object.tim100ms_handle);     
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET){
    if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET){
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
        hall_object.bTimeout = RT_TRUE;    
        hall_object.expSpeed = 0;        
        //hall_object.expAcc = 0;        
    }
  }
    rt_interrupt_leave();
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if (htim == &hall_object.hall_handle){
        uint8_t idxFlag = GetQuadrantIndex();
        if ((idxFlag == 0) || (idxFlag == 7)){
            rt_kprintf("hall get index: %d\n", idxFlag);
        }
        else if (idxFlag != hall_object.idxFlag ){
						hall_object.expAngle = hall_object.baseAngles[idxFlag];
            hall_object.expSpeed = GetQuadrantSpeed(hall_object.idxFlag, idxFlag);
            ResetQuadrantTime();
           //float t = GetQuadrantTime();
            //hall_object.expAcc = (newSpeed - hall_object.expSpeed)/t;
            hall_object.idxFlag = idxFlag;
        }
        //if (hall_object.parent.rx_indicate)
        //    hall_object.parent.rx_indicate(&hall_object.parent,1);
        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10); 
    }
}
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim == &hall_object.tim100ms_handle){        
        hall_object.bTimeout = RT_TRUE;
    }   
}*/
static int stm32_hall_init(void)
{
    int result = RT_EOK;
    //rt_memset(&pwm_object, 0 , sizeof(pwm_object));
    //pwm_object.name = "pwmcom";
    hall_object.parent.type = RT_Device_Class_Miscellaneous;
    hall_object.parent.init         = rt_stm32_hall_init;
    hall_object.parent.open         = rt_stm32_hall_open;
    hall_object.parent.close        = rt_stm32_hall_close;
    hall_object.parent.read         = rt_stm32_hall_read;
    hall_object.parent.write        = RT_NULL;
    hall_object.parent.control      = RT_NULL; 
    result =   rt_device_register(&hall_object.parent, hall_object.name, RT_DEVICE_FLAG_RDWR);
    return result;
}
INIT_DEVICE_EXPORT(stm32_hall_init);
