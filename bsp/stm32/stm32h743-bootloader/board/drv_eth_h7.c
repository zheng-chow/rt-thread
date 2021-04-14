/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-19     SummerGift   first version
 * 2018-12-25     zylx         fix some bugs
 * 2019-06-10     SummerGift   optimize PHY state detection process
 * 2019-09-03     xiaofan      optimize link change detection process
 */

#include "board.h"
#include "drv_config.h"
#include <netif/ethernetif.h>
#include "lwipopts.h"
#include "drv_eth_h7.h"




/*
* Emac driver uses CubeMX tool to generate emac and phy's configuration,
* the configuration files can be found in CubeMX_Config floder.
*/

/* debug option */
//#define ETH_RX_DUMP
//#define ETH_TX_DUMP
//#define DRV_DEBUG
#define LOG_TAG             "drv.emac"
#include <drv_log.h>

#define MAX_ADDR_LEN 6

struct rt_stm32_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;
#ifndef PHY_USING_INTERRUPT_MODE
    rt_timer_t poll_link_timer;
#endif

    /* interface address info, hw address */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];
    /* ETH_Speed */
    uint32_t    ETH_Speed;
    /* ETH_Duplex_Mode */
    uint32_t    ETH_Mode;

		uint32_t 		PhyAddress;
};

//static ETH_DMADescTypeDef *DMARxDscrTab, *DMATxDscrTab;
//static rt_uint8_t *Rx_Buff, *Tx_Buff;


#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */ 

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

static  ETH_HandleTypeDef EthHandle;
static ETH_TxPacketConfig TxConfig;
static struct rt_stm32_eth stm32_eth_device;
#ifdef     PHY_USING_IP101G
static int DumpPhy(void);
#endif
#if defined(ETH_RX_DUMP) || defined(ETH_TX_DUMP)
#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
static void dump_hex(const rt_uint8_t *ptr, rt_size_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    int i, j;

    for (i = 0; i < buflen; i += 16)
    {
        rt_kprintf("%08X: ", i);

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%02X ", buf[i + j]);
            else
                rt_kprintf("   ");
        rt_kprintf(" ");

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
        rt_kprintf("\n");
    }
}
#endif

extern void phy_reset(void);
/* EMAC initialization function */
static rt_err_t rt_stm32_eth_init(rt_device_t dev)
{
	ETH_MACConfigTypeDef MACConf;	
/*    GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)GPIO_PIN_RESET);*/
	
   //phy_reset();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (GPIO_PinState)GPIO_PIN_RESET);
	rt_thread_mdelay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (GPIO_PinState)GPIO_PIN_SET);
	rt_thread_mdelay(200);
/*	
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	*/
	
    /* ETHERNET Configuration */
    EthHandle.Instance = ETH;
    EthHandle.Init.MACAddr = (rt_uint8_t *)&stm32_eth_device.dev_addr[0];
    EthHandle.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
	
    EthHandle.Init.RxDesc = DMARxDscrTab;
    EthHandle.Init.TxDesc = DMATxDscrTab;
    EthHandle.Init.RxBuffLen = ETH_MAX_PACKET_SIZE;

    /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
    if (HAL_ETH_Init(&EthHandle) != HAL_OK)
    {
        LOG_E("eth hardware init failed");
    }
    else
    {
        LOG_D("eth hardware init success");
    }

		
	for(uint32_t idx = 0; idx < ETH_RX_DESC_CNT; idx ++)
	{
		HAL_ETH_DescAssignMemory(&EthHandle, idx, Rx_Buff[idx], NULL);

		/* Set Custom pbuf free function */
		//rx_pbuf[idx].custom_free_function = pbuf_free_custom;
	}
	
	rt_memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));  
	TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
	TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
	TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

	
	HAL_ETH_SetMDIOClockRange(&EthHandle);
	
	/* ETH interrupt Init */
	HAL_NVIC_SetPriority(ETH_IRQn, 0x00, 0);
	HAL_NVIC_EnableIRQ(ETH_IRQn);

	
	HAL_ETH_GetMACConfig(&EthHandle, &MACConf); 
	MACConf.DuplexMode = stm32_eth_device.ETH_Mode;
	MACConf.Speed = stm32_eth_device.ETH_Speed;
	HAL_ETH_SetMACConfig(&EthHandle, &MACConf);

    /* Enable MAC and DMA transmission and reception */
    if (HAL_ETH_Start_IT(&EthHandle) == HAL_OK)
    {
        LOG_D("emac hardware start");
    }
    else
    {
        LOG_E("emac hardware start faild");
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t rt_stm32_eth_open(rt_device_t dev, rt_uint16_t oflag)
{
    LOG_D("emac open");
    return RT_EOK;
}

static rt_err_t rt_stm32_eth_close(rt_device_t dev)
{
    LOG_D("emac close");
    return RT_EOK;
}

static rt_size_t rt_stm32_eth_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    LOG_D("emac read");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_stm32_eth_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    LOG_D("emac write");
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_stm32_eth_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args) rt_memcpy(args, stm32_eth_device.dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

/* ethernet device interface */
/* transmit data*/
rt_err_t rt_stm32_eth_tx(rt_device_t dev, struct pbuf *p){
	uint32_t i=0, framelen = 0;
	struct pbuf *q;
	err_t errval = ERR_OK;
	ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
  
	
	rt_memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

	for(q = p; q != NULL; q = q->next)
	{
		if(i >= ETH_TX_DESC_CNT)	
			return ERR_IF;

		Txbuffer[i].buffer = q->payload;
		Txbuffer[i].len = q->len;
		framelen += q->len;

		if(i>0)
		{
			Txbuffer[i-1].next = &Txbuffer[i];
		}

		i++;
	}

	TxConfig.Length = framelen;
	TxConfig.TxBuffer = Txbuffer;

	/* Clean and Invalidate data cache */
	SCB_CleanInvalidateDCache();  

	if (HAL_OK != HAL_ETH_Transmit(&EthHandle, &TxConfig, 5))//HAL_MAX_DELAY //Transmit an ETH frame in blocking mode
        LOG_I("emac tx timeout");


  return errval;	
	
}
/* receive data*/
struct pbuf *rt_stm32_eth_rx(rt_device_t dev){

	struct pbuf* p = RT_NULL;
	ETH_BufferTypeDef RxBuff;
	uint32_t framelength = 0;

	/* Clean and Invalidate data cache */
	SCB_CleanInvalidateDCache();

	if(HAL_ETH_GetRxDataBuffer(&EthHandle, &RxBuff) == HAL_OK) {
        HAL_ETH_GetRxDataLength(&EthHandle, &framelength);

		p = pbuf_alloc(PBUF_LINK, framelength, PBUF_RAM);
		if (p){
			/* copy to the pbuf */
			pbuf_take(p, (void*)RxBuff.buffer, framelength);    
		}
		else{
				rt_kprintf("alloc rx pbuf failed\n");
		}
			
        /*p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_POOL, &rx_pbuf[current_pbuf_idx], RxBuff.buffer, framelength);

        if(current_pbuf_idx < (ETH_RX_DESC_CNT -1))
        {
            current_pbuf_idx++;
        }
        else
        {
            current_pbuf_idx = 0;
        }*/
	}
	HAL_ETH_BuildRxDescriptors(&EthHandle);

 	return p;
}
/* interrupt service routine */
void ETH_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_ETH_IRQHandler(&EthHandle);

    /* leave interrupt */
    rt_interrupt_leave();
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    rt_err_t result;
    result = eth_device_ready(&(stm32_eth_device.parent));
    if (result != RT_EOK)
        LOG_D("RxCpltCallback err = %d", result);
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
    LOG_E("eth err");
}

enum {
    PHY_LINK        = (1 << 0),
    PHY_100M        = (1 << 1),
    PHY_FULL_DUPLEX = (1 << 2),
};

static void phy_linkchange()
{
    static rt_uint8_t phy_speed = 0;
    rt_uint8_t phy_speed_new = 0;
    rt_uint32_t status;


    HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, PHY_BASIC_STATUS_REG, (uint32_t *)&status);	
    LOG_D("phy basic status reg is 0x%X", status);

    if (status & (PHY_AUTONEGO_COMPLETE_MASK | PHY_LINKED_STATUS_MASK))
    {
        rt_uint32_t SR;

        phy_speed_new |= PHY_LINK;

        SR = HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, PHY_Status_REG, (uint32_t *)&status);
        LOG_D("phy control status reg is 0x%X", status);

        if (PHY_Status_SPEED_100M(status))
        {
            phy_speed_new |= PHY_100M;
        }

        if (PHY_Status_FULL_DUPLEX(status))
        {
            phy_speed_new |= PHY_FULL_DUPLEX;
        }
    }

    if (phy_speed != phy_speed_new)
    {
        phy_speed = phy_speed_new;
        if (phy_speed & PHY_LINK)
        {
            LOG_D("link up");
            if (phy_speed & PHY_100M)
            {
                LOG_D("100Mbps");
                stm32_eth_device.ETH_Speed = ETH_SPEED_100M;
            }
            else
            {
                stm32_eth_device.ETH_Speed = ETH_SPEED_10M;
                LOG_D("10Mbps");
            }

            if (phy_speed & PHY_FULL_DUPLEX)
            {
                LOG_D("full-duplex");
                stm32_eth_device.ETH_Mode = ETH_MODE_FULLDUPLEX;
            }
            else
            {
                LOG_D("half-duplex");
                stm32_eth_device.ETH_Mode = ETH_MODE_HALFDUPLEX;
            }
            LOG_I("link up");

            /* send link up. */
            eth_device_linkchange(&stm32_eth_device.parent, RT_TRUE);
        }
        else
        {
            LOG_I("link down");
            eth_device_linkchange(&stm32_eth_device.parent, RT_FALSE);
        }
    }
}

#ifdef PHY_USING_INTERRUPT_MODE
static void eth_phy_isr(void *args)
{
    rt_uint32_t status = 0;

    HAL_ETH_ReadPHYRegister(&EthHandle, PHY_INTERRUPT_FLAG_REG, (uint32_t *)&status);
    LOG_D("phy interrupt status reg is 0x%X", status);

    phy_linkchange();
}
#endif /* PHY_USING_INTERRUPT_MODE */

static void phy_monitor_thread_entry(void *parameter)
{
    uint8_t phy_addr = 0xFF;
    uint8_t detected_count = 0;
	rt_uint32_t status;
    while(phy_addr == 0xFF){
        /* phy search */
        rt_uint32_t i, temp;
        for (i = 0; i <= 0x1F; i++)
        {
            stm32_eth_device.PhyAddress	= i;
			HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, PHY_ID1_REG, (uint32_t *)&temp);

            if (temp != 0xFFFF && temp != 0x00)
            {
                phy_addr = i;
                break;
            }
        }

        detected_count++;
        rt_thread_mdelay(1000);

        if (detected_count > 10)
        {
            LOG_E("No PHY device was detected, please check hardware!");
        }
    }

    LOG_D("Found a phy, address:0x%02X", phy_addr);
    /* RESET PHY */
    LOG_D("RESET PHY!");  
    HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, PHY_BASIC_CONTROL_REG, (uint32_t *)&status);
    HAL_ETH_WritePHYRegister(&EthHandle, stm32_eth_device.PhyAddress, PHY_BASIC_CONTROL_REG, PHY_RESET_MASK|status);
    detected_count = 0;
    do{
        rt_thread_mdelay(200);
        HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, PHY_BASIC_CONTROL_REG, (uint32_t *)&status);
        detected_count++;
    }while ((status & PHY_RESET_MASK) && (detected_count < 10));
    if (status & PHY_RESET_MASK)
        LOG_D("RESET PHY FAILED!");	
    
#if defined(PHY_USING_IP101G)
    HAL_ETH_WritePHYRegister(&EthHandle, stm32_eth_device.PhyAddress, 17, 1<<15);
    HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, 16, (uint32_t *)&status);
    HAL_ETH_WritePHYRegister(&EthHandle, stm32_eth_device.PhyAddress, 16, status | 2 |(1<<7));
#endif    

    HAL_ETH_WritePHYRegister(&EthHandle, stm32_eth_device.PhyAddress, PHY_BASIC_CONTROL_REG, PHY_AUTO_NEGOTIATION_MASK);    
    phy_linkchange();
#ifdef PHY_USING_INTERRUPT_MODE
    /* configuration intterrupt pin */
    rt_pin_mode(PHY_INT_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(PHY_INT_PIN, PIN_IRQ_MODE_FALLING, eth_phy_isr, (void *)"callbackargs");
    rt_pin_irq_enable(PHY_INT_PIN, PIN_IRQ_ENABLE);

    /* enable phy interrupt */
    HAL_ETH_WritePHYRegister(&EthHandle, PHY_INTERRUPT_MASK_REG, PHY_INT_MASK);
#if defined(PHY_INTERRUPT_CTRL_REG)
    HAL_ETH_WritePHYRegister(&EthHandle, PHY_INTERRUPT_CTRL_REG, PHY_INTERRUPT_EN);
#endif
#else /* PHY_USING_INTERRUPT_MODE */
    stm32_eth_device.poll_link_timer = rt_timer_create("phylnk", (void (*)(void*))phy_linkchange,
                                        NULL, RT_TICK_PER_SECOND, RT_TIMER_FLAG_PERIODIC);
    if (!stm32_eth_device.poll_link_timer || rt_timer_start(stm32_eth_device.poll_link_timer) != RT_EOK)
    {
        LOG_E("Start link change detection timer failed");
    }
#endif /* PHY_USING_INTERRUPT_MODE */
}

/* Register the EMAC device */
static int rt_hw_stm32_eth_init(void)
{
    rt_err_t state = RT_EOK;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /*__HAL_RCC_GPIOC_CLK_ENABLE();
    Configure GPIO pin : PC9 
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);*/	

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);		
	
   
    stm32_eth_device.ETH_Speed = ETH_SPEED_100M;
    stm32_eth_device.ETH_Mode  = ETH_MODE_FULLDUPLEX;

    /* OUI CC-80-E1 STMICROELECTRONICS. */
    stm32_eth_device.dev_addr[0] = 0xCC;
    stm32_eth_device.dev_addr[1] = 0x80;
    stm32_eth_device.dev_addr[2] = 0xE1;
    /* generate MAC addr from 96bit unique ID (only for test). */
    stm32_eth_device.dev_addr[3] = *(rt_uint8_t *)(UID_BASE + 4);
    stm32_eth_device.dev_addr[4] = *(rt_uint8_t *)(UID_BASE + 2);
    stm32_eth_device.dev_addr[5] = *(rt_uint8_t *)(UID_BASE + 0);

    stm32_eth_device.parent.parent.init       = rt_stm32_eth_init;
    stm32_eth_device.parent.parent.open       = rt_stm32_eth_open;
    stm32_eth_device.parent.parent.close      = rt_stm32_eth_close;
    stm32_eth_device.parent.parent.read       = rt_stm32_eth_read;
    stm32_eth_device.parent.parent.write      = rt_stm32_eth_write;
    stm32_eth_device.parent.parent.control    = rt_stm32_eth_control;
    stm32_eth_device.parent.parent.user_data  = RT_NULL;

    stm32_eth_device.parent.eth_rx     = rt_stm32_eth_rx;
    stm32_eth_device.parent.eth_tx     = rt_stm32_eth_tx;

  /* register eth device */
    state = eth_device_init(&(stm32_eth_device.parent), "e0");
    if (RT_EOK == state)
    {
        LOG_D("emac device init success");
    }
    else
    {
        LOG_E("emac device init faild: %d", state);
        state = -RT_ERROR;
        goto __exit;
    }

    /* start phy monitor */
    rt_thread_t tid;
    tid = rt_thread_create("phy",
                           phy_monitor_thread_entry,
                           RT_NULL,
                           1024,
                           RT_THREAD_PRIORITY_MAX - 2,
                           2);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
    }
    else
    {
        state = -RT_ERROR;
    }
__exit:
    
    return state;
}
INIT_DEVICE_EXPORT(rt_hw_stm32_eth_init);
#ifdef     PHY_USING_IP101G
static int DumpPhy(void){
	rt_uint32_t status;
    //HAL_ETH_WritePHYRegister(&EthHandle, stm32_eth_device.PhyAddress, PHY_BASIC_CONTROL_REG, PHY_AUTO_NEGOTIATION_MASK|PHY_AUTO_NEGOTIATION_RESTART_MASK);
    rt_kprintf("\n");
    for (uint32_t n = 0; n< 9; n++){
        HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, n, (uint32_t *)&status);
        rt_kprintf("REG%02d  : %04X\n", n, status);
    }
    HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, 20, (uint32_t *)&status);
    rt_kprintf("REG%02d  : %04X\n", 20, status);
    for (uint32_t n = 16; n< 19; n++){
        HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, n, (uint32_t *)&status);
        rt_kprintf("REG%02d  : %04X\n", n, status);
    }
    for (uint32_t n = 26; n< 31; n++){
        HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, n, (uint32_t *)&status);
        rt_kprintf("REG%02d  : %04X\n", n, status);
    }
    return 0;
}
MSH_CMD_EXPORT(DumpPhy, dump phy register)
void GetPhyStatus(void){
 	rt_uint32_t status;
    HAL_ETH_ReadPHYRegister(&EthHandle, stm32_eth_device.PhyAddress, 5, (uint32_t *)&status);
    rt_kprintf("PhyStatus: %04X\n", status);
}
#endif

