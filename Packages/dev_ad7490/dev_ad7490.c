/*
 * Copyright (c)	BC Lab 2021
 *
 * Creator	: Thompson
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-25     Thompson     Create this driver
 
 * 2021-03-04			Thompson		 Add force take and release functions
 *
 */

#include "dev_ad7490.h"
//#include "rtthread.h"

#define CS_PULL_UP 		HAL_GPIO_WritePin(dev->cs_gpiox, dev->cs_gpio_pin, GPIO_PIN_SET)
#define CS_PULL_DOWN	HAL_GPIO_WritePin(dev->cs_gpiox, dev->cs_gpio_pin, GPIO_PIN_RESET)
static const uint16_t power_up_seq = 0xffff;


static void dev_ad7490_config(dev_ad7490_t* dev);
static void dev_ad7490_generate_shadow_seq(dev_ad7490_t* dev,uint16_t chns);
																

/*	device control function group	*/
static void dev_ad7490_set_range(dev_ad7490_t* dev,Range_t range);
static void dev_ad7490_set_option_mode(dev_ad7490_t* dev,OP_mode_t opmode);
static void dev_ad7490_set_power_mode(dev_ad7490_t* dev,Pwr_mode_t pwrmode);

/*	fetch data function group	*/
static void dev_ad7490_fetch_data_in_independent_mode(dev_ad7490_t* dev);
static void dev_ad7490_fetch_data_in_regular_scan_mode(dev_ad7490_t* dev);
static void dev_ad7490_fetch_data_in_continuous_mode(dev_ad7490_t* dev);

static void dev_ad7490_power_up(dev_ad7490_t* dev);


void dev_ad7490_init(dev_ad7490_t* dev)
{
	/* Initialize CS pin	*/
	GPIO_InitTypeDef GPIO_Initure;
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Initure.Pin = dev->cs_gpio_pin;
	GPIO_Initure.Pull = GPIO_PULLUP;
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(dev->cs_gpiox, &GPIO_Initure);
	CS_PULL_UP;
	delay_us(20);
	/*	Generate configuration control sequence	*/
	dev->pak.WRITE = 1;
	dev_ad7490_config(dev);
	dev->DIN = dev_ad7490_generate_control_seq(dev);
	/*****	Powering up AD7490	*****/
	/*	SPI start signal, A.K.A., CS pin pull down	*/
	/*	First CS signal	*/
	CS_PULL_DOWN;
	delay_us(5);
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Transmit_DMA(dev->bus_handle,&power_up_seq,16);
	#else
		HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&power_up_seq,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	delay_us(5);
	CS_PULL_UP;
	delay_us(20);
	/*	Second CS signal	*/
	CS_PULL_DOWN;
	delay_us(5);
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Transmit_DMA(dev->bus_handle,&power_up_seq,1);
	#else
		HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&power_up_seq,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	delay_us(5);
	CS_PULL_UP;
	delay_us(20);
	CS_PULL_DOWN;
	delay_us(5);
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Transmit_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,1);
	#else
		HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&dev->DIN,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	delay_us(5);
	dev->state = Control_writen;
	CS_PULL_UP;
	/*	Read init value of pak.ADD and send shadow seq	*/
	if (regular_scan_mode == dev->config.operation_mode)
	{
		dev->DIN = AD7490_CHANNELS_ENA_MSK;
		CS_PULL_DOWN;
		HAL_SPI_TransmitReceive(dev->bus_handle,(uint8_t*)&dev->DIN,(uint8_t*)&dev->DOUT,1,1000);
		CS_PULL_UP;
		dev->state = Shadow_writen;
		dev->nbr_of_chn = 0;
		for (uint8_t i = 0; i < 16; i ++)
		{
			if (READ_BITS(AD7490_CHANNELS_ENA_MSK,i))
			{
				dev->nbr_of_chn ++;
			}
		}
	}
}



uint16_t dev_ad7490_generate_control_seq(dev_ad7490_t* dev)
{
	/*	reset DIN data frame	*/
	//dev->DIN = 0;
	uint16_t DIN=0;
	/*	write bits of DIN from control packet*/
	WRITE_BITS(DIN, 0, dev->pak.CODING);
	WRITE_BITS(DIN, 1, dev->pak.RANGE);
	WRITE_BITS(DIN, 2, dev->pak.WEAK_TRI);
	WRITE_BITS(DIN, 3, dev->pak.SHADOW);
	DIN |= dev->pak.PM << 4;
	DIN |= dev->pak.ADD << 6;
	WRITE_BITS(DIN, 10, dev->pak.SEQ);
	WRITE_BITS(DIN, 11, dev->pak.WRITE);
	DIN = (DIN<<4)&0xfff0;
	return DIN;
}

static void dev_ad7490_generate_shadow_seq(dev_ad7490_t* dev,uint16_t chns)
{
	if (dev->config.operation_mode == regular_scan_mode)
	{
		dev->DIN = chns;
	}
}

static void dev_ad7490_config(dev_ad7490_t* dev)
{
	/* configure coding type	*/
	if (dev->config.code_type == b_polar)
	{
		dev->pak.CODING = 0;
	}
	else if (dev->config.code_type == s_polar)
	{
		dev->pak.CODING = 1;
	}
	/* configure conversion range	*/
	if (dev->config.range_type == double_range)
	{
		dev->pak.RANGE = 0;
	}
	else if (dev->config.range_type == single_range)
	{
		dev->pak.RANGE = 1;
	}
	/*	configure conversion type	*/
	if (dev->config.operation_mode == independent_mode)
	{
		dev->pak.SEQ = 0;
		dev->pak.SHADOW = 0;
	}
	else if (dev->config.operation_mode == regular_scan_mode)
	{
		dev->pak.SEQ = 0;
		dev->pak.SHADOW = 1;
		dev->pak.ADD = 0;
	}
	else if (dev->config.operation_mode == continuous_mode)
	{
		dev->pak.SEQ = 1;
		dev->pak.SHADOW = 1;
	}
	/*	configure power mode	*/
	if (dev->config.power_mode == normal_mode)
	{
		dev->pak.PM = 3;
	}
	else if (dev->config.power_mode == full_shutdown)
	{
		dev->pak.PM = 2;
	}
	else if (dev->config.power_mode == auto_shutdown)
	{
		dev->pak.PM = 1;
	}
	else if (dev->config.power_mode == auto_standby)
	{
		dev->pak.PM = 0;
	}
	
	dev->pak.WEAK_TRI = 1;
}

#ifdef USE_FULL_RTT
static rt_err_t dev_ad7490_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
	
}
#else
void dev_ad7490_control(dev_ad7490_t* dev, int cmd, void *args)
{
	switch(cmd)
	{
		case AD_7490_SET_RANGE:
		{
			dev_ad7490_set_range(dev,*(Range_t*)args);
			dev->state = Control_writen;
			break;
		}
		case AD_7490_SET_OPT_MODE:
		{
			dev_ad7490_set_option_mode(dev,*(OP_mode_t*)args);
			dev->state = Control_writen;
			break;
		}
		case AD_7490_SET_PWR_MODE:
		{
			dev_ad7490_set_power_mode(dev,*(Pwr_mode_t*)args);
			dev->state = Control_writen;
			break;
		}
		
		case AD_7490_CFG_DEVICE:
		{
			goto __SPI_Write;
			dev->state = Control_writen;
			break;
		}
		
		case AD_7490_SELF_TEST:
		{
			
			break;
		}
			
	}
	__SPI_Write:
	{
		CS_PULL_UP;
		delay_us(500);
		CS_PULL_DOWN;
		#ifdef BSP_USE_SPI_DMA
			HAL_SPI_Transmit_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,1);
		#else
			HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&dev->DIN,1,1000);
			while(READ_BITS(dev->bus_handle->Instance->SR,7));
		#endif
		CS_PULL_UP;
	}
}
#endif


static void dev_ad7490_set_range(dev_ad7490_t* dev,Range_t range)
{
	CS_PULL_UP;
	if (range == double_range)
	{
		dev->config.range_type = double_range;
		dev->pak.RANGE = 0;
	}
	else if (range == single_range)
	{
		dev->config.range_type = single_range;
		dev->pak.RANGE = 1;
	}
	else
	{
		dev->DIN = 0;
		return;
	}
	dev->pak.WRITE = 1;
	dev->DIN = dev_ad7490_generate_control_seq(dev);
	delay_us(20);
	CS_PULL_DOWN;
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Transmit_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,1);
	#else
		HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&dev->DIN,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	delay_us(20);
	CS_PULL_UP;
}

static void dev_ad7490_set_option_mode(dev_ad7490_t* dev,OP_mode_t opmode)
{
	CS_PULL_UP;
	if (opmode == independent_mode)
	{
		dev->config.operation_mode = independent_mode;
		dev->pak.SEQ = 0;
		dev->pak.SHADOW = 0;
	}
	else if (opmode == regular_scan_mode)
	{
		dev->config.operation_mode = regular_scan_mode;
		dev->pak.SEQ = 0;
		dev->pak.SHADOW = 1;
	}
	else if (opmode == continuous_mode)
	{
		dev->config.operation_mode = continuous_mode;
		dev->pak.SEQ = 1;
		dev->pak.SHADOW = 1;
	}
	else
	{
		dev->DIN = 0;
		return;
	}
	dev->pak.WRITE = 1;
	dev->DIN = dev_ad7490_generate_control_seq(dev);
	delay_us(20);
	CS_PULL_DOWN;
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Transmit_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,1);
	#else
		HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&dev->DIN,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	delay_us(20);
	CS_PULL_UP;
	/*	Read init value of pak.ADD and send shadow seq	*/
	if (regular_scan_mode == dev->config.operation_mode)
	{
		dev->DIN = dev->DOUT;
		for (uint8_t i = 0; i < 16; i ++)
		{
			if (READ_BITS(dev->DOUT,i))
			{
				dev->nbr_of_chn ++;
			}
		}
		CS_PULL_DOWN;
		HAL_SPI_TransmitReceive(dev->bus_handle,(uint8_t*)&dev->DIN,(uint8_t*)&dev->DOUT,1,1000);
		CS_PULL_UP;
		dev->state = Shadow_writen;
		dev->nbr_of_chn = 0;
	}
}

static void dev_ad7490_set_power_mode(dev_ad7490_t* dev,Pwr_mode_t pwrmode)
{
	CS_PULL_UP;
	if (pwrmode == normal_mode)
	{
		dev->config.power_mode = normal_mode;
		dev->pak.PM = 3;
	}
	else if (pwrmode == full_shutdown)
	{
		dev->config.power_mode = full_shutdown;
		dev->pak.PM = 2;
	}
	else if (pwrmode == auto_shutdown)
	{
		dev->config.power_mode = auto_shutdown;
		dev->pak.PM = 1;
	}
	else if (pwrmode == auto_standby)
	{
		dev->config.power_mode = auto_standby;
		dev->pak.PM = 0;
	}
	else
	{
		dev->DIN = 0;
		return;
	}
	dev->pak.WRITE = 1;
	dev->DIN = dev_ad7490_generate_control_seq(dev);
	delay_us(20);
	CS_PULL_DOWN;
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Transmit_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,1);
	#else
		HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&dev->DIN,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	delay_us(20);
	CS_PULL_UP;
}


/*
	This function is used to fetch data from device ad7490, before call this function, user needs 
	to genetate DIN. If enable SPI DMA mode, it doesn't need to call this function to
	fetch data each time.

*/
void dev_ad7490_fetch_data(dev_ad7490_t* dev)
{
	if (dev->config.power_mode == normal_mode)
	{
		switch (dev->config.operation_mode)
		{
			case independent_mode:
			{
				dev_ad7490_fetch_data_in_independent_mode(dev);
				break;
			}
			case regular_scan_mode:
			{
				dev_ad7490_fetch_data_in_regular_scan_mode(dev);
				break;
			}
			case continuous_mode:
			{
				dev_ad7490_fetch_data_in_continuous_mode(dev);
				break;
			}
		}
	}
	else if ((dev->config.power_mode == auto_shutdown)||(dev->config.power_mode == auto_standby))
	{
		/*	power up device	*/
		dev_ad7490_power_up(dev);
		dev_ad7490_generate_control_seq(dev);
		switch (dev->config.operation_mode)
		{
			case independent_mode:
			{
				dev_ad7490_fetch_data_in_independent_mode(dev);
				break;
			}
			case regular_scan_mode:
			{
				dev_ad7490_fetch_data_in_regular_scan_mode(dev);
				break;
			}
			case continuous_mode:
			{
				dev_ad7490_fetch_data_in_continuous_mode(dev);
				break;
			}
		}
	}
}

static void dev_ad7490_fetch_data_in_independent_mode(dev_ad7490_t* dev)
{
	CS_PULL_UP;
	delay_us(20);
	CS_PULL_DOWN;
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Transmit_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,1);
	#else
		HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&dev->DIN,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	delay_us(5);
	CS_PULL_UP;
	delay_us(20);
	CS_PULL_DOWN;
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Receive_DMA(dev->bus_handle,(uint8_t*)&dev->DOUT,1);
	#else
		HAL_SPI_Receive(dev->bus_handle,(uint8_t*)&dev->DOUT,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	delay_us(5);
	CS_PULL_UP;
	dev_ad7490_decode_data(dev);
}

static void dev_ad7490_fetch_data_in_regular_scan_mode(dev_ad7490_t* dev)
{
	static uint8_t nbr_of_chns = 0;
	switch (dev->state)
	{
		case Shadow_writen:
		{
			/*	receive data from device	*/
			/*	In this part, there is no need to write anything to device	*/
			if (READ_BITS(dev->DIN,15))			//clear write bit
			{
				WRITE_BITS(dev->DIN,15,0);
			}
			CS_PULL_UP;
			for (uint8_t i = 0; i < dev->nbr_of_chn; i ++)
			{
				CS_PULL_DOWN;
				#ifdef BSP_USE_SPI_DMA
					HAL_SPI_TransmitReceive_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,(uint8_t*)&dev->DOUT,1);
				#else
					HAL_SPI_TransmitReceive(dev->bus_handle,(uint8_t*)&dev->DIN,(uint8_t*)&dev->DOUT,1,1000);
					while(READ_BITS(dev->bus_handle->Instance->SR,7));
				#endif
				CS_PULL_UP;
				dev_ad7490_decode_data(dev);
				delay_us(200);
			}
			dev->state = Shadow_writen;
			break;
		}
	}
}

static void dev_ad7490_fetch_data_in_continuous_mode(dev_ad7490_t* dev)
{
	CS_PULL_UP;
	/*	In this part, there is no need to write anything to device	*/
	if (READ_BITS(dev->DIN,15))			//clear write bit
	{
		WRITE_BITS(dev->DIN,15,0);
	}
	uint16_t contin = 0;
	for (uint8_t i = 0; i < (dev->pak.ADD + 1); i ++)
	{
		CS_PULL_DOWN;
		delay_us(5);
		#ifdef BSP_USE_SPI_DMA
			HAL_SPI_TransmitReceive_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,(uint8_t*)&dev->DOUT,1);
		#else
			HAL_SPI_TransmitReceive(dev->bus_handle,(uint8_t*)&dev->DIN,(uint8_t*)&dev->DOUT,1,1000);
			//HAL_SPI_Receive(dev->bus_handle,(uint8_t*)&dev->DOUT,1,1000);
			while(READ_BITS(dev->bus_handle->Instance->SR,7));
		#endif
		delay_us(5);
		CS_PULL_UP;
		dev_ad7490_decode_data(dev);
		delay_us(100);
	}
}

/*
Note : This function only used to power up device that in auto standby or auto poweroff
mode.
*/
static void dev_ad7490_power_up(dev_ad7490_t* dev)
{
	CS_PULL_UP;
	delay_us(200);
	if (READ_BITS(dev->DIN,11))
	{
		WRITE_BITS(dev->DIN,11,0);
	}
	CS_PULL_DOWN;
	#ifdef BSP_USE_SPI_DMA
		HAL_SPI_Transmit_DMA(dev->bus_handle,(uint8_t*)&dev->DIN,1);
	#else
		HAL_SPI_Transmit(dev->bus_handle,(uint8_t*)&dev->DIN,1,1000);
		while(READ_BITS(dev->bus_handle->Instance->SR,7));
	#endif
	CS_PULL_UP;
}

void dev_ad7490_decode_data(dev_ad7490_t* dev)
{
	uint8_t channel = dev->DOUT>>12;
	uint16_t channel_data = dev->DOUT;
	if ((channel < 16)&&(channel >= 0))
	{
		if (channel == 0)
		{
			;
		}
		channel_data &= 0x0fff;
		dev->buf[channel] = channel_data;
		//rt_kprintf("Chn=%d, val=%d\n",channel,channel_data);
	}
}

void dev_ad7490_force_taken(dev_ad7490_t* dev)
{
	CS_PULL_DOWN;
}

void dev_ad7490_force_release(dev_ad7490_t* dev)
{
	CS_PULL_UP;
}