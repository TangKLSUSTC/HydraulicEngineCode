/*
 * Copyright (c)	BC Lab 2021
 *
 * Creator	: Thompson
 *
 * Change Logs:
 * Date           Author       		Notes
 * 2021-02-25     Shijian Wu     Create this driver
 
 * 2021-03-18			Shijian Wu		Add a configuration parameter for function dev_tca9539_init()
																Add marco definitions of pins
 *
 */
#include "dev_tca9539.h"

#define RESET_PULL_UP 		HAL_GPIO_WritePin(dev->reset_gpiox, dev->reset_gpio_pin, GPIO_PIN_SET)
#define RESET_PULL_DOWN		HAL_GPIO_WritePin(dev->reset_gpiox, dev->reset_gpio_pin, GPIO_PIN_RESET)


static uint8_t dev_tca9539_write_reg(dev_tca9539_t* dev, dev_tca9539_reg name);
static uint8_t dev_tca9539_write_reg_bit(dev_tca9539_t* dev, dev_tca9539_reg name, uint8_t Number, uint8_t x);

void dev_tca9539_init(dev_tca9539_t* dev, uint16_t pinState)
{
	/* Initialize reset pin	*/
	/*
	RESET_PULL_UP;
	
	GPIO_InitTypeDef GPIO_Initure;
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Initure.Pin = dev->reset_gpio_pin;
	GPIO_Initure.Pull = GPIO_PULLUP;
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(dev->reset_gpiox, &GPIO_Initure);
	#ifdef DEV_USE_EXIT
	GPIO_Initure.Pin = dev->int_gpio_pin;
	
	
	#endif
	
	RESET_PULL_DOWN;
	delay_us(50);
	RESET_PULL_UP;
	*/
	dev->cfg = pinState;
	dev->GPIO_OUT = 0x0000;
	dev_tca9539_write_reg(dev,config);
	delay_us(5);
	dev_tca9539_write_reg(dev,output);
	delay_us(5);
}

void dev_tca9539_control(dev_tca9539_t* dev, int cmd, void* argv)
{
	
}

/*with a struct dev, write all of the 16 bits of one pair of registers among output registers, 
polarity inversion registers, and configuration registers */
static uint8_t dev_tca9539_write_reg(dev_tca9539_t* dev, dev_tca9539_reg name)
{
	void* tmp = 0;
	switch (name)
	{
		case output:
		{
			tmp = &(dev->GPIO_OUT);
			break;
		}
		case polarity:
		{
			tmp = &(dev->polar);
			break;
		}
		case config:
		{
			tmp = &(dev->cfg);
			break;
		}
		default:
		{
			return 0;
		}
	}
	if (HAL_OK == HAL_I2C_Mem_Write(dev->bus_handle, ((uint16_t)(dev->bus_addr))<<1, (uint16_t)name, 1, (uint8_t*)tmp, 2, 10))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*with a struct dev, modify certain single bit of one pair of registers among output registers, 
polarity inversion registers, and configuration registers */
static uint8_t dev_tca9539_write_reg_bit(dev_tca9539_t* dev, dev_tca9539_reg name, uint8_t Number, uint8_t x)
{
	void* tmp = 0;
	switch (name)
	{
		case output:
		{
			tmp = &(dev->GPIO_OUT);
			break;
		}
		case polarity:
		{
			tmp = &(dev->polar);
			break;
		}
		case config:
		{
			tmp = &(dev->cfg);
			break;
		}
		default:
		{
			return 0;
		}
		WRITE_BITS(*(uint16_t*)tmp, Number, x);
	}
	
	if (HAL_OK == HAL_I2C_Mem_Write(dev->bus_handle, ((uint16_t)(dev->bus_addr))<<1, (uint16_t)name, 1, (uint8_t*)tmp, 2, 500))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/* device tca9539 read register */
static uint8_t dev_tca9539_read_reg(dev_tca9539_t* dev, dev_tca9539_reg name, uint8_t* reg_buf)
{
	void* tmp = 0;
	uint16_t device_addr_tmp = 0;
	switch (name)
	{
		case output:
		{
			tmp = &(dev->GPIO_OUT);
			break;
		}
		case polarity:
		{
			tmp = &(dev->polar);
			break;
		}
		case config:
		{
			tmp = &(dev->cfg);
			break;
		}
		default:
		{
			return 0;
		}
	}
	device_addr_tmp = (dev->bus_addr)<<1;
	WRITE_BITS(device_addr_tmp, 0, 1);
	if (1 == HAL_I2C_Master_Transmit(dev->bus_handle,((uint16_t)(dev->bus_addr))<<1,(uint8_t*)&name,1,100))
	{
		return HAL_I2C_Master_Receive(dev->bus_handle,device_addr_tmp,reg_buf,2,100);
	}
	else return 0;
}

 /*
	Write single io port of TCA9539
*/
uint8_t dev_tca9539_write_sio(dev_tca9539_t* dev, uint8_t channel, uint16_t value)
{
		return dev_tca9539_write_reg_bit(dev,output,channel,value);
}


/*
	Write all channel bits at same time
*/
uint8_t dev_tca9539_write_aio(dev_tca9539_t* dev, uint16_t value)
{
		dev->GPIO_OUT = value;
		return dev_tca9539_write_reg(dev,output);
}

/*
	set IO direction
*/
uint8_t dev_tca9539_set_sio_dir(dev_tca9539_t* dev,uint8_t channel,uint16_t value)
{
	return dev_tca9539_write_reg_bit(dev, config, channel, value);
}

uint8_t dev_tca9539_set_aio_dir(dev_tca9539_t* dev, uint16_t value)
{
	dev->cfg = value;
	return dev_tca9539_write_reg(dev,config);
}


/*
	set IO Polarity
*/
uint8_t dev_tca9539_set_sio_pol(dev_tca9539_t* dev,uint8_t channel,uint16_t value)
{
	return dev_tca9539_write_reg_bit(dev, polarity, channel, value);
}

uint8_t dev_tca9539_set_aio_pol(dev_tca9539_t* dev, uint16_t value)
{
	dev->cfg = value;
	return dev_tca9539_write_reg(dev,polarity);
}

uint8_t tca9539_probe(dev_tca9539_t* dev)
{
	if (HAL_OK == HAL_I2C_Mem_Write(dev->bus_handle, ((uint16_t)(dev->bus_addr))<<1,(uint16_t)config,1,(uint8_t*)&dev->cfg,2,500))
	{
		rt_kprintf("Probe an exisisted tca9539!\n");
		rt_kprintf("Bus address is 0x%X\n",(uint8_t)dev->bus_addr);
		return 1;
	}
	else
	{
		rt_kprintf("Can not probe an exisisted tca9539 io chip at address 0x%X!\n",(uint8_t)dev->bus_addr);
		return 0;
	}
}
