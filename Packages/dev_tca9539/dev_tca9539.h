#ifndef DEV_TCA9539_H_
#define DEV_TCA9539_H_
#include <stdint.h>
#include "proj_config.h"
#include "perf_counter.h"
#include "dev_common.h"


/************************** I2C Address ***************************************/
#define TCA9539_ADDRESS		0x74<<1 		// I2C Address 0100 00 + ADDR + R/W
											// ADDR tied to P2.2 of LaunchPad
/************************** I2C Registers *************************************/
#define TCA9539_INPUT_REG0 	0x00		// Input status register
#define TCA9539_OUTPUT_REG0	0x02		// Output register to change state of output BIT set to 1, output set HIGH
#define TCA9539_POLARITY_REG0 	0x04		// Polarity inversion register. BIT '1' inverts input polarity of register 0x00
#define TCA9539_CONFIG_REG0		0x06		// Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output


#define TCA9539_SET		

#define ALL_PINS 0xFFFF
#define PIN_0 1
#define PIN_1 1<<1
#define PIN_2 1<<2
#define PIN_3 1<<3
#define PIN_4 1<<4
#define PIN_5 1<<5
#define PIN_6 1<<6
#define PIN_7 1<<7
#define PIN_8 1<<8
#define PIN_9 1<<9
#define PIN_10 1<<10
#define PIN_11 1<<11
#define PIN_12 1<<12
#define PIN_13 1<<13
#define PIN_14 1<<14
#define PIN_15 1<<15

//#define DEV_USE_EXIT

typedef enum __dev_tca9539_addr:uint8_t
{
	ADD_NULL = 0x00,
	LL = 0x74,
	LH = 0x75,
	HL = 0x76,
	HH = 0x77
}dev_tca9539_addr;

typedef enum __dev_tca9539_reg:uint8_t
{
	input = 0x00,
	//input1,
	output = 0x02,
	//output1,
	polarity = 0x04,
	//polarity1,
	config = 0x06
	//config1
}dev_tca9539_reg;


struct dev_tca9539_control_packet
{
	 unsigned char B0:1;
	 unsigned char B1:1;
	 unsigned char B2:1;
	 unsigned char B3:1;
	 unsigned char B4:1;
	 unsigned char B5:1;
	 unsigned char B6:1;
	 unsigned char B7:1;
};

typedef struct __dev_tca9539
{
	/*	device name	*/
	char name[16];
	#ifdef USE_FULL_RTT
	rt_device_t device;
	#endif
	/*	device configuration struct	*/
	struct dev_tca9539_control_packet pak;
	
	/*	I2C bus handle	*/
	I2C_HandleTypeDef* bus_handle;
	
	/*	I2C hardware bus addr	*/
	dev_tca9539_addr bus_addr;
	
	/*	device output data 	*/
	uint16_t GPIO_OUT;
	
	/*	device input data	*/
	uint16_t GPIO_IN;
	
	/*	Reset IO Port(MCU)	*/
	GPIO_TypeDef* reset_gpiox;
	
	/*	Reset pin(MCU)	*/
	uint16_t 		 reset_gpio_pin;
	
	#ifdef DEV_USE_EXIT
	/*	Interrupt IO Port(MCU)	*/
	GPIO_TypeDef* int_gpiox;
	
	/*	Interrupt pin(MCU)	*/
	uint16_t 		 int_gpio_pin;
	#endif
	
	/*	configuration register	*/
	uint16_t cfg;
	
	/*	Inversion register	*/
	uint16_t inv;
	
	/*	control register	*/
	uint16_t ctrl;
	
	/*	polarity register*/
	uint16_t polar;
}dev_tca9539_t;

void dev_tca9539_init(dev_tca9539_t* dev, uint16_t pinState);
uint8_t dev_tca9539_write_sio(dev_tca9539_t* dev, uint8_t channel, uint16_t value);
uint8_t dev_tca9539_write_aio(dev_tca9539_t* dev, uint16_t value);
uint8_t dev_tca9539_read_io(dev_tca9539_t* dev, uint8_t channel);
uint8_t tca9539_probe(dev_tca9539_t* dev);
#endif