#ifndef DEV_AD7490_H_
#define DEV_AD7490_H_
#include <stdint.h>
#include "proj_config.h"
#include "perf_counter.h"
#include "dev_common.h"
#include "rtconfig.h"

/*
	SPI configure:
	SPI_DATASIZE_16BIT;
	SPI_POLARITY_HIGH;
	SPI_PHASE_1EDGE;
	SPI_NSS_SOFT;
	SPI_FIRSTBIT_MSB;
*/
// <<< Use Configuration Wizard in Context Menu >>>
/*	Configuration Wizard	*/

// <h>ADC Configuration
// <i>ADC Configuration

// <o>ADC power mode
// <i>choose adc power mode, default = normal_mode
// <0x10=>normal mode <0x11=>full shutdown <0x12=>auto shutdown <0x13=>auto standby
#define AD7490_PWR_MODE	0x10

// <o>ADC Option mode
// <i>choose adc sampling mode, default = independent_mode
// <0X0a=>independent mode <0x0b=>regular scan_mode <0x0c=>continuous mode
#define AD7490_OP_MODE	0x0C

// <o>ADC output coding type
// <i>choose adc output coding type, default = single polar
// <0=> single polar <1=>bipolar
#define AD7490_CODE_TYPE	0x00

// <o>ADC sample range
// <i>choose adc sample range, default = double range, single range is from 0V to Vref
// <0=>double range <1=>single range
#define AD7490_OP_RANGE	0x00

// <o>Input final channal number
// <i>0-15
#define AD7490_LAST_CHN	15
// <e1.0>ADC channel choices
// <i>If Operation mode is continuous mode, this selection is unavailable
	#define CHANNEL_CONF 0
// <o0.15>Ch 0
// <o0.14>Ch 1
// <o0.13>Ch 2
// <o0.12>Ch 3
// <o0.11>Ch 4
// <o0.10>Ch 5
// <o0.9>Ch 6
// <o0.8>Ch 7
// <o0.7>Ch 8
// <o0.6>Ch 9
// <o0.5>Ch 10
// <o0.4>Ch 11
// <o0.3>Ch 12
// <o0.2>Ch 13
// <o0.1>Ch 14
// <o0.0>Ch 15
#define AD7490_CHANNELS_ENA_MSK 0x8000
// </e>
// </h>

// <<< end of configuration section >>>

/*	Control CMDs	*/
#define AD_7490_SET_RANGE					(int)(0)			//set adc range
#define AD_7490_SET_OPT_MODE			(int)(1)			//set operation mode
#define AD_7490_SET_PWR_MODE			(int)(2)			//set power mode
#define AD_7490_SELF_TEST					(int)(3)			//self test
#define AD_7490_SET_CODE_TYPE			(int)(4)			//set output code type
#define	AD_7490_CFG_DEVICE				(int)(5)			//manual configure device

/*
	Codine type
	If it is set to 0, the output coding is twos complement. This is used for bipolar signal.
	AKA, signed varible. (Needs extra hardwares.)
	If it is set to 1, the output is straight binary code. AKA, unsigned varible.
*/
typedef enum Coding:uint8_t
{
	s_polar = 0,
	b_polar = 1
}Code_t;

/*
	Range type
	If it is set to 0, the analog input range extends from 0 V to 2 × REF IN ;
	If it is set to 1,  the analog input range extends from 0 V to REF IN ;
	This setting will take effect in the next conversion.
*/
typedef enum Range:uint8_t
{
	double_range = 0,
	single_range = 1
}Range_t;

/*
	Conversion type
	Inpendent mode	:	convert single channel selected by ADD0-ADD3, the result will
						return to master device in next conversion.
	Regular mode		: convert selected channels in each cycle. Channels need not to be consecutive.
	Scan node				: based on regualr mode, but this mode can change mode, coding and so on without interrupting conversion.
										(cannot change shadow register)
	Continuous mode : convert channels from chn 0 to selected channel in each cycle.
*/

typedef enum Op_mode:uint8_t
{
	independent_mode = 0x0a,
	regular_scan_mode = 0x0b,
	continuous_mode = 0x0c
}OP_mode_t;

/*
	Power mode
	
*/

typedef enum Pwr_mode:uint8_t
{
	normal_mode = 0x10,
	full_shutdown = 0x11,
	auto_shutdown = 0x12,
	auto_standby = 0x13
}Pwr_mode_t;


typedef enum __data_fetch_sta:uint8_t
{
	Initial,
	Taken,
	Control_writen,
	Shadow_writen,
	DOUT_Read,
	DIN_and_DOUT
}data_fetch_sta;

struct dev_ad7490_config
{
	Code_t code_type;
	Range_t range_type;
	OP_mode_t operation_mode;
	Pwr_mode_t power_mode;
};

struct dev_ad7490_control_packet
{
	uint8_t WRITE			:1;
	uint8_t SEQ				:1;
	uint8_t ADD				:4;
	uint8_t PM				:2;
	uint8_t SHADOW		:1;
	uint8_t WEAK_TRI	:1;
	uint8_t	RANGE			:1;
	uint8_t	CODING		:1;
};



typedef struct __dev_ad7490
{
	/*	device name	*/
	char name[16];
	#ifdef USE_FULL_RTT
	rt_device_t device;
	#endif
	/*	device configuration struct	*/
	struct dev_ad7490_config config;
	
	/*	device control packet	*/
	struct dev_ad7490_control_packet pak;
	
	/*	SPI bus handle	*/
	SPI_HandleTypeDef* bus_handle;
	
	/*	Chip select IO port	*/
	GPIO_TypeDef* cs_gpiox;
	
	/*	Chip select pin	*/
	uint16_t 		 cs_gpio_pin;
	
	/*	SPI data TX	*/
	uint16_t 		 DIN;
	
	/*	SPI data RX	*/
	uint16_t     DOUT;
	
	/*	device data fetch state	*/
	data_fetch_sta	state;
	
	/*	device output data buffer array	*/
	uint16_t* buf;
	
	/*	In regular scan mode, number of channels	*/
	uint8_t	nbr_of_chn;
}dev_ad7490_t;


/*	API for user	*/

void dev_ad7490_init(dev_ad7490_t* dev);
uint16_t dev_ad7490_generate_control_seq(dev_ad7490_t* dev);
void dev_ad7490_control(dev_ad7490_t* dev, int cmd, void *args);
void dev_ad7490_fetch_data(dev_ad7490_t* dev);

void dev_ad7490_force_taken(dev_ad7490_t* dev);
void dev_ad7490_force_release(dev_ad7490_t* dev);
void dev_ad7490_decode_data(dev_ad7490_t* dev);
#endif