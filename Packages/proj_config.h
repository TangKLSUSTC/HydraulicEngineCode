
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     proj_config.h
* @Author       Thompson
* @Version         
* @Date         2021.12.08
* @Note            
* @Copyright    Biorobotics & Control Lab
************************************************************************/

/*************************************************************************
* @Function

* @ChangeLog
    Date               Author               Notes
    2021.12.08         Thompson             Create this file, first version
************************************************************************/
#ifndef __PROJ_CONFIG_H
#define __PROJ_CONFIG_H


#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <rtconfig.h>
#include <proj_def.h>
// <<< Use Configuration Wizard in Context Menu >>>

// <h>Basic Configuration

// <s>Firmware name
#define FIRMWARE_NAME	"HPT sys"

// <s> Firmware version infomation
#define VERSION "V3.2.1"


// <c1>Use RT-Thread Operating System
// <i>Default: USE
#define USE_RTT
// </c>




// <e>Use system service threads
// <i>Use system service threads, these threads have lower priority and less timeslides
	#define USE_SERVICES 1
	// <c1>Use Led signal
	// <i>led signals is blinking led to express some info
	#if (USE_SERVICES)&&defined(USE_RTT)
	#define USE_LED_SIGNALS
	#endif
	// </c>
	// <c1>Use Key listener
	// <i>Listening key pressed and released event
	//#if (USE_SERVICES)&&defined(USE_RTT)
	//#define USE_KEY_LISTENER
	//#endif
	// </c>
	// <c1>Use External ADC 7490
	// <i>Fetching adc data
	#if (USE_SERVICES)&&defined(USE_RTT)
	#define USE_AD7490_FETCHER
	#endif
	// </c>
	// <c1>Use Extend IO tca9539
	// <i>IO output or input
	#if (USE_SERVICES)&&defined(USE_RTT)
	#define USE_EXTEND_IO
	#endif
	// </c>
// </e>
// </h>

// <h>OS unrelated components
	// <c1>Use DMA io port through USART
	// <i>Use DMA io port through USART; Default: USE
	#define USE_DMA_IO
	#if defined(USE_DMA_IO)
		#include <usart_dma_io.h>
		#define SHELL_PORT		usart1
	#endif
	// </c>
	
	// <c1>Use high resulotion hardware timer (systick timer)
	// <i>Use high resulotion hardware timer (systick timer)
	#define PERF_COUNTER
	#if defined(PERF_COUNTER)
		#include <perf_counter.h>
	#endif
	// </c>
	
	// <c1>Use high resulotion hardware timer (Core DWT timer)
	// <i>Use high resulotion hardware timer (Core DWT timer)
	#define DWT_COUNTER
	#if defined(DWT_COUNTER)
		#include <core_delay.h>
	#endif
	// </c>

	
// </h>
// <<< end of configuration section >>>

#ifndef USE_RTT
	#define USE_RTT
#endif



#if defined(USE_SERVICES)
	#if defined(USE_LED_SIGNALS)
		#include "serv_led_sign.h"
		//#define LED_SIGNAL_USE_IDLE
	#endif
	
	#if defined(USE_AD7490_FETCHER)
		#include "serv_ad7490_fetcher.h"
		#define FS_1000HZ 1000
		#define FS_500HZ  500
		#define FS_100HZ  100
		#define FS_50HZ   50
		#define FS_10HZ   10
		/* This defination can be defined as FS_1000HZ, FS_500HZ, FS_100HZ, FS_50HZ, FS_10HZ*/
		#define AD7490_SAMPLE_RATE	FS_1000HZ
	#endif
	
	#if defined(USE_KEY_LISTENER)
		#include "serv_key_listener.h"
	#endif
	
	#if defined(USE_EXTEND_IO)
		#include "serv_tca_io.h"
		#define IO_CHIP_NBR 2
		#define IO_CMD(x)        (1<<x)
		#define USART_CMD        IO_CMD(0)
		#define THREAD_CMD       IO_CMD(1)
		#define EMERGENCY_STOP   IO_CMD(2)
	#endif
#endif


/*	Project includes	*/
#if defined(STM32F401xE)
	#include "stm32f4xx_hal.h"
#endif
#if defined(STM32F103xB)
	#include "stm32f1xx_hal.h"
#endif
#if defined(USE_RTT)
	#include <rtthread.h>
#endif
/*	Common includes	*/
#include <main.h>
#include <dev_common.h>
#include <dev_ad7490.h>
#include <dev_tca9539.h>
#define BSP_USING_UART1
#define BSP_USING_UART2
//#define BSP_USING_UART3
#define BSP_USING_UART6

/* App thread configure */
#define SENSORS_LEGANCY_MODE
#define SENSORS_FLEXIBLE_MODE


#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/