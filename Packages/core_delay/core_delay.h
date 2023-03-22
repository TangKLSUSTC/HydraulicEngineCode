/*
 * Copyright (c)	BC Lab 2021
 *
 * Creator	: Thompson
 *
 * brief		:	Use kernal register to generate accurate delay
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-27     Thompson     Create this file
 *
 */

/*
Usage: This delay depends on kernal clock, there is a peripheral named CYCCNT in DWT.
			 It's a 32-bit count-up counter.

*/
#ifndef __CORE_DELAY_H
#define __CORE_DELAY_H
/*	Inlcudes	*/
#include <stdbool.h>
#if defined(STM32F401xE)
//	#include <stm32f4xx.h>
//	#include <core_cm4.h>
	#include <stm32f4xx_hal.h>
#endif
#if defined(STM32F103xB)
	#include "stm32f1xx_hal.h"
#endif

/*	Global Varibles	*/
extern uint32_t SystemCoreClock;
/*	Defines	*/
#define GET_CPU_Freq()				HAL_RCC_GetSysClockFreq()
#define SYS_CLK								SystemCoreClock
#if defined(STM32F1)
	#define ATOM_OPERATION_START	__disable_irq()
	#define ATOM_OPERATION_END		__enable_irq()
#endif
#if defined(STM32F4)
	#define ATOM_OPERATION_START	__disable_irq()
	#define ATOM_OPERATION_END		__enable_irq()
#endif
// <<< Use Configuration Wizard in Context Menu >>>
// <h>User Configuration
		//<c1> Use DWT core debug delay auto initialize
		//<i> Use DWT core debug delay auto initialize
		//#define USE_DWT_DELAY_AUTOINIT
		//</c>
		// <c1>Initial DWT in each delay
		// <i>Initial DWT in each delay
		#define CPU_TS_INIT_IN_DELAY_FUNC
		//</c>
//</h>
// <<< end of configuration section >>>
/*****************************************************************
												Function Prototypes
******************************************************************/
//#if defined(USE_DWT_DELAY_AUTOINIT)
uint32_t CPU_TS_TmrRd(void);
bool core_delay_init(void);
void core_delay_us(uint32_t us);
void core_delay_ms(uint32_t ms);

uint32_t CPU_TS_TmrRd(void);

bool start_core_count(void);
uint32_t stop_core_count(void);

#define core_delay_ms(ms) core_delay_us(1000*ms)

//#endif




#endif