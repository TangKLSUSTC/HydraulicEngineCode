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
 
#include "core_delay.h"



#define READ_BITS(Instance, Number)			(Instance>>Number)&0b1

#define WRITE_BITS(Instance, Number, x)	\
				do{															\
				if (x==0)												\
				{																\
					Instance&=~(1<<Number);				\
				}																\
				else if (x==1)									\
				{																\
					Instance|=(1<<Number);				\
				}																\
				}while(0)

#define DEM_CR_TRCENA 		WRITE_BITS(CoreDebug->DEMCR,CoreDebug_DEMCR_TRCENA_Pos,1)
#define DWT_CR_CYCCNTENA	WRITE_BITS(DWT->CTRL,DWT_CTRL_CYCCNTENA_Pos,1)
/*****************************************************************
												Local Varibles
******************************************************************/
static uint16_t us_coe = 0;
static uint32_t s_nCycleCounts = 0;
/*
	This function is used to initialize core delay.
	Core delay may affect system debug.
*/
#if defined(USE_DWT_DELAY_AUTOINIT)
__attribute((constructor(255)))
#endif
bool core_delay_init(void)
{
	DEM_CR_TRCENA;
	DWT->CYCCNT = (uint32_t)0u;
	DWT_CR_CYCCNTENA;
	us_coe = SYS_CLK/(1000000ul);
	return true;
}

uint32_t CPU_TS_TmrRd(void)
{
	return ((uint32_t)(DWT->CYCCNT));
}

uint32_t CPU_TS_GetTick(void)
{
	return ((uint32_t)(DWT->CYCCNT*1000)/SYS_CLK);
}

bool start_core_count(void)
{
	DEM_CR_TRCENA;
	DWT_CR_CYCCNTENA;
	us_coe = SYS_CLK/(1000000ul);
	if (READ_BITS(DWT->CTRL,0)&&READ_BITS(CoreDebug->DEMCR,24))
	{
		ATOM_OPERATION_START;
		s_nCycleCounts = (uint32_t)(DWT->CYCCNT);
		ATOM_OPERATION_END;
		return true;
	}
	else
	{
		return false;
	}
}

uint32_t stop_core_count(void)
{
	uint32_t tmp = 0;
	if (READ_BITS(DWT->CTRL,0)&&READ_BITS(CoreDebug->DEMCR,24))
	{
		ATOM_OPERATION_START;
	  tmp = (uint32_t)(DWT->CYCCNT);
		ATOM_OPERATION_END;
		if (tmp > s_nCycleCounts)
		{
			tmp -= s_nCycleCounts;
			goto __exit;
		}
		else
		{
			tmp += (UINT32_MAX - s_nCycleCounts);
			goto __exit;
		}
	}
	else goto __exit;
	__exit:
	return tmp;
}

void core_delay_us(uint32_t us)
{
	uint32_t ticks,told,tnow,tcnt = 0;
	#ifdef CPU_TS_INIT_IN_DELAY_FUNC
		//core_delay_init();
	DEM_CR_TRCENA;
	DWT_CR_CYCCNTENA;
	us_coe = SYS_CLK/(1000000ul);
	#endif
	ticks = us * us_coe;
	tcnt = 0;
	told = (uint32_t)(DWT->CYCCNT);
	while(1)
	{
		tnow = (uint32_t)(DWT->CYCCNT);
		if (tnow != told)
		{
			if (tnow > told)
			{
				tcnt += tnow - told;
			}
			else
			{
				tcnt += UINT32_MAX - told +tnow;
			}
			told = tnow;
			
			if (tcnt >= ticks)break;
		}
	}
}
