
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_config.h
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
#ifndef __SERV_CONFIG_H
#define __SERV_CONFIG_H


#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <proj_config.h>

/*	These includes depend on*/
#if defined(USE_SERVICES)
	#if defined(USE_LED_SIGNALS)
		#include "serv_led_sign.h"
	#endif
	
	#if defined(USE_AD7490_FETCHER)
		#include "serv_ad7490_fetcher.h"
	#endif
	
	#if defined(USE_KEY_LISTENER)
		#include "serv_key_listener.h"
	#endif
	
	#if defined(USE_EXTEND_IO)
		#include "serv_tca_io.h"
	#endif  /*USE_EXTEND_IO*/
#endif

#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/