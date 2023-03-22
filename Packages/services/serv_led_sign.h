
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_led_sign.h
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
#ifndef __SERV_LED_SIGN_H
#define __SERV_LED_SIGN_H


#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <rtthread.h>
#include <proj_config.h>

/*	Please add your includes here	*/



/*	Please add your macro here	*/
#define LED_PORT			GPIOC
#define LED_PIN				GPIO_PIN_13




/*	Please add your varibles extern here	*/

typedef enum:uint32_t
{
	NORMAL 								= (1<<0),
	LONG_LIGHT 						= (1<<1),
	SHORT_LIGHT_ONESHOT		= (1<<2),
	SHORT_LIGHT_PERIODIC	= (1<<3),
	LONG_OFF							= (1<<4),
	FAST_BLINK						= (1<<5),
	WAITING								= (1<<6)
}serv_led_event_t;





void serv_led_sign_init();
void serv_led_sign_send_event(serv_led_event_t event, uint16_t param);
void serv_led_sign_hw_init();
// <<< Use Configuration Wizard in Context Menu >>>
/*	Configuration Wizard	*/
#if defined(USE_LED_SIGNALS)&&defined(USE_RTT)
// <h>LED events defines
// <i>Bits configuration
// <e.5>Led events
// <o0.0>Normal event
// <o0.1>Long light event
// <o0.2>Short light event (one shot)
// <o0.3>Short light event (periodic)
// <o0.4>Long off
// <o0.5>Fast blink
// <o0.6>
#define LED_EVENT_RX 0x3F
// </e>
// </h>
#endif
// <<< end of configuration section >>>

#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/