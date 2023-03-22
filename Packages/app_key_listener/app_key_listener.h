
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     app_key_listener.h
* @Author       Thompson
* @Version         
* @Date         2021.12.02
* @Note            
* @Copyright    Biorobotics & Control Lab
************************************************************************/

/*************************************************************************
* @Function

* @ChangeLog
    Date               Author               Notes
    2021.12.02         Thompson             Create this file, first version
************************************************************************/
#ifndef __APP_KEY_LISTENER_H
#define __APP_KEY_LISTENER_H


#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <rtthread.h>
#include <dev_common.h>

/*	Please add your includes here	*/
#include "stm32f1xx_hal.h"


/*	Please add your macro here	*/
#define TEST_KEY_PORT GPIOA
#define TEST_KEY_PIN GPIO_PIN_2
#if defined(ROLL_BACK)
#define TEST_KEY_ONCE_PRESSED		(1<<0)
#define TEST_KEY_TWICE_PRESSED		(1<<1)
#define TEST_KEY_UP							(1<<2)
#else
#define TESTKEY_PRESSED					(1<<0)
#endif


/*	Please add your varibles extern here	*/
extern rt_sem_t key_sem;



typedef enum
{
	PRESSED,
	RELEASED,
	KEEP
}key_status_t;




void app_key_listener_init();



#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/