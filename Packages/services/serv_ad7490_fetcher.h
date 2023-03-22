
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_ad7490_fetcher.h
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
#ifndef __SERV_AD7490_FETCHER_H
#define __SERV_AD7490_FETCHER_H


#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <rtthread.h>
#include <proj_config.h>
#include "ringbuffer.h"
/*	Please add your includes here	*/



/*	Please add your macro here	*/
#define CS_PORT			GPIOB
#define CS_PIN			GPIO_PIN_1





/*	Please add your varibles extern here	*/
extern rt_mailbox_t serv_ad_mailbox;
extern rt_mailbox_t serv_index_mailbox;

/*	Please add your structure defination here	*/


void serv_ad7490_fetcher_init();
int serv_ad7490_hw_init(SPI_HandleTypeDef* spihandle);





#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/