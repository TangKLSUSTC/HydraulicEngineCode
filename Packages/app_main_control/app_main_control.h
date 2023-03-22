
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     app_main_control.h
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
#ifndef __APP_MAIN_CONTROL_H
#define __APP_MAIN_CONTROL_H


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
#define PUMP_PIN						GPIO_PIN_12
#define INLET_VALVE_PIN			GPIO_PIN_13
#define OUTLET_VALVE_PIN		GPIO_PIN_14



#define OPEN_PUMP						HAL_GPIO_WritePin(GPIOB,PUMP_PIN,GPIO_PIN_SET)
#define CLOSE_PUMP					HAL_GPIO_WritePin(GPIOB,PUMP_PIN,GPIO_PIN_RESET)

#define OPEN_INLET_VALVE		HAL_GPIO_WritePin(GPIOB,INLET_VALVE_PIN,GPIO_PIN_SET)
#define CLOSE_INLET_VALVE		HAL_GPIO_WritePin(GPIOB,INLET_VALVE_PIN,GPIO_PIN_RESET)
#define OPEN_OUTLET_VALVE		HAL_GPIO_WritePin(GPIOB,OUTLET_VALVE_PIN,GPIO_PIN_SET)
#define CLOSE_OUTLET_VALVE	HAL_GPIO_WritePin(GPIOB,OUTLET_VALVE_PIN,GPIO_PIN_RESET)


#define ASSIGN_SENSOR(sensor_index,channel)    sensor_assignment[channel] = &adc_os_data[sensor_index]

/*	Please add your varibles extern here	*/




typedef enum __ctrl_sta
{
	CTRL_INIT = 0x01,
	CTRL_UNSTABLE,
	CTRL_TIMER_READY,
	CTRL_TIMER_ARMED,
	CTRL_TIMER_DONE,
	UNCALI,
	CALI_DONE
}ctrl_sta_t;


void app_main_control_init();









#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/