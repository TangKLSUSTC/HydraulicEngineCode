
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_tca_io.h
* @Author       Thompson
* @Version         
* @Date         2022.02.22
* @Note            
* @Copyright    Biorobotics & Control Lab
************************************************************************/

/*************************************************************************
* @Function

* @ChangeLog
    Date               Author               Notes
    2022.02.22         Thompson             Create this file, first version
************************************************************************/
#ifndef __SERV_TCA_IO_H
#define __SERV_TCA_IO_H


#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <rtthread.h>
#include <proj_config.h>
#include <dev_tca9539.h>

/*	Please add your includes here	*/



/*	Please add your macro here	*/






/*	Please add your varibles extern here	*/


void serv_tca_io_init();
void serv_tca_hw_init(I2C_HandleTypeDef* iic_handle);










#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/