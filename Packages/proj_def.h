
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
#ifndef __PROJ_DEF_H
#define __PROJ_DEF_H
#include <rtdef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* This data structure is used in sending and receiving mails in ipc */
typedef struct __mail_msg mail_msg;
typedef struct __mail_msg* mail_msg_t;
struct __mail_msg
{
	void* data_ptr;
	rt_uint32_t size;
};
enum __mail_head
{
	CTRL_NORMAL = 0x01,
	CTRL_ERROR,
	CTRL_TIMEOUT,
	CTRL_EMERGENCY
};


#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/