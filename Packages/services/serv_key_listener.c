
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_key_listener.c
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
#include "serv_key_listener.h"





/*	Kernal Object Declaration	*/
rt_thread_t serv_key_listener = RT_NULL;
rt_event_t key_event;

/*	Static Varibles Declaration	*/


/*	Extern Varibles Reference	*/
extern dma_io_t serial;

/*	Common Varibles Declaration	*/


/*	Static Function Prototype	*/
void serv_key_listener_main(void* param);


void serv_key_listener_init()
{
	//rt_thread_init(key_listener,"key_listener",app_key_listener_main,RT_NULL,app_key_listener_stack,sizeof(app_key_listener_stack)/4,5,1);
	key_event = rt_event_create("key_event",RT_IPC_FLAG_PRIO);
	serv_key_listener = rt_thread_create("key_listener",serv_key_listener_main,RT_NULL,512,5,5);
	if(RT_NULL != serv_key_listener && RT_NULL != key_event)
	{
		rt_thread_startup(serv_key_listener);
	}
}



uint8_t key_tmp = 0,key_sta = 0 , key_his = 0, pre_cnt = 0;
key_status_t key_state = KEEP;
void serv_key_listener_main(void* param)
{
	while (1)
	{
		key_tmp = HAL_GPIO_ReadPin(TEST_KEY_PORT,TEST_KEY_PIN);
		switch (key_tmp)
		{
			case GPIO_PIN_SET:
			{
				WRITE_BITS(key_sta,0,1);
				break;
			}
			case GPIO_PIN_RESET:
			{
				WRITE_BITS(key_sta,0,0);
				break;
			}
		}
		key_tmp = key_his^key_sta;
		/*	Input state changed	*/
		if (READ_BITS(key_tmp,0))
		{
			/*	Falling edge	*/
			if (READ_BITS(key_his,0))
			{
				key_state = RELEASED;
				WRITE_BITS(key_his,0,0);
			}
			/*	Rising edge	*/
			else
			{
				key_state = PRESSED;
				WRITE_BITS(key_his,0,1);
			}
		}
		else
		{
			key_state = KEEP;
		}
		
		//Key pressed
		switch (key_state)
		{
			case PRESSED:
			{
				rt_event_send(key_event,TESTKEY_PRESSED);
			}
			case RELEASED:
			{
				break;
			}
			case KEEP:
			{
				
				break;
			}
		}
		rt_thread_mdelay(50);

	}

}








/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/