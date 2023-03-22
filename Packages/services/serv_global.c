
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_global.c
* @Author       Thompson
* @Version         
* @Date         2021.12.16
* @Note            
* @Copyright    Biorobotics & Control Lab
************************************************************************/

/*************************************************************************
* @Function

* @ChangeLog
    Date               Author               Notes
    2021.12.16         Thompson             Create this file, first version
************************************************************************/
#include "serv_global.h"





/*	Static Varibles Declaration	*/


/*	Extern Varibles Reference	*/


/*	Common Varibles Declaration	*/


/*	Static Function Prototype	*/

void serv_thread_init()
{
	/*	Init service threads or components	*/
	#if defined(USE_AD7490_FETCHER)
		serv_ad7490_fetcher_init();
	#endif
	#if defined(USE_KEY_LISTENER)
		serv_key_listener_init();
	#endif
	#if defined(USE_LED_SIGNALS)
		serv_led_sign_init();
	#endif
	#if defined(USE_EXTEND_IO)
		serv_tca_io_init();
	#endif
}









/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/