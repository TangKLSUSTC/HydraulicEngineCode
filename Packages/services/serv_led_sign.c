
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_led_sign.c
* @Author       Thompson
* @Version         
* @Date         2021.12.08
* @Note            
* @Copyright    Biorobotics & Control Lab
************************************************************************/

/*************************************************************************
* @Function
		Generate LED output signal. If use thread mode, thread time quantam is 10ms
* @ChangeLog
    Date               Author               Notes
    2021.12.08         Thompson             Create this file, first version
************************************************************************/
#include "serv_led_sign.h"


/*	define led macros	*/
#define LED_ON	HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_RESET)
#define LED_OFF	HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_SET)

/*	Kernal Object Declaration	*/
static rt_event_t serv_led_sign_event;
#if !defined(RT_USING_IDLE_HOOK)||!defined(LED_SIGNAL_USE_IDLE)
	static rt_thread_t serv_led_sign;
#endif
/*	Static Varibles Declaration	*/
	/*	led blink chart	*/
const static uint16_t led_table[] = 
{
	100,800,100,800,							//Normal
	50,200,50,200,							//Fast blink
	100,300,100,900,							//Loss connection
	100,300,100,300,							//System error
	200,2000,200,2000,					//System Waiting
	0,0,0,0,
	0,0,0,0,
	40,10,10,10,							//Debug mode
};
static uint16_t led_param = 0;

/*	Extern Varibles Reference	*/


/*	Common Varibles Declaration	*/
uint16_t ticks = 0;

/*	Static Function Prototype	*/
#if defined(RT_USING_IDLE_HOOK)&&defined(LED_SIGNAL_USE_IDLE)
	static void serv_led_sign_idle();
#else
	static void serv_led_sign_main(void* para);
#endif
static void led_sig_set_index(serv_led_event_t event, uint8_t* index);
static void serv_write_led(uint8_t sig);
static void serv_toggle_led(uint8_t* sta);


void serv_led_sign_init()
{
	/*	Check if use RT-thread and if event IPC is enabled	*/
	#if !defined(USE_RTT)
		#error "Please check if use RT-thread system!"
	#endif
	#if defined(USE_RTT)&&!defined(RT_USING_EVENT)
		#error "Please uncomment "#define RT_USING_EVENT" in "rtconfig.h"!"
	#endif
	/*	Init event	*/
	serv_led_sign_event = rt_event_create("led_event",RT_IPC_FLAG_PRIO);
	if (RT_NULL == serv_led_sign_event)
	{
		#if defined(RT_USING_CONSOLE)
			rt_kprintf("Service led signal event creat fail!\r\n");
		#endif
	}
	/*	If defined RT_USING_IDLE_HOOK, this module will use idle hook to update
			led blink signal. If not, this module will create a service thread to 
			execute led signal.
	*/
	#if defined(RT_USING_IDLE_HOOK)&&defined(LED_SIGNAL_USE_IDLE)
		rt_thread_idle_sethook(serv_led_sign_idle);
	#else
	#if defined(RT_DEBUG)
		serv_led_sign = rt_thread_create("serv_led_sign",serv_led_sign_main,RT_NULL,512,RT_THREAD_PRIORITY_MAX/2 - 4, 2);
	#else
		serv_led_sign = rt_thread_create("serv_led_sign",serv_led_sign_main,RT_NULL,256,RT_THREAD_PRIORITY_MAX - 4, 1);
	#endif
	if (RT_NULL != serv_led_sign)
		{
			rt_thread_startup(serv_led_sign);
		}
	#endif
}
#if defined(RT_USING_IDLE_HOOK)&&defined(LED_SIGNAL_USE_IDLE)
void serv_led_sign_idle()
{
	
}
#else
static void serv_led_sign_main(void* para)
{
	uint32_t led_event = (uint32_t)NORMAL;
	rt_err_t slwu = RT_EOK;
	uint8_t led_sta = 0;
	
	uint8_t index = 0;
	uint16_t i = 0;
	while (1)
	{
		/*	First step: reveive led event	(RT_WAITING_NO)	*/
		slwu = rt_event_recv(serv_led_sign_event,(NORMAL|LONG_LIGHT|SHORT_LIGHT_ONESHOT|SHORT_LIGHT_PERIODIC|LONG_OFF|FAST_BLINK|WAITING),RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,ticks,&led_event);
		/*	If receive an event	*/
		if (RT_EOK == slwu)
		{
			led_sig_set_index(led_event,&index);
			i = index;
		}
		else if (-RT_ETIMEOUT == slwu)
		{
			switch (led_event)
			{
				case NORMAL:
				case FAST_BLINK:
				case WAITING:
				{
					serv_write_led(i);
					ticks = led_table[i];
					i ++;
					if (index + 4 == i)
					{
						i = i - 4;
					}
					break;
				}
				case LONG_LIGHT:
				{
					LED_ON;
					ticks = 1000;
					break;
				}
				case SHORT_LIGHT_ONESHOT:
				{
					/*	Led is on	*/
					if (0 == led_sta)
					{
						LED_ON;
						led_sta = 1;
						ticks = led_param;
					}
					/*	led off	*/
					else if (1 == led_sta)
					{
						LED_OFF;
						led_sta = 0;
						led_event = LONG_OFF;
						ticks = 100;
					}
					break;
				}
				case SHORT_LIGHT_PERIODIC:
				{
					serv_write_led(i);
					ticks = led_param;
					i ++;
					if (index + 4 == i)
					{
						i = i - 4;
					}
					break;
				}
				case LONG_OFF:
				{
					LED_OFF;
					ticks = 1000;
					break;
				}
//				case WAITING:
//				{
//					
//				}
				default:
				{
					
					break;
				}
			}
		}
	}
	
}
#endif

void serv_led_sign_send_event(serv_led_event_t event, uint16_t param)
{
	rt_event_send(serv_led_sign_event,(rt_uint32_t)event);
	led_param = param;
}

static void led_sig_set_index(serv_led_event_t event, uint8_t* index)
{
	switch (event)
	{
		case NORMAL:
		{
			*index = 0;
			ticks = led_table[*index];
			break;
		}
		case FAST_BLINK:
		{
			*index = 4;
			ticks = led_table[*index];
			break;
		}
		case WAITING:
		{
			*index = 16;
			ticks = led_table[*index];
			break;
		}
	}
}

static void serv_write_led(uint8_t sig)
{
	if (sig%2 == 0)
	{
		LED_ON;
	}
	else if (sig%2 == 1)
	{
		LED_OFF;
	}
}

static void serv_toggle_led(uint8_t* sta)
{
	switch (*sta)
	{
		case 0:
		{
			LED_ON;
			*sta = 1;
			break;
		}
		case 1:
		{
			LED_OFF;
			*sta = 0;
			break;
		}
	}
}

void serv_led_sign_hw_init()
{
	GPIO_InitTypeDef CS_Initure = {.Mode = GPIO_MODE_OUTPUT_PP,
																 .Pull = GPIO_PULLUP,
																 .Speed = GPIO_SPEED_FREQ_LOW,
																 .Pin = LED_PIN};
	HAL_GPIO_Init(LED_PORT, &CS_Initure);
}

/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/