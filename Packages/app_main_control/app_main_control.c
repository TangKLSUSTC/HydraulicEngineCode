
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     app_main_control.c
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
#include "app_main_control.h"

#define SWITCH_TO_AUTO_MODE \
				do {\
					WRITE_BITS(io_mode,0,1);\
					WRITE_BITS(io_mode,1,0);\
				}while(1)

#define SWITCH_TO_MANUAL_MODE \
				do {\
					WRITE_BITS(io_mode,0,0);\
					WRITE_BITS(io_mode,1,1);\
				}while(1)
#define DISABLE_IO_OUTPUT \
				do {\
					WRITE_BITS(io_mode,0,0);\
					WRITE_BITS(io_mode,1,0);\
				}while(1)

#define THREAD_STATE(thread) (thread->stat&RT_THREAD_STAT_MASK)
				
#define SWTIMER_ENABLE_FLAG (~READ_BITS(stop_timer->parent.flag , 0))
#define VALVE_ADDR 0x77
#define PUMP_ADDR  0x74
#define CHANNEL_NBR 8
#define PRESSURE_LIMIT	80	/* Global pressure limitation */
#define CMD_FEED_BACK
uint8_t io_mode = 0;			//Bit 0-1 : 00: disable; 01: auto; 10: manual; 11: reserved


#if 1
/*	Kernal Object Declaration	*/
rt_mutex_t control_mutex;
rt_sem_t sync_sem, usart_cmd_sem;
rt_timer_t stop_timer;
rt_timer_t calibration_timer;
rt_thread_t app_control = RT_NULL;
rt_thread_t cali_thread = RT_NULL;
rt_thread_t eject_air_thread = RT_NULL;
/*	Static Varibles Declaration	*/
double control_target[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};		//target is 60 kpa
double dead_zone[2] = {10,-10};
uint16_t ctrl_tmp[4] = {VALVE_ADDR,0x0000,PUMP_ADDR,0x0000};
uint16_t zero_point[16] = {2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048,2048};
double voltage_zero_point[16] = {0};
double real_pressure[16] = {0};
uint16_t* sensor_pointer_assignment[16] = {};
/*	Extern Varibles Reference	*/
extern dma_io_t usart1;
extern dma_io_t usart2;
extern dma_io_t usart6;
extern rt_event_t key_event;
#ifdef HPT_V3_1
extern rt_mailbox_t serv_ad_mailbox;
extern rt_mailbox_t io_data;
extern rt_event_t io_cmd_event;
#endif /*HPT_3_1*/
#ifdef HPT_V3_2
extern uint16_t adc_os_data[16];
extern uint8_t io_chip_nbr;
extern dev_tca9539_t tca9539[4];

#endif /*HPT_3_2*/
/*	Common Varibles Declaration	*/
#if defined(SENSORS_LEGANCY_MODE)

#endif /* SENSORS_LEGANCY_MODE */
#ifdef HPT_V3_1
mail_msg_t ad_msg_rec;
mail_msg_t io_msg_send;
#endif /*HPT_3_1*/
#ifdef HPT_V3_2
uint8_t chip_index = 0;
uint16_t loop_time = 65;
#endif /*HPT_3_2*/
uint16_t *adc_val_tmp;
double ctrl_err[16] = {0};
ctrl_sta_t timer_flag = CTRL_INIT, control_sta = CTRL_INIT;
uint16_t valve_lock = 0;
/*	Static Function Prototype	*/
static void timeout_entry(void* para);
static void test_process_start();
static void release_muscle();
static void app_main_control_main(void* para);
static int8_t serial_cmd_wrapper(uint8_t* buf);
static void app_adc2pressure(uint16_t* adc_val_array, double* pressure_array);
static void start_cali();
static void app_cali_func_auto();
static void app_eject_air_main(void* param);

void app_main_control_init()
{
	stop_timer = rt_timer_create("stop_timer",timeout_entry,RT_NULL,500,RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
	calibration_timer = rt_timer_create("cali_timer",start_cali,RT_NULL,500,RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
	usart_cmd_sem = rt_sem_create("serial_cmd", 0, RT_IPC_FLAG_PRIO);
	app_control = rt_thread_create("control",app_main_control_main,\
																	RT_NULL,1024,12,10);
	cali_thread = rt_thread_create("cali",app_cali_func_auto,\
																	RT_NULL,4096,6,5);
	eject_air_thread = rt_thread_create("Eject_air",app_eject_air_main,RT_NULL,512,8,2);
	#if defined(SENSORS_LEGANCY_MODE)
	
	#endif /* SENSORS_LEGANCY_MODE */
	#ifdef HPT_V3_1
	ad_msg_rec = (mail_msg_t)rt_malloc(sizeof(mail_msg));
	io_msg_send = (mail_msg_t)rt_malloc(sizeof(mail_msg));
	#endif /*HPT_3_1*/
	if (RT_NULL != app_control)
	{
		rt_thread_startup(app_control);
		serv_led_sign_send_event(WAITING,300);
	}
	
}
#ifdef DEBUG
MSH_CMD_EXPORT_ALIAS(app_main_control_init,ctrl_init,start main control thread);
#endif


/*********************************************************************
                        RTOS Thread Functions
**********************************************************************/
static void app_main_control_main(void* para)
{
	//rt_event_init(key_event,"key_event",RT_IPC_FLAG_PRIO);
	
	//rt_timer_init(stop_timer,"stop_timer",timeout_entry,RT_NULL,3000,RT_TIMER_FLAG_ONE_SHOT|RT_TIMER_FLAG_SOFT_TIMER);
	
	uint32_t key_event_var;
	rt_err_t key_event_err;
	rt_err_t serv_ad_mb_err,control_merr, serial_err;
	uint8_t pre_cnt = 0;
	uint16_t stable_flag = 0;
	rt_uint32_t cmd_event = 0;
	//int16_t *ctrl_err = (int16_t*)rt_calloc(16,sizeof(uint16_t));
	for (uint8_t k = 0; k < CHANNEL_NBR; k ++)
	{
		voltage_zero_point[k] = 0.00122*zero_point[k];
	}
	#ifdef HPT_V3_1
	io_msg_send->data_ptr = (void*)ctrl_tmp;
	io_msg_send->size = 4;
	/* Clear mailbox */
	serv_ad_mailbox->entry = 0;
	#endif /*HPT_3_1*/
	#ifdef HPT_V3_2

	#endif /*HPT_V3_2*/
	timer_flag = CTRL_TIMER_READY;
	control_sta = UNCALI;
	
	/* Auto calibration */
	rt_timer_start(calibration_timer);
	rt_kprintf("Control thread loop time = %dms\n",loop_time);
	//while (rt_mb_recv(serv_ad_mailbox,(rt_ubase_t*)&ad_msg_rec,RT_WAITING_FOREVER) == RT_EOK);
	while (1)
	{
		#ifdef HPT_V3_1
		/*	Receive adc value mail	*/
		serv_ad_mb_err = rt_mb_recv(serv_ad_mailbox,(rt_ubase_t*)&ad_msg_rec,RT_WAITING_FOREVER);
		if (RT_EOK == serv_ad_mb_err)
		{
			adc_val_tmp = (uint16_t*)(ad_msg_rec->data_ptr);
			app_adc2pressure(adc_val_tmp,real_pressure);
		#endif /*HPT_V3_1*/
		#ifdef HPT_V3_2
			app_adc2pressure(adc_os_data,real_pressure);
		#endif /*HPT_V3_2*/
			if (CALI_DONE == control_sta)
			{
				//usart2.dma_printf(&usart2,"Ch 0 =%d\r\n",adc_val_tmp[0]);
				for (uint8_t i_channel = 0; i_channel < CHANNEL_NBR; i_channel ++)
				{
					/* Check if target pressure is larger than limitation */
					if (control_target[i_channel] > PRESSURE_LIMIT)
					{
						control_target[i_channel] = 80;
					}
					else if (control_target[i_channel] < -PRESSURE_LIMIT)
					{
						control_target[i_channel] = -80;
					}
					if (~READ_BITS(valve_lock,i_channel))
					{
						ctrl_err[i_channel] = control_target[i_channel] - real_pressure[i_channel];
						/* Under low dead zone */
						if (ctrl_err[i_channel] < dead_zone[1])
						{
							/* Open input valve */
							WRITE_BITS(ctrl_tmp[1],i_channel,1);
							/* Close output valve */
							WRITE_BITS(ctrl_tmp[1],(i_channel+8),0);
							/* Open Pumps */
							WRITE_BITS(ctrl_tmp[3],0,1);
							WRITE_BITS(ctrl_tmp[3],1,1);
							WRITE_BITS(stable_flag,i_channel,0);
						}
						/* Upper high dead zone */
						else if (ctrl_err[i_channel] > dead_zone[0])
						{
							/* Close input valve */
							WRITE_BITS(ctrl_tmp[1],i_channel,0);
							/* Open output valve */
							WRITE_BITS(ctrl_tmp[1],(i_channel+8),1);
							/* Open Pumps */
							WRITE_BITS(ctrl_tmp[3],0,1);
							WRITE_BITS(ctrl_tmp[3],1,1);
							WRITE_BITS(stable_flag,i_channel,0);
						}
						else
						{
							/* Close input valve */
							WRITE_BITS(ctrl_tmp[1],i_channel,0);
							/* Close output valve */
							WRITE_BITS(ctrl_tmp[1],(i_channel+8),0);
							WRITE_BITS(stable_flag,i_channel,1);
						}
					}
					else
					{
						WRITE_BITS(stable_flag,i_channel,1);
						/* Close input valve */
						WRITE_BITS(ctrl_tmp[1],i_channel,0);
						/* Close output valve */
						WRITE_BITS(ctrl_tmp[1],(i_channel+8),0);
					}
					
				}
				if (stable_flag == 0x00ff)
				{
					
				}
				usart6.dma_printf(&usart6,"/*AT,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f*/\r\n",ctrl_err[0],ctrl_err[1],ctrl_err[2],ctrl_err[3],ctrl_err[4],ctrl_err[5],ctrl_err[6],ctrl_err[7]);
				//usart2.dma_printf(&usart2,"V = 0x%X, P = 0x%x\r\n",ctrl_tmp[1],ctrl_tmp[3]);
				#ifdef HPT_V3_1
				rt_mb_send(io_data,(rt_ubase_t)io_msg_send);
				#endif /*HPT_V3_1*/
				#ifdef HPT_V3_2
				#if 1
				for (uint8_t i = 0; i < io_chip_nbr*2; i += 2)
				{
					if (ctrl_tmp[i] < 0x78 && ctrl_tmp[i] > 0x73)
					{
						chip_index = ctrl_tmp[i]&0x0f - 4;		
						dev_tca9539_write_aio(&tca9539[chip_index],ctrl_tmp[i+1]);
						core_delay_us(100);
						//rt_thread_mdelay(1);
					}
				}
				#else
				dev_tca9539_write_aio(&tca9539[0],0xffff);
				dev_tca9539_write_aio(&tca9539[3],0xffff);
				rt_thread_mdelay(10);
				dev_tca9539_write_aio(&tca9539[0],0x0000);
				dev_tca9539_write_aio(&tca9539[3],0x0000);
				rt_thread_mdelay(15);
				#endif /*0*/
				#endif /*HPT_V3_2*/
				//rt_event_send(io_cmd_event,THREAD_CMD);
				
				if ((stable_flag & 0x00ff) == 0xff)
				{
					/* Stable */
					
					if (CTRL_TIMER_READY == timer_flag)
					{
						timer_flag = CTRL_TIMER_ARMED;
						serv_led_sign_send_event(LONG_LIGHT,0);
					}
				}
				else
				{
					/* Unstable */
					if (CTRL_TIMER_DONE == timer_flag)
					{
						timer_flag = CTRL_TIMER_READY;
						serv_led_sign_send_event(FAST_BLINK,300);
					}
					//rt_timer_stop(stop_timer);
					
				}
				if (CTRL_TIMER_ARMED == timer_flag)
				{
					rt_timer_start(stop_timer);
					timer_flag = CTRL_TIMER_DONE;
				}
			}
		#ifdef HPT_V3_1
		}
		#endif /*HPT_V3_1*/
		/*
		Update control value
		*/
		#if 0
		serial_err = rt_event_recv(io_cmd_event,USART_CMD,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,&cmd_event);
		if (RT_EOK == serial_err)
		{
			serial_cmd_wrapper(usart2.dma_rx_buf);
		}
		#endif
		/*	Receive key pressed event	*/
		
		#ifdef HPT_V3_2
			rt_thread_mdelay(loop_time);
		#endif /*HPT_V3_2*/
	}
}

/*	test process comes to end	*/
static void timeout_entry(void* para)
{
	/* Close all valves and pumps */
	ctrl_tmp[1] = 0x0000;
	ctrl_tmp[3] = 0x0000;
	//rt_event_send(io_cmd_event,THREAD_CMD);
	#ifdef HPT_V3_1
	rt_mb_send(io_data,(rt_ubase_t)ctrl_tmp);
	#endif /*HPT_V3_1*/
	#ifdef HPT_V3_2
	for (uint8_t i = 0; i < io_chip_nbr*2; i += 2)
	{
		if (ctrl_tmp[i] < 0x78 && ctrl_tmp[i] > 0x73)
		{
			chip_index = ctrl_tmp[i]&0x0f - 4;		
			dev_tca9539_write_aio(&tca9539[chip_index],ctrl_tmp[i+1]);
			delay_us(20);
		}
	}
	#endif /*HPT_V3_2*/
	//serv_led_sign_send_event(LONG_LIGHT,0);
	rt_kprintf("Reach target!\n");
	
	//timer_flag = 3;
}

/*
	Calibration thread start
*/
static void start_cali()
{
	rt_thread_startup(cali_thread);
}

/*
	Ejecting air thread entry
*/
static void app_eject_air_main(void* param)
{
	while (1)
	{
		#if 1
		for (uint8_t i_channel = 0; i_channel < CHANNEL_NBR; i_channel ++)
		{
			control_target[i_channel] = 20;
		}
		rt_thread_mdelay(1000);
		for (uint8_t i_channel = 0; i_channel < CHANNEL_NBR; i_channel ++)
		{
			control_target[i_channel] = -20;
		}
		rt_thread_mdelay(1000);
		#else
		control_target[0] = 40;
		rt_thread_mdelay(1700);
		control_target[0] = -40;
		rt_thread_mdelay(4000);
		#endif
	}
}


/* Calibration thread entry */
static void app_cali_func_auto()
{
	#if defined(SENSORS_LEGANCY_MODE)
	dev_tca9539_write_aio(&tca9539[3],0xffff);
	rt_thread_mdelay(1000);
	uint16_t tmp[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		for (uint8_t j = 0; j < 16; j ++)
		{
			for (uint8_t i = 0; i < 16; i ++)
			{
				tmp[i] += adc_os_data[i];
			}
			rt_thread_mdelay(40);
		}
		#if 1
		rt_kprintf("ADC value Zero calibration:\n");
		for (uint8_t i = 0; i < 16; i ++)
		{
			tmp[i] = tmp[i]>>4;
			rt_kprintf("Ch %d = %d\n",i,tmp[i]);
			zero_point[i] = tmp[i];
			//control_target[i] = tmp[i];
		}
		rt_kprintf("Voltage value Zero calibration:\n");
		for (uint8_t k = 0; k < CHANNEL_NBR; k ++)
		{
			voltage_zero_point[k] = 0.00122*zero_point[k];
			usart1.dma_printf(&usart1,"Ch %d = %.2fV\r\n",k,voltage_zero_point[k]);
		}
		#endif
		rt_kprintf("Done\n");
		control_sta = CALI_DONE;
		serv_led_sign_send_event(FAST_BLINK,0);
//		rt_timer_stop(calibration_timer);
//		rt_timer_delete(calibration_timer);
		return;
		#endif /* SENSORS_LEGANCY_MODE */
}

#endif



/*********************************************************************
                          Static Functions
**********************************************************************/

/*
		Serial command format:
		Frame head: /,length
		Data: channel,value_high 8-bit,value_low 8-bit
		Frame tail: *
*/
static int8_t serial_cmd_wrapper(uint8_t* buf)
{
	/* Check frame head */
	if (buf[0] != '/')
	{
		goto serial_cmd_wrapper_err_exit;
	}
	/* Check frame integrity */
	if (buf[1] != usart2.dma_rx_length)
	{
		goto serial_cmd_wrapper_err_exit;
	}
	/* Check frame tail */
	if (buf[buf[1]-1] != '*')
	{
		goto serial_cmd_wrapper_err_exit;
	}
	/* Check numbers of data groups */
	if (buf[1]%3 != 0)
	{
		goto serial_cmd_wrapper_err_exit;
	}
	
	/* Put data into control target group */
	uint8_t channel_tmp = 0;
	uint16_t value_tmp = 0, low = 0, high = 0;
	
	for (uint8_t i = 2; i < buf[1] - 3; i += 3)
	{
		channel_tmp = buf[i];
		high = ((uint16_t)buf[i+1])<<8;
		low = (uint16_t)buf[i+2];
		value_tmp = high | low;
		if (value_tmp > 4095)
		{
			goto serial_cmd_wrapper_err_exit;
		}
		if (channel_tmp > 15)
		{
			goto serial_cmd_wrapper_err_exit;
		}
		control_target[channel_tmp] = value_tmp;
	}
	return 1;
	
	serial_cmd_wrapper_err_exit:
	{
		return -1;
	}
}

/*
		Calculate real pressure values
*/
static void app_adc2pressure(uint16_t* adc_val_array, double* pressure_array)
{
	double voltage_tmp;
	for (uint8_t i = 0; i < CHANNEL_NBR; i ++)
	{
		voltage_tmp = 0.00122*adc_val_array[i] - voltage_zero_point[i];
		pressure_array[i] = 124*voltage_tmp;
	}
}




/*********************************************************************
                          Command Functions
**********************************************************************/

/*
		Shell command format:
		ctrl <channel,value>
*/
void app_update_control_value(int argc, char** argv)
{
	uint8_t chntmp = 0;
	double value_tmp = 0;
	if (argc < 2)
	{
		goto app_update_control_value_err_exit;
	}
	if (argc > 2)
	{
		if (argc > 18) goto app_update_control_value_err_exit;
		if (argc%2 == 1)
		{
			
			for (uint8_t i = 1; i < argc; i += 2)
			{
				chntmp = (uint8_t)atoi(argv[i]);
				value_tmp = (double)atof(argv[i+1]);
				if (chntmp < 16 && value_tmp < 200 && value_tmp > -100)
				{
					if (value_tmp > PRESSURE_LIMIT)
					{
						#ifdef CMD_FEED_BACK
						//usart1.dma_printf(&usart1,"Chn %d ctrl value is larger than limitation!\r\n");
						usart1.dma_printf(&usart1,"Chn %d valves are locked!\r\n");
						#endif /* CMD_FEED_BACK */
						WRITE_BITS(valve_lock, chntmp, 1);
						//control_target[chntmp] = PRESSURE_LIMIT;
						continue;
					}
					else if (value_tmp < -PRESSURE_LIMIT)
					{
						#ifdef CMD_FEED_BACK
						//usart1.dma_printf(&usart1,"Chn %d ctrl value is lower than limitation!\r\n");
						usart1.dma_printf(&usart1,"Chn %d valves are unlocked!\r\n");
						#endif /* CMD_FEED_BACK */
						WRITE_BITS(valve_lock, chntmp, 0);
						//control_target[chntmp] = -PRESSURE_LIMIT;
						continue;
					}
					else
					{
						control_target[chntmp] = value_tmp;
						#ifdef CMD_FEED_BACK
						usart1.dma_printf(&usart1,"Chn %d ctrl value = %.2f\r\n",chntmp,control_target[chntmp]);
						#endif /* CMD_FEED_BACK */
					}
				}
				else goto app_update_control_value_exception_exit;
			}
		}
		return;
	}
	app_update_control_value_err_exit:
	{
		#ifdef CMD_FEED_BACK
		rt_kprintf("Please check parameter!\n");
		rt_kprintf("The right command is \n");
		rt_kprintf("set_ctrl <channel, target>\n");
		#endif /* CMD_FEED_BACK */
		return;
	}
	app_update_control_value_exception_exit:
	{
		#ifdef CMD_FEED_BACK
		if (chntmp > 15)
		{
			rt_kprintf("Please check channel number\n");
		}
		if (value_tmp > 4095)
		{
			rt_kprintf("Please check channel target value\n");
		}
		#endif /* CMD_FEED_BACK */
		return;
	}
}
MSH_CMD_EXPORT_ALIAS(app_update_control_value,set_ctrl,set_ctrl <channel_nbr target>);

/*
		Shell command format:
		checktarget
*/
void app_check_control_value(int argc, char** argv)
{
	if (argc > 2)
	{
		goto app_check_control_value_exit;
	}
	else
	{
		for (uint8_t i = 0; i < 16; i ++)
		{
			#ifdef CMD_FEED_BACK
			usart1.dma_printf(&usart1,"Ch %d target is %.2f\r\n",i,control_target[i]);
			#endif /* CMD_FEED_BACK */
		}
		return;
	}
	app_check_control_value_exit:
	{
		#ifdef CMD_FEED_BACK
		rt_kprintf("This command do not need any parameters!\n");
		#endif /* CMD_FEED_BACK */
		return;
	}
}
MSH_CMD_EXPORT_ALIAS(app_check_control_value,checktarget,check control target);

/*
		Shell command format:
		checkerr
*/
void app_check_error_value(int argc, char** argv)
{
	if (argc > 2)
	{
		goto app_check_error_value_exit;
	}
	else
	{
		for (uint8_t i = 0; i < 16; i ++)
		{
			#ifdef CMD_FEED_BACK
			usart1.dma_printf(&usart1,"Ch %d error is %.2f\r\n",i,ctrl_err[i]);
			#endif /* CMD_FEED_BACK */
		}
		return;
	}
	app_check_error_value_exit:
	{
		#ifdef CMD_FEED_BACK
		rt_kprintf("This command do not need any parameters!\n");
		#endif /* CMD_FEED_BACK */
		return;
	}
}
MSH_CMD_EXPORT_ALIAS(app_check_error_value,checkerr,check control error);


/*
		Shell command format:
		zerocali
*/
void app_zero_calibration(int argc, char** argv)
{
	if (argc > 2)
	{
		goto app_zero_calibration_err_exit;
	}
	else
	{
		/* Check if app thread is running */
		if (app_control == RT_NULL)
		{
			#ifdef CMD_FEED_BACK
			rt_kprintf("Please run command \"ctrl_init\" first!\n");
			#endif /* CMD_FEED_BACK */
			return;
		}
		uint16_t tmp[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		for (uint8_t j = 0; j < 16; j ++)
		{
			for (uint8_t i = 0; i < 16; i ++)
			{
				tmp[i] += adc_val_tmp[i];
			}
			rt_thread_mdelay(40);
		}
		#if 1
		#ifdef CMD_FEED_BACK
		rt_kprintf("ADC value Zero calibration:\n");
		#endif /* CMD_FEED_BACK */
		for (uint8_t i = 0; i < 16; i ++)
		{
			tmp[i] = tmp[i]>>4;
			#ifdef CMD_FEED_BACK
			rt_kprintf("Ch %d = %d\n",i,tmp[i]);
			#endif /* CMD_FEED_BACK */
			zero_point[i] = tmp[i];
			//control_target[i] = tmp[i];
		}
		#ifdef CMD_FEED_BACK
		rt_kprintf("Voltage value Zero calibration:\n");
		#endif /* CMD_FEED_BACK */
		for (uint8_t k = 0; k < CHANNEL_NBR; k ++)
		{
			voltage_zero_point[k] = 0.00122*zero_point[k];
			#ifdef CMD_FEED_BACK
			usart1.dma_printf(&usart1,"Ch %d = %.2fV\r\n",k,voltage_zero_point[k]);
			#endif /* CMD_FEED_BACK */
		}
		#endif
		#ifdef CMD_FEED_BACK
		rt_kprintf("Done\n");
		#endif /* CMD_FEED_BACK */
		control_sta = CALI_DONE;
		serv_led_sign_send_event(FAST_BLINK,0);
		return;
	}
	app_zero_calibration_err_exit:
	{
		#ifdef CMD_FEED_BACK
		rt_kprintf("This command do not need any parameters!\n");
		#endif /* CMD_FEED_BACK */
		return;
	}
}
MSH_CMD_EXPORT_ALIAS(app_zero_calibration,zerocali,calibration controller zero points);


/*
		Shell command format:
		set_deadzone up <number> down <number>
*/
void app_set_deadzone(int argc, char** argv)
{
	if (argc == 1)
	{
		#ifdef CMD_FEED_BACK
		rt_kprintf("This command need parameters\n");
		#endif /* CMD_FEED_BACK */
	}
	else if (argc % 2 == 1)
	{
		double tmp[2] = {0};
		for (uint8_t i = 1; i < argc - 1; i += 2)
		{
			if (!rt_strcmp(argv[i],"up"))
			{
				tmp[0] = (double)atof(argv[i+1]);
				if (tmp[0] < 0)
				{
					#ifdef CMD_FEED_BACK
					rt_kprintf("Up boundry cannot be lower than 0!\n");
					#endif /* CMD_FEED_BACK */
					return;
				}
                else
                {
                    dead_zone[0] = tmp[0];
                }
			}
			if (!rt_strcmp(argv[i],"down"))
			{
                tmp[1] = (double)atof(argv[i+1]);
                if (tmp[1] > 0)
                {
									#ifdef CMD_FEED_BACK
									rt_kprintf("Down boundry cannot be larger than 0!\n");
									#endif /* CMD_FEED_BACK */
									return;
                }
				dead_zone[1] = tmp[1];
			}
		}
		#ifdef CMD_FEED_BACK
		rt_kprintf("Dead zone is set to\n");
		usart1.dma_printf(&usart1,"Up bound = %.2f\r\n",dead_zone[0]);
		usart1.dma_printf(&usart1,"Down bound = %.2f\r\n",dead_zone[1]);
		#endif /* CMD_FEED_BACK */
	}
	return;
}
MSH_CMD_EXPORT_ALIAS(app_set_deadzone,set_deadzone,set_deadzone up <number> down <number>);

/*
		Shell command format:
		eject_air <channel>
*/
void app_eject_air(int argc, char** argv)
{
	#if 0
	
	#endif
	switch (THREAD_STATE(eject_air_thread))
	{
		case RT_THREAD_READY:
		case RT_THREAD_RUNNING:
		case RT_THREAD_SUSPEND:
		{
			break;
		}
		case RT_THREAD_INIT:
		{
			rt_thread_startup(eject_air_thread);
			#ifdef CMD_FEED_BACK
			rt_kprintf("Eject air start!\n");
			#endif /* CMD_FEED_BACK */
			return;
		}
		case RT_THREAD_CLOSE:
		{
			eject_air_thread = rt_thread_create("Eject_air",app_eject_air_main,RT_NULL,512,8,2);
			if (RT_NULL != eject_air_thread)
			{
				rt_thread_startup(eject_air_thread);
				#ifdef CMD_FEED_BACK
				rt_kprintf("Eject air start!\n");
				#endif /* CMD_FEED_BACK */
			}
		}
	}
	
}
MSH_CMD_EXPORT_ALIAS(app_eject_air,eject_air,eject_air <number>);

/*
		Shell command format:
		eject_air_stop
*/
void app_eject_air_stop(int argc, char** argv)
{
	switch (THREAD_STATE(eject_air_thread))
	{
		case RT_THREAD_READY:
		case RT_THREAD_RUNNING:
		case RT_THREAD_SUSPEND:
		{
			rt_thread_delete(eject_air_thread);
		}
		case RT_THREAD_INIT:
		{
			#ifdef CMD_FEED_BACK
			rt_kprintf("Eject air thread is closing!\n");
			#endif /* CMD_FEED_BACK */
			break;
		}
		case RT_THREAD_CLOSE:
		{
			#ifdef CMD_FEED_BACK
			rt_kprintf("Eject air thread is already closed!\n");
			#endif /* CMD_FEED_BACK */
			return;
		}
	}
	rt_thread_mdelay(1000);
	for (uint8_t i_channel = 0; i_channel < CHANNEL_NBR; i_channel ++)
	{
		control_target[i_channel] = 0;
	}
}
MSH_CMD_EXPORT_ALIAS(app_eject_air_stop,eject_air_stop,eject_air_stop);

/*
	Release pressure
	release <channel>
*/
void app_release_pressure_start(int argc, char** argv)
{
	uint8_t chntmp = 0;
	uint16_t value_tmp = 0;
	switch ((app_control->stat&RT_THREAD_STAT_MASK))
	{
		case RT_THREAD_READY:
		case RT_THREAD_RUNNING:
		case RT_THREAD_SUSPEND:
		{
			if (argc < 2)
			{
				goto app_release_pressure_err_exit;
			}
			if (argc >= 2)
			{
				if (argc > 11) goto app_release_pressure_err_exit;
				
				rt_thread_delete(app_control);
				if (!rt_strcmp(argv[1],"all"))
				{
					dev_tca9539_write_aio(&tca9539[3],0xffff);
					dev_tca9539_write_aio(&tca9539[0],0x0000);
					#ifdef CMD_FEED_BACK
					rt_kprintf("Release all muscle pressure!\n");
					#endif /* CMD_FEED_BACK */
					return;
				}
				for (uint8_t i = 1; i < argc; i ++)
				{
					chntmp = (uint8_t)atoi(argv[i]);
					if (chntmp < 8)
					{
						WRITE_BITS(value_tmp, chntmp, 1);
						WRITE_BITS(value_tmp, (chntmp + 8), 1);
					}
					else goto app_release_pressure_exception_exit;
				}
				dev_tca9539_write_aio(&tca9539[3],value_tmp);
				dev_tca9539_write_aio(&tca9539[0],0x0000);
				//dev_tca9539_write_aio(&tca9539[0],0x0003);
				return;
			}
		}
		case RT_THREAD_INIT:
		{
			#ifdef CMD_FEED_BACK
			rt_kprintf("App Muscle control is not startup!\n");
			#endif /* CMD_FEED_BACK */
			return;
		}
		case RT_THREAD_CLOSE:
		{
			#ifdef CMD_FEED_BACK
			rt_kprintf("App Muscle control is stopped!\n");
			#endif /* CMD_FEED_BACK */
			return;
		}
	}
	
	app_release_pressure_err_exit:
	{
		#ifdef CMD_FEED_BACK
		rt_kprintf("Please check parameter!\n");
		rt_kprintf("The right command is \n");
		rt_kprintf("release <channel>\n");
		#endif /* CMD_FEED_BACK */
		return;
	}
	app_release_pressure_exception_exit:
	{
		#ifdef CMD_FEED_BACK
		if (chntmp > 15)
		{
			rt_kprintf("Please check channel number\n");
		}
		#endif /* CMD_FEED_BACK */
		return;
	}
}
MSH_CMD_EXPORT_ALIAS(app_release_pressure_start,release,release <channel>);

/*
	Resume valve and control
	resume <channel>
*/
void app_release_pressure_stop(int argc, char** argv)
{
	rt_err_t uwret;
	dev_tca9539_write_aio(&tca9539[3],0x0000);
	dev_tca9539_write_aio(&tca9539[0],0x0000);
	for (uint8_t i_channel = 0; i_channel < CHANNEL_NBR; i_channel ++)
	{
		control_target[i_channel] = 0;
	}
	switch ((app_control->stat&RT_THREAD_STAT_MASK))
	{
		case RT_THREAD_READY:
		case RT_THREAD_RUNNING:
		case RT_THREAD_SUSPEND:
		{
			#ifdef CMD_FEED_BACK
			rt_kprintf("App Muscle control Thread is already running!\n");
			#endif /* CMD_FEED_BACK */
			return;
		}
		case RT_THREAD_INIT:
		{
			uwret = rt_thread_startup(app_control);
			if (uwret!=RT_EOK)
			{
				#ifdef CMD_FEED_BACK
				rt_kprintf("App Muscle Control Thread error!\n");
				rt_kprintf("Please press \"RESET\" button\n");
				#endif /* CMD_FEED_BACK */
				return;
			}
			return;
		}
		case RT_THREAD_CLOSE:
		{
			#ifdef CMD_FEED_BACK
			rt_kprintf("Muscle control is stopped!\n");
			#endif /* CMD_FEED_BACK */
			//rt_free(app_control);
			app_control = rt_thread_create("control",app_main_control_main,\
																	RT_NULL,1024,12,10);
			if (THREAD_STATE(cali_thread) == RT_THREAD_CLOSE)
			{
				cali_thread = rt_thread_create("cali",app_cali_func_auto,\
																	RT_NULL,4096,6,5);
			}
			if (RT_NULL != app_control)
			{
				uwret = rt_thread_startup(app_control);
			}
			if (uwret!=RT_EOK)
			{
				#ifdef CMD_FEED_BACK
				rt_kprintf("App Muscle Control Thread error!\n");
				rt_kprintf("Please press \"RESET\" button\n");
				#endif /* CMD_FEED_BACK */
				return;
			}
			else
			{
				#ifdef CMD_FEED_BACK
				rt_kprintf("App Muscle Control Thread Restarted!\n");
				#endif /* CMD_FEED_BACK */
			}
			break;
		}
	}
	
}
MSH_CMD_EXPORT_ALIAS(app_release_pressure_stop,resume,resume <channel>);

/*
	Change control loop time in ms
	loop_time <time>
*/
void app_change_loop_time(int argc, char** argv)
{
	if (argc < 2)
	{
		#ifdef CMD_FEED_BACK
		rt_kprintf("Check parameters\n");
		#endif /* CMD_FEED_BACK */
	}
	if (argc == 2)
	{
		uint16_t time_tmp = 0;
		time_tmp = (uint16_t)atoi(argv[1]);
		if (time_tmp > 0)
		{
			loop_time = time_tmp;
			#ifdef CMD_FEED_BACK
			rt_kprintf("Control loop time is changed into %dms\n",loop_time);
			#endif /* CMD_FEED_BACK */
		}
	}
}
MSH_CMD_EXPORT_ALIAS(app_change_loop_time,loop_time,loop_time <time>);

/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/