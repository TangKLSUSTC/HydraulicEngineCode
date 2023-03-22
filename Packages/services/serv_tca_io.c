
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_tca_io.c
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
#include "serv_tca_io.h"



#if 0
				
/*	Kernal Object Declaration	*/

rt_thread_t serv_tca_io = RT_NULL;


rt_mutex_t io_lock = RT_NULL;
rt_event_t  io_cmd_event = RT_NULL;
rt_mailbox_t io_data = RT_NULL;

/*	Static Varibles Declaration	*/
const dev_tca9539_addr tcaAddr[4] = {LL,LH,HL,HH};
dev_tca9539_addr exist_addr[4] = {};
dev_tca9539_t tca9539[4] = {};
uint16_t ioValue_buf[4] = {};
uint8_t io_chip_nbr = 0;
	
uint16_t urgent_tmp[5] = {CTRL_EMERGENCY,0x0000,0x0000,0x0000,0x0000};
/*	Extern Varibles Reference	*/
extern dma_io_t usart2;

/*	Common Varibles Declaration	*/
mail_msg_t io_urgent;
mail_msg_t io_msg_rec;

/*	Static Function Prototype	*/
static void serv_tca_io_main(void* para);
static void io_cmd_wrapper(uint8_t* cmd_buf, uint16_t* ioValue_buf);
uint8_t tca9539_probe();
static void serv_io_emergency_attach();
static void serv_tca_io_hook(struct rt_thread* from, struct rt_thread* to);
void serv_tca_io_init()
{
	io_lock = rt_mutex_create("io_lock",RT_IPC_FLAG_PRIO);
	io_cmd_event = rt_event_create("io_event",RT_IPC_FLAG_PRIO);
	io_data = rt_mb_create("io_data",64,RT_IPC_FLAG_PRIO);
	/* Receive io data from other threads */
	io_msg_rec = (mail_msg_t)rt_malloc(sizeof(mail_msg));
	/* Urgent message */
	io_urgent = (mail_msg_t)rt_malloc(sizeof(mail_msg));
	io_urgent->data_ptr = (void*)urgent_tmp;
	io_urgent->size = 5;
	serv_tca_io = rt_thread_create("serv_tcaio_t",serv_tca_io_main,RT_NULL,2048,5,5);
	
	//rt_scheduler_sethook(serv_tca_io_hook); 
	if (RT_NULL != serv_tca_io && RT_NULL != io_lock)
	{
		rt_thread_startup(serv_tca_io);
	}
	else
	{
		#if defined(USE_DMA_IO)
			rt_kprintf("Dynamic heap allocate fail!\n");
		#endif
	}
}
	
static void serv_tca_io_main(void* para)
{
	rt_err_t res_event, res_iodata, res_mutex;
	rt_uint32_t cmd_event = 0, output = 0;
	uint8_t chip_index = 0;
	uint16_t *io_val_tmp = (uint16_t*)rt_calloc(io_chip_nbr*2,sizeof(uint16_t));
	//uint16_t *io_history_data = (uint16_t*)rt_calloc(io_chip_nbr*2,sizeof(uint16_t));
//	for (uint8_t i = 0; i < io_chip_nbr*2; i ++)
//	{
//		io_history_data[i] = 0;
//	}
	while (1)
	{
		//res_sem = rt_sem_take(usart_cmd_sem,RT_WAITING_FOREVER);
		#if 0
		res_event = rt_event_recv(io_cmd_event,USART_CMD,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,&cmd_event);
		/* Write usart control value */
		if (RT_EOK == res_event)
		{
			io_cmd_wrapper(usart2.dma_rx_buf,ioValue_buf);
			res_mutex = rt_mutex_take(io_lock,RT_WAITING_FOREVER);
			if (RT_EOK == res_mutex)
			{
				for (uint8_t i = 0; i < IO_CHIP_NBR; i ++)
				{
					dev_tca9539_write_aio(&tca9539[i],ioValue_buf[i]);
				}
				rt_mutex_release(io_lock);
			}
		} /* Write usart control value */
		#endif
		#if 0
		res_event = rt_event_recv(io_cmd_event,THREAD_CMD,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,&cmd_event);
		if (RT_EOK == res_event)
		{
		#endif
		#ifdef HPT_V3_1
			res_iodata = rt_mb_recv(io_data,(rt_ubase_t*)&io_msg_rec,RT_WAITING_FOREVER);
			if (RT_EOK == res_iodata)
			{
				io_val_tmp = (uint16_t*)(io_msg_rec->data_ptr);
				if (io_val_tmp[0] == CTRL_NORMAL)
				{
					rt_kprintf("Receive normal data\n");
				}
				res_mutex = rt_mutex_take(io_lock,RT_WAITING_FOREVER);
				if (RT_EOK == res_mutex)
				{
					
					for (uint8_t i = 0; i < io_chip_nbr*2; i += 2)
					{
						if (io_val_tmp[i] < 0x78 && io_val_tmp[i] > 0x73)
						{
							chip_index = io_val_tmp[i]&0x0f - 4;
							/* Check address */
//							if (chip_index < io_chip_nbr)
//							{
								//rt_kprintf("%x ctrl data %x\n", chip_index, io_val_tmp[i+1]);
							/* Check if data changes */			
							dev_tca9539_write_aio(&tca9539[chip_index],io_val_tmp[i+1]);
							delay_us(100);
//							}
						}
					}
					rt_mutex_release(io_lock);
				}
				#endif /*HPT_V3_1*/
				#if 0
				switch (io_val_tmp[0])
				{
					case CTRL_NORMAL:
					{
						#ifdef DEBUG
						//rt_kprintf("IO service receive normal control\n");
						#endif
						res_mutex = rt_mutex_take(io_lock,RT_WAITING_FOREVER);
						if (RT_EOK == res_mutex)
						{
							
							for (uint8_t i = 1; i < io_chip_nbr*2+1; i += 2)
							{
								if (io_val_tmp[i] < 0x78 && io_val_tmp[i] > 0x73)
								{
									chip_index = io_val_tmp[i]&0x0f - 4;
									/* Check address */
		//							if (chip_index < io_chip_nbr)
		//							{
										//rt_kprintf("%x ctrl data %x\n", chip_index, io_val_tmp[i+1]);
										dev_tca9539_write_aio(&tca9539[chip_index],io_val_tmp[i+1]);
		//							}
								}
							}
							rt_mutex_release(io_lock);
						}
						break;
					}
					case CTRL_EMERGENCY:
					{
					
						#ifdef DEBUG
						rt_kprintf("IO service receive emergency\n");
						#endif
						for (uint8_t i = 1; i < io_chip_nbr*2+1; i += 2)
						{
							if (io_val_tmp[i] < 0x78 && io_val_tmp[i] > 0x73)
							{
								chip_index = io_val_tmp[i]&0x0f - 4;
								/* Check address */
	//							if (chip_index < io_chip_nbr)
	//							{
									//rt_kprintf("%x ctrl data %x\n", chip_index, io_val_tmp[i+1]);
								dev_tca9539_write_aio(&tca9539[chip_index],0x0000);
	//							}
							}
						}
						io_data->entry = 0;
						break;
					}
				}
				#endif
			#ifdef DEBUG
			//rt_thread_mdelay(10);
			#endif
			}
		#if 0
		
		}
		#endif
			/* Emergency stop */
		#if 0
		res_event = rt_event_recv(io_cmd_event,EMERGENCY_STOP,RT_EVENT_FLAG_OR|RT_EVENT_FLAG_CLEAR,RT_WAITING_FOREVER,&cmd_event);
		{
			#ifdef DEBUG
			rt_kprintf("Triger an emergency stop event!\n");
			#endif
			if (RT_EOK == res_event)
			{
				for (uint8_t i = 0; i < IO_CHIP_NBR; i ++)
				{
					dev_tca9539_write_aio(&tca9539[i],0x0000);
				}
			}
			#ifndef DEBUG
				rt_thread_t self = rt_thread_self();
				rt_thread_control(self,RT_THREAD_CTRL_CLOSE,RT_NULL);
			#endif
		}
		#endif
	}
}

extern rt_thread_t app_control;
static void serv_tca_io_hook(struct rt_thread* from, struct rt_thread* to)
{
	if (from ==  app_control && to == serv_tca_io)
	{
		rt_kprintf("App wake up io thread!\n");
	}
	
}


int serv_tca_hw_init(I2C_HandleTypeDef* iic_handle)
{
	dev_tca9539_t tca_tmp;
	uint8_t nbr = 0;
	tca_tmp.bus_handle = iic_handle;
	tca_tmp.cfg = 0x0000;
	tca_tmp.GPIO_OUT = 0x0000;
	//rt_kprintf("Probing tca9539 chip!\n");
	for (uint8_t i = 0; i < 4; i ++)
	{
		rt_sprintf(tca_tmp.name,"exio%d",0);
		tca_tmp.bus_addr = tcaAddr[i];
		tca9539[i].bus_handle = iic_handle;
		tca9539[i].bus_addr = tcaAddr[i];
		tca9539[i].cfg = 0x0000;
		tca9539[i].GPIO_OUT = 0x0000;
		rt_sprintf(tca9539[i].name,"exio%d",i);
		if (1 == tca9539_probe(&tca_tmp))
		{
			nbr ++;
			dev_tca9539_init(&tca9539[i],0x0000);
		}
		delay_us(100);
	}
	#if 1
	if (nbr != 0)
	{
		io_chip_nbr = nbr;
	}
	else
	{
		rt_kprintf("Please check line connection!\n");
		return 0;
	}
	if (io_chip_nbr != IO_CHIP_NBR)
	{
		rt_kprintf("Please check line connection!\n");
	}
	#endif
	//serv_io_emergency_attach();
	return 1;
}

uint8_t tca9539_check()
{
	dev_tca9539_t tca_tmp;
	uint8_t nbr = 0;
	tca_tmp.bus_handle = &hi2c1;
	tca_tmp.cfg = 0x0000;
	tca_tmp.GPIO_OUT = 0x0000;
	rt_kprintf("Probing tca9539 chip!\n");
	for (uint8_t i = 0; i < 4; i ++)
	{
		rt_sprintf(tca_tmp.name,"exio%d",0);
		tca_tmp.bus_addr = tcaAddr[i];
		if (1 == tca9539_probe(&tca_tmp))
		{
			nbr ++;
		}
		delay_us(100);
	}
	return nbr;
}
MSH_CMD_EXPORT(tca9539_check, manual probe tca9539 chip);
/*
Cmd format:
[0] : chip number(address)
[1] : High 8-bit channel value (P1.x)
[2] : Low 8-bit channel value (P0.x)
*/
static void io_cmd_wrapper(uint8_t* cmd_buf, uint16_t* ioValue)
{
	uint16_t tmp = 0,low = 0,high = 0;
	for (uint8_t i = 0; i < (io_chip_nbr*3); i += 3)
	{
		if (cmd_buf[i] >= LL && cmd_buf[i] <= HH)
		{
			low = (uint16_t)cmd_buf[i+2];
			high = ((uint16_t)cmd_buf[i+1])<<8;
			tmp = low | high;
			rt_kprintf("Input control value:\n");
			rt_kprintf("Bus addr = 0x%X\n",cmd_buf[i]);
			rt_kprintf("Low 8-bit = 0x%X\n",low);
			rt_kprintf("High 8-bit = 0x%X\n",high);
			rt_kprintf("Total = 0x%X\n",tmp);
		}
	}
}

/*
Cmd format:
[0] : chip number(address)
[1] : channel value
*/
static void thread_cmd_wrapper(uint16_t* cmd_buf, uint16_t* ioValue)
{
	uint16_t chn_tmp = 0, value_tmp = 0;
	if (cmd_buf[0] < 0x78 && cmd_buf[0] > 0x73)
	{
		
	}
}

static void serv_io_emergency_attach()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	/*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}



/* IIC IT functions */
/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */
	
  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_PIN_0 == GPIO_Pin)
	{
		//rt_event_send(io_cmd_event,EMERGENCY_STOP);
		rt_mb_urgent(io_data,(rt_ubase_t)io_urgent);
	}
}
#endif /*USE_EXTEND_IO*/
#ifdef HPT_V3_2
const dev_tca9539_addr tcaAddr[4] = {LL,LH,HL,HH};
dev_tca9539_addr exist_addr[4] = {};
dev_tca9539_t tca9539[4] = {};
uint16_t ioValue_buf[4] = {};
uint8_t io_chip_nbr = 0;

	
void serv_tca_io_init()
{
	rt_kprintf("Serv_tca9539 initializing!\n");
}
	
void serv_tca_hw_init(I2C_HandleTypeDef* iic_handle)
{
	dev_tca9539_t tca_tmp;
	uint8_t nbr = 0;
	tca_tmp.bus_handle = iic_handle;
	tca_tmp.cfg = 0x0000;
	tca_tmp.GPIO_OUT = 0x0000;
	for (uint8_t i = 0; i < 4; i ++)
	{
		rt_sprintf(tca_tmp.name,"exio%d",0);
		tca_tmp.bus_addr = tcaAddr[i];
		tca9539[i].bus_handle = iic_handle;
		tca9539[i].bus_addr = tcaAddr[i];
		tca9539[i].cfg = 0x0000;
		tca9539[i].GPIO_OUT = 0x0000;
		rt_sprintf(tca9539[i].name,"exio%d",i);
	}
	dev_tca9539_init(&tca9539[0],0x0000);
	dev_tca9539_init(&tca9539[3],0x0000);
	
	__HAL_I2C_ENABLE_IT(iic_handle,I2C_IT_ERR);
	#if 1
	nbr = 2;
	if (nbr != 0)
	{
		io_chip_nbr = nbr;
	}
	else
	{
		rt_kprintf("Please check line connection!\n");
	}
	#endif
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */
	rt_interrupt_enter();
  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */
	rt_interrupt_leave();
  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */
	rt_interrupt_enter();
  /* USER CODE END I2C1_ER_IRQn 0 */
//	if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_BERR))
//	{
//		__HAL_I2C_CLEAR_FLAG(&hi2c1,I2C_FLAG_BERR);
//		
//		//HAL_I2C_Init(&hi2c1);
//	}
  HAL_I2C_ER_IRQHandler(&hi2c1);
	__HAL_I2C_ENABLE_IT(&hi2c1,I2C_IT_ERR);
	//WRITE_BITS(hi2c1.Instance->CR1,15,1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */
	rt_interrupt_leave();
  /* USER CODE END I2C1_ER_IRQn 1 */
}

#if 0
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (&hi2c1 == hi2c)
	{
		if (__HAL_I2C_GET_FLAG(&hi2c1,I2C_FLAG_BERR))
		{
			__HAL_I2C_CLEAR_FLAG(&hi2c1,I2C_FLAG_BERR);
			//HAL_I2C_Init(&hi2c1);
		}
	}
}
#endif
#endif /*HPT_3_2*/



/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/