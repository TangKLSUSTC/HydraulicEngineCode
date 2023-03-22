
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     serv_ad7490_fetcher.c
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
#include "serv_ad7490_fetcher.h"
#include "rthw.h"



#if defined(USE_AD7490_FETCHER)&&defined(USE_RTT)
/*	Constants defines	*/
#define AD7490_BUF_SIZE	256
#define SAMPLE_NBR			4
#define CONT_NBR        8
/* Macro defines */

/*	Kernal Object Declaration	*/
rt_thread_t serv_ad7490_fetcher = RT_NULL;
rt_sem_t serv_ad7490_sem = RT_NULL;
#ifdef HPT_V3_1
rt_mailbox_t serv_ad_mailbox = RT_NULL;
#endif /*HPT_V3_1*/
struct rt_ringbuffer ad7490_rb;
rt_mutex_t ad7490_lock;

/*	Static Varibles Declaration	*/
uint8_t nbr_of_channel = 0, k = 0;
uint8_t	mb_index[16] = {};
double mb_value[2] = {};
#ifdef HPT_V3_1
uint16_t* mb_message = RT_NULL;
mail_msg_t ad_data_mail;
#endif /*HPT_V3_1*/
uint16_t chn_ena_msk = AD7490_CHANNELS_ENA_MSK;
uint16_t internal_buffer[16] = {};
uint16_t data_buffer[AD7490_BUF_SIZE] = {};
uint8_t	sample_nbr = 0,sample_chn = 0, sampling_entry_flag = 0;
uint16_t spi_dma_target[16];
uint8_t chn_index[16] = {};
uint16_t data_buf[16];
uint8_t data_pack[3] = {};



const static uint16_t FS1000Hz[3] = {20,249,230};
#define FS1000Hz_Profile  FS1000Hz[0],FS1000Hz[1],FS1000Hz[2]
const static uint16_t FS500Hz[3] =  {20,499,380};
#define FS500Hz_Profile  FS500Hz[0],FS500Hz[1],FS500Hz[2]
const static uint16_t FS100Hz[3] =  {20,2499,200};
#define FS100Hz_Profile  FS100Hz[0],FS100Hz[1],FS100Hz[2]
const static uint16_t FS50Hz[3] =   {41,2499,100};
#define FS50Hz_Profile  FS50Hz[0],FS50Hz[1],FS50Hz[2]
const static uint16_t FS10Hz[3] =   {209,2499,80};
#define FS10Hz_Profile  FS10Hz[0],FS10Hz[1],FS10Hz[2]
/*	Extern Varibles Reference	*/
extern dma_io_t usart1;
extern dma_io_t usart2;

/*	Common Varibles Declaration	*/
dev_ad7490_t ad7490 = {.config = \
																	{.code_type 		 = s_polar,\
																	 .operation_mode = continuous_mode,\
																	 .power_mode 		 = normal_mode,\
																	 .range_type 		 = double_range},
											.cs_gpiox 		= CS_PORT,
											.cs_gpio_pin  = CS_PIN,
											.buf					= internal_buffer,
											.state				= Initial,
											.pak = {.ADD = 15}
											};
double value_tmp[16] = {}, value_his[16] = {};
#ifdef HPT_V3_2
uint16_t mb_message[16] = {0};
uint16_t adc_os_data[16] = {0};
double adc_real_pressure[16] = {0};
#endif
TIM_HandleTypeDef htim3;
/*	Static Function Prototype	*/
static void serv_ad7490_fetcher_main(void* para);
static void ad7490_data_pack(uint8_t channel, uint16_t data, uint8_t* buf);
void serv_ad7490_cs_start();
static void serv_adc2pressure(uint16_t* adc_val_array, double* pressure_array);


void serv_ad7490_fetcher_init()
{
	serv_ad7490_sem = rt_sem_create("serv_7490_sem",0,RT_IPC_FLAG_PRIO);
	ad7490_lock = rt_mutex_create("serv_7490_lock",RT_IPC_FLAG_PRIO);
	serv_ad7490_fetcher = rt_thread_create("serv_7490_t",serv_ad7490_fetcher_main,RT_NULL,8192*2,3,5);
	
	rt_ringbuffer_init(&ad7490_rb,(uint8_t*)data_buffer,AD7490_BUF_SIZE);
	#ifdef HPT_V3_1
	serv_ad_mailbox = rt_mb_create("serv_ad_mb",64,RT_IPC_FLAG_PRIO);
	mb_message = (uint16_t*)rt_calloc(16, sizeof(uint16_t));
	ad_data_mail = (mail_msg_t)rt_malloc(sizeof(mail_msg));
	#endif /*HPT_V3_1*/
	if (RT_NULL != serv_ad7490_fetcher && RT_NULL != serv_ad7490_sem && RT_NULL != ad7490_lock)
	{
		rt_thread_startup(serv_ad7490_fetcher);
	}
	else
	{
		#if defined(USE_DMA_IO)
			rt_kprintf("Dynamic heap allocate fail!\n");
		#endif
	}
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

static void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
    //__HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

static void cs_change_pin_manual()
{
	GPIO_InitTypeDef CS_Initure = {.Mode = GPIO_MODE_OUTPUT_PP,
																 .Pull = GPIO_PULLUP,
																 .Speed = GPIO_SPEED_FREQ_LOW,
																 .Pin = CS_PIN};
	HAL_GPIO_Init(CS_PORT, &CS_Initure);
}

static void cs_change_pin_pwm()
{
	GPIO_InitTypeDef CS_Initure = {.Mode = GPIO_MODE_AF_PP,
																 .Pull = GPIO_NOPULL,
																 .Speed = GPIO_SPEED_FREQ_LOW,
																 .Pin = CS_PIN,
																 .Alternate = GPIO_AF2_TIM3};
	HAL_GPIO_Init(CS_PORT, &CS_Initure);
}

static void cs_timer_init(uint16_t psc, uint16_t arr, uint16_t ccr)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
	htim3.Instance = TIM3;
  htim3.Init.Prescaler = psc;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = arr;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = ccr;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}


void serv_ad7490_cs_start()
{
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}

void serv_ad7490_cs_stop()
{
	__HAL_TIM_DISABLE_IT(&htim3,TIM_IT_UPDATE);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
}

static void serv_ad7490_fetcher_main(void* para)
{
	rt_err_t aduwt = RT_EOK;
	#if defined(DEBUG)
	int64_t cycle;
	double time;
	#endif
	/*	Get channel enable number	*/
	if (regular_scan_mode == ad7490.config.operation_mode)
	{
		for (uint8_t i = 15; i > 0; i --)
			{
				if (READ_BITS(chn_ena_msk,i))
				{
					chn_index[nbr_of_channel] = 15 - i;
					nbr_of_channel ++;
				}
			}
		rt_kprintf("AD7490 enabled channels: ");
		for (uint8_t i = 0; i < nbr_of_channel; i ++)
		{
			rt_kprintf("%d, ",chn_index[i]);
		}
		rt_kprintf("\n");
	}
	sampling_entry_flag = 1;
	/* Start sampling */
	//HAL_SPI_Receive_DMA(ad7490.bus_handle,(uint8_t*)&ad7490.DOUT,1);
	if (READ_BITS(ad7490.DIN,15))			//clear write bit
	{
		WRITE_BITS(ad7490.DIN,15,0);
	}
	#ifdef HPT_V3_1
	ad_data_mail->data_ptr = (void*)mb_message;
	#endif /*HPT_V3_1*/
	serv_ad7490_cs_start();
	while (1)
	{			
		/* Waiting for spi dma clpt */
		aduwt = rt_sem_take(serv_ad7490_sem,RT_WAITING_FOREVER);
		if (RT_EOK == aduwt)
		{
			//rt_ringbuffer_get(&ad7490_rb,(uint8_t*)data_buf,1);
			//usart2.dma_printf(&usart2,"Ch 0 =%d\r\n",ad7490.buf[0]);
			
			//HAL_SPI_TransmitReceive_DMA(ad7490.bus_handle,(uint8_t*)&ad7490.DIN,(uint8_t*)&ad7490.DOUT,1);
			#if 1
			#if defined(DEBUG)&&0
			start_core_count();
			#endif
			switch (ad7490.config.operation_mode)
			{
				case continuous_mode:
				{
					for (uint8_t i_sample = 0; i_sample < SAMPLE_NBR; i_sample ++)
					{
						rt_ringbuffer_get(&ad7490_rb,(uint8_t*)data_buf,16);
						for (uint8_t i_channel = 0; i_channel < CONT_NBR; i_channel ++)
						{
							mb_message[i_channel] += data_buf[i_channel];
							//ad7490_data_pack(i_channel,data_buf[i_channel],data_pack);
							//usart2.dma_output(&usart2,(char*)data_pack,3);
							//usart2.dma_printf(&usart2,"Ch%d=%d\r\n",i_channel,data_buf[i_channel]);
						}
						//usart2.dma_printf(&usart2,"Ch 0 =%d\r\n",data_buf[0]);
					}
					
					for (uint8_t i_channel = 0; i_channel < CONT_NBR; i_channel ++)
					{
						mb_message[i_channel] = mb_message[i_channel]/SAMPLE_NBR;
						
						#ifdef HPT_V3_2
						adc_os_data[i_channel] = mb_message[i_channel];
						mb_message[i_channel] = 0;
						#endif /*HPT_V3_2*/
						
						//usart2.dma_printf(&usart2,"Ch%d=%d\r\n",i_channel,mb_message[i_channel]);
						//usart2.dma_printf(&usart2,"/*AT,%d,%d,%d,%d,%d,%d,%d,%d*/\r\n",adc_os_data[0],adc_os_data[1],adc_os_data[2],adc_os_data[3],adc_os_data[4],adc_os_data[5],adc_os_data[6],adc_os_data[7]);
					}
					serv_adc2pressure(adc_os_data,adc_real_pressure);
					usart2.dma_printf(&usart2,"/*AT,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f*/\r\n",\
					adc_real_pressure[0],\
					adc_real_pressure[1],\
					adc_real_pressure[2],\
					adc_real_pressure[3],\
					adc_real_pressure[4],\
					adc_real_pressure[5],\
					adc_real_pressure[6],\
					adc_real_pressure[7]);
					#ifdef HPT_V3_2
					
					#endif /*HPT_V3_2*/
					#ifdef HPT_V3_1
					ad_data_mail->size = CONT_NBR;
					rt_mb_send(serv_ad_mailbox,(rt_ubase_t)ad_data_mail);
					for (uint8_t i_channel = 0; i_channel < CONT_NBR; i_channel ++)
					{
						mb_message[i_channel] = 0;
					}
					#endif /*HPT_V3_1*/
					
					//usart2.dma_printf(&usart2,"/*AT,%d,%d,%d,%d,%d,%d,%d,%d*/\r\n",mb_message[0],mb_message[1],mb_message[2],mb_message[3],mb_message[4],mb_message[5],mb_message[6],mb_message[7]);
					break;
				}
				case regular_scan_mode:
				{
					for (uint8_t i_sample = 0; i_sample < SAMPLE_NBR; i_sample ++)
					{
						rt_ringbuffer_get(&ad7490_rb,(uint8_t*)data_buf, 16);
						for (uint8_t i_channel = 0; i_channel < nbr_of_channel; i_channel ++)
						{
							//ad7490_data_pack(chn_index[i_channel],data_buf[i_channel],data_pack);
							//usart2.dma_printf(&usart2,"Ch %d =%d\r\n",chn_index[i_channel],data_buf[i_channel]);
						}
						//usart2.dma_printf(&usart2,"Ch 0 =%d\r\n",ad7490.buf[0]);
					}
					break;
				}
				case independent_mode:
				default:
				{
					/* Do nothing */
					break;
				}
			}
			#if defined(DEBUG)&&0
			cycle = stop_core_count();
			time = cycle/84;
			usart1.dma_printf(&usart1,"Time used %.2fns\r\n",time);
			#endif
			#endif
		}
		//rt_thread_mdelay(5);
	}
}

/*	This function is called by BSP initialization stage	*/
int serv_ad7490_hw_init(SPI_HandleTypeDef* spihandle)
{
	/*	First init CS pin	*/
	GPIO_InitTypeDef CS_Initure = {.Mode = GPIO_MODE_OUTPUT_PP,
																 .Pull = GPIO_PULLUP,
																 .Speed = GPIO_SPEED_FREQ_LOW,
																 .Pin = CS_PIN};
	HAL_GPIO_Init(CS_PORT, &CS_Initure);
																 //Todo : Init ad7490
	ad7490.bus_handle = spihandle;
	rt_sprintf(ad7490.name,"ad7490");
	if (continuous_mode == ad7490.config.operation_mode)
	{
		ad7490.pak.ADD = CONT_NBR;
	}
	else if (regular_scan_mode == ad7490.config.operation_mode)
	{
		ad7490.pak.ADD = 15;
	}
	else if (independent_mode == ad7490.config.operation_mode)
	{
		ad7490.pak.ADD = 0x00;
	}
	//rt_base_t lock = rt_hw_interrupt_disable();
	dev_ad7490_init(&ad7490);
	WRITE_BITS(ad7490.DIN,11,0);
	//rt_hw_interrupt_enable(lock);
	rt_thread_mdelay(5);
	#if AD7490_SAMPLE_RATE == FS_1000HZ
	cs_timer_init(FS1000Hz_Profile);
	rt_kprintf("AD7490 Fs is set to 1000Hz as default\n");
	#endif /* (AD7490_SAMPLE_RATE == FS_1000HZ) */
	#if AD7490_SAMPLE_RATE == FS_500HZ
	cs_timer_init(FS500Hz_Profile);
	rt_kprintf("AD7490 Fs is set to 500Hz as default\n");
	#endif /* (AD7490_SAMPLE_RATE == FS_500HZ) */
	#if AD7490_SAMPLE_RATE == FS_100HZ
	cs_timer_init(FS100Hz_Profile);
	rt_kprintf("AD7490 Fs is set to 100Hz as default\n");
	#endif /* (AD7490_SAMPLE_RATE == FS_100HZ) */
	#if AD7490_SAMPLE_RATE == FS50HZ
	cs_timer_init(FS50Hz_Profile);
	rt_kprintf("AD7490 Fs is set to 50Hz as default\n");
	#endif /* (AD7490_SAMPLE_RATE == FS_50HZ) */
	#if AD7490_SAMPLE_RATE == FS10HZ
	cs_timer_init(FS50Hz_Profile);
	rt_kprintf("AD7490 Fs is set to 10Hz as default\n");
	#endif /* (AD7490_SAMPLE_RATE == FS_10HZ) */
	return 0;
}


/*
		Calculate real pressure values
*/
extern double voltage_zero_point[16];
static void serv_adc2pressure(uint16_t* adc_val_array, double* pressure_array)
{
	double voltage_tmp;
	for (uint8_t i = 0; i < CONT_NBR; i ++)
	{
		voltage_tmp = 0.00122*adc_val_array[i] - voltage_zero_point[i];
		pressure_array[i] = 124*voltage_tmp;
	}
}

/*****************************************************************/

												/* IT functions */

/*****************************************************************/
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (SPI1 == hspi->Instance&&1 == sampling_entry_flag)
	{
		
		dev_ad7490_decode_data(&ad7490);
		sample_chn ++;
		if (16 == sample_chn)
		{
			sample_chn = 0;
			rt_ringbuffer_put(&ad7490_rb,(uint8_t*)internal_buffer,16);
			sample_nbr ++;
			if (SAMPLE_NBR == sample_nbr)
			{
				sample_nbr = 0;
				rt_sem_release(serv_ad7490_sem);
			}
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (SPI1 == hspi->Instance&&1 == sampling_entry_flag)
	{
		
		dev_ad7490_decode_data(&ad7490);
		sample_chn ++;
		if (16 == sample_chn)
		{
			sample_chn = 0;
			rt_ringbuffer_put(&ad7490_rb,(uint8_t*)internal_buffer,16);
			sample_nbr ++;
			if (SAMPLE_NBR == sample_nbr)
			{
				sample_nbr = 0;
				rt_sem_release(serv_ad7490_sem);
			}
		}
	}
}

static void ad7490_data_pack(uint8_t channel, uint16_t data, uint8_t* buf)
{
	buf[0] = channel;
	buf[1] = (uint8_t)(data&0x00ff);			/* lower 8 bits */
	buf[2] = (uint8_t)(data&0xff00)>>8;		/* uper 8 bits */
}

uint8_t icc = 0;
void TIM3_IRQHandler(void)
{
	rt_interrupt_enter();
  /* USER CODE BEGIN TIM3_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim3,TIM_IT_UPDATE) != RESET)
	{
		if (continuous_mode == ad7490.config.operation_mode)
		{
			HAL_SPI_TransmitReceive_IT(ad7490.bus_handle,(uint8_t*)&ad7490.DIN,(uint8_t*)&ad7490.DOUT,1);
			//HAL_SPI_Receive_IT(ad7490.bus_handle,(uint8_t*)&ad7490.DOUT,1);
			__HAL_TIM_CLEAR_FLAG(&htim3,TIM_IT_UPDATE);
		}
		if (regular_scan_mode == ad7490.config.operation_mode)
		{
			if (READ_BITS(chn_ena_msk,icc))
			{
				HAL_SPI_TransmitReceive_IT(ad7490.bus_handle,(uint8_t*)&ad7490.DIN,(uint8_t*)&ad7490.DOUT,1);
			__HAL_TIM_CLEAR_FLAG(&htim3,TIM_IT_UPDATE);
			}
			icc ++;
			if (icc == 16)icc = 0;
		}
	}
	rt_interrupt_leave();
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (TIM1 == htim->Instance) 
	{
    HAL_IncTick();
  }
}

/* External Commands */
void serv_set_sample_rate(int argc, char** argv)
{
	if (argc != 2)
	{
		rt_kprintf("Please check parameter!\n");
		rt_kprintf("The right command is \n");
		rt_kprintf("ad7490_set_fs <1000Hz|500Hz|100Hz|10Hz>\n");
	}
	if (argc == 2)
	{
		/* Set ad7490 sample rate to 1000Hz */
		if (!rt_strcmp(argv[1],"1000Hz"))
		{
			/* Stop sampling */
			serv_ad7490_cs_stop();
			HAL_SPI_Abort_IT(ad7490.bus_handle);
			
			/* Update timer configure */
			cs_timer_init(FS1000Hz[0],FS1000Hz[1],FS1000Hz[2]);
			WRITE_BITS(ad7490.DIN,11,0);
			serv_ad7490_cs_start();
			rt_kprintf("AD7490 Fs is set to 1000Hz\n");
			return;
		}
		/* Set ad7490 sample rate to 500Hz */
		if (!rt_strcmp(argv[1],"500Hz"))
		{
			/* Stop sampling */
			serv_ad7490_cs_stop();
			HAL_SPI_Abort_IT(ad7490.bus_handle);
			
			/* Update timer configure */
			cs_timer_init(FS500Hz[0],FS500Hz[1],FS500Hz[2]);
			WRITE_BITS(ad7490.DIN,11,0);
			serv_ad7490_cs_start();
			rt_kprintf("AD7490 Fs is set to 500Hz\n");
			return;
		}
		if (!rt_strcmp(argv[1],"100Hz"))
		{
			/* Stop sampling */
			serv_ad7490_cs_stop();
			HAL_SPI_Abort_IT(ad7490.bus_handle);
			
			/* Update timer configure */
			cs_timer_init(FS100Hz[0],FS100Hz[1],FS100Hz[2]);
			WRITE_BITS(ad7490.DIN,11,0);
			serv_ad7490_cs_start();
			rt_kprintf("AD7490 Fs is set to 100Hz\n");
			return;
		}
		if (!rt_strcmp(argv[1],"50Hz"))
		{
			/* Stop sampling */
			serv_ad7490_cs_stop();
			HAL_SPI_Abort_IT(ad7490.bus_handle);
			
			/* Update timer configure */
			cs_timer_init(FS50Hz[0],FS50Hz[1],FS50Hz[2]);
			WRITE_BITS(ad7490.DIN,11,0);
			serv_ad7490_cs_start();
			rt_kprintf("AD7490 Fs is set to 50Hz\n");
			return;
		}
		if (!rt_strcmp(argv[1],"10Hz"))
		{
			/* Stop sampling */
			serv_ad7490_cs_stop();
			HAL_SPI_Abort_IT(ad7490.bus_handle);
			
			/* Update timer configure */
			cs_timer_init(FS10Hz[0],FS10Hz[1],FS10Hz[2]);
			WRITE_BITS(ad7490.DIN,11,0);
			serv_ad7490_cs_start();
			rt_kprintf("AD7490 Fs is set to 10Hz\n");
			return;
		}
	}
}
MSH_CMD_EXPORT_ALIAS(serv_set_sample_rate,ad7490_set_fs,ad7490_set_fs <1000Hz|500Hz|100Hz>);

void serv_set_sample_mode(int argc, char** argv)
{
	if (argc < 2)
	{
		goto __err_exit;
	}
	if (argc == 2)
	{
		if (!rt_strcmp(argv[1],"cont"))
		{
			ad7490.config.operation_mode = continuous_mode;
			serv_ad7490_cs_stop();
			HAL_SPI_Abort_IT(ad7490.bus_handle);
			sampling_entry_flag = 0;
			cs_change_pin_manual();
			dev_ad7490_control(&ad7490,AD_7490_SET_OPT_MODE,(void*)ad7490.config.operation_mode);
			sampling_entry_flag = 1;
			cs_change_pin_pwm();
			serv_ad7490_cs_start();
			rt_kprintf("AD7490 mode is set to continuous mode\n");
		}
		else
		{
			goto __err_exit;
		}
	}
	if (argc > 2)
	{
		if (!rt_strcmp(argv[1],"scan"))
		{
			if (argc > 18) goto __err_exit;
			uint8_t chntmp = 0, type = 0;
			uint16_t shadow_tmp = 0;
			for (uint8_t i = 2; i < argc; i ++)
			{
				chntmp = (uint8_t)atoi(argv[i]);
				WRITE_BITS(shadow_tmp,(15-chntmp),1);
			}
			if (shadow_tmp == 0)
			{
				goto __exception_exit;
			}
			chn_ena_msk = shadow_tmp;
			nbr_of_channel = 0;
			for (uint8_t i = 15; i > 0; i --)
			{
				if (READ_BITS(chn_ena_msk,i))
				{
					chn_index[nbr_of_channel] = 15 - i;
					nbr_of_channel ++;
				}
			}
			rt_kprintf("AD7490 enabled channels: ");
			for (uint8_t i = 0; i < nbr_of_channel; i ++)
			{
				rt_kprintf("%d, ",chn_index[i]);
			}
			rt_kprintf("\n");
			ad7490.DOUT = shadow_tmp;
			ad7490.config.operation_mode = regular_scan_mode;
			serv_ad7490_cs_stop();
			HAL_SPI_Abort_IT(ad7490.bus_handle);
			sampling_entry_flag = 0;
			cs_change_pin_manual();
			dev_ad7490_control(&ad7490,AD_7490_SET_OPT_MODE,(void*)ad7490.config.operation_mode);
			sampling_entry_flag = 1;
			cs_change_pin_pwm();
			serv_ad7490_cs_start();
			rt_kprintf("AD7490 mode is set to regular scan mode\n");
			sample_nbr = 0;
			sample_chn = 0;
			return;
		}
		return;
	}
	__err_exit:
	{
		rt_kprintf("Please check parameter!\n");
		rt_kprintf("The right command is \n");
		rt_kprintf("ad7490_set_mode <cont|scan,|1,2,3...>\n");
		return;
	}
	__exception_exit:
	{
		rt_kprintf("Regular scan mode set error!\n");
		rt_kprintf("Quit setting!\n");
		return;
	}
}
MSH_CMD_EXPORT_ALIAS(serv_set_sample_mode,ad7490_set_mode,ad7490_set_mode <cont|scan>);
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/