
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     usart_dma_io.c
* @Author       Thompson
* @Version         
* @Date         2021.11.24
* @Note            
* @Copyright    Biorobotics & Control Lab
************************************************************************/

/*************************************************************************
* @Function

* @ChangeLog
    Date               Author               Notes
    2021.11.24         Thompson             Create this file, first version
************************************************************************/
#include "usart_dma_io.h"





/*	Static Varibles Declaration	*/
char buffer[128] = "";

/*	Extern Varibles Reference	*/


/*	Common Varibles Declaration	*/


/*	Static Function Prototype	*/




//dmap_t dmap;
//double time_quantum = 0;
uint32_t delay_t = 0;

/*	Static Function Prototype	*/

void dma_io_construct(dma_io_t* obj)
{
	/*	Check if DMA is enabled	*/
	obj->RX_status = (obj->dma_rx_handle == NULL)?DMA_ERR:DMA_RX_IDLE;
	obj->TX_status = (obj->dma_tx_handle == NULL)?DMA_ERR:DMA_TX_IDLE;
	if (obj->TX_status == DMA_ERR && obj->RX_status == DMA_ERR)
	{
		return;
	}
	/*	Set USART IT	*/
	__HAL_UART_ENABLE_IT(obj->uart_handle,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(obj->uart_handle,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(obj->uart_handle,UART_IT_TC);

	obj->dma_tc_flag = __HAL_DMA_GET_TC_FLAG_INDEX(obj->dma_tx_handle);
	obj->time_quantum = 9000000/obj->uart_handle->Init.BaudRate;
	obj->dma_rx_length = 0;
	obj->dma_printf = dma_printf;
	obj->dma_output = dma_output;
	obj->dma_echo = dma_io_echo;
	obj->dma_getRecLength = dma_io_get_reclength;
	obj->usart_set_RX_IT = dma_io_setRX_IT;
	#if defined(USE_RTT)&&defined(RT_USING_CONSOLE)
	rt_ringbuffer_init(&obj->rx_ring_buf,obj->rx_poll_ring,DMA_IO_BUF_MAX);
	rt_sem_init(&obj->shell_rx_sem,"shell_sem",0,RT_IPC_FLAG_PRIO);
	#endif
	#if defined(USE_RTT)
	
	#endif
}


//void dma_printf(const char* format,...)
//{
//	va_list args;
//	int count = 0;
//	char* str = NULL;
//	char buffer[128] = "";
//	va_start(args,format);
//	count = vsprintf(buffer, format, args);
//	dmap.dma_output(buffer, count);
//	va_end(args);
//}

void dma_printf(dma_io_t* obj, const char* format, ...)
{
	va_list args;
	int count = 0;
	while (obj->uart_handle->gState != HAL_UART_STATE_READY || obj->dma_tx_handle->State == HAL_DMA_STATE_BUSY);
	va_start(args,format);
//	#if defined(USE_RTT)
//	count = rt_vsprintf(buffer, format, args);
//	#else
//	count = vsprintf(buffer, format, args);
//	#endif
	count = vsprintf((char*)obj->dma_tx_buf, format, args);
	obj->dma_output(obj, (char*)obj->dma_tx_buf, count);
	va_end(args);
}

__WEAK void dma_output(dma_io_t* obj, const char* str, int len)
{
	/*	Check DMA transmit is completed or not	*/
	//while (obj->uart_handle->gState != HAL_UART_STATE_READY);
	__HAL_DMA_DISABLE(obj->dma_tx_handle);
	if (HAL_UART_Transmit_DMA(obj->uart_handle,(uint8_t*)str,len) != HAL_OK)
	{
		// TODO : Exception handling
	}
}

/*	In order to keep system Real-time, you shoule not use rx function in 
		blocking mode with waiting for DMA_RX_IDLE flag. You should use semphore or 
		event to wake up thread to handle the received data.
*/
__WEAK void dma_input(dma_io_t* obj, uint8_t *buffer, uint16_t *size)	
{
	if (obj->RX_status != DMA_RX_BUSY || obj->RX_status == DMA_RX_CPLT)
	{
		/*	Waiting for usart complete	*/
		while (obj->RX_status == DMA_RX_CPLT);\
		/*	Copy rx data to external buffer	*/
		for (uint8_t i = 0; i < obj->dma_rx_length; i++)
		{
			buffer[i] = obj->dma_rx_buf[i];
		}
		*size = obj->dma_rx_length;
		obj->RX_status = DMA_RX_IDLE;
	}
}

void dma_io_echo(dma_io_t* obj)
{
	if (DMA_RX_CPLT == obj->RX_status)
	{
		obj->dma_output(obj, (char*)obj->dma_rx_buf,obj->dma_rx_length);
		obj->RX_status = DMA_RX_IDLE;
	}
}


/**
 * @note		Get DMA input data length
 * @param		dma_io standard object
 *
 * @return	data length
*/
uint16_t dma_io_get_reclength(dma_io_t* obj)
{
	return (DMA_IO_BUF_MAX - __HAL_DMA_GET_COUNTER(obj->dma_rx_handle));
}

/**
 * @note		Enable RX interrupt
 * @param		dma_io standard object
 *
 * @return	None
*/
void dma_io_setRX_IT(dma_io_t* obj)
{
	__HAL_UART_ENABLE_IT(obj->uart_handle,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(obj->uart_handle,UART_IT_IDLE);
}

/*		dma io output and receive function rely on DMA TC and	USART Idle interrupt	*/
/*		You need to copy dma and usart it handler function from stm32fxxx_it.c to
			where you declare dma_io handler																						*/




/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/