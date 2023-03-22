
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     dma_io_declaration.c
* @Author       Thompson
* @Version         
* @Date         2021.11.25
* @Note            
* @Copyright    Biorobotics & Control Lab
************************************************************************/

/*************************************************************************
* @Function

* @ChangeLog
    Date               Author               Notes
    2021.11.25         Thompson             Create this file, first version
************************************************************************/
#include "dma_io_declaration.h"


#if defined(USE_RTT)
	#include <rtthread.h>
	#include <rtconfig.h>
#endif

/*	Static Varibles Declaration	*/
#if defined(USE_RTT)&&defined(RT_USING_CONSOLE)

#endif
/*	Extern Varibles Reference	*/


/*	Common Varibles Declaration	*/
/*	You need to write your used dma_io handler here	*/
#if defined(BSP_USING_UART1)
	extern dma_io_t usart1;
#endif
#if defined(BSP_USING_UART2)
	extern dma_io_t usart2;
#endif
#if defined(BSP_USING_UART3)
	extern dma_io_t usart3;
#endif
#if defined(BSP_USING_UART6)
	extern dma_io_t usart6;
#endif
/*	Static Function Prototype	*/


/*	Name defines	*/
#define RT_SHELL_PORT	usart1

#if defined(DMA_IO_USE_RX)
#if defined(BSP_USING_UART1)
void USART1_IRQHandler(void)
{
	#if defined(USE_RTT)
	rt_interrupt_enter();
	#endif
	if (__HAL_UART_GET_FLAG(usart1.uart_handle,UART_FLAG_RXNE) != RESET)
	{
		usart1.RX_status = DMA_RX_BUSY;
		usart1.dma_rx_buf[0] = usart1.uart_handle->Instance->DR;
		HAL_UARTEx_ReceiveToIdle_DMA(usart1.uart_handle,usart1.dma_rx_buf + 1,DMA_IO_BUF_MAX - 1);
	}
	HAL_UART_IRQHandler(usart1.uart_handle);
	#if defined(USE_RTT)
	rt_interrupt_leave();
	#endif
}
#endif
#if defined(BSP_USING_UART2)
void USART2_IRQHandler(void)
{
	#if defined(USE_RTT)
	rt_interrupt_enter();
	#endif
	if (__HAL_UART_GET_FLAG(usart2.uart_handle,UART_FLAG_RXNE) != RESET)
	{
		usart2.RX_status = DMA_RX_BUSY;
		usart2.dma_rx_buf[0] = usart2.uart_handle->Instance->DR;
		HAL_UARTEx_ReceiveToIdle_DMA(usart2.uart_handle,usart2.dma_rx_buf + 1,DMA_IO_BUF_MAX - 1);
	}
  HAL_UART_IRQHandler(usart2.uart_handle);
	#if defined(USE_RTT)
	rt_interrupt_leave();
	#endif
}
#endif
#if defined(BSP_USING_UART3)
void USART3_IRQHandler(void)
{
	#if defined(USE_RTT)
	rt_interrupt_enter();
	#endif
	if (__HAL_UART_GET_FLAG(usart3.uart_handle,UART_FLAG_RXNE) != RESET)
	{
		usart3.RX_status = DMA_RX_BUSY;
		usart3.dma_rx_buf[0] = usart3.uart_handle->Instance->DR;
		HAL_UARTEx_ReceiveToIdle_DMA(usart3.uart_handle,usart3.dma_rx_buf + 1,DMA_IO_BUF_MAX - 1);
	}
  HAL_UART_IRQHandler(usart3.uart_handle);
	#if defined(USE_RTT)
	rt_interrupt_leave();
	#endif
}
#endif
#if defined(BSP_USING_UART6)
void USART6_IRQHandler(void)
{
	#if defined(USE_RTT)
	rt_interrupt_enter();
	#endif
	if (__HAL_UART_GET_FLAG(usart6.uart_handle,UART_FLAG_RXNE) != RESET)
	{
		usart6.RX_status = DMA_RX_BUSY;
		usart6.dma_rx_buf[0] = usart6.uart_handle->Instance->DR;
		HAL_UARTEx_ReceiveToIdle_DMA(usart6.uart_handle,usart6.dma_rx_buf + 1,DMA_IO_BUF_MAX - 1);
	}
  HAL_UART_IRQHandler(usart6.uart_handle);
	#if defined(USE_RTT)
	rt_interrupt_leave();
	#endif
}
#endif

#endif
//{
//	/*
//	If USART has an idle interrupt, it means USART receiving is done.
//	You need to complete this turn of receiving and compute data length.
//	*/
//	//HAL_UART_IRQHandler(RT_SHELL_PORT.uart_handle);
//	if (__HAL_UART_GET_FLAG(RT_SHELL_PORT.uart_handle,UART_FLAG_TC) != RESET)
//	{
//		RT_SHELL_PORT.TX_status = DMA_TX_IDLE;
//		__HAL_UART_CLEAR_FLAG(RT_SHELL_PORT.uart_handle, UART_FLAG_TC);
//		RT_SHELL_PORT.uart_handle->gState = HAL_UART_STATE_READY;
//		__HAL_UART_DISABLE_IT(RT_SHELL_PORT.uart_handle, UART_IT_TC);
//	}
//	
//  if (__HAL_UART_GET_FLAG(RT_SHELL_PORT.uart_handle,UART_FLAG_IDLE) != RESET)
//	{
//		/*	Clear USART IDLE IT flag	*/
//		__HAL_UART_CLEAR_IDLEFLAG(RT_SHELL_PORT.uart_handle);
//		/*	Disbale the DMA IO stream and reset data length	*/
//		//__HAL_DMA_DISABLE(RT_SHELL_PORT.dma_rx_handle);
//		/*	Stop USART DMA receiving	*/
//		HAL_UART_AbortReceive(RT_SHELL_PORT.uart_handle);
//		RT_SHELL_PORT.dma_rx_length = RT_SHELL_PORT.dma_getRecLength(&RT_SHELL_PORT);
//		__HAL_DMA_SET_COUNTER(RT_SHELL_PORT.dma_rx_handle, DMA_IO_BUF_MAX);
//		RT_SHELL_PORT.usart_set_RX_IT(&RT_SHELL_PORT);
//		RT_SHELL_PORT.RX_status = DMA_RX_IDLE;
//		UART_Start_Receive_DMA(RT_SHELL_PORT.uart_handle,RT_SHELL_PORT.dma_rx_buf,DMA_IO_BUF_MAX);
//	}
//	
//	/*
//	If USART has an RXNE interrupt, it means USART receives data but no peripheral
//	reads it.
//	Start USART DMA receive
//	*/
//	if (__HAL_UART_GET_FLAG(RT_SHELL_PORT.uart_handle,UART_FLAG_RXNE) != RESET)
//	{
//		RT_SHELL_PORT.RX_status = DMA_RX_BUSY;
//		UART_Start_Receive_DMA(RT_SHELL_PORT.uart_handle,RT_SHELL_PORT.dma_rx_buf,DMA_IO_BUF_MAX);
//	}
//}


/*	DMA interrupt is used to handle DMA transfer complete event	*/
//void DMA2_Stream7_IRQHandler(void)
//{
//	if (__HAL_DMA_GET_FLAG(RT_SHELL_PORT.dma_tx_handle, RT_SHELL_PORT.dma_tc_flag) != RESET)
//	{
//		__HAL_DMA_CLEAR_FLAG(RT_SHELL_PORT.dma_tx_handle, RT_SHELL_PORT.dma_tc_flag);
//		__HAL_DMA_CLEAR_FLAG(RT_SHELL_PORT.dma_tx_handle, __HAL_DMA_GET_HT_FLAG_INDEX(RT_SHELL_PORT.dma_tx_handle));
//		RT_SHELL_PORT.TX_status = DMA_TX_IDLE;
//		//RT_SHELL_PORT.uart_handle->gState = HAL_UART_STATE_READY;
//		RT_SHELL_PORT.dma_tx_handle->State = HAL_DMA_STATE_READY;
//		__HAL_DMA_DISABLE_IT(RT_SHELL_PORT.dma_tx_handle,DMA_IT_HT|DMA_IT_DME|DMA_IT_FE|DMA_IT_TE|DMA_IT_TC);
//	}
//	
//}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (RT_SHELL_PORT.uart_handle == huart)
//	{
//		RT_SHELL_PORT.TX_status = DMA_TX_IDLE;
//	}
//}

extern rt_sem_t usart_cmd_sem;
extern rt_event_t  io_cmd_event;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (RT_SHELL_PORT.uart_handle == huart)
	{
		RT_SHELL_PORT.dma_rx_length = Size + 1;
		RT_SHELL_PORT.RX_status = DMA_RX_CPLT;
		rt_ringbuffer_put(&RT_SHELL_PORT.rx_ring_buf,RT_SHELL_PORT.dma_rx_buf,RT_SHELL_PORT.dma_rx_length);
		RT_SHELL_PORT.shell_rx_sem.value = Size;
		rt_sem_release(&RT_SHELL_PORT.shell_rx_sem);
	}
	if (usart2.uart_handle == huart)
	{
		usart2.dma_rx_length = Size + 1;
		usart2.RX_status = DMA_RX_CPLT;
		//rt_sem_release(usart_cmd_sem);
		//rt_event_send(io_cmd_event,THREAD_CMD);
	}
	if (usart6.uart_handle == huart)
	{
		usart6.dma_rx_length = Size + 1;
		usart6.RX_status = DMA_RX_CPLT;
	}
}

#if defined(USE_RTT)&&defined(RT_USING_CONSOLE)
/*	Define rt-thread console output function and input function	*/
static char strtmp[DMA_IO_BUF_MAX];
void rt_console_output_block(const char *str);
void rt_hw_console_output(const char *str)
{
	rt_size_t i = 0,size = 0;
	__HAL_UNLOCK(RT_SHELL_PORT.uart_handle);
	/*	Get overall interrupt state, 1 means only allow NMI and the hard fault exception,
			0 means no masking is set.
	*/
	i = __get_PRIMASK();
	if (1 == i)
	{
		//while (RT_SHELL_PORT.uart_handle->gState != HAL_UART_STATE_READY);
//		RT_SHELL_PORT.uart_handle->gState = HAL_UART_STATE_READY;
//		size = rt_sprintf(strtmp,"%s",str);
//		if ('\n' == strtmp[size - 1])
//		{
//			strtmp[size - 1] = '\r';
//			strtmp[size] = '\n';
//			size ++;
//		}
//		HAL_UART_Transmit(RT_SHELL_PORT.uart_handle,(uint8_t*)strtmp,size,10);
//		i = __get_PRIMASK();
//		if (1 == i)
//		{
//			RT_SHELL_PORT.uart_handle->gState = HAL_UART_STATE_READY;
//		}
		RT_SHELL_PORT.uart_handle->gState = HAL_UART_STATE_READY;
		rt_console_output_block(str);
	}
	else
	{
		while (RT_SHELL_PORT.uart_handle->gState != HAL_UART_STATE_READY || RT_SHELL_PORT.dma_tx_handle->State == HAL_DMA_STATE_BUSY);
		size = rt_sprintf(strtmp,"%s",str);
		if ('\n' == strtmp[size - 1])
		{
			strtmp[size - 1] = '\r';
			strtmp[size] = '\n';
			size ++;
		}
		__HAL_DMA_DISABLE(RT_SHELL_PORT.dma_tx_handle);
		HAL_UART_Transmit_DMA(RT_SHELL_PORT.uart_handle,(uint8_t*)strtmp,size);
	}
}

char rt_hw_console_getchar(void)
{
	int ch = -1;
	while (rt_ringbuffer_getchar(&RT_SHELL_PORT.rx_ring_buf, (rt_uint8_t*)&ch) != 1)
	{
		rt_sem_take(&RT_SHELL_PORT.shell_rx_sem, RT_WAITING_FOREVER);
	} 
  return ch; 
}

void rt_console_output_block(const char *str)
{
	  rt_size_t i = 0, size = 0;
    char a = '\r';

    __HAL_UNLOCK(RT_SHELL_PORT.uart_handle);

    size = rt_strlen(str);

    for (i = 0; i < size; i++)
    {
        if (*(str + i) == '\n')
        {
            HAL_UART_Transmit(RT_SHELL_PORT.uart_handle, (uint8_t *)&a, 1, 1);
        }
        HAL_UART_Transmit(RT_SHELL_PORT.uart_handle, (uint8_t *)(str + i), 1, 1);
    }
}
#endif

/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/