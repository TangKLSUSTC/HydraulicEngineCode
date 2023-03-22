
/*************************************************************************
*                            BCL
*                   SUSTech Robotic Institute
*
* @FileName     usart_dma_io.h
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
#ifndef __USART_DMA_IO_H
#define __USART_DMA_IO_H


#ifdef __cplusplus
extern "C" {
#endif
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <core_delay.h>
#include <proj_config.h>
#if defined(USE_RTT)&&defined(RT_USING_CONSOLE)
#include <ringbuffer.h>
#endif
/*	Please add your includes here	*/



/*	Please add your macro here	*/
#define DMA_IO_BUF_MAX	128


/*	Please add your varibles extern here	*/



//void dma_printf(const char* format,...);

typedef void (*dma_p)(const char* format,...);
typedef void (*dma_out)(const char* str, int len);

typedef enum _dma_io_status:uint8_t
{
	DMA_TX_BUSY,
	DMA_RX_BUSY,
	DMA_TX_IDLE,
	DMA_RX_IDLE,
	DMA_RX_CPLT,
	DMA_ERR
}dma_io_status;


typedef struct __attribute__((aligned (1))) __dma_io
{
	UART_HandleTypeDef* 	uart_handle;
	DMA_HandleTypeDef* 		dma_tx_handle;
	DMA_HandleTypeDef*		dma_rx_handle;
	#if defined(USE_RTT)&&defined(RT_USING_CONSOLE)
	struct rt_ringbuffer	rx_ring_buf;
	uint8_t 							rx_poll_ring[DMA_IO_BUF_MAX];
	struct rt_semaphore 	shell_rx_sem;
	#endif
	#if defined(USE_RTT)
	
	#endif
	uint32_t 							dma_tc_flag;
	float 								time_quantum;
	uint8_t 							dma_rx_buf[DMA_IO_BUF_MAX];
	uint8_t								dma_tx_buf[DMA_IO_BUF_MAX];
	uint16_t							dma_rx_length;
	dma_io_status					TX_status;
	dma_io_status					RX_status;
	void (*dma_io_construct)(struct __dma_io* obj);
	void (*dma_output)(struct __dma_io* obj, const char* str, int len);
	void (*dma_printf)(struct __dma_io* obj, const char* format, ...);
	void (*dma_echo)(struct __dma_io* obj);
	uint16_t (*dma_getRecLength)(struct __dma_io* obj);
	void (*usart_set_RX_IT)(struct __dma_io* obj);
}dma_io_t;




void dma_io_init(dma_io_t* hdma_io);
void dma_io_construct(dma_io_t* obj);
void dma_output(dma_io_t* obj, const char* str, int len);
void dma_printf(struct __dma_io* obj, const char* format, ...);
void dma_input(dma_io_t* obj, uint8_t *buffer, uint16_t *size);
void dma_io_echo(dma_io_t* obj);
uint16_t dma_io_get_reclength(dma_io_t* obj);
void dma_io_setRX_IT(dma_io_t* obj);

#ifdef __cplusplus
}
#endif
#endif
/******************************END OF FILE******************************/
/**************************(C) COPYRIGHT BCL****************************/