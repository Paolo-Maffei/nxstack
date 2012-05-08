/**
 *
 * \file bus.c
 * \brief micro.bus controls.
 *
 *  Micro bus: mode control and support functions.
 *  General support functions and hardware initialization.
 *   
 *	
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <signal.h>
#include <string.h>

#include "debug.h"
#include "bus.h"

#ifdef HAVE_POWERSAVE
#include "powersave.h"
#endif

#ifndef BUS_UART_BUFLEN
#define BUS_UART_BUFLEN 16
#endif

#ifndef portACLK_FREQUENCY_HZ
#define portACLK_FREQUENCY_HZ ( ( unsigned portLONG ) 32768 )
#endif

void bus_spi_init();

void bus_uart_init();


/**
 * Global bus lock.
 *
 */
 
xSemaphoreHandle bus_lock = NULL;

#ifdef HAVE_POWERSAVE
/**
 * Powersaving mode control.
 *
 */
xPowerHandle bus_ph = 0;
#endif

uint8_t bus_serial[4] = { 0x01, 0x02, 0x03, 0x04 };

/**
 * Initialize bus and MCU clock. 
 * First function to call.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	semaphore creation failed
 */
portCHAR bus_init(void)
{
	vSemaphoreCreateBinary( bus_lock );
	bus_spi_init(BUS_STE + BUS_PHASE_INVERT);
	bus_uart_init();
	return pdTRUE;
}

/**
 * Bus select.
 *
 * \param id module ID;
 * \param mode communication mode
 * \param flags mode specific flags
 *
 * \return pdTRUE
 * \return pdFALSE	bus reserved
 */
portCHAR bus_select()
{
    //multitask env drivers need protection
	if( xSemaphoreTake( bus_lock, ( portTickType ) 0 ) == pdTRUE )
	{	/*bus free, do select*/
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}	


/**
 * Free bus.
 *
 * \return pdTRUE
 */

portCHAR bus_free(void)
{
    xSemaphoreGive( bus_lock ); /*free lock*/
    return pdTRUE;
}

/**
 * Bus SPI init.
 *
 * \param flags SPI mode flags
 *
 * \return pdTRUE
 * \return pdFALSE	bus reserved
 */
 
void bus_spi_init()
{
  SPI_InitTypeDef   SPI_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Configure SPI_MASTER pins: SCK and MOSI ---------------------------------*/
  /* Configure SCK and MOSI pins as Alternate Function Push Pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* NSS GPIO configuration ------------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /*MISO-----------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* SPI Config -------------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7; 
  SPI_Init(SPI1, &SPI_InitStructure);
  
  SPI_Cmd(SPI1, ENABLE);
}

/**
 * Bus SPI exchange.
 *
 * \param out byte to transmit
 *
 * \return byte from SPI
 */
uint8_t bus_spi_exchange(uint8_t out)
{
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1,(uint16_t)out);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  return (char)SPI_I2S_ReceiveData(SPI1);
}

#if 0
/**
 * Bus parallel init.
 *
 */
void bus_par_init(void)
{
	P3SEL &= ~0x3F; /*GPIO*/
	P3OUT |= 0x0F;
	P3DIR |= 0x0F;	/*control pins out*/
	P3DIR &= ~0x30; /*UART pins in*/
	P4DIR = 0x00;		/*data bus in*/
}
#endif 

/** Interrupt service routines. */
//interrupt (UART0RX_VECTOR) bus_rxISR( void );
//interrupt (UART0TX_VECTOR) bus_txISR( void );

/** The queues for UART  */
static xQueueHandle bus_rx = 0; 
static xQueueHandle bus_tx = 0; 

static volatile portCHAR bus_txempty;

/**
 * Bus UART init.
 *
 * \param flags UART mode flags
 *
 * \return SUCCESS
 * \return FAILURE	bus reserved
 */
 
void bus_uart_init()
{
	
	if (bus_rx == 0)
	{
		/* Create the queues used by the com test task. */
		bus_rx = xQueueCreate( BUS_UART_BUFLEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
		bus_tx = xQueueCreate( BUS_UART_BUFLEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
	}
	/* Private variables ---------------------------------------------------------*/
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	portENTER_CRITICAL();
	/* Enable GPIOA, GPIOD and USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1  , ENABLE);

	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USARTx configured as follow:
	   - BaudRate = 115200 baud  
	   - Word Length = 8 Bits
	   - One Stop Bit
	   - No parity
	   - Hardware flow control disabled (RTS and CTS signals)
	   - Receive and transmit enabled
	   */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	/* Enable USART1 Receive and Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	USART_Cmd(USART1,ENABLE);
		
	/* Nothing in the buffer yet. */
	bus_txempty = pdTRUE;
	
	portEXIT_CRITICAL();

	return;
}

/**
 * Bus UART send.
 *
 * \param byte data to be sent
 * \param blocktime time to block
 *
 * \return pdTRUE
 * \return pdFALSE	buffer full
 */ 
portCHAR bus_uart_put( uint8_t byte, portTickType blocktime )
{
	portCHAR retval = pdFALSE;
	
	portENTER_CRITICAL();
	
	if(bus_txempty == pdTRUE )
	{
		bus_txempty = pdFALSE;
  		USART_SendData(USART1, (uint8_t) byte);
		retval = pdTRUE;
	}
	else
	{
		retval = xQueueSend(bus_tx, &byte, blocktime );

		if( ( bus_txempty == pdTRUE ) && ( retval == pdPASS ) )
		{
			/* Get back the character we just posted. */
			xQueueReceive(bus_tx, &byte, 0);
			bus_txempty = pdFALSE;
  			USART_SendData(USART1, (uint8_t) byte);
			retval = pdTRUE;
		}
		else if (retval == pdPASS)
		{
			retval = pdTRUE;
		}
		else
		{
			retval = pdFALSE;
		}
	}

	portEXIT_CRITICAL();

	return retval;
}

/**
 * Bus UART read.
 *
 * \param byte pointer to data byte
 * \param blocktime time to block
 *
 * \return pdTRUE
 * \return pdFALSE	buffer empty
 */ 
 
portCHAR bus_uart_get( uint8_t *byte, portTickType blocktime )
{
	if( xQueueReceive( bus_rx, byte, blocktime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}

/*
 * USART1 ISR
 */
void bus_rxISR( void );
void bus_txISR( void );

void __attribute__ ((interrupt)) __cs3_isr_usart1(void)
{
    if( USART_GetITStatus(USART1,USART_IT_RXNE) != RESET )
    {
	bus_rxISR();
    }
    if( USART_GetITStatus(USART1,USART_IT_TXE) != RESET )
    {
	bus_txISR();
    }
}
/**
 * Bus UART RX interrupt.
 */
void bus_rxISR( void )
{
	uint8_t byte;

	byte = USART_ReceiveData(USART1);

	if( xQueueSendFromISR( bus_rx, &byte, pdFALSE ) )
	{
		taskYIELD();
	}
}

/**
 * Bus UART Tx interrupt.
 */
void bus_txISR( void )
{
	uint8_t byte;
	portBASE_TYPE task;

	if( xQueueReceiveFromISR( bus_tx, &byte, &task ) == pdTRUE )
	{
	
  		USART_SendData(USART1, (uint8_t) byte);
	
	}
	else
	{
		bus_txempty = pdTRUE;
	}
}

/**
 * Approximate CPU loop pause.
 *
 *	Approximates multiples of 1 ms delay.
 *
 * \param time time in ms
 *
 */
 
void pause (uint16_t time)
{
#if 0
  uint16_t i, j;
	
  for (i = 0; i < time; i++)
		/*configCPU_CLOCK_HZ/1000 = millisec, 8 cycles/loop*/
    for (j = 0; j < (configCPU_CLOCK_HZ/(1000*8)); j++) 
    {
        NOP;
    }
#endif 
}

/**
 * Approximate CPU loop pause.
 *
 *	Approximates multiples of 1 us delay.
 *
 * \param time time in us
 *
 */
void pause_us (uint16_t time)
{
#if 0
  uint16_t i;
  for (i = 1; i < time; i++)
	{
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
	}
#endif
}

/* Random value table */
const uint8_t rand_table[16] =
{
	0x08, 0x0b, 0x03, 0x06, 0x10, 0x0d, 0x02, 0x0a, 0x04, 0x0f, 0x0e, 0x05, 0x0c, 0x09, 0x07, 0x11
};

static uint16_t rand_seed = 0xCBFA;

/**
 * Random value function.
 *
 * Function generates a value between 0 and range.
 *
 *  \return  rand random value.
 */
uint16_t random_generate(uint16_t range) 
{
	uint8_t i;
	uint16_t retval;
	
	for (i=0; i<32; i++)
	{
		if((((rand_seed << 1) ^ rand_seed) & 0x08) != 0)
			rand_seed = (rand_seed << 1) | 1;
	  else
			rand_seed <<= 1;
	
		rand_seed += rand_table[rand_seed & 0x0f];
		if ((i>4) && (rand_seed < range)) break;
	}
	retval = rand_seed;
	while (retval > range)
	{
		retval = retval >> 1;
	}
	return retval;
}


#ifndef HAVE_POWERSAVE
#include <task.h>
//#include <iomacros.h>
void vApplicationIdleHook( void );

/**
 *  Application idle hook. Set proper power mode.
 *
 */
void vApplicationIdleHook( void )
{
#if 0
  for(;;)
  {
		_BIS_SR(GIE+CPUOFF+SCG0);
    taskYIELD();
  }
#endif
}
#endif
