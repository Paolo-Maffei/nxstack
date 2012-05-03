/*
    NanoStack: MCU software and PC tools for IP-based wireless sensor networking.
		
    Copyright (C) 2006-2007 Sensinode Ltd.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

		Address:
		Sensinode Ltd.
		Teknologiantie 6	
		90570 Oulu, Finland

		E-mail:
		info@sensinode.com
*/


/**
 *
 * \file debug.c
 * \brief UART printout library.
 *
 *  Debugging support library: printout functions,
 *  number formatting functions.
 *   
 *	Note: requires the preprocessor value DEBUG to
 *  exist.
 */

 

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "debug.h"

#include <sys/inttypes.h>

#include <signal.h>
#include <string.h>
#include <stdio.h>

#include "powersave.h"

#ifdef HAVE_DEBUG
#ifndef HAVE_NRP
#define HAVE_DEBUG_UART
#else
#undef HAVE_DEBUG_UART
#endif
#else
int16_t debug_read_blocking(uint32_t time)
{
	vTaskDelay( time / portTICK_RATE_MS );
	return -1;
}
#endif

#ifdef HAVE_DEBUG_UART
#ifndef portACLK_FREQUENCY_HZ
#define portACLK_FREQUENCY_HZ			( ( unsigned portLONG ) 32768 )
#endif

/* Enable the UART Tx interrupt. */
#define vInterrupt1On() IFG2 |= UTXIFG1
/** The queue used to hold received characters. */
static xQueueHandle debug_rx = 0; 

/** The queue used to hold characters waiting transmission. */
static xQueueHandle debug_tx = 0; 

#ifdef HAVE_POWERSAVE
static xPowerHandle debug_ph = 0;
#endif

xSemaphoreHandle debug_tx_lock = NULL;

static volatile uint8_t debug_txempty = pdTRUE;

int8_t debug_string(uint8_t *ptr);
int8_t debug_string_constant(prog_char *s);
int16_t debug_read(void);
int8_t debug_putchar(uint8_t c);

uint8_t debug_buffer[64];

/** Interrupts */
interrupt (UART1RX_VECTOR) uart1_rxISR( void );
interrupt (UART1TX_VECTOR) uart1_txISR( void );


#ifndef DEBUG_RX_LEN
#define DEBUG_RX_LEN 8
#endif
#ifndef DEBUG_TX_LEN
#define DEBUG_TX_LEN 64
#endif

/**
 * Initialize debug port.
 *
 * \param port debug port ID
 * \param speed port speed
 *
 * \return SUCCESS
 * \return FAILURE	insufficient memory
 */
void debug_init(uint32_t speed)
{
	unsigned long rate;
	uint8_t clock_sel;
	uint8_t mod = 0;

	portENTER_CRITICAL();
#ifdef HAVE_POWERSAVE
	if (debug_ph == 0)
	{
		debug_ph = power_alloc();
	}
#endif
	if (debug_rx == 0)
	{
		debug_rx = xQueueCreate( DEBUG_RX_LEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
		debug_tx = xQueueCreate( DEBUG_TX_LEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
	}
	if (speed > 9600)
	{
		clock_sel = (SSEL0 | SSEL1);
#ifndef configSMCLK_HZ
		rate = configCPU_CLOCK_HZ / speed;
#else
		rate = configSMCLK_HZ / speed;
#endif

#ifdef HAVE_POWERSAVE
		power_set(POWER_LPM0, debug_ph);
#endif		
	}
	else
	{	
		clock_sel = (SSEL0);
		rate = portACLK_FREQUENCY_HZ / speed;
#ifdef HAVE_POWERSAVE
		power_set(POWER_LPM3, debug_ph);
#endif
	}
	
	
	U1CTL |= SWRST;
	P3SEL |= (BIT6|BIT7);
	P3DIR |= BIT6;					/* Use P3.6 as TX */
	P3DIR &= ~BIT7;					/* Use P3.7 as RX */

	P5DIR &= ~(BIT2|BIT1);	/* SPI pins are hooked into these on micro.4*/

	U1CTL = CHAR + SWRST;	/* 8N1 */
	U1TCTL = clock_sel;

	U1MCTL = mod;

	/* Setup baud rate */
	U1BR0 = (uint8_t) ( rate );
	rate >>= 8;
	U1BR1 = (uint8_t) ( rate );

	/* Module enable */
	ME2 |= UTXE1 + URXE1;
	U1CTL &= ~SWRST;
	/* IRQ flags */
	IFG2 |= UTXIFG1;
	IFG2 &= ~URXIFG1;

	debug_txempty = pdTRUE;

	/* Enable interrupts. */
	IE2 |= URXIE1 + UTXIE1;
	portEXIT_CRITICAL();	

	if( debug_tx_lock == NULL )
	{
		vSemaphoreCreateBinary( debug_tx_lock );
	}
}

/**
 * Close debug port.
 * This only affects power saving.
 *
 */
void debug_close(void)
{
	/* Disable interrupts. */
	IE2 &= ~(URXIE1 + UTXIE1);
	ME2 &= ~(UTXE1 + URXE1);
#ifdef HAVE_POWERSAVE
	power_set(POWER_LPM4, debug_ph);
#endif
}


/**
 * Print a number to the debug port.
 *
 * \param width string maximum length
 * \param base base number (16 for hex, 10 for decimal etc.)
 * \param n number value
 *
 * \return pointer to the formatted string
 */
uint8_t *debug_integer(uint8_t width, uint8_t base, int n)
{
	uint8_t buffer[16];
	uint8_t i;
	if (base == 16)
	{
		sprintf(buffer, "%X", (unsigned int) n);
		i = strlen(buffer) + 1;
		if ((i & 1) == 0)
		{
			while (i > 0)
			{
				buffer[i] = buffer[i-1];
				i--;
			}
			buffer[0] = '0';		
		}
	} 
	else 
	{
		sprintf(buffer, "%d", n);
	}
/*	if (width > strlen(buffer))
	{
		memmove(&buffer[width-strlen(buffer)], buffer, strlen(buffer));
		memset(&buffer, '0',  width-strlen(buffer));
	}*/
	debug_send(buffer, strlen(buffer) );
  return buffer;
}

/**
 * Send a constant string.
 *
 * \param s pointer to string
 *
 * \todo wait time for task
 *
 * \return nr. of bytes sent
 */
int8_t debug_constant(prog_char *s)
{
	uint8_t i = 0;

	if (xSemaphoreTake( debug_tx_lock, 20 / portTICK_RATE_MS) != pdTRUE) return 0;
	
	while(*s)
	{
		if (debug_putchar(*s) == -1)
		{	/*buffer full, discard the rest*/
/*			taskYIELD();*/
			xSemaphoreGive( debug_tx_lock );
			return i;			
		}
		else
		{
			s++;
			i++;
		}
	}
	xSemaphoreGive( debug_tx_lock );
  return i;
}

/**
 * Send a single byte
 *
 * \param c byte value
 *
 * \return 0 success
 * \return -1 failure
 */
int8_t debug_put(uint8_t c)
{
	int8_t retval;
	if (xSemaphoreTake( debug_tx_lock, 20 / portTICK_RATE_MS) != pdTRUE) return -1;
	retval = debug_putchar(c);
	xSemaphoreGive( debug_tx_lock );
	return retval;
}

/**
 * Send a single byte, don't check locks
 *
 * \param c byte value
 *
 * \return 0 success
 * \return -1 failure
 */
int8_t debug_putchar(uint8_t c)
{
	int8_t retval = -1;
	portCHAR status;
	
 	portENTER_CRITICAL();
	if( debug_txempty == pdTRUE )
	{
		/* Start TX shifter. */
		debug_txempty = pdFALSE;
		U1TXBUF = c;
		retval = 0;
	}
	else
	{
		status = xQueueSend( debug_tx, &c, 0);

		/* Depending on queue sizing and task prioritisation:  While we 
		were blocked waiting to post on the queue interrupts were not 
		disabled.  It is possible that the serial ISR has emptied the 
		Tx queue, in which case we need to start the Tx off again
		writing directly to the Tx register. */
		if (status == pdFAIL)
		{
			retval = -1;
		}
		else
		{
			if( ( debug_txempty == pdTRUE ) && ( status == pdPASS ) )
			{
				xQueueReceive( debug_tx, &c, 0);
				debug_txempty = pdFALSE;
				U1TXBUF = c;
			}
			retval = 0;
		}
	}
	portEXIT_CRITICAL();
	return retval;
}

/**
 * Send multiple bytes
 *
 * \param buffer pointer to data
 * \param length buffer length
 *
 * \todo wait time for task
 *
 * \return number of bytes sent
 */
int8_t debug_send(uint8_t *buffer, uint8_t length)
{
	uint8_t i = 0;

	if (xSemaphoreTake( debug_tx_lock, 20 / portTICK_RATE_MS) != pdTRUE) return 0;
	
	while(i < length)
	{
		if (debug_putchar(buffer[i]) == -1)
		{	/*buffer full, discard the rest*/
/*			taskYIELD();*/
			xSemaphoreGive( debug_tx_lock );
			return i;			

		}
		else
		{
			i++;
		}
	}
	xSemaphoreGive( debug_tx_lock );
  return i;
}

/**
 * Read a single byte
 *
 * \return byte value
 * \return -1 no bytes available
 */
int16_t debug_read(void)
{
  uint8_t byte;
  if (xQueueReceive(debug_rx, &byte, 0 ) == pdTRUE)
  {
    return 0 + (uint16_t) byte;
  }
  else
  {
    return -1;
  }
}

/**
 * Read a single byte, blocking version
 *
 * \param time time to block (in milliseconds)
 *
 * \return byte value
 * \return -1 no bytes available
 */
int16_t debug_read_blocking(uint32_t time)
{
  uint8_t byte;
  
	if ( xQueueReceive(debug_rx, &byte, 
	    (portTickType) (time/portTICK_RATE_MS) ) == pdTRUE)
  {
    return 0 + (uint16_t) byte;
  }
  else
  {
    return -1;
  }
}

/**
 * Print an address
 *
 * \param addr pointer to sockaddr struct
 *
 */
void debug_address(sockaddr_t *addr)
{
	uint8_t i;
	uint8_t *ptr;
	
	ptr = addr->address;
	switch (addr->addr_type)
	{
		case ADDR_802_15_4_PAN_LONG:
				ptr += 8;
				debug_hex(*ptr++);
				debug_hex(*ptr++);
				debug(" ");
				ptr -= 10;
		case ADDR_802_15_4_LONG:
				ptr += 8;
				for (i=0; i<8; i++)
				{
					debug_hex(*--ptr);
					if (i < 7) debug(":");
				}
				break;
		case ADDR_802_15_4_PAN_SHORT:
				ptr += 2;
				debug_hex(*ptr++);
				debug_hex(*ptr++);
				debug(" ");
				ptr -= 4;
		case ADDR_802_15_4_SHORT:
				ptr += 2;
				for (i=0; i<2; i++)
				{
					debug_hex(*--ptr);
					if (i== 0) debug(":");
				}
				break;
		case ADDR_BROADCAST:
				debug("Broadcast");
				break;
		default:
				break;
	}
}

/**
 * UART RX interrupt service routine
 * for UART 1
 */

interrupt (UART1RX_VECTOR) uart1_rxISR( void )
{
	uint8_t byte;

	/* Get the character from the UART and post it on the queue of Rxed 
	characters. */
	byte = U1RXBUF;

	if( xQueueSendFromISR(debug_rx, &byte, pdFALSE ) )
	{
		/*If the post causes a task to wake force a context switch 
		as the woken task may have a higher priority than the task we have 
		interrupted. */
		taskYIELD();
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

/**
 * UART Tx interrupt service routine.
 * for UART 1
 */
interrupt (UART1TX_VECTOR) uart1_txISR( void )
{
	uint8_t byte;
	portBASE_TYPE task;

	if( xQueueReceiveFromISR( debug_tx, &byte, &task ) == pdTRUE )
	{
		U1TXBUF = byte;
	}
	else
	{
		debug_txempty = pdTRUE;
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

#endif /*HAVE_DEBUG_UART*/



