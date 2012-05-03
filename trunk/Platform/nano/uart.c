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
 * \file uart.c
 * \brief nano.4 UART driver.
 *
 *  Nano.4: UART control functions.
 *   
 *	
 */


/* Standard includes. */
#include <stdlib.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "semphr.h"

typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short int uint16_t;
typedef unsigned long int uint32_t;
typedef signed short int int16_t;

typedef void tskTCB;
extern volatile tskTCB * volatile pxCurrentTCB;

#include "uart.h"

#define UART_TX_NO_QUEUE

#ifdef HAVE_UART0

#ifndef UART0_RX_LEN
#define UART0_RX_LEN 4
#endif

#ifndef UART0_TX_LEN
#define UART0_TX_LEN 128
#endif
volatile portCHAR uart0_txempty = pdTRUE;

/** The queue used to hold received characters. */
xQueueHandle uart0_rx = 0;

#ifndef UART_TX_NO_QUEUE
/** The queue used to hold characters waiting transmission. */
xQueueHandle uart0_tx = 0; 
#else
uint8_t uart0_tx_buffer[UART0_TX_LEN];
uint8_t uart0_tx_rd = 0;
uint8_t uart0_tx_wr = 0;
#endif
#ifdef HAVE_POWERSAVE
xPowerHandle uart0_ph = 0;
#endif

void uart0_init(uint32_t speed)
{
	if (speed != 115200) return;
	if (uart0_rx == 0)
	{
		uart0_rx = xQueueCreate(UART0_RX_LEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
#ifndef UART_TX_NO_QUEUE
		uart0_tx = xQueueCreate(UART0_TX_LEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
#else
		uart0_tx_rd = 0;
		uart0_tx_wr = 0;
#endif
	}
	/*Baud rate = ((256+UxBAUD) * 2^UxGCR)*crystal / (2^28)*/
#ifdef UART0_ALTERNATIVE_2
	PERCFG |= U0CFG;	/*alternative port 2 = P1.5-2*/
#ifdef UART0_RTSCTS
	P1SEL |= 0x3C;		/*peripheral select for TX and RX, RTS, CTS*/
#else
	P1SEL |= 0x30;		/*peripheral select for TX and RX*/
	P1 &= ~0x08;		/*RTS down*/
#endif
	P1DIR |= 0x28;		/*RTS, TX out*/
	P1DIR &= ~0x14;		/*CTS & RX in*/
#else
	PERCFG &= ~U0CFG;	/*alternative port 1 = P0.5-2*/
#ifdef UART0_RTSCTS
	P0SEL |= 0x3C;		/*peripheral select for TX and RX, RTS, CTS*/
#else
	P0SEL |= 0x0C;		/*peripheral select for TX and RX*/
	P0 &= ~0x20;		/*RTS down*/
#endif
	P0DIR |= 0x28;		/*RTS & TX out*/
	P0DIR &= ~0x14;		/*CTS & RX in*/
#endif
	
	U0BAUD=216;		/*115200*/
	U0GCR =/* U_ORDER |*/ 11; /*LSB first and 115200*/

/*	U0BAUD= 59;
	U0GCR = 8;*/ /*LSB first and 9600*/
	
#ifdef UART0_RTSCTS
	U0UCR = 0x42;	/*defaults: 8N1, RTS/CTS, high stop bit*/
#else
	U0UCR = 0x02;	/*defaults: 8N1, no flow control, high stop bit*/
#endif
	U0CSR = U_MODE | U_RE |U_TXB; /*UART mode, receiver enable, TX done*/
	IEN0_URX0IE = 1;
	IEN2 |= UTX0IE;

	uart0_txempty = pdTRUE;
}

int16_t uart0_get_blocking(portTickType time)
{
	uint8_t byte;

	if ( xQueueReceive(uart0_rx, &byte, time ) == pdTRUE)
	{
		return 0 + (uint16_t) byte;
	}
	else
	{
		return -1;
	}
}

int16_t uart0_get(void)
{
	uint8_t byte;
	
	if ( xQueueReceive(uart0_rx, &byte, (portTickType) 0 ) == pdTRUE)
	{
		return 0 + (uint16_t) byte;
	}
	else
	{
		return -1;
	}
}

int8_t uart0_put(uint8_t byte)
{
	int8_t retval = 0;
	
	if (uart0_txempty == pdTRUE)
	{
		uart0_txempty = pdFALSE;
		U0BUF = byte;
	}
	else
	{
#ifndef UART_TX_NO_QUEUE
		portCHAR status = xQueueSend(uart0_tx, &byte, 1);
		if(status == pdFAIL)
		{
			retval = -1;
		}
		else if (uart0_txempty == pdTRUE)
		{
			xQueueReceive(uart0_tx, &byte, 0);
			uart0_txempty = pdFALSE;
			U0BUF = byte;
		}
#else
		uint8_t new_ptr = uart0_tx_wr +1;
		if (new_ptr >= UART0_TX_LEN) new_ptr -= UART0_TX_LEN;
		if (new_ptr == uart0_tx_rd)
		{
			retval = -1;
		}
		else
		{
			uart0_tx_buffer[new_ptr] = byte;
			IEN0_EA = 0;
			uart0_tx_wr = new_ptr;
			if (uart0_txempty == pdTRUE)
			{
				uart0_tx_rd++;
				if (uart0_tx_rd >= UART0_TX_LEN) uart0_tx_rd -= UART0_TX_LEN;
				uart0_txempty = pdFALSE;
				U0BUF = byte;
			}
			IEN0_EA = 1;
		}
#endif
	}

	return retval;
}


/**
 * UART RX interrupt service routine
 * for UART 0
 */

void uart0_rxISR( void ) interrupt (URX0_VECTOR)
{
	uint8_t byte;
	
	TCON_URX0IF = 0;

	/* Get the character from the UART and post it on the queue of Rxed 
	characters. */
	byte = U0BUF;

	if( xQueueSendFromISR(uart0_rx, &byte, pdFALSE ) == pdTRUE )
	{
		taskYIELD();
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}


/**
 * UART Tx interrupt service routine.
 * for UART 0
 */
void uart0_txISR( void ) interrupt (UTX0_VECTOR)
{
#ifndef UART_TX_NO_QUEUE
	uint8_t byte;
	portBASE_TYPE task = pdFALSE;

	IRCON2_UTX0IF = 0;

	if( xQueueReceiveFromISR( uart0_tx, &byte, &task ) == pdTRUE )
	{
		U0BUF = byte;
	}
#else
	IRCON2_UTX0IF = 0;
	if (uart0_tx_rd != uart0_tx_wr)
	{
		uart0_tx_rd++;
		if (uart0_tx_rd >= UART0_TX_LEN) uart0_tx_rd -= UART0_TX_LEN;
		U0BUF = uart0_tx_buffer[uart0_tx_rd];
	}
#endif
	else
	{
		uart0_txempty = pdTRUE;
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}
#endif

#ifdef HAVE_UART1

#ifndef UART1_RX_LEN
#define UART1_RX_LEN 4
#endif

#ifndef UART1_TX_LEN
#define UART1_TX_LEN 128
#endif

volatile portCHAR uart1_txempty = pdTRUE;

/** The queue used to hold received characters. */
xQueueHandle uart1_rx = 0;

#ifndef UART_TX_NO_QUEUE
/** The queue used to hold characters waiting transmission. */
xQueueHandle uart1_tx = 0; 
#else
uint8_t uart1_tx_buffer[UART1_TX_LEN];
uint8_t uart1_tx_rd = 0;
uint8_t uart1_tx_wr = 0;
#endif

#ifdef HAVE_POWERSAVE
xPowerHandle uart1_ph = 0;
#endif

void uart1_init(uint32_t speed)
{
	if (speed != 115200) return;
	if (uart1_rx == 0)
	{
		uart1_rx = xQueueCreate(UART1_RX_LEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
#ifndef UART_TX_NO_QUEUE
		uart1_tx = xQueueCreate(UART1_TX_LEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
#else
		uart1_tx_rd = 0;
		uart1_tx_wr = 0;
#endif
	}
#ifdef UART1_ALTERNATIVE_1
	PERCFG &= ~U1CFG;	/*alternative port 1 = P0.5-2*/
#ifdef UART1_RTSCTS
	P0SEL |= 0x3C;		/*peripheral select for TX and RX, RTS, CTS*/
#else
	P0SEL |= 0x30;		/*peripheral select for TX and RX*/
	P0 &= ~0x08;		/*RTS down*/	
#endif
	P0DIR |= 0x18;		/*RTS, TX out*/
	P0DIR &= ~0x24;		/*CTS, RX in*/
#else
	PERCFG |= U1CFG;	/*alternative port 2 = P1.7-4*/
#ifdef UART1_RTSCTS
	P1SEL |= 0xF0;		/*peripheral select for TX and RX*/
#else
	P1SEL |= 0xC0;		/*peripheral select for TX and RX*/
	P1 &= ~0x20;		/*RTS down*/
#endif
	P1DIR |= 0x60;		/*RTS, TX out*/
	P1DIR &= ~0x90;		/*CTS, RX in*/
#endif

	/*Baud rate = ((256+UxBAUD) * 2^UxGCR)*crystal / (2^28)*/
	U1BAUD=216;		/*115200*/
	U1GCR = /*U_ORDER |*/ 11; /*LSB first and 115200*/
	
#ifdef UART1_RTSCTS
	U1UCR = 0x42;	/*defaults: 8N1, RTS/CTS, high stop bit*/
#else
	U1UCR = 0x02;	/*defaults: 8N1, no flow control, high stop bit*/
#endif
	U1CSR = U_MODE | U_RE |U_TXB; /*UART mode, receiver enable, TX done*/

	IEN0_URX1IE = 1;
	IEN2 |= UTX1IE;

	uart1_txempty = pdTRUE;
}

int16_t uart1_get_blocking(portTickType time)
{
	uint8_t byte;

	if ( xQueueReceive(uart1_rx, &byte, time ) == pdTRUE)
	{
		return 0 + (uint16_t) byte;
	}
	else
	{
		return -1;
	}
}

int16_t uart1_get(void)
{
	uint8_t byte;
	
	if ( xQueueReceive(uart1_rx, &byte, (portTickType) 0 ) == pdTRUE)
	{
		return 0 + (uint16_t) byte;
	}
	else
	{
		return -1;
	}
}

int8_t uart1_put(uint8_t byte)
{
	int8_t retval = 0;
	
	if (uart1_txempty == pdTRUE)
	{
		uart1_txempty = pdFALSE;
		U1BUF = byte;
	}
	else
	{
#ifndef UART_TX_NO_QUEUE
		portCHAR status = xQueueSend(uart1_tx, &byte, 0);
		if(status == pdFAIL)
		{
			retval = -1;
		}
		else if (uart1_txempty == pdTRUE)
		{
			xQueueReceive(uart1_tx, &byte, 0);
			uart1_txempty = pdFALSE;
			U1BUF = byte;
		}
#else
		uint8_t new_ptr = uart1_tx_wr +1;
		if (new_ptr >= UART1_TX_LEN) new_ptr -= UART1_TX_LEN;
		if (new_ptr == uart1_tx_rd)
		{
			retval = -1;
		}
		else
		{
			uart1_tx_buffer[new_ptr] = byte;
			IEN0_EA = 0;
			uart1_tx_wr = new_ptr;
			if (uart1_txempty == pdTRUE)
			{
				uart1_tx_rd++;
				if (uart1_tx_rd >= UART1_TX_LEN) uart1_tx_rd -= UART1_TX_LEN;
				uart1_txempty = pdFALSE;
				U1BUF = byte;
			}
			IEN0_EA = 1;
		}
#endif
	}

	return retval;
}


/**
 * UART RX interrupt service routine
 * for UART 1
 */

void uart1_rxISR( void ) interrupt (URX1_VECTOR)
{
	uint8_t byte;
	
	TCON_URX1IF = 0;

	/* Get the character from the UART and post it on the queue of Rxed 
	characters. */
	byte = U1BUF;

	if( xQueueSendFromISR(uart1_rx, &byte, pdFALSE ) == pdTRUE)
	{
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
void uart1_txISR( void ) interrupt (UTX1_VECTOR)
{
#ifndef UART_TX_NO_QUEUE
	uint8_t byte;
	portBASE_TYPE task = pdFALSE;

	IRCON2_UTX1IF = 0;
	if( xQueueReceiveFromISR( uart1_tx, &byte, &task ) == pdTRUE )
	{
		U1BUF = byte;
	}
#else
	IRCON2_UTX1IF = 0;
	if (uart1_tx_rd != uart1_tx_wr)
	{
		uart1_tx_rd++;
		if (uart1_tx_rd >= UART1_TX_LEN) uart1_tx_rd -= UART1_TX_LEN;
		U1BUF = uart1_tx_buffer[uart1_tx_rd];
	}
#endif
	else
	{
		uart1_txempty = pdTRUE;
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

#endif
