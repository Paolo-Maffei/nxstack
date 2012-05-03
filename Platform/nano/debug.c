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

#include <string.h>
#include <stdio.h>


#ifdef HAVE_DEBUG
#ifndef HAVE_NRP
#define HAVE_DEBUG_UART
#else
#undef HAVE_DEBUG_UART
#endif

/*typedef __code char prog_char;
#define PSTR(z) ((const prog_char *)z)
#define PRG_RDB(z) *z

#define debug(y) debug_constant((prog_char *)PSTR(y))
*/

uint8_t debug_buffer[64];
#ifndef DEBUG_UART
#define DEBUG_UART 1
#endif

#include "uart.h"

#if DEBUG_UART == 0
void debug_init(uint32_t speed)
{
	uart0_init(speed);
}

int16_t debug_read_blocking(uint32_t time)
{
	return uart0_get_blocking(time);
}

int8_t debug_put(uint8_t byte)
{
	return uart0_put(byte);
}
#else
void debug_init(uint32_t speed)
{
	uart1_init(speed);
}

int16_t debug_read_blocking(uint32_t time)
{
	return uart1_get_blocking(time);
}

int8_t debug_put(uint8_t byte)
{
	return uart1_put(byte);
}
#endif

int8_t debug_constant(const prog_char *s)
{
	uint8_t i = 0;

	while(*s)
	{
		if (debug_put(*s) == -1)
		{
			return i;
		}
		else
		{
			s++;
			i++;
		}
	}
	return i;
}

prog_char debug_hex_table[16] = 
{
	'0','1','2','3','4','5','6','7',
	'8','9','A','B','C','D','E','F'
};

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
	uint8_t i=0;
	uint8_t *ptr = debug_buffer + sizeof(debug_buffer) - 1;
	
	*ptr-- = 0;
	
	if (base == 16)
	{
		do
		{
			*ptr-- = debug_hex_table[n & 0x0F];
			n >>= 4;
			*ptr-- = debug_hex_table[n & 0x0F];
			n >>= 4;
		}while(n);
	} 
	else 
	{
		uint8_t negative = 0;
		if (n < 0)
		{ negative = 1;
			n = -n;
		}
		do
		{
			*ptr-- = (n % 10) + '0';
			n /= 10;
		}while (n);
		if (negative)
		{
			*ptr-- = '-';
		}
		else
		{
			*ptr-- = ' ';
		}
	}
	ptr++;
	debug_send(ptr, strlen(ptr) );
	return ptr;
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

	while(i < length)
	{
		if (debug_put(buffer[i]) == -1)
		{
			return i;
		}
		i++;
	}
	return i;
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
					if (i == 0) debug(":");
				}
				break;
		case ADDR_BROADCAST:
				debug("Broadcast");
				break;
		default:
				break;
	}
}
#else
int16_t debug_read_blocking(uint32_t time)
{
	vTaskDelay( time / portTICK_RATE_MS );
	return -1;
}
#endif
