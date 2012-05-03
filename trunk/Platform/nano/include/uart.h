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


#ifndef _UART_H
#define _UART_H

#ifdef HAVE_UART0
void uart0_rxISR( void ) interrupt (URX0_VECTOR);

void uart0_txISR( void ) interrupt (UTX0_VECTOR);

extern void uart0_init(uint32_t speed);
extern int16_t uart0_get(void);
extern int8_t uart0_put(uint8_t byte);
extern int16_t uart0_get_blocking(portTickType time);
#endif

#ifdef HAVE_UART1
void uart1_rxISR( void ) interrupt (URX1_VECTOR);

void uart1_txISR( void ) interrupt (UTX1_VECTOR);

extern void uart1_init(uint32_t speed);
extern int16_t uart1_get(void);
extern int8_t uart1_put(uint8_t byte);
extern int16_t uart1_get_blocking(portTickType time);
#endif

#endif /*_UART_H*/
