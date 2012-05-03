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


#ifndef _NRP_UART_H
#define _NRP_UART_H


#ifndef NRP_UART_DMA_RX		
void nrp_rxISR( void ) interrupt (URX1_VECTOR);
#endif
#ifndef NRP_UART_DMA_TX		
void nrp_txISR( void ) interrupt (UTX1_VECTOR);
#endif

extern void nrp_uart_init(uint32_t speed);
extern int16_t nrp_uart_get(void);
extern int8_t nrp_uart_put(uint8_t byte);
extern int16_t nrp_uart_get_blocking(uint16_t time);
extern void nrp_uart_launch(void);
extern void nrp_uart_rx_reset(void);

/*
#define nrp_uart_init(x) uart1_init(x)
#define nrp_uart_get(x) uart1_get(x)
#define nrp_uart_get_blocking(x) uart1_get_blocking(x)
#define nrp_uart_put(x) uart1_put(x)

#include "uart.h"
*/
#endif /*_NRP_UART_H*/
