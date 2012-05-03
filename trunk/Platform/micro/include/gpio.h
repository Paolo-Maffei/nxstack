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
 * \file gpio.h
 * \brief micro GPIO API.
 *
 *  Micro interrupt allocation for ports 1 and 2.
 *   
 *	
 */


#ifndef _MICRO_GPIO_H
#define _MICRO_GPIO_H

#define LED_INIT() P6DIR |= 0x30
#define LED2_OFF() P6OUT &= ~0x20;
#define LED2_ON() P6OUT |= 0x20;
#define LED1_OFF() P6OUT &= ~0x10;
#define LED1_ON() P6OUT |= 0x10;

portCHAR gpio1_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge);

portCHAR gpio2_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge);


#endif /* _MICRO_GPIO_H */
