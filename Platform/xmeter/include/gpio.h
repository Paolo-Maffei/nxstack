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

#define LED_INIT() led_init()
#define LED2_OFF() led2_off()
#define LED2_ON()  led2_on()
#define LED1_OFF() led1_off()
#define LED1_ON()  led1_on()

extern void gpio_init(void);

extern portCHAR gpio1_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge);

extern portCHAR gpio2_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge);

extern void gpio_irq_enable(uint8_t pin);
extern void gpio_irq_disable(uint8_t pin);

extern void led_init(void);
extern void led1_off(void);
extern void led1_on(void);
extern void led2_off(void);
extern void led2_on(void);
#endif /* _MICRO_GPIO_H */
