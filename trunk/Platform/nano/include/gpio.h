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
 * \brief nano GPIO API.
 *
 *  Nano interrupt allocation for port 0 and 1.
 *   
 *	
 */


#ifndef _NANO_GPIO_H
#define _NANO_GPIO_H

#ifndef GPIO_MODEL
#define LED_INIT() 
#define LED1_ON() 
#define LED1_OFF() 
#define LED2_ON() 
#define LED2_OFF() 
#define LED1_TOGGLE()
#define LED2_TOGGLE()

#define LED1() P0_4
#define LED2() P0_5

#else

/* This default is for use with the N600 NanoRouter */
#if GPIO_MODEL == 1
#define LED_INIT()	{ P0DIR |= 0x30; }
#define LED1_ON() 	{ P0_4 = 1; }
#define LED1_OFF()	{ P0_4 = 0; }
#define LED2_ON()		{ P0_5 = 1; }
#define LED2_OFF()	{ P0_5 = 0; }
#define LED1_TOGGLE() {P0_4 ^= 1;}
#define LED2_TOGGLE() {P0_5 ^= 1;}

#define LED1() P0_4
#define LED2() P0_5
#endif

#if GPIO_MODEL == 2
/* Inits for the N710 NanoSensor */
#define LED_INIT() { P0DIR |= 0x30; } 
#define LED1_ON() {P0_4 = 1;}
#define LED1_OFF() {P0_4 = 0;}
#define LED2_ON() {P0_5 = 1;}
#define LED2_OFF() {P0_5 = 0;}
#define LED1_TOGGLE() {P0_4 ^= 1;}
#define LED2_TOGGLE() {P0_5 ^= 1;}

#define LED1() P0_4
#define LED2() P0_5

#define N710_BUTTON_INIT() { P0DIR &= ~0xc0; }
#define N710_SENSOR_INIT() { P0DIR &= ~0x0c; }

#define N710_BUTTON_1 P0_6
#define N710_BUTTON_2 P0_7

#define N710_TEMP ADC_AIN3
#define N710_LIGHT ADC_AIN2
#endif

#if GPIO_MODEL == 3
/* Inits for the N711 NanoSensor */
#define LED_INIT() { P0DIR |= 0x30; } 
#define LED1_ON() {P0_4 = 1;}
#define LED1_OFF() {P0_4 = 0;}
#define LED2_ON() {P0_5 = 1;}
#define LED2_OFF() {P0_5 = 0;}
#define LED1_TOGGLE() {P0_4 ^= 1;}
#define LED2_TOGGLE() {P0_5 ^= 1;}

#define LED1() P0_4
#define LED2() P0_5

#define N710_BUTTON_INIT() { P0DIR &= ~0xc0; }
#define N710_SENSOR_INIT() { P0DIR &= ~0x0c; }

#define N710_BUTTON_1 P0_6
#define N710_BUTTON_2 P0_7

#define N710_TEMP ADC_AIN0
#define N710_LIGHT ADC_AIN1
#endif

#endif


#ifdef HAVE_GPIO
extern portCHAR gpio0_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge);
extern portCHAR gpio1_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge);

extern void vPort0_ISR( void ) interrupt (P0INT_VECTOR);
extern void vPort1_ISR( void ) interrupt (P1INT_VECTOR);
#endif

#endif /* _NANO_GPIO_H */
