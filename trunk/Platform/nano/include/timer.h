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
 * \file timer.h
 * \brief nano.4 timer support.
 *
 *  Nano.4: timer headers.
 *   
 *	
 */


#ifndef _TIMER_H
#define _TIMER_H
extern void timer_1_ISR( void ) interrupt (T1_VECTOR);
extern void timer_2_ISR( void ) interrupt (T2_VECTOR);
extern void timer_3_ISR( void ) interrupt (T3_VECTOR);
extern void timer_4_ISR( void ) interrupt (T4_VECTOR);

extern int8_t timer_mac_launch(uint8_t ticks);
extern void timer_mac_stop(void);

extern int8_t timer_rf_launch(uint8_t ticks);
extern void timer_rf_stop(void);


#endif /*_RF_H*/
