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
 * \file timer.c
 * \brief Period timer API.
 *
 *  Period timer API: platform dependent hooks.
 *   
 */



#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>
#include <sys/inttypes.h>


int8_t timer_rf_launch(uint8_t ticks);
void timer_rf_stop(void);

int8_t timer_rf_launch(uint8_t ticks)
{
	T3CTL = 0;
	T3CCTL0 = 0;
	T3CC0 = ticks;
	IEN1_T3IE = 1;
	T3CTL = 0xA0 + 0x10 + 0x08 + 0x04 + 0x02;
/*	T3CCTL0 = 0x40 + 0x38 + 0x04;*/
	return 0;
}

void timer_rf_stop(void)
{
	T3CTL = 0;
}
#ifdef HAVE_MAC_15_4
int8_t timer_mac_launch(uint8_t ticks);
void timer_mac_stop(void);

int8_t timer_mac_launch(uint8_t ticks)
{
	T4CTL = 0;
	T4CCTL0 = 0;
	T4CC0 = ticks;
	IEN1_T4IE = 1;
	T4CTL = 0xA0 + 0x10 + 0x08 + 0x04 + 0x02;
	return 0;
}

void timer_mac_stop(void)
{
	T4CTL = 0;
}
#endif
/*
int8_t timer_mac_launch(uint16_t ticks);
void timer_mac_stop(void);

int8_t timer_mac_launch(uint16_t symbols)
{
	uint8_t dummy;
	uint32_t current;
	
	T2PEROF2 &= ~OFCMPIM;
	T2CNF &= ~OFCMPIF;
	
	symbols >>= 3;	/divide by 8/
	symbols ++;			/round up/
	
	current = (uint32_t)T2OF0;
	current |= ((uint32_t)T2OF1 << 8);
	current |= ((uint32_t)T2OF2 << 16);
	current += symbols;
	
	T2PEROF0 = current;
	T2PEROF1 = current >> 8;
	T2PEROF2 = ((current >> 16) & 0x0F) | OFCMPIM;
	
	T2CNF |= RUN;
	return 0;
}

void timer_mac_stop(void)
{
	T2PEROF2 &= ~OFCMPIM;
	T2CNF &= ~OFCMPIF;
}
*/
extern void rf_timer_callback(void);
extern void mac_timer_callback(void);

void timer_1_ISR( void ) interrupt (T1_VECTOR)
{
	T1CTL = 0;
}

void timer_2_ISR( void ) interrupt (T2_VECTOR)
{
	T2CNF &= ~OFCMPIF;
	T2CNF = 0;
}

void timer_3_ISR( void ) interrupt (T3_VECTOR)
{
	TIMIF = ~(T3CH1IF | T3CH0IF| T3OVFIF) & 0x3F;
	T3CTL = 0;
	rf_timer_callback();
}

void timer_4_ISR( void ) interrupt (T4_VECTOR)
{
	TIMIF = ~(T4CH1IF | T4CH0IF| T4OVFIF) & 0x3F;
	T4CTL = 0;
#ifdef HAVE_MAC_15_4
	mac_timer_callback();
#endif
}
