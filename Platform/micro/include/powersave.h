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
 * \file powersave.h
 * \brief micro power saving API.
 *
 *  Micro power saving mode control and support function headers.
 *   
 *	
 */


#ifndef POWERSAVE_H
#define POWERSAVE_H
#ifdef HAVE_POWERSAVE
/** power modes*/
typedef enum power_mode_t
{
	POWER_ACTIVE = 0,	/*running*/
	POWER_LPM0 = 1,       /*CPU, MCLK disabled, SMCLK, ACLK active*/
	POWER_LPM1 = 2,       /*CPU, MCLK, DCO disabled, SMCLK, ACLK active, DC gen off*/
	POWER_LPM2 = 3,       /*CPU, MCLK, SMCLK, DCO disabled, ACLK active, DC gen on*/
	POWER_LPM3 = 4,       /*CPU, MCLK, SMCLK, DCO disabled, ACLK active, DC gen off*/
	POWER_LPM4 = 5,       /*CPU, MCLK, SMCLK, ACLK, DCO disabled, DC gen off, clock down*/
	POWER_XT1 = 0x80      /*Crystal must be kept running*/
}power_mode_t;

typedef power_mode_t *xPowerHandle;

extern void power_init(void);
extern xPowerHandle power_alloc(void);
extern void power_set(power_mode_t new_mode, xPowerHandle ph);
#define power_interrupt_epilogue() 	__asm__ __volatile__( "bic	%0, 0(r1)" : : "i" ((uint16_t)CPUOFF+SCG1+SCG0+OSCOFF) );
#endif
#endif /* POWERSAVE_H */
