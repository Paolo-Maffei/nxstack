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
 * \file gpio.c
 * \brief nano GPIO functions.
 *
 *  Nano: General purpose functions. LED control, IRQ allocation.
 *	
 */

 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>

#include "bus.h"
#include "debug.h"

#include "gpio.h"
#include "portable.h"

#ifdef HAVE_POWERSAVE
#include "powersave.h"
#endif

#ifdef HAVE_GPIO

typedef void (*irq_handler_t)(void);

irq_handler_t vPort0_ISRs[8] =
{
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

irq_handler_t vPort1_ISRs[8] =
{
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

/**
 *  Allocate interrupt in port 0.
 *
 *	\param pin  port pin to allocate
 *	\param isr  interrupt service routine
 *  \param edge 0 = rising edge, anything else falling edge
 *
 *  \return   pdTRUE  allocation succeeded
 *  \return   pdFALSE interrupt reserved or not available on platform
 */
portCHAR gpio0_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge)
{
	uint8_t mask = (1 << pin);
	mask &= 0x0F;
		
	if (mask)
	{
		if (vPort0_ISRs[pin] == 0) vPort0_ISRs[pin] = isr;
		if (vPort0_ISRs[pin] == isr)
		{
			if (edge)
			{
				PICTL |= P0ICON;
			}
			else
			{
				PICTL &= ~P0ICON;
			}
			P0IFG &= ~mask;
			if (mask & 0xF0)
			{	/*high nibble*/
				PICTL |= P0IENH;
			}
			else
			{	/*low nibble*/
				PICTL |= P0IENL;				
			}
			IEN1_P0IE = 1;
			return pdTRUE;
		}
	}
	return pdFALSE;
}

/**
 *  Allocate interrupt in port 1.
 *
 *	\param pin  port pin to allocate
 *	\param isr  interrupt service routine
 *  \param edge 0 = rising edge, anything else falling edge
 *
 *  \return   pdTRUE  allocation succeeded
 *  \return   pdFALSE interrupt reserved or not available on platform
 */
portCHAR gpio1_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge)
{
	uint8_t mask = (1 << pin);
	
	if ((pin >= 1) && (pin <= 4)) mask = 0;
	
	if (mask)
	{
		if (vPort1_ISRs[pin] == 0) vPort1_ISRs[pin] = isr;
		if (vPort1_ISRs[pin] == isr)
		{
			P1IEN &= ~mask;
			if (edge)
			{
				PICTL |= P1ICON;
			}
			else
			{
				PICTL &= P1ICON;
			}
			P1IFG &= ~mask;
			P1IEN |= mask;
			IEN2 |= P1IE;
			return pdTRUE;
		}
	}
	return pdFALSE;
}

/**
 *  Port 0 interrupt handler.
 *
 *
 */
void vPort0_ISR( void ) interrupt (P0INT_VECTOR)
{
	{
		uint8_t i,mask;
		mask = 1;

		for (i=0; i<8; i++)
		{
			if (P0IFG & mask)
			{
				if (vPort0_ISRs[i])
				{
					vPort0_ISRs[i]();
				}
				P0IFG &= ~mask;
			}
			mask <<= 1;		
		}
	}
	IRCON_P0IF = 0;
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}
 
/**
 *  Port 1 interrupt handler.
 *
 *
 */
void vPort1_ISR( void ) interrupt (P1INT_VECTOR)
{
	{
		uint8_t i,mask;
		mask = 1;

		for (i=0; i<4; i++)
		{
			if (P1IFG & mask)
			{
				if (vPort1_ISRs[i])
				{
					vPort1_ISRs[i]();
				}
				P1IFG &= ~mask;
			}
			mask <<= 1;
		}
	}
	IRCON2_P1IF = 0;
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

#endif /*HAVE_GPIO*/
