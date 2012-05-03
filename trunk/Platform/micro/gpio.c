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
 * \brief micro GPIO functions.
 *
 *  Micro: General purpose functions. LED control, IRQ allocation.
 *	
 */

 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <signal.h>

#include "bus.h"
#include "debug.h"

#include "gpio.h"
#include "portable.h"

#ifdef HAVE_POWERSAVE
#include "powersave.h"
#endif

extern void bus_irq(void);
 
typedef void (*irq_handler_t)(void);

irq_handler_t vPort1_ISRs[8] =
{
	bus_irq,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

irq_handler_t vPort2_ISRs[4] =
{
	0,
	0,
	0,
	0
};


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
	if (mask)
	{
		if (vPort1_ISRs[pin] == 0) vPort1_ISRs[pin] = isr;
		if (vPort1_ISRs[pin] == isr)
		{
			P1IE &= ~mask;
			if (edge)
			{
				P1IES |= mask;
			}
			else
			{
				P1IES &= ~mask;
			}
			P1IFG &= ~mask;
			P1IE |= mask;
			return pdTRUE;
		}
	}
	return pdFALSE;
}

interrupt (PORT1_VECTOR) vPort1_ISR( void );

/**
 *  Port 1 interrupt handler.
 *
 *
 */
interrupt (PORT1_VECTOR) vPort1_ISR( void )
{
	{
		uint8_t i,mask;
		mask = 1;

		for (i=0; i<8; i++)
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
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

/**
 *  Allocate interrupt in port 2.
 *
 *	\param pin  port pin to allocate
 *	\param isr  interrupt service routine
 *  \param edge 0 = rising edge, anything else falling edge
 *
 *  \return   pdTRUE  allocation succeeded
 *  \return   pdFALSE interrupt reserved or not available on platform
 */
portCHAR gpio2_irq_allocate(uint8_t pin, void (*isr)(void), uint8_t edge)
{
	uint8_t mask = (1 << pin);
	mask &= 0x0F;
		
	if (mask)
	{
		if (vPort2_ISRs[pin] == 0) vPort2_ISRs[pin] = isr;
		if (vPort2_ISRs[pin] == isr)
		{
			P2IE &= ~mask;
			if (edge)
			{
				P2IES |= mask;
			}
			else
			{
				P2IES &= ~mask;
			}
			P2IFG &= ~mask;
			P2IE |= mask;
			return pdTRUE;
		}
	}
	return pdFALSE;
}
 
interrupt (PORT2_VECTOR) vPort2_ISR( void );
/**
 *  Port 2 interrupt handler.
 *
 *
 */
interrupt (PORT2_VECTOR) vPort2_ISR( void )
{
	{
		uint8_t i,mask;
		mask = 1;

		for (i=0; i<4; i++)
		{
			if (P2IFG & mask)
			{
				if (vPort2_ISRs[i])
				{
					vPort2_ISRs[i]();
				}
				P2IFG &= ~mask;
			}
			mask <<= 1;
		}
		P2IFG &= ~0xF0;	/*Port 2 high pins are MS3:0, should not be used for this*/
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

