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

#ifdef HAVE_TIMERA_OSTICK
#define PT_TIMER_CCR0		TACCR0
#define PT_TIMER_CCR1		TACCR1
#define PT_TIMER_CCR2		TACCR2
#define PT_TIMER_TR			TAR
#define PT_TIMER_CCTL0	TACCTL0
#define PT_TIMER_CCTL1	TACCTL1
#define PT_TIMER_CCTL2	TACCTL2
#define PT_TIMER_CTL 		TACTL
#define PT_TIMER_IV 		TAIV
#define PT_TIMER_VECTOR TIMERA1_VECTOR
#else
#define PT_TIMER_CCR0		TBCCR0
#define PT_TIMER_CCR1		TBCCR1
#define PT_TIMER_CCR2		TBCCR2
#define PT_TIMER_TR			TBR
#define PT_TIMER_CCTL0	TBCCTL0
#define PT_TIMER_CCTL1	TBCCTL1
#define PT_TIMER_CCTL2	TBCCTL2
#define PT_TIMER_CTL		TBCTL
#define PT_TIMER_IV 		TBIV
#define PT_TIMER_VECTOR TIMERB1_VECTOR
#endif

#ifdef HAVE_PERIOD_TIMER

#include "event_timer.h"

#include "stack.h"

void period_timer_hook(void);

uint16_t ptTickPeriod = 0;
uint16_t ptTickOverflow = 0;

uint16_t ptTickCount = 0;
uint16_t ptRegVal = 0;
void (*ptCallBack)(void *);

void ptTimerStart(uint16_t time_100us, void (*function)(void *));
void ptTimerStop(void);

#ifndef portACLK_FREQUENCY_HZ
#define portACLK_FREQUENCY_HZ			( ( unsigned portLONG ) 32768 )
#endif
/**
	* Start period timer. There can be only one!
	*
	* \param time_100us timer period in 10^-4 seconds
	* \param function   function to call periodically
	*
	*/
void ptTimerStart(uint16_t time_100us, void (*function)(void *))
{
	uint32_t ticks;
	
	ticks = (time_100us * portACLK_FREQUENCY_HZ) / 10000; /*unit conversion*/
	ptTickPeriod = ticks/PT_TIMER_CCR0;
	ptTickOverflow = ticks - (ptTickPeriod * PT_TIMER_CCR0);
	ptTickCount = 0;

	ptCallBack = function;
	
	PT_TIMER_CCR1 = PT_TIMER_CCR0 >> 1;
	PT_TIMER_CCTL1 = CCIE;	
}

/**
	* Stop period timer.
	*
	*/
void ptTimerStop(void)
{
	PT_TIMER_CCTL1 = 0;
	ptTickCount = 0;
}

/**
	* Period timer hook is called from event timer hook.
	*
	*/
void period_timer_hook(void)
{
	if (ptTickCount)
	{
		ptTickCount--;
		if (ptTickCount == 0)
		{
			PT_TIMER_CCR1 = ptRegVal;
			PT_TIMER_CCTL1 = CCIE;
		}
	}
}
#endif /*HAVE_PERIOD_TIMER*/
int8_t timer_rf_launch(uint16_t ticks);
void timer_rf_stop(void);
volatile uint16_t rf_pt_timer_ticks=0;

int8_t timer_rf_launch(uint16_t ticks)
{
	//if (ticks > PT_TIMER_CCR0) return -1;
	if (ticks > PT_TIMER_CCR0)
	{
		rf_pt_timer_ticks = (ticks / PT_TIMER_CCR0);
		ticks %= PT_TIMER_CCR0;

	}
	if(rf_pt_timer_ticks==0)
	{
		ticks += PT_TIMER_TR;
		if (ticks > PT_TIMER_CCR0) ticks-= PT_TIMER_CCR0;
		
		PT_TIMER_CCR2 = ticks;
		PT_TIMER_CCTL2 = CCIE;
	}
	else
	{
		//ticks += PT_TIMER_TR;
		//if (ticks > PT_TIMER_CCR0) ticks-= PT_TIMER_CCR0;
		PT_TIMER_CCR2 = ticks;
	}

	/*ticks += PT_TIMER_TR;
	if (ticks > PT_TIMER_CCR0) ticks-= PT_TIMER_CCR0;
	
	PT_TIMER_CCR2 = ticks;
	PT_TIMER_CCTL2 = CCIE;*/
	return 0;
}

void timer_rf_stop(void)
{
	PT_TIMER_CCTL2 = 0;
}

extern void rf_timer_callback(void);

#include <signal.h>
interrupt (PT_TIMER_VECTOR) prvPeriodISR( void );

/**
	* Period timer ISR.
	*
	*/
interrupt (PT_TIMER_VECTOR) prvPeriodISR( void )
{
#ifdef HAVE_PERIOD_TIMER
	uint16_t reg_val = PT_TIMER_CCR1;
	
	if (PT_TIMER_IV == 2) /*period timer*/
	{
		event_t event;
		ptTickCount = ptTickPeriod;
	
		reg_val += ptTickOverflow;
		if (reg_val >= PT_TIMER_CCR0)
		{
			ptTickCount++;
			reg_val -= PT_TIMER_CCR0;
		}
		ptRegVal = reg_val;
		PT_TIMER_CCTL1 = 0;
		event.param = 0;
		event.process = ptCallBack;
		xQueueSendFromISR(events, &event, pdFALSE);
	}
#endif
	if (PT_TIMER_IV == 4) /*rf driver timer*/
	{
			PT_TIMER_CCTL2 = 0;
			rf_timer_callback();
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

