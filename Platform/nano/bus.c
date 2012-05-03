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
 * \file bus.c
 * \brief nano bus controls.
 *
 *  Nano: basic mode control and support functions.
 *  General support functions and hardware initialization.
 *   
 *	
 */

 

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

#include "debug.h"
#include "bus.h"
#include "gpio.h"

#include "progmem.h"

#ifdef HAVE_POWERSAVE
#include "powersave.h"
#endif

extern void dma_init(void);

/**
 * Initialize bus and MCU clock. 
 * First function to call.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	semaphore creation failed
 */
portCHAR bus_init(void)
{
	
	CLKCON |= OSC32K;
	CLKCON &= ~(OSC | CLKSPD); /*Osc on*/

	LED_INIT();
	
	P1_1 = 0;						/*Gain*/
	P1_2 = 1;						/*Regulator*/
	P1_3 = 1;						/*Amp*/
	
	P1DIR |= 0x0E;
#if 0
	T2CAPHPH = 16;			/*512 * 31.25 ns = 16 us = symbol period * 8 = 96 us*/
	T2CAPLPL = 0;
	T2CNF = 2;				/*Sync operation, compare high reg*/
#endif
/*	IP1 |= IP1_5;
	IP0 &= ~IP0_5;*/
	
	dma_init();
	
	return pdTRUE;
}

void bus_amp(uint8_t on)
{
	if (on) P1_3 = 0;
	else P1_3 = 1;
}

void bus_gain(uint8_t on)
{
	if (on) P1_1 = 1;
	else P1_1 = 0;
}

void bus_reg(uint8_t on)
{
	if (on) P1_2 = 1;
	else P1_2 = 0;
}

/**
 * Read a block of code memory.
 * The code must be placed in the lowest bank of flash.
 *
 * \param address address to read from flash
 * \param buffer  buffer to store data
 * \param size    number of bytes to read
 */
void flash_read(uint8_t *buffer, uint32_t address, uint8_t size)
{
	buffer;	 	/*dptr0*/
	address; 	/*stack-6*/
	size;			/*stack-7*/
	
	buffer;
	
	portDISABLE_INTERRUPTS();
	_asm
			mov dpl, r2
			mov dph, r3
			mov a, r0
			push acc
			mov a, r2
			push acc
			mov a, _MEMCTR
			push acc
			
			mov a, _bp
			add a, #0xf9 		;stack - 7 = size
			mov r0,a
			mov a, @r0  		;r2 = size
			mov r2, a   		;r2 = size
			
			inc r0
			mov a, @r0
			mov _DPL1, a		;DPTR1 = address & 0x7FFF | 0x8000
			inc r0
			mov a, @r0
			orl a, #0x80
			mov _DPH1, a
			inc r0					;MEMCTR = ((address >> 15 & 3) << 4) | 0x01 (bank select)
			mov a, @r0
			dec r0
			rrc a
			mov a, @r0
			rrc a
			rr a
			rr a
			anl a, #0x30
			orl a, #1
			mov _MEMCTR,a
lp1:
			mov _DPS, #1		;active DPTR = 1
			clr a
			movc a, @a+dptr			;read flash (DPTR1)
			inc dptr
			mov _DPS, #0 				;active DPTR = 0
			movx @dptr,a				;write to DPTR0
			inc dptr
			djnz r2,lp1					;while (--size)
			
			pop acc
			mov _MEMCTR, a	;restore bank
			
			pop acc
			mov r2,a
			pop acc
			mov r0,a
	_endasm;
	portENABLE_INTERRUPTS();
	DPL1 = *buffer++;
}

/**
 * Approximate CPU loop pause.
 *
 *	Approximates multiples of 1 us delay.
 *
 * \param time time in us
 *
 */
void pause_us(uint16_t time)
{
	uint16_t i;
	for (i = 0; i< time; i++)
	{
		portNOP();
	}
}

/**
 * Approximate CPU loop pause.
 *
 *	Approximates multiples of 1 ms delay.
 *
 * \param time time in ms
 *
 */
void pause(uint16_t time)
{
	uint16_t i;
	for (i = 0; i< time; i++)
	{
		pause_us(1000);
	}
}

static uint16_t rand_seed = 0xCBFA;

/**
 * Random value function.
 *
 * Function generates a value between 0 and range.
 *
 *  \return  rand random value.
 */
uint16_t random_generate(uint16_t range) 
{
	uint8_t i;
	uint32_t retval;

	RNDL = ST0;	
	for (i=0; i<4; i++)
	{
		rand_seed += ((uint16_t) RNDH << 8) + RNDL;
		RNDL = ST0;	
	}
	
	retval = rand_seed * range;
	return (retval >> 16);
}
