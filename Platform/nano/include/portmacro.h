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


/* 
	FreeRTOS.org V4.0.2 - Copyright (C) 2003-2006 Richard Barry.

	This file is part of the FreeRTOS.org distribution.

	FreeRTOS.org is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	FreeRTOS.org is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with FreeRTOS.org; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

	A special exception to the GPL can be applied should you wish to distribute
	a combined work that includes FreeRTOS.org, without being obliged to provide
	the source code for any proprietary components.  See the licensing section 
	of http://www.FreeRTOS.org for full details of how and when the exception
	can be applied.

	***************************************************************************
	See http://www.FreeRTOS.org for documentation, latest information, license 
	and contact details.  Please ensure to read the configuration and relevant 
	port sections of the online documentation.
	***************************************************************************
*/

#ifndef PORTMACRO_H
#define PORTMACRO_H

extern unsigned char __xdata __at(0xE000) volatile_ram1[0x1000];
extern unsigned char __xdata __at(0xFD58) volatile_ram2[424];

#if configUSE_PREEMPTION == 0
	void vST_ISR( void ) interrupt (ST_VECTOR);
#else
	void vST_ISR( void ) interrupt (ST_VECTOR) _naked;
#endif

/*-----------------------------------------------------------
 * Port specific definitions.  
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		float
#define portLONG		long
#define portSHORT		short
#define portSTACK_TYPE	unsigned portCHAR
#define portBASE_TYPE	char

#if( configUSE_16_BIT_TICKS == 1 )
	typedef unsigned portSHORT portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffff
#else
	typedef unsigned portLONG portTickType;
	#define portMAX_DELAY ( portTickType ) 0xffffffff
#endif
/*-----------------------------------------------------------*/	

/* Critical section management. */
#define portENTER_CRITICAL()	\
{	\
	_asm		\
	push	ACC	\
	push	IE	\
	_endasm;	\
}	\
	EA = 0;

#define portEXIT_CRITICAL()	\
{	\
	_asm			\
	pop		ACC	\
	_endasm;		\
	ACC &= 0x80;		\
	IE |= ACC;		\
	_asm			\
	pop		ACC	\
	_endasm; 		\
}

#define portDISABLE_INTERRUPTS()	EA = 0;
#define portENABLE_INTERRUPTS()		EA = 1;
/*-----------------------------------------------------------*/	

/* Hardware specifics. */
#define portBYTE_ALIGNMENT			1
#define portSTACK_GROWTH			( 1 )
#define portTICK_RATE_MS			( ( unsigned portLONG ) 1000 / configTICK_RATE_HZ )		
/*-----------------------------------------------------------*/	

/* Task utilities. */
void vPortYield( void ) _naked;
#define portYIELD()	vPortYield();
/*-----------------------------------------------------------*/	

/* Compiler specifics. */
#define inline
#define portNOP()				_asm	\
									nop \
								_endasm;

/*-----------------------------------------------------------*/	

/* Task function macros as described on the FreeRTOS.org WEB site. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )


/* Used during a context switch to store the size of the stack being copied
to or from XRAM. */
extern data unsigned portCHAR ucStackBytes;

/* Used during a context switch to point to the next byte in XRAM from/to which
a RAM byte is to be copied. */
extern xdata portSTACK_TYPE * data pxXRAMStack;

/* Used during a context switch to point to the next byte in RAM from/to which
an XRAM byte is to be copied. */
extern data portSTACK_TYPE * data pxRAMStack;

/*-----------------------------------------------------------*/
/*
 * Macro that copies the current stack from internal RAM to XRAM.  This is 
 * required as the 8051 only contains enough internal RAM for a single stack, 
 * but we have a stack for every task.
 */
#define portCOPY_STACK_TO_XRAM()																\
{																								\
	/* pxCurrentTCB points to a TCB which itself points to the location into					\
	which the first	stack byte should be copied.  Set pxXRAMStack to point						\
	to the location into which the first stack byte is to be copied. */							\
	pxXRAMStack = ( xdata portSTACK_TYPE * ) *( ( xdata portSTACK_TYPE ** ) pxCurrentTCB );		\
																								\
	/* Set pxRAMStack to point to the first byte to be coped from the stack. */					\
	pxRAMStack = ( data portSTACK_TYPE * data ) configSTACK_START;								\
																								\
	/* Calculate the size of the stack we are about to copy from the current					\
	stack pointer value. */																		\
	ucStackBytes = SP - ( configSTACK_START - 1 );												\
																								\
	/* Before starting to copy the stack, store the calculated stack size so					\
	the stack can be restored when the task is resumed. */										\
	*pxXRAMStack = ucStackBytes;																\
																								\
	/* Copy each stack byte in turn.  pxXRAMStack is incremented first as we					\
	have already stored the stack size into XRAM. */											\
	while( ucStackBytes )																		\
	{																							\
		pxXRAMStack++;																			\
		*pxXRAMStack = *pxRAMStack;																\
		pxRAMStack++;																			\
		ucStackBytes--;																			\
	}																							\
}
/*-----------------------------------------------------------*/

/*
 * Macro that copies the stack of the task being resumed from XRAM into 
 * internal RAM.
 */
#define portCOPY_XRAM_TO_STACK()																\
{																								\
	/* Setup the pointers as per portCOPY_STACK_TO_XRAM(), but this time to						\
	copy the data back out of XRAM and into the stack. */										\
	pxXRAMStack = ( xdata portSTACK_TYPE * ) *( ( xdata portSTACK_TYPE ** ) pxCurrentTCB );		\
	pxRAMStack = ( data portSTACK_TYPE * data ) ( configSTACK_START - 1 );						\
																								\
	/* The first value stored in XRAM was the size of the stack - i.e. the						\
	number of bytes we need to copy back. */													\
	ucStackBytes = pxXRAMStack[ 0 ];															\
																								\
	/* Copy the required number of bytes back into the stack. */								\
	do																							\
	{																							\
		pxXRAMStack++;																			\
		pxRAMStack++;																			\
		*pxRAMStack = *pxXRAMStack;																\
		ucStackBytes--;																			\
	} while( ucStackBytes );																	\
																								\
	/* Restore the stack pointer ready to use the restored stack. */							\
	SP = ( unsigned portCHAR ) pxRAMStack;														\
}
/*-----------------------------------------------------------*/

/*
 * Macro to push the current execution context onto the stack, before the stack 
 * is moved to XRAM. 
 */
#define portSAVE_CONTEXT()																		\
{																								\
	_asm																						\
		/* Push ACC first, as when restoring the context it must be restored					\
		last (it is used to set the IE register). */											\
		push	ACC																				\
		/* Store the IE register then disable interrupts. */									\
		push	IE																				\
		clr		_EA																				\
		push	DPL																				\
		push	DPH																				\
		push	b																				\
		push	ar2																				\
		push	ar3																				\
		push	ar4																				\
		push	ar5																				\
		push	ar6																				\
		push	ar7																				\
		push	ar0																				\
		push	ar1																				\
		push	PSW																				\
	_endasm;																					\
		PSW = 0;																				\
	_asm																						\
		push	_bp																				\
		push  _FMAP																			\
	_endasm;																					\
}
/*-----------------------------------------------------------*/

/*
 * Macro that restores the execution context from the stack.  The execution 
 * context was saved into the stack before the stack was copied into XRAM.
 */
#define portRESTORE_CONTEXT()																	\
{																								\
	_asm																						\
		pop   _FMAP																			\
		pop		_bp																				\
		pop		PSW																				\
		pop		ar1																				\
		pop		ar0																				\
		pop		ar7																				\
		pop		ar6																				\
		pop		ar5																				\
		pop		ar4																				\
		pop		ar3																				\
		pop		ar2																				\
		pop		b																				\
		pop		DPH																				\
		pop		DPL																				\
		/* The next byte of the stack is the IE register.  Only the global						\
		enable bit forms part of the task context.  Pop off the IE then set						\
		the global enable bit to match that of the stored IE register. */						\
		pop		ACC																				\
		JB		ACC.7,0098$																		\
		CLR		IE.7																			\
		LJMP	0099$																			\
	0098$:																						\
		SETB	IE.7																			\
	0099$:																						\
		/* Finally pop off the ACC, which was the first register saved. */						\
		pop		ACC																				\
		/*reti*/																					\
	_endasm;																					\
}
/*-----------------------------------------------------------*/

#endif /* PORTMACRO_H */


