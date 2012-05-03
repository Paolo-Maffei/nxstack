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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the CC2430 port.
 * Code is based on the Cygnal port.
 *----------------------------------------------------------*/

/* Standard includes. */
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* because of the interesting RAM behaviour make sure we dont allocate
   statics or anything else in these areas */
   
/*volatile __xdata __at (0xE000) volatile_ram1[4096];
volatile __xdata __at (0xFD58) volatile_ram2[423];*/
extern unsigned char __xdata __at(0xE000) volatile_ram1[0x1000];
extern unsigned char __xdata __at(0xFD58) volatile_ram2[424];

unsigned char __xdata __at(0xE000) volatile_ram1[0x1000];
unsigned char __xdata __at(0xFD58) volatile_ram2[424];

#define portGLOBAL_INTERRUPT_BIT	( ( portSTACK_TYPE ) 0x80 )

#if 0
/* Constants required to setup timer 2 to produce the RTOS tick. */
#define portCLOCK_DIVISOR				( ( unsigned portLONG ) 12 )
#define portMAX_TIMER_VALUE				( ( unsigned portLONG ) 0xffff )
#define portENABLE_TIMER				( ( unsigned portCHAR ) 0x04 )
#define portTIMER_2_INTERRUPT_ENABLE	( ( unsigned portCHAR ) 0x20 )

/* The value used in the IE register when a task first starts. */

/* The value used in the PSW register when a task first starts. */
#define portINITIAL_PSW				( ( portSTACK_TYPE ) 0x00 )

/* Macro to clear the timer 2 interrupt flag. */
#define portCLEAR_INTERRUPT_FLAG()	TMR2CN &= ~0x80;
#endif

/* Used during a context switch to store the size of the stack being copied
to or from XRAM. */
data unsigned portCHAR ucStackBytes;

/* Used in timer interrupt for calculating the next tick time */
data static unsigned long ulTimerValue;

/* Used during a context switch to point to the next byte in XRAM from/to which
a RAM byte is to be copied. */
xdata portSTACK_TYPE * data pxXRAMStack;

/* Used during a context switch to point to the next byte in RAM from/to which
an XRAM byte is to be copied. */
data portSTACK_TYPE * data pxRAMStack;

/* We require the address of the pxCurrentTCB variable, but don't want to know
any details of its type. */
typedef void tskTCB;
extern volatile tskTCB * volatile pxCurrentTCB;

/*
 * Setup the hardware to generate an interrupt off timer 2 at the required 
 * frequency.
 */
static void prvSetupTimerInterrupt( void );


/* 
 * See header file for description. 
 */
portSTACK_TYPE *pxPortInitialiseStack( portSTACK_TYPE *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters )
{
unsigned portLONG ulAddress;
portSTACK_TYPE *pxStartOfStack;

	/* Leave space to write the size of the stack as the first byte. */
	pxStartOfStack = pxTopOfStack;
	pxTopOfStack++;

	/* Place a few bytes of known values on the bottom of the stack. 
	This is just useful for debugging and can be uncommented if required.
	*pxTopOfStack = 0x11;
	pxTopOfStack++;
	*pxTopOfStack = 0x22;
	pxTopOfStack++;
	*pxTopOfStack = 0x33;
	pxTopOfStack++;
	*/

	/* Simulate how the stack would look after a call to the scheduler tick 
	ISR. 

	The return address that would have been pushed by the MCU. */
	ulAddress = ( unsigned portLONG ) pxCode;
	*pxTopOfStack = ( portSTACK_TYPE ) ulAddress;
	ulAddress >>= 8;
	pxTopOfStack++;
	*pxTopOfStack = ( portSTACK_TYPE ) ( ulAddress );
	pxTopOfStack++;

	/* Next all the registers will have been pushed by portSAVE_CONTEXT(). */
	*pxTopOfStack = 0xaa;	/* acc */
	pxTopOfStack++;	

	/* We want tasks to start with interrupts enabled. */
	*pxTopOfStack = portGLOBAL_INTERRUPT_BIT;
	pxTopOfStack++;

	/* The function parameters will be passed in the DPTR and B register as
	a three byte generic pointer is used. */
	ulAddress = ( unsigned portLONG ) pvParameters;
	*pxTopOfStack = ( portSTACK_TYPE ) ulAddress;	/* DPL */
	ulAddress >>= 8;
	*pxTopOfStack++;
	*pxTopOfStack = ( portSTACK_TYPE ) ulAddress;	/* DPH */
	ulAddress >>= 8;
	pxTopOfStack++;
	*pxTopOfStack = ( portSTACK_TYPE ) ulAddress;	/* b */
	pxTopOfStack++;

	/* The remaining registers are straight forward. */
	*pxTopOfStack = 0x02;	/* R2 */
	pxTopOfStack++;
	*pxTopOfStack = 0x03;	/* R3 */
	pxTopOfStack++;
	*pxTopOfStack = 0x04;	/* R4 */
	pxTopOfStack++;
	*pxTopOfStack = 0x05;	/* R5 */
	pxTopOfStack++;
	*pxTopOfStack = 0x06;	/* R6 */
	pxTopOfStack++;
	*pxTopOfStack = 0x07;	/* R7 */
	pxTopOfStack++;
	*pxTopOfStack = 0x00;	/* R0 */
	pxTopOfStack++;
	*pxTopOfStack = 0x01;	/* R1 */
	pxTopOfStack++;
	*pxTopOfStack = 0x00;	/* PSW */
	pxTopOfStack++;
	*pxTopOfStack = 0xbb;	/* BP */
	pxTopOfStack++;
	*pxTopOfStack = 0x01;	/* FMAP */

	/* Dont increment the stack size here as we don't want to include
	the stack size byte as part of the stack size count.

	Finally we place the stack size at the beginning. */
	*pxStartOfStack = ( portSTACK_TYPE ) ( pxTopOfStack - pxStartOfStack );

	/* Unlike most ports, we return the start of the stack as this is where the
	size of the stack is stored. */
	return pxStartOfStack;
}
/*-----------------------------------------------------------*/

/* 
 * See header file for description. 
 */
portBASE_TYPE xPortStartScheduler( void )
{
/*	volatile_ram1[0] = 0;
	volatile_ram2[0] = 0;*/
	
	/* Setup timer 2 to generate the RTOS tick. */
	prvSetupTimerInterrupt();	

	/* Make sure we start with the expected SFR page.  This line should not
	really be required. */
/*	SFRPAGE = 0;*/

	/* Copy the stack for the first task to execute from XRAM into the stack,
	restore the task context from the new stack, then start running the task. */
	portCOPY_XRAM_TO_STACK();
	portRESTORE_CONTEXT();

	/* Should never get here! */
	return pdTRUE;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* Not implemented for this port. */
}
/*-----------------------------------------------------------*/

/*
 * Manual context switch.  The first thing we do is save the registers so we
 * can use a naked attribute.
 */
void vPortYield( void ) _naked
{
	/* Save the execution context onto the stack, then copy the entire stack
	to XRAM.  This is necessary as the internal RAM is only large enough to
	hold one stack, and we want one per task. 
	
	PERFORMANCE COULD BE IMPROVED BY ONLY COPYING TO XRAM IF A TASK SWITCH
	IS REQUIRED. */
	portSAVE_CONTEXT();
	portCOPY_STACK_TO_XRAM();

	/* Call the standard scheduler context switch function. */
	vTaskSwitchContext();

	/* Copy the stack of the task about to execute from XRAM into RAM and
	restore it's context ready to run on exiting. */
	portCOPY_XRAM_TO_STACK();
	portRESTORE_CONTEXT();
}
/*-----------------------------------------------------------*/

#ifdef HAVE_NRP
#ifndef NRP_UART_DMA_RX
extern void	nrp_uart_rx_check(void);
#endif
#endif


/*Sleep timer runs on the 32k768 crystal */
#define TICK_VAL (32768 / configTICK_RATE_HZ)

#if configUSE_PREEMPTION == 1
	void vST_ISR( void ) interrupt (ST_VECTOR) _naked
	{
		EA = 0;
		/* Preemptive context switch function triggered by the timer 2 ISR.
		This does the same as vPortYield() (see above) with the addition
		of incrementing the RTOS tick count. */

		portSAVE_CONTEXT();
		portCOPY_STACK_TO_XRAM();

		ulTimerValue = ST0;
		ulTimerValue += ((unsigned long int)ST1) << 8;
		ulTimerValue += ((unsigned long int)ST2) << 16;
		ulTimerValue += TICK_VAL;
		ST2 = (unsigned char) (ulTimerValue >> 16);
		ST1 = (unsigned char) (ulTimerValue >> 8);
		ST0 = (unsigned char) ulTimerValue;
		
		vTaskIncrementTick();
#ifdef HAVE_NRP
#ifndef NRP_UART_DMA_RX
		nrp_uart_rx_check();
#endif
#endif
		vTaskSwitchContext();

		
		IRCON &= ~STIF;

		portCOPY_XRAM_TO_STACK();
		portRESTORE_CONTEXT();
		EA = 1;
		{		\
			_asm	\
			reti	\
			_endasm;\
		}
	}
#else
	void vST_ISR( void ) interrupt (ST_VECTOR)
	{
		IEN0_EA = 0;

		/* When using the cooperative scheduler the timer 2 ISR is only 
		required to increment the RTOS tick count. */

		ulTimerValue = ST0;
		ulTimerValue += ((unsigned long int)ST1) << 8;
		ulTimerValue += ((unsigned long int)ST2) << 16;
		ulTimerValue += TICK_VAL;
		ST2 = (unsigned char) (ulTimerValue >> 16);
		ST1 = (unsigned char) (ulTimerValue >> 8);
		ST0 = (unsigned char) ulTimerValue;

		vTaskIncrementTick();
		
		IRCON &= ~STIF;
		IEN0_EA = 1;
	}
#endif
/*-----------------------------------------------------------*/

static void prvSetupTimerInterrupt( void )
{
	CLKCON = OSC32K |  TICKSPD2|TICKSPD1|TICKSPD0;
	
	/*Initialize tick value and enable interrupt*/
	ulTimerValue = ST0;
	ulTimerValue += ((unsigned long int)ST1) << 8;
	ulTimerValue += ((unsigned long int)ST2) << 16;
	ulTimerValue += TICK_VAL;
	ST2 = (unsigned char) (ulTimerValue >> 16);
	ST1 = (unsigned char) (ulTimerValue >> 8);
	ST0 = (unsigned char) ulTimerValue;
	IEN0 |= STIE;
}




