/*
	FreeRTOS.org V4.1.3 - Copyright (C) 2003-2006 Richard Barry.

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

/* 
Changes from V3.0.0
	+ ISRcode pulled inline to reduce stack-usage.

	+ Added functionality to only call vTaskSwitchContext() once
	  when handling multiple interruptsources in a single interruptcall.

	+ Filename changed to a .c extension to allow stepping through code
	  using F7.

Changes from V3.0.1
*/

/*
 * ISR for the tick.
 * This increments the tick count and, if using the preemptive scheduler, 
 * performs a context switch.  This must be identical to the manual 
 * context switch in how it stores the context of a task. 
 */

#ifndef _FREERTOS_DRIVERS_TICK_ISRTICK_C
#define _FREERTOS_DRIVERS_TICK_ISRTICK_C

{
	/*
	 * Was the interrupt the SystemClock?
	 */
	if( bCCP1IF && bCCP1IE )
	{
		/*
		 * Reset the interrupt flag
		 */
		bCCP1IF = 0;
	
		/*
	 	 * Maintain the tick count.
	 	 */
		vTaskIncrementTick();
		
		#if configUSE_PREEMPTION == 1
		{
			/*
		 	 * Ask for a switch to the highest priority task
		 	 * that is ready to run.
		 	 */
			uxSwitchRequested = pdTRUE;
		}
		#endif
	}
}

#pragma wizcpp uselib     "$__PATHNAME__/Tick.c"

#endif	/* _FREERTOS_DRIVERS_TICK_ISRTICK_C */
