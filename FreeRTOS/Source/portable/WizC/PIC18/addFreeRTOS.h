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

Changes from V3.0.1

Changes from V4.0.1
    Uselib pragma added for Croutine.c
*/

/*
 * The installation script will automatically prepend this file to the default FreeRTOS.h.
 */

#ifndef WIZC_FREERTOS_H
#define WIZC_FREERTOS_H

#pragma	noheap
#pragma wizcpp expandnl   on
#pragma wizcpp searchpath "$__PATHNAME__/libFreeRTOS/Include/"
#pragma wizcpp uselib     "$__PATHNAME__/libFreeRTOS/Modules/Croutine.c"
#pragma wizcpp uselib     "$__PATHNAME__/libFreeRTOS/Modules/Tasks.c"
#pragma wizcpp uselib     "$__PATHNAME__/libFreeRTOS/Modules/Queue.c"
#pragma wizcpp uselib     "$__PATHNAME__/libFreeRTOS/Modules/List.c"
#pragma wizcpp uselib     "$__PATHNAME__/libFreeRTOS/Modules/Port.c"

#endif	/* WIZC_FREERTOS_H */
