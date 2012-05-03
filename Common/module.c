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
 * \file module.c
 * \brief Protocol extension modules.
 *
 *  Protocol stack extension modules: initialization, module table,
 *  management functions
 *   
 */


#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define COMPILING_MODULE_C

#include "debug.h"
#include "stack.h"
#include "buffer.h"

#include "module.h"

/**
 *  Initialize modules.
 *
 *  \return  pdTRUE    OK
 *  \return  pdFALSE   out of resources
 */
portCHAR module_init(void)
{
	uint8_t i = 0;

	while (modules[i].id != MODULE_NONE)
	{
		modules[i].init(0);
		i++;
	}

	return pdTRUE;
}

/**
 *  Find a protocol module.
 *
 *	\param id 	Protocol ID
 *
 *  \return  pointer to module structure
 */
module_t *module_get(module_id_t id)
{
	uint8_t i = 0;
	
	while ((modules[i].id != MODULE_NONE) && (modules[i].id != id))
	{
		i++;
	}
	if (modules[i].id != MODULE_NONE)
	{
			return &(modules[i]);
	}
	return 0;
}

/**
 *  Call module handler.
 *
 *	\param id 	Protocol ID
 *	\param buffer 	Buffer to be sent to selected module
 *
 *  \return  pdTRUE handler called
 *  \return	pdFALSE module not found or buffer not handled by module
 */
portCHAR module_call(module_id_t id, buffer_t *buffer)
{
	module_t *mod = module_get(id);
	
	if (mod)
	{
		return mod->handle(buffer);
	}
	
	return pdFALSE;
}

#ifdef HAVE_DEBUG
/**
 *  Print out available module ID's.
 *
 *  \return  pdTRUE    OK
 *  \return  pdFALSE   out of resources
 */
portCHAR module_setup(void)
{
	uint8_t i = 0;

	debug("Module setup:\r\n");	

	
	while (modules[i].id != MODULE_NONE)
	{
		if (i) debug(" ");
		debug_int(modules[i].id);
		i++;
	}

	debug("\r\n");	

	return pdTRUE;
}

#endif
