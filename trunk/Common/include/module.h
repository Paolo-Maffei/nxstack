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
 * \file module.h
 * \brief Protocol stack module API.
 *
 *  Protocol stack module structure definitions
 *   
 */

 
#ifndef _NS_MODULE_H
#define _NS_MODULE_H 

typedef void *(*module_ext_func)(uint8_t function, void *param);
typedef portCHAR (*module_handler)(buffer_t *b);

#include "module_id.h"

/** Module structure */
typedef struct
{
  module_handler init;                   /*!< Init function of the module  */
  module_handler handle;                 /*!< Handler function of the module  */
  module_handler check;                  /*!< Handler function of the module  */  
	module_ext_func		ext;							/*!< Extension handler, if any */
  module_id_t       id;                   /*!< ID of the module  */
  uint8_t       hdr_size;               /*!< Header space to reserve */
	addrtype_t 	addr_type;							/*!< Address type for the module */
  uint16_t      mtu;                    /*!< Maximum transmission unit (0 if no limit) */
} module_t;

#include "modules_conf.h"

extern portCHAR module_init(void);
extern module_t *module_get(module_id_t id);
extern portCHAR module_setup(void);
extern portCHAR module_call(module_id_t id, buffer_t *buffer);

#endif /*_NS_MODULE_H*/
