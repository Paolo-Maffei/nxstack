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
 * \file flash.h
 * \brief micro.4 external flash
 *
 *  Micro.4 external flash: support function headers.
 *   
 */


#ifndef _MICRO_FLASH_H
#define _MICRO_FLASH_H

extern portCHAR flash_read(uint32_t address, uint8_t *buffer, uint16_t length);
extern portCHAR flash_write(uint32_t address, uint8_t *buffer, uint16_t length);
extern portCHAR flash_write_wait(void);
extern portCHAR flash_erase_sector(uint32_t address);
extern int16_t flash_signature_read(void);
extern int16_t flash_status_read(void);

/** Flash status register bits */
#define FLASH_SR_SRWD 0x80 /*!< Status register write protect */
#define FLASH_SR_BP2  0x10 /*!< Block protect bit 2*/
#define FLASH_SR_BP1  0x08 /*!< Block protect bit 1*/
#define FLASH_SR_BP0  0x04 /*!< Block protect bit 0*/
#define FLASH_SR_WEL  0x02 /*!< Write enable latch*/
#define FLASH_SR_WIP  0x01 /*!< Write in progress (busy)*/

#endif  /* _MICRO_FLASH_H  */
