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
 * \file 1wire.c
 * \brief 1-wire interface functions.
 *
 *  Micro.4: 1-wire interface support functions.
 *	
 */

 

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <signal.h>
#include <string.h>

#include "debug.h"
#include "bus.h"
#include "1wire.h"

#define B1W_0() P3OUT &= ~0x02 
#define B1W_1() P3OUT |= 0x02 
#define B1W_R() (P3IN & 0x04) 

void bus_1wire_write_bit(uint8_t bit);
uint8_t bus_1wire_read_bit(void);

/**
 *  Execute 1-wire bus reset sequence
 *
 *	Bus reset and device present pulse detection for 1-wire bus.
 *
 * \return pdTRUE devices present
 * \return pdFAIL no devices present
 *
 */
portCHAR bus_1wire_reset(void)
{
	portCHAR retval = pdFALSE;
	uint8_t i;
		
	portENTER_CRITICAL();	
	B1W_0();
	pause_us(250);
	pause_us(250);
	B1W_1();
	pause_us(15);
	for (i=0; i<55;i++)
	{
		if (B1W_R() == 0)
		{
			retval = pdTRUE;
		}	
		pause_us(1);
	}
	portEXIT_CRITICAL();
	pause_us(230);
	pause_us(200);

	return retval;
}

/**
 *  Write a bit to 1-wire bus
 *
 * \param bit   bit to send
 *
 */
void bus_1wire_write_bit(uint8_t bit)
{
		portENTER_CRITICAL();
		dint();
		B1W_0();
		pause_us(6);
		if (bit)
		{
			B1W_1();
		}
		eint();
		portEXIT_CRITICAL();
		pause_us(80);
		B1W_1();
		pause_us(60);
}

/**
 *  Write a byte to 1-wire bus
 *
 * \param byte   byte to send
 *
 */
void bus_1wire_write(uint8_t byte)
{
	uint8_t i;
	
	for (i=0; i<8; i++)
	{
		bus_1wire_write_bit(byte & 1);
		byte >>= 1;
	}
}

/**
 *  Read a bit from 1-wire bus
 *
 * \return bit from bus
 *
 */
uint8_t bus_1wire_read_bit(void)
{
	uint8_t retval = 1;
	uint8_t i;
	
	portENTER_CRITICAL();
	dint();
	B1W_0();
	pause_us(6);
	B1W_1();
	pause_us(4);
	for (i=0; i<40; i++)
	{
		if (!B1W_R())
		{
			retval = 0;
		}
		pause_us(1);
	}
	eint();
	portEXIT_CRITICAL();
	pause_us(60);

	return retval;
}


/**
 *  Read a byte from 1-wire bus
 *
 * \return byte from bus
 *
 */

uint8_t bus_1wire_read(void)
{
	uint8_t i;
	uint8_t byte = 0;
		
	for (i=0; i<8; i++)
	{
		byte >>= 1;
		if (bus_1wire_read_bit())
		{
			byte |= 0x80;
		}
	}
	return byte;
}

/**
 *  Search 1-wire bus
 *
 * \param device    array of device structures
 * \param num_id    size of array
 *
 * \return number of devices found on the bus
 *
 */
uint8_t bus_1wire_search(b1w_reg *device, uint8_t num_id)
{
	uint8_t i;
	uint8_t tmp;
	
	uint8_t bit = 0;
	uint8_t pass = 0;
	uint8_t crc;
	uint8_t first_miss = 0xFF;
	uint8_t last_miss = 0;
	uint8_t byte_index, bit_mask;
	
	bit = 0;
		
	while ((num_id > pass))
	{
		if (bus_1wire_reset() == -1)
		{
			return 0;
		}

		bus_1wire_write(0xF0); //search
		
		crc = 0;
		
		if (bit)
		{
			if (bit == first_miss) first_miss = 0xFF;
			for (i=0; i <= bit; i++)
			{
				byte_index = 7 - (i >> 3);
				tmp = bus_1wire_read_bit();
				tmp <<= 1;
				tmp |= bus_1wire_read_bit();
				
				bit_mask = 0x01 << (i & 7);
				if (i==bit) device[pass][byte_index] |= bit_mask;
				bus_1wire_write_bit(device[pass][byte_index] & bit_mask);
			}
			last_miss = 0;
			bit ++;
		}	
		while (bit < 64)
		{
			byte_index = 7 -  (bit >> 3);
			bit_mask =   1 << (bit &  7);
			
			tmp = bus_1wire_read_bit();
			tmp <<= 1;
			tmp |= bus_1wire_read_bit();
			
			switch(tmp)
			{
				case 0: //can't tell
					if (bit == last_miss) tmp = 1;	//the last junction, move to 1
					else tmp = 0;
					if (first_miss == 0xFF)
					{
						first_miss = bit;
					}
					if (bit > last_miss) last_miss = bit;
					if (tmp) goto bit_one;

				case 1: //all zeroes
/*bit_zero:*/
					tmp = 0;
					device[pass][byte_index] &= ~bit_mask;
					bit++;
					break;

				case 2: //all ones
bit_one:
					device[pass][byte_index] |= bit_mask;
					bit++;
					break;

				case 3: //no devices
					bit = 64;
					pass = 100;
					break;
			}
			bus_1wire_write_bit(tmp);
		} //address scanned
		pass++;
		if (pass < num_id)
		{
			crc = 0;
			for (i=7; i>0; i--)
			{
				bus_1wire_crc_add(&crc, device[pass-1][i]);
			}
			if (crc != device[pass-1][0]) pass = 100;
			memcpy(device[pass], device[pass-1], 8);
		}
		if ((last_miss == 0) && (first_miss == 0xFF)) break;
		
		if (last_miss == 0)
		{ bit=first_miss;
		}
		else bit = last_miss;
		
		pause(100);
	}
/*	if ((pass > 1) && (pass < num_id))
	{
		if (memcmp(device[pass-1], device[pass-2], 8) == 0) pass--;
	}*/
	if (pass < num_id) return pass;
	else return 0;
}


/**
 *  Select 1-wire bus device by address (Match ROM)
 *
 * \param device    device address
 *
 * \return  pdTRUE or pdFALSE (no devices present)
 *
 */
portCHAR bus_1wire_select(b1w_reg device)
{
	uint8_t i;
	
	if (bus_1wire_reset() != pdTRUE)
	{
			return pdFALSE;
	}
	
	bus_1wire_write(0x55); /*match ROM*/
	for (i=0; i<8; i++)
	{
		bus_1wire_write(device[7-i]);
	}
	
	return pdTRUE;
}


/**
 *  Read 1-wire bus device address (Read ROM)
 *  Works only if there is just one device on the bus.
 *
 * \param device    device address is returned here
 *
 * \return  pdTRUE or pdFALSE (no devices present)
 *
 */
portCHAR bus_1wire_read_rom(b1w_reg device)
{
	uint8_t i;

	if (bus_1wire_reset() != pdTRUE)
	{
		debug("Reset failed.\r\n");
		return pdFALSE;
	}
	pause(10);
	bus_1wire_write(0x33); /*read ROM*/
	
	for (i=0; i<8; i++)
	{
		pause(10);
		device[7-i] = bus_1wire_read();
	}
	
	return pdTRUE;
}


/**
 *  Read DS2502 device memory (Read memory)
 *
 * \param device    device address
 * \param address		address to start read
 * \param buffer		data buffer
 * \param bytes			amount of bytes to read
 *
 * \return  pdTRUE or pdFALSE (no devices present or invalid family)
 *
 */
portCHAR bus_1wire_read_memory(b1w_reg device, uint16_t address, uint8_t *buffer, uint8_t bytes)
{
	uint8_t i, crc;
	
	if (device[7] != 0x09)
	{ /*2502 support only*/
		debug("Invalid device family.\r\n");
		return pdFALSE;
	}
	
	if (bus_1wire_select(device) != pdTRUE)
	{
		debug("Select failed.\r\n");
		return pdFALSE;
	}
	
	bus_1wire_write(0xF0); /*read memory*/
	bus_1wire_write(address);
	bus_1wire_write(address>>8);
	
	crc = bus_1wire_read();
	/*if crc fails, return pdFALSE*/
	
	for (i=0; i<bytes; i++)
	{
		buffer[i] = bus_1wire_read();
	}
	
	return pdTRUE;
}

/**
 *  Count 8-bit CRC for 1-wire. Variable crc is updated.
 *
 * \param crc       checksum variable, set to 0 before adding first byte
 * \param byte			value to add to checksum
 *
 */
void bus_1wire_crc_add(uint8_t *crc, uint8_t byte)
{
	uint8_t j;
	uint8_t bit;

	for (j=0; j<8; j++)
	{
		bit = 0;
		bit = (byte & 1) ^ (*crc & 1);

		*crc >>= 1;
		if (bit)
		{
			*crc |= 0x80;
			*crc ^= 0x0C;
		}
		else
		{
		}
		byte >>= 1;
	}
}

