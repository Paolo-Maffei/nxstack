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
 * \file adc.h
 * \brief micro ADC driver header.
 *
 *  Micro: ADC support function headers.
 *   
 *	
 */


#ifndef _ADC_H
#define _ADC_H

/** ADC state, poll this for 0=conversion complete*/
extern uint8_t adc_state;
extern uint16_t adc_value[16];

extern portCHAR adc_read(uint8_t *channels, uint8_t n_channels);

#endif /*_ADC_H*/
