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
 * \file adc.c
 * \brief nano.4 ADC driver.
 *
 *  Nano.4: ADC control functions.
 *   
 *	
 */


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

#include <sys/inttypes.h>

#include "stack.h"
#include "debug.h"

#include "dma.h"
#include "adc.h"

uint16_t adc_value;
uint8_t adc_state;

/**
 * Do a single ADC conversion
 *
 * The application shall monitor the adc_state variable.
 * Once the variable goes to zero, the ADC value can be read
 * from adc_value.
 *
 * \param source channel to get ADC value from
 * \param ref reference source
 * \param resolution ADC resolution for conversion
 * \return 0 OK
 * \return -1 conversion already active
 **/
int8_t adc_convert_single(adc_input_t source, adc_ref_t ref, adc_res_t resolution)
{
	if ((ADCCON1 & (ADEOC|ADST)) != 0) return -1;

	if (source <= ADC_AIN7)	
	{	/*single channel*/
		ADCCFG = (1 << source);
	}
	else if (source < ADC_GND)	
	{	/*differential inputs*/
		ADCCFG = (3 << ((source-8)*2));
	}
	else ADCCFG = 0;
	
	ADCCON2 = ref | resolution | source;
	ADCCON1 = ADST | (ADSTS1|ADSTS0) /*| (ADCCON1 & (ADRCTRL1|ADRCTRL0))*/ | 0x0C | 3;
	
	return 0;
}

/**
 * Read single ADC conversion result
 *
 * Once the conversion is ready the function will return 1
 * and store the result in parameter result
 *
 * \param result pointer to store conversion
 *
 * \return 1 result ready
 * \return 0 conversion active
 * \return -1 no conversion active
 **/
int8_t adc_result_single(int16_t *result)
{
	uint8_t low;
	int8_t high;
	
	switch (ADCCON1 & (ADEOC|ADST))
	{
		case ADEOC:
			low = ADCL;
			high = ADCH;
			*result = ((int16_t) high) << 8;		
			*result |= low;
			return 1;
		case ADST:
			return 0;
		default:
			return -1;
	}
}
