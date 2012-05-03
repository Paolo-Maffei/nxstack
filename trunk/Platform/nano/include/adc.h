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
 * \brief nano.4 ADC library.
 *
 *  Nano.4: ADC control headers.
 *   
 *	
 */


#ifndef _ADC_H
#define _ADC_H

typedef enum
{
	ADCREF_125V		= 0x00,
	ADCREF_AIN7		= 0x40,
	ADCREF_AVDD		= 0x80,
	ADCREF_DIFF67	= 0xC0	
}adc_ref_t;

typedef enum
{
	ADCRES_8BIT		= 0x00,
	ADCRES_10BIT	= 0x10,
	ADCRES_12BIT	= 0x20,
	ADCRES_14BIT	= 0x30	
}adc_res_t;

typedef enum
{
	ADC_AIN0		= 0x00,
	ADC_AIN1		= 0x01,
	ADC_AIN2		= 0x02,
	ADC_AIN3		= 0x03,
	ADC_AIN4		= 0x04,
	ADC_AIN5		= 0x05,
	ADC_AIN6		= 0x06,
	ADC_AIN7		= 0x07,
	ADC_DIFF01	= 0x08,
	ADC_DIFF23	= 0x09,
	ADC_DIFF45	= 0x0A,
	ADC_DIFF67	= 0x0B,
	ADC_GND			= 0x0C,
	ADC_REF			= 0x0D,
	ADC_TEMP		= 0x0E,
	ADC_VDD_DIV3= 0x0F
}adc_input_t;

extern int8_t adc_convert_single(adc_input_t source, adc_ref_t ref, adc_res_t resolution);
extern int8_t adc_result_single(int16_t *result);

#endif /*_ADC_H*/
