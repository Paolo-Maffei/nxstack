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
 * \brief Simple ADC driver for micro.4.
 *
 *  Support library: an example ADC interrupt and support functions.
 *   
 */


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <signal.h>

#include "debug.h"
#include "adc.h"

#include "powersave.h"

/*compiler version check*/
#ifndef CONSEQ0 
#define CONSEQ0 (1<<1)
#define CONSEQ1 (2<<1)
#define ADC12SSEL0 (1 << 3)
#define ADC12SSEL1 (2 << 3)
#define ADC12DIV0 (1<<5)
#define ADC12DIV1 (2<<5)
#define ADC12DIV2 (4<<5)

#define SHT00              (1<<8)
#define SHT01              (2<<8)
#define SHT10              (1<<12)
#define SHT11              (2<<12)

#endif
uint16_t adc_value[16];
uint8_t adc_state = 16;


/**
 * Read ADC values.
 *
 * Read a sequence of channels, defined by channels and n_channels.
 *
 * The application shall wait for the adc_state variable to go to zero.
 * Once the adc_state variable is zero, values can be read from the
 * adc_value[] table. The values are indexed by the ADC channel,
 * i.e. if you read channels 0, 1 and 6, the values are at
 * adc_value[0], adc_value[1] and adc_value[6].
 *
 * \param channels array of channel ID's
 * \param n_channels number of channels to read
 *
 * \return pdTRUE
 * \return pdFALSE	invalid channel or too many channels selected
 */
portCHAR adc_read(uint8_t *channels, uint8_t n_channels)
{
	uint8_t i;
	
	if (!n_channels || (n_channels > 16)) return pdFALSE;
	for (i=0; i<n_channels; i++)
	{	/*check that channels are available*/
		if ((channels[i] > 7) || (channels[i] == 5) || (channels[i] == 4))
		{
			return pdFALSE;
		}
	}
	ADC12IE = 0;/*(1 << i)-1;*/ /*bit mask for interrupts*/
	ADC12CTL0 = 0;	
	for (i=0; i<n_channels; i++)
	{	/*allocate buffer to channel and enable ADC*/
		ADC12MCTL[i] = channels[i];
		P6SEL |= (1 << i);
	}
	ADC12MCTL[n_channels-1] |= 0x80; /*mark end of sequence*/
	ADC12CTL0 = SHT10 | SHT00 | MSC | ADC12ON;	
	/*SHTx = 8 * ADCClock (MCLK/8), Multiple sequence*/
	ADC12CTL1 = 0 | SHP| ADC12SSEL1 | 
		    ADC12DIV2 | ADC12DIV1 | ADC12DIV0 | CONSEQ0; 
	/*Starting address 0, sample and hold from ADC12SC, 
	  sourced from sampling timer, clock divider=8, MCLK, 
		Sequence of channels*/
	ADC12IE = (1 << (n_channels-1)); /*bit mask for interrupts*/
	ADC12CTL0 |=  ENC | ADC12SC;	
	/*Start*/
	adc_state = n_channels;
	return pdTRUE;
}

interrupt (ADC_VECTOR) ADC_ISR( void );

/**
 * ADC interrupt.
 *
 */
interrupt (ADC_VECTOR) ADC_ISR( void )
{
	uint8_t chan_index = 0;
	uint16_t mask = 1;
	
	while(ADC12IFG && mask)
	{
		if (ADC12IFG & mask)
		{
			adc_value[chan_index] = ADC12MEM[chan_index];
			adc_state--;
			if (!adc_state) 
			{	/*all conversions done, stop ADC*/
				P6SEL &= ~0xCF;
				ADC12CTL0 &= ~(ENC|ADC12SC);
			}
		}
		chan_index++;
		mask <<= 1;
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

