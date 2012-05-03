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
 * \file dma.c
 * \brief DMA controller library.
 *
 *  DMA: mode control and support functions.
 *  General support functions and hardware initialization.
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

dma_config_t dma_conf[4];
dma_func dma_callback[4];

/**
 * Init DMA structures.
 *
 */
void dma_init(void)
{
	uint16_t tmp_ptr;
	
	memset(dma_conf, 0, 4*sizeof(dma_config_t));
	for (tmp_ptr = 0; tmp_ptr < 4; tmp_ptr++)
	{
		dma_callback[tmp_ptr] = 0;
	}
	tmp_ptr = (uint16_t) &(dma_conf[0]);

	DMA1CFGH = tmp_ptr >> 8;
	DMA1CFGL = tmp_ptr;
	IEN1_DMAIE = 1;	/*enable DMA interrupts*/	
}

#ifdef HAVE_DMA
/**
 * Configure a DMA channel.
 *
 * \param channel channel ID;
 * \param src source address;
 * \param src_inc source increment mode;
 * \param dst dest address;
 * \param dst_inc dest increment mode;
 * \param length maximum length;
 * \param vlen_mode variable length mode;
 * \param t_mode DMA transfer mode;
 * \param trigger DMA trigger;
 * \param function event function;
 *
 * \return Handle to DMA channel
 * \return 0 invalid channel
 */
xDMAHandle dma_config(uint8_t channel, void *src, dma_inc_t src_inc, void *dst, dma_inc_t dst_inc, 
                             uint16_t length, dma_vlen_t vlen_mode, dma_type_t t_mode, dma_trigger_t trigger,
			     dma_func function)
{
	if ((!channel) || (channel > 4)) return 0;
	
	DMAIRQ &= ~(1 << channel);
	
	channel--;
	
	dma_conf[channel].src_h = ((uint16_t) src) >> 8;
	dma_conf[channel].src_l = ((uint16_t) src);
	dma_conf[channel].dst_h = ((uint16_t) dst) >> 8;
	dma_conf[channel].dst_l = ((uint16_t) dst);
	dma_conf[channel].len_h = vlen_mode + (length >> 8);
	dma_conf[channel].len_l = length;
	dma_conf[channel].t_mode = (t_mode << 5) + trigger;
	dma_conf[channel].addr_mode = (src_inc << 6) + (dst_inc << 4) + 2; /*DMA has priority*/
	
	if (function)
	{
		dma_conf[channel].addr_mode |= 8;	/*set IRQMASK*/
		IEN1_DMAIE = 1;	/*enable DMA interrupts*/
	}
	dma_callback[channel] = function;
	
	return (xDMAHandle) channel+1;
}


/**
 * Arm a DMA channel.
 *
 * \param channel channel handle;
 *
 * \return pdTRUE
 * \return pdFALSE	semaphore creation failed
 */
portCHAR dma_arm(xDMAHandle channel)
{
	uint8_t ch_id = ((uint8_t) channel);
	if (!ch_id || (ch_id > 4)) return pdFALSE;
	DMAARM |= (1 << ch_id);
	return pdTRUE;
}

/**
 * Stop a DMA channel.
 *
 * \param channel channel handle;
 *
 * \return pdTRUE
 * \return pdFALSE	semaphore creation failed
 */
portCHAR dma_abort(xDMAHandle channel)
{
	uint8_t ch_id = ((uint8_t) channel);
	if (!ch_id || (ch_id > 4)) return pdFALSE;
	DMAARM = 0x80 + (1 << ch_id);	/*ABORT + channel bit*/
	return pdTRUE;
}

/**
 * Trigger a DMA channel.
 *
 * \param channel channel handle;
 *
 * \return pdTRUE
 * \return pdFALSE	semaphore creation failed
 */
portCHAR dma_trigger(xDMAHandle channel)
{
	uint8_t ch_id = ((uint8_t) channel);
	if (!ch_id || (ch_id > 4)) return pdFALSE;
	DMAREQ |= (1 << ch_id);
	return pdTRUE;
}

/**
 * Get DMA state.
 *
 * \param channel channel handle;
 *
 * \return pdTRUE	active
 * \return pdFALSE	not active
 */
portCHAR dma_state(xDMAHandle channel)
{
	uint8_t ch_id = ((uint8_t) channel);
	if (!ch_id || (ch_id > 4)) return pdFALSE;
	if ((DMAIRQ &(1 << ch_id)) == 0)
	{
		return pdTRUE;
	}
	return pdFALSE;
}



void dma_config_print(xDMAHandle channel)
{
	uint8_t ch_id = channel - 1;

	if (ch_id > 4) return;

	debug("DMA config ");
	debug_int(ch_id);
	debug(" @ ");
	debug_hex( (uint16_t) &(dma_conf[ch_id]) >> 8);
	debug_hex( (uint16_t) &(dma_conf[ch_id]) & 0xFF);
	
	debug(":\r\n");
	{
		uint8_t i;
		uint8_t *ptr = (uint8_t *) &(dma_conf[ch_id]);
		for (i = 0; i< 8; i++)
		{
			if (i != 0) debug(":");
			debug_hex(*ptr++);
		}
		debug("\r\n");
	}
	debug(".\r\n");
}
#endif

#ifdef HAVE_RF_DMA
extern void rf_dma_callback_isr(void);
#endif

#ifdef NRP_UART_DMA_RX
extern void nrp_uart_rx_dma_callback(void);
#endif

#ifdef NRP_UART_DMA_TX
extern void nrp_uart_tx_dma_callback(void);
#endif

/**
 * DMA interrupt service routine.
 *
 */
void dma_ISR( void ) interrupt (DMA_VECTOR)
{
#ifdef HAVE_DMA
	portBASE_TYPE prev_task = pdFALSE;
	uint8_t i;
#endif
	
#ifdef HAVE_RF_DMA
	if ((DMAIRQ & 1) != 0)
	{
		DMAIRQ &= ~1;
		rf_dma_callback_isr();
	}
#endif
#ifdef NRP_UART_DMA_RX
	if ((DMAIRQ & 0x08) != 0)
	{
		DMAIRQ &= ~(1 << 3);
		nrp_uart_rx_dma_callback();
	}		
#endif
#ifdef NRP_UART_DMA_TX		
	if ((DMAIRQ & 0x10) != 0)
	{
		DMAIRQ &= ~(1 << 4);
		nrp_uart_tx_dma_callback();
	}		
#endif
#ifdef HAVE_DMA
	for (i=0; i<4; i++)
	{
		if ((DMAIRQ & (1 << (i+1))) != 0)
		{
			DMAIRQ &= ~(1 << (i+1));
			if (dma_callback[i] != 0)
			{
				event_t event;
				event.process = dma_callback[i];
				event.param = (void *) 0;
				prev_task = xQueueSendFromISR(events, &event, prev_task);
			}
		}
	}
#endif
	IRCON_DMAIF = 0;
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

