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
 * \file nrp_uart.c
 * \brief nano.4 NRP UART driver.
 *
 *  Nano.4: NRP UART control: no message queues.
 *   
 *	
 */



/* Standard includes. */
#include <sys/inttypes.h>

#include <cc2430_sfr.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "dma.h"

/* Scheduler includes. */
#include "nrp_uart.h"

#include "debug.h"

extern dma_func dma_callback[4];

#ifndef NRP_UART_RX_LEN
#define NRP_UART_RX_LEN 256
#define NRP_UART_RX_MASK 255
#endif

#ifndef NRP_UART_TX_LEN
#define NRP_UART_TX_LEN 256
#define NRP_UART_TX_MASK 255
#endif

volatile uint8_t nrp_uart_txempty = 1;

volatile uint8_t nrp_uart_error = 0;

/** The queue used to hold received characters. */
uint8_t nrp_uart_rx_buffer[NRP_UART_RX_LEN];
volatile uint8_t nrp_uart_rx_rd = 0;
volatile uint8_t nrp_uart_rx_wr = 0;

uint8_t nrp_uart_tx_buffer[NRP_UART_TX_LEN];
volatile uint8_t nrp_uart_tx_rd = 0;
volatile uint8_t nrp_uart_tx_wr = 0;

xSemaphoreHandle nrp_rx_lock = NULL;

#ifdef NRP_UART_DMA_TX		
void nrp_uart_tx_dma_callback( void );
#endif
#ifdef NRP_UART_DMA_RX		
void nrp_uart_rx_dma_callback( void );
volatile uint16_t nrp_uart_rx_dma_len; 
volatile uint8_t nrp_tag_mode; 
uint8_t nrp_uart_dma_buffer[128];
#endif
void nrp_uart_init(uint32_t speed)
{
	if (speed != 115200) return;

	nrp_uart_rx_rd = 0;
	nrp_uart_rx_wr = 0;
	nrp_uart_tx_rd = 0;
	nrp_uart_tx_wr = 0;

	if (nrp_rx_lock == NULL)
	{
		vSemaphoreCreateBinary( nrp_rx_lock );
	}
	/*Baud rate = ((256+UxBAUD) * 2^UxGCR)*crystal / (2^28)*/

	PERCFG |= U1CFG;	/*alternative port 2 = P1.7-4*/
#ifdef NRP_NO_RTSCTS
	P1SEL |= 0xC0;		/*peripheral select for TX and RX*/
	P1DIR |= 0x20;		/*RTS out*/
	P1 &= ~0x20;		/*RTS low*/
#else
	P1SEL |= 0xF0;		/*peripheral select for all pins*/
#endif
	U1BAUD=216;		/*115200*/

	U1GCR = /*U_ORDER |*/ 11; /*LSB first and 115200*/
#ifdef NRP_NO_RTSCTS
	U1UCR = 0x02;					/*defaults: 8N1, no flow control, high stop bit*/
#else
	U1UCR = 0x02 + 0x40;	/*8N1, RTS/CTS, high stop bit*/
#endif
	U1CSR = U_MODE | U_RE |U_TXB; /*UART mode, receiver enable, TX done*/

	/*set priority group of group 3 to highest, so UART won't miss bytes*/
	
	IP1 |= IP1_3;
	IP0 |= IP0_3;

#ifdef NRP_UART_DMA
	{	/*DMA setup*/
		void *src;
#ifdef NRP_UART_DMA_RX		
		src = &U1BUF_SHADOW;
		nrp_uart_rx_dma_len = 4;
		nrp_tag_mode = 0;
			
		dma_callback[2] = nrp_uart_rx_dma_callback;
		
		dma_conf[2].src_h = ((uint16_t) src) >> 8;
		dma_conf[2].src_l = ((uint16_t) src);
		src = &nrp_uart_dma_buffer[0];
		dma_conf[2].dst_h = ((uint16_t) src) >> 8;
		dma_conf[2].dst_l = ((uint16_t) src);
		dma_conf[2].len_h = DMA_VLEN_LEN ;
		dma_conf[2].len_l = nrp_uart_rx_dma_len;
		dma_conf[2].t_mode = (DMA_SINGLE << 5) + DMA_T_URX1;
		dma_conf[2].addr_mode = (DMA_NOINC << 6) + (DMA_INC << 4) + 8 + 4 + 1; /*IRQMASK, DMA has low priority*/
		DMAARM = (1 << 3);
#endif		
#ifdef NRP_UART_DMA_TX		
		dma_callback[3] = nrp_uart_tx_dma_callback;
		
		src = &nrp_uart_tx_buffer[0];
		dma_conf[3].src_h = ((uint16_t) src) >> 8;
		dma_conf[3].src_l = ((uint16_t) src);
		src = &U1BUF_SHADOW;
		dma_conf[3].dst_h = ((uint16_t) src) >> 8;
		dma_conf[3].dst_l = ((uint16_t) src);
		dma_conf[3].len_h = DMA_VLEN_LEN ;
		dma_conf[3].len_l = 1;
		dma_conf[3].t_mode = (DMA_SINGLE << 5) + DMA_T_UTX1;
		dma_conf[3].addr_mode = (DMA_INC << 6) + (DMA_NOINC << 4) + 8 + 4 + 1; /*IRQMASK, DMA has low priority*/
#endif
	}
#endif
	
#ifndef NRP_UART_DMA_RX	
	IEN0_URX1IE = 1;
#endif
	nrp_uart_txempty = 1;
}

int16_t nrp_uart_get(void)
{
	int16_t byte;
	
	if (nrp_uart_rx_rd != nrp_uart_rx_wr)
	{
		uint16_t tmp = nrp_uart_rx_rd;
		
		byte = nrp_uart_rx_buffer[tmp++];
		if (tmp >= NRP_UART_RX_LEN) tmp = 0;
		nrp_uart_rx_rd= tmp;
		
		byte &= 0xFF;
	}
	else
	{
//		xSemaphoreTake( nrp_rx_lock, 0);

		byte = -1;
	}
	return byte;
}

int16_t nrp_uart_get_blocking(uint16_t time)
{
	int16_t byte = -1;
	uint8_t time_left = time / portTICK_RATE_MS;
	
	time_left >>= 2;
	if (!time_left) time_left = 1;
	
	while (byte == -1)
	{
#ifdef HAVE_DEBUG
		if (nrp_uart_error != 0)
		{
			debug("U!");
			nrp_uart_error = 0;
		}
#endif
		if (nrp_uart_rx_rd != nrp_uart_rx_wr)
		{
			uint16_t tmp = nrp_uart_rx_rd;
			
			byte = nrp_uart_rx_buffer[tmp++];
			if (tmp >= NRP_UART_RX_LEN) tmp = 0;
			nrp_uart_rx_rd= tmp;			
			
			byte &= 0xFF;
		}
		else if (time_left)
		{
			vTaskDelay(( portTickType ) 4);
			time_left --;
		}
		else
		{	/*timeout*/
			return -1;
		}
	}

	return byte;
}

int8_t nrp_uart_put(uint8_t byte)
{
	int8_t retval = 0;
	uint8_t new_ptr = nrp_uart_tx_wr +1;

	new_ptr &= NRP_UART_TX_MASK;
	if (new_ptr == nrp_uart_tx_rd)
	{
		retval = -1;
	}
	else
	{	
		nrp_uart_tx_buffer[nrp_uart_tx_wr] = byte;
		nrp_uart_tx_wr = new_ptr;
#ifdef NRP_UART_DMA_TX
		nrp_uart_txempty = 0;
#else		
		if ((IEN2 & UTX1IE) == 0)
		{
			nrp_uart_txempty = 0;
			IRCON2_UTX1IF = 0;
			IEN2 |= UTX1IE;
			IRCON2_UTX1IF = 1;
		}
#endif
	}

	return retval;
}

void nrp_uart_launch(void)
{
#ifdef NRP_UART_DMA_TX		
	if (nrp_uart_tx_wr)
	{	/*launch DMA*/
		dma_conf[3].len_l = nrp_uart_tx_wr;
		nrp_uart_tx_wr = 0;
		nrp_uart_tx_rd = 0;
		nrp_uart_txempty = 0;
		DMAARM = (1 << 4);
		DMAREQ = (1 << 4);
	}
#endif
	return;
}

void nrp_uart_rx_reset(void)
{
#ifdef NRP_UART_DMA_RX		
	DMAARM = 0x80 +(1 << 3);
	nrp_tag_mode = 0;
	dma_conf[2].len_l = 4;
	nrp_uart_rx_dma_len = 4;
	DMAARM = (1 << 3);
#endif
}

#ifdef NRP_UART_DMA_RX		
/**
 * UART Rx DMA interrupt service routine.
 * for UART 1
 */
void nrp_uart_rx_dma_callback( void )
{
	uint8_t *d_ptr = nrp_uart_rx_buffer;
	uint8_t *s_ptr = nrp_uart_dma_buffer;
	uint16_t len = dma_conf[2].len_l;
	uint16_t wr_ptr = nrp_uart_rx_wr;
	d_ptr += wr_ptr;
	
	while(len)
	{
		*d_ptr = *s_ptr++;
		wr_ptr++;
		if (wr_ptr >= NRP_UART_RX_LEN)
		{	
			wr_ptr = 0;
			d_ptr = nrp_uart_rx_buffer;
		}
		else d_ptr++;
		len--;
		nrp_uart_rx_wr = wr_ptr;
	}
	 
	if (nrp_tag_mode == 0)
	{	/*get header*/
		s_ptr -= 4;
		if ((*s_ptr++ == 'N') && (*s_ptr++ == 'R')  && (*s_ptr++ == 'P'))
		{
			nrp_tag_mode = 1;
			nrp_uart_rx_dma_len = 3;
			dma_conf[2].len_l = 3;
		}
		else
		{
			nrp_tag_mode = 0;
			nrp_uart_rx_dma_len = 4;
			dma_conf[2].len_l = 4;
		}
	}
	else if (nrp_tag_mode == 1)
	{	/*get tag and len*/
		s_ptr-=2;
		nrp_uart_rx_dma_len = *s_ptr;
		nrp_uart_rx_dma_len <<= 8;
		nrp_uart_rx_dma_len += *s_ptr;
		if (nrp_uart_rx_dma_len)
		{
			dma_conf[2].len_l = nrp_uart_rx_dma_len;
			if (nrp_uart_rx_dma_len > 0x7F)
			{
				dma_conf[2].len_l = 0x7F;
			}
			if (nrp_uart_dma_buffer[0] & 0x80)
			{
				nrp_tag_mode = 3;
			}
			else
			{
				nrp_tag_mode = 2;
			}
		}
		else
		{
			nrp_uart_rx_dma_len = 4;
			dma_conf[2].len_l = 4;
			nrp_tag_mode = 0;
		}
	}
	else if (nrp_tag_mode >= 2)
	{	/*get tag data*/
		nrp_uart_rx_dma_len -= dma_conf[2].len_l;
		if (nrp_uart_rx_dma_len)
		{	/*stuff left*/
			dma_conf[2].len_l = nrp_uart_rx_dma_len;
			if (nrp_uart_rx_dma_len > sizeof(nrp_uart_dma_buffer-8))
			{
				dma_conf[2].len_l = sizeof(nrp_uart_dma_buffer-8);
			}
		}
		else 
		{	/*all data in*/
			if (nrp_tag_mode == 3)
			{	/*last tag*/
				nrp_tag_mode = 0;
				dma_conf[2].len_l = 4;
				nrp_uart_rx_dma_len = 4;
			}
			else
			{	/*next tag len*/
				nrp_tag_mode = 1;
				dma_conf[2].len_l = 3;
				nrp_uart_rx_dma_len = 3;
			}
		}
	}
	else
	{
		nrp_tag_mode = 0;
		dma_conf[2].len_l = 4;
		nrp_uart_rx_dma_len = 4;
	}
	DMAARM = (1 << 3);
}
#else
void nrp_uart_rx_check(void)
{
	if (U1CSR & U_RXB)
	{ 
		if ((U1CSR & (U_FE | U_ERR)) == 0)
		{
			nrp_uart_rx_buffer[nrp_uart_rx_wr++] = U1BUF;
			nrp_uart_rx_wr &= NRP_UART_RX_MASK;
		}
		else
		{
			uint8_t dummy = U1BUF;
			U1CSR &= ~(U_FE | U_ERR);
			nrp_uart_error = 1;
		}
		TCON_URX1IF = 0;
	}
}

/**
 * UART RX interrupt service routine
 * for UART 1
 */

void nrp_rxISR( void ) interrupt (URX1_VECTOR)
{
	EA = 0;
	TCON_URX1IF = 0;
	if (U1CSR & U_RXB)
	{
		if ((U1CSR & (U_FE | U_ERR)) == 0)
		{
		nrp_uart_rx_buffer[nrp_uart_rx_wr++] = U1BUF;
		nrp_uart_rx_wr &= NRP_UART_RX_MASK;
		}
		else
		{
			uint8_t dummy = U1BUF;
			U1CSR &= ~(U_FE | U_ERR);
			nrp_uart_error = 1;
		}
	}
	EA = 1;
//  xSemaphoreGiveFromISR( nrp_rx_lock, pdFALSE ); /*free lock*/
}
#endif

#ifdef NRP_UART_DMA_TX		

/**
 * UART Tx DMA interrupt service routine.
 * for UART 1
 */
void nrp_uart_tx_dma_callback( void )
{
	nrp_uart_txempty = 1;
}
#else
/**
 * UART Tx interrupt service routine.
 * for UART 1
 */
void nrp_txISR( void ) interrupt (UTX1_VECTOR)
{
	IRCON2_UTX1IF = 0;

	if (nrp_uart_tx_rd != nrp_uart_tx_wr)
	{
		U1BUF = nrp_uart_tx_buffer[nrp_uart_tx_rd];
		nrp_uart_tx_rd++;
		nrp_uart_tx_rd &= NRP_UART_TX_MASK;
	}
	else
	{
		IEN2 &= ~UTX1IE;
		nrp_uart_txempty = 1;

	}
}
#endif
