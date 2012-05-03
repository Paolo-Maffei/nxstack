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


/*
    NanoStack: MCU software and PC tools for sensor networking.
		
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
		PO Box 1
		90571 Oulu, Finland

		E-mail:
		info@sensinode.com
*/


/**
 *
 * \file     dri.c
 * \brief    DRI (Direct Radio Interface) module.
 *
 *  This module allows the future libpcap plugin (pcap-sensinode.c)
 *	to receive packets directly from the rf driver of a Sensinode
 *	micro series board.
 *
 *	The module still lacks more or less all the documentation, but those
 *	will be added later (probably in the NanoStack v1.1 release).
 *
 *	The interface between the pcap-sensinode and the dri module is very 
 *	rudimentary and is subject to changes in later releases. At the moment
 *	a packet is preceeded by a four byte header (0x44,0x52,0x49,0xLL) 
 *	where the last byte is the lenght of the payload, including the two 
 *	trailing bytes (RSSI, FCS ok/not ok and Corr.)
 *
 */



#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "debug.h"
#include "stack.h"
#include "buffer.h"
#include "bus.h"
#include "gpio.h"
#include "module.h"
//#include "neighbor_routing_table.h"
//#include "control_message.h"
#include "event_timer.h"
#include "rf.h"
#include "rf_802_15_4.h"
	
	
#include "semphr.h"
#include <signal.h>
#include "powersave.h"
	
/*
[NAME]
DRI

[ID]
MODULE_DRI,

[INFO]
#ifdef HAVE_DRI
  {dri_init, dri_handle, dri_check, 0, MODULE_DRI, 0, ADDR_NONE, 0 },
#endif

[FUNCS]*/
extern portCHAR dri_init(buffer_t *buf);
extern portCHAR dri_handle( buffer_t *buf );
extern portCHAR dri_check( buffer_t *buf );

extern xQueueHandle     buffers;

#ifndef DRI_SPEED
#define DRI_SPEED 230400
#endif

uint8_t dri_tx_buffer[256];
uint8_t dri_tx_wr, dri_tx_rd;

static volatile uint8_t dri_txempty = pdTRUE;
xSemaphoreHandle dri_lock = NULL;

/* Global */	
xQueueHandle dri_queue;

void vdri_task( void *pvParameters );

#ifndef HAVE_DEBUG
interrupt (UART1RX_VECTOR) uart1_rxISR( void );
interrupt (UART1TX_VECTOR) uart1_txISR( void );
#endif

/**
 *  Standard nanostack module initalizer.
 *
 *  \return  pdTRUE    OK
 */
portCHAR dri_init( buffer_t *buf )
{
	unsigned long rate;
	uint8_t clock_sel;
	dri_tx_wr = dri_tx_rd = 0;
	uint8_t mod = 0;

#ifndef HAVE_DEBUG
	clock_sel = (SSEL0 | SSEL1);
	rate = configCPU_CLOCK_HZ / DRI_SPEED;
		
	U1CTL |= SWRST;
	P3SEL |= (BIT6|BIT7);
	P3DIR |= BIT6;					/* Use P3.6 as TX */
	P3DIR &= ~BIT7;					/* Use P3.7 as RX */

	P5DIR &= ~(BIT2|BIT1);	/* SPI pins are hooked into these on micro.4*/

	U1CTL = CHAR + SWRST;	/* 8N1 */
	U1TCTL = clock_sel;

	U1MCTL = mod;

	/* Setup baud rate */
	U1BR0 = (uint8_t) ( rate );
	rate >>= 8;
	U1BR1 = (uint8_t) ( rate );

	/* Module enable */
	ME2 |= UTXE1 + URXE1;
	U1CTL &= ~SWRST;
	/* IRQ flags */
	IFG2 |= UTXIFG1;
	IFG2 &= ~URXIFG1;

	dri_txempty = pdTRUE;

	/* Enable interrupts. */
	IE2 |= URXIE1 + UTXIE1;
	portEXIT_CRITICAL();
	
	vSemaphoreCreateBinary( dri_lock );
	xSemaphoreGive(dri_lock);	

	dri_txempty = pdFALSE;
#endif

	debug("rf_init()\r\n\r\n");
	rf_init();

#ifndef HAVE_DEBUG
	dri_queue = xQueueCreate( 5, sizeof( buffer_t * ) );
#endif
	debug("creating DRI task\r\n\r\n");
	xTaskCreate( vdri_task, "DRI", 256, NULL, (tskIDLE_PRIORITY + 5 ), NULL );
	return pdTRUE;
}

#ifndef HAVE_DEBUG
/**
 *	Interrupt TX function
 *
 *	\brief This function is not compiled when HAVE_DEBUG is set.
 *
 */
interrupt (UART1TX_VECTOR) uart1_txISR( void )
{
	debug("uart1_txISR\r\n\r\n");
	if (dri_tx_rd == dri_tx_wr)
	{
		debug("uart1_txISR - EQ\r\n\r\n");
		dri_txempty = pdTRUE;
		//LED1_OFF();
		xSemaphoreGiveFromISR( dri_lock, pdFALSE );		
	}
	else
	{
		debug("uart1_txISR - NEQ\r\n\r\n");
		U1TXBUF = dri_tx_buffer[dri_tx_rd++];
	}
}

/**
 *	TX start function
 *
 *	\brief Start the UART TX
 *	\brief This function is not compiled when HAVE_DEBUG is set.
 *
 */
void dri_tx_start(void)
{	/*start buffer transmit*/
	uint8_t c;

	debug("dri_tx_start()\r\n\r\n");

//	LED1_ON();
//	vTaskDelay(500 / portTICK_RATE_MS );
//	LED1_OFF();
//	vTaskDelay(500 / portTICK_RATE_MS );
//	LED1_ON();
//	vTaskDelay(500 / portTICK_RATE_MS );
//	LED1_OFF();
	
	if (dri_tx_rd != dri_tx_wr)
	{
		/* Start TX shifter. */
		dri_txempty = pdFALSE;
		c = dri_tx_buffer[dri_tx_rd++];	
		U1TXBUF = c;
	}
}
#endif

/**
 *	DRI handle function
 *
 *	\brief This function receives buffer from the RF driver, adds a four byte preamble and sends it to UART
 *	\brief This function is not compiled when HAVE_DEBUG is set.
 *
 *	\param buf	A pointer to a standard buffer_t structure containing the packet.
 *
 */
portCHAR dri_handle( buffer_t *buf )
{
	uint8_t f_type;
	uint8_t tmp, tx_index=0, ind=0;

#ifndef HAVE_DEBUG
	dri_tx_wr = dri_tx_rd = 0;
#endif	
	debug("received something...\r\n\r\n");
#ifndef HAVE_DEBUG
/*	dri_tx_buffer[tx_index++] = 0x36;
	dri_tx_buffer[tx_index++] = 0x36;
	dri_tx_buffer[tx_index++] = 0x36;
	dri_tx_wr = 3;
	dri_tx_start();
*/
#endif

	if(buf && (buf->dir == BUFFER_DOWN))
	{
		/* Discard packet going down here! */
		stack_buffer_free(buf);
		buf=0;
	}
	else if(buf &&  (buf->dir == BUFFER_UP))
	{
#ifndef HAVE_DEBUG
		tmp = buf->buf[buf->buf_ptr];
		f_type = (tmp & FC_FRAME_TYPE_MASK);
		if(buf->options.rf_lqi > 0) 												
		{
			if((f_type == FC_ACK_FRAME) || (f_type == FC_DATA_FRAME ) || (f_type == FC_BEACON_FRAME ))
			{
				uint16_t tmp_16;
				tmp_16 = buf->buf_end - buf->buf_ptr; /*length of data*/

				debug("forwarding packet...\r\n\r\n");

				dri_tx_wr = tmp_16+5;

				dri_tx_buffer[tx_index++] = 0x44;		/*	'D'	*/
				dri_tx_buffer[tx_index++] = 0x52;		/*	'R'	*/
				dri_tx_buffer[tx_index++] = 0x49;		/*	'I'	*/
				dri_tx_buffer[tx_index++] = tmp_16;	/*	Data length	*/

				while (tmp_16)
				{
					dri_tx_buffer[tx_index++] = buf->buf[ind++];
					tmp_16--;
				}

				memcpy(&(dri_tx_buffer[tx_index++]), &(buf->options.rf_dbm), 1);		/*	RSSI	*/

				dri_tx_start();

				stack_buffer_free(buf);
				buf=0;
			}
			else
			{
				/* Discard buffer if type is not 802.15.4-standard compliant */
				stack_buffer_free(buf);
				buf=0;
			}
		}
		else	
		{
			/* Discard buffer if LQI is zero */
			stack_buffer_free(buf);
			buf=0;
		}
#else
		debug("Got packet going UP!\n");
		stack_buffer_free(buf);
#endif
	}
	return pdTRUE; 
}

portCHAR dri_check( buffer_t *buf )
{
	return pdFALSE;
}

/**
 *	DRI task
 *
 *	\brief This function is not doing anything
 *	\brief This function is not compiled when HAVE_DEBUG is set.
 *
 *	\param pvParameters	stadard parameter pointer for a new task
 */
void vdri_task( void *pvParameters )
{
	sockaddr_t device_own_address;
	buffer_t *buf = 0;
	uint8_t i;

	rf_rx_enable();
	
	for(;;)
	{
		vTaskDelay(1500 / portTICK_RATE_MS );
	}
}

