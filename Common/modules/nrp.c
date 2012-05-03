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
 * \file     nrp.c
 * \brief    nRP protocol module.
 *
 *  The nRoute protocol module: handler functions.
 *   
 */


#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <signal.h>

#include "stack.h"
#include "buffer.h"

#include "module.h"

//#include "debug.h"
#include "nrp.h"
#include "gpio.h"
#include "powersave.h"

/*start module config*/
/*
[NAME]
NRP

[ID]
MODULE_NRP,

[INFO]
#ifdef HAVE_NRP
  {nrp_init, nrp_handle, nrp_check, 0, MODULE_NRP, 0, ADDR_NONE, 0 },
#endif

[FUNCS]*/
extern portCHAR nrp_init(buffer_t *buf);
extern portCHAR nrp_handle( buffer_t *buf );
extern portCHAR nrp_check( buffer_t *buf );


/*end module config*/

#ifdef HAVE_NRP
#ifndef portACLK_FREQUENCY_HZ
#define portACLK_FREQUENCY_HZ			( ( unsigned portLONG ) 32768 )
#endif

#ifndef NRP_SPEED
#define NRP_SPEED 115200
#endif

#ifndef NRP_FILTERS
#define NRP_FILTERS 4
#endif

#ifdef HAVE_POWERSAVE
static powerHandle nrp_ph = 0;
#endif

/**
 * NRP TX lock.
 *
 */
 
xSemaphoreHandle nrp_lock = NULL;

static volatile uint8_t nrp_txempty = pdTRUE;
int8_t nrp_init_done = -1;

#ifdef HAVE_DEBUG
#if 0
int8_t debug_string(uint8_t *ptr);
int8_t debug_string_constant(prog_char *s);
int16_t debug_read(void);
uint8_t debug_buffer[64];

uint8_t nrp_debug_buffer[256];
uint8_t nrp_debug_rd, nrp_debug_wr;
void nrp_debug_push(void);
#endif
#endif

uint8_t nrp_tx_buffer[256];
uint8_t nrp_rx_buffer[256];

uint8_t nrp_tx_rd, nrp_tx_wr, nrp_rx_rd, nrp_rx_wr;

uint8_t nrp_timer_id;

static xQueueHandle nrp_rx_queue = 0; 
void vnrp_task( void *pvParameters );

void nrp_rx(void *param);
void nrp_tx_start(void);
void nrp_pack_address(nrp_data_tag_t tag, sockaddr_t *sa, uint8_t *tx_ind);
void nrp_parse_address(sockaddr_t *sa, uint8_t *tag, uint8_t tag_len);

module_id_t nrp_filter_protocol[NRP_FILTERS];
sockaddr_t nrp_filter_address[NRP_FILTERS];

portCHAR nrp_data(buffer_t *buf);
portCHAR nrp_control(buffer_t *buf);

void nrp_filter_init(void);
portCHAR nrp_filter_match(buffer_t *buf);
int8_t nrp_filter_add(module_id_t module, sockaddr_t *sa);

/** Interrupts */
interrupt (UART1RX_VECTOR) uart1_rxISR( void );
interrupt (UART1TX_VECTOR) uart1_txISR( void );

/**
 * Initialize nRP module and UART 1.
 *
 *
 * \return SUCCESS
 * \return FAILURE	insufficient memory
 */
portCHAR nrp_init(buffer_t *buf)
{
	unsigned long rate;
	uint8_t clock_sel;
	uint8_t mod = 0;
		
	if (nrp_init_done > 0) return pdTRUE;

	nrp_tx_rd = nrp_tx_wr = nrp_rx_rd = nrp_rx_wr = 0;
	
	portENTER_CRITICAL();
#ifdef HAVE_POWERSAVE
	if (nrp_ph == 0)
	{
		powersave_alloc(&nrp_ph);
		powersave_set(nrp_ph, LPM1);
	}
#endif
	
	clock_sel = (SSEL0 | SSEL1);
	rate = configCPU_CLOCK_HZ / NRP_SPEED;
		
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

	nrp_txempty = pdTRUE;

	/* Enable interrupts. */
	IE2 |= URXIE1 + UTXIE1;
	portEXIT_CRITICAL();
	
	vSemaphoreCreateBinary( nrp_lock );
	xSemaphoreGive(nrp_lock);	

	nrp_txempty = pdFALSE;

	nrp_rx_queue = xQueueCreate( 100, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
	xTaskCreate( vnrp_task, "NRP", configMINIMAL_STACK_SIZE + 150, NULL, 
				(tskIDLE_PRIORITY + 3 ), NULL );

	nrp_init_done = 1;
	nrp_filter_init();
	
	return pdTRUE;	
}

/**
 * Parse nRP packets.
 *
 * \param param pointer to buffer end
 *
 */
void nrp_rx(void *param)
{
	uint8_t end, ind,k;
	uint8_t byte;
	nrp_config_tag_t conf_tag;
	nrp_data_tag_t tag;
	buffer_t *buffer = 0;
	uint16_t tag_len;
	uint16_t i = (uint16_t) param;
	nrp_type_t type;
	
	end = (uint8_t) i & 0xff;
	ind = (i >> 8);
	byte = nrp_rx_buffer[ind++];
	type = byte & 0x0F;
	
	byte = end - ind;

	//debug_string("nrp_rx\r\n");

	switch(type)
	{
		case NRP_CONFIG:
			buffer = stack_buffer_get(5000);
			if (buffer)
			{
				buffer->from = MODULE_APP;
				buffer->to = MODULE_NRP;
				buffer->options.type = BUFFER_CONTROL;

				buffer->src_sa.addr_type = ADDR_NONE;
				buffer->dst_sa.addr_type = ADDR_NONE;
				buffer->src_sa.port = 0;
				buffer->dst_sa.port = 0;

				conf_tag = nrp_rx_buffer[ind++];
				tag_len = nrp_rx_buffer[ind++];
				tag_len <<= 8;
				tag_len += nrp_rx_buffer[ind++];

				switch (conf_tag & 0x7F)
				{
					case NRPC_SUBSCRIBE:
					case NRPC_RF_ID:
					case NRPC_PROTOCOL_ID:
					case NRPC_RESET:
					case NRPC_VERSION:
						buffer_push_uint8(buffer,(conf_tag & 0x7F));
						ind += tag_len;
						break;

					default:	/*unknown config, ignore*/
						ind = end;		
						nrp_rx_rd = end;
						break;
				}
				if (conf_tag & 0x80) 
				{
					ind = end;		
					nrp_rx_rd = end;
				}
			}
			else
			{
				ind = end;		
				nrp_rx_rd = end;
				return;
			}

		case NRP_DATA:	/*parse data packet*/
			if (!buffer)
			{
				buffer = stack_buffer_get(50);
				if (!buffer) 
				{
					nrp_rx_rd = end;
					return;
				}
				buffer->buf_ptr = 22;	/*this should be fetched from module def*/
				buffer->buf_end = 22;
				buffer->from = MODULE_NRP;
				buffer->to = MODULE_NONE;
				buffer->dir = BUFFER_DOWN;
				buffer->options.type = BUFFER_DATA;
				buffer->src_sa.addr_type = ADDR_NONE;
				buffer->dst_sa.addr_type = ADDR_NONE;
				buffer->socket = 0;
			}

			while (ind != end)
			{
				
				tag = nrp_rx_buffer[ind++];
				tag_len = nrp_rx_buffer[ind++];
				tag_len <<= 8;
				tag_len += nrp_rx_buffer[ind++];

				switch (tag & 0x7F)
				{
					case NRP_PDATA:
						while (tag_len)
						{
							buffer->buf[buffer->buf_end++] = nrp_rx_buffer[ind++];
							tag_len--;
						}
						break;
						
					case NRP_SRC:
						if (tag_len != 0)
						{
							nrp_parse_address(&(buffer->src_sa), &(nrp_rx_buffer[ind]), tag_len);
							ind += tag_len;
							tag_len = 0;
						}
						else
						{
							buffer->src_sa.addr_type = ADDR_NONE;
						}
						break;
						
					case NRP_DST:
						if (tag_len != 0)
						{
							nrp_parse_address(&(buffer->dst_sa), &(nrp_rx_buffer[ind]), tag_len);
							ind += tag_len;
							tag_len = 0;
						}
						else
						{
							buffer->dst_sa.addr_type = ADDR_NONE;
						}
						break;
						
					case NRP_PROTOCOL:
						i = 0;
						if ((tag_len == 1) && (buffer->options.type == BUFFER_CONTROL))
						{
							buffer_push_uint8(buffer, NRP_PROTOCOL);
							buffer_push_uint8(buffer, nrp_rx_buffer[ind++]);
							tag_len--;
						}
						else if ((tag_len == 1))
						{
							nrp_protocol_t proto;
							proto = nrp_rx_buffer[ind++];
							switch(proto)
							{
								case NRP_PROTO_802_15_4:
									buffer->to = MODULE_RF_802_15_4;
									break;

#ifdef HAVE_NUDP
								case NRP_PROTO_NUDP:
									buffer->to = MODULE_NUDP;
									break;
#endif
#ifdef HAVE_CUDP
								/*todo: move header headroom handling to 6lowpan modules*/
								case NRP_PROTO_6LOWPAN:
									buffer->to = MODULE_CUDP;
									break;
#endif									
#ifdef HAVE_ZIGBEE
								case NRP_PROTO_ZIGBEE:
									buffer->to = MODULE_ZIGBEE;
									break;
#endif
								default:
									buffer->to = MODULE_NONE;
									break;
							}
							tag_len--;
						}
						else goto nrp_tag_eject;
						break;
						
					case NRP_SRC_PORT:
						i = 0;
						if (tag_len == 2)
						{
							buffer->src_sa.port = nrp_rx_buffer[ind++];
							buffer->src_sa.port <<= 8;
							buffer->src_sa.port += nrp_rx_buffer[ind++];
							tag_len -= 2;
						}
						else goto nrp_tag_eject;
						break;
						
					case NRP_DST_PORT:
						i = 0;
						if (tag_len == 2)
						{
							buffer->dst_sa.port = nrp_rx_buffer[ind++];
							buffer->dst_sa.port <<= 8;
							buffer->dst_sa.port += nrp_rx_buffer[ind++];
							tag_len -= 2;
						}
						else goto nrp_tag_eject;
						break;
						
					default:
nrp_tag_eject:
						ind += tag_len;
				}
				if (tag & 0x80) nrp_rx_rd = end;
			}
			break;
					
		default:
			nrp_rx_rd = end;
	}
	if (buffer)
	{
#ifdef NRP_LOOPBACK
		if (buffer->options.type == BUFFER_DATA)
		{
			buffer_t *b = stack_buffer_get(0);
			if (b)
			{
				memcpy(b, buffer, sizeof(buffer_t)+buffer->size);
				b->from = b->to;
				b->to = MODULE_NRP;
				b->dir = BUFFER_UP;
				stack_buffer_push(b);
			}
		}
#endif
		stack_buffer_push(buffer);
	}
	
}

void	nrp_pack_address(nrp_data_tag_t tag, sockaddr_t *sa, uint8_t *tx_ind)
{
	uint8_t i;
	
	//debug_string("nrp_pack_addr\r\n");

	switch(sa->addr_type)
	{
		case ADDR_802_15_4_PAN_LONG:
			if ((sa->address[8] != 0xFF) || (sa->address[9] != 0xFF))
			{
				nrp_tx_buffer[(*tx_ind)++] = tag;
				nrp_tx_buffer[(*tx_ind)++] = 0;
				nrp_tx_buffer[(*tx_ind)++] = 3;
				nrp_tx_buffer[(*tx_ind)++] = NRP_802_15_4_PAN;
				nrp_tx_buffer[(*tx_ind)++] = sa->address[9];
				nrp_tx_buffer[(*tx_ind)++] = sa->address[8];
			}
			/*break missing intentionally*/
		case ADDR_802_15_4_LONG:
			nrp_tx_buffer[(*tx_ind)++] = tag;
			nrp_tx_buffer[(*tx_ind)++] = 0;
			nrp_tx_buffer[(*tx_ind)++] = 9;
			nrp_tx_buffer[(*tx_ind)++] = NRP_802_15_4_LONG;
			for (i=0; i < 8; i++)
			{
				nrp_tx_buffer[(*tx_ind)++] = sa->address[7-i];
			}
			break;
			
		case ADDR_802_15_4_PAN_SHORT:
			if ((sa->address[2] != 0xFF) || (sa->address[3] != 0xFF))
			{
				nrp_tx_buffer[(*tx_ind)++] = tag;
				nrp_tx_buffer[(*tx_ind)++] = 0;
				nrp_tx_buffer[(*tx_ind)++] = 3;
				nrp_tx_buffer[(*tx_ind)++] = NRP_802_15_4_PAN;
				nrp_tx_buffer[(*tx_ind)++] = sa->address[3];
				nrp_tx_buffer[(*tx_ind)++] = sa->address[2];
			}
			/*break missing intentionally*/
		case ADDR_802_15_4_SHORT:
			nrp_tx_buffer[(*tx_ind)++] = tag;
			nrp_tx_buffer[(*tx_ind)++] = 0;
			nrp_tx_buffer[(*tx_ind)++] = 3;
			nrp_tx_buffer[(*tx_ind)++] = NRP_802_15_4_SHORT;
			nrp_tx_buffer[(*tx_ind)++] = sa->address[1];
			nrp_tx_buffer[(*tx_ind)++] = sa->address[0];
			break;
			
		case ADDR_BROADCAST:
			nrp_tx_buffer[(*tx_ind)++] = tag;
			nrp_tx_buffer[(*tx_ind)++] = 0;
			nrp_tx_buffer[(*tx_ind)++] = 3;
			nrp_tx_buffer[(*tx_ind)++] = NRP_802_15_4_SHORT;
			nrp_tx_buffer[(*tx_ind)++] = 0xFF;
			nrp_tx_buffer[(*tx_ind)++] = 0xFF;
			nrp_tx_buffer[(*tx_ind)++] = tag;
			nrp_tx_buffer[(*tx_ind)++] = 0;
			nrp_tx_buffer[(*tx_ind)++] = 3;
			nrp_tx_buffer[(*tx_ind)++] = NRP_802_15_4_PAN;
			nrp_tx_buffer[(*tx_ind)++] = 0xFF;
			nrp_tx_buffer[(*tx_ind)++] = 0xFF;
			break;
		default:
			break;
	}
}

void nrp_parse_address(sockaddr_t *sa, uint8_t *tag, uint8_t tag_len)
{
	uint8_t len;

	//debug_string("nrp_parse_addr\r\n");


	switch (*tag)
	{
		case NRP_802_15_4_LONG:
			len = 8;
			if (sa->addr_type == ADDR_PAN)
			{
				sa->addr_type = ADDR_802_15_4_PAN_LONG;
			}
			else
			{
				sa->addr_type = ADDR_802_15_4_PAN_LONG;
				sa->address[8] = 0xFF;
				sa->address[9] = 0xFF;
			}
			tag++;
			break;
			
		case NRP_802_15_4_SHORT:
			len = 2;
			if (sa->addr_type == ADDR_PAN)
			{
				sa->addr_type = ADDR_802_15_4_PAN_SHORT;
				sa->address[3] = sa->address[9];
				sa->address[2] = sa->address[8];
			}
			else
			{
				sa->addr_type = ADDR_802_15_4_PAN_SHORT;
				sa->address[2] = 0xFF;
				sa->address[3] = 0xFF;
			}
			tag++;
			break;
			
		case NRP_802_15_4_PAN:
			tag++;
			sa->address[9] = *tag++;
			sa->address[8] = *tag++;
			if (sa->addr_type == ADDR_802_15_4_PAN_SHORT)
			{
				sa->address[3] = sa->address[9];
				sa->address[2] = sa->address[8];
			}
			else if (sa->addr_type == ADDR_802_15_4_PAN_LONG)
			{
			}
			else
			{
				sa->addr_type = ADDR_PAN;
				sa->address[1] = sa->address[9];
				sa->address[0] = sa->address[8];
			}
			return;
			
		default:
			sa->addr_type = *tag++;
			memcpy(sa->address, tag, tag_len-1);
			return;
	}
	while(len)
	{
		len--;
		sa->address[len] = *tag++;
	}
}


void nrp_tx_start(void)
{	/*start buffer transmit*/
	uint8_t c;

	//debug_string("nrp_tx_start\r\n");
	
	if (nrp_tx_rd != nrp_tx_wr)
	{
		/* Start TX shifter. */
		nrp_txempty = pdFALSE;
		c = nrp_tx_buffer[nrp_tx_rd++];	
		U1TXBUF = c;
		LED1_ON();
	}
}

/**
 *  The nRP buffer handler.
 *
 *	\param buf pointer to buffer
 *
 *  \return  pdTRUE
 */
portCHAR nrp_handle( buffer_t *buf )
{  /* Process the packet */

	//debug_string("nrp_handle\r\n");

	switch(buf->dir)
	{
		case BUFFER_DOWN:	
			if (xSemaphoreTake( nrp_lock, 1) == pdFALSE)
			{	/*tx not complete, reschedule*/
				stack_buffer_push(buf);
				return pdTRUE;
			}
			else if (buf->options.type == BUFFER_CONTROL)
			{
				nrp_control(buf);
			}
			else
			{
				stack_buffer_free(buf);
				xSemaphoreGive(nrp_lock);	
			}
			break;
/*( portTickType ) 20 / portTICK_RATE_MS*/ 
		case BUFFER_UP:			/* From driver */
			LED2_OFF();
			LED1_OFF();
			if (xSemaphoreTake( nrp_lock, 1) == pdFALSE)
			{	/*tx not complete, reschedule*/
				stack_buffer_push(buf);
				return pdTRUE;
			}
			else
			{
				if (buf->options.type == BUFFER_CONTROL)
				{
					nrp_control(buf);
				}
				else
				{
					if (nrp_filter_match(buf) == pdFALSE)
					{	/*not subscribed*/

						stack_buffer_free(buf);
						xSemaphoreGive(nrp_lock);	
					}
					else
					{
						nrp_data(buf);
					}
				}
			}
			break;

	} 
  return pdTRUE;  
}

/**
 *  The nRP buffer checker.
 *
 *	\param buf pointer to buffer
 *
 *  \return  pdTRUE 
 *  \return  pdFALSE 
 */
portCHAR nrp_check( buffer_t *buf )
{
	buffer_t *dup;
	
	switch (buf->from)
	{
#ifdef HAVE_NUDP		
		case MODULE_NUDP:
			return pdTRUE;
#endif
#ifdef HAVE_CUDP
{
	uint8_t i;

	//debug_string("nrp_check\r\n");

	
	for (i=0; i< NRP_FILTERS; i++)
	{
		
		uint8_t match = 0;
		
		if (nrp_filter_protocol[i] != MODULE_NONE)
		{
			
			if (nrp_filter_protocol[i] == (uint8_t) buf->from)
			{	/*protocol match*/

				
				if (nrp_filter_address[i].addr_type != ADDR_NONE)
				{
					if (nrp_filter_address[i].addr_type == buf->dst_sa.addr_type)
					{
						if (memcmp(nrp_filter_address[i].address, buf->dst_sa.address, 8) == 0)
						{
							match = 1;
						}
					}
				}
				else
				{
				 match = 1; 	/*no address in filter, match*/
				}
				if (nrp_filter_address[i].port != 0)
				{	
					if (nrp_filter_address[i].port == buf->dst_sa.port)
					{
						match++;
					}
				}
				else match++;			/*no port in filter, match*/
				if (match >= 2) return pdTRUE;
			}
		}
	}
	return pdFALSE;
}
		case MODULE_CUDP:
			return pdTRUE;
#endif
		case MODULE_RF_802_15_4:
			dup = stack_buffer_get(20);
			if (dup != 0)
			{
				memcpy(dup, buf, sizeof(buffer_t) + buf->buf_end);
				dup->to = MODULE_NRP;
				stack_buffer_push(dup);
			}
			break;

		default:
			break;
	}
	return pdFALSE;  
}

uint8_t nrp_rx_state = 0;
uint16_t nrp_rx_count = 0;
#if 0
void nrp_rx_debug(void *param);
void nrp_rx_debug(void *param)
{
	uint16_t byte = (uint16_t) param;
	
#ifdef NRP_USE_DEBUG
	debug_printf("NRP: rx 0x%2.2X, st %d, ct %d.\r\n", byte, nrp_rx_state, nrp_rx_count);
#else
	byte++;
#endif
}
#endif
/**
 * UART RX interrupt service routine
 * for UART 1
 */
interrupt (UART1RX_VECTOR) uart1_rxISR( void )
{
	uint8_t byte;
	/* Get the character from the UART and post it on the queue of Rxed 
	characters. */
	byte = U1RXBUF;

	if( xQueueSendFromISR(nrp_rx_queue, &byte, pdFALSE ) )
	{
		/*If the post causes a task to wake force a context switch 
		as the woken task may have a higher priority than the task we have 
		interrupted. */
		//taskYIELD();
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

uint8_t nrp_pkt_start = 0;
/**
 * NRP receiver task.
 *
 * Reads packets from serial, timeouts
 *
 * \param pvParameters not used
 */
void vnrp_task( void *pvParameters )
{
  uint8_t byte;
	for (;;)
	{
	  if (xQueueReceive(nrp_rx_queue, &byte, 50 / portTICK_RATE_MS ) == pdTRUE)
		{
			switch (nrp_rx_state)
			{
				case 0: /*idle*/
					if (byte == 'R')
					{
						nrp_rx_state = 1;
						nrp_rx_count = 2;
					}
					break;

				case 1:
					switch(nrp_rx_count)
					{
						case 2:
							if (byte == 'P') nrp_rx_count++;
							else nrp_rx_state = 0;
							break;

						case 3:
							nrp_rx_state = 2;
							nrp_rx_count = 0;
							nrp_pkt_start = nrp_rx_wr;
							nrp_rx_rd = nrp_pkt_start;
							nrp_rx_buffer[nrp_rx_wr++] = byte;
							LED1_ON();
							break;

						default:
							nrp_rx_state = 0;
							break;
					}
					break;

				case 2:
					if (byte & 0x80)
					{	/*end mark*/
						nrp_rx_state = 6;
					}
					else nrp_rx_state = 3;
					nrp_rx_buffer[nrp_rx_wr++] = byte;	/*store tag*/
					break;

				case 6:
				case 3:
					nrp_rx_buffer[nrp_rx_wr++] = byte;	/*store len MSB*/
					nrp_rx_count = byte;
					nrp_rx_count <<= 8;
					nrp_rx_state++;
					break;

				case 7:
				case 4:
					nrp_rx_buffer[nrp_rx_wr++] = byte;	/*store len LSB*/
					nrp_rx_count += byte;
					if (nrp_rx_count)
					{
						nrp_rx_state++;
					}
					else goto nrp_field_end;
					break;

				case 8:
				case 5:
					nrp_rx_buffer[nrp_rx_wr++] = byte;	/*store byte*/
					nrp_rx_count--;
					if (nrp_rx_count == 0)
					{
						goto nrp_field_end;
					}
					break;

		case 80:			
nrp_field_end:		
					if (nrp_rx_state > 6)
					{
						event_t event;
						event.process = &nrp_rx;

						event.param = (void *) (((uint16_t)nrp_pkt_start << 8) + nrp_rx_wr);
						xQueueSend(events, &event, 20);
						nrp_rx_state = 0;
						nrp_rx_count = 0;
						LED1_OFF();
					}
					else
					{
						nrp_rx_state = 2;
						nrp_rx_count = 0;
					}
				default:
					break;
			}
		}
		else
		{	/*timeout*/
			nrp_rx_state = 0;
			nrp_rx_count = 0;
			LED1_OFF();
		}
	}
}

/**
 * UART Tx interrupt service routine.
 * for UART 1
 */
interrupt (UART1TX_VECTOR) uart1_txISR( void )
{
	if (nrp_tx_rd == nrp_tx_wr)
	{
		nrp_txempty = pdTRUE;
		LED1_OFF();
		xSemaphoreGiveFromISR( nrp_lock, pdFALSE );		
	}
	else
	{
		U1TXBUF = nrp_tx_buffer[nrp_tx_rd++];
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}


void nrp_filter_init(void)
{
	uint8_t i;
	for (i=0; i< NRP_FILTERS; i++)
	{
		nrp_filter_protocol[i] = MODULE_NONE;
		nrp_filter_address[i].addr_type = ADDR_NONE;
		nrp_filter_address[i].port = 0;
	}
#ifdef NRP_DEFAULT_NUDP
	{
		sockaddr_t piip_addr;
		
		piip_addr.port = 0;
		piip_addr.addr_type = ADDR_NONE;
		
		nrp_filter_add(MODULE_NUDP, &piip_addr);
	}
#endif
#ifdef NRP_DEFAULT_MAC
	{
		sockaddr_t piip_addr;
		
		piip_addr.port = 0;
		piip_addr.addr_type = ADDR_NONE;
		
		nrp_filter_add(MODULE_RF_802_15_4, &piip_addr);
	}
#endif
}

portCHAR nrp_filter_match(buffer_t *buf)
{
	uint8_t i;
	
	for (i=0; i< NRP_FILTERS; i++)
	{
		
		uint8_t match = 0;
		
		if (nrp_filter_protocol[i] != MODULE_NONE)
		{
			
			if (nrp_filter_protocol[i] == (uint8_t) buf->from)
			{	/*protocol match*/

				
				if (nrp_filter_address[i].addr_type != ADDR_NONE)
				{
					if (nrp_filter_address[i].addr_type == buf->dst_sa.addr_type)
					{
						if (memcmp(nrp_filter_address[i].address, buf->dst_sa.address, 8) == 0)
						{
							match = 1;
						}
					}
				}
				else
				{
					 match = 1; 	/*no address in filter, match*/
				}
				if (nrp_filter_address[i].port != 0)
				{	
					if (nrp_filter_address[i].port == buf->dst_sa.port)
					{
						match++;
					}
				}
				else
				{
					match++;			/*no port in filter, match*/
					
				}
				if (match > 1)
				{
					return pdTRUE;
				}
			}
		}
	}
	return pdFALSE;
}

int8_t nrp_filter_add(module_id_t module, sockaddr_t *sa)
{
	int8_t filter_id = -1;
	uint8_t i;
		
	for (i=0; i< NRP_FILTERS; i++)
	{
		if (nrp_filter_protocol[i] == module)
		{
			if (nrp_filter_address[i].addr_type == sa->addr_type)
			{
				if (nrp_filter_address[i].port == sa->port)
				{
					return -1;
				}
			}
		}
	}
	
	for (i=0; i< NRP_FILTERS; i++)
	{
		if (nrp_filter_protocol[i] == MODULE_NONE)
		{
			nrp_filter_protocol[i] = module;
			nrp_filter_address[i].addr_type = sa->addr_type;
			memcpy(nrp_filter_address[i].address, sa->address, sizeof(nrp_filter_address[i].address));
			nrp_filter_address[i].port = sa->port;
			filter_id = i;
			return i;
		}
	}
	
	return filter_id;
}

portCHAR nrp_control(buffer_t *buf)
{
	nrp_config_tag_t msg_type;
	nrp_type_t type = NRP_CONFIG_REPLY;
	uint8_t tx_index = nrp_tx_wr = nrp_tx_rd = 0;
	uint8_t tag_len;
	
	nrp_tx_buffer[tx_index++] = 'N';
	nrp_tx_buffer[tx_index++] = 'R';
	nrp_tx_buffer[tx_index++] = 'P';
	nrp_tx_buffer[tx_index++] = NRP_VERSION + type;
	
	msg_type = buffer_pull_uint8(buf);
	switch(msg_type)
	{
		case NRPC_VERSION:
			nrp_tx_buffer[tx_index++] = (nrp_config_tag_t) NRPC_VERSION | 0x80;	/*tag*/
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 1;	/*length*/
			nrp_tx_buffer[tx_index++] = NRP_VERSION;			
			break;
		case NRPC_RF_ID:
			nrp_tx_buffer[tx_index++] = (nrp_config_tag_t)NRPC_RF_ID | 0x80;	/*tag*/
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 1;	/*length*/
			nrp_tx_buffer[tx_index++] = 0x01;			
			break;
		case NRPC_PROTOCOL_ID:
			nrp_tx_buffer[tx_index++] = (nrp_config_tag_t)NRPC_PROTOCOL_ID | 0x80;	/*tag*/
			tag_len = 1;
#ifdef HAVE_NUDP
			tag_len++;
#endif
#ifdef HAVE_CUDP
			tag_len++;
#endif
#ifdef HAVE_ZIGBEE
			tag_len++;
#endif
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = tag_len;	/*length*/
			nrp_tx_buffer[tx_index++] = NRP_PROTO_802_15_4;
#ifdef HAVE_NUDP
			nrp_tx_buffer[tx_index++] = NRP_PROTO_NUDP;
#endif
#ifdef HAVE_CUDP
			nrp_tx_buffer[tx_index++] = NRP_PROTO_6LOWPAN;
#endif
#ifdef HAVE_ZIGBEE
			nrp_tx_buffer[tx_index++] = NRP_PROTO_ZIGBEE;
#endif
			break;
			
		case NRPC_RESET:
			nrp_filter_init();
			nrp_tx_buffer[tx_index++] = (nrp_config_tag_t)NRPC_RESET | 0x80;	/*tag*/
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 1;	/*length*/
#ifdef NRP_DEFAULT_MAC
			nrp_tx_buffer[tx_index++] = 1;
#else
#ifdef NRP_DEFAULT_NUDP
			nrp_tx_buffer[tx_index++] = 2;
#else
			nrp_tx_buffer[tx_index++] = 0;
#endif
#endif
			break;
		
		case NRPC_SUBSCRIBE:
			{
				nrp_protocol_t protocol = buffer_pull_uint8(buf);
				module_id_t module = MODULE_NONE;
				protocol = buffer_pull_uint8(buf);
				
				switch(protocol)
				{
					case NRP_PROTO_802_15_4:
						module = MODULE_RF_802_15_4;
						break;
#ifdef HAVE_NUDP
					case NRP_PROTO_NUDP:
						module = MODULE_NUDP;
						break;
#endif					
#ifdef HAVE_CUDP
					case NRP_PROTO_6LOWPAN:
						module = MODULE_CUDP;
						break;
#endif					
#ifdef HAVE_ZIGBEE
					case NRP_PROTO_ZIGBEE:
						module = MODULE_ZIGBEE;
						break;
#endif
					default:
						break;
				}
				nrp_tx_buffer[tx_index++] = (nrp_config_tag_t)NRPC_SUBSCRIBE | 0x80;	/*tag*/
				nrp_tx_buffer[tx_index++] = 0;	/*set*/
				nrp_tx_buffer[tx_index++] = 1;	/*length*/
				if (module != MODULE_NONE)
				{
					nrp_tx_buffer[tx_index++] = nrp_filter_add(module, &(buf->dst_sa));
				}
				else
				{
					nrp_tx_buffer[tx_index++] = -1;
				}
			}
			break;
			
		default:
			stack_buffer_free(buf);
			xSemaphoreGive(nrp_lock);	
			return pdFALSE;
	}
	nrp_tx_wr = tx_index;				
	nrp_tx_start();
	stack_buffer_free(buf);
	return pdTRUE;
}

portCHAR nrp_data(buffer_t *buf)
{
	uint8_t ind;
	uint8_t tx_index = nrp_tx_wr = nrp_tx_rd = 0;
	uint16_t tmp_16;
	nrp_type_t type = NRP_DATA;
	nrp_data_tag_t tag = NRP_PROTOCOL;

	nrp_tx_buffer[tx_index++] = 'N';
	nrp_tx_buffer[tx_index++] = 'R';
	nrp_tx_buffer[tx_index++] = 'P';
	nrp_tx_buffer[tx_index++] = NRP_VERSION + type;

	switch (buf->from)
	{
		case MODULE_RF_802_15_4:
			nrp_tx_buffer[tx_index++] = tag;
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 1;	/*length*/
			nrp_tx_buffer[tx_index++] = NRP_PROTO_802_15_4;	/*Raw MAC data*/
			break;

		case MODULE_NUDP:
			nrp_tx_buffer[tx_index++] = tag;
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 1;	/*length*/
			nrp_tx_buffer[tx_index++] = NRP_PROTO_NUDP;	/*NUDP data*/
			nrp_tx_buffer[tx_index++] = NRP_SRC_PORT;
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 2;	/*length*/
			nrp_tx_buffer[tx_index++] = buf->src_sa.port >> 8;	/*NUDP port*/
			nrp_tx_buffer[tx_index++] = buf->src_sa.port;	/*NUDP port*/
			nrp_tx_buffer[tx_index++] = NRP_DST_PORT;
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 2;	/*length*/
			nrp_tx_buffer[tx_index++] = buf->dst_sa.port >> 8;	/*NUDP port*/
			nrp_tx_buffer[tx_index++] = buf->dst_sa.port;	/*NUDP port*/
			break;

#ifdef HAVE_CUDP
		case MODULE_CUDP:
			nrp_tx_buffer[tx_index++] = tag;
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 1;	/*length*/
			nrp_tx_buffer[tx_index++] = NRP_PROTO_6LOWPAN;	/*data*/
			nrp_tx_buffer[tx_index++] = NRP_SRC_PORT;
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 2;	/*length*/
			nrp_tx_buffer[tx_index++] = buf->src_sa.port >> 8;	/*port*/
			nrp_tx_buffer[tx_index++] = buf->src_sa.port;				/*port*/
			nrp_tx_buffer[tx_index++] = NRP_DST_PORT;
			nrp_tx_buffer[tx_index++] = 0;	/*set*/
			nrp_tx_buffer[tx_index++] = 2;	/*length*/
			nrp_tx_buffer[tx_index++] = buf->dst_sa.port >> 8;	/*port*/
			nrp_tx_buffer[tx_index++] = buf->dst_sa.port;				/*port*/
			break;
#endif

		default:	/*not handled*/
			stack_buffer_free(buf);
			xSemaphoreGive(nrp_lock);	
			return pdTRUE;
	}
	nrp_pack_address(NRP_SRC, &(buf->src_sa), &tx_index);
	nrp_pack_address(NRP_DST, &(buf->dst_sa), &tx_index);

	ind = buf->buf_ptr;	/*set index to start of data*/
	tmp_16 = buf->buf_end - buf->buf_ptr; /*length of data*/
	tag = NRP_PDATA;
	nrp_tx_buffer[tx_index++] = tag;
	nrp_tx_buffer[tx_index++] = (tmp_16 >> 8);	/*push length*/
	nrp_tx_buffer[tx_index++] = tmp_16;
	while (tmp_16)
	{
		nrp_tx_buffer[tx_index++] = buf->buf[ind++];
		tmp_16--;
	}
	tmp_16 = (uint16_t) buf->options.rf_dbm;
	tag = NRP_DBM;
	nrp_tx_buffer[tx_index++] = tag + 0x80;
	nrp_tx_buffer[tx_index++] = 0;	/*set*/
	nrp_tx_buffer[tx_index++] = 2;	/*length*/
	nrp_tx_buffer[tx_index++] = (tmp_16 >> 8);	/*push dbm value*/
	nrp_tx_buffer[tx_index++] = tmp_16;

	nrp_tx_wr = tx_index;				
	nrp_tx_start();

	stack_buffer_free(buf);
	return pdTRUE;
}

#endif /* HAVE_NRP*/
