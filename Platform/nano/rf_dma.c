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
 * \file rf_dma.c
 * \brief nano.4 DMA based RF driver.
 *
 *  Nano.4: RF control functions.
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

#include "bus.h"
#include "dma.h"
#include "mac.h"
#include "gpio.h"
#include "rf.h"

#ifndef RF_DEFAULT_POWER
#define RF_DEFAULT_POWER 100
#endif

#ifndef RF_DEFAULT_CHANNEL
#define RF_DEFAULT_CHANNEL 18
#endif

#ifdef HAVE_RF_LED
#if HAVE_RF_LED == 1
#define RF_RX_LED_ON() LED1_ON()
#define RF_RX_LED_OFF() LED1_OFF()
#define RF_TX_LED_ON() LED1_ON()
#define RF_TX_LED_OFF() LED1_OFF()
#else
#define RF_RX_LED_ON() LED2_ON()
#define RF_RX_LED_OFF() LED2_OFF()
#define RF_TX_LED_ON() LED2_ON()
#define RF_TX_LED_OFF() LED2_OFF()
#endif /*RF_LED == 1 */
#else
#ifdef HAVE_RF_DUAL_LED
#define RF_RX_LED_ON() LED1_ON()
#define RF_RX_LED_OFF() LED1_OFF()
#define RF_TX_LED_ON() LED2_ON()
#define RF_TX_LED_OFF() LED2_OFF()
#else
#define RF_RX_LED_ON()
#define RF_RX_LED_OFF()
#define RF_TX_LED_ON()
#define RF_TX_LED_OFF()
#endif /*HAVE_RF_DUAL_LED*/
#endif /*HAVE_RF_LED*/

uint8_t rf_initialized = 0;

uint8_t rf_tx_power;
uint8_t rf_flags;

#define RX_ACTIVE 0x80
#define TX_ACK 0x40
#define TX_ON_AIR 0x20

uint16_t rf_manfid;

extern int8_t timer_rf_launch(uint8_t ticks);
extern void timer_rf_stop(void);

#ifdef HAVE_RF_ERROR
uint8_t rf_error = 0;
#endif

uint8_t rf_softack;

/**
 * Execute a single CSP command.
 *
 * \param command command to execute
 *
 */
void rf_command(uint8_t command)
{
	if (command >= 0xE0)
	{	/*immediate strobe*/
		uint8_t fifo_count;
		switch (command)
		{	/*hardware bug workaround*/
			case ISRFOFF:
			case ISRXON:
			case ISTXON:
				fifo_count = RXFIFOCNT;
				RFST = command;
				pause_us(1);
				if (fifo_count != RXFIFOCNT)
				{
					RFST = ISFLUSHRX;
					RFST = ISFLUSHRX;
				}
				break;
				
			default:
				RFST = command;
		}
	}
	else if (command == SSTART)
	{
		RFIF &= ~IRQ_CSP_STOP;	/*clear IRQ flag*/
		RFST = SSTOP;	/*make sure there is a stop in the end*/
		RFST = ISSTART;	/*start execution*/
		while((RFIF & IRQ_CSP_STOP) == 0);
	}
	else
	{
		RFST = command;	/*write command*/
	}
}

/**
 * Select RF channel.
 *
 * \param channel channel number to select
 *
 * \return channel value or negative (invalid channel number)
 */
 
 /* channel freqdiv = (2048 + FSCTRL(9:0)) / 4
            freq = (2048 + FSCTRL(9:0)) MHz */

portCHAR rf_channel_set(uint8_t channel)
{
	uint16_t freq;
	
	if ( (channel < 11) || (channel > 26) ) return -1;
	
	/* Channel values: 11-26 */
	freq = (uint16_t) channel - 11;
	freq *= 5;	/*channel spacing*/
	freq += 357; /*correct channel range*/
	freq |= 0x4000; /*LOCK_THR = 1*/

	FSCTRLH = (freq >> 8);
	FSCTRLL = (uint8_t)freq;	
	
	return (int8_t) channel;
}

/*PA_LEVEL TXCTRL register Output Power [dBm] Current Consumption [mA] 
	31 0xA0FF 0 17.4 
	27 0xA0FB -1 16.5 
	23 0xA0F7 -3 15.2 
	19 0xA0F3 -5 13.9 
	15 0xA0EF -7 12.5 
	11 0xA0EB -10 11.2 
	 7 0xA0E7 -15 9.9 
	 3 0xA0E3 -25 8.5*/

/**
 * Select RF transmit power.
 *
 * \param new_power new power level (in per cent)
 *
 * \return new level or negative (value out of range)
 */
 
portCHAR rf_power_set(uint8_t new_power)
{
	uint16_t power;
	
	if (new_power > 100) return -1;
	
	power = 31 * new_power;
	power /= 100;
	power += 0xA160;
		
	/* Set transmitter power */
	TXCTRLH = (power >> 8);
	TXCTRLL = (uint8_t)power;	
	
	rf_tx_power = (int8_t) new_power;
	return rf_tx_power;
}

/* RF DMA setup should be:
	source RFD,
	destination volatile_ram2,
	length = VLEN_N1,
	LEN = 131, 128 + 2 RSSI + len,
	byte mode + single block transfer, trigger=RADIO,
	src_inc = none, dst_inc = increment 1,  IRQ on, priority = 2 */

dma_config_t rf_dma_conf;
uint8_t rf_dma_buffer[138];

/**
 * Enable RF receiver.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR rf_rx_enable(void)
{
/*	DMAARM = 0x80 + (1 << 0);*/	/*ABORT + channel bit*/
	if (!(rf_flags & RX_ACTIVE))
	{
  		IOCFG0 = 20;   // Set the FIFOP threshold
				
		rf_flags |= RX_ACTIVE;

		RFPWR &= ~RREG_RADIO_PD;	/*make sure it's powered*/
		while((RFIF & IRQ_RREG_ON) == 0);	/*wait for power up*/
		SLEEP &= ~OSC_PD; /*Osc on*/
		while((SLEEP & XOSC_STB) == 0);	/*wait for power up*/

		rf_command(ISRXON);		
	}
	DMAIRQ &= ~(1 << 0);
	IEN1_DMAIE = 1;	/*enable DMA interrupts*/
		
	DMAARM = (1 << 0); /*arm DMA*/
	
	return pdTRUE;
}

/**
 * Disable RF receiver.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR rf_rx_disable(void)
{
	rf_command(ISSTOP);	/*make sure CSP is not running*/
	DMAARM = 0x80 + (1 << 0);	/*ABORT + channel bit*/

	rf_command(ISRFOFF);

	RFPWR |= RREG_RADIO_PD;		/*RF powerdown*/

	rf_flags = 0;
	return pdTRUE;
}


/**
 * Enable RF transmitter.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR rf_tx_enable(void)
{
	DMAARM = 0x80 + (1 << 0);	/*ABORT + channel bit*/

	return pdTRUE;
}
   
/**
 * Initialize RF.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free or init failed
 */

portCHAR rf_init(void)
{
	portCHAR retval = pdFALSE;

	if (rf_initialized) return pdTRUE;

	RFPWR &= ~RREG_RADIO_PD;	/*make sure it's powered*/
	while ((RFPWR & ADI_RADIO_PD) == 1);
	while((RFIF & IRQ_RREG_ON) == 0);	/*wait for power up*/
	SLEEP &= ~OSC_PD; /*Osc on*/
	while((SLEEP & XOSC_STB) == 0);	/*wait for power up*/
	
	rf_flags = 0;
	rf_softack = 0;
		
	retval = pdTRUE;

	FSMTC1 = 1;	/*don't abort reception, if enable called, accept ack, auto rx after tx*/
	
	MDMCTRL0H = 0x0A;	 /* Generic client, standard hysteresis, decoder on */
	MDMCTRL0L = 0xF2;	 /* automatic ACK and CRC, standard CCA and preamble */

	MDMCTRL1H = 0x30;			/*defaults*/
	MDMCTRL1L = 0x00;
	
	/* get ID for MAC */
	rf_manfid = CHVER;
	rf_manfid <<= 8;		
	rf_manfid += CHIPID;

	rf_channel_set(RF_DEFAULT_CHANNEL);

	/* Set transmitter power */
	rf_power_set(RF_DEFAULT_POWER);

	retval = pdTRUE;
		
	rf_command(ISFLUSHTX);
	rf_command(ISFLUSHRX);

	{	/*DMA setup*/
		void *src = &RFD_SHADOW;
		uint16_t tmp_ptr = (uint16_t) &(rf_dma_conf);

		DMA0CFGH = tmp_ptr >> 8;
		DMA0CFGL = tmp_ptr;
		
		rf_dma_conf.src_h = ((uint16_t) src) >> 8;
		rf_dma_conf.src_l = ((uint16_t) src);
		src = &rf_dma_buffer[0];
		rf_dma_conf.dst_h = ((uint16_t) src) >> 8;
		rf_dma_conf.dst_l = ((uint16_t) src);
		rf_dma_conf.len_h = DMA_VLEN_N1 ;
		rf_dma_conf.len_l = 131;
		rf_dma_conf.t_mode = (DMA_RPT << 5) + DMA_T_RADIO;
		rf_dma_conf.addr_mode = (DMA_NOINC << 6) + (DMA_INC << 4) + 8 + 4 + 3; /*IRQMASK, DMA has priority*/
	}
	
	RFIM = IRQ_FIFOP | IRQ_SFD | IRQ_TXDONE;
	RFIF &= ~(IRQ_FIFOP | IRQ_SFD| IRQ_TXDONE);

	S1CON &= ~(RFIF_0 | RFIF_1);
	IEN2 |= RFIE;
	IEN0_RFERRIE = 1;
	
	rf_rx_enable();

	RF_TX_LED_OFF();
	RF_RX_LED_OFF();
	rf_initialized = 1;
	return retval;
}

/**
	* Set address decoder parameters
	*
	*	\param address address for decoder
	*/
void rf_set_address(sockaddr_t *address)
{
	uint8_t i;
	__xdata unsigned char *ptr;
	
	switch(address->addr_type)
	{
		case ADDR_802_15_4_PAN_LONG:
			PANIDH = address->address[8];
			PANIDL = address->address[9];
		case ADDR_802_15_4_LONG:
			ptr = &IEEE_ADDR0;
			for(i=0; i<8;i++)
			{
				*ptr++ = address->address[i];
			}
			break;
		
		case ADDR_802_15_4_PAN_SHORT:
			PANIDH = address->address[2];
			PANIDL = address->address[3];
		case ADDR_802_15_4_SHORT:
			SHORTADDRH = address->address[0];
			SHORTADDRL = address->address[1];
			break;
		
		default:
			break;
	}			
}
/**
 * Set address decoder on/off.
 *
 * \param param 1=on 0=off. 
 * \return pdTRUE operation successful
 */
portCHAR rf_address_decoder_mode(rf_address_mode_t mode)
{
	portCHAR retval = pdFALSE;

	rf_softack = 0;
	/* set oscillator on*/	
	switch(mode)
	{
		case RF_SOFTACK_MONITOR:
			rf_softack = 1;
		case RF_MONITOR:
			MDMCTRL0H |= 0x10;	 /*Address-decode off , coordinator*/
			MDMCTRL0L &= ~0x10;	 /*no automatic ACK */
			break;
			
		case RF_DECODER_COORDINATOR:
			MDMCTRL0H |= 0x18;	 /*Address-decode on , coordinator*/
			MDMCTRL0L |= 0x10;	 /*automatic ACK */
			break;
			
		case RF_DECODER_ON:
			MDMCTRL0H |= 0x08;	 /*Address-decode on */
			MDMCTRL0L |= 0x10;	 /*automatic ACK */
			break;

		default:
			MDMCTRL0H &= ~0x18;	 /* Generic client */
			MDMCTRL0L &= ~0x10;	 /* no automatic ACK */
			break;
	}
	retval = pdTRUE;
	return retval; 
}

/**
 * Channel energy detect.
 *
 * Coordinator use this function detect best channel for PAN-network.
 * \return RSSI-energy level dBm.
 * \return 0	operation failed.
 */
 
int8_t rf_analyze_rssi(void)
{
	uint8_t counter, i;
	int16_t sum=0;
	int8_t retval = 0, temp=0;
	
	rf_command(ISRXON);
	pause_us(16);				/* waiting one symbol period */
	pause_us(16);				/* waiting one symbol period */
	counter = 0;
	for(i=0; i<8; i++)
	{
		temp = (int8_t)RSSIL;
		temp -= 45;
		sum += (int16_t)temp;
		pause_us(16);				/* waiting one symbol period */
	}
	sum /=8;
	retval = (int8_t)sum;
	
	return retval; 
}

/**
 * Clear channel assesment check.
 *
 * \return pdTRUE	CCA clear
 * \return pdFALSE	CCA reserved
 */
portCHAR rf_cca_check(uint8_t backoff_count, uint8_t slotted)
{
	uint8_t counter, cca=1;
	portCHAR retval = pdTRUE;
	backoff_count;
	rf_command(ISRXON);

	pause_us(16);
	pause_us(16);
	pause_us(16);
	pause_us(16);
	switch (slotted)
	{
		case 1:

		if(RFSTATUS & CCA)
		{
			counter=0;
			cca=1;
			while(cca!=0) 
			{
				if(counter > 1)
					cca=0;
				pause_us(250);
				if(!(RFSTATUS & CCA))
				{
					cca=0;
					retval = pdFALSE;
				}
				counter++;
			}
		}
		else
			retval = pdFALSE;
		break;

		case 0:
			if(!(RFSTATUS & CCA))
			{
				retval = pdFALSE;
			}
			else
			{
			}
			break;
	}
	return retval;		
}



/**
 * Send ACK.
 *
 *\param pending set up pending flag if pending > 0. 
 */
void rf_send_ack(uint8_t pending)
{
	if(pending)
	{
		rf_command(ISACKPEND);
	}
	else
	{
		rf_command(ISACK);
	}
}

/**
 * Transmit packet.
 *
 * Missing feature: address type check
 *
 * \param mac RF HW address
 * \param dst destination HW address
 * \param buffer data buffer pointer
 * \param length length of data
 *
 * \return pdTRUE+1 channel occupied
 * \return pdTRUE
 * \return pdFALSE	bus reserved
 */
 
portCHAR rf_write(buffer_t *buffer)
{
	uint8_t counter, i;
	portCHAR retval = pdTRUE;
	int16_t length =  buffer->buf_end - buffer->buf_ptr;
	
	if (rf_flags & TX_ACK) return pdFALSE;
	if ((rf_flags & RX_ACTIVE) == 0) rf_rx_enable();

	if (length <= 128) 
	{
		uint8_t *ptr = buffer->buf + buffer->buf_ptr;
		
		rf_command(ISFLUSHTX);

		RFD = (length+2);

		for (i = 0 ; i < length ; i++)
		{
			RFD = *ptr++;
		}
		RFD = (0);
		RFD = (0);

		if (rf_cca_check(0,0) == pdFALSE)
		{
			return pdTRUE+1;
		}

	
		RFIF &= ~IRQ_TXDONE;
		rf_command(ISTXONCCA);
		counter = 0;	
		while(!(RFSTATUS & TX_ACTIVE) && (counter++ < 200))
		{
			if (RFSTATUS & (SFD | TX_ACTIVE) == SFD)
			{
				rf_command(ISRXON);
				counter = 200;
			}
			else
			{
				pause_us(10);
			}
		}

		if (counter >= 200)
		{
#ifdef RF_DEBUG
			debug("\r\nRF: TX never active.\r\n");
#endif
			retval = pdFALSE;
		}
		else
		{
			RF_RX_LED_OFF();
			RF_TX_LED_ON();
			rf_flags |= TX_ACK | TX_ON_AIR;
		}
	}
	if ((retval == pdTRUE) && (length > 128))
	{
#ifdef RF_DEBUG
		debug("Packet too long(");
		debug_int(length);
		debug(").\r\n");
#endif
		retval = pdFALSE;
	}

	return retval;		
}

uint8_t ft_buffer[8];
/**
	* Get HW MAC address
	*
	* \param address address to set
	*/
portCHAR rf_mac_get(sockaddr_t *address)
{
	mac_get(address);
	if (address->addr_type == ADDR_NONE)
	{
		uint8_t i;
		flash_read(&ft_buffer[0], 0x1FFF8, 8);
		for (i=0; i<8; i++)
		{
			address->address[7-i] = ft_buffer[i];
		}
		address->addr_type = ADDR_802_15_4_LONG;
		if (stack_check_broadcast(address->address, ADDR_802_15_4_LONG) == pdTRUE)
		{
			debug("No address in flash.\n");
			address->address[0]=0x00;
			address->address[1]=0x00;
			address->address[2]=0x00;
			address->address[3]=0x00;
			address->address[4]=0x00;
			address->address[5]=0x00;
#ifdef SHORT_ADDRESS
			address->address[6]= (SHORT_ADDRESS & 0xff);
			address->address[7]= (SHORT_ADDRESS >> 8);
#else
#warning "SHORT_ADDRESS not defined"
			address->address[6]= 0x12;
			address->address[7]= 0x34;
#endif
		}
		mac_set(address);
	}

	return pdTRUE;
}

extern xQueueHandle buffers;
#ifdef HAVE_MAC_15_4
extern void mac_push(buffer_t *b);
extern void mac_rx_push(void);
#endif

#ifdef HAVE_NRP
extern sockaddr_t mac_short;
extern sockaddr_t mac_long;
#endif

/**
 * RF DMA callback, ISR version.
 *
 * \param param not used;
 *
 */
void rf_dma_callback_isr(void)
{
	portBASE_TYPE prev_task = pdFALSE;
	uint8_t *ptr = rf_dma_buffer;
	uint8_t len;
	buffer_t *rf_buf = 0;
	
	DMAARM=0x81;
	
	RF_RX_LED_ON();
	
	len = *ptr++;
	if (len > 131) len = 131;
	
	if (len >= 2) len -= 2;
	
	if (len >= 3)
	{
#ifdef HAVE_NRP
		rf_softack &= ~0x80;
		if ((rf_softack) && (*ptr & 0x20))
		{	/*softack in use and ACK requested*/
			uint8_t fcf;
			
			ptr++;
			fcf = *ptr;
			ptr--;
			
			if (fcf & 0x0C) 
			{	/*has dest address*/
				if ((fcf & 0x0C) == 0x08)
				{
					uint8_t i = 0;
					ptr += 3;
					
					for (i=0; i<4 ; i++)
					{
						if (ptr[i] != 0xFF) break;
					}
					
					if ((i<4) && (mac_short.addr_type != ADDR_NONE))
					{
						for (i=0; i<4 ; i++)
						{
							if (ptr[i] != mac_short.address[(i+2)&3]) break;
						}
					}
					
					ptr -= 3;
					if (i >= 4)
					{
						rf_softack |= 0x80;
					}
				}
				else
				{
					uint8_t i;
					
					ptr += 5;	/*start of long, skip PAN ID*/
					
					for (i=0; i<8 ; i++)
					{
						if (*ptr++ != mac_long.address[i]) break;
					}
					if (i != 8)
					{	/*long address does not match, drop*/
						ptr -= i;				
						ptr--;
					}
					else
					{	/* hit, ok */
						ptr -= 8;
						rf_softack |= 0x80;
					}
					ptr -= 5;
				}
			}
		}
#endif
		if (xQueueReceiveFromISR( buffers, &( rf_buf ), &prev_task) == pdTRUE)
		{
			uint8_t *ptr2 = &(rf_buf->buf[0]);
			uint8_t i;
			
			rf_buf->options.type = BUFFER_DATA;
			rf_buf->socket = 0;
			rf_buf->buf_ptr = 0;
#ifdef HAVE_802_15_4_RAW
			rf_buf->to = MODULE_802_15_4_RAW;
#endif
#ifdef HAVE_RF_802_15_4
			rf_buf->to = MODULE_RF_802_15_4;
#endif
			rf_buf->dir = BUFFER_UP;
			
#ifdef HAVE_NRP
			if (rf_softack & 0x80)
			{	/*ack requested*/
				if (rf_softack & 0x40)
				{	/*pending flag*/
					RFST = ISACKPEND;
				}
				else
				{
					RFST = ISACK;
				}
			}
#endif
			for (i=0; i<len; i++)
			{
				*ptr2++ = *ptr++;
			}
		
			rf_buf->options.rf_dbm = ((int8_t) *ptr++) - 45;
			
			rf_buf->options.rf_lqi = *ptr;

			rf_buf->buf_end = len;
			rf_buf->options.rf_lqi &= 0x7F;
			
			rf_flags |= RX_ACTIVE | TX_ACK;
			timer_rf_launch(MAC_IFS/4); /*200something us in timer ticks*/
			
			if (*ptr & 0x80)
			{
#ifdef HAVE_MAC_15_4
				mac_push(rf_buf);
				mac_rx_push();
#else
				{
					event_t event;
					event.process = 0;
					event.param = (void *) rf_buf;
	
					prev_task = xQueueSendFromISR( events, ( void * ) &event, prev_task);
					taskYIELD();
				}
#endif
			}
			else
			{
				rf_buf->buf_end = 0;
				prev_task = xQueueSendFromISR( buffers, ( void * ) &rf_buf, prev_task);
			}
		}
	}
	RF_RX_LED_OFF();
	EA = 0;
	if ((RXFIFOCNT & 0xFF) != 0)
	{
		if ((RFSTATUS & (FIFO|FIFOP)) == FIFOP)
		{
				DMAARM = 0x81;
				if (RFSTATUS & TX_ACTIVE == 0)
				{	rf_command(ISRFOFF);
					rf_command(ISFLUSHRX);
					rf_command(ISRXON);
				}
				else
				{
					rf_command(ISFLUSHRX);
				}
				DMAARM=1;
		}
		else
		{
			RF_RX_LED_ON();
			DMAARM=1;
			DMAREQ = 1;
		}
	}
	else
	{
		DMAARM=1;
	}
	EA = 1;
}

void rf_rx_callback(void *param)
{
	param;
	return;
}

void rf_timer_callback(void);
void rf_timer_callback(void)
{
	rf_flags &= ~TX_ACK;
}

/**
 * RF interrupt service routine.
 *
 */
void rf_ISR( void ) interrupt (RF_VECTOR)
{
/*	portBASE_TYPE task = pdFALSE;*/
	EA = 0;

	if (RFIF & IRQ_TXDONE)
	{
		RFIF &= ~IRQ_TXDONE;
		
		if (rf_flags & TX_ON_AIR)
		{
			rf_flags |= TX_ACK;
			rf_flags ^= TX_ON_AIR;
			timer_rf_launch(MAC_IFS); /*832 us in timer ticks*/
		}

		RF_TX_LED_OFF();
		DMAARM = 1;
	}
	if (RFIF & IRQ_SFD)
	{
		if ((RFSTATUS & TX_ACTIVE) == 0) 
		{
			uint8_t count = RXFIFOCNT;
			if (count > 0x80)
			{
				DMAARM = 0x81;
				rf_command(ISRFOFF);
				rf_command(ISFLUSHRX);
				rf_command(ISFLUSHRX);
				rf_command(ISRXON);
				DMAARM = 1;
#ifdef HAVE_RF_ERROR
				rf_error = 0xFF;
#endif
			}
			else if (count > 0x01)
			{
				DMAARM = 0x81;
#ifdef HAVE_RF_ERROR
				rf_error = RXFIFOCNT;
#endif
				while (count)
				{
					uint8_t dummy = RFD;
					count--;
				}
				DMAARM = 1;
			}
		}
		RFIF &= ~IRQ_SFD;
	}
	if (RFIF & IRQ_FIFOP)
	{
		RFIF &= ~IRQ_FIFOP;
		if (RFSTATUS & FIFO)
		{
			if ((DMAARM & 1) == 0)
			{
				DMAARM = 1;
			}
			RF_RX_LED_ON();
			if ((RFSTATUS & SFD) == 0)
			{
				DMAREQ=1;
			}
		}
/*		else
		{
			if ((DMAARM & 1) == 0)	rf_command(ISFLUSHRX);
		}*/
	}
	S1CON &= ~(RFIF_0 | RFIF_1);
	RFIM = IRQ_FIFOP | IRQ_SFD | IRQ_TXDONE;
	EA = 1;
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

/**
 * RF error interrupt service routine.
 *
 */
void rf_error_ISR( void ) interrupt (RFERR_VECTOR)
{
	TCON_RFERRIF = 0;
#ifdef HAVE_RF_ERROR
	rf_error = 254;
#endif
	DMAARM= 0x81; /*stop DMA*/
	rf_command(ISRFOFF);
	rf_command(ISFLUSHRX);
	rf_command(ISFLUSHRX);
	rf_command(ISRXON);
	DMAARM= 0x01; /*start DMA*/
	RF_RX_LED_OFF();
	RF_TX_LED_OFF();
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

