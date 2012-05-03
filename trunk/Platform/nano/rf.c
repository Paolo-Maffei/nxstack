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
 * \file rf.c
 * \brief nano.4 RF driver.
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
#include "rf.h"

#ifndef RF_DEFAULT_POWER
#define RF_DEFAULT_POWER 100
#endif

#ifndef RF_DEFAULT_CHANNEL
#define RF_DEFAULT_CHANNEL 18
#endif

#ifdef HAVE_RF_LED
#ifdef HAVE_NRP
#define RF_RX_LED_ON() LED1_ON()
#define RF_RX_LED_OFF() LED1_OFF()
#define RF_TX_LED_ON() LED2_ON()
#define RF_TX_LED_OFF() LED2_OFF()
#else
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
#endif	/*HAVE_NRP*/
#else
#define RF_RX_LED_ON()
#define RF_RX_LED_OFF()
#define RF_TX_LED_ON()
#define RF_TX_LED_OFF()
#endif /*HAVE_RF_LED*/

uint8_t rf_initialized = 0;

uint8_t rf_tx_power;
uint8_t rx_flags, tx_flags;
#define RX_ACTIVE 0x80

uint16_t rf_manfid;

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

/**
 * Enable RF receiver.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR rf_rx_enable(void)
{
	if (rx_flags == 0)
	{
		RFIF &= ~(IRQ_SFD);

  	IOCFG0 = 0x04;   // Set the FIFOP threshold
		tx_flags = 0;
		rx_flags = RX_ACTIVE;
		S1CON &= ~(RFIF_0 | RFIF_1);
		RFPWR &= ~RREG_RADIO_PD;	/*make sure it's powered*/
		while((RFIF & IRQ_RREG_ON) == 0);	/*wait for power up*/
		SLEEP &= ~OSC_PD; /*Osc on*/
		while((SLEEP & XOSC_STB) == 0);	/*wait for power up*/
		
		RFIM |= IRQ_SFD;
		RFIF &= ~(IRQ_SFD);

		S1CON &= ~(RFIF_0 | RFIF_1);
		IEN2 |= RFIE;
		rf_command(ISRXON);
	}
	
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
	RFIM &= ~IRQ_SFD;

	rf_command(ISSTOP);
	rf_command(ISRFOFF);

	RFPWR |= RREG_RADIO_PD;		/*RF powerdown*/
	IEN2 &= ~RFIE;
	rx_flags = 0;
	
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
	RFIM &= ~IRQ_SFD;

	IEN2 &= ~RFIE;
	tx_flags |= RX_ACTIVE;
	rx_flags = 0;
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
	
	rx_flags = tx_flags = 0;
	
	retval = pdTRUE;

	FSMTC1 &= ~ABORTRX_ON_SRXON;	/*don't abort reception, if enable called*/
	FSMTC1 |= ACCEPT_ACKPKT;
	
	MDMCTRL0H &= ~0x18;	 /* Generic client */
	MDMCTRL0L &= ~0x10;	 /* no automatic ACK */

	MDMCTRL1H = 0x14;	/* CC2420 behaviour*/
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

	rf_rx_enable();

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
			ptr = &IEEE_ADDR0;
			for(i=0; i<8;i++)
			{
				*ptr++ = address->address[i];
			}
			PANIDH = address->address[8];
			PANIDL = address->address[9];
			break;
		
		case ADDR_802_15_4_PAN_SHORT:
			SHORTADDRH = address->address[0];
			SHORTADDRL = address->address[1];
			PANIDH = address->address[2];
			PANIDL = address->address[3];
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
portCHAR rf_address_decoder_mode(uint8_t param)
{
	portCHAR retval = pdFALSE;

	/* set oscillator on*/	
	if(param)
	{
#ifdef COORDINATOR
		MDMCTRL0H |= 0x18;	 /*Coordinator, Address-decode on */
		MDMCTRL0L &= ~0x10;	 /* no automatic ACK *//* Enable receive beacon if address decoder is enabled */
#else
#ifdef HAVE_NRP
		MDMCTRL0H &= ~0x18;	 /* Generic client */
		MDMCTRL0L &= ~0x10;	 /* no automatic ACK */
#else
		MDMCTRL0H |= 0x08;	 /*Address-decode on */
		MDMCTRL0L |= 0x10;	 /*automatic ACK */ /* Enable receive beacon if address decoder is enabled */
#endif
#endif	
	}
	else
	{
		MDMCTRL0H &= ~0x18;	 /* Generic client */
		MDMCTRL0L &= ~0x10;	 /* no automatic ACK */
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
	
	RF_TX_LED_ON();
	if (rx_flags & RX_ACTIVE)
	{
		if ( (RFSTATUS & FIFOP) || (RFSTATUS & SFD) )
		{
#ifdef RF_DEBUG				
			debug("RF: Packet IF busy.\r\n");
#endif
			if ((RFSTATUS & FIFOP))
			{
				rf_command(ISFLUSHTX);
				rf_command(ISFLUSHRX);
				rf_command(ISFLUSHRX);
			}
			RF_TX_LED_OFF();
			retval = pdFALSE;		
		}
	}
	
	rf_tx_enable();
	if ( (length <= 128) && (retval == pdTRUE) )
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
			rf_command(ISFLUSHTX);
			rx_flags = 0;
			rf_rx_enable();
			RF_TX_LED_OFF();
			return pdTRUE+1;
		}

		i= 0;
		RFIF &= ~IRQ_TXDONE;
		while (i++ < 3)
		{
			rf_command(ISTXON);
			counter = 0;	
			while(!(RFSTATUS & TX_ACTIVE) && (counter++ < 200))
			{
				pause_us(10);
			}
			if (RFSTATUS & TX_ACTIVE) i = 200;
		}

		if (i ==3)
		{
#ifdef RF_DEBUG
			debug("\r\nRF: TX never active.\r\n");
#endif
			rf_command(ISRFOFF);
			tx_flags = 0;
			retval = pdFALSE;
		}
		else
		{
			while(!(RFIF & IRQ_TXDONE))
			{
				pause_us(10);
			}
		}
		RF_TX_LED_OFF();
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
	tx_flags = 0;
	rf_rx_enable();


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

void rf_rx_callback(void *param);

void rf_rx_callback(void *param)
{
	buffer_t *b;
	uint8_t i;
	uint8_t *ptr;
	uint16_t tmp_16;
		
	param;

	b = stack_buffer_get(20);
	if (b)
	{
		b->options.type = BUFFER_DATA;
#ifdef HAVE_802_15_4_RAW
		b->to = MODULE_802_15_4_RAW;
#endif

#ifdef HAVE_RF_802_15_4
		b->to = MODULE_RF_802_15_4;
#endif
		b->dir = BUFFER_UP;
		b->buf_ptr = 0;
		
		tmp_16 = 0;
		if ((RFSTATUS & SFD) != 0)
		{
			while( ((RFSTATUS & SFD) != 0) && (tmp_16++ > 1000))
			{
			}
		}
		if (tmp_16 <= 1000)
		{
			if( ((RFSTATUS & FIFOP)) && (! ((RFSTATUS & FIFO)) ) ) 
			{
#ifdef RF_DEBUG
				debug("RF:OV.\r\n");
#endif
				rf_command(ISFLUSHRX);
				rf_rx_enable();
			}
			else if( ((RFSTATUS & FIFO)) && (! ((RFSTATUS & SFD)) ))
			{ /*Data in buffer*/
				i = RFD & 0x7F;
				i -= 2;

				b->buf_end = i;
				ptr = b->buf;

				while(i)
				{
					*ptr++ = RFD;			
					i--;
				}		

				b->options.rf_dbm = ((int8_t) RFD) - 45;
				b->options.rf_lqi = RFD;

				if (b->options.rf_lqi & 0x80)
				{
					b->options.rf_lqi &= ~0x80;
					stack_buffer_push(b);
					b = 0;
				}
				else
				{
#ifdef RF_DEBUG
					debug("RF: drop\r\n");	
#endif
				}
				rf_rx_enable();
			}/*end data in buffer*/
		}
		else
		{
			rf_command(ISFLUSHRX);
		}
		if (b != 0) stack_buffer_free(b);
	}
	else
	{
		rf_command(ISFLUSHRX);
#ifdef RF_DEBUG_RSSI
		debug("-");
#endif		
	}
}

/**
 * RF interrupt service routine.
 *
 */
void rf_ISR( void ) interrupt (RF_VECTOR)
{
	portBASE_TYPE prev_task = pdFALSE;
	
	if ((RFIF & IRQ_SFD) && (tx_flags == 0))
	{
		event_t event;
		event.process = rf_rx_callback;
		event.param = (void *) 0;

		prev_task = xQueueSendFromISR(events, &event, prev_task);
		if (prev_task == pdTRUE)
		{
			taskYIELD();
		}
	}
	if (RFIF & IRQ_SFD)
	{
		RFIF &= ~IRQ_SFD;
	}

	S1CON &= ~(RFIF_0 | RFIF_1);
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
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}
