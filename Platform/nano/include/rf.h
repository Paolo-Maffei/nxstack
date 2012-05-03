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
 * \file rf.h
 * \brief nano.4 RF driver.
 *
 *  Nano.4: RF control headers.
 *   
 *	
 */


#ifndef _RF_H
#define _RF_H

typedef enum rf_address_mode_t
{
	RF_DECODER_NONE = 0,
	RF_DECODER_COORDINATOR,
	RF_SOFTACK_MONITOR,
	RF_MONITOR,
	RF_DECODER_ON
}rf_address_mode_t;

/*CSP command set*/
#define SSTOP		0xDF
/*this is not a real command but a way of having rf_command
  wait until the script is done*/
#define SSTART		0xDE	

#define SNOP		0xC0
#define STXCALN 0xC1
#define SRXON		0xC2
#define STXON		0xC3
#define STXONCCA	0xC4
#define SRFOFF		0xC5
#define SFLUSHRX	0xC6
#define SFLUSHTX	0xC7
#define SACK			0xC8
#define SACKPEND	0xC9

#define ISTXCALN 	0xE1
#define ISRXON		0xE2
#define ISTXON		0xE3
#define ISTXONCCA	0xE4
#define ISRFOFF		0xE5
#define ISFLUSHRX	0xE6
#define ISFLUSHTX	0xE7
#define ISACK			0xE8
#define ISACKPEND	0xE9

#define ISSTOP		0xFF
#define ISSTART		0xFE

#define MAC_IFS (1200/32)
#define PLATFORM_TIMER_DIV 32

extern portCHAR rf_init(void);

extern portCHAR rf_rx_enable(void);
extern portCHAR rf_rx_disable(void);

extern portCHAR rf_write(buffer_t *buffer);
extern void rf_send_ack(uint8_t pending);
extern int8_t rf_analyze_rssi(void);
extern portCHAR rf_cca_check(uint8_t backoff_count, uint8_t slotted);
extern portCHAR rf_write_no_cca(buffer_t *buffer);

extern portCHAR rf_mac_get(sockaddr_t *address);
extern int8_t rf_power_set(uint8_t new_power);
extern int8_t rf_channel_set(uint8_t channel);

extern void rf_ISR( void ) interrupt (RF_VECTOR);
extern void rf_error_ISR( void ) interrupt (RFERR_VECTOR);
extern portCHAR mac_set_channel(uint8_t new_channel);
extern uint8_t mac_current_channel(void);
#ifdef HAVE_RF_DMA
void rf_dma_callback_isr(void);
#endif
extern portCHAR rf_address_decoder_mode(rf_address_mode_t mode);
extern void rf_set_address(sockaddr_t *address);
#endif /*_RF_H*/
