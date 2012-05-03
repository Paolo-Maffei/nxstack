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
 * \brief micro RF driver header.
 *
 *  Micro: RF control function headers.
 *   
 *	
 */


#ifndef RF_H
#define RF_H
/** Address decoder mode value */
typedef enum rf_address_mode_t
{
	RF_DECODER_NONE = 0,			/*!< Decoder off. */
	RF_DECODER_COORDINATOR,			/*!< Decoder on with coordinator radio setups. */
	RF_DECODER_ON					/*!< Decoder on with general features. */
}rf_address_mode_t;
#define MAC_IFS (900/32)
#define PLATFORM_TIMER_DIV 32
extern portCHAR rf_init(void);
extern portCHAR rf_write(buffer_t *buffer);
extern portCHAR rf_rx_enable(void);
extern portCHAR rf_rx_disable(void);
extern portCHAR rf_shutdown(void);
extern portCHAR rf_mac_get(sockaddr_t *address);
extern int8_t rf_power_set(uint8_t new_power);
extern int8_t rf_channel_set(uint8_t channel);
extern void rf_send_ack(uint8_t pending);
extern int8_t rf_analyze_rssi(void);
extern portCHAR rf_cca_check(uint8_t backoff_count, uint8_t slotted);
extern portCHAR rf_write_no_cca(buffer_t *buffer);

//extern portCHAR rf_address_decoder_mode(uint8_t param);
extern portCHAR rf_address_decoder_mode(rf_address_mode_t mode);
extern void rf_set_address(sockaddr_t *address);
extern portCHAR start_tx_test_mode(uint8_t channel);
extern portCHAR stop_tx_test_mode(void);
#endif /* RF_H */
