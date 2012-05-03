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
 * \brief micro RF driver.
 *
 *  Micro: RF control functions.
 *   
 *	
 */

 
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <sys/inttypes.h>
#include <signal.h>
#include <string.h>

#include "bus.h"
#include "stack.h"
#ifndef CC2420_DEBUG
#undef HAVE_DEBUG
#endif

#include "debug.h"
#include "stack.h"
#include "mac.h"

#include "gpio.h"
#include "rf.h"
#include "rf_802_15_4.h"
#ifdef HAVE_POWERSAVE
#include "powersave.h"
#endif

#define RF_NO_TASK

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
#define RF_RX_LED_ON()
#define RF_RX_LED_OFF()
#define RF_TX_LED_ON()
#define RF_TX_LED_OFF()
#endif /*HAVE_RF_LED*/


#define RSSI_OFFSET -38
#define RSSI_2_ED(rssi)   ((rssi) < RSSI_OFFSET ? 0 : ((rssi) - (RSSI_OFFSET)))
#define ED_2_LQI(ed) (((ed) > 63 ? 255 : ((ed) << 2)))


typedef enum cc2420_addr_t
{
 CC_ADDR_TXFIFO		=0x000,
 CC_ADDR_RXFIFO		=0x080,
 CC_ADDR_KEY0		=0x100,
 CC_ADDR_RXNONCE	=0x110,
 CC_ADDR_SABUF		=0x120,
 CC_ADDR_KEY1		=0x130,
 CC_ADDR_TXNONCE	=0x140,
 CC_ADDR_CBCSTATE	=0x150,
 CC_ADDR_IEEEADDR	=0x160,
 CC_ADDR_PANID		=0x168,
 CC_ADDR_SHORTADDR	=0x16A
}cc2420_addr_t;

typedef enum ram_address_t
{
 RAM_ADDR_IEEEADDR	=1,
 RAM_ADDR_PANID		=2,
 RAM_ADDR_SHORTADDR	=3
}ram_address_t;

typedef struct
{
	cc2420_addr_t address;
	uint8_t 	data[8];
}rf_ram_access_t;

typedef enum
{
 CC_REG_SNOP             =0x00,
 CC_REG_SXOSCON          =0x01,
 CC_REG_STXCAL           =0x02,
 CC_REG_SRXON            =0x03,
 CC_REG_STXON            =0x04,
 CC_REG_STXONCCA         =0x05,
 CC_REG_SRFOFF           =0x06,
 CC_REG_SXOSCOFF         =0x07,
 CC_REG_SFLUSHRX         =0x08,
 CC_REG_SFLUSHTX         =0x09,
 CC_REG_SACK             =0x0A,
 CC_REG_SACKPEND         =0x0B,
 CC_REG_SRXDEC           =0x0C,
 CC_REG_STXENC           =0x0D,
 CC_REG_SAES             =0x0E,

 CC_REG_MAIN             =0x10,
 CC_REG_MDMCTRL0         =0x11,
 CC_REG_MDMCTRL1         =0x12,
 CC_REG_RSSI             =0x13,
 CC_REG_SYNCWORD         =0x14,
 CC_REG_TXCTRL           =0x15,
 CC_REG_RXCTRL0          =0x16,
 CC_REG_RXCTRL1          =0x17,
 CC_REG_FSCTRL           =0x18,
 CC_REG_SECCTRL0         =0x19,
 CC_REG_SECCTRL1         =0x1A,
 CC_REG_BATTMON          =0x1B,
 CC_REG_IOCFG0           =0x1C,
 CC_REG_IOCFG1           =0x1D,
 CC_REG_MANFIDL          =0x1E,
 CC_REG_MANFIDH          =0x1F,
 CC_REG_FSMTC            =0x20,
 CC_REG_MANAND           =0x21,
 CC_REG_MANOR            =0x22,
 CC_REG_AGCCTRL          =0x23,
 CC_REG_AGCTST0          =0x24,
 CC_REG_AGCTST1          =0x25,
 CC_REG_AGCTST2          =0x26,
 CC_REG_FSTST0           =0x27,
 CC_REG_FSTST1           =0x28,
 CC_REG_FSTST2           =0x29,
 CC_REG_FSTST3           =0x2A,
 CC_REG_RXBPFTST         =0x2B,
 CC_REG_FSMSTATE         =0x2C,
 CC_REG_ADCTST           =0x2D,
 CC_REG_DACTST           =0x2E,
 CC_REG_TOPTST           =0x2F,
 CC_REG_RESERVED         =0x30,

 CC_REG_TXFIFO           =0x3E,
 CC_REG_RXFIFO           =0x3F
}cc2420_reg_t;
/*
typedef enum cc2420_addr_t
{
 CC_ADDR_TXFIFO		=0x000,
 CC_ADDR_RXFIFO		=0x080,
 CC_ADDR_KEY0			=0x100,
 CC_ADDR_RXNONCE		=0x110,
 CC_ADDR_SABUF			=0x120,
 CC_ADDR_KEY1			=0x130,
 CC_ADDR_TXNONCE		=0x140,
 CC_ADDR_CBCSTATE		=0x150,
 CC_ADDR_IEEEADDR		=0x160,
 CC_ADDR_PANID			=0x168,
 CC_ADDR_SHORTADDR		=0x16A
}cc2420_addr_t;*/

/*#define CC2420_RSSI_VALID 0x80
#define CC2420_TX_ACTIVE 0x40*/

/*MDMCTRL0 bits*/
#define CC2420_MC0_RESERVED_FRAME_MODE  0x2000
#define CC2420_MC0_PAN_COORDINATOR  		0x1000
#define CC2420_MC0_ADDR_DECODE		  		0x0800
#define CC2420_MC0_CCA_HYST_MASK	  		0x0700
#define CC2420_MC0_CCA_MODE_MASK	  		0x00C0
#define CC2420_MC0_CCA_MODE_RSSI	  		0x0040
#define CC2420_MC0_CCA_MODE_802154  		0x0080
#define CC2420_MC0_CCA_MODE_BOTH	  		0x00C0
#define CC2420_MC0_AUTOCRC				  		0x0020
#define CC2420_MC0_AUTOACK				  		0x0010
#define CC2420_MC0_PREAMB_LEN_MASK  		0x000F

/*MDMCTRL1 bits*/
#define CC2420_MC1_CORR_THR_MASK	  		0x07C0
#define CC2420_MC1_DEMOD_AVG_MODE	  		0x0020
#define CC2420_MC1_MODULATION_MODE  		0x0010
#define CC2420_MC1_TX_MODE_MASK 	  		0x000C
#define CC2420_MC1_TX_MODE_BUFFERED  		0x0000
#define CC2420_MC1_TX_MODE_SERIAL	  		0x0001
#define CC2420_MC1_TX_MODE_LOOP 	  		0x0002
#define CC2420_MC1_RX_MODE_MASK 	  		0x0003
#define CC2420_MC1_RX_MODE_BUFFERED  		0x0000
#define CC2420_MC1_RX_MODE_SERIAL	  		0x0001
#define CC2420_MC1_RX_MODE_LOOP 	  		0x0002

#define CC2420_MDMCTRL1_MODULED_TEST_TX  	0x050c

#define TXCTRL_INIT				0xA0FF
/* Status byte */
#define CC2420_XOSC16M_STABLE	(1 << 6)
#define CC2420_TX_UNDERFLOW		(1 << 5)
#define CC2420_ENC_BUSY				(1 << 4)
#define CC2420_TX_ACTIVE			(1 << 3)
#define CC2420_LOCK						(1 << 2)
#define CC2420_RSSI_VALID		  (1 << 1)

/*RF SPI mode open and close*/
#define CC2420_OPEN(x) bus_select(1, BUS_SPI, BUS_STE + BUS_PHASE_INVERT)
#define CC2420_CLOSE(x) bus_free()

/*These macros control RF select signal and assume correct bus state*/
#define CC2420_SELECT(x) P3OUT |= 0x01
#define CC2420_UNSELECT(x) P3OUT &= ~0x01

/*I/O init and status readout macros*/
#define CC2420_INIT(x) P5DIR &= ~0x70; P1DIR &= ~0x80; P5SEL &= ~0x70; P1SEL &= ~0x80

#define CC2420_FIFO(x) ( (P5IN >> 5) & 1 ) 
#define CC2420_FIFOP(x) ( (P5IN >> 6) & 1 )
#define CC2420_CCA(x) ( (P5IN >> 4) & 1 )
#define CC2420_SFD(x) ( (P1IN >> 7) & 1 )

/**
 * State flags.
 *
 */
 
uint8_t rx_flags, tx_flags;
int8_t tx_power;
uint32_t rf_manfid = 0;
#define ACTIVE  0x80
#define CARRIER 0x40
#define COMPLETE 0x20
#define TX_ACK 0x10

#define NO_TX 0x40

/**
 * Receiver lock.
 *
 */

void rf_isr(void);
portCHAR rf_rx_enable(void);
portCHAR rf_rx_disable(void);
//void rf_status(void);
uint8_t rf_state(void);
void rf_config(void);
void rf_rxflush(void);

void CC2420_COMMAND(uint8_t command);
uint8_t CC2420_COMMAND_GET(uint8_t command);
void CC2420_REG_SET(cc2420_reg_t reg, uint16_t data);
uint16_t CC2420_REG_GET(cc2420_reg_t reg);
int8_t CC2420_CHANNEL_SET(uint8_t channel);
int8_t CC2420_POWER_SET(uint8_t new_power);
void CC2420_STAT(uint8_t status);

void CC2420_SPI_RESET(void);

void rf_set_ram(address_t address, ram_address_t type);
portCHAR rf_ram_set(rf_ram_access_t *ram_access);

void rx_enable(uint8_t irq_state);
void rx_disable(void);
void tx_enable(void);

void rf_rx_callback( void *param );

#ifdef HAVE_POWERSAVE
static xPowerHandle rf_ph = 0;
#endif

extern int8_t timer_rf_launch(uint16_t ticks);
extern void timer_rf_stop(void);
extern xQueueHandle rf_802_15_4_queue;
extern void mac_push(buffer_t *b);

/**
 * MAC address.
 *
 */

extern sockaddr_t mac_long;

portCHAR rf_mac_get(sockaddr_t *address);

/**
 * RF single byte command.
 *
 * \param command RF command to send
 *
 */
 
void CC2420_COMMAND(uint8_t command)  
{
	CC2420_SELECT();
	bus_spi_exchange(command);
	CC2420_UNSELECT();
}

/**
 * RF single byte command with return value.
 *
 * \param command RF command to send
 *
 * \return status byte from SPI
 */
 
uint8_t CC2420_COMMAND_GET(uint8_t command)  
{
	uint8_t value;
	
	CC2420_SELECT();
	value = bus_spi_exchange(command);
	CC2420_UNSELECT();
	return value;
}


/**
 * RF register write.
 *
 *
 */
 
void CC2420_REG_SET(cc2420_reg_t reg, uint16_t data)
{
	uint8_t byte;
	
	CC2420_SELECT();
	bus_spi_exchange((uint8_t) reg & 0x3F);
	byte = ((uint16_t)data >> 8);
	bus_spi_exchange((uint8_t) byte);
	bus_spi_exchange((uint8_t) data);
	CC2420_UNSELECT();
}

/**
 * RF register read.
 *
 *
 */
 
uint16_t CC2420_REG_GET(cc2420_reg_t reg)
{
	uint16_t value;
	
	CC2420_SELECT();
	bus_spi_exchange((uint8_t) reg | 0x40);
	value = bus_spi_exchange(0);
	value <<= 8;
	value |= (uint16_t)bus_spi_exchange(0);
	CC2420_UNSELECT();
	return value;
}

/**
 * RF SPI reset.
 *
 *
 */

void CC2420_SPI_RESET(void)
{
	CC2420_REG_SET(CC_REG_MAIN, 0x0000);
	CC2420_REG_SET(CC_REG_MAIN, 0xF801);	/*bit 1 activates external clock*/
}

/**
 * Select RF channel.
 *
 * \param channel channel number to select
 *
 * \return channel value or negative (invalid channel number)
 */
 
int8_t CC2420_CHANNEL_SET(uint8_t channel)
{
	uint16_t freq;
	
	if ( (channel < 11) || (channel > 26) ) return -1;
	/* Channel values: 11-26 */
	freq = (uint16_t) channel - 11;
	freq *= 5;	/*channel spacing*/
	freq += 357; /*correct channel range*/
	freq |= 0x4000; /*LOCK_THR = 1*/
	
	CC2420_SELECT();
	bus_spi_exchange(CC_REG_FSCTRL);
	bus_spi_exchange(freq >> 8);
	bus_spi_exchange((uint8_t) freq);
	CC2420_UNSELECT();
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
 
int8_t CC2420_POWER_SET(uint8_t new_power)
{
	uint16_t power;
	
	if (new_power > 100) return -1;
	
	power = 31 * new_power;
	power /= 100;
	power += 0xA0E0;	/* 0x80e0 has 128us TX calibrate, A0E0 192us */
		
	/* Set transmitter power */
	CC2420_REG_SET(CC_REG_TXCTRL, (uint16_t) power);
	
	tx_power = (int8_t) new_power;
	return tx_power;
}

/**
 * Print RF status byte in human readable form.
 *
 * \param status status byte value
 *
 */
 
void CC2420_STAT(uint8_t status)
{
#ifdef DEBUG
	if (status & CC2420_XOSC16M_STABLE)
	{
		debug(" OSC");
	}
	
	if (status & CC2420_TX_UNDERFLOW)
	{
		debug(" TX_UND");
	}

	if (status & CC2420_ENC_BUSY)
	{
		debug(" ENC");
	}

	if (status & CC2420_TX_ACTIVE)
	{
		debug(" TX");
	}
	
	if (status & CC2420_LOCK)
	{
		debug(" LOCK");
	}
	
	if (status & CC2420_RSSI_VALID)
	{
		debug(" RSSI");
	}
#endif
}


/**
 * Set RF to receive mode.
 *
 *
 */

void rx_enable(uint8_t irq_state)
{
	CC2420_COMMAND(CC_REG_SRXON);
	
	P1IE &= ~0x80;
#if 0 
	P1IES &= ~0x80; /*rising edge*/
#endif
	P1IES |= 0x80; /*falling edge*/

	rx_flags &= ~ACTIVE;
	if (irq_state) P1IE |= 0x80;
	P1IFG &= ~0x80;
	rx_flags |= ACTIVE;
}

/**
 * Turn off RF receive mode.
 *
 *
 */

void rx_disable(void)
{
	while (rx_flags & TX_ACK);
	
	CC2420_COMMAND(CC_REG_SRFOFF);
	CC2420_COMMAND(CC_REG_SFLUSHRX);
	CC2420_COMMAND(CC_REG_SFLUSHRX);
	timer_rf_stop();
	rx_flags = 0;
	P1IE &= ~0x80;
	P1IFG &= ~0x80;
}

/**
 * Initialize RF for tx mode.
 *
 *
 */
void tx_enable(void)
{
	P1IE &= ~0x80;
	P1IFG &= ~0x80;
	timer_rf_stop();
	rx_flags = 0;
	tx_flags |= ACTIVE;
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
#ifdef HAVE_POWERSAVE
	power_set(POWER_LPM3+POWER_XT1, rf_ph);
#endif
	if (CC2420_OPEN() == pdTRUE)
	{		
		rx_enable(1);
		CC2420_CLOSE();
		return pdTRUE;
	}
	return pdFALSE;
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
#ifdef HAVE_POWERSAVE
	power_set(POWER_LPM3+POWER_XT1, rf_ph);
#endif
	if (CC2420_OPEN() == pdTRUE)
	{		
		rx_disable();
		CC2420_CLOSE();
		return pdTRUE;
	}
	return pdFALSE;
}

/**
 * Shut down RF transceiver.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	bus not free
 */
portCHAR rf_shutdown(void)
{
	if (CC2420_OPEN() == pdTRUE)
	{		
		rx_disable();
		CC2420_CLOSE();
#ifdef HAVE_POWERSAVE
		power_set(POWER_LPM4, rf_ph);
#endif
		return pdTRUE;
	}
	return pdFALSE;
}

/**
 * Send ACK.
 *
 *\param pending set up pending flag if pending > 0. 
 */
void rf_send_ack(uint8_t pending)
{
	if (CC2420_OPEN() == pdTRUE)
	{
		CC2420_SELECT();
		if(pending)
		{
			CC2420_COMMAND(CC_REG_SACKPEND);
		}
		else
		{
			CC2420_COMMAND(CC_REG_SACK);
		}
		CC2420_UNSELECT();	
		CC2420_CLOSE();
	}	
}
/**
 * Select RF channel.
 *
 * \param channel channel number to select
 *
 * \return channel value or negative (invalid channel number)
 */
 
int8_t rf_channel_set(uint8_t channel)
{	int8_t retval = -1;
	if (CC2420_OPEN() == pdTRUE)
	{	
		rx_disable();	
		retval = CC2420_CHANNEL_SET(channel);
		rx_enable(1);
		CC2420_CLOSE();
	}
	return retval;
}

/**
 * Select RF transmit power.
 *
 * \param new_power new power level (in per cent)
 *
 * \return new level or negative (value out of range)
 */
int8_t rf_power_set(uint8_t new_power)
{	int8_t retval = -1;
	if (CC2420_OPEN() == pdTRUE)
	{		
		retval = CC2420_POWER_SET(new_power);
		CC2420_CLOSE();
	}
	return retval;
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
	uint8_t i, j;
	uint8_t status;
	//rx_lauch_buffer = pvPortMalloc(sizeof(buffer_t));	


	RF_RX_LED_OFF();
	RF_TX_LED_OFF();

#ifdef HAVE_POWERSAVE
	if (rf_ph == 0)
	{
		rf_ph = power_alloc();
	}
#endif

	CC2420_INIT();
	rx_flags = tx_flags = 0;
	if (CC2420_OPEN() == pdTRUE)
	{		
		/*do SPI reset, selects clock*/
		
 		CC2420_SPI_RESET();
		
		debug("RF_CC2420: SPI init");
		j=0;
		status = 0;
		while( !(status == CC2420_XOSC16M_STABLE) && (j++ < 3) )
		{	
			debug(".");	
			/* Reset CC2420 using SPI*/
  		CC2420_SPI_RESET();
	
			pause(2);

			/* Set oscillator on */
			CC2420_COMMAND(CC_REG_SXOSCON);	
			pause(2);

			status = CC2420_COMMAND_GET(CC_REG_SNOP);
	
			pause(2);
	 		/* Wait until oscillator is stable */
			status = 0;
			i = 0;
 			do 
			{
				pause(1);
				status = CC2420_COMMAND_GET(CC_REG_SNOP);
				i++;
 			} while (!(status == CC2420_XOSC16M_STABLE) && (i<20));

		}
		if (i >= 20)
		{
			CC2420_STAT(status);
			debug("\r\nRF_CC2420: Init failed.\r\n");
			CC2420_CLOSE();
			return pdFALSE;
		}
		
		retval = pdTRUE;
		CC2420_REG_SET(CC_REG_MDMCTRL0, 0x02e2); //0x02e2 earlie
 		CC2420_REG_SET(CC_REG_MDMCTRL1, 0x0500); // Set the correlation threshold = 20    

 		CC2420_REG_SET(CC_REG_IOCFG0, 0x007F);   // Set the FIFOP threshold to maximum 
 		CC2420_REG_SET(CC_REG_SECCTRL0, 0x01C4); // Turn off "Security enable"

		/* get ID for MAC */
		rf_manfid = CC2420_REG_GET(CC_REG_MANFIDH);
		rf_manfid <<= 16;		
		rf_manfid += (uint32_t) CC2420_REG_GET(CC_REG_MANFIDL);

		CC2420_CHANNEL_SET(RF_DEFAULT_CHANNEL);

		/* Set transmitter power */
		CC2420_POWER_SET(RF_DEFAULT_POWER);


		CC2420_UNSELECT();  	/*CS up*/
	
		gpio1_irq_allocate(7, rf_isr, 0);
				
		debug("RF_CC2420: Init complete.\r\n");		
		retval = pdTRUE;
		
		CC2420_COMMAND(CC_REG_SFLUSHTX);
		tx_flags = 0;
		
		while(CC2420_SFD());
/*		rx_enable(1);*/
		CC2420_CLOSE();
		
		rf_mac_get(&mac_long);				
		debug_printf("MAC: %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\r\n",
		mac_long.address[0],mac_long.address[1],
		mac_long.address[2],mac_long.address[3],
		mac_long.address[4],mac_long.address[5],
		mac_long.address[6],mac_long.address[7]);
		rf_address_decoder_mode(RF_DECODER_NONE);
		
		return retval;
	}				
	debug("RF_CC2420: Bus allocation failed.\r\n");		
	return pdFALSE;
}

portCHAR rf_ram_set(rf_ram_access_t *ram_access)
{
	uint8_t status, counter, i, byte, byte2;
	portCHAR retval = pdFALSE;
	
	if (CC2420_OPEN() == pdTRUE)
	{						
		debug("RF_CC2420: Open.\r\n");
		CC2420_SELECT();
		CC2420_COMMAND(CC_REG_SXOSCON);
		pause(2);	
		status = CC2420_COMMAND_GET(CC_REG_SNOP);
		counter = 0;
		do
		{
			pause(1);
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
		}while (!(status & CC2420_XOSC16M_STABLE) && (counter++ < 20)); 
		if (!(status & CC2420_XOSC16M_STABLE))
		{
			CC2420_STAT(status);		
			debug("RF_CC2420: CO never stabil.\r\n");
			retval = pdFALSE;
		}
		else
		{
			byte2 =0x80;
			CC2420_UNSELECT();
			switch(ram_access->address)
			{
				case  CC_ADDR_IEEEADDR:
					
					byte=0xe0;
					CC2420_SELECT();
					/* Send ram access address */
					bus_spi_exchange((uint8_t) byte);
					bus_spi_exchange((uint8_t) byte2);
					for(i=0; i< 8; i++)
					{
						bus_spi_exchange(ram_access->data[i]);
					}
					CC2420_UNSELECT();
					break;

				case CC_ADDR_PANID:
					byte=0xe8;
					CC2420_SELECT();
					/* Send ram access address */
					bus_spi_exchange((uint8_t) byte);
					bus_spi_exchange((uint8_t) byte2);
					for(i=0; i< 2; i++)
					{
						bus_spi_exchange(ram_access->data[i]);
					}
					CC2420_UNSELECT();
					break;
				case CC_ADDR_SHORTADDR:
					byte=0xeA;
					CC2420_SELECT();
					/* Send ram access address */
					bus_spi_exchange((uint8_t) byte);
					bus_spi_exchange((uint8_t) byte2);
					for(i=0; i< 2; i++)
					{
						bus_spi_exchange(ram_access->data[i]);
					}
					CC2420_UNSELECT();
					break;

				default:		
					debug("Not supported yet\r\n");
					break;
			}
			retval = pdTRUE;
		}
		CC2420_UNSELECT();	
		CC2420_CLOSE();
	}
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
	rf_ram_access_t ram_access_variable;
	
	switch(address->addr_type)
	{
		case ADDR_802_15_4_LONG:
			address->address[8] = 0xff;
			address->address[9] = 0xff;
		case ADDR_802_15_4_PAN_LONG:
			for(i=0; i<8;i++)
			{
				ram_access_variable.data[i]= address->address[i];
			}
			ram_access_variable.address = CC_ADDR_IEEEADDR;
			rf_ram_set(&ram_access_variable);

			for(i=0; i<2;i++)
			{
				ram_access_variable.data[i]= address->address[i+8];
			}
			ram_access_variable.address = CC_ADDR_PANID;
			rf_ram_set(&ram_access_variable);
			break;
		
		case ADDR_802_15_4_SHORT:
			address->address[2] = 0xff;
			address->address[3] = 0xff;
		case ADDR_802_15_4_PAN_SHORT:
			for(i=0; i<2;i++)
			{
				ram_access_variable.data[i]= address->address[i];
			}
			ram_access_variable.address = CC_ADDR_SHORTADDR;
			rf_ram_set(&ram_access_variable);
			for(i=0; i<2;i++)
			{
				ram_access_variable.data[i]= address->address[i+2];
			}
			ram_access_variable.address = CC_ADDR_PANID;
			rf_ram_set(&ram_access_variable);
			break;
		
		default:
			break;
	}			
}
/*
void CC2420_RAM_READ(cc2420_addr_t ram_access)
{
	uint8_t value[8], status, counter, i, byte, byte2;
	if (CC2420_OPEN() == pdTRUE)
	{						
		debug("RF_CC2420: Open.\r\n");
		CC2420_SELECT();
		CC2420_COMMAND(CC_REG_SXOSCON);
		pause(2);	
		status = CC2420_COMMAND_GET(CC_REG_SNOP);
		counter = 0;
		do
		{
			pause(1);
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
		}while (!(status & CC2420_XOSC16M_STABLE) && (counter++ < 20)); 
		if (!(status & CC2420_XOSC16M_STABLE))
		{
			CC2420_STAT(status);		
			debug("RF_CC2420: CO never stable.\r\n");
		}
		else
		{
			byte2 =0xa0;
			CC2420_UNSELECT();
			switch(ram_access)
			{
				case  CC_ADDR_IEEEADDR:
					byte=0xe0;
					CC2420_SELECT();
					// Send ram access address
					bus_spi_exchange((uint8_t) byte);
					bus_spi_exchange((uint8_t) byte2);
					for(i=0; i< 8; i++)
					{
						value[i] = bus_spi_exchange(0);
					}
					CC2420_UNSELECT();
					
					debug("IEEE\r\n");
					for(i=0; i< 8; i++)
					{
						if(i) debug(":");
						debug_hex(value[i]);
					}
					break;

				case CC_ADDR_PANID:
					byte=0xe8;
					CC2420_SELECT();
					// Send ram access address
					bus_spi_exchange((uint8_t) byte);
					bus_spi_exchange((uint8_t) byte2);
					for(i=0; i< 2; i++)
					{
						value[i] = bus_spi_exchange(0);
					}
					CC2420_UNSELECT();
					debug("\r\npan-id\r\n");
					for(i=0; i< 2; i++)
					{
						if(i) debug(":");
						debug_hex(value[i]);
					}


					break;
				case CC_ADDR_SHORTADDR:
					byte=0xea;
					CC2420_SELECT();
					// Send ram access address
					bus_spi_exchange((uint8_t) byte);
					bus_spi_exchange((uint8_t) byte2);
					for(i=0; i< 2; i++)
					{
						value[i] = bus_spi_exchange(0);
					}
					CC2420_UNSELECT();
					debug("\r\nshort\r\n");
					for(i=0; i< 2; i++)
					{
						if(i) debug(":");
						debug_hex(value[i]);
					}
					
					break;
				default:
					debug("Not supported yet\r\n");
					CC2420_UNSELECT();
					break;
			}
		}
		CC2420_UNSELECT();	
		CC2420_CLOSE();
	}
}*/
/**
 * Set address decoder on/off.
 *
 * \param mode RF_DECODER_NONE/RF_DECODER_ON/RF_DECODER_COORDINATOR 
 * \return pdTRUE operation successful
 */
portCHAR rf_address_decoder_mode(rf_address_mode_t mode)
{
	uint8_t status, counter;
	portCHAR retval = pdFALSE;
	
	if (CC2420_OPEN() == pdTRUE)
	{						
		debug("RF_CC2420: Open.\r\n");
		CC2420_SELECT();
		CC2420_COMMAND(CC_REG_SXOSCON);
		pause(2);

		status = CC2420_COMMAND_GET(CC_REG_SNOP);
		counter = 0;
		do
		{
			pause(1);
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
		}while (!(status & CC2420_XOSC16M_STABLE) && (counter++ < 20)); 
		if (!(status & CC2420_XOSC16M_STABLE))
		{
			CC2420_STAT(status);
			debug("RF_CC2420: CO never stable.\r\n");
			retval = pdFALSE;
		}
		else
		{
			switch(mode)
			{
#ifndef HAVE_NRP
				case RF_DECODER_COORDINATOR:
					CC2420_REG_SET(CC_REG_MDMCTRL0, 0x1AF2);	/*Coordinator, Addres-decode, NO AUTO ACK=E */
					CC2420_REG_SET(CC_REG_IOCFG0, 0x087f);		/* Enable receive beacon if address decoder is enabled */
					break;
				case RF_DECODER_ON:
					CC2420_REG_SET(CC_REG_MDMCTRL0, 0x0AF2);	/*Addres-decode, AUTO ACK */
					CC2420_REG_SET(CC_REG_IOCFG0, 0x087f);		/* Enable receive beacon if address decoder is enabled */
					break;
#endif
				default:
					CC2420_REG_SET(CC_REG_MDMCTRL0, 0x02E2);
					break;
			}
			retval = pdTRUE;
		}
		CC2420_UNSELECT();	
		CC2420_CLOSE();
	}
	return retval; 
}


/**
 * Channel energy detect.
 *
 * Coordinator use this function detect best channel for PAN-network.
 * \return RSSI-energy level dBm.
 * \return 0	operation failed.
 */
 #ifndef HAVE_NRP
int8_t rf_analyze_rssi(void)
{
	uint8_t status, counter, i;
	int16_t sum=0;
	int8_t retval = 0, temp=0;
	
	if (CC2420_OPEN() == pdTRUE)
	{						
		debug("RF_CC2420: Open.\r\n");
		CC2420_COMMAND(CC_REG_SRXON);

		status = CC2420_COMMAND_GET(CC_REG_SNOP);
		counter = 0;
		do
		{
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
		}while (!(status & CC2420_RSSI_VALID) && (counter++ < 130)); 
		if (!(status & CC2420_RSSI_VALID))
		{
			CC2420_STAT(status);
			debug("RF_CC2420: RSSI never valid.\r\n");
			retval = 0;
		}
		else
		{
			for(i=0; i<8; i++)
			{
				temp = (int8_t)CC2420_REG_GET(CC_REG_RSSI);
				temp -= 45;
				sum += (int16_t)temp;
				pause_us(16);				/* waiting one symbol period */
			}
			sum /=8;
			retval = (int8_t)sum;
		}
		CC2420_UNSELECT();	
		CC2420_CLOSE();
	}
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
	uint8_t status, counter, cca=1;
	portCHAR retval = pdTRUE;

	if (CC2420_OPEN() == pdTRUE)
	{
		CC2420_COMMAND(CC_REG_SRXON);

		status = CC2420_COMMAND_GET(CC_REG_SNOP);
		counter = 0;
		do
		{
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
		}while (!(status & CC2420_RSSI_VALID) && (counter++ < 130)); 
		if (!(status & CC2420_RSSI_VALID))
		{
			CC2420_STAT(status);
			debug("RF_CC2420: RSSI never valid.\r\n");
			retval = pdFALSE;
		}
		if (retval == pdTRUE)
		{
			switch (slotted)
			{
				case 1:
			
				if(CC2420_CCA())
				{
					counter=0;
					cca=1;
					while(cca!=0) 
					{
						if(counter > 1)
							cca=0;
						pause_us(250);
						if(!(CC2420_CCA()))
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
					if(!(CC2420_CCA()))
					{
						retval = pdFALSE;
					}
					break;
			}
		}
		CC2420_CLOSE();
	}
	return retval;		
}
#endif

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
	uint8_t status, counter, i;
	portCHAR retval = pdTRUE;
	int16_t length = 	buffer->buf_end - buffer->buf_ptr;

	if ((rx_flags & ACTIVE) == 0) rf_rx_enable();

	if (CC2420_OPEN() == pdTRUE)
	{
		CC2420_COMMAND(CC_REG_SFLUSHTX);
		status = CC2420_COMMAND_GET(CC_REG_SNOP);
		counter = 0;
		do
		{
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
			status = CC2420_COMMAND_GET(CC_REG_SNOP);
		}while (!(status & CC2420_RSSI_VALID) && (counter++ < 130));

		/*CC2420_SELECT();
		bus_spi_exchange((cc2420_reg_t) CC_REG_TXFIFO);

		bus_spi_exchange(length+2);
				
		status = 0;
		for (i = buffer->buf_ptr; i < (buffer->buf_ptr+length) ; i++)
		{
			bus_spi_exchange(buffer->buf[i]);
			if (!CC2420_CCA() || (rx_flags & TX_ACK))
			{
				i = 250;
				status = 1;
			}
		}
		
		if (status)
		{
			CC2420_UNSELECT();
			CC2420_COMMAND(CC_REG_SFLUSHTX);
			CC2420_CLOSE();
			return pdFALSE;
			//return pdTRUE+1;
		}
		bus_spi_exchange(0);
		bus_spi_exchange(0);
		CC2420_UNSELECT();*/

		tx_enable();
		i= 0;
		while (i++ < 3)
		{
			CC2420_COMMAND(CC_REG_STXONCCA);
			counter = 0;	
			do
			{
				status = CC2420_COMMAND_GET(CC_REG_SNOP);
				status = CC2420_COMMAND_GET(CC_REG_SNOP);
			}while ( !(status & CC2420_TX_ACTIVE)  && (counter++ < 130)); 
			if (status & CC2420_TX_ACTIVE) i = 200;
		}

		if (!(status & CC2420_TX_ACTIVE))
		{
			CC2420_STAT(status);
			//CC2420_COMMAND(CC_REG_SFLUSHTX);
			retval = pdFALSE;
		}

		if (retval == pdTRUE)
		{
			RF_TX_LED_ON();
			CC2420_SELECT();
			bus_spi_exchange((cc2420_reg_t) CC_REG_TXFIFO);
			bus_spi_exchange(length+2);
				
			for (i = buffer->buf_ptr; i < (buffer->buf_ptr+length) ; i++)
			{
				bus_spi_exchange(buffer->buf[i]);
			}
			bus_spi_exchange(0);
			bus_spi_exchange(0);
			CC2420_UNSELECT();
			while (!CC2420_SFD())
			{
			}

			while (CC2420_SFD())
			{ /*wait for transmit to complete*/
			}
		}

		//while(CC2420_SFD());
		tx_flags = 0;
		rx_enable(1);
		CC2420_CLOSE();
		RF_TX_LED_OFF();
		return retval;		
	}
	debug("RF_CC2420: Bus allocation failed.\r\n");
	return pdTRUE+1;
}

/**
 * Transmit packet without channel check.
 *
 * Missing feature: address type check
 *
 * \param mac RF HW address
 * \param dst destination HW address
 * \param buffer data buffer pointer
 * \param length length of data
 *
 * \return pdTRUE
 * \return pdFALSE	bus reserved
 */
#ifndef HAVE_NRP
portCHAR rf_write_no_cca(buffer_t *buffer)
{
	uint8_t status, counter, i;
	portCHAR retval = pdTRUE;
	int16_t length = 	buffer->buf_end - buffer->buf_ptr;
	
	if (CC2420_OPEN() == pdTRUE)
	{						
		if (rx_flags & ACTIVE)
		{
			if ( CC2420_FIFOP() || CC2420_SFD() )
			{
				if (CC2420_FIFOP())
				{
					CC2420_COMMAND(CC_REG_SFLUSHTX);
					CC2420_COMMAND(CC_REG_SFLUSHRX);
				}
				
				retval = pdFALSE;		
			}
		}
		tx_enable();
		if ( (length <= 128) && (retval == pdTRUE) )
		{

			CC2420_SELECT();
			bus_spi_exchange((cc2420_reg_t) CC_REG_TXFIFO);

			bus_spi_exchange(length+2); 

			for (i = buffer->buf_ptr; i < (buffer->buf_ptr+length) ; i++)
			{
				bus_spi_exchange(buffer->buf[i]);
			}
			bus_spi_exchange(0);
			bus_spi_exchange(0);
			CC2420_UNSELECT();
			CC2420_COMMAND(CC_REG_STXON);
			counter = 0;	
			do
			{
				status = CC2420_COMMAND_GET(CC_REG_SNOP);
				status = CC2420_COMMAND_GET(CC_REG_SNOP);
			}while ( !(status & CC2420_TX_ACTIVE)  && (counter++ < 200));

			if (status & CC2420_TX_ACTIVE)
			{
				RF_TX_LED_ON();
				while (!CC2420_SFD())
				{
				}

				while (CC2420_SFD())
				{ /*wait for transmit to complete*/
				}
				CC2420_COMMAND(CC_REG_SFLUSHTX);
			}
			else
			{
				CC2420_STAT(status);
				CC2420_COMMAND(CC_REG_SFLUSHTX);

				retval = pdFALSE;
			}

		}
		if ((retval == pdTRUE) && (length > 128))
		{
			debug("Packet too long(");
			debug_int(length);
			debug(").\r\n");
			retval = pdFALSE;
		}
		while(CC2420_SFD());
		rx_enable(1);
		tx_flags = 0;
		//timer_rf_launch(800 / 32); /*832 us in timer ticks*/
		CC2420_CLOSE();
		RF_TX_LED_OFF();				
		return retval;		
	}
	debug("RF_CC2420: Bus allocation failed.\r\n");
	return pdFALSE;
}
#endif
uint8_t rf_state(void)
{
	uint8_t byte = 0;
	
	if (CC2420_OPEN() == pdTRUE)
	{		
		if (rx_flags & ACTIVE)
		{
			if ( CC2420_SFD() )
			{
				byte |= 0x08;
			}
			if ( CC2420_CCA() )
			{
				byte |= 0x04;
			}
			if ( CC2420_FIFOP() )
			{
				byte |= 0x02;
			}
			if ( CC2420_FIFO() )
			{
				byte |= 0x01;
			}
		}
		CC2420_CLOSE();
	}
	else
	{				
		debug("RF_CC2420: Bus allocation failed.\r\n");
	}
	return byte;
}

void rf_rxflush(void)
{
	if (CC2420_OPEN() == pdTRUE)
	{		
		CC2420_COMMAND(CC_REG_SFLUSHRX);
		CC2420_COMMAND(CC_REG_SFLUSHRX);
		
		CC2420_CLOSE();
	}
}

extern uint8_t bus_serial[4];

portCHAR rf_mac_get(sockaddr_t *address)
{
	if (rf_manfid)
	{
		address->addr_type = ADDR_802_15_4_LONG;
		address->address[9] = 0xFF;
		address->address[8] = 0xFF;
		address->address[7] = (rf_manfid >> 24);
		address->address[6] = (rf_manfid >> 16);
		address->address[5] = (rf_manfid >> 8);
		address->address[4] = (rf_manfid);
		address->address[3] = bus_serial[0];
		address->address[2] = bus_serial[1];
		address->address[1] = bus_serial[2];
		address->address[0] = bus_serial[3];
		mac_set(address);

		return pdTRUE;
	}
	return pdFALSE;
}



/**
	* RF receive callback
	* \param param not used
	*/	
void rf_rx_callback( void *param )
{
	buffer_t *rf_buf = 0;
	uint8_t i=0;
	//while(CC2420_SFD());
	if (CC2420_FIFOP() == 0) return;
	
	
	rf_buf=0;
	rf_buf = stack_buffer_get(0);
	if (rf_buf != 0)
	{
		uint8_t length;
		rf_buf->options.rf_dbm	 = -90;
		rf_buf->options.rf_lqi	 = 0;
		rf_buf->buf_end = 0;
		rf_buf->buf_ptr = 0;
		rf_buf->to = MODULE_NONE;		

		if(CC2420_OPEN() == pdFALSE)
		{
			debug("bus err\r\n");
			stack_buffer_free(rf_buf);
			while(1){}
			return;
		}
		RF_RX_LED_ON();
		CC2420_SELECT();
		bus_spi_exchange((cc2420_reg_t) CC_REG_RXFIFO | 0x40); /*read bit up!*/
		length = bus_spi_exchange(0) & 0x7F;
		if( length < 128 && length > 4)
		{
			length -= 2;
			rf_buf->buf_ptr = 0;
			rf_buf->buf_end = length;
			for (i=0; i < length ; i++)
			{
				rf_buf->buf[i] = bus_spi_exchange(0);	
			}

			rf_buf->options.rf_dbm = ((int8_t) bus_spi_exchange(0)) - 45;
			rf_buf->options.rf_lqi = bus_spi_exchange(0);

			if (rf_buf->options.rf_lqi & 0x80)
			{	/*CRC OK*/
				rf_buf->options.rf_lqi &= 0x7F;
#ifdef HAVE_DRI
				rf_buf->to = MODULE_DRI;
#else
				rf_buf->to = MODULE_RF_802_15_4;
#endif
			}
			else
			{	/*CRC failed*/
				CC2420_UNSELECT();
				rf_buf->to = MODULE_NONE;
			}
		}	/*end data in buffer*/
		else
		{
			CC2420_UNSELECT();
			CC2420_COMMAND(CC_REG_SFLUSHRX);
			CC2420_COMMAND(CC_REG_SFLUSHRX);
			rf_buf->to = MODULE_NONE;
		}
		CC2420_UNSELECT();

		if (CC2420_FIFO() == 0 && CC2420_FIFOP())
		{	//buffer overflow
			CC2420_COMMAND(CC_REG_SFLUSHRX);
			CC2420_COMMAND(CC_REG_SFLUSHRX);
		}

		rx_flags = ACTIVE;
		P1IES |= 0x80; /*falling edge*/
		P1IE |= 0x80;
		CC2420_CLOSE();

		if (rf_buf->to != MODULE_NONE)
		{
			mac_push(rf_buf);
			RF_RX_LED_OFF();
			
		}
		else
		{
			stack_buffer_free(rf_buf);
		}
		RF_RX_LED_OFF();
		return;
	}
	else
	{
		if(CC2420_OPEN() == pdFALSE)
		{
			debug("no sem for bus\r\n");
			while(1){}
		}
		else
		{
			CC2420_SELECT();
			CC2420_UNSELECT();
			CC2420_COMMAND(CC_REG_SFLUSHRX);
			CC2420_COMMAND(CC_REG_SFLUSHRX);
			CC2420_CLOSE();
			RF_RX_LED_OFF();
		}
		rx_flags = ACTIVE;
		P1IES |= 0x80; /*falling edge*/
		P1IE |= 0x80;
		CC2420_CLOSE();
	}
	RF_RX_LED_OFF();
	return;
}

#ifdef HAVE_MAC_15_4
extern void mac_rx_push(void);
#endif

void rf_timer_callback(void);
void rf_timer_callback(void)
{
	mac_15_4_event_t event;
	event.id = MAC_TIMER_INT_CB;  
	if(xQueueSendFromISR(rf_802_15_4_queue, &event, pdFALSE) == pdTRUE)
	{
		taskYIELD();
	}
}
#if 0
void rf_isr(void)
{
	//portBASE_TYPE prev_task = pdFALSE;
	event_t event;

	//if (rx_flags & ACTIVE)
	//{	
		if (CC2420_FIFOP())
		{
#ifdef HAVE_MAC_15_4
			mac_rx_push();
#else
			event.process = rf_rx_callback;
			event.param = 0;
		/*	P1IE &= ~0x80;*/
			//prev_task = xQueueSendFromISR(events, &event, prev_task);
			xQueueSendFromISR(events, &event, 0);
			/*if (prev_task == pdTRUE)
			{
				taskYIELD();
			}*/
#endif
			rx_flags |= TX_ACK;
			RF_RX_LED_ON();
		}
		timer_rf_launch(MAC_IFS); /*832 us in timer ticks*/
	//}
	P1IFG &= ~0x80;
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}
#endif

void rf_isr(void)
{
	mac_15_4_event_t event;
	RF_RX_LED_ON();
	P1IFG &= ~0x80;
	if (rx_flags & ACTIVE)
	{
		if (CC2420_FIFOP())
		{
			event.id = MAC_RX_MES;  
			if(xQueueSendFromISR(rf_802_15_4_queue, &event, pdFALSE) ==pdTRUE)
			{
				taskYIELD();
			}
		}
	}
	
	RF_RX_LED_OFF();
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

#ifdef TX_TES_MODE_LOOP
portCHAR start_tx_test_mode(uint8_t channel)
{
	portCHAR retval = pdFALSE;
	uint8_t i, j;
	uint8_t status;
	
	RF_RX_LED_ON();
	RF_TX_LED_ON();
	if(channel < 11 || channel > 26)
		return pdFALSE;
	if (CC2420_OPEN() == pdTRUE)
	{		
		/*do SPI reset, selects clock*/
		CC2420_SELECT();	
		retval = pdTRUE;
 		CC2420_REG_SET(CC_REG_MDMCTRL1, 0x050C); // Set the correlation threshold = 20    
		CC2420_CHANNEL_SET(channel);
		CC2420_COMMAND(CC_REG_STXON);
		CC2420_UNSELECT();	
		CC2420_CLOSE();
		return retval;
	}				
	debug("RF_CC2420: Bus allocation failed.\r\n");		
	return pdFALSE;
}

portCHAR stop_tx_test_mode(void)
{
	portCHAR retval = pdFALSE;
	uint8_t i, j;
	uint8_t status;
	RF_RX_LED_OFF();
	RF_TX_LED_OFF();
	if (CC2420_OPEN() == pdTRUE)
	{		
		/*do SPI reset, selects clock*/
		CC2420_SELECT();	
		retval = pdTRUE;
 		CC2420_REG_SET(CC_REG_MDMCTRL1, 0x0500); // Set the correlation threshold = 20    
		CC2420_COMMAND(CC_REG_SRXON);
		CC2420_UNSELECT();	
		CC2420_CLOSE();
		return retval;
	}				
	debug("RF_CC2420: Bus allocation failed.\r\n");		
	return pdFALSE;
}
#endif
