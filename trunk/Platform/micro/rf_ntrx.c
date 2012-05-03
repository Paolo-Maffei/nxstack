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
 * \file rf_ntrx.c
 * \brief micro RF driver for the Nanotron NA5TR1 radio.
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
#include "debug.h"
#include "stack.h"
#include "mac.h"

#include "gpio.h"
#include "rf_ntrx.h"

#include "progmem.h"

#ifdef HAVE_POWERSAVE
#include "powersave.h"
#endif

#define RF_NO_TASK

#ifndef TRX_DEFAULT_POWER
#define TRX_DEFAULT_POWER 100
#endif

#ifndef TRX_DEFAULT_CHANNEL
#define TRX_DEFAULT_CHANNEL 1
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

#define TxEND (0x01 << NA_TxEnd_B)
#define RxEND (0x01 << NA_RxEnd_B)
#define RxOVERFLOW (0x01 << NA_RxOverflow_B)

/*RF SPI mode open and close*/
#define TRX_OPEN(x) bus_select(0x0d, BUS_SPI, BUS_CLOCK_INVERT | BUS_CLOCK_115kHZ)
#define TRX_CLOSE(x) bus_free()

/*These macros control RF select signal and assume correct bus /dev/ttyS0state*/
//#define TRX_SELECT(x) P3OUT |= 0x01
//#define TRX_UNSELECT(x) P3OUT &= ~0x01

#define TRX_UNSELECT(x) P2OUT |= 0x08
#define TRX_SELECT(x) P2OUT &= ~0x08

/*I/O init and status readout macros*/
#define TRX_INIT(x) P5DIR &= ~0x70; P1DIR &= ~0x80; P5SEL &= ~0x70; P1SEL &= ~0x80

#define TRX_FIFO(x) ( (P5IN >> 5) & 1 ) 
#define TRX_FIFOP(x) ( (P5IN >> 6) & 1 )
#define TRX_CCA(x) ( (P5IN >> 4) & 1 )
#define TRX_SFD(x) ( (P1IN >> 7) & 1 )

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

#define NO_TX 0x40

uint8_t RxCRC2Mode = FALSE;
uint8_t trxState = TRX_STATE_IDLE;
uint8_t trxRun = FALSE;



#define TRX_DEBUG



/**
 * Receiver lock.
 *
 */

void rf_isr(void);
portCHAR rf_rx_enable(void);
portCHAR rf_rx_disable(void);
uint8_t rf_state(void);
void rf_config(void);
void rf_rxflush(void);

void TRX_COMMAND(uint8_t command);
uint8_t TRX_COMMAND_GET(uint8_t command);
int8_t TRX_CHANNEL_SET(uint8_t channel);
int8_t TRX_POWER_SET(uint8_t new_power);
void TRX_STAT(uint8_t status);

void TRX_SPI_RESET(void);

int8_t TRX_REG_GET(uint8_t reg, uint8_t len, uint8_t *buf);
int8_t TRX_SPI_WRITE(uint8_t addr, uint8_t *data, uint8_t len);
int8_t TRX_REG_SET(TRX_W_REG reg, TRX_REG_MASK mask, uint8_t buf);
int8_t TRX_REG_SET_N(TRX_W_REG reg, TRX_REG_MASK mask, uint8_t len, uint8_t *buf);
void TRX_SPI_SETUP(void);
int8_t rf_check_TRX_version(void);
void TRX_Set_Index_Reg(uint8_t page);
void NTRX_SET_AGC_VALUES(uint8_t bw, uint8_t sdur, uint8_t sr);
void TRX_SET_RX_IQ(uint8_t bw, uint8_t sdur);
void TRX_SET_THR(uint8_t bw, uint8_t sdur);
void TRX_SET_TX_IQ(uint8_t bw, uint8_t sdur);
void TRX_CAL(void);
void TRXRxLoCalibration(void);
void TRXTxLoCalibration(void);
uint8_t NTRXGetRxCRC2mode(void);
uint8_t NTRXRestart (void);
void TRX_Setup_Mode(uint8_t fdma, uint8_t symbolw, uint8_t symbolr);
portCHAR rf_address_decoder_mode(uint8_t mode);

void rx_enable(uint8_t irq_state);
void rx_disable(void);
void tx_enable(void);

#ifdef HAVE_POWERSAVE
static xPowerHandle rf_ph = 0;
#endif


/**
 * MAC address.
 *
 */

extern sockaddr_t mac_long;
extern uint8_t irq_wanker;
extern uint8_t irq_buf;

portCHAR rf_mac_get(sockaddr_t *address);

/**
 * RF single byte command.
 *
 * \param command RF command to send
 *
 */
 
void TRX_COMMAND(uint8_t command)  
{
	TRX_SELECT();
	bus_spi_exchange(command);
	TRX_UNSELECT();
}

/**
 * RF single byte command with return value.
 *
 * \param command RF command to send
 *
 * \return status byte from SPI
 */
 
uint8_t TRX_COMMAND_GET(uint8_t command)  
{
	uint8_t value;
	
	TRX_SELECT();
	value = bus_spi_exchange(command);
	TRX_UNSELECT();
	return value;
}


int8_t TRX_SPI_WRITE(uint8_t addr, uint8_t *data, uint8_t len)
{
	TRX_SELECT();
	
	bus_spi_exchange(0x80 | len);
	bus_spi_exchange(addr);
	while(len--)
		bus_spi_exchange(*(data++));

	TRX_UNSELECT();

	return(0);
}

//uint8_t TRX_SPI_READ(uint8_t address, uint8_t *data, uint8_t len)
//{
	

/**
 * RF register write.
 *
 *
 */
int8_t TRX_REG_SET(TRX_W_REG reg, TRX_REG_MASK mask, uint8_t buf)
{
	uint8_t orig_reg;

// Read the current register value from the shadow register
	orig_reg = TRXShadowReg[reg];
// Reset the unmasked bits to zero
	orig_reg &= ~mask;
// Set the new bits
	orig_reg |= buf;

	TRX_SELECT();
	
// Write the length to the SPI, the high bit must be set for write operations
	bus_spi_exchange(0x81);

// Write the address to the SPI
	bus_spi_exchange((uint8_t) reg);

	bus_spi_exchange(orig_reg);

	debug("SPI WRITE AT:");
	debug_hex(reg);
	debug(" DATA:");
	debug_hex(orig_reg);
	debug("\r\n");


	TRX_UNSELECT();

	return(0);
}


int8_t TRX_REG_SET_N(TRX_W_REG reg, TRX_REG_MASK mask, uint8_t len, uint8_t *buf)
{
	if(len == 0 || len > 0x80)
		return(-1);

	TRX_SELECT();
	
// Write the length to the SPI, the high bit must be set for write operations
	bus_spi_exchange((uint8_t) (len | 0x80));

// Write the address to the SPI
	bus_spi_exchange((uint8_t) reg);

	while(len--)
	{
		 bus_spi_exchange(*buf++);
	}
	TRX_UNSELECT();

	return(0);
}
	
	

/**
 * RF register read.
 *
 *
 */
int8_t TRX_REG_GET(uint8_t reg, uint8_t len, uint8_t *buf)
{
	if(len == 0 || len > 0x80)
		return(-1);
	
	TRX_SELECT();
	/* write the length to the SPI */
	bus_spi_exchange((uint8_t) (len & 0x7f));

	/* write the address of the register to the SPI */
	bus_spi_exchange((uint8_t) reg);

	while(len--)
	{
		*buf++ = bus_spi_exchange(0xff);
	}
	TRX_UNSELECT();

	return(0);
}




/**
 * RF SPI reset.
 *
 *
 */
void TRX_SPI_SETUP(void)
{
	TRX_REG_SET(TRX_REG_SpiBitOrder, NTRX_SpiBitOrder_MASK, 0xc3);
}

/**
 * Select RF channel.
 *
 * \param channel channel number to select
 *
 * \return channel value or negative (invalid channel number)
 */
 
int8_t TRX_CHANNEL_SET(uint8_t channel)
{
#ifdef TRX_DEBUG
	debug("Doing nothing in rf_channel_set()\r\n");
#endif
	return (int8_t) channel;
}

/**
 * Select RF transmit power.
 *
 * \param new_power new power level (in per cent)
 *
 * \return new level or negative (value out of range)
 */
 
int8_t TRX_POWER_SET(uint8_t new_power)
{
	return tx_power;
}

/**
 * Print RF status byte in human readable form.
 *
 * \param status status byte value
 *
 */
 void TRX_STAT(uint8_t status)
{
}


/**
 * Set RF to receive mode.
 *
 *
 */

void rx_enable(uint8_t irq_state)
{
#ifdef TRX_DEBUG
	debug("Enabling RF\r\n");
#endif

	TRX_REG_SET(TRX_REG_RxCmdStop, NTRX_RxCmdStop_MASK, 0x00);
}

/**
 * Turn off RF receive mode.
 *
 *
 */
void rx_disable(void)
{
#ifdef TRX_DEBUG
	debug("Disabling RF\r\n");
#endif

	TRX_REG_SET(TRX_REG_RxCmdStop, NTRX_RxCmdStop_MASK, 0x01);
}

/**
 * Initialize RF for tx mode.
 *
 *
 */
void tx_enable(void)
{
#ifdef TRX_DEBUG
	debug("Doing nothing in tx_enable()\r\n");
#endif
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
	rx_enable(0x00);
	return(pdFALSE);
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
	rx_disable();
	return(pdFALSE);
}


/**
 * Send ACK.
 *
 *\param pending set up pending flag if pending > 0. 
 */
void rf_send_ack(uint8_t pending)
{
}
/**
 * Select RF channel.
 *
 * \param channel channel number to select
 *
 * \return channel value or negative (invalid channel number)
 */
 
int8_t rf_channel_set(uint8_t channel)
{
	int8_t retval = -1;
	if (TRX_OPEN() == pdTRUE)
	{	
		rx_disable();	
		retval = TRX_CHANNEL_SET(channel);
		rx_enable(1);
		TRX_CLOSE();
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
{	
	int8_t retval = -1;
	if (TRX_OPEN() == pdTRUE)
	{		
		retval = TRX_POWER_SET(new_power);
		TRX_CLOSE();
	}
	return retval;
}

/**
 * Check NA5TR1 version and revision.
 *
 * \return pdTRUE if version and revision ok, pdFALSE if either/both wrong
 */
int8_t rf_check_TRX_version(void)
{
	uint8_t buff[2];
	buff[0] = 0xc3;

	TRX_REG_SET(0x00, 0x01, buff[0]);
	TRX_REG_GET(TRX_Version_O, 1, buff);
	TRX_REG_GET(TRX_Revision_O, 1, &(buff[1]));
	
#ifdef TRX_DEBUG
	debug("Ver:Rev:");
	debug_hex(buff[0]);
	debug_hex(buff[1]);
	debug("\r\n");
#	endif

	if (buff[0] == TRX_Version_I && buff[1] == TRX_Revision_I)
	{
		debug("VER & REV OK\r\n");
		return pdTRUE;
	}
	else
	{
		debug("VER/REV check FAIL\r\n");
		return pdFALSE;
	}
}






void TRX_Set_Index_Reg(uint8_t page)
{
debug("TRX_Set_Index_Reg() 1. for page ");
debug_hex(page);
debug("\r\n");
	if (page != TRXShadowReg[TRX_REG_RamIndex])
	{
debug("TRX_Set_Index_Reg() 2.\r\n");
		TRXShadowReg[TRX_REG_RamIndex] = page;
		TRX_REG_SET(TRX_REG_RamIndex, NTRX_RamIndex_MASK, page);
	}
debug("TRX_Set_Index_Reg() 3.\r\n");
}





void NTRX_SET_AGC_VALUES(uint8_t bw, uint8_t sdur, uint8_t sr)
{
	uint8_t regvalue[2];
	uint8_t thr1, thr2, dtime, nreg, itime, agcA, agcROff;

	thr2 = 1;
	dtime = 12;
	nreg = 0;
	itime = 122;

	TRX_REG_SET(TRX_REG_SymbolDur, NTRX_SymbolDur_MASK, sdur);
	TRX_REG_SET(TRX_REG_SymbolRate, NTRX_SymbolRate_MASK, sr<<NA_SymbolRate_LSB);

	if (bw == TRX_BANDWIDTH_80MHz)
	{
		TRX_REG_SET(TRX_REG_FdmaEnable, NTRX_FdmaEnable_MASK, FALSE<<NA_FdmaEnable_B);
		TRX_REG_SET(TRX_REG_PulseDetDelay, NTRX_PulseDetDelay_MASK, 4);
		TRX_REG_SET(TRX_REG_LnaFreqAdjust, NTRX_LnaFreqAdjust_MASK, TRUE);

		thr1 = 4;
		agcA = 15;
		agcROff = 5;
	}
	else
	{
		TRX_REG_SET(TRX_REG_FdmaEnable, NTRX_FdmaEnable_MASK, TRUE<<NA_FdmaEnable_B);
		TRX_REG_SET(TRX_REG_PulseDetDelay, NTRX_PulseDetDelay_MASK, 2);
		TRX_REG_SET(TRX_REG_LnaFreqAdjust, NTRX_LnaFreqAdjust_MASK, FALSE);

		thr1 = 5;
		agcA = 22;
		agcROff = 0;
	}

	TRX_REG_SET(TRX_REG_AgcThresHold1, NTRX_AgcThresHold1_MASK, thr1);
	TRX_REG_SET(TRX_REG_AgcThresHold2, NTRX_AgcThresHold2_MASK, thr2);
	TRX_REG_SET(TRX_REG_AgcDeadTime, NTRX_AgcDeadTime_MASK, dtime);
	TRX_REG_SET(TRX_REG_AgcNregLength, NTRX_AgcNregLength_MASK, nreg<<NA_AgcNregLength_LSB);

	regvalue[0] = itime; 
	regvalue[1] = 0;	

	TRX_REG_SET_N(TRX_REG_AgcIntTime, NTRX_AgcIntTime_MASK, 2, regvalue);
	TRX_REG_SET(TRX_REG_AgcAmplitude, NTRX_AgcAmplitude_MASK, agcA);
	TRX_REG_SET(TRX_REG_AgcRangeOffset, NTRX_AgcRangeOffset_MASK, agcROff);
}



void TRX_SET_RX_IQ(uint8_t bw, uint8_t sdur)
{
	TRX_Set_Index_Reg(0x30);
	
	if(bw == TRX_BANDWIDTH_80MHz)
	{
		TRX_REG_SET_N((NA_RamD3lPatI_O & 0xff), 0x00, 16, &(NA5TR1_Rx_80MHz[0]));
	}
	else if(bw == TRX_BANDWIDTH_22MHz)
	{
		TRX_REG_SET_N((NA_RamD3lPatI_O & 0xff), 0x00, 4, &(NA5TR1_Rx_22MHz[0]));
	}
	else
		return;
	
	TRX_Set_Index_Reg(0x31);

	if(bw == TRX_BANDWIDTH_80MHz)
	{
		TRX_Set_Index_Reg(0x01);
		TRX_REG_SET_N((NA_RamD3lPatQ_O & 0xff), 0x00, 16, &(NA5TR1_Rx_80MHz[16]));
	}
	else if(bw == TRX_BANDWIDTH_22MHz)
	{
		TRX_Set_Index_Reg(0x01);
		TRX_REG_SET_N((NA_RamD3lPatQ_O & 0xff), 0x00, 4, &(NA5TR1_Rx_22MHz[4]));
	}
	else
	{
		TRX_Set_Index_Reg(0x00);
		return;
	}

	TRX_Set_Index_Reg(0x00);

	return;
}	


void TRX_SET_THR(uint8_t bw, uint8_t sdur)
{
	TRX_Set_Index_Reg(0x32);
	if (bw == TRX_BANDWIDTH_80MHz)
	{
		uint8_t thr[] = { 0x00, 0x36, 0x36, 0x36, 0x36 };
		TRX_REG_SET_N(0x80, 0x00, 5, thr);
	}	
	else
	{
		uint8_t thr[] = { 0x00, 0x11, 0x11, 0x11, 0x11 };
		TRX_REG_SET_N(0x80, 0x00, 5, thr);
	}

	TRX_Set_Index_Reg(0x00);
}








void TRX_SET_TX_IQ(uint8_t bw, uint8_t sdur)
{
	TRX_Set_Index_Reg(0x20);
	TRX_REG_SET(TRX_REG_CsqMemAddrInit, NTRX_CsqMemAddrInit_MASK, TRUE<<NA_CsqMemAddrInit_B);

	if(bw == TRX_BANDWIDTH_80MHz)
	{
		TRX_REG_SET_N(0x80, 0x00, 61, NA5TR1_Tx_80MHz);
		TRX_REG_SET(TRX_REG_CsqMemAddrInit, NTRX_CsqMemAddrInit_MASK, TRUE<<NA_CsqMemAddrInit_B);

		TRX_Set_Index_Reg(0x21);
		TRX_REG_SET_N(0x80, 0x00, 61, &(NA5TR1_Tx_80MHz[61]));

		TRX_REG_SET(TRX_REG_CsqMemAddrInit, NTRX_CsqMemAddrInit_MASK, TRUE<<NA_CsqMemAddrInit_B);
		TRX_Set_Index_Reg(0x22);

		TRX_REG_SET_N(0x80, 0x00, 61, &(NA5TR1_Tx_80MHz[122]));
		TRX_Set_Index_Reg(0x00);
	}
	else if(bw == TRX_BANDWIDTH_22MHz)
	{
		TRX_REG_SET_N(0x80, 0x00, 8, NA5TR1_Tx_80MHz);
		TRX_REG_SET(TRX_REG_CsqMemAddrInit, NTRX_CsqMemAddrInit_MASK, TRUE<<NA_CsqMemAddrInit_B);

		TRX_Set_Index_Reg(0x21);
		TRX_REG_SET_N(0x80, 0x00, 8, &(NA5TR1_Tx_80MHz[8]));

		TRX_REG_SET(TRX_REG_CsqMemAddrInit, NTRX_CsqMemAddrInit_MASK, TRUE<<NA_CsqMemAddrInit_B);
		TRX_Set_Index_Reg(0x22);

		TRX_REG_SET_N(0x80, 0x00, 8, &(NA5TR1_Tx_80MHz[16]));
		TRX_Set_Index_Reg(0x00);
	}
	else
		return;

	return;
}
	

void TRX_CAL(void)
{
	uint8_t data = 0x10;
	int16_t cval = 6;		// Capacitor value
	int16_t clock;
	int16_t ctr;
	int16_t targetMin = 152;
	int16_t targetMax = 174;
	
	TRX_REG_SET(0x27, 0x00, data);
	vTaskDelay(1 / portTICK_RATE_MS );
	
	while(cval < 16 && cval >= 0)
	{
		data = 0x10 + cval;
		TRX_REG_SET(0x27, 0x00, data);
		
		clock = 0;
		for(ctr=0; ctr<4; ctr++)
		{
			data = 0x30 + cval;
			TRX_REG_SET(0x27, 0x00, data);
			vTaskDelay(3 / portTICK_RATE_MS );
			
			TRX_REG_GET(0x27, 1, &data);
			clock += data;
		}
		
		if(clock >= targetMax)
			cval--;
		else if(clock <= targetMin)
			cval++;
		else
			break;
	}
	
	data = 0x00;
	TRX_REG_SET(0x27, 0x00, data);

	return;
}





void TRXRxLoCalibration(void)
{
	uint8_t data[3];

	if (trxState != TRX_STATE_IDLE)
	{
//		ntrxCal |= TxCAL;
		return;
	}
	TRX_REG_SET(TRX_REG_RxCmdStop, NTRX_RxCmdStop_MASK, TRUE << NTRX_RxCmdStop_MASK);
	TRX_REG_SET(TRX_REG_RxIntsReset, NTRX_RxIntsReset_MASK, 0xff);
	TRX_REG_SET(TRX_REG_RxBufferCmd, NTRX_RxBufferCmd_MASK, 0x0c);
//	rxIrqStatus = 0;

	TRX_REG_SET(TRX_REG_EnableLO, NTRX_EnableLO_MASK, TRUE);
	
	TRX_REG_SET(TRX_REG_EnableLOdiv10, NTRX_EnableLOdiv10_MASK, TRUE<<NA_EnableLOdiv10_B);

//	hwdelay (4000);
	vTaskDelay(4 / portTICK_RATE_MS );
	
	TRX_REG_SET(TRX_REG_UseLoRxCaps, NTRX_UseLoRxCaps_MASK, TRUE<<NA_UseLoRxCaps_B);

	data[0] = 0x5a;
	data[1] = 0x68;
	TRX_REG_SET_N(TRX_REG_LoTargetValue, NTRX_LoTargetValue_MASK, 0x00, data);	

//	hwdelay (8000);
//	hwdelay (8000);
//	hwdelay (8000);
	vTaskDelay(24 / portTICK_RATE_MS );

	TRX_REG_GET(0x16, 3, data);

	TRX_REG_SET(TRX_REG_UseLoRxCaps, NTRX_UseLoRxCaps_MASK, FALSE<<NA_UseLoRxCaps_B);
	TRX_REG_SET(TRX_REG_EnableLOdiv10, NTRX_EnableLOdiv10_MASK, FALSE<<NA_EnableLOdiv10_B);
	TRX_REG_SET(TRX_REG_EnableLO, NTRX_EnableLO_MASK, FALSE);

	TRX_REG_SET(TRX_REG_RxCmdStart, NTRX_RxCmdStart_MASK, TRUE<<NA_RxCmdStart_B);
//	ntrxCal &= ~RxCAL;
}

void TRXTxLoCalibration(void)
{
	uint8_t data[3];

	if (trxState != TRX_STATE_IDLE)
	{
//		ntrxCal |= TxCAL;
		return;
	}

	TRX_REG_SET(TRX_REG_RxCmdStop, NTRX_RxCmdStop_MASK, TRUE << NTRX_RxCmdStop_MASK);
	TRX_REG_SET(TRX_REG_RxIntsReset, NTRX_RxIntsReset_MASK, 0xff);
	TRX_REG_SET(TRX_REG_RxBufferCmd, NTRX_RxBufferCmd_MASK, 0x03);
//	rxIrqStatus = 0;

	TRX_REG_SET(TRX_REG_EnableLO, NTRX_EnableLO_MASK, TRUE);

	TRX_REG_SET(TRX_REG_EnableLOdiv10, NTRX_EnableLOdiv10_MASK, TRUE<<NA_EnableLOdiv10_B);

	vTaskDelay(4 / portTICK_RATE_MS );

	TRX_REG_SET(TRX_REG_UseLoRxCaps, NTRX_UseLoRxCaps_MASK, TRUE<<NA_UseLoRxCaps_B);

	data[0] = 0x5a;
	data[1] = 0x68;
	TRX_REG_SET_N(TRX_REG_LoTargetValue, NTRX_LoTargetValue_MASK, 2, data);	

	vTaskDelay(24 / portTICK_RATE_MS );
	TRX_REG_GET(0x16, 3, data);

	TRX_REG_SET(TRX_REG_EnableLOdiv10, NTRX_EnableLOdiv10_MASK, FALSE<<NA_EnableLOdiv10_B);
	TRX_REG_SET(TRX_REG_EnableLO, NTRX_EnableLO_MASK, FALSE);
	TRX_REG_SET(TRX_REG_RxCmdStart, NTRX_RxCmdStart_MASK, TRUE<<NA_RxCmdStart_B);

//	ntrxCal &= ~TxCAL;
}


uint8_t NTRXGetRxCRC2mode(void)
{	
	return((TRXShadowReg[NA_RxCrc2Mode_O] & (1<< NA_RxCrc2Mode_B)) >> NA_RxCrc2Mode_B);
}


uint8_t NTRXRestart (void)
{
	uint8_t rxIrq = 0xFF;
	uint8_t txIrq = TX_IRQ_MASK;

	/* 
   * clear any pending interrupts in the nanoNET TRX chip 
   */
	TRX_REG_SET(TRX_REG_RxIntsReset, NTRX_TxIntsReset_MASK, rxIrq);
	TRX_REG_SET(TRX_REG_RxIntsReset, NTRX_RxIntsReset_MASK, rxIrq);

	/* 
   * allow some events in the receiver to trigger an interrupt 
   */
  if (NTRXGetRxCRC2mode() == TRUE)
	{
		rxIrq =  RxEND;
	}
	else
	{
		rxIrq = (RxEND | RxOVERFLOW);
	}

	debug("rxIrq: ");
	debug_hex(rxIrq);
	debug("\r\n");

	/* 
   * select specific events in the nanoNET TRX chip to generate interrupts 
   */
	TRX_REG_SET(TRX_REG_RxIntsEn, NTRX_RxIntsEn_MASK, rxIrq);
	TRX_REG_SET(TRX_REG_TxIntsEn, NTRX_RxIntsEn_MASK, rxIrq);
{
		uint8_t buf;
//		TRX_OPEN();
		TRX_REG_GET(0x0f, 1, &buf);
//		TRX_CLOSE();
		debug("NTRXRestart 0x0f: ");
		debug_hex(buf);
		debug("\r\n");
}
//	txIrqStatus = 0;
//	rxIrqStatus = 0;
	txIrq = 0;
	rxIrq = 0;

	/* 
   * allow the the nanoNET TRX chip to drive the interrupt line 
   */
	TRX_REG_SET(TRX_REG_RxIrqEnable, NTRX_RxIrqEnable_MASK, TRUE<<NA_RxIrqEnable_B);
	TRX_REG_SET(TRX_REG_TxIrqEnable, NTRX_TxIrqEnable_MASK, TRUE<<NA_TxIrqEnable_B);

	/* 
   * start the receiver of the TRX chip 
   */
	TRX_REG_SET(TRX_REG_RxCmdStart, NTRX_RxCmdStart_MASK, TRUE<<NA_RxCmdStart_B);

	trxRun      = TRUE;
 	trxState	 = TRX_STATE_IDLE;
//	ntrxTxPoll	 = FALSE;
	/* 
   * select message type date for transmitted messages 
   */
	TRX_REG_SET(TRX_REG_TxPacketType, NTRX_TxPacketType_MASK, NA_TypeCodeData_VC_C);
	/* 
   * enable retransmissions 
   */
	TRX_REG_SET(TRX_REG_TxArq, NTRX_TxArq_MASK, TRUE<<NA_TxArq_B);
	return TRUE;
}


/**
 * Set a specific transmission mode.
 *
 * \param fdma Channel bandwidth
 * \param symbolw Symbol width (duration)
 * \param symbolr Symbol rate
 *
 */
void TRX_Setup_Mode(uint8_t fdma, uint8_t symbolw, uint8_t symbolr)
{
	uint8_t value_new[5];

	TRX_Set_Index_Reg(0x00);

	TRX_REG_SET(TRX_REG_RxCmdStop, NTRX_RxCmdStop_MASK, TRUE << NTRX_RxCmdStop_MASK);
	TRX_REG_SET(TRX_REG_RxIntsReset, NTRX_RxIntsReset_MASK, 0xff);
	TRX_REG_SET(TRX_REG_RxBufferCmd, NTRX_RxBufferCmd_MASK, 0x0c);
	TRX_REG_SET(TRX_REG_ResetBbRadioCtrl, NTRX_ResetBbRadioCtrl_MASK, TRUE << 3);
	TRX_REG_SET(TRX_REG_ResetBbRadioCtrl, NTRX_ResetBbRadioCtrl_MASK, FALSE << 3);
	TRX_REG_SET(TRX_REG_EnableBbClock, NTRX_EnableBbClock_MASK, FALSE << 1);
	TRX_REG_SET(TRX_REG_ResetBbClockGate, NTRX_ResetBbClockGate_MASK, TRUE << 1);
	TRX_REG_SET(TRX_REG_EnableBbCrystal, NTRX_EnableBbCrystal_MASK, TRUE);
	TRX_REG_SET(TRX_REG_ResetBbClockGate, NTRX_ResetBbClockGate_MASK, TRUE << 1);
	TRX_REG_SET(TRX_REG_EnableBbCrystal, NTRX_EnableBbCrystal_MASK, TRUE);

	vTaskDelay(30 / portTICK_RATE_MS );

	TRX_REG_SET(TRX_REG_ResetBbClockGate, NTRX_ResetBbClockGate_MASK, FALSE << 1);
	TRX_REG_SET(TRX_REG_EnableBbClock, NTRX_EnableBbClock_MASK, TRUE << 1);
	TRX_REG_SET(TRX_REG_ResetBbRadioCtrl, NTRX_ResetBbRadioCtrl_MASK, TRUE << 3);
	TRX_REG_SET(TRX_REG_ResetBbRadioCtrl, NTRX_ResetBbRadioCtrl_MASK, FALSE << 3);
	
	NTRX_SET_AGC_VALUES(fdma, symbolw, symbolr);


	TRX_REG_SET(TRX_REG_RxArqMode, NTRX_RxArqMode_MASK, 0x02<<NA_RxArqMode_LSB);
	TRX_REG_SET(TRX_REG_RxCrc2Mode, NTRX_RxCrc2Mode_MASK, TRUE<<NA_RxCrc2Mode_B);
	TRX_REG_SET(TRX_REG_RfRxCompValueI, NTRX_RfRxCompValueI_MASK, 15);
	TRX_REG_SET(TRX_REG_RfRxCompValueI, NTRX_RfRxCompValueI_MASK, 15);

	TRX_SET_RX_IQ(fdma, symbolw);

	TRX_REG_SET(TRX_REG_DeviceSelect, NTRX_DeviceSelect_MASK, 0x00<<NA_DeviceSelect_LSB);
	TRX_REG_SET(TRX_REG_HoldAgcInFrameSync, NTRX_HoldAgcInFrameSync_MASK, TRUE<<NA_HoldAgcInFrameSync_B);
	TRX_REG_SET(TRX_REG_HoldAgcInBitSync, NTRX_HoldAgcInBitSync_MASK, 24);
	TRX_REG_SET(TRX_REG_UseAlternativeAgc, NTRX_UseAlternativeAgc_MASK, TRUE<<NA_UseAlternativeAgc_B);
	TRX_REG_SET(TRX_REG_ChirpFilterCaps, NTRX_ChirpFilterCaps_MASK, 0x00);
	TRX_REG_SET(TRX_REG_RfRxCompValueI, NTRX_RfRxCompValueI_MASK, 15);
	TRX_REG_SET(TRX_REG_RfRxCompValueQ, NTRX_RfRxCompValueQ_MASK, 15);
	TRX_REG_SET(TRX_REG_TestModes, NTRX_TestModes_MASK, 0x05);
	TRX_REG_SET(TRX_REG_RfTestSelect, NTRX_RfTestSelect_MASK, 0x00);
	TRX_REG_SET(TRX_REG_GateAdjFramesyncEn, NTRX_GateAdjFramesyncEn_MASK, FALSE<<NA_GateAdjFramesyncEn_B);
	TRX_REG_SET(TRX_REG_GateAdjBitsyncEn, NTRX_GateAdjBitsyncEn_MASK, FALSE<<NA_GateAdjBitsyncEn_B);

	TRX_SET_THR(fdma, symbolw);

	TRX_REG_SET(TRX_REG_DeviceSelect, NTRX_DeviceSelect_MASK, 0x00<<NA_DeviceSelect_LSB);
	TRX_REG_SET(TRX_REG_AgcValue, NTRX_AgcValue_MASK, 16);
	vTaskDelay(2 / portTICK_RATE_MS );

	/* Tx Part */
	TRX_REG_SET(TRX_REG_TxArq, NTRX_TxArq_MASK, TRUE<<NA_TxArq_B);
	TRX_REG_SET(TRX_REG_TxArqMax, NTRX_TxArqMax_MASK, 0x0a<<NA_TxArqMax_LSB);
	TRX_REG_SET(TRX_REG_TxScrambEn, NTRX_TxScrambEn_MASK, TRUE<<NA_TxScrambEn_B);
	TRX_REG_SET(TRX_REG_EnableExtPA, NTRX_EnableExtPA_MASK, TRUE<<NA_EnableExtPA_B);
	TRX_REG_SET(TRX_REG_RxAddrMode, NTRX_RxAddrMode_MASK, TRUE<<NA_RxAddrMode_B);
	TRX_REG_SET(TRX_REG_CsqUsePhaseShift, NTRX_CsqUsePhaseShift_MASK, FALSE<<NA_CsqUsePhaseShift_B);
	TRX_REG_SET(TRX_REG_EnableLO, NTRX_EnableLO_MASK, TRUE);
	TRX_REG_SET(TRX_REG_EnableLOdiv10, NTRX_EnableLOdiv10_MASK, TRUE<<NA_EnableLOdiv10_B);

	vTaskDelay(3 / portTICK_RATE_MS );

	value_new[0] = 0x5a;
	value_new[1] = 0x68;
	TRX_REG_SET_N(TRX_REG_LoTargetValue, 0x00, 2, value_new);

	vTaskDelay(14 / portTICK_RATE_MS ); 

	TRX_REG_SET(TRX_REG_EnableCsqClock, NTRX_EnableCsqClock_MASK, TRUE<<NA_EnableCsqClock_B);
	TRX_REG_SET(TRX_REG_CsqUseRam, NTRX_CsqUseRam_MASK, TRUE<<NA_CsqUseRam_B);
	TRX_REG_SET(TRX_REG_CsqAsyMode, NTRX_CsqAsyMode_MASK, FALSE<<NA_CsqAsyMode_B);
	
	TRX_SET_TX_IQ(fdma, symbolw);

	TRX_CAL();

	TRX_REG_SET(TRX_REG_EnableLO, NTRX_EnableLO_MASK, FALSE);
	TRX_REG_SET(TRX_REG_EnableLOdiv10, NTRX_EnableLOdiv10_MASK, FALSE<<NA_EnableLOdiv10_B);
	TRX_REG_SET(TRX_REG_TxAddrSlct, NTRX_TxAddrSlct_MASK, 0<<NA_TxAddrSlct_B);
	
	RxCRC2Mode=TRUE;



	TRXRxLoCalibration();
	TRXTxLoCalibration();


	NTRXRestart ();
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
	uint8_t bwidth = TRX_BANDWIDTH_80MHz;
	uint8_t symbolwidth = TRX_SD_1us;
	uint8_t symbolrate = TRX_1M_S;
	uint8_t addr[6] = { 0x66, 0x66, 0x66, 0x66, 0x66, 0x66 };

	P2DIR |= 0x08;
		
	RF_RX_LED_OFF();
	RF_TX_LED_OFF();

#ifdef HAVE_POWERSAVE
	if (rf_ph == 0)
	{
		rf_ph = power_alloc();
	}
#endif

	rx_flags = tx_flags = 0;
	if (TRX_OPEN() == pdTRUE)
	{		
		TRX_SELECT();

// Setup the SPI bitorder on the NA5TR1
		TRX_SPI_SETUP();

		if(!rf_check_TRX_version())
		{
#ifdef TRX_DEBUG
			debug("NA5TR1 Version/Revision check failed.\r\n");
#endif
			TRX_UNSELECT();
			TRX_CLOSE();
			return(pdFALSE);
		}

		gpio2_irq_allocate(3, rf_isr, 1);

// Setup the NA5TR1 chip
		TRX_Setup_Mode(bwidth, symbolwidth, symbolrate);

/*	Set own address */
		TRX_REG_SET_N(0x80, 0x00, 6, addr);
/*	Set transparent mode */
		TRX_REG_SET(TRX_REG_TxRxMode, NTRX_TxRxMode_MASK, TRUE<<NA_TxRxMode_B);
		
#ifdef TRX_DEBUG				
		debug("RF_NTRX: Init complete.\r\n");
#endif
		
		retval = pdTRUE;
		
		tx_flags = 0;
{
		uint8_t buf;
		TRX_REG_GET(0x0f, 1, &buf);
		debug("0x0f: ");
		debug_hex(buf);
		debug("\r\n");
}

		TRX_UNSELECT();

		TRX_CLOSE();
		
		return retval;
	}
	
#ifdef TRX_DEBUG				
	debug("RF_NTRX: Bus allocation failed.\r\n");
#endif

	TRX_UNSELECT();
	TRX_CLOSE();
	return pdFALSE;
}




/**
	* Set address decoder parameters
	*
	*	\param address address for decoder
	*/
void rf_set_address(sockaddr_t *address)
{
#ifdef TRX_DEBUG
	debug("doing nothing in rf_set_address()\r\n");
#endif
}


/**
 * Set address decoder on/off.
 *
 * \param mode RF_DECODER_NONE/RF_DECODER_ON/RF_DECODER_COORDINATOR 
 * \return pdTRUE operation successful
 */
portCHAR rf_address_decoder_mode(uint8_t mode)
{
#ifdef TRX_DEBUG
	debug("doing nothing in rf_address_decoder_mode()\r\n");
#endif
	return pdTRUE;
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
#ifdef TRX_DEBUG
		debug("Doing nothing in rf_analyze_rssi()\r\n");
#endif
	return 0x42;
}

/**
 * Clear channel assesment check.
 *
 * \return pdTRUE	CCA clear
 * \return pdFALSE	CCA reserved
 */
portCHAR rf_cca_check(uint8_t backoff_count, uint8_t slotted)
{
#ifdef TRX_DEBUG
		debug("Doing nothing in rf_cca_check()\r\n");
#endif
	return pdTRUE;
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
	uint8_t txLen[2];
	uint8_t rsvd, seq_n, flush;
	uint8_t len;

	debug("Doing nothing in rf_write()\r\n");
	
	len = (buffer->buf_end - buffer->buf_ptr);
	rsvd=seq_n=flush=0;

	TRX_Set_Index_Reg(3);

	TRX_REG_SET_N(NA_RamTxBuffer_O & 0xff, 0x00, (uint8_t)(len & 0xff), (uint8_t *)buffer->buf);


	TRX_Set_Index_Reg(0);

	/* copy the destination address to temp buffer */
	if(buffer->dst_sa.addr_type == ADDR_802_15_4_SHORT || buffer->dst_sa.addr_type == ADDR_PAN)
	{
		TRX_REG_SET_N(NA_RamTxDstAddr_O, 0x00, 2, (uint8_t *)&(buffer->dst_sa.address));
	}
	else if(buffer->dst_sa.addr_type == ADDR_802_15_4_LONG)
	{
		debug("Warning, truncating address to highest 48 bits due to HW limitations.\r\n");
		TRX_REG_SET_N(NA_RamTxDstAddr_O, 0x00, 6, (uint8_t *)&(buffer->dst_sa.address));
	}
	else
	{
		/* Unsupported address type */
		debug("Unsupported address type ");
		debug_hex(buffer->dst_sa.addr_type);
		debug("\r\n");
		stack_buffer_free(buffer);
		return(pdFALSE);
	}

	/* merge the three bits into the temp buffer */
	txLen[0] = (uint8_t)len;
	txLen[1] = (len & 0x1F00) >> 8;
/* For now all the fragmentation, logical channel and sequence are zeros... */
	txLen[1] |= (rsvd == 1)  ? 0x20 : 0;
  txLen[1] |= (seq_n == 1) ? 0x40 : 0;
	txLen[1] |= (flush == 1) ? 0x80 : 0;

	TRX_REG_SET_N(NA_RamTxLength_O, 0x00, 2, txLen);

/* Use both Chirp detection and RSSI carrier sense */
	TRX_REG_SET(TRX_REG_TxPhCarrSenseMode, NTRX_TxPhCarrSenseMode_MASK, 0x06);
	
	TRX_REG_SET(NA_TxBufferCmd_O, 0x00, NTRX_TX_BUFF0 | NTRX_TX_BUFF1 | NTRX_TX_START);

	return pdTRUE;
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
 
portCHAR rf_write_no_cca(buffer_t *buffer)
{
	uint8_t txLen[2];
	uint8_t rsvd, seq_n, flush;
	uint8_t len;
	
	TRX_OPEN();
	
	len = (buffer->buf_end - buffer->buf_ptr);
	rsvd=seq_n=flush=0;

	debug("Doing nothing in rf_write_no_cca()\r\n");

	TRX_Set_Index_Reg(3);
debug("1\r\n");

	TRX_REG_SET_N(NA_RamTxBuffer_O & 0xff, 0x00, (uint8_t)(len & 0xff), (uint8_t *)buffer->buf);
debug("2\r\n");


	TRX_Set_Index_Reg(0);
debug("3\r\n");

	/* copy the destination address to temp buffer */
	if(buffer->dst_sa.addr_type == ADDR_802_15_4_SHORT || buffer->dst_sa.addr_type == ADDR_PAN)
	{
debug("4\r\n");
		TRX_REG_SET_N(NA_RamTxDstAddr_O, 0x00, 2, (uint8_t *)&(buffer->dst_sa.address));
	}
	else if(buffer->dst_sa.addr_type == ADDR_802_15_4_LONG)
	{
debug("5\r\n");
		debug("Warning, truncating address to highest 48 bits due to HW limitations.\r\n");
		TRX_REG_SET_N(NA_RamTxDstAddr_O, 0x00, 6, (uint8_t *)&(buffer->dst_sa.address));
	}
	else
	{
debug("6\r\n");
		/* Unsupported address type */
		debug("Unsupported address type ");
		debug_hex(buffer->dst_sa.addr_type);
		debug("\r\n");
		stack_buffer_free(buffer);
		TRX_CLOSE();
		return(pdFALSE);
	}
debug("7\r\n");

	/* merge the three bits into the temp buffer */
	txLen[0] = (uint8_t)len;
	txLen[1] = (len & 0x1F00) >> 8;
/* For now all the fragmentation, logical channel and sequence are zeros... */
	txLen[1] |= (rsvd == 1)  ? 0x20 : 0;
  txLen[1] |= (seq_n == 1) ? 0x40 : 0;
	txLen[1] |= (flush == 1) ? 0x80 : 0;
debug("8\r\n");

	TRX_REG_SET_N(NA_RamTxLength_O, 0x00, 2, txLen);
debug("9\r\n");

/* Use both Chirp detection and RSSI carrier sense */
	TRX_REG_SET(TRX_REG_TxPhCarrSenseMode, NTRX_TxPhCarrSenseMode_MASK, 0x00);
debug("10\r\n");

	TRX_REG_SET(NA_TxBufferCmd_O, 0x00, NTRX_TX_BUFF0 | NTRX_TX_BUFF1 | NTRX_TX_START);
debug("11\r\n");

	TRX_CLOSE();
	return pdTRUE;
}



uint8_t rf_state(void)
{
#ifdef TRX_DEBUG
		debug("Doing nothing in rf_state()\r\n");
#endif
	return(0);
}


void rf_rxflush(void)
{
#ifdef TRX_DEBUG
	debug("Doing nothing in rf_rxflush()\r\n");
#endif
	return;
}

extern uint8_t bus_serial[4];

portCHAR rf_mac_get(sockaddr_t *address)
{
#ifdef TRX_DEBUG
	debug("Doing nothing in rf_mac_get()\r\n");
#endif
	return pdFALSE;
}

void rf_rx_callback( void *param );

/**
	* RF receive callback
	* \param param not used
	*/	
void rf_rx_callback( void *param )
{
#ifdef TRX_DEBUG
	debug("Doing nothing in rf_rx_callback()\r\n");
#endif
	buffer_t *rf_buf;

  uint8_t status;
	uint8_t *rxPayload;    
	uint8_t LCh, SeqN, FragC;
	uint8_t len;
	uint8_t value;
	uint8_t source[8];
	uint8_t dest[8];
	uint8_t reg[2];

	rf_buf = stack_buffer_get(2);

	if(rf_buf != 0)
	{
		TRX_REG_GET(NA_RxCrc2Stat_O, 1, &status);

		/* check if data is valid */
		if ((status & (1 << NA_RxCrc2Stat_B)) == (1 << NA_RxCrc2Stat_B))
		{
			TRX_Set_Index_Reg(0);
			TRX_REG_GET(NA_RamRxDstAddr_O, 6, dest);
			TRX_REG_GET(NA_RamRxSrcAddr_O, 6, source);

			dest[6]=dest[7]=source[6]=source[7]= 0xff;

			memcpy(rf_buf->dst_sa.address, dest, 8);
			memcpy(rf_buf->src_sa.address, source, 8);

			rf_buf->dst_sa.addr_type = ADDR_802_15_4_LONG;
			rf_buf->src_sa.addr_type = ADDR_802_15_4_LONG;

			TRX_REG_GET(NA_RamRxLength_O, 2, reg);
			/* read length plus additionl bits */
			len	 = (reg[0]  | (reg[1] & 0x1F) << 8);
			if (len > 128)
			{
				len = 0;
			}
			else
			{
				TRX_Set_Index_Reg(2);
				TRX_REG_GET((NA_RamRxBuffer_O & 0xFF), len, rf_buf->buf);
			}
		}
		value = (uint8_t)(TRXShadowReg[NA_RxCmdStart_O] 	
								| (1 << NA_RxCmdStart_B) 
								| (1 << NA_RxBufferCmd_LSB)
								| (1 << NA_RxBufferCmd_MSB));


		TRX_REG_SET(NA_RxCmdStart_O, 1, value);

		rf_buf->options.type = BUFFER_DATA;
		rf_buf->dir = BUFFER_UP;	
		stack_buffer_push(rf_buf);
		rf_buf = 0;
	}
	else
	{
		debug("Could not allocate buffers for rx!\r\n");
	}
}

void rf_isr(void)
{
	uint8_t buf;
	
//	TRX_OPEN();

//	TRX_REG_GET(0x0f, 1, &buf);

//	TRX_CLOSE();

	RF_RX_LED_OFF();
	RF_RX_LED_ON();
	RF_TX_LED_OFF();
	RF_TX_LED_ON();
	RF_RX_LED_OFF();
	RF_RX_LED_ON();
	RF_TX_LED_OFF();
	RF_TX_LED_ON();
	RF_RX_LED_OFF();
	RF_RX_LED_ON();
	RF_TX_LED_OFF();
	RF_TX_LED_ON();
	RF_RX_LED_OFF();
	RF_TX_LED_OFF();
	P2IFG &= ~0x08;
	
	irq_wanker = 1;
	irq_buf = buf;
}

void rf_timer_callback(void);
void rf_timer_callback(void)
{
}
