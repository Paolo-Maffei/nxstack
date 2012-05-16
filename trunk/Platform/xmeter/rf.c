/**
 *
 * \file rf.c
 * \brief micro RF driver.
 *
 *  Micro: RF control functions.
 *   
 *	
 */

#include "stm32f10x.h"
 
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

#include "cc2520hal.h"
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
#define RF_DEFAULT_CHANNEL 26
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

typedef struct {
    uint8_t reg;
    uint8_t val;
} cc2520_reg_t;

// Recommended register settings which differ from the data sheet
static cc2520_reg_t cc2520_reg[]= {
    // Tuning settings
    { CC2520_TXPOWER,     0xF7 },       // Max TX output power
    { CC2520_CCACTRL0,    0xF8 },       // CCA treshold -80dBm
    // Recommended RX settings
    { CC2520_MDMCTRL0,    0x85 },
    { CC2520_MDMCTRL1,    0x14 },
    { CC2520_RXCTRL,      0x3F },
    { CC2520_FSCTRL,      0x5A },
    { CC2520_FSCAL1,      0x03 },

    { CC2520_AGCCTRL1,    0x11 },
    { CC2520_ADCTEST0,    0x10 },
    { CC2520_ADCTEST1,    0x0E },
    { CC2520_ADCTEST2,    0x03 },
    // Configuration for applications using halRfInit()
    { CC2520_FRMCTRL0,    0x60 },               // no Auto-ack
    { CC2520_EXTCLOCK,    0x00 },

    { CC2520_GPIOCTRL1,   CC2520_GPIO_FIFO },
    { CC2520_GPIOCTRL2,   CC2520_GPIO_FIFOP },

    { CC2520_GPIOCTRL3,   CC2520_GPIO_CCA },
    { CC2520_GPIOCTRL4,   CC2520_GPIO_SFD },
    // Terminate array
    { 0,                  0x00 }
};
/*#define CC2420_RSSI_VALID 0x80
#define CC2420_TX_ACTIVE 0x40*/

/*RF SPI mode open and close*/
#define CC2420_OPEN(x) 		bus_select()
#define CC2420_CLOSE(x) 	bus_free()

/*These macros control RF select signal and assume correct bus state*/
#define CC2420_SELECT(x)   	bus_spi_cs_low()
#define CC2420_UNSELECT(x) 	bus_spi_cs_high();

/*I/O init and status readout macros*/
#define CC2420_INIT(x) 		//function move to gpio_init()

#define CC2420_FIFO(x) 		GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) 
#define CC2420_FIFOP(x) 	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)
#define CC2420_CCA(x) 		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define CC2420_SFD(x) 		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)

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
	freq += 11; /*correct channel range*/
	
	CC2520_REGWR8(CC2520_FREQCTRL,freq);
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
	/*
	uint16_t power;
	
	if (new_power > 100) return -1;
	
	power = 31 * new_power;
	power /= 100;
	power += 0xA0E0;	
	*/
	/* Set transmitter max power */
	CC2520_REGWR8(CC2520_TXPOWER,0xF7);
	
	return new_power;
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
	if (status & CC2520_STB_XOSC_STABLE_BV)
	{
		debug(" OSC");
	}
	
	if (status & CC2520_STB_RSSI_VALID_BV)
	{
		debug(" RSSI_VALID");
	}

	if (status & CC2520_STB_TX_ACTIVE_BV)
	{
		debug(" TX_ACTIVE");
	}

	if (status & CC2520_STB_RX_ACTIVE_BV)
	{
		debug(" RX_ACTIVE");
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
	CC2520_INS_STROBE(CC2520_INS_SRXON);
	
	//disable rx interrupt
	gpio_irq_disable(1);
	rx_flags &= ~ACTIVE;
	//enable rx interrupt
	if (irq_state) 
	    gpio_irq_enable(1);
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
	
	CC2520_INS_STROBE(CC2520_INS_SRFOFF);
	CC2520_INS_STROBE(CC2520_INS_SFLUSHRX);
	timer_rf_stop();
	rx_flags = 0;
	//disable rx interrupt
	gpio_irq_disable(1);
}

/**
 * Initialize RF for tx mode.
 *
 *
 */
void tx_enable(void)
{
	//disable rx interrupt
	gpio_irq_disable(1);
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
		    CC2520_INS_STROBE(CC2520_INS_SACKPEND);
		}
		else
		{
		    CC2520_INS_STROBE(CC2520_INS_SACK);
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
  		CC2520_SRES();
		
		debug("RF_CC2420: SPI init");
		j=0;
		status = 0;
		while( !(status == CC2520_STB_XOSC_STABLE_BV) && (j++ < 3) )
		{	
			debug(".");	
			/* Reset CC2420 using SPI*/
  			CC2520_SRES();
	
			pause(2);

			/* Set oscillator on */
			CC2520_INS_STROBE(CC2520_INS_SXOSCON);	
			pause(2);

			status = CC2520_INS_STROBE(CC2520_INS_SNOP);
	
			pause(2);
	 		/* Wait until oscillator is stable */
			status = 0;
			i = 0;
 			do 
			{
				pause(1);
				status = CC2520_INS_STROBE(CC2520_INS_SNOP);
				i++;
 			} while (!(status == CC2520_STB_XOSC_STABLE_BV) && (i<20));

		}
		if (i >= 20)
		{
			CC2420_STAT(status);
			debug("\r\nRF_CC2420: Init failed.\r\n");
			CC2420_CLOSE();
			return pdFALSE;
		}

		retval = pdTRUE;
		cc2520_reg_t* p = cc2520_reg;
		    // Write non-default register values
		while (p->reg!=0) {
		    CC2520_MEMWR8(p->reg,p->val);
			p++;
		}
		
		/* get ID for MAC */
		rf_manfid = CC2520_MEMRD8(CC2520_CHIPID);
		rf_manfid <<= 8;		
		rf_manfid += (uint8_t) CC2520_MEMRD8(CC2520_VERSION);

		CC2420_CHANNEL_SET(RF_DEFAULT_CHANNEL);

		/* Set transmitter power */
		CC2420_POWER_SET(RF_DEFAULT_POWER);


		CC2420_UNSELECT();  	/*CS up*/
	
		gpio1_irq_allocate(1, rf_isr, 0);
				
		debug("RF_CC2420: Init complete.\r\n");		
		retval = pdTRUE;
		
		CC2520_INS_STROBE(CC2520_INS_SFLUSHTX);
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
