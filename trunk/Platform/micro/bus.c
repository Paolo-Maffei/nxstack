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
 * \file bus.c
 * \brief micro.bus controls.
 *
 *  Micro bus: mode control and support functions.
 *  General support functions and hardware initialization.
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

#include "debug.h"
#include "bus.h"

#include "1wire.h"
#ifdef HAVE_POWERSAVE
#include "powersave.h"
#endif

#ifndef configSMCLK_HZ
#define SPI_115K  (configCPU_CLOCK_HZ / 115200)
#define SPI_1M  (configCPU_CLOCK_HZ / 1000000)
#define SPI_4M  (configCPU_CLOCK_HZ / 4000000)
#else
#define SPI_115K  (configSMCLK_HZ / 115200)
#define SPI_1M  (configSMCLK_HZ / 1000000)
#define SPI_4M  (configSMCLK_HZ / 4000000)
#endif

#ifndef BUS_UART_BUFLEN
#define BUS_UART_BUFLEN 16
#endif

#ifndef portACLK_FREQUENCY_HZ
#define portACLK_FREQUENCY_HZ ( ( unsigned portLONG ) 32768 )
#endif

void bus_spi_init(bus_spi_flags flags);

void bus_par_init(void);

void bus_i2c_init(bus_i2c_flags flags);

void bus_uart_init(bus_uart_flags flags);

void bus_1wire_init(void);

/**
 * Global bus lock.
 *
 */
 
xSemaphoreHandle bus_lock = NULL;

#ifdef HAVE_POWERSAVE
/**
 * Powersaving mode control.
 *
 */
xPowerHandle bus_ph = 0;
#endif

uint8_t bus_serial[4] = { 0x01, 0x02, 0x03, 0x04 };

/**
 * Initialize bus and MCU clock. 
 * First function to call.
 *
 *
 * \return pdTRUE
 * \return pdFALSE	semaphore creation failed
 */
portCHAR bus_init(void)
{
	WDTCTL = WDTPW + WDTHOLD;

	P1SEL = 0x00;
	P2SEL = 0x00;
	P3SEL = 0x00;
	P4SEL = 0x00;
	P5SEL = 0x00;
		
	BCSCTL1 = 0x80; /* XT2OFF | LFXT1=low | DIVA=00 -> 1 |
									   XT5V=0 | RSEL=000 */
	BCSCTL2 = 0x88; /* SELMx=10 -> XT2    | DIVMx=00 -> 1 | 
									   SELS=1 -> MCLK | DIVS=00 -> 1 | DCOR=0 */
	DCOCTL = 0x00;	/* DCO is not used */
	
#ifdef HAVE_1WIRE_PROGRAM
	P5OUT &= ~0x09; /*Enable and pulse off*/
	P5DIR |= 0x09;
#endif

	P4DIR = 0;	/*parport input mode*/
	P3DIR &= ~0x3F; /*bus pins input*/
	P2DIR |= 0xF0;
	P2OUT |= 0xF0; /*module select none*/
	

	vSemaphoreCreateBinary( bus_lock );

	if( bus_lock != NULL )
	{
		b1w_reg devices[4];
		uint8_t n_devices = 0;
		uint8_t retry = 0;
		uint8_t device_id = 8;
		uint8_t i, j;

		while(!n_devices && (retry++ < 5))
		{
			uint8_t data[64];
			uint8_t string[] = "U100";
			
			P2OUT = (P2OUT & 0x0F);
			bus_1wire_init();
		
			n_devices =	bus_1wire_search(devices, 4);
			
			for (i=0; i<n_devices; i++)
			{	/*find U100 board address*/
				if (bus_1wire_read_memory(devices[i], 0, data, 64) == pdTRUE)
				{	/*scan data for string "U100"*/
					for(j=0; j< 64;j++)
					{
						if(memcmp(&data[j], string, 4) == 0)
						{
							device_id = i;
							i = n_devices;
							break;
						}
					}
				}
			}
		}
		P4DIR = 0;	/*parport input mode*/
		P3DIR &= ~0x3F; /*bus pins input*/
		P3SEL &= ~0x3F; /*turn module i/o off*/
		ME1 &= ~(UTXE0 | URXE0 | USPIE0); /* Modules off */
		P2OUT |= 0xF0; /*module select none*/
		
		if (n_devices)
		{
			memcpy(bus_serial, &(devices[device_id][3]), 4);
		}
		/*Quick RF off*/

		P2OUT |= 0xF0; /*module select none*/
		bus_spi_init(BUS_STE + BUS_PHASE_INVERT);
		
		P2OUT &= 0xE0; /*module select RF*/

		P3OUT |= 0x01;
		bus_spi_exchange((uint8_t) 0x10 & 0x3F);
		bus_spi_exchange((uint8_t) 0x00);
		bus_spi_exchange((uint8_t) 0x00);
		P3OUT &= ~0x01;

		P3OUT |= 0x01;
		bus_spi_exchange((uint8_t) 0x10 & 0x3F);
		bus_spi_exchange((uint8_t) 0xF8);
		bus_spi_exchange((uint8_t) 0x01);
		P3OUT &= ~0x01;

		P3OUT |= 0x01;
		bus_spi_exchange((uint8_t) 0x06);
		P3OUT &= ~0x01;

		P4DIR = 0;	/*parport input mode*/
		P3DIR &= ~0x3F; /*bus pins input*/
		P3SEL &= ~0x3F; /*turn module i/o off*/
		ME1 &= ~(UTXE0 | URXE0 | USPIE0); /* Modules off */
		P2OUT |= 0xF0; /*module select none*/
#ifdef HAVE_POWERSAVE
		power_init();
		if (bus_ph == 0)
		{
			bus_ph = power_alloc();
			power_set(POWER_LPM0 + POWER_XT1, bus_ph);
		}
#endif
		return pdTRUE;
	}
	return pdFALSE;
}

/**
 * Bus select.
 *
 * \param id module ID;
 * \param mode communication mode
 * \param flags mode specific flags
 *
 * \return pdTRUE
 * \return pdFALSE	bus reserved
 */
portCHAR bus_select(uint8_t id, bus_mode mode, uint8_t flags)
{
	if( xSemaphoreTake( bus_lock, ( portTickType ) 0 ) == pdTRUE )
	{	/*bus free, do select*/
		switch (mode)
		{
			case BUS_SPI:
#ifdef HAVE_POWERSAVE
				power_set(POWER_LPM3, bus_ph);
#endif
				bus_spi_init((bus_spi_flags)flags);
				break;
			
			case BUS_PARALLEL:
#ifdef HAVE_POWERSAVE
				power_set(POWER_LPM3, bus_ph);
#endif
				bus_par_init();
				break;
			
			case BUS_I2C:
#ifdef HAVE_POWERSAVE
				power_set(POWER_LPM3, bus_ph);
#endif
				bus_i2c_init((bus_i2c_flags)flags);
				break;
				
			case BUS_UART:
#ifdef HAVE_POWERSAVE
				power_set(POWER_LPM0 + POWER_XT1, bus_ph);
#endif
				bus_uart_init((bus_uart_flags)flags);
				break;
				
			case BUS_1WIRE:
#ifdef HAVE_POWERSAVE
				power_set(POWER_LPM0 + POWER_XT1, bus_ph);
#endif
				bus_1wire_init();
				id = 0x00;
				break;
				
			default:
				break;
		}
		pause_us(3);
		P2OUT = (P2OUT & 0x0F)| (id << 4);
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}	


/**
 * Free bus.
 *
 * \return pdTRUE
 */

portCHAR bus_free(void)
{
#ifdef HAVE_1WIRE_PROGRAM
	P5OUT &= ~0x09; /*Enable and pulse off*/
	P5DIR |= 0x09;
#endif
	P2OUT |= 0xF0; /*module select none*/
	P4DIR = 0;	/*parport input mode*/
	P3DIR &= ~0x3F; /*bus pins input*/
	P3SEL &= ~0x3F; /*turn module i/o off*/
	ME1 &= ~(UTXE0 | URXE0 | USPIE0); /* Modules off */
	
#ifdef HAVE_POWERSAVE
	power_set(POWER_LPM4, bus_ph);
#endif
	
  xSemaphoreGive( bus_lock ); /*free lock*/
	
	return pdTRUE;
}

/**
 * Bus SPI init.
 *
 * \param flags SPI mode flags
 *
 * \return pdTRUE
 * \return pdFALSE	bus reserved
 */
 
void bus_spi_init(bus_spi_flags flags)
{
}

/**
 * Bus SPI exchange.
 *
 * \param out byte to transmit
 *
 * \return byte from SPI
 */
uint8_t bus_spi_exchange(uint8_t out)
{
	U0TXBUF = out;
	while(!(IFG1 & URXIFG0));
	return U0RXBUF;	
}

/**
 * Bus parallel init.
 *
 */
void bus_par_init(void)
{
	P3SEL &= ~0x3F; /*GPIO*/
	P3OUT |= 0x0F;
	P3DIR |= 0x0F;	/*control pins out*/
	P3DIR &= ~0x30; /*UART pins in*/
	P4DIR = 0x00;		/*data bus in*/
}

/**
 * Bus I2C init.
 *
 *	not implemented yet.
 *
 * \param flags I2C mode flags
 *
 */
void bus_i2c_init(bus_i2c_flags flags)
{
	return;
}


/** Interrupt service routines. */
interrupt (UART0RX_VECTOR) bus_rxISR( void );
interrupt (UART0TX_VECTOR) bus_txISR( void );

/** The queues for UART  */
static xQueueHandle bus_rx = 0; 
static xQueueHandle bus_tx = 0; 

static volatile portCHAR bus_txempty;

/**
 * Bus UART init.
 *
 * \param flags UART mode flags
 *
 * \return SUCCESS
 * \return FAILURE	bus reserved
 */
 
void bus_uart_init(bus_uart_flags flags)
{
	uint32_t rate = 0;
	uint8_t mod = 0;
	uint8_t clock_sel = (SSEL0 | SSEL1);
	
	if (bus_rx == 0)
	{
		/* Create the queues used by the com test task. */
		bus_rx = xQueueCreate( BUS_UART_BUFLEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
		bus_tx = xQueueCreate( BUS_UART_BUFLEN, ( unsigned portBASE_TYPE ) sizeof( signed portCHAR ) );
	}

	switch (flags & 0x0F)
	{
		case BUS_UART_28800:
			if (!rate) rate = 28800;
		case BUS_UART_19200:
			if (!rate) rate = 19200;
		case BUS_UART_9600:
			if (!rate) rate = 9600;		
			clock_sel = (SSEL0); /*use ACLK */
			rate = portACLK_FREQUENCY_HZ / rate;
			break;
			
		case BUS_UART_921600:
			rate = 921600;
		case BUS_UART_230400:
			if (!rate) rate = 230400;
		case BUS_UART_115200:
			if (!rate) rate = 115200;
			clock_sel = (SSEL0 | SSEL1);
#ifndef configSMCLK_HZ
			rate = configCPU_CLOCK_HZ / rate;	/*use MCLK rate*/
#else
			rate = configSMCLK_HZ / rate;			/*use SMCLK rate*/
#endif
			break;
	}
	
	if (rate)
	{
		portENTER_CRITICAL();
		/* Reset UART. */
		U0CTL = SWRST;

		P3DIR |= BIT4;
 		P3SEL |= (BIT4|BIT5);
		P3DIR &= ~(BIT5|BIT2|BIT1);

		/* All other bits remain at zero for n, 8, 1 interrupt driven operation. */
		U0CTL |= CHAR;
		
		switch (flags & 0xF0)
		{
			default:
					break;
		}
		
		U0TCTL = clock_sel;

		U0BR0 = ( unsigned portCHAR ) ( rate & ( unsigned portLONG ) 0xff );
		rate >>= 8UL;
		U0BR1 = ( unsigned portCHAR ) ( rate & ( unsigned portLONG ) 0xff );

		U0MCTL = mod;

		/* Enable ports. */
		ME1 |= UTXE0 + URXE0;

		/* Set. */
		U0CTL &= ~SWRST;

		/* Nothing in the buffer yet. */
		bus_txempty = pdTRUE;

		/* Enable interrupts. */
		IE1 |= URXIE0 + UTXIE0;
		portEXIT_CRITICAL();

		return;
	}
	return;
}

/**
 * Bus UART send.
 *
 * \param byte data to be sent
 * \param blocktime time to block
 *
 * \return pdTRUE
 * \return pdFALSE	buffer full
 */ 
portCHAR bus_uart_put( uint8_t byte, portTickType blocktime )
{
	portCHAR retval = pdFALSE;
	
	portENTER_CRITICAL();
	
	if(bus_txempty == pdTRUE )
	{
		bus_txempty = pdFALSE;
		U0TXBUF = byte;
		retval = pdTRUE;
	}
	else
	{
		retval = xQueueSend(bus_tx, &byte, blocktime );

		if( ( bus_txempty == pdTRUE ) && ( retval == pdPASS ) )
		{
			/* Get back the character we just posted. */
			xQueueReceive(bus_tx, &byte, 0);
			bus_txempty = pdFALSE;
			U0TXBUF = byte;
			retval = pdTRUE;
		}
		else if (retval == pdPASS)
		{
			retval = pdTRUE;
		}
		else
		{
			retval = pdFALSE;
		}
	}

	portEXIT_CRITICAL();

	return retval;
}

/**
 * Bus UART read.
 *
 * \param byte pointer to data byte
 * \param blocktime time to block
 *
 * \return pdTRUE
 * \return pdFALSE	buffer empty
 */ 
 
portCHAR bus_uart_get( uint8_t *byte, portTickType blocktime )
{
	if( xQueueReceive( bus_rx, byte, blocktime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}

/**
 * Bus UART RX interrupt.
 */
interrupt (UART0RX_VECTOR) bus_rxISR( void )
{
	uint8_t byte;

	byte = U0RXBUF;

	if( xQueueSendFromISR( bus_rx, &byte, pdFALSE ) )
	{
		taskYIELD();
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}


/**
 * Bus UART Tx interrupt.
 */
interrupt (UART0TX_VECTOR) bus_txISR( void )
{
	uint8_t byte;
	portBASE_TYPE task;

	if( xQueueReceiveFromISR( bus_tx, &byte, &task ) == pdTRUE )
	{
		U0TXBUF = byte;
	}
	else
	{
		bus_txempty = pdTRUE;
	}
#ifdef HAVE_POWERSAVE
	power_interrupt_epilogue();
#endif
}

/**
 * Bus 1-wire mode init.
 *
 */
 
void bus_1wire_init(void)
{
	P3DIR &= ~0x30; /*UART pins inputs*/
	P3SEL &= ~0x30; /*UART pins GPIO*/
	
	P3SEL &= ~0x0F; /*STE, MISO,MOSI,UCLK = GPIO*/
	P3DIR |= 0x03;	/*MOSI, STE out*/
	P3DIR &= ~0x0C; /*MISO, SCK in*/
	P3OUT |= 0x03;	/*Select 1-wire (STE) and set idle mode (MOSI)*/
#ifdef HAVE_1WIRE_PROGRAM
	P5OUT &= ~0x09; /*Enable and pulse off*/
	P5DIR |= 0x09;
#endif
}

void bus_irq(void);
/**
 *  Bus interrupt.
 *
 *	Multiplexed bus interrupt in pin 1.0
 *  Enumeration sequence and execution of handlers.
 *
 *	\todo enumeration and allocation functions missing
 */
void bus_irq(void)
{
}


#define NOP __asm__ __volatile__("; nop")

/**
 * Approximate CPU loop pause.
 *
 *	Approximates multiples of 1 ms delay.
 *
 * \param time time in ms
 *
 */
 
void pause (uint16_t time)
{
  uint16_t i, j;
	
  for (i = 0; i < time; i++)
		/*configCPU_CLOCK_HZ/1000 = millisec, 8 cycles/loop*/
    for (j = 0; j < (configCPU_CLOCK_HZ/(1000*8)); j++) 
    {
        NOP;
    }
}

/**
 * Approximate CPU loop pause.
 *
 *	Approximates multiples of 1 us delay.
 *
 * \param time time in us
 *
 */
void pause_us (uint16_t time)
{
  uint16_t i;
  for (i = 1; i < time; i++)
	{
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
	}
}

/* Random value table */
const uint8_t rand_table[16] =
{
	0x08, 0x0b, 0x03, 0x06, 0x10, 0x0d, 0x02, 0x0a, 0x04, 0x0f, 0x0e, 0x05, 0x0c, 0x09, 0x07, 0x11
};

static uint16_t rand_seed = 0xCBFA;

/**
 * Random value function.
 *
 * Function generates a value between 0 and range.
 *
 *  \return  rand random value.
 */
uint16_t random_generate(uint16_t range) 
{
	uint8_t i;
	uint16_t retval;
	
	for (i=0; i<32; i++)
	{
		if((((rand_seed << 1) ^ rand_seed) & 0x08) != 0)
			rand_seed = (rand_seed << 1) | 1;
	  else
			rand_seed <<= 1;
	
		rand_seed += rand_table[rand_seed & 0x0f];
		if ((i>4) && (rand_seed < range)) break;
	}
	retval = rand_seed;
	while (retval > range)
	{
		retval = retval >> 1;
	}
	return retval;
}


#ifndef HAVE_POWERSAVE
#include <task.h>
#include <iomacros.h>
void vApplicationIdleHook( void );

/**
 *  Application idle hook. Set proper power mode.
 *
 */
void vApplicationIdleHook( void )
{
  for(;;)
  {
		_BIS_SR(GIE+CPUOFF+SCG0);
    taskYIELD();
  }
}
#endif
