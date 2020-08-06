/*
 ============================================================================
 Name        : 74lv8153.c
 Author      : AK
 Version     : V1.05
 Copyright   : Property of Londelec UK Ltd
 Description : LED driver module

  Change log :

  *********V1.05 04/07/2019**************
  Station communication structure created

  *********V1.04 07/09/2016**************
  Local functions marked static
  LED UART is always initialized

  *********V1.03 09/04/2016**************
  Interrupt levels defined in irq.h now

  *********V1.02 24/08/2015**************
  Minor corrections, use shift instead of multiply

  *********V1.01 17/08/2015**************
  New hardware 3100 without MX board

  *********V1.00 25/02/2015**************
  Initial revision

 ============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>
#include <string.h>

#include "74lv8153.h"
#include "usart.h"
//#include "irq.h"


#ifdef GLOBAL_DEBUG
//#define DEBUG_LEDTEST
#endif	// GLOBAL_DEBUG


// Activate RESET pin
#define LEDDRV_RESET_ACTIVATE ledport->OUTCLR = leddriver.resetpin;
#define LEDDRV_RESET_RELEASE ledport->OUTSET = leddriver.resetpin;

#define LEDF_SET_TXDELAY timerset_fine(chanptr->chtxdelay, &leddriver.stacoms.comtimer);


ic74lv8153_t leddriver;




/*
 * Update LED indication, send data to 74LV8153
 * [25/02/2015]
 * Minor corrections, use shift instead of multiply
 * [24/08/2015]
 */
static void ledupdate(void) {
	uint8_t				i;
	uartat_t 			*uartptr = &leddriver.channel->usart;


	for (i = 0; i < LED_DRIVER_COUNT; i++) {
		uartptr->rxtxbuff.fifo[((i + 1) << 1) - 1] = ((leddriver.leddata[i] & 0x0F) << 4) | (i << 1) | 1;
		uartptr->rxtxbuff.fifo[(i + 1) << 1] = (leddriver.leddata[i] & 0xF0) | (i << 1) | 1;
	}


	leddriver.rflags &= ~LEDRF_UPDATE_LED;	// Reset update request flag
	leddriver.ackflags = 0;					// Reset acknowledge flags
	uartptr->rxtxbuff.outptr = 0;
	uartptr->rxtxbuff.inptr = (LED_DRIVER_COUNT << 1);
	leddriver.stacoms.serstate = ser_flush;	// This state is used to ignore led update requests while data is being transmitted to 74LV8153
	UART_ENABLE_DREIRQ		// Enable triggers the DRE interrupt immediately because TX register is empty
}


/*
 * PORT pin interrupts
 * [26/02/2015]
 */
ISR(LED_PORT_INT_VECT) {
	uint8_t			i;


	for (i = 0; i < LED_DRIVER_COUNT; i++) {
		if (!(leddriver.channel->usart.port->IN & leddriver.soutpin[i])) {
			leddriver.ackflags |= (1 << i);
		}
	}
}


/*
 * Main process
 * [26/02/2015]
 * Port access optimized
 * [08/09/2016]
 */
void leddrv_mainproc(void) {
	channel_t		*chanptr = leddriver.channel;
	PORT_t 			*ledport = chanptr->usart.port;


	switch (leddriver.stacoms.serstate) {
	case ser_readyrx:		// Data transmission is complete, check ack flags set by SOUT pins
		if (leddriver.ackflags == ((1 << LED_DRIVER_COUNT) - 1)) {
			leddriver.ackflags = 0;
			leddriver.stacoms.serstate = ser_receiving;		// Data has been sent successfully and acknowledged, check new request in a main loop
		}
		else {
			LEDDRV_RESET_ACTIVATE					// Activate RESET pin
			leddriver.stacoms.serstate = ser_pretxdelay;	// RESET pin is asserted, release after timeout and start transmission
			LEDF_SET_TXDELAY						// Set pre Tx delay timer
		}
		break;


	case ser_pretxdelay:		// RESET pin is asserted, release after timeout and start transmission
		if (timercheck_fine(&leddriver.stacoms.comtimer) == LE_OK) {	// Check delay
			LEDDRV_RESET_RELEASE					// Release RESET pin
			ledupdate();
		}
		break;


	case ser_receiving:			// Check new update request
		if (leddriver.rflags & LEDRF_UPDATE_LED) {
			ledupdate();
		}
		break;

	case ser_flush:
	default:					// Do nothing while data is being transmitted
		break;
	}
}


/*
 * Initialize UART for communication to 74LV8153
 * [25/02/2015]
 * New hardware 3100 without MX board
 * [17/08/2015]
 * SOUT pin interrupt level defined in irq.h now
 * [09/04/2016]
 * LED UART is always initialized
 * [08/09/2016]
 * Station communication structure created
 * [04/07/2019]
 */
void leddrv_init(void) {
	channel_t		*chanptr;
	uint8_t			i;
	PORT_t 			*ledport;
	uint8_t			soutpinmask = 0;


	chanptr = channel_create();
	leddriver.channel = chanptr;
	chanptr->stacoms = &leddriver.stacoms;
	leddriver.stacoms.serstate = ser_pretxdelay;	// RESET pin is asserted, release after timeout and start transmission
	chanptr->chtxdelay = LED_RESET_HOLD;	// RESET pin asserted holding period
	LEDF_SET_TXDELAY						// Set pre Tx delay timer
	leddriver.ackflags = 0;					// Reset acknowledge flags


	usartport_init(&chanptr->usart, &USARTC1, &MCUP_LED, 0, PIN_LEDTX, 0, 0, 0);
	leddriver.soutpin[0] = PIN5_bm;
	leddriver.soutpin[1] = PIN4_bm;
	leddriver.resetpin = PIN6_bm;

#ifdef DEBUG_LEDTEST
	//leddriver.leddata[0] = 0x5A;
	//leddriver.leddata[1] = 0xFA;
#endif

	usarthw_init(&chanptr->usart, 19200, 'N', (LED_DRIVER_COUNT << 1));


	for (i = 0; i < LED_DRIVER_COUNT; i++) {
		soutpinmask |= leddriver.soutpin[i];
	}
	ledport = chanptr->usart.port;
	ledport->OUTCLR = soutpinmask;				// Clear all SOUT pins
	ledport->DIRCLR = soutpinmask;				// SOUT pins are inputs
	PORTCFG.MPCMASK = soutpinmask;				// Apply new configuration to SOUT pins
	ledport->PIN0CTRL = PORT_ISC_FALLING_gc;	// Falling edge interrupt


	ledport->INT1MASK = soutpinmask;			// Enable SOUT pin interrupts
	ledport->INTCTRL = (ledport->INTCTRL & ~PORT_INT1LVL_gm) | LEDSOUT_INTLVL;	// Enable port interrupt

	LEDDRV_RESET_ACTIVATE						// Clear RESET pin
	ledport->DIRSET = leddriver.resetpin;		// RESET pin is output
}
