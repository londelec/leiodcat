/*
 ============================================================================
 Name        : 74lv8153.c
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : LED driver module

  Change log  :

  *********V1.01 17/08/2015**************
  New hardware 3100 without MX board

  *********V1.00 25/02/2015**************
  Initial revision

 ============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>

#include "74lv8153.h"
#include "usart.h"


// Activate RESET pin
#define LEDDRV_RESET_ACTIVATE leddriver.channel->usart.port->OUTCLR = leddriver.resetpin;
#define LEDDRV_RESET_RELEASE leddriver.channel->usart.port->OUTSET = leddriver.resetpin;


ic74lv8153str	leddriver;


/***************************************************************************
* Initialize UART for communication to 74LV8153
* [25/02/2015]
* New hardware 3100 without MX board
* [17/08/2015]
***************************************************************************/
void leddrv_init() {
	ChannelStr		*chanptr;
	uint8_t			cnt;
	uint8_t			soutpinmask = 0;


	chanptr = channelinit();				// Initialize new channel
	leddriver.channel = chanptr;
	chanptr->chserstate = enumchpretxdelay;	// RESET pin is asserted, release after timeout and start transmission
	chanptr->chtxdelay = LED_RESET_HOLD;	// RESET pin asserted holding period
	MAINF_SET_CHTXDELAY						// Set pre Tx delay timer
	leddriver.ackflags = 0;					// Reset acknowledge flags


	switch (BoardHardware) {
	/*case somerevision:
		break;*/

	case athwenmx3100v11:
	case athwenat3100v11:
	default:
		usartport_init(&chanptr->usart, &USARTC1, &PORTC, 0, PIN7_bm, 0, 0, 0);
		leddriver.soutpin[0] = PIN5_bm;
		leddriver.soutpin[1] = PIN4_bm;
		leddriver.resetpin = PIN6_bm;
		break;
	}

	//leddriver.leddata[0] = 0x5A;
	//leddriver.leddata[1] = 0xFA;

	usarthw_init(&chanptr->usart, 19200, 'N', (LED_DRIVER_COUNT * 2));


	for (cnt = 0; cnt < LED_DRIVER_COUNT; cnt++) {
		soutpinmask |= leddriver.soutpin[cnt];
	}
	chanptr->usart.port->OUTCLR = soutpinmask;			// Clear all SOUT pins
	chanptr->usart.port->DIRCLR = soutpinmask;			// SOUT pins are inputs
	PORTCFG.MPCMASK = soutpinmask;						// Apply new configuration to SOUT pins
	chanptr->usart.port->PIN0CTRL |= PORT_ISC_FALLING_gc;	// Falling edge interrupt

	chanptr->usart.port->INT1MASK = soutpinmask;		// Enable SOUT pin interrupts
	chanptr->usart.port->INTCTRL = (chanptr->usart.port->INTCTRL & ~PORT_INT1LVL_gm) | PORT_INT1LVL_LO_gc;	// Enable port interrupt

	LEDDRV_RESET_ACTIVATE								// Clear RESET pin
	chanptr->usart.port->DIRSET = leddriver.resetpin;	// RESET pin is output
}




/***************************************************************************
* Main process
* [26/02/2015]
***************************************************************************/
void leddrv_mainproc() {
	ChannelStr				*chanptr = leddriver.channel;


	switch (chanptr->chserstate) {
	case enumchreadyrx:		// Data transmission is complete, check ack flags set by SOUT pins
		if (leddriver.ackflags == ((1 << LED_DRIVER_COUNT) - 1)) {
			leddriver.ackflags = 0;
			chanptr->chserstate = enumchreceiving;	// Data has been sent successfully and acknowledged, check new request in a main loop
		}
		else {
			LEDDRV_RESET_ACTIVATE					// Activate RESET pin
			chanptr->chserstate = enumchpretxdelay;	// RESET pin is asserted, release after timeout and start transmission
			MAINF_SET_CHTXDELAY						// Set pre Tx delay timer
		}
		break;


	case enumchpretxdelay:		// RESET pin is asserted, release after timeout and start transmission
		if (MAINF_CHECK_CHTIMER == EXIT_SUCCESS) {	// Check delay
			LEDDRV_RESET_RELEASE					// Release RESET pin
			ledupdate();
		}
		break;


	case enumchreceiving:		// Check new update request
		if (leddriver.rflags & LEDRF_UPDATE_LED) {
			ledupdate();
		}
		break;

	//case enumchflush:
	default:					// Do nothing while data is being transmitted
		break;
	}
}


/*void led_set(uint8_t index)
{
	if(index < 8)
	{
		ledsCurrentState[0] |= (1 << index);
		buffer[0] = 1 | (0xF0 & ledsCurrentState[0]);
		buffer[1] = 1 | ((0x0F & ledsCurrentState[0]) << 4);
	}
	else
	{
		ledsCurrentState[1] |= (1 << (index % 8));
		buffer[0] = 1 | 2 | (0xF0 & ledsCurrentState[1]);
		buffer[1] = 1 | 2 | ((0x0F & ledsCurrentState[1]) << 4);
	}
	
	//usart_write(&usart, &buffer[0], 2);
}

void led_clear(uint8_t index) {
	if(index < 8)
	{
		ledsCurrentState[0] &= ~(1 << index);
		buffer[0] = 1 | (0xF0 & ledsCurrentState[0]);
		buffer[1] = 1 | ((0x0F & ledsCurrentState[0]) << 4);
	}
	else
	{
		ledsCurrentState[1] &= ~(1 << (index % 8));
		buffer[0] = 1 | 2 | (0xF0 & ledsCurrentState[1]);
		buffer[1] = 1 | 2 | ((0x0F & ledsCurrentState[1]) << 4);
	}
	
	//usart_write(&usart, &buffer[0], 2);
}*/


/***************************************************************************
* Update LED indication, send data to 74LV8153
* [25/02/2015]
***************************************************************************/
void ledupdate() {
	uint8_t					cnt;
	uartatstr 				*uartptr = &leddriver.channel->usart;


	for (cnt = 0; cnt < LED_DRIVER_COUNT; cnt++) {
		uartptr->rxtxbuff.fifo[((cnt + 1) * 2) - 1] = ((leddriver.leddata[cnt] & 0x0F) << 4) | (cnt << 1) | 1;
		uartptr->rxtxbuff.fifo[(cnt + 1) * 2] = (leddriver.leddata[cnt] & 0xF0) | (cnt << 1) | 1;
	}


	leddriver.rflags &= ~LEDRF_UPDATE_LED;	// Reset update request flag
	leddriver.ackflags = 0;					// Reset acknowledge flags
	uartptr->rxtxbuff.outptr = 0;
	uartptr->rxtxbuff.inptr = (LED_DRIVER_COUNT * 2);
	leddriver.channel->chserstate = enumchflush;	// This state is used to ignore led update requests while data is being transmitted to 74LV8153
	UART_ENABLE_DREIRQ		// Enable triggers the DRE interrupt immediately because TX register is empty
}



/***************************************************************************
* Clear LED status registers
* [26/02/2015]
***************************************************************************/
void ledregclear() {
	uint8_t					cnt;


	for (cnt = 0; cnt < LED_DRIVER_COUNT; cnt++) {
		leddriver.leddata[cnt] = 0;
	}
}


/***************************************************************************
* PORT pin interrupts
* [26/02/2015]
***************************************************************************/
ISR(LED_PORT_INT_VECT) {
	uint8_t			cnt;


	for (cnt = 0; cnt < LED_DRIVER_COUNT; cnt++) {
		if (!(leddriver.channel->usart.port->IN & leddriver.soutpin[cnt])) {
			leddriver.ackflags |= (1 << cnt);
		}
	}
}
