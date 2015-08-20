/*
 ============================================================================
 Name        : usart.c
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Atmel UART module

  Change log  :

  *********V1.01 17/08/2015**************
  Fixed: Baudrate calculation fixed by changing bselval to 16bit
  New hardware 3100 without MX board
  RTS pin control added

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */

//#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>


#include "usart.h"


#ifdef GLOBAL_DEBUG
//#define DEBUG_NOUARTTX
#endif	// GLOBAL_DEBUG


// UART control macros
#define UART_RX_ENABLE		(uartptr->mcuuart->CTRLB |= USART_RXEN_bm);
#define UART_RX_DISABLE		(uartptr->mcuuart->CTRLB &= ~USART_RXEN_bm);


// PIN manipulation macros
#define UART_RTS_RELEASE	if (uartptr->rtspin) (uartptr->ctrlport->OUTSET = uartptr->rtspin);
#define UART_RTS_ACTIVATE	if (uartptr->rtspin) (uartptr->ctrlport->OUTCLR = uartptr->rtspin);
#define UART_TXLED_ON		if (uartptr->led) uartptr->led->port->OUTCLR = uartptr->led->txled;
#define UART_TXLED_OFF		if (uartptr->led) uartptr->led->port->OUTSET = uartptr->led->txled;
#define UART_RXLED_ON		if (uartptr->led) uartptr->led->port->OUTCLR = uartptr->led->rxled;
#define UART_RXLED_OFF		if (uartptr->led) uartptr->led->port->OUTSET = uartptr->led->rxled;




/***************************************************************************
* Initialize UART
* [18/02/2015]
* New hardware 3100 without MX board
* RTS pin control added
* [20/08/2015]
***************************************************************************/
void usart_init(uartatstr *uartptr, atbaudratedef baudrate, atparitydef parity) {


	switch (BoardHardware) {
	/*case somerevision:
		break;*/

	case athwenat3100v11:
		PORTCFG.MPCMASK = PIN5_bm;
		PORTK.PIN0CTRL = PORT_INVEN_bm;
		//pinctrl_setbit(&PORTK, PIN5_bm, PORT_INVEN_bm);	// Invert RTS pin
		usartport_init(uartptr, &USARTE1, &PORTE, &PORTK, PIN7_bm, PIN6_bm, 0, PIN5_bm);
		break;

	case athwenmx3100v11:
	default:
#ifdef DEBUG_NOUARTTX
		usartport_init(uartptr, &USARTE1, &PORTE, 0, 0, PIN6_bm, 0, 0);
#else
		usartport_init(uartptr, &USARTE1, &PORTE, 0, 0, PIN6_bm, PIN7_bm, 0);
#endif
		break;
	}


	usarthw_init(uartptr, baudrate, parity, USART_RX_BUFFER_SIZE);
	uartptr->mcuuart->CTRLA = ((uartptr->mcuuart->CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc); // Enable RXC interrupt
	UART_RX_ENABLE								// Enable receiver
}



/***************************************************************************
* Initialize UART port and pins
* [04/03/2015]
* RTS pin control added
* [17/08/2015]
***************************************************************************/
void usartport_init(uartatstr *uartptr, USART_t *mcuuart, PORT_t *mcuport, PORT_t *ctrlport, uint8_t outputpin, uint8_t inputpin, uint8_t disabletx, uint8_t rtspin) {

	uartptr->mcuuart = mcuuart;					// MCU UART pointer
	uartptr->port = mcuport;					// MCU UART port
	uartptr->port->DIRSET = outputpin;			// Make pin output
	uartptr->port->DIRCLR = inputpin;			// Make pin input
	uartptr->disabletxpin = disabletx;			// Initialize disable Tx pin
	if (ctrlport) {
		uartptr->ctrlport = ctrlport;			// MCU control port
		uartptr->rtspin = rtspin;				// RTS pin
		UART_RTS_RELEASE						// Release RTS pin if defined
		uartptr->ctrlport->DIRSET = rtspin;		// Make RTS pin output
	}
}


/***************************************************************************
* Initialize UART hardware settings
* [25/02/2015]
* Fixed: bselval changed to 16bit
* [19/08/2015]
***************************************************************************/
void usarthw_init(uartatstr *uartptr, atbaudratedef baudrate, atparitydef parity, uint16_t rxtxbuffsize) {
	uint16_t 		bselval;


	uartptr->rxtxbuff.inptr = 0;
	uartptr->rxtxbuff.outptr = 0;
	uartptr->rxtxbuff.size = rxtxbuffsize;
	uartptr->rxtxbuff.fifo = calloc(rxtxbuffsize, sizeof(uint8_t));


	switch (parity) {
	case 'N':
		uartptr->mcuuart->CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;	// 8 Data bits, NONE Parity, 1 Stop bit
		break;
	case 'O':
		uartptr->mcuuart->CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_ODD_gc;		// 8 Data bits, ODD Parity, 1 Stop bit
		break;
	default:	//EVEN
		uartptr->mcuuart->CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_EVEN_gc;		// 8 Data bits, EVEN Parity, 1 Stop bit
		break;
	}

	// *  	If ScaleFactor >= 0
	// *  		BSEL = ((I/O clock frequency)/(2^(ScaleFactor)*16*Baudrate))-1
	// *  	If ScaleFactor < 0
	// *  		BSEL = (1/(2^(ScaleFactor)*16))*(((I/O clock frequency)/Baudrate)-1)

	bselval = (F_USART / (baudrate << 4)) - 1;
	USART_BAUDRATE_SET(uartptr->mcuuart, bselval, 0);


	uartptr->mcuuart->CTRLB |= USART_TXEN_bm;	// Enable transmitter
}




/***************************************************************************
* Generic receive function
* [18/02/2015]
***************************************************************************/
CHStateEnum channelrx(StatStr *staptr, uint8_t *rxbuffer, TxRx16bitDef *rxlength) {
	uartatstr 				*uartptr = &staptr->channelinst->usart;
	TxRx16bitDef			cnt;


	for (cnt = 0;; cnt++) {
		if (popfifo(&uartptr->rxtxbuff) == EXIT_SUCCESS) {
			rxbuffer[cnt] = uartptr->rxtxbuff.fifo[uartptr->rxtxbuff.outptr];
		}
		else break;
	}
	*rxlength = cnt;
	if (cnt) return CHCommsRxTx;
	return CHCommsEmpty;
}


/***************************************************************************
* Generic channel send function wrapper
* [18/02/2015]
***************************************************************************/
CHStateEnum channeltx(StatStr *staptr, uint8_t *txbuffer, TxRx16bitDef txlength) {
	uartatstr 				*uartptr = &staptr->channelinst->usart;
	TxRx16bitDef			cnt;


	uartptr->rxtxbuff.inptr = 0;
	uartptr->rxtxbuff.outptr = 0;

	for (cnt = 0; cnt < txlength; cnt++) {
		if (pushfifo(&uartptr->rxtxbuff, 0) == EXIT_SUCCESS) {		// We have adopted the 'preserve buffer contents' policy here
			uartptr->rxtxbuff.fifo[uartptr->rxtxbuff.inptr] = txbuffer[cnt];
		}
		else return CHCommsEmpty;
	}


	UART_TXLED_ON			// Turn on the TX LED if defined
	UART_RTS_ACTIVATE		// Activate RTS pin if defined
	UART_RX_DISABLE			// Disable receiver
	UART_ENABLE_DREIRQ		// Enable triggers the DRE interrupt immediately because TX register is empty
	return CHCommsRxTx;
}




/***************************************************************************
* Check if UART has received data
* [20/02/2015]
***************************************************************************/
uint8_t checkrx(uartatstr *uartptr) {

	if (uartptr->rxtxbuff.inptr == uartptr->rxtxbuff.outptr)
		return EXIT_FAILURE;

	return EXIT_SUCCESS;
}


/***************************************************************************
* Allocate space in the EVENT buffer for next entry
* [18/02/2015]
***************************************************************************/
uint8_t pushfifo(genbuffstr *buffer, uint8_t discardold) {

	if (buffer->inptr == buffer->outptr) {
		buffer->inptr++;
		if (buffer->inptr == buffer->size)
			buffer->inptr = 0;
		return EXIT_SUCCESS;
	}


	if (buffer->inptr > buffer->outptr) {
		if ((buffer->inptr + 1) == buffer->size) {
			if (discardold) {	// Discard oldest policy
				buffer->inptr = 0;
				if (buffer->outptr == 0) {
					buffer->outptr++;
					return EXIT_FAILURE;		// One byte removed from the buffer
				}
			}
			else {		// Preserve buffer contents policy
				if (buffer->outptr == 0) {
					return EXIT_FAILURE;		// Can't allocate space buffer overflow
				}
			}
		}
		else buffer->inptr++;
		return EXIT_SUCCESS;
	}


	if (buffer->inptr < buffer->outptr) {
		if (buffer->inptr == (buffer->outptr - 1)) {
			if (discardold) {	// Discard oldest policy
				buffer->inptr++;
				buffer->outptr++;
				if (buffer->outptr == buffer->size) buffer->outptr = 0;
				return EXIT_FAILURE;		// One byte removed from the buffer
			}
			else {		// Preserve buffer contents policy
				return EXIT_FAILURE;		// Can't allocate space buffer overflow
			}
		}
		else {	// No overflow
			buffer->inptr++;
			return EXIT_SUCCESS;
		}
	}
	return EXIT_FAILURE;		// Impossible situation
}


/***************************************************************************
* Get pointer to next entry in the buffer
* [18/02/2015]
***************************************************************************/
uint8_t popfifo(genbuffstr *buffer) {

	if (buffer->inptr == buffer->outptr)
		return EXIT_FAILURE;		// FIFO already empty


	if (buffer->inptr > buffer->outptr) {
		buffer->outptr++;
		return EXIT_SUCCESS;
	}

	if (buffer->inptr < buffer->outptr) {
		if (buffer->outptr == (buffer->size - 1))
			buffer->outptr = 0;
		else
			buffer->outptr++;
		return EXIT_SUCCESS;
	}
	return EXIT_FAILURE;		// Impossible situation
}




/***************************************************************************
* UART receive ISR
* [18/02/2015]
***************************************************************************/
void uartisr_rx(uartatstr *uartptr) {

	if (pushfifo(&uartptr->rxtxbuff, 0) == EXIT_SUCCESS) {		// We have adopted the 'preserve buffer contents' policy here
		uartptr->rxtxbuff.fifo[uartptr->rxtxbuff.inptr] = uartptr->mcuuart->DATA;	// Store received data byte in fifo
	}
	else uartptr->flags |= UARTRTF_BUFFOVFL;


	UART_RXLED_ON		// Turn the Rx LED ON
}


/***************************************************************************
* UART transmission complete ISR
* [18/02/2015]
***************************************************************************/
void usartisr_txcomplete(uartatstr *uartptr) {

	uartptr->rxtxbuff.inptr = 0;
	uartptr->rxtxbuff.outptr = 0;


	UART_RTS_RELEASE		// Release RTS pin if defined
	UART_TXLED_OFF			// Turn off the TX LED if defined

	uint8_t tempCTRLA = uartptr->mcuuart->CTRLA;
	tempCTRLA = (tempCTRLA & ~USART_TXCINTLVL_gm);	// Disable TXC interrupt
	uartptr->mcuuart->CTRLA = tempCTRLA;

	uartptr->mcuuart->CTRLB |= USART_RXEN_bm;		// Enable Receiver
}


/***************************************************************************
* UART TX register empty ISR
* Load next byte into TX register
* [18/02/2015]
***************************************************************************/
void usartisr_dataregempty(uartatstr *uartptr) {
	uint8_t 		txbyte;


	if (popfifo(&uartptr->rxtxbuff) == EXIT_SUCCESS) {
		txbyte = uartptr->rxtxbuff.fifo[uartptr->rxtxbuff.outptr];		// Get data from fifo
		if (uartptr->rxtxbuff.inptr == uartptr->rxtxbuff.outptr) {		// Last byte to transmit, enable Tx complete interrupt
			uint8_t tempCTRLA = uartptr->mcuuart->CTRLA;
			tempCTRLA = (tempCTRLA & ~USART_TXCINTLVL_gm) | USART_TXCINTLVL_LO_gc;	// Enable TXC interrupt
			uartptr->mcuuart->CTRLA = tempCTRLA;
		}
		uartptr->mcuuart->DATA = txbyte;	// Store byte in TX register
	}
	else {	// All data has been transmitted
		uint8_t tempCTRLA = uartptr->mcuuart->CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm);			// Disable DRE interrupt
		uartptr->mcuuart->CTRLA = tempCTRLA;
	}
}


/***************************************************************************
* USART interrupts
* [25/02/2015]
***************************************************************************/
ISR(UART_GENERIC_IRQ_VECTOR) {
	ChannelStr		*chanptr;
	USART_t 		*requart;


	switch (irqasmenum) {
	case USARTC1_RXC_vect_num:
	case USARTC1_DRE_vect_num:
	case USARTC1_TXC_vect_num:
		requart = &USARTC1;
		break;

	case USARTE1_RXC_vect_num:
	case USARTE1_DRE_vect_num:
	case USARTE1_TXC_vect_num:
		requart = &USARTE1;
		break;

	default:
		break;
	}


	for (chanptr = Channel0Ptr; chanptr; chanptr = chanptr->next) {
		if (chanptr->usart.mcuuart == requart) {
			switch (irqasmenum) {
			case USARTC1_RXC_vect_num:
			case USARTE1_RXC_vect_num:
				uartisr_rx(&chanptr->usart);
				break;

			case USARTC1_DRE_vect_num:
			case USARTE1_DRE_vect_num:
				usartisr_dataregempty(&chanptr->usart);
				break;

			case USARTC1_TXC_vect_num:
			case USARTE1_TXC_vect_num:
				usartisr_txcomplete(&chanptr->usart);
				chanptr->chserstate = enumchreadyrx;
				MAINF_SET_CHTIMEOUT		// Activate Timeout timer
				break;

			default:
				break;
			}
		}
	}
}
