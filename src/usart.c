/*
 ============================================================================
 Name        : usart.c
 Author      : AK
 Version     : V1.05
 Copyright   : Property of Londelec UK Ltd
 Description : Atmel UART module

  Change log :

  *********V1.05 04/07/2019**************
  Fixed: don't check rx fifo while receiver is disabled

  *********V1.04 07/09/2016**************
  Local functions marked static
  UART interface type added

  *********V1.03 09/04/2016**************
  Fixed: Must enable TXC interrupt when loading last byte
  Interrupt levels defined in irq.h now

  *********V1.02 24/08/2015**************
  Fixed: Reset TXCIF flag before enabling TXC interrupt

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
//#define DEBUG_AI3100
#endif	// GLOBAL_DEBUG


// Clock source frequency
#define F_USART							F_CPU			// Source clock frequency

// Buffer sizes
#define USART_RX_BUFFER_SIZE			512				// Raw buffer size

// UART buffer flags
#define UARTRTF_BUFFOVFL				0x01			// Receive buffer overflow
#define UARTRTF_NORX					0x02			// UART doesn't receive (for 74lv8153 communication)


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
#define UART_RS232_ON		MCUP_IFACE.OUTSET = 0x80;
#define UART_RS485_ON		MCUP_IFACE.OUTCLR = 0x40;
#define UART_RS422_ON		MCUP_IFACE.OUTCLR = 0x20;\
							MCUP_CTRL.OUTSET = PIN_RTS;\
							MCUP_CTRL.DIRSET = PIN_RTS;




/*! \brief Set USART baud rate.
 *
 *  Sets the USART's baud rate register.
 *
 *  UBRR_Value   : Value written to UBRR
 *  ScaleFactor  : Time Base Generator Scale Factor
 *
 *  Equation for calculation of BSEL value in asynchronous normal speed mode:
 *  	If ScaleFactor >= 0
 *  		BSEL = ((I/O clock frequency)/(2^(ScaleFactor)*16*Baudrate))-1
 *  	If ScaleFactor < 0
 *  		BSEL = (1/(2^(ScaleFactor)*16))*(((I/O clock frequency)/Baudrate)-1)
 *
 *	\note See XMEGA manual for equations for calculation of BSEL value in other
 *        modes.
 *
 *  \param _usart          Pointer to the USART module.
 *  \param _bselValue      Value to write to BSEL part of Baud control register.
 *                         Use uint16_t type.
 *  \param _bScaleFactor   USART baud rate scale factor.
 *                         Use uint8_t type
 */
#define USART_BAUDRATE_SET(_usart, _bselValue, _bScaleFactor)\
		(_usart)->BAUDCTRLA =(uint8_t)_bselValue;\
		(_usart)->BAUDCTRLB =(_bScaleFactor << USART_BSCALE0_bp)|(_bselValue >> 8)




/***************************************************************************
* Allocate space in the buffer for next byte
* [18/02/2015]
***************************************************************************/
static uint8_t pushfifo(genbuff_t *buffer, uint8_t discardold) {

	if (buffer->inptr == buffer->outptr) {
		buffer->inptr++;
		if (buffer->inptr == buffer->size)
			buffer->inptr = 0;
		return LE_OK;
	}


	if (buffer->inptr > buffer->outptr) {
		if ((buffer->inptr + 1) == buffer->size) {
			if (discardold) {	// Discard oldest policy
				buffer->inptr = 0;
				if (buffer->outptr == 0) {
					buffer->outptr++;
					return LE_FAIL;		// One byte removed from the buffer
				}
			}
			else {		// Preserve buffer contents policy
				if (buffer->outptr == 0) {
					return LE_FAIL;		// Can't allocate space buffer overflow
				}
			}
		}
		else
			buffer->inptr++;
		return LE_OK;
	}


	if (buffer->inptr < buffer->outptr) {
		if (buffer->inptr == (buffer->outptr - 1)) {
			if (discardold) {	// Discard oldest policy
				buffer->inptr++;
				buffer->outptr++;
				if (buffer->outptr == buffer->size)
					buffer->outptr = 0;
				return LE_FAIL;		// One byte removed from the buffer
			}
			else {		// Preserve buffer contents policy
				return LE_FAIL;		// Can't allocate space buffer overflow
			}
		}
		else {	// No overflow
			buffer->inptr++;
			return LE_OK;
		}
	}
	return LE_FAIL;		// Impossible situation
}


/***************************************************************************
* Get pointer to next entry in the buffer
* [18/02/2015]
***************************************************************************/
static uint8_t popfifo(genbuff_t *buffer) {

	if (buffer->inptr == buffer->outptr)
		return LE_FAIL;		// FIFO already empty


	if (buffer->inptr > buffer->outptr) {
		buffer->outptr++;
		return LE_OK;
	}

	if (buffer->inptr < buffer->outptr) {
		if (buffer->outptr == (buffer->size - 1))
			buffer->outptr = 0;
		else
			buffer->outptr++;
		return LE_OK;
	}
	return LE_FAIL;		// Impossible situation
}


/***************************************************************************
* Initialize UART port and pins
* [04/03/2015]
* RTS pin control added
* [17/08/2015]
* Set no receive flag is input pin is not defined
* [08/04/2016]
* RTS pin inversion moved to this function
* [08/09/2016]
***************************************************************************/
void usartport_init(uartat_t *uartptr, USART_t *mcuuart, PORT_t *mcuport, PORT_t *ctrlport, uint8_t outputpin, uint8_t inputpin, uint8_t disabletx, uint8_t rtspin) {

	uartptr->mcuuart = mcuuart;				// MCU UART pointer
	uartptr->port = mcuport;				// MCU UART port
	mcuport->DIRSET = outputpin;			// Make pin output
	mcuport->DIRCLR = inputpin;				// Make pin input
	uartptr->disabletxpin = disabletx;		// Initialize disable Tx pin

	if (ctrlport) {
		uartptr->ctrlport = ctrlport;		// MCU control port
		uartptr->rtspin = rtspin;			// RTS pin
		PORTCFG.MPCMASK = rtspin;
		ctrlport->PIN0CTRL = PORT_INVEN_bm;	// Invert RTS pin

		UART_RTS_RELEASE					// Release RTS pin if defined
		ctrlport->DIRSET = rtspin;			// Make RTS pin output
	}

	if (!inputpin)
		uartptr->flags |= UARTRTF_NORX;		// This UART doesn't receive
}


/***************************************************************************
* Initialize UART hardware settings
* [25/02/2015]
* Fixed: bselval changed to 16bit
* [19/08/2015]
***************************************************************************/
void usarthw_init(uartat_t *uartptr, atbaud_t baudrate, atparity_t parity, uint16_t bufsize) {
	uint16_t 		bselval;


	uartptr->rxtxbuff.inptr = 0;
	uartptr->rxtxbuff.outptr = 0;
	uartptr->rxtxbuff.size = bufsize;
	uartptr->rxtxbuff.fifo = calloc(bufsize, sizeof(uint8_t));


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


/*
 * Flush any socket data station has received
 * [04/07/2019]
 */
chret_e station_flush(station_t *staptr) {
	uartat_t 		*uartptr = &staptr->stacoms->chaninst->usart;


	uartptr->rxtxbuff.inptr = 0;
	uartptr->rxtxbuff.outptr = 0;
	return chret_empty;
}


/*
 * Genreic station receive function
 * [18/02/2015]
 * Argument changed to station pointer
 * [03/07/2019]
 */
chret_e station_receive(station_t *staptr, uint8_t *rxbuff, txrx16_t *rxlength) {
	uartat_t 		*uartptr = &staptr->stacoms->chaninst->usart;
	txrx16_t		i = 0;


	while ((popfifo(&uartptr->rxtxbuff) == LE_OK)) {
		rxbuff[i] = uartptr->rxtxbuff.fifo[uartptr->rxtxbuff.outptr];
		i++;
	}

	*rxlength = i;
	if (i)
		return chret_rxtx;

	return chret_empty;
}


/*
 * Send data to Channel
 * [18/02/2015]
 * Argument changed to channel pointer
 * [03/07/2019]
 */
chret_e channel_send(channel_t *chanptr) {
	uartat_t 		*uartptr = &chanptr->usart;
	txrx16_t		i;


	uartptr->rxtxbuff.inptr = 0;
	uartptr->rxtxbuff.outptr = 0;


	for (i = 0; i < chanptr->txlen; i++) {
		if (pushfifo(&uartptr->rxtxbuff, 0) == LE_OK) {		// We have adopted the 'preserve buffer contents' policy here
			uartptr->rxtxbuff.fifo[uartptr->rxtxbuff.inptr] = chanptr->txptr[i];
		}
		else
			return chret_empty;
	}


	UART_TXLED_ON			// Turn on the TX LED if defined
	UART_RTS_ACTIVATE		// Activate RTS pin if defined
	UART_RX_DISABLE			// Disable receiver
	UART_ENABLE_DREIRQ		// Enable triggers the DRE interrupt immediately because TX register is empty
	return chret_rxtx;
}


/*
 * Check if UART has received data
 * [20/02/2015]
 * Fixed: don't check rx fifo while receiver is disabled
 * [04/07/2019]
 */
int rxfifo_check(uartat_t *uartptr) {

	if (
			(uartptr->mcuuart->CTRLB & USART_RXEN_bm) &&
			(uartptr->rxtxbuff.inptr != uartptr->rxtxbuff.outptr))
		return LE_OK;

	return LE_FAIL;
}


/*
 * UART receive ISR
 * [18/02/2015]
 */
static void uartisr_rx(uartat_t *uartptr) {

	if (pushfifo(&uartptr->rxtxbuff, 0) == LE_OK) {		// We have adopted the 'preserve buffer contents' policy here
		uartptr->rxtxbuff.fifo[uartptr->rxtxbuff.inptr] = uartptr->mcuuart->DATA;	// Store received data byte in fifo
	}
	else
		uartptr->flags |= UARTRTF_BUFFOVFL;


	UART_RXLED_ON		// Turn the Rx LED ON
}


/*
 * UART transmission complete ISR
 * [18/02/2015]
 * Enable receiver only if UART is supposed to receive
 * [08/04/2016]
 */
static void usartisr_txcomplete(uartat_t *uartptr) {

	uartptr->rxtxbuff.inptr = 0;
	uartptr->rxtxbuff.outptr = 0;

	UART_RTS_RELEASE		// Release RTS pin if defined
	UART_TXLED_OFF			// Turn off the TX LED if defined

	uartptr->mcuuart->CTRLA &= ~USART_TXCINTLVL_gm;	// Disable TXC interrupt

	if (!(uartptr->flags & UARTRTF_NORX))
		UART_RX_ENABLE		// Enable Receiver
}


/*
 * UART TX register empty ISR
 * Load next byte into TX register
 * [18/02/2015]
 * Fixed: Reset TXCIF flag before enabling TXC interrupt
 * [24/08/2015]
 * Fixed: Must enable TXC interrupt when loading last byte
 * The problem occurred when TXC and DRE interrupts happened at the same time.
 * DRE being higher priority was serviced first and it cleared TXCIF status bit.
 * After returning from DRE, TXCIF bit was no longer set which lead to TXC interrupt
 * never being serviced.
 * [09/04/2016]
 */
static void usartisr_dataregempty(uartat_t *uartptr) {
	uint8_t 		txbyte;


	if (popfifo(&uartptr->rxtxbuff) == LE_OK) {
		txbyte = uartptr->rxtxbuff.fifo[uartptr->rxtxbuff.outptr];		// Get data from fifo

		uartptr->mcuuart->DATA = txbyte;	// We must store byte in TX register before enabling TXC interrupt

		if (uartptr->rxtxbuff.inptr == uartptr->rxtxbuff.outptr) {		// Last byte to transmit, enable Tx complete interrupt
			uartptr->mcuuart->STATUS |= USART_TXCIF_bm;					// Reset TXC interrupt flag
			uint8_t tempCTRLA = uartptr->mcuuart->CTRLA;
			tempCTRLA = (tempCTRLA & ~USART_TXCINTLVL_gm) | USART_TXCINTLVL;	// Enable TXC interrupt
			uartptr->mcuuart->CTRLA = tempCTRLA;
		}
	}
	else {	// All data has been transmitted
		uartptr->mcuuart->CTRLA &= ~USART_DREINTLVL_gm;		// Disable DRE interrupt
	}
}


/*
 * USART interrupts
 * [25/02/2015]
 * Return if irq number is not recognized
 * [08/04/2016]
 */
ISR(UART_GENERIC_IRQ_VECTOR) {
	channel_t		*chanptr;
	USART_t 		*requart;
	stacom_t		*stacoms;


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

	default:	// Unknown irq number
		return;
	}


	for (chanptr = MainLeiodc.chan; chanptr; chanptr = chanptr->next) {
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
				stacoms = chanptr->stacoms;
				stacoms->serstate = ser_readyrx;
				MAINF_SET_CTIMEOUT		// Activate Timeout timer
				break;

			default:
				break;
			}
			break;	// UART found and processed
		}
	}
}


/*
 * Initialize UART
 * [18/02/2015]
 * New hardware 3100 without MX board
 * RTS pin control added
 * [20/08/2015]
 * Rx interrupt level defined in irq.h now
 * [09/04/2016]
 * UART interface type added
 * [08/09/2016]
 */
void usart_init(uartat_t *uartptr, atbaud_t baudrate, atparity_t parity, uartint_e uartif) {

	if (MainLeiodc.hw & ATHWF_MXBOARD) {	// MX board present
#ifdef DEBUG_NOUARTTX
		usartport_init(uartptr, &USARTE1, &MCUP_UART, 0, 0, PIN_RXD, 0, 0);
#else
		usartport_init(uartptr, &USARTE1, &MCUP_UART, 0, 0, PIN_RXD, PIN_TXD, 0);
#endif
	}
	else {	// IO module without MX board
#ifdef DEBUG_AI3100
		goto nocontrol;
#else
		MCUP_IFACE.OUT = 0x60;				// All interfaces disabled RS232[7] = 0; RS485[6] = 1; RS422[5] = 1
		MCUP_IFACE.DIRSET = MASK_UIFACE;	// Make pins [5..7] outputs
#endif

		switch (uartif) {
		case RS232:
			UART_RS232_ON
			goto nocontrol;

		case RS422:
			UART_RS422_ON
			nocontrol:
			usartport_init(uartptr, &USARTE1, &MCUP_UART, 0, PIN_TXD, PIN_RXD, 0, 0);
			break;

		case RS485:
		default:
			UART_RS485_ON
			usartport_init(uartptr, &USARTE1, &MCUP_UART, &MCUP_CTRL, PIN_TXD, PIN_RXD, 0, PIN_RTS);
			break;
		}
	}


	usarthw_init(uartptr, baudrate, parity, USART_RX_BUFFER_SIZE);
	uartptr->mcuuart->CTRLA = ((uartptr->mcuuart->CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL); // Enable RXC interrupt
	UART_RX_ENABLE	// Enable receiver
}

