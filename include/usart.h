/*
 ============================================================================
 Name        : usart.h
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Atmel UART

  Change log  :

  *********V1.01 18/08/2015**************
  RTS pin added to initialization function

  *********V1.00 19/02/2015**************
  Initial revision

 ============================================================================
 */


#ifndef USART_H_
#define USART_H_

#include "ledefs.h"
#include "main.h"


#define F_USART							F_CPU			// Source clock frequency
#define USART_RX_BUFFER_SIZE			512				// Raw buffer size
//#define USART_TX_BUFFER_SIZE			256


// UART timing definitions
#define DEFAULT_TIMEOUT					50000			// UART Timeout (x100usec), default 5sec


//#define USART_FRAME35_TIMER_50US_TICKS		((F_TIMER/8) / 1000000) * 50 //115.2 ticks = 50us

// UART buffer flags
#define UARTRTF_BUFFOVFL				0x01


// UART default settings
#define DEFAULT_PARITY					'E'				// Default parity


/*! \brief Struct used when interrupt driven driver is used.
*
*  Struct containing pointer to a usart, a buffer and a location to store Data
*  register interrupt level temporary.
*/


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


// Enable DRE interrupt
#define UART_ENABLE_DREIRQ\
	uint8_t tempCTRLA = uartptr->mcuuart->CTRLA;\
	tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;\
	uartptr->mcuuart->CTRLA = tempCTRLA;




void usart_init(uartatstr *uartptr, atbaudratedef baudrate, atparitydef parity);
void usartport_init(uartatstr *uartptr, USART_t *mcuuart, PORT_t *mcuport, PORT_t *ctrlport, uint8_t outputpin, uint8_t inputpin, uint8_t disabletx, uint8_t rtspin);
void usarthw_init(uartatstr *uartptr, atbaudratedef baudrate, atparitydef parity, uint16_t rxtxbuffsize);

CHStateEnum channelrx(StatStr *staptr, uint8_t *rxbuffer, TxRx16bitDef *rxlength);
CHStateEnum channeltx(StatStr *staptr, uint8_t *txbuffer, TxRx16bitDef txlength);
uint8_t checkrx(uartatstr *uartptr);

uint8_t pushfifo(genbuffstr *buffer, uint8_t discardold);
uint8_t popfifo(genbuffstr *buffer);


void uartisr_rx(uartatstr *uartptr);
void usartisr_txcomplete(uartatstr *uartptr);
void usartisr_dataregempty(uartatstr *uartptr);


#endif /* USART_H_ */
