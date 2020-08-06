/*
 ============================================================================
 Name        : usart.h
 Author      : AK
 Version     : V1.03
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Atmel UART

  Change log :

  *********V1.03 07/09/2016**************
  Local function prototypes removed
  UART interface type added

  *********V1.02 09/04/2016**************
  Interrupt levels defined in irq.h now

  *********V1.01 18/08/2015**************
  RTS pin added to initialization function

  *********V1.00 19/02/2015**************
  Initial revision

 ============================================================================
 */


#ifndef USART_H_
#define USART_H_

#include "ledefs.h"
#include "irq.h"
#include "leiodcat.h"


// Enable DRE interrupt
#define UART_ENABLE_DREIRQ\
	uint8_t tempCTRLA = uartptr->mcuuart->CTRLA;\
	tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL;\
	uartptr->mcuuart->CTRLA = tempCTRLA;




void usart_init(uartat_t *uartptr, atbaud_t baudrate, atparity_t parity, uartint_e uartif);
void usartport_init(uartat_t *uartptr, USART_t *mcuuart, PORT_t *mcuport, PORT_t *ctrlport, uint8_t outputpin, uint8_t inputpin, uint8_t disabletx, uint8_t rtspin);
void usarthw_init(uartat_t *uartptr, atbaud_t baudrate, atparity_t parity, uint16_t bufsize);

chret_e channel_send(channel_t *chanptr);
int rxfifo_check(uartat_t *uartptr);

#endif /* USART_H_ */
