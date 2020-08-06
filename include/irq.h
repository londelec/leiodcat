/*
 ============================================================================
 Name        : irq.h
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Interrupt definitions header

  Change log :

  *********V1.01 01/06/2018**************
  SPI interrupt added

  *********V1.00 09/04/2016**************
  Initial revision

 ============================================================================
 */

#ifndef AT_IRQ_H_
#define AT_IRQ_H_

#include <avr/io.h>


// Interrupt levels
#define USART_RXCINTLVL		USART_RXCINTLVL_MED_gc		// UART Rx interrupt - medium level
#define USART_DREINTLVL		USART_DREINTLVL_MED_gc		// UART Data register empty interrupt - medium level
#define USART_TXCINTLVL		USART_TXCINTLVL_MED_gc		// UART Transmit complete interrupt - medium level

#define LEDSOUT_INTLVL		PORT_INT1LVL_MED_gc			// LED driver SOUT pin interrupt - medium level
#define FINETIMER_INTLVL	TC0_OVFINTLVL0_bm			// 100us timer, used as general timer - low level
#define TIMER1MS_INTLVL		TC1_OVFINTLVL0_bm			// 1ms timer, used for IO pin processing - low level
#define POWMANHB_INTLVL		PORT_INT1LVL_LO_gc			// Powman MX heartbeat pin interrupt - low level
#define SPI_INTLVL			SPI_INTLVL0_bm				// SPI interrupt - low level


#endif /* AT_IRQ_H_ */
