/*
 ============================================================================
 Name        : atmeldefs.h
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Definitions header file for Atmel MCU projects

  Change log  :

  *********V1.01 16/06/2015**************
  MCU bit set type definition created
  ADC unsigned offset constant created

  *********V1.00 18/02/2015**************
  Initial revision

 ============================================================================
 */

#ifndef AT_DEFS_H_
#define AT_DEFS_H_

#include <avr/io.h>
#include <stdint.h>

#include "ledefs.h"


typedef	uint8_t						rxbytesdef;			/* recv/read/send function return size definition */
typedef	uint32_t					TimerConstDef;		/* 32bit Timer Constant size definition */
typedef	uint16_t					mcubitsetDef;		/* Bit set size definition */


// IO definitions
#define ATMELMCU_ADC_USIGNOFS	190			// ADC offset when using unsigned mode


// Atmel board/hardware types
// Don't change enums, these need to match
// bootstrap resistors on PE port to properly identify the board type
typedef enum {
	athwenundefined				= 0,		// Undefined hardware
	athwenmx3100v11				= 1,		// Board with MX present IO=3100
} LEOPACK athwenum;


typedef struct genbuffstr_ {
	uint8_t					*fifo;
	uint16_t				inptr;
	uint16_t				outptr;
	uint16_t				size;
} genbuffstr;


typedef struct uartledstr_ {
	PORT_t					*port;			// MCU port structure
	uint8_t					rxled;			// RX led pin number
	uint8_t 				txled;			// TX led pin number
	uint8_t 				errled;			// Error led pin number
} uartledstr;


typedef struct uartatstr_ {
	USART_t 				*mcuuart;		// Pointer to USART module to use
	genbuffstr 				rxtxbuff;		// RX/TX buffer structure
	PORT_t	 				*port;			// MCU UART port
	uint8_t 				rtspin;			// RTS pin (0 - if not defined)
	uint8_t 				disabletxpin;	// Disable Tx pin (while rebooting MX)
	uint8_t 				flags;			// Runtime flags
	uartledstr	 			*led;			// LED structure
} uartatstr;



#endif /* AT_DEFS_H_ */
