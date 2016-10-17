/*
 ============================================================================
 Name        : atmeldefs.h
 Author      : AK
 Version     : V1.03
 Copyright   : Property of Londelec UK Ltd
 Description : Definitions header file for Atmel MCU projects

  Change log :

  *********V1.03 07/09/2016**************
  New hardware enums 4000, 0400, 2200 added

  *********V1.02 17/08/2015**************
  New hardware enum 3100 without MX board
  Control port added to uart structure

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
typedef	uint32_t					atbaudratedef;		/* Baudrate size definition */
typedef	uint8_t						atparitydef;		/* Parity size definition */
typedef	uint32_t					atuarttodef;		/* UART timeout size definition */


// IO definitions
#define ATMELMCU_ADC_USIGNOFS	190			// ADC offset when using unsigned mode


// Atmel board/hardware types
// Don't change enums, these need to match
// bootstrap resistors on PE port to identify the board type
#define ATHW_ID_MASK			0x3F		// Hardware ID mask
#define ATHWF_MXBOARD			0x80		// MX board identification flag
typedef enum {
	athwenundefined				= 0x00,		// Undefined hardware
	athwenmx3100v11				= 0x01,		// IO=3100 PCB V1.1 V1.2 V1.3, MX presence detected with PIN_MXAUTOID [PK5]
	athwenat3100v11				= 0x02,		// Legacy, used only for FW <= 1.04, No MX IO=3100 PCB V1.1 V1.2
	athwenat2200v10				= 0x03,		// IO=2200 PCB V1.0, MX presence detected with PIN_MXAUTOID [PK5]
	athwenat4000v10				= 0x04,		// IO=4000 PCB V1.0, MX presence detected with PIN_MXAUTOID [PK5]
	athwenat0400v10				= 0x06,		// IO=0400 PCB V1.0, MX presence detected with PIN_MXAUTOID [PK5]
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
	PORT_t	 				*ctrlport;		// MCU control port
	uint8_t 				rtspin;			// RTS pin (0 - if not defined)
	uint8_t 				disabletxpin;	// Disable Tx pin (while rebooting MX)
	uint8_t 				flags;			// Runtime flags
	uartledstr	 			*led;			// LED structure
} uartatstr;


#endif /* AT_DEFS_H_ */
