/*
============================================================================
 Name        : mcueecfg.h
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for MCU EEPROM configuration structures

  Change log  :

  *********V1.00 28/12/2014**************
  Initial revision

 ============================================================================
 */

#ifndef MCUEECFG_H_
#define MCUEECFG_H_


#include <stdint.h>


#define	MCUEE_EESIZE			EEPROM_SIZE			// Inbuilt EEPROM


// Macros
#define EEDATA_HEADER\
	uint8_t				type;\
	uint8_t				size;


#define EEDATAVAR_DEFINE(mname, mintsize)\
	struct {\
	EEDATA_HEADER\
	mintsize			value;\
	} mname;


#define EEDTSTR_DEFINE(mname, mintsize)\
	typedef struct {\
	EEDATA_HEADER\
	mintsize			value;\
	} mname;

//#define MCUEECFG_CONCAT2(s1, s2)     		s1 ## s2
#define MCUEECFG_CONCAT4(s1, s2, s3, s4)	s1 ## s2 ## s3 ## s4

// Macro for initializing configuration with actual data
// Structure will be initialized with:
// name of enum - by concatenating strings
// Size of data - byte, word, dword
// Data value - passed as argument mvalue
//#define EEDATA_ENTER_OLD(mstrname, mdname, mvalue)
//		mstrname.mdname = {MCUEECFG_CONCAT4(eedten_, mstrname, _, mdname), sizeof(eepromcfg.mstrname.mdname.value), mvalue},
#define EEDATA_ENTER(meedname, mgname, mdname, mvalue)\
		meedname = {MCUEECFG_CONCAT4(eedten_, mgname, _, mdname), sizeof(eepromcfg.meedname.value), mvalue},



typedef enum {
	eegren_empty					= 0,				// Empty
	eegren_uart0					= 1,				// UART0
	eegren_uart1					= 2,				// UART1
	eegren_board					= 3,				// Board settings
	eegren_powman					= 4,				// Power manager
	eegren_undefined				= 0xFF				// Undefined
} LEOPACK mcueegrpenum;


// Don't change existing enums, add new ones at the end
// in order to preserve backward compatibility
// Don't change naming convention, this will affect
// configuration data initialization macro
typedef enum {
	eedten_powman_empty				= 0,				// Empty
	eedten_powman_thadc3v2			= 1,				// ADC threshold for 3V2 detection
	eedten_powman_undefined			= 0xFF				// Undefined
} LEOPACK mcueedpowmanenum;


// Don't change existing enums, add new ones at the end
// in order to preserve backward compatibility
// Don't change naming convention, this will affect
// configuration data initialization macro
typedef enum {
	eedten_uart_empty				= 0,				// Empty
	eedten_uart_baudrate			= 1,				// Baudrate
	eedten_uart_parity				= 2,				// Parity, data bits, stop bit
	eedten_uart_txdelay				= 3,				// Tx delay in 100x microseconds
	eedten_uart_timeout				= 4,				// Timeout in 100x microseconds
	eedten_uart_t35					= 5,				// T35 timeout in 100x microseconds
	eedten_uart_address				= 6,				// Device address
	eedten_uart_undefined			= 0xFF				// Undefined
} LEOPACK mcueeduartenum;


// Don't change existing enums, add new ones at the end
// in order to preserve backward compatibility
// Don't change naming convention, this will affect
// configuration data initialization macro
typedef enum {
	eedten_board_empty				= 0,				// Empty
	eedten_board_diflt00			= 0x10,				// DI filter values
	eedten_board_diflt01,
	eedten_board_diflt02,
	eedten_board_diflt03,
	eedten_board_diflt04,
	eedten_board_diflt05,
	eedten_board_diflt06,
	eedten_board_diflt07,
	eedten_board_diflt08,
	eedten_board_diflt09,
	eedten_board_diflt0A,
	eedten_board_diflt0B,
	eedten_board_diflt0C,
	eedten_board_diflt0D,
	eedten_board_diflt0E,
	eedten_board_diflt0F,
	eedten_board_dohld00			= 0x20,				// DO hold period values
	eedten_board_dohld01,
	eedten_board_dohld02,
	eedten_board_dohld03,
	eedten_board_dohld04,
	eedten_board_dohld05,
	eedten_board_dohld06,
	eedten_board_dohld07,
	eedten_board_dohld08,
	eedten_board_dohld09,
	eedten_board_dohld0A,
	eedten_board_dohld0B,
	eedten_board_dohld0C,
	eedten_board_dohld0D,
	eedten_board_dohld0E,
	eedten_board_dohld0F,
	eedten_board_dopol00			= 0x30,				// DO policy values
	eedten_board_dopol01,
	eedten_board_dopol02,
	eedten_board_dopol03,
	eedten_board_dopol04,
	eedten_board_dopol05,
	eedten_board_dopol06,
	eedten_board_dopol07,
	eedten_board_dopol08,
	eedten_board_dopol09,
	eedten_board_dopol0A,
	eedten_board_dopol0B,
	eedten_board_dopol0C,
	eedten_board_dopol0D,
	eedten_board_dopol0E,
	eedten_board_dopol0F,
	eedten_board_undefined			= 0xFF				// Undefined
} LEOPACK mcueedboardenum;


typedef struct mcueeheader_ {
	uint8_t					name[11];					// Configuration name e.g. LEIODC
	uint8_t					revmajor;					// Revision major number
	uint8_t					delimiter;					// Delimiter e.g.g dot '.'
	uint8_t					revminor;					// Revision minor number
	uint16_t				size;						// Size of all configuration including EE header
} mcueeheader;


typedef struct mcueegrphead_ {
	mcueegrpenum			id;							// Group ID (UART, Powman, etc)
	uint16_t				size;						// Group size excluding group header
} mcueegrphead;


typedef struct mcueedthead_ {							// Data header
	EEDATA_HEADER										// Data type (enum) and size - byte, word, dword
} mcueedthead;


EEDTSTR_DEFINE(eeuartbaudrate,	uint32_t)				// UART baudrate
EEDTSTR_DEFINE(eeuartparity,	uint8_t)				// UART parity, data bits, stop bit
EEDTSTR_DEFINE(eeuarttxdelay,	uint32_t)				// UART Tx delay in 100x microseconds
EEDTSTR_DEFINE(eeuarttimeout,	uint32_t)				// UART Timeout in 100x microseconds
EEDTSTR_DEFINE(eeuartt35,		uint32_t)				// UART t35 Timeout in 100x microseconds
EEDTSTR_DEFINE(eeuartaddress,	uint8_t)				// Device address


/*typedef struct mcueeuart_ {
	mcueegrphead				head;					// Structure generic header
	EEDATAVAR_DEFINE(parity,	uint8_t)				// UART parity, data bits, stop bit
	EEDATAVAR_DEFINE(txdelay,	uint32_t)				// UART Tx delay in 100x microseconds
	EEDATAVAR_DEFINE(timeout,	uint32_t)				// UART Timeout in 100x microseconds
	EEDATAVAR_DEFINE(t35,		uint32_t)				// UART t35 Timeout in 100x microseconds
	EEDATAVAR_DEFINE(address,	uint8_t)				// Device address
} mcueeuart;*/

EEDTSTR_DEFINE(eepowmanthadc3v2, uint16_t)				// ADC threshold for 3.2V detection


EEDTSTR_DEFINE(eeboarddiflt,	uint16_t)				// DI filter
EEDTSTR_DEFINE(eeboarddohld,	uint16_t)				// DO hold period
EEDTSTR_DEFINE(eeboarddopol,	uint16_t)				// DO policy



uint8_t getee_data(mcueegrpenum groupid, uint8_t dataid, uint32_t *retdword);

#endif /* MCUEECFG_H_ */
