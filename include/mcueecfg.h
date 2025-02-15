/*
============================================================================
 Name        : mcueecfg.h
 Author      : AK
 Version     : V3.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for MCU EEPROM configuration structures

  Change log :

  *********V3.00 03/06/2018**************
  EEPROM header is checked and updated in runtime now

  *********V2.02 07/09/2016**************
  Local function prototypes removed
  UART interface type added

  *********V2.01 18/08/2015**************
  Group data prepare function pointers are now stored and called from table
  Configuration validation function created
  uart setting processing added

  *********V2.00 12/06/2015**************
  New functions enabling EEPROM writes, CRC calculation. etc
  EEPROM specific type definitions added
  Changes in EEPROM group and data enums
  DI modes added

  *********V1.00 28/12/2014**************
  Initial revision

 ============================================================================
 */

#ifndef MCUEECFG_H_
#define MCUEECFG_H_


#include <stdint.h>


#include "modbussl.h"


#define MCUEE_EESIZE			EEPROM_SIZE			// Inbuilt EEPROM


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


// EEPROM specific type definitions
typedef	uint16_t						eecfgsize_t;		// EEPROM configuration size definition
typedef	uint16_t						eegrpsize_t;		// EEPROM group size definition


// Warning, group IDs must not exceed 0x0F because
// they are used in bit set calculation, (1 << eegren_*)
typedef enum {
	eegren_empty					= 0,				// Empty
	eegren_uart0					= 1,				// UART0
	eegren_uart1					= 2,				// UART1
	eegren_powman					= 3,				// Power manager
	eegren_board					= 0x08,				// Board settings, we want to keep it higher because hopefully it will be written to EEPROM last
	eegren_last						= 0x0F,				// Last group enum
	eegren_undefined				= 0xFF				// Undefined
} LEOPACK mcueegrp_e;


// Don't change existing enums, add new ones at the end
// in order to preserve backward compatibility
// Don't change naming convention, this will affect
// configuration data initialization macro
enum {
	eedten_powman_empty				= 0,				// Empty
	eedten_powman_thadc3v2			= 1,				// ADC threshold for 3V2 detection
	eedten_powman_undefined			= 0xFF				// Undefined
} LEOPACK;


// Don't change existing enums, add new ones at the end
// in order to preserve backward compatibility
// Don't change naming convention, this will affect
// configuration data initialization macro
enum {
	eedten_uart_empty				= 0,				// Empty
	eedten_uart_baudrate			= 1,				// Baudrate
	eedten_uart_parity				= 2,				// Parity, data bits, stop bit
	eedten_uart_txdelay				= 3,				// Tx delay in 100x microseconds
	eedten_uart_timeout				= 4,				// Timeout in 100x microseconds
	eedten_uart_t35					= 5,				// T35 timeout in 100x microseconds
	eedten_uart_address				= 6,				// Device address
	eedten_uart_iface				= 7,				// Interface RS232/485/422
	eedten_uart_undefined			= 0xFF				// Undefined
} LEOPACK;


// Don't change existing enums, add new ones at the end
// in order to preserve backward compatibility
// Don't change naming convention, this will affect
// configuration data initialization macro
enum {
	eedten_board_empty				= 0,				// Empty
	eedten_board_dimode00			= 0x10,				// DI modes
	eedten_board_dimode01,
	eedten_board_dimode02,
	eedten_board_dimode03,
	eedten_board_dimode04,
	eedten_board_dimode05,
	eedten_board_dimode06,
	eedten_board_dimode07,
	eedten_board_dimode08,
	eedten_board_dimode09,
	eedten_board_dimode0A,
	eedten_board_dimode0B,
	eedten_board_dimode0C,
	eedten_board_dimode0D,
	eedten_board_dimode0E,
	eedten_board_dimode0F,
	eedten_board_diflt00			= 0x20,				// DI filter values
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
	eedten_board_domode00			= 0x30,				// DO modes
	eedten_board_domode01,
	eedten_board_domode02,
	eedten_board_domode03,
	eedten_board_domode04,
	eedten_board_domode05,
	eedten_board_domode06,
	eedten_board_domode07,
	eedten_board_domode08,
	eedten_board_domode09,
	eedten_board_domode0A,
	eedten_board_domode0B,
	eedten_board_domode0C,
	eedten_board_domode0D,
	eedten_board_domode0E,
	eedten_board_domode0F,
	eedten_board_dopul00			= 0x40,				// DO hold period values
	eedten_board_dopul01,
	eedten_board_dopul02,
	eedten_board_dopul03,
	eedten_board_dopul04,
	eedten_board_dopul05,
	eedten_board_dopul06,
	eedten_board_dopul07,
	eedten_board_dopul08,
	eedten_board_dopul09,
	eedten_board_dopul0A,
	eedten_board_dopul0B,
	eedten_board_dopul0C,
	eedten_board_dopul0D,
	eedten_board_dopul0E,
	eedten_board_dopul0F,
	eedten_board_undefined			= 0xFF				// Undefined
} LEOPACK;


typedef struct mcueeheader_s {
	uint8_t					name[11];					// Configuration name e.g. LEIODC
	uint8_t					revmajor;					// Revision major number
	uint8_t					delimiter;					// Delimiter e.g.g dot '.'
	uint8_t					revminor;					// Revision minor number
	eecfgsize_t				size;						// Size of all configuration including EE header, but excluding CRC
} mcueeheader;


/*
EEDTSTR_DEFINE(eeuartbaudrate,	uint16_t)				// UART baudrate
EEDTSTR_DEFINE(eeuartparity,	atparity_t)				// UART parity, data bits, stop bit
EEDTSTR_DEFINE(eeuarttxdelay,	atuartto_t)				// UART Tx delay in 100x microseconds
EEDTSTR_DEFINE(eeuarttimeout,	atuartto_t)				// UART Timeout in 100x microseconds
EEDTSTR_DEFINE(eeuartt35,		uint16_t)				// UART t35 Timeout in 100x microseconds
EEDTSTR_DEFINE(eeuartaddress,	DevAddrDef)				// Device address


EEDTSTR_DEFINE(eepowmanthadc3v2, uint16_t)				// ADC threshold for 3.2V detection


EEDTSTR_DEFINE(eeboarddimode,	uint16_t)				// DI mode
EEDTSTR_DEFINE(eeboarddiflt,	uint16_t)				// DI filter
EEDTSTR_DEFINE(eeboarddomode,	uint16_t)				// DO mode
EEDTSTR_DEFINE(eeboarddopul,	uint16_t)				// DO pulse duration
*/


mcueegrp_e eedata_validate(atmapping_e mapreg, Moddata16_t val, uint8_t *valid);
int eeconf_validate(int backup);
int eeconf_get(mcueegrp_e groupid, uint8_t dataid, uint32_t *rddword);
int eegroup_validate(mcueegrp_e groupid);
void eeconf_update(Modbusreg_t *regmem);
void eeconf_copy(int backup);
void eeconf_rebuild(void);


#endif /* MCUEECFG_H_ */
