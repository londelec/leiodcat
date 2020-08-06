/*
 ============================================================================
 Name        : leiodcat.h
 Author      : AK
 Version     : V2.01
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for LEIODC MCU main module

  Change log :

  *********V2.01 01/06/2018**************
  SPI configuration added
  Station communication structure created

  *********V2.00 07/09/2016**************
  Local function prototypes moved to leiodcat.c
  MCU port and pin definitions added
  Renamed to leiodcat.h

  *********V1.02 18/08/2015**************
  Modbus mapping created for UART settings
  Baudrate enums, table structure and handling functions created
  UART setting update functionality introduced

  *********V1.01 12/06/2015**************
  Modbus mapping changes
  Hardware string table moved to flash memory

  *********V1.00 20/02/2015**************
  Initial revision

 ============================================================================
 */

#ifndef MAIN_H_
#define MAIN_H_


#include "ledefs.h"
#include "modbusdef.h"
#include "atmeldefs.h"
#include "timer.h"


// Version numbers and constants
#define VERSION_MAJOR			1			// Firmware version number major
#define VERSION_MINOR			6			// Firmware version number minor
#if VERSION_MINOR < 10
#define FWVERSION_10TH_ZERO		"0"
#else
#define FWVERSION_10TH_ZERO		""
#endif
#define VERSION_STRING	STRINGIFY(VERSION_MAJOR) "." FWVERSION_10TH_ZERO STRINGIFY(VERSION_MINOR)


// General constants
#define MULT10USEC						100000			// Multiplier to convert from sec to 10usec


// MCU ports
#define MCUP_LED						PORTC			// LED driver
#define MCUP_IFACE						PORTD			// IO module RS232/285/422 interface selection
#define MCUP_UART						PORTE			// UART Rx and Tx pins
#define MCUP_AUTOID						PORTE			// Hardware identification resistors
#define MCUP_IOL						PORTF			// IO port [0..7]
#define MCUP_IOH						PORTH			// IO port [8..16]
#define MCUP_CTRL						PORTK			// UART RTS pin
#define MCUP_MXAUTOID					PORTK			// MX board hardware identification
#define MCUP_ADCSPI						PORTF			// MCU port where ADCs are connected (over SPI interface)

// MCU pins and masks
#define PIN_TXD							PIN7_bm			// UART TxD pin
#define PIN_LEDTX						PIN7_bm			// LED TxD pin
#define PIN_RXD							PIN6_bm			// UART RxD pin
#define PIN_MXAUTOID					PIN5_bm			// Pin to identify MX board presence
#define PIN_RTS							PIN5_bm			// UART RTS pin
#define MASK_AUTOID						ATHW_ID_MASK	// Select pins used for ID resistors
#define MASK_UIFACE						0xE0			// Select pins used for RS232/285/422 interface selection


// Reset register values
#define RESETREG_PLACEHOLDER			0x5500 			// Constant for reset register highbyte
#define RESETREG_RESET					0xaaaa 			// Reset value


// Station callback arguments
#define STAARG_MAINPROC struct station_s *staptr
#define STAARG_RX struct station_s *staptr, uint8_t *rxbuff


// Commonly used function definitions
#define MAINF_SET_CTIMEOUT timerset_fine(chanptr->chtimeout, &stacoms->comtimer);
#define MAINF_SET_SLTXDELAY timerset_fine(chanptr->chtxdelay, &stacoms->comtimer);
#define TIMER_SET_10USEC(mtconst, mtimer) timerset_fine(mtconst, mtimer);
#define MAINF_CHECK_CTIMER timercheck_fine(&stacoms->comtimer)
#define MAINF_CHECK_CHARTIMER timercheck_fine(&stacoms->chartimer)

// Various Macros
#define STATION_ISSLAVE(msta) (1)
#define LEF_MEMZEROP(mptr) memset(mptr, 0, sizeof(*(mptr)));
#define LEF_MEMZEROS(mptr) memset(&(mptr), 0, sizeof(mptr));
#define LEF_FREE(mptr)\
		if (mptr != NULL) {\
			free(mptr);\
			mptr = NULL;\
		}

// Failing generic protocol also causes station fail
#define GENPROT_SETFAIL(mgp)




// Overall project-wide type definitions
typedef	uint8_t							chanid_t;			// Channel ID size definition
typedef	uint16_t						staddr_t;			// Station address size definition
typedef	uint16_t						stacnt_t;			// Station container count
typedef	uint8_t							txrx8_t;			// Rx/Tx buffer pointer size definition
typedef	uint16_t						rxtxsize_t;			// recv/read/send function return size definition


// Channel communication return statuses
// (function return values)
typedef enum {
	chret_empty = 0,						// No data received or no data to send
	chret_rxtx,								// Data received from UART or Socket
	chret_dataerror,						// Data error, socket close requested by protocol
	chret_disabled,							// Station disabled by service
	chret_closed,							// Socket already closed due to receive error
} chret_e;


// Channel serial communication states
typedef enum {
	ser_readyrx = 0,						// Ready to receive first byte
	ser_pretxdelay,							// Delay before transmission
	ser_receiving,							// First byte received, continuing to receive
	ser_flush,								// Message receive error, flush following junk from buffer
} serstate_e;


// Modbus register addresses enums
// Don't forget to update constant definitions
// when adding new enums
typedef enum {
	atmapen_fwrev				= 0x0000,
	atmapen_temperature			= 0x0001,			// Temperature, scaled
	atmapen_tempraw,								// Raw temperature value
	atmapen_tempcal85,								// Temperature calibration value at 85 degrees
	atmapen_vddio,									// VDDIO voltage measurement
	atmapen_3v2th,									// 3V2 measurement threshold
	atmapen_reset				= 0x0077,			// Software reset
	atmapen_baudrate			= 0x0080,			// Baudrate
	atmapen_parity				= 0x0081,			// Parity
	atmapen_txdelayh			= 0x0082,			// TX delay highword
	atmapen_txdelayl			= 0x0083,			// TX delay lowword
	atmapen_timeouth			= 0x0084,			// Timeout highword
	atmapen_timeoutl			= 0x0085,			// Timeout lowword
	atmapen_t35					= 0x0086,			// t35 Timeout
	atmapen_devaddr				= 0x0087,			// Device address
	atmapen_uartif				= 0x0088,			// UART interface
#define MODBUS_SYSREG_COUNT		16					// Number of system register to be mapped
	atmapen_direg				= 0x0100,			// DI status register
	atmapen_dimode00			= 0x0110,			// DI modes
	atmapen_dimode01,
	atmapen_dimode02,
	atmapen_dimode03,
	atmapen_dimode04,
	atmapen_dimode05,
	atmapen_dimode06,
	atmapen_dimode07,
	atmapen_dimode08,
	atmapen_dimode09,
	atmapen_dimode0A,
	atmapen_dimode0B,
	atmapen_dimode0C,
	atmapen_dimode0D,
	atmapen_dimode0E,
	atmapen_dimode0F,
	atmapen_dif00				= 0x0120,			// DI filters
	atmapen_dif01,
	atmapen_dif02,
	atmapen_dif03,
	atmapen_dif04,
	atmapen_dif05,
	atmapen_dif06,
	atmapen_dif07,
	atmapen_dif08,
	atmapen_dif09,
	atmapen_dif0A,
	atmapen_dif0B,
	atmapen_dif0C,
	atmapen_dif0D,
	atmapen_dif0E,
	atmapen_dif0F,
#define MODBUS_DIMULT			2					// Number of register for each DI (multiplier)
#define MODBUS_DIREGCNT			1					// Number of DI status registers
	atmapen_airegi00			= 0x0200,			// AI value registers
	atmapen_airegi01,
	atmapen_airegi02,
	atmapen_airegi03,
	atmapen_airegi04,
	atmapen_airegi05,
	atmapen_airegi06,
	atmapen_airegi07,
	atmapen_aimode00			= 0x0210,			// AI modes
	atmapen_aimode01,
	atmapen_aimode02,
	atmapen_aimode03,
	atmapen_aimode04,
	atmapen_aimode05,
	atmapen_aimode06,
	atmapen_aimode07,
#define MODBUS_AIMULT			2					// Number of register for each AI (multiplier)
	atmapen_doreg				= 0x0300,			// DO control/status register
	atmapen_domode00			= 0x0310,			// DO modes
	atmapen_domode01,
	atmapen_domode02,
	atmapen_domode03,
	atmapen_domode04,
	atmapen_domode05,
	atmapen_domode06,
	atmapen_domode07,
	atmapen_domode08,
	atmapen_domode09,
	atmapen_domode0A,
	atmapen_domode0B,
	atmapen_domode0C,
	atmapen_domode0D,
	atmapen_domode0E,
	atmapen_domode0F,
	atmapen_dopul00				= 0x0320,			// DO pulse durations
	atmapen_dopul01,
	atmapen_dopul02,
	atmapen_dopul03,
	atmapen_dopul04,
	atmapen_dopul05,
	atmapen_dopul06,
	atmapen_dopul07,
	atmapen_dopul08,
	atmapen_dopul09,
	atmapen_dopul0A,
	atmapen_dopul0B,
	atmapen_dopul0C,
	atmapen_dopul0D,
	atmapen_dopul0E,
	atmapen_dopul0F,
#define MODBUS_DOMULT			2					// Number of register for each DO (multiplier)
#define MODBUS_DOREGCNT			1					// Number of DO status registers
} LEOPACK atmapping_e;



// Baudrate enums
typedef enum {
	atbrundefined				= 0x00,
	atbr300						= 0x11,
	atbr600						= 0x12,
	atbr1200					= 0x13,
	atbr2400					= 0x14,
	atbr4800					= 0x15,
	atbr9600					= 0x16,
	atbr19200					= 0x17,
	atbr38400					= 0x18,
	atbr57600					= 0x19,
	atbr115200					= 0x1A,
} LEOPACK atbaudrate_e;


// UART interface types
typedef enum {
	RS485		 				= 0,
	RS232						= 1,
	RS422						= 2,
} LEOPACK uartint_e;


typedef struct mainl_s {
	struct channel_s		*chan;						// Channel base pointer
	struct station_s		*sta;						// Station base pointer
	struct genprot_s		*gp;						// Generic protocol base pointer
	athw_e					hw;							// Board hardware
} mainl_t;


typedef struct channel_s {
	uint8_t					*txptr;						// Tx buffer pointer
	struct channel_s		*next;						// Next Channel Structure
	struct stacom_s			*stacoms;					// Station communication structure
	uartat_t	 			usart;						// Atmel application specific UART structure
	timerconst_t			chtxdelay;					// Delay before transmit in 10's us
	timerconst_t			chtimeout;					// Communication timeout in seconds
	txrx16_t				txlen;						// Transmit data size
} channel_t;


typedef struct stacom_s {
	void					*priv;						// Private protocol structure
	channel_t				*chaninst;					// Channel instance pointer
	finetm_t				comtimer;					// Timeout or delay timer for comparison to the monotonic time
	finetm_t				chartimer;					// Timer for message end detection by measuring idle time after each received character
	//chanid_t				chid;						// Linked channel id
	serstate_e				serstate;					// Communication state of serial station
} stacom_t;


typedef struct station_s {
	struct station_s		*next;						// Next Station pointer
	stacom_t				*stacoms;					// Station communication structure stared between stations of the same UART or socket
	void					*protocolp;					// Protocol pointer
	chret_e					(*func_mainproc)(STAARG_MAINPROC);	// Main processing function
	chret_e					(*func_rx)(STAARG_RX);		// Generic receive function
	staddr_t				address;					// Serial station address e.g. link address
	//leflags8_t				runflags;					// Runtime flags, various indications
	//leflags8_t				xmlflags;					// XML configuration flags
} station_t;


typedef struct genprot_s {
	struct genprot_s		*next;						// Next Generic Protocol Structure pointer
	struct station_s		*statptr;					// Parent Station pointer
	void					*applayer;					// Real Protocol Application structure
	//leflags8_t				xmlflags;					// XML configuration flags
} genprot_t;


/*typedef struct regvalidtablestr_  {
	atmapping_e 			mapreg;
	ModData16bitDef			lowlimit;
	ModData16bitDef			highlimit;
} regvalidtablestr;*/




// Always define global variables in C source file
extern mainl_t MainLeiodc;
extern uint8_t irqasmenum;


channel_t *channel_create(void);
genprot_t *genprot_create(station_t *staptr, size_t privsize, int addcont);
void calc_crc16(uint16_t *crc, const uint16_t poly, uint8_t databyte);
uint8_t gethwname(lechar *namebuf);
uint8_t mappinginit(Modreg16_t reg, leptr *rdptr, leptr *wrptr);
void outputpinctrl(uint8_t disable);
int uartsettvalidate(atmapping_e mapreg, Modreg16_t val);
uint8_t writevalidate(atmapping_e mapreg, Modreg16_t val, uint8_t *eeupd);

chret_e station_flush(station_t *staptr);
chret_e station_receive(station_t *staptr, uint8_t *rxbuff, txrx16_t *rxlength);


#endif /* MAIN_H_ */
