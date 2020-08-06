/*
 ============================================================================
 Name        : leiodcat.c
 Author      : AK
 Version     : V2.01
 Copyright   : Property of Londelec UK Ltd
 Description : LEIODC MCU main module

  Change log :

  *********V2.01 01/06/2018**************
  New hardware 0040 added
  Analog inputs added
  Station communication structure created
  EEPROM include file removed
  Timeout and t35 min value checks added

  *********V2.00 07/09/2016**************
  UART interface type added
  MX board flag created
  Main LEIODC structure created
  Local functions marked static
  Renamed to leiodcat.c

  *********V1.03 24/08/2015**************
  Additional control added to ensure t35 is always greater than 1msec
  LED driver main process moved to board.c

  *********V1.02 19/08/2015**************
  New hardware 3100 without MX board
  Baudrate enums, table and handling functions created
  UART setting update functionality introduced
  t35 timeout automatic calculation added
  Default configuration flag created

  *********V1.01 12/06/2015**************
  Firmware string constants and tables moved to flash memory
  UART settings are hardcoded by default, not read from EEPROM
  EEPROM configuration is check and restructured if necessary on startup
  Modbus mapping changes

  *********V1.00 24/02/2015**************
  Initial revision

 ============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "modbus.h"
#include "powman.h"
#include "timer.h"
#include "usart.h"
#include "74lv8153.h"


#ifdef EEPROM_CFG
#include <avr/eeprom.h>
#include "mcueecfg.h"
//#include "ateecfg.inc"
#endif


#ifndef VERSION_MAJOR
#error "VERSION_MAJOR must be defined"
#endif
#ifndef VERSION_MINOR
#error "VERSION_MINOR must be defined"
#endif


const lechar FirmwareVersion[] PROGMEM = " FirmwareVersion="\
								VERSION_STRING\
								" ";
#include "leiodcatdate.txt"


#ifdef GLOBAL_DEBUG
//#define HARDCODED_UART_SETTINGS				// Don't load UART setting from EEPROM
//#define IGNORE_EE_CRC_ERROR						// Don't rebuild EEPROM configuration if there is a CRC error
#endif	// GLOBAL_DEBUG


// UART default settings
#define DEFAULT_PARITY					'E'				// Default parity
#define MODBUS_RXT35CONST				350000			// Modbus t35 (char) timeout conversion constant in 100usecs
#define MIN_T35							10				// Minimal value of t35 (char) timer in 100usecs (1msec)


// Board hardware type
static const uint16_t FwRevNumber = (VERSION_MAJOR * 100) + VERSION_MINOR;


// Global variables
mainl_t MainLeiodc = {
		.chan = NULL,
		.sta = NULL,
		.gp = NULL,
		.hw = athwenundefined,
};


static const lechar athwundefinedc[] PROGMEM = "Unknown";
static const lechar athwmxc[] PROGMEM = "LEIODC-MX-";
static const lechar athwatc[] PROGMEM = "LEIODC-AT-";
static const lechar athw3100c[] PROGMEM = "3100";
static const lechar athw2200c[] PROGMEM = "2200";
static const lechar athw4000c[] PROGMEM = "4000";
static const lechar athw0400c[] PROGMEM = "0400";
static const lechar athw0040c[] PROGMEM = "0040";
static const struct hardwarename_s {
	athw_e					type;
	leptr					sptr;
} hwnametable[] PROGMEM = {
	{athwenmx3100v11,	 	(leptr) athw3100c},
	{athwenat3100v11,	 	(leptr) athw3100c},
	{athwenat2200v10,	 	(leptr) athw2200c},
	{athwenat4000v10,	 	(leptr) athw4000c},
	{athwenat0400v10,	 	(leptr) athw0400c},
	{athwenat0040v10,	 	(leptr) athw0040c},
};


//const regvalidtablestr regvalidtable[] PROGMEM = {
//	{atmapen_devaddr, 		1,						254},
//};


typedef struct baudratetab_s {
	atbaudrate_e			brenum;
	atbaud_t 				baudrate;
} baudratetab_t;

static const baudratetab_t UARTBaudrateTable[] PROGMEM =
{
	{atbr300,				300},
	{atbr600,				600},
	{atbr1200,				1200},
	{atbr2400,				2400},
	{atbr4800,				4800},
	{atbr9600,				9600},
	{atbr19200,				19200},
	{atbr38400,				38400},
	{atbr57600,				57600},
	{atbr115200,			115200},
};




/*
 * Calculate CRC16
 * [27/10/2014]
 */
void calc_crc16(uint16_t *crc, const uint16_t poly, uint8_t databyte) {
	uint8_t		i;


	(*crc) ^= databyte;
	for (i = 0; i < 8; i++) {
		if ((*crc) & 1) {
			(*crc) >>= 1;
			(*crc) ^= poly;
		}
		else
			(*crc) >>= 1;
	}
}


/*
 * Protocol receive process
 * [20/02/2015]
 * Rx function called directly
 * [07/09/2016]
 * Station communication structure created
 * [03/07/2019]
 */
static void protocol_rxproc(void) {
	station_t		*staptr;
	//chret_e			chstate;


	for (staptr = MainLeiodc.sta; staptr; staptr = staptr->next) {
		if (rxfifo_check(&staptr->stacoms->chaninst->usart) == LE_OK) {
			if (staptr->func_rx)
				staptr->func_rx(staptr, NULL);
			//	chstate = staptr->func_rx(staptr, NULL);
			else
				station_flush(staptr);
			POWMAN_RSTIDLE		// Reset MX idle counter
		}
	}
}


/*
 * Protocol main process
 * [20/02/2015]
 * Main processing function called directly
 * [07/09/2016]
 */
static void protocol_mainproc(station_t *staptr) {
	chret_e			chstate = chret_empty;
	channel_t		*chanptr = staptr->stacoms->chaninst;


	chanptr->txlen = 0;
	if (staptr->func_mainproc)
		chstate = staptr->func_mainproc(staptr);


	switch (chstate) {
	case chret_rxtx:
		if (chanptr->txlen) {
			chstate = channel_send(chanptr);
		}
		break;


	default:
		break;
	}
}


/*
 * Station main process
 * [03/07/2019]
 */
static void station_main(void) {
	station_t		*staptr;


	for (staptr = MainLeiodc.sta; staptr; staptr = staptr->next) {
		if ((!staptr->stacoms) || (!staptr->stacoms->chaninst))
			continue;

		protocol_mainproc(staptr);
	}
}


/*
 * Create new channel
 * [03/07/2019]
 */
channel_t *channel_create(void) {
	channel_t		*chanptr;


	if ((chanptr = calloc(1, sizeof(*chanptr)))) {
		if (!MainLeiodc.chan) 	// This is the first channel
			MainLeiodc.chan = chanptr;
		else {
			channel_t *prevch;
			for (prevch = MainLeiodc.chan; prevch->next; prevch = prevch->next) {}
			prevch->next = chanptr;
		}
	}
	return chanptr;
}


/*
 * Create new generic protocol
 * [02/07/2019]
 */
genprot_t *genprot_create(station_t *staptr, size_t privsize, int addcont) {
	genprot_t		*gprot;
	size_t			memsize = sizeof(*gprot) + privsize;


	if (!(gprot = calloc(1, memsize)))
		return NULL;


	if (!MainLeiodc.gp) 	// This is the first Generic protocol
		MainLeiodc.gp = gprot;
	else {
		genprot_t *prevgp;
		for (prevgp = MainLeiodc.gp; prevgp->next; prevgp = prevgp->next) {}
		prevgp->next = gprot;
	}
	gprot->statptr = staptr;

	if (privsize)	// App layer doesn't exist if size is 0
		gprot->applayer = (void *) (((leptr) gprot) + sizeof(*gprot));

	return gprot;
}


/*
 * Create new station
 * [03/07/2019]
 */
static station_t *station_create(void) {
	station_t			*staptr;
	stacom_t			*stacoms;


	if ((staptr = calloc(1, sizeof(*staptr) + sizeof(*stacoms)))) {
		if (!MainLeiodc.sta) 	// This is the first station
			MainLeiodc.sta = staptr;
		else {
			station_t *prevst;
			for (prevst = MainLeiodc.sta; prevst->next; prevst = prevst->next) {}
			prevst->next = staptr;
		}

		staptr->stacoms = (void *) (((uint8_t *) staptr) + sizeof(*staptr));
	}
	return staptr;
}


/*
 * Convert between baudrate enum and decimal value or vice versa
 * [19/08/2015]
 * Program space memcpy function used
 * [10/09/2016]
 */
static uint8_t baudrateconv(uint8_t brenum, atbaud_t *baudrate) {
	uint8_t			cnt;
	baudratetab_t	tabval;


	for (cnt = 0; cnt < (ARRAY_SIZE(UARTBaudrateTable)); cnt++) {
		memcpy_P(&tabval, &UARTBaudrateTable[cnt], sizeof(UARTBaudrateTable[0])); // Read baudrate table data program space
		if (tabval.brenum == brenum) {
			if (baudrate)
				*baudrate = tabval.baudrate;
			return LE_OK;
		}
		//else if (baudrate && *baudrate && (tabbrval == *baudrate)) {
		//	if (brenum16) *brenum16 = tabenum;
		//	return LE_OK;
		//}
	}
	return LE_FAIL;	// Baudrate not found
}


/*
 * Initialize communication interfaces
 * [17/02/2015]
 * UART setting update functionality introduced
 * t35 timeout automatic calculation added
 * Default configuration flag created
 * [18/08/2015]
 * Additional control added to ensure t35 is always greater than 1msec
 * [24/08/2015]
 * UART interface type added
 * [08/09/2016]
 * Station communication structure created
 * [04/07/2019]
 * Timeout lower limit introduced
 * [05/08/2020]
 */
static void comms_init(void) {
	channel_t			*chanptr;
	station_t			*staptr;
	genprot_t			*gprot;
	uint32_t			eedword;
	atbaud_t			baudrate = 0;
	atparity_t			parity = DEFAULT_PARITY;
	atbaudrate_e		defaultbr;
	int					reqeeupd = 0;
	uint8_t				devaddr = 1;
	uint16_t			t35;
	uartint_e			uartif = RS485;


	chanptr = channel_create();
	staptr = station_create();
	staptr->stacoms->chaninst = chanptr;
	chanptr->stacoms = staptr->stacoms;
	boardio.uartee.parity = DEFAULT_PARITY;	// Default parity
	*((atuartto_t *) &boardio.uartee.timeoutl) = MIN_T35;	// Default value 1msec
	boardio.uartee.t35 = 0;			// Undefined, default value will be used
	boardio.uartee.devaddr = 1;		// Default address


	if (MainLeiodc.hw & ATHWF_MXBOARD)
		defaultbr = atbr115200;		// Default baudrate
	else
		defaultbr = atbr9600;		// Default baudrate
	boardio.uartee.bren16 = defaultbr;


#ifndef HARDCODED_UART_SETTINGS
	if (eeconf_get(eegren_uart0, eedten_uart_baudrate, &eedword) == LE_OK) {
		if (baudrateconv(eedword, &baudrate) == LE_OK)
			boardio.uartee.bren16 = eedword;
		else
			reqeeupd = 1;
	}

	if (eeconf_get(eegren_uart0, eedten_uart_parity, &eedword) == LE_OK) {
		if (uartsettvalidate(atmapen_parity, eedword) == LE_OK)
			boardio.uartee.parity = eedword;
		else
			reqeeupd = 1;
	}

	if (eeconf_get(eegren_uart0, eedten_uart_txdelay, &eedword) == LE_OK)
		*((atuartto_t *) &boardio.uartee.txdelayl) = eedword;

	if (eeconf_get(eegren_uart0, eedten_uart_timeout, &eedword) == LE_OK)
		*((atuartto_t *) &boardio.uartee.timeoutl) = eedword;

	if (eeconf_get(eegren_uart0, eedten_uart_t35, &eedword) == LE_OK)
		boardio.uartee.t35 = eedword;


	if (eeconf_get(eegren_uart0, eedten_uart_address, &eedword) == LE_OK) {
		if (uartsettvalidate(atmapen_devaddr, eedword) == LE_OK)
			boardio.uartee.devaddr = eedword;
		else
			reqeeupd = 1;
	}

	if (eeconf_get(eegren_uart0, eedten_uart_iface, &eedword) == LE_OK)
		boardio.uartee.uartif = eedword;
#endif


	if ((!baudrate) || (boardio.rflags & BOARDRF_DEFCONF))
		baudrateconv(defaultbr, &baudrate);	// Set default baudrate


	if (boardio.rflags & BOARDRF_DEFCONF) {
		// We are not checking or adjusting EEPROM values if DEFCONF switch is activated
		// just making sure actual t35 and timeout values are not to small
		chanptr->chtxdelay = 0;
		t35 = (MODBUS_RXT35CONST / baudrate);
		if (t35 < MIN_T35) 					// Ensure actual t35 is at least 1msec
			t35 = MIN_T35;
		//if (boardio.uartee.t35 < MIN_T35) 	// Ensure EE saved t35 is at least 1msec
		//	boardio.uartee.t35 = MIN_T35;
		if (chanptr->chtimeout < t35) 		// Ensure actual timeout is at least t35
			chanptr->chtimeout = t35;
	}
	else {
		if (boardio.uartee.t35 < MIN_T35) {	// Ensure t35 is at least 1msec
			boardio.uartee.t35 = MIN_T35;
			reqeeupd = 1;
		}
		if (boardio.uartee.t35 < (MODBUS_RXT35CONST / baudrate)) {	// Ensure t35 is at least 35 bits long
			boardio.uartee.t35 = (MODBUS_RXT35CONST / baudrate);
			reqeeupd = 1;
		}
		t35 = boardio.uartee.t35;

		memcpy(&chanptr->chtimeout, &boardio.uartee.timeoutl, sizeof(atuartto_t));
		if (chanptr->chtimeout < t35) {		// Ensure timeout is at least t35
			chanptr->chtimeout = t35;
			boardio.uartee.timeoutl = t35;
			boardio.uartee.timeouth = 0;
			reqeeupd = 1;
		}

		parity = boardio.uartee.parity;
		memcpy(&chanptr->chtxdelay, &boardio.uartee.txdelayl, sizeof(atuartto_t));
		devaddr = boardio.uartee.devaddr;
		uartif = boardio.uartee.uartif;
	}


	if (reqeeupd)
		boardio.eesize = 0;


	usart_init(&chanptr->usart, baudrate, parity, uartif);	// Initialize UART
	gprot = Modbussl_create(staptr, chanptr);
	Modbussl_postinit(gprot, boardio.mapsize);
	staptr->address = devaddr;
	((Modbus_shlink_t *) staptr->stacoms->priv)->rxtimeout35 = t35;
	((Modbus_shlink_t *) staptr->stacoms->priv)->fformat = ModbusRTU;
}


/*
 * Get name of the current hardware
 * [24/02/2015]
 * Table moved to program space
 * [15/06/2015]
 * MX board flag created
 * [10/09/2016]
 */
uint8_t gethwname(lechar *namebuf) {
	uint8_t					length, tabcnt;
	athw_e					hwtype;
	leptr					lpmptr = (leptr) athwundefinedc;


	namebuf[0] = 0;

	for (tabcnt = 0; tabcnt < (ARRAY_SIZE(hwnametable)); tabcnt++) {
		hwtype = pgm_read_byte(&hwnametable[tabcnt].type);			// Read hardware type from program space

		if (hwtype == (MainLeiodc.hw & ATHW_ID_MASK)) {
			if (MainLeiodc.hw & ATHWF_MXBOARD)
				lpmptr = (leptr) athwmxc;
			else
				lpmptr = (leptr) athwatc;
			strcat_P(namebuf, (const prog_char *) lpmptr);			// cat from program space

			lpmptr = pgm_read_word(&hwnametable[tabcnt].sptr);		// Read string pointer from program space
			break;
		}
	}


	strcat_P(namebuf, (const prog_char *) lpmptr);	// cat from program space
	length = strlen(namebuf);
	return length;		// Resulting string length
}


/*
 * Initialize protocol mapping
 * [28/02/2015]
 * UART settings mapped
 * [18/08/2015]
 * UART interface type added
 * [08/09/2016]
 * Analog inputs added
 * [03/06/2018]
 */
uint8_t mappinginit(Modreg16_t reg, leptr *rdptr, leptr *wrptr) {


	switch (reg) {
	case atmapen_fwrev:
		*rdptr = (leptr) &FwRevNumber;
		return LE_OK;

	case atmapen_direg:
		if (boardio.diptr) {
			*rdptr = (leptr) &boardio.diptr->distates;
			//debugreg = irqasmenum;
			//*rdptr = (leptr) &irqasmenum;
			//*rdptr = (leptr) &leddriver.ackflags;
			//*rdptr = (leptr) &PORTE.IN;
			return LE_OK;
		}
		break;

	case atmapen_dimode00:
	case atmapen_dimode01:
	case atmapen_dimode02:
	case atmapen_dimode03:
	case atmapen_dimode04:
	case atmapen_dimode05:
	case atmapen_dimode06:
	case atmapen_dimode07:
	case atmapen_dimode08:
	case atmapen_dimode09:
	case atmapen_dimode0A:
	case atmapen_dimode0B:
	case atmapen_dimode0C:
	case atmapen_dimode0D:
	case atmapen_dimode0E:
	case atmapen_dimode0F:
		if (boardio.diptr) {
			if ((reg - atmapen_dimode00) < boardio.diptr->count) {
				*rdptr = (leptr) &boardio.diptr->mode[reg - atmapen_dimode00];
				*wrptr = *rdptr;
				return LE_OK;
			}
		}
		break;

	case atmapen_dif00:
	case atmapen_dif01:
	case atmapen_dif02:
	case atmapen_dif03:
	case atmapen_dif04:
	case atmapen_dif05:
	case atmapen_dif06:
	case atmapen_dif07:
	case atmapen_dif08:
	case atmapen_dif09:
	case atmapen_dif0A:
	case atmapen_dif0B:
	case atmapen_dif0C:
	case atmapen_dif0D:
	case atmapen_dif0E:
	case atmapen_dif0F:
		if (boardio.diptr) {
			if ((reg - atmapen_dif00) < boardio.diptr->count) {
				*rdptr = (leptr) &boardio.diptr->filterconst[reg - atmapen_dif00];
				*wrptr = *rdptr;
				return LE_OK;
			}
		}
		break;

	case atmapen_airegi00:
	case atmapen_airegi01:
	case atmapen_airegi02:
	case atmapen_airegi03:
	case atmapen_airegi04:
	case atmapen_airegi05:
	case atmapen_airegi06:
	case atmapen_airegi07:
		if (boardio.aiptr) {
			if ((reg - atmapen_airegi00) < boardio.aiptr->count) {
				*rdptr = (leptr) &boardio.aiptr->uval[reg - atmapen_airegi00];
				return LE_OK;
			}
		}
		break;

	case atmapen_aimode00:
	case atmapen_aimode01:
	case atmapen_aimode02:
	case atmapen_aimode03:
	case atmapen_aimode04:
	case atmapen_aimode05:
	case atmapen_aimode06:
	case atmapen_aimode07:
		if (boardio.aiptr) {
			if ((reg - atmapen_aimode00) < boardio.aiptr->count) {
				*rdptr = (leptr) &boardio.aiptr->mode[reg - atmapen_aimode00];
				*wrptr = *rdptr;
				return LE_OK;
			}
		}
		break;

	case atmapen_doreg:
		if (boardio.doptr) {
			*rdptr = (leptr) &boardio.doptr->dostates;
			*wrptr = (leptr) &boardio.doptr->updatereg;
			return LE_OK;
		}
		break;

	case atmapen_dopul00:
	case atmapen_dopul01:
	case atmapen_dopul02:
	case atmapen_dopul03:
	case atmapen_dopul04:
	case atmapen_dopul05:
	case atmapen_dopul06:
	case atmapen_dopul07:
	case atmapen_dopul08:
	case atmapen_dopul09:
	case atmapen_dopul0A:
	case atmapen_dopul0B:
	case atmapen_dopul0C:
	case atmapen_dopul0D:
	case atmapen_dopul0E:
	case atmapen_dopul0F:
		if (boardio.doptr) {
			if ((reg - atmapen_dopul00) < boardio.doptr->count) {
				*rdptr = (leptr) &boardio.doptr->pulsedur[reg - atmapen_dopul00];
				*wrptr = *rdptr;
				return LE_OK;
			}
		}
		break;

	case atmapen_domode00:
	case atmapen_domode01:
	case atmapen_domode02:
	case atmapen_domode03:
	case atmapen_domode04:
	case atmapen_domode05:
	case atmapen_domode06:
	case atmapen_domode07:
	case atmapen_domode08:
	case atmapen_domode09:
	case atmapen_domode0A:
	case atmapen_domode0B:
	case atmapen_domode0C:
	case atmapen_domode0D:
	case atmapen_domode0E:
	case atmapen_domode0F:
		if (boardio.doptr) {
			if ((reg - atmapen_domode00) < boardio.doptr->count) {
				*rdptr = (leptr) &boardio.doptr->mode[reg - atmapen_domode00];
				*wrptr = *rdptr;
				return LE_OK;
			}
		}
		break;

	case atmapen_baudrate:
		*rdptr = (leptr) &boardio.uartee.bren16;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_parity:
		*rdptr = (leptr) &boardio.uartee.parity;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_txdelayh:
		*rdptr = (leptr) &boardio.uartee.txdelayh;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_txdelayl:
		*rdptr = (leptr) &boardio.uartee.txdelayl;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_timeouth:
		*rdptr = (leptr) &boardio.uartee.timeouth;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_timeoutl:
		*rdptr = (leptr) &boardio.uartee.timeoutl;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_t35:
		*rdptr = (leptr) &boardio.uartee.t35;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_devaddr:
		*rdptr = (leptr) &boardio.uartee.devaddr;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_uartif:
		*rdptr = (leptr) &boardio.uartee.uartif;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_temperature:
		*rdptr = (leptr) &boardio.tempscaled;
		return LE_OK;

	case atmapen_tempraw:
		*rdptr = (leptr) &TEMP_CHAN.RES;
		return LE_OK;

	case atmapen_tempcal85:
		*rdptr = (leptr) &boardio.caltemp85;
		return LE_OK;

	case atmapen_vddio:
		*rdptr = (leptr) &BOARD_VDDIOCH.RES;
		return LE_OK;

	case atmapen_3v2th:
		*rdptr = (leptr) &MXpower.cfg.thadc3v2;
		*wrptr = *rdptr;
		return LE_OK;

	case atmapen_reset:
		*rdptr = (leptr) &boardio.resetreg;
		*wrptr = *rdptr;
		return LE_OK;

	default:
		break;
	}
	return LE_FAIL;
}


/***************************************************************************
* Disable output pins while rebooting MX and enable when started
* [04/03/2015]
***************************************************************************/
void outputpinctrl(uint8_t disable) {
	channel_t		*chanptr;


	for (chanptr = MainLeiodc.chan; chanptr; chanptr = chanptr->next) {
		if (chanptr->usart.disabletxpin) {
			if (disable)
				chanptr->usart.port->DIRCLR = chanptr->usart.disabletxpin;		// Make pin input
			else
				chanptr->usart.port->DIRSET = chanptr->usart.disabletxpin;		// Make pin output
		}
	}
}


/***************************************************************************
* Set bit in pincontrol structure of a particular pin
* [17/08/2015]
***************************************************************************/
/*void pinctrl_setbit(PORT_t *mcuport, uint8_t pin, uint8_t setbit) {
	uint8_t			cnt;


	for (cnt = 0; cnt < 8; cnt++) {
		if (pin == (1 << cnt)) {
			(&mcuport->PIN0CTRL)[cnt] |= setbit;
			break;
		}
	}
}*/


/*
 * Validate UART setting
 * [19/08/2015]
 * UART interface type added
 * [10/09/2016]
 * Timeout and t35 min value checks added
 * [05/08/2020]
 */
int uartsettvalidate(atmapping_e mapreg, Modreg16_t val) {

	switch (mapreg) {
	case atmapen_baudrate:
		if (
				(val & 0xFF00) ||						// Only lowbyte used
				baudrateconv(val, NULL) == LE_FAIL)		// Validate baudrate
			goto failed;
		break;

	case atmapen_parity:
		switch (val) {
		case 'N':		// 0x4E
		case 'E':		// 0x45
		case 'O':		// 0x4F
		//case 'M':		// 0x4D
		//case 'S':		// 0x53
			break;
		default:
			goto failed;
		}
		break;

	case atmapen_txdelayh:
		if (val > 1)
			goto failed;	// Tx delay must be less than 0x3FFFF (13.1071sec)
		break;

	case atmapen_t35:
	case atmapen_timeoutl:
		if (!val)
			goto failed;	// Timeouts must be > 0
		break;

	case atmapen_devaddr:
		if ((val < 1) || (val > 254))
			goto failed;
		break;

	case atmapen_uartif:
		if (val > RS422)
			goto failed;
		break;

	default:	// Don't validate undefined settings, assume OK
		break;
	}
	return LE_OK;	// Setting is valid

	failed:
	return LE_FAIL;	// Setting is invalid
}


/***************************************************************************
* Validate data before updating EEPROM configuration
* [18/08/2015]
***************************************************************************/
/*uint8_t updatecfg_validate(atmapping_e mapreg, ModData16bitDef val) {
	uint8_t			cnt;
	ModReg16bitDef	tabreg;
	ModData16bitDef	lowlimit, highlimit;


	for (cnt = 0; cnt < (ARRAY_SIZE(regvalidtable)); cnt++) {
		tabreg = pgm_read_word(&regvalidtable[cnt].mapreg);				// Read register from program space
		if (mapreg == tabreg) {
			lowlimit = pgm_read_word(&regvalidtable[cnt].lowlimit);		// Read low limit from program space
			highlimit = pgm_read_word(&regvalidtable[cnt].highlimit);	// Read high limit from program space
			if ((val >= lowlimit) && (val <= highlimit))
				return LE_OK;
			else
				return LE_FAIL;
		}
	}
	return LE_OK;	// Assume data is valid if not found in the table
}*/


/***************************************************************************
* Validate received write register value
* [10/09/2016]
***************************************************************************/
uint8_t writevalidate(atmapping_e mapreg, Modreg16_t val, uint8_t *eeupd) {
	mcueegrp_e		groupid;
	uint8_t			valid;


	if ((groupid = eedata_validate(mapreg, val, &valid))) {
		if (valid) {
			if (eegroup_validate(groupid) == LE_OK) {
				*eeupd = 1;		// Request to update EEPROM
				return LE_OK;
			}
		}
	}
	else {	// Received register is not part of EEPROM configuration
		switch (mapreg) {
		case atmapen_doreg:
			return LE_OK;	// Can write any value to DO control register

		case atmapen_reset:
			if (val == RESETREG_RESET) {
				//CCP = CCP_IOREG_gc;
				//RST.CTRL = RST_SWRST_bm;
				asm volatile("cli\n\t"				// disable interrupts
						"ldi r24, 0xD8\n\t"	// value to write to CCP
						"ldi r25, 0x01\n\t"	// value to write to SWRST
						"ldi r30, 0x78\n\t"	// base address of RST peripheral
						"ldi r31, 0\n\t"
						"out __CCP__, r24\n\t"
						"std Z+1, r25\n\t"	// +1 is the offset of RST.CTRL
						::);					// no clobber list because we don't return
			}
			break;

		default:	// Write attempted to unknown register
			break;
		}
	}
	return LE_FAIL;
}


/*
 * Main function
 * [24/02/2015]
 * LED driver main process moved to board.c
 * [24/08/2015]
 */
int main(void) {

	board_init();
	timer_init();
	powman_init();
	comms_init();


	if (!boardio.eesize) {
			boardio.eeupdatebs |=
				(1 << eegren_board) |
				(1 << eegren_powman) |
				(1 << eegren_uart0);

#ifdef IGNORE_EE_CRC_ERROR
		if (!(boardio.rflags & BOARDRF_EECONF_CORRUPTED))
#endif
			eeconf_rebuild();
	}
	else {
		 eeconf_copy(1);
	}


#ifdef GLOBAL_DEBUG
	//uint8_t debugint = 0;
	//boardio.caltemp85 = sizeof(atbaudrate_e);
#endif


	while(1) {
		//wdt_reset();
		if (powman_mainproc() == LE_OK) {		// Power manager processor, main function
			board_mainproc();
			protocol_rxproc();
			station_main();
#ifdef GLOBAL_DEBUG
			/*if (!debugint) {
				debugint = 1;
				uint8_t temptest[3];
				//temptest[0] = 'Q';
				//temptest[1] = FineTimer.sec;
				//channeltx(Station0ptr, temptest, 2);
				temptest[0] = 0xA5;
				temptest[1] = 0x5A;
				leds_update(temptest);
			}*/
#endif
		}

	}
	//return 0;
}
