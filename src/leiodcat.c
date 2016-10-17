/*
 ============================================================================
 Name        : leiodcat.c
 Author      : AK
 Version     : V2.00
 Copyright   : Property of Londelec UK Ltd
 Description : LEIODC MCU main module

  Change log :

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
#include "modbussl.h"
#include "powman.h"
#include "timer.h"
#include "usart.h"
#include "74lv8153.h"


#ifdef EEPROM_CFG
#include <avr/eeprom.h>
#include "mcueecfg.h"
#include "ateecfg.inc"
#endif


#define	FWVERSION_MAJOR			1			// Firmware version number major
#define	FWVERSION_MINOR			5			// Firmware version number minor
#if FWVERSION_MINOR < 10
#define	FWVERSION_10TH_ZERO		"0"
#else
#define	FWVERSION_10TH_ZERO		""
#endif
const lechar FirmwareVersion[] PROGMEM = " FirmwareVersion="\
								STRINGIFY(FWVERSION_MAJOR)\
								"."\
								FWVERSION_10TH_ZERO\
								STRINGIFY(FWVERSION_MINOR)\
								" ";
#include "builddate.txt"


//#define	HARDCODED_UART_SETTINGS				// Don't load UART setting from EEPROM


// UART default settings
#define DEFAULT_PARITY					'E'				// Default parity
#define DEFAULT_TIMEOUT					50000			// UART Timeout (x100usec), default 5sec


// Board hardware type
static const uint16_t FwRevNumber = (FWVERSION_MAJOR * 100) + FWVERSION_MINOR;


// Global variables
mainlStr MainLeiodc = {
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
static const struct hardwarenamestr__  {
	athwenum				type;
	leptr					sptr;
} hwnametable[] PROGMEM = {
	{athwenmx3100v11,	 	(leptr) athw3100c},
	{athwenat3100v11,	 	(leptr) athw3100c},
	{athwenat2200v10,	 	(leptr) athw2200c},
	{athwenat4000v10,	 	(leptr) athw4000c},
	{athwenat0400v10,	 	(leptr) athw0400c},
};


//const regvalidtablestr regvalidtable[] PROGMEM = {
//	{atmapen_devaddr, 		1,						254},
//};


typedef struct UARTBaudrateStr_ {
	atbaudrateenum			brenum;
	atbaudratedef 			baudrate;
} UARTBaudrateStr;

static const UARTBaudrateStr UARTBaudrateTable[] PROGMEM =
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




/***************************************************************************
* Initialize channels
* [19/02/2015]
***************************************************************************/
ChannelStr *channelinit(void) {
	ChannelStr		*chanptr, *lastchan = MainLeiodc.chan;


	for (chanptr = MainLeiodc.chan; chanptr; chanptr = chanptr->next) {
		if (chanptr->next)
			lastchan = chanptr->next;
	}
	chanptr = calloc(1, sizeof(ChannelStr));

	if (MainLeiodc.chan)
		lastchan->next = chanptr;
	else
		MainLeiodc.chan = chanptr;
	return chanptr;
}


/***************************************************************************
* Initialize stations
* [19/02/2015]
***************************************************************************/
static StatStr *stationinit(void) {
	StatStr			*staptr, *laststa = MainLeiodc.sta;


	for (staptr = MainLeiodc.sta; staptr; staptr = staptr->next) {
		if (staptr->next)
			laststa = staptr->next;
	}
	staptr = calloc(1, sizeof(StatStr));

	if (MainLeiodc.sta)
		laststa->next = staptr;
	else
		MainLeiodc.sta = staptr;
	return staptr;
}


/***************************************************************************
* Initialize generic protocols
* [19/02/2015]
***************************************************************************/
static GenProtocolStr *genprotinit(void) {
	GenProtocolStr		*gprot, *lastgp = MainLeiodc.gp;


	for (gprot = MainLeiodc.gp; gprot; gprot = gprot->next) {
		if (gprot->next)
			lastgp = gprot->next;
	}
	gprot = calloc(1, sizeof(GenProtocolStr));

	if (MainLeiodc.gp)
		lastgp->next = gprot;
	else
		MainLeiodc.gp = gprot;
	return gprot;
}


/***************************************************************************
* Convert between baudrate enum and decimal value or vice versa
* [19/08/2015]
* Program space memcpy function used
* [10/09/2016]
***************************************************************************/
static uint8_t baudrateconv(uint8_t brenum, atbaudratedef *baudrate) {
	uint8_t			cnt;
	UARTBaudrateStr	tabval;


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


/***************************************************************************
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
***************************************************************************/
static void comms_init(void) {
	ChannelStr			*chanptr;
	StatStr				*staptr;
	GenProtocolStr		*gprot;
	uint32_t			eedword;
	atbaudratedef		baudrate = 0;
	atparitydef			parity = DEFAULT_PARITY;
	atbaudrateenum		defaultbr;
	uint8_t				devaddr = 1;
	uint16_t			t35;
	UartIntEnum			uartif = RS485;
	uint8_t				reqeeupdate = 0;


	chanptr = channelinit();		// Initialize new channel
	staptr = stationinit();			// Initialize new station
	gprot = genprotinit();			// Initialize new generic protocol
	staptr->channelinst = chanptr;	// Link station to channel
	gprot->statptr = staptr;		// Link general protocol to station
	chanptr->serialsta = staptr;	// Select serial station
	boardio.uartee.parity = DEFAULT_PARITY;	// Default parity
	*((atuarttodef *) &boardio.uartee.timeoutl) = DEFAULT_TIMEOUT;	// Default value 5sec
	boardio.uartee.t35 = 0;			// Undefined, default value will be used
	boardio.uartee.devaddr = 1;		// Default address


	/*switch (MainLeiodc.hw) {
	case somerevision:
		break;
	case athwenat3100v11:
		defaultbr = atbr9600;		// Default baudrate
		break;
	case athwenmx3100v11:
	default:
		defaultbr = atbr115200;		// Default baudrate
		break;
	}*/
	if (MainLeiodc.hw & ATHWF_MXBOARD)
		defaultbr = atbr115200;		// Default baudrate
	else
		defaultbr = atbr9600;		// Default baudrate
	boardio.uartee.bren16 = defaultbr;


#ifdef HARDCODED_UART_SETTINGS
	boardio.eerdmask |= (1 << eegren_uart0);
#endif


	if (eeconf_get(eegren_uart0, eedten_uart_baudrate, &eedword, &reqeeupdate) == LE_OK) {
		if (baudrateconv(eedword, &baudrate) == LE_OK)
			boardio.uartee.bren16 = eedword;
		else
			reqeeupdate = 1;
	}

	if (eeconf_get(eegren_uart0, eedten_uart_parity, &eedword, &reqeeupdate) == LE_OK) {
		if (uartsettvalidate(atmapen_parity, eedword) == LE_OK)
			boardio.uartee.parity = eedword;
		else
			reqeeupdate = 1;
	}

	if (eeconf_get(eegren_uart0, eedten_uart_txdelay, &eedword, &reqeeupdate) == LE_OK)
		*((atuarttodef *) &boardio.uartee.txdelayl) = eedword;

	if (eeconf_get(eegren_uart0, eedten_uart_timeout, &eedword, &reqeeupdate) == LE_OK)
		*((atuarttodef *) &boardio.uartee.timeoutl) = eedword;

	if (eeconf_get(eegren_uart0, eedten_uart_t35, &eedword, &reqeeupdate) == LE_OK) {
		boardio.uartee.t35 = eedword;
	}

	if (eeconf_get(eegren_uart0, eedten_uart_address, &eedword, &reqeeupdate) == LE_OK) {
		if (uartsettvalidate(atmapen_devaddr, eedword) == LE_OK)
			boardio.uartee.devaddr = eedword;
		else
			reqeeupdate = 1;
	}

	if (eeconf_get(eegren_uart0, eedten_uart_iface, &eedword, &reqeeupdate) == LE_OK) {
		boardio.uartee.uartif = eedword;
	}


	if ((!baudrate) || (boardio.rflags & BOARDRF_DEFCONF)) {
		baudrateconv(defaultbr, &baudrate);	// Set default baudrate
	}


	if (boardio.rflags & BOARDRF_DEFCONF) {
		chanptr->chtimeout = DEFAULT_TIMEOUT;
		chanptr->chtxdelay = 0;
		t35 = (MODBUS_RXT35CONST / baudrate);
		if (boardio.uartee.t35 < 10) {	// Ensure t35 is at least 1msec
			boardio.uartee.t35 = 10;
		}
	}
	else {
		if (boardio.uartee.t35 < 10) {	// Ensure t35 is at least 1msec
			boardio.uartee.t35 = 10;
			reqeeupdate = 1;
		}
		if (boardio.uartee.t35 < (MODBUS_RXT35CONST / baudrate)) {	// Ensure t35 is at least 35 bits long
			boardio.uartee.t35 = (MODBUS_RXT35CONST / baudrate);
			reqeeupdate = 1;
		}
		t35 = boardio.uartee.t35;
		parity = boardio.uartee.parity;
		memcpy(&chanptr->chtimeout, &boardio.uartee.timeoutl, sizeof(atuarttodef));
		memcpy(&chanptr->chtxdelay, &boardio.uartee.txdelayl, sizeof(atuarttodef));
		devaddr = boardio.uartee.devaddr;
		uartif = boardio.uartee.uartif;
	}


	if (reqeeupdate)	// EEPROM update required
		boardio.eeupdatebs |= (1 << eegren_uart0);


	usart_init(&chanptr->usart, baudrate, parity, uartif);	// Initialize UART
	Modbussl_preinit(gprot, devaddr);
	Modbussl_postinit(gprot, t35, boardio.mapsize);
}


/***************************************************************************
* Protocol receive process
* [20/02/2015]
* Rx function called directly
* [07/09/2016]
***************************************************************************/
static void protocolrxproc(void) {
	ChannelStr			*chanptr;
	CHStateEnum			chstate = CHCommsEmpty;		// Return state of functions


	for (chanptr = MainLeiodc.chan; chanptr; chanptr = chanptr->next) {
		if (checkrx(&chanptr->usart) == LE_OK) {
			if (chanptr->serialsta->func_rx) {
				chstate = chanptr->serialsta->func_rx(chanptr->serialsta);
			}
			//else flushrx(chanptr);
			POWMAN_RSTIDLE		// Reset MX idle counter
		}
	}
}


/***************************************************************************
* Protocol main process
* [20/02/2015]
* Main processing function called directly
* [07/09/2016]
***************************************************************************/
static void protocolmainproc(void) {
	StatStr				*staptr;
	uint8_t				*txbuffer;			// Buffer pointer will be initialized by outgoing message prepare functions
	TxRx16bitDef		txlength = 0;
	CHStateEnum			chstate = CHCommsEmpty;			// Return state of functions


	for (staptr = MainLeiodc.sta; staptr; staptr = staptr->next) {
		if (staptr->func_mainproc) {		// New main processing concept
			chstate = staptr->func_mainproc(staptr, &txbuffer, &txlength);
		}


		// Analyze the state process function has returned
		switch (chstate) {
		case CHCommsRxTx:		// Send message
			if (txlength) {
				chstate = channeltx(staptr,	txbuffer, txlength);
			}
			break;


		//case CHCommsDataError:	// Error during main process
		//	Channel_reqclose(staptr, CHCloseDataErr);
		//	break;


		//case CHCommsDisabled:	// Station Disabled by service
		//	Channel_reqclose(staptr, CHCloseDisabled);
		//	break;


		default:
			break;
		}
	}
}


/***************************************************************************
* Get name of the current hardware
* [24/02/2015]
* Table moved to program space
* [15/06/2015]
* MX board flag created
* [10/09/2016]
***************************************************************************/
uint8_t gethwname(lechar *namebuf) {
	uint8_t					length, tabcnt;
	athwenum				hwtype;
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


/***************************************************************************
* Initialize protocol mapping
* [28/02/2015]
* UART settings mapped
* [18/08/2015]
* UART interface type added
* [08/09/2016]
***************************************************************************/
uint8_t mappinginit(ModReg16bitDef reg, leptr *rdptr, leptr *wrptr) {


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
	ChannelStr		*chanptr;


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



/***************************************************************************
* Validate UART setting
* [19/08/2015]
* UART interface type added
* [10/09/2016]
***************************************************************************/
uint8_t uartsettvalidate(atmappingenum mapreg, ModData16bitDef val) {

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
/*uint8_t updatecfg_validate(atmappingenum mapreg, ModData16bitDef val) {
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
uint8_t writevalidate(atmappingenum mapreg, ModData16bitDef val, uint8_t *eeupd) {
	mcueegrpenum	groupid;
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


/***************************************************************************
* Main function
* [24/02/2015]
* LED driver main process moved to board.c
* [24/08/2015]
***************************************************************************/
int main(void) {


	board_init();
	timer_init();
	powman_init();
	comms_init();

	if (boardio.eeupdatebs)
		eeconf_restructure();


#ifdef GLOBAL_DEBUG
	//uint8_t debugint = 0;
	//boardio.caltemp85 = sizeof(atbaudrateenum);
#endif


	while(1) {
		//wdt_reset();
		if (powman_mainproc() == LE_OK) {		// Power manager processor, main function
			board_mainproc();
			protocolrxproc();
			protocolmainproc();
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
