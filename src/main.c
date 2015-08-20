/*
 ============================================================================
 Name        : main.c
 Author      : AK
 Version     : V1.02
 Copyright   : Property of Londelec UK Ltd
 Description : LEIODC MCU main module

  Change log  :

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
#define	FWVERSION_MINOR			3			// Firmware version number minor
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


// Board hardware type
athwenum				BoardHardware = athwenundefined;
const uint16_t			FwRevNumber = (FWVERSION_MAJOR * 100) + FWVERSION_MINOR;


// Base pointers
ChannelStr				*Channel0Ptr = NULL;
StatStr					*Station0ptr = NULL;
GenProtocolStr			*Gprotocol0ptr = NULL;


const lechar athwenundefinedc[] PROGMEM = "Unknown";
const lechar athwenmx3100v11c[] PROGMEM = "LEIODC-MX-3100";
const lechar athwenat3100v11c[] PROGMEM = "LEIODC-AT-3100";
const hardwarenamestr hwnametable[] PROGMEM = {
	{athwenundefined, 		(leptr) athwenundefinedc},
	{athwenmx3100v11,	 	(leptr) athwenmx3100v11c},
	{athwenat3100v11,	 	(leptr) athwenat3100v11c},
};


//const regvalidtablestr regvalidtable[] PROGMEM = {
//	{atmapen_devaddr, 		1,						254},
//};


const UARTBaudrateStr UARTBaudrateTable[] PROGMEM =
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



#ifdef GLOBAL_DEBUG
//uint16_t debugreg;
#endif


/***************************************************************************
* Main function
* [24/02/2015]
***************************************************************************/
int main(void) {


	board_init();
	timer_init();
	powman_init();
	comms_init();
	if (boardio.eeupdatebs) eeconf_restructure();
#ifdef GLOBAL_DEBUG
	//uint8_t debugint = 0;
#endif


	while(1) {
		//wdt_reset();
		if (powman_mainproc() == EXIT_SUCCESS) {		// Power manager processor, main function
			board_mainproc();
			leddrv_mainproc();
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


/***************************************************************************
* Initialize communication interfaces
* [17/02/2015]
* UART setting update functionality introduced
* t35 timeout automatic calculation added
* Default configuration flag created
* [18/08/2015]
***************************************************************************/
void comms_init() {
	ChannelStr			*chanptr;
	StatStr				*staptr;
	GenProtocolStr		*genprotocol;
	uint32_t			eedword;
	atbaudratedef		baudrate = 0;
	atparitydef			parity;
	atbaudrateenum		defaultbr;
	uint8_t				devaddr;
	uint16_t			t35;
	uint8_t				reqeeupdate = 0;


	chanptr = channelinit();		// Initialize new channel
	staptr = stationinit();			// Initialize new station
	genprotocol = genprotinit();	// Initialize new generic protocol
	staptr->channelinst = chanptr;	// Link station to channel
	genprotocol->statptr = staptr;	// Link general protocol to station
	chanptr->serialsta = staptr;	// Select serial station
	boardio.uartee.parity = DEFAULT_PARITY;	// Default parity
	*((atuarttodef *) &boardio.uartee.timeoutl) = DEFAULT_TIMEOUT;	// Default value 5sec
	boardio.uartee.t35 = 0;			// Undefined, default value will be used
	boardio.uartee.devaddr = 1;		// Default address


	switch (BoardHardware) {
	/*case somerevision:
		break;*/
	case athwenat3100v11:
		defaultbr = atbr9600;		// Default baudrate
		break;
	case athwenmx3100v11:
	default:
		defaultbr = atbr115200;		// Default baudrate
		break;
	}
	boardio.uartee.brenum = defaultbr;


#ifdef HARDCODED_UART_SETTINGS
	boardio.eerdmask |= (1 << eegren_uart0);
#endif


	if (eeconf_get(eegren_uart0, eedten_uart_baudrate, &eedword) == EXIT_SUCCESS) {
		if (baudrateconv(eedword, &baudrate) == EXIT_SUCCESS)
			boardio.uartee.brenum = eedword;
		else reqeeupdate = 1;
	}
	else reqeeupdate = 1;

	if (eeconf_get(eegren_uart0, eedten_uart_parity, &eedword) == EXIT_SUCCESS) {
		if (uartsettvalidate(atmapen_parity, eedword) == EXIT_SUCCESS)
			boardio.uartee.parity = eedword;
		else reqeeupdate = 1;
	}
	else reqeeupdate = 1;

	if (eeconf_get(eegren_uart0, eedten_uart_txdelay, &eedword) == EXIT_SUCCESS)
		*((atuarttodef *) &boardio.uartee.txdelayl) = eedword;
	else reqeeupdate = 1;

	if (eeconf_get(eegren_uart0, eedten_uart_timeout, &eedword) == EXIT_SUCCESS)
		*((atuarttodef *) &boardio.uartee.timeoutl) = eedword;
	else reqeeupdate = 1;

	if (eeconf_get(eegren_uart0, eedten_uart_t35, &eedword) == EXIT_SUCCESS) {
		boardio.uartee.t35 = eedword;
	}
	else reqeeupdate = 1;

	if (eeconf_get(eegren_uart0, eedten_uart_address, &eedword) == EXIT_SUCCESS) {
		if (uartsettvalidate(atmapen_devaddr, eedword) == EXIT_SUCCESS)
			boardio.uartee.devaddr = eedword;
		else reqeeupdate = 1;
	}
	else reqeeupdate = 1;


	//if (baudrate > 19200) {
	//	chanptr-> tperiod = 35 * USART_FRAME35_TIMER_50US_TICKS;	/* ~1750us. */
	//}
	//else {
	//	tperiod = (( 7UL * 220000 ) / ( 2UL * baudrate )) * USART_FRAME35_TIMER_50US_TICKS;
	//}


	if ((!baudrate) || (boardio.rflags & BOARDRF_DEFCONF)) {
		baudrateconv(defaultbr, &baudrate);			// Set default baudrate
	}


	if (boardio.rflags & BOARDRF_DEFCONF) {
		parity = DEFAULT_PARITY;
		chanptr->chtimeout = DEFAULT_TIMEOUT;
		chanptr->chtxdelay = 0;
		t35 = (350000 / baudrate);
		devaddr = 1;
	}
	else {
		if (boardio.uartee.t35 < (350000 / baudrate)) {	// Ensure t35 is at least 35 bits long
			boardio.uartee.t35 = (350000 / baudrate);
			reqeeupdate = 1;
		}
		t35 = boardio.uartee.t35;
		parity = boardio.uartee.parity;
		chanptr->chtimeout = *((atuarttodef *) &boardio.uartee.timeoutl);
		chanptr->chtxdelay = *((atuarttodef *) &boardio.uartee.txdelayl);
		devaddr = boardio.uartee.devaddr;
	}


	if (reqeeupdate)		// EEPROM update required
		boardio.eeupdatebs |= (1 << eegren_uart0);


	usart_init(&chanptr->usart, baudrate, parity);	// Initialize UART
	Modbussl_preinit(genprotocol, devaddr);
	Modbussl_postinit(genprotocol, t35, boardio.mapsize);
}


/***************************************************************************
* Protocol receive process
* [20/02/2015]
***************************************************************************/
void protocolrxproc() {
	ChannelStr			*chanptr;
	CHStateEnum			chstate = CHCommsEmpty;		// Return state of functions
	CHStateEnum			(*procfunc)(DRVARGDEF_RX);


	for (chanptr = Channel0Ptr; chanptr; chanptr = chanptr->next) {
		if (checkrx(&chanptr->usart) == EXIT_SUCCESS) {
			if (chanptr->serialsta->func_rx) {
				procfunc = chanptr->serialsta->func_rx;
				chstate = procfunc(chanptr->serialsta);
			}
			//else flushrx(chanptr);
			POWMAN_RSTIDLE		// Reset MX idle counter
		}
	}
}


/***************************************************************************
* Protocol main process
* [20/02/2015]
***************************************************************************/
void protocolmainproc() {
	StatStr				*staptr;
	uint8_t				*txbuffer;			// Buffer pointer will be initialized by outgoing message prepare functions
	TxRx16bitDef		txlength = 0;
	CHStateEnum			chstate = CHCommsEmpty;			// Return state of functions
	CHStateEnum			(*procfunc)(DRVARGDEF_MAINPROC);


	for (staptr = Station0ptr; staptr; staptr = staptr->next) {
		if (staptr->func_mainproc) {		// New main processing concept
			procfunc = staptr->func_mainproc;
			chstate = procfunc(staptr, &txbuffer, &txlength);
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
* Initialize channels
* [19/02/2015]
***************************************************************************/
ChannelStr *channelinit() {
	ChannelStr		*chanptr, *lastchan = Channel0Ptr;


	for (chanptr = Channel0Ptr; chanptr; chanptr = chanptr->next) {
		if (chanptr->next) lastchan = chanptr->next;
	}
	chanptr = calloc(1, sizeof(ChannelStr));
	if (Channel0Ptr) lastchan->next = chanptr;
	else Channel0Ptr = chanptr;
	return chanptr;
}


/***************************************************************************
* Initialize stations
* [19/02/2015]
***************************************************************************/
StatStr *stationinit() {
	StatStr			*staptr, *laststa = Station0ptr;


	for (staptr = Station0ptr; staptr; staptr = staptr->next) {
		if (staptr->next) laststa = staptr->next;
	}
	staptr = calloc(1, sizeof(StatStr));
	if (Station0ptr) laststa->next = staptr;
	else Station0ptr = staptr;
	return staptr;
}


/***************************************************************************
* Initialize generic protocols
* [19/02/2015]
***************************************************************************/
GenProtocolStr *genprotinit() {
	GenProtocolStr		*genprotocol, *lastgenprotocol = Gprotocol0ptr;


	for (genprotocol = Gprotocol0ptr; genprotocol; genprotocol = genprotocol->next) {
		if (genprotocol->next) lastgenprotocol = genprotocol->next;
	}
	genprotocol = calloc(1, sizeof(GenProtocolStr));
	if (Gprotocol0ptr) lastgenprotocol->next = genprotocol;
	else Gprotocol0ptr = genprotocol;
	return genprotocol;
}


/***************************************************************************
* Get name of the current hardware
* [24/02/2015]
* Table moved to program space
* [15/06/2015]
***************************************************************************/
uint8_t gethwname(lechar *hwnamebuff, uint8_t bufflen) {
	uint8_t					cnt, tabcnt;
	athwenum				hwtype;
	leptr					lpmptr;


	for (tabcnt = 0; tabcnt < (ARRAY_SIZE(hwnametable)); tabcnt++) {
		hwtype = pgm_read_byte(&hwnametable[tabcnt].type);			// Read hardware type from program space
		if (hwtype == BoardHardware) {
			lpmptr = pgm_read_word(&hwnametable[tabcnt].strptr);	// Read string pointer from program space
			for (cnt = 0; cnt < bufflen; cnt++) {
				hwnamebuff[cnt] = pgm_read_byte(lpmptr + cnt);		// Read bytes of the hardware string
				if (!hwnamebuff[cnt]) return cnt;
			}
		}
	}
	return 0;		// String not found in the table
}


/***************************************************************************
* Initialize protocol mapping
* [28/02/2015]
* UART settings mapped
* [18/08/2015]
***************************************************************************/
uint8_t mappinginit(ModReg16bitDef reg, leptr *rdptr, leptr *wrptr) {


	switch (reg) {
	case atmapen_fwrev:
		*rdptr = (leptr) &FwRevNumber;
		return EXIT_SUCCESS;

	case atmapen_direg:
		if (boardio.diptr) {
			*rdptr = (leptr) &boardio.diptr->distates;
			//debugreg = irqasmenum;
			//*rdptr = (leptr) &irqasmenum;
			//*rdptr = (leptr) &leddriver.ackflags;
			//*rdptr = (leptr) &PORTE.IN;
			return EXIT_SUCCESS;
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
				return EXIT_SUCCESS;
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
				return EXIT_SUCCESS;
			}
		}
		break;

	case atmapen_doreg:
		if (boardio.doptr) {
			*rdptr = (leptr) &boardio.doptr->dostates;
			*wrptr = (leptr) &boardio.doptr->updatereg;
			return EXIT_SUCCESS;
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
				return EXIT_SUCCESS;
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
				return EXIT_SUCCESS;
			}
		}
		break;

	case atmapen_baudrate:
		*rdptr = (leptr) &boardio.uartee.brenum;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	case atmapen_parity:
		*rdptr = (leptr) &boardio.uartee.parity;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	case atmapen_txdelayh:
		*rdptr = (leptr) &boardio.uartee.txdelayh;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	case atmapen_txdelayl:
		*rdptr = (leptr) &boardio.uartee.txdelayl;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	case atmapen_timeouth:
		*rdptr = (leptr) &boardio.uartee.timeouth;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	case atmapen_timeoutl:
		*rdptr = (leptr) &boardio.uartee.timeoutl;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	case atmapen_t35:
		*rdptr = (leptr) &boardio.uartee.t35;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	case atmapen_devaddr:
		*rdptr = (leptr) &boardio.uartee.devaddr;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	case atmapen_temperature:
		*rdptr = (leptr) &tempscaled;
		return EXIT_SUCCESS;

	case atmapen_tempraw:
		*rdptr = (leptr) &TEMP_CHAN.RES;
		return EXIT_SUCCESS;

	case atmapen_tempcal85:
		*rdptr = (leptr) &caltemp85;
		return EXIT_SUCCESS;

	case atmapen_vddio:
		*rdptr = (leptr) &BOARD_VDDIOCH.RES;
		return EXIT_SUCCESS;

	case atmapen_3v2th:
		*rdptr = (leptr) &MXpower.cfg.thadc3v2;
		*wrptr = *rdptr;
		return EXIT_SUCCESS;

	default:
		break;
	}
	return EXIT_FAILURE;
}


/***************************************************************************
* Disable output pins while rebooting MX and enable when started
* [04/03/2015]
***************************************************************************/
void outputpinctrl(uint8_t disable) {
	ChannelStr		*chanptr;


	for (chanptr = Channel0Ptr; chanptr; chanptr = chanptr->next) {
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
* Convert between baudrate enum and decimal value or vice versa
* [19/08/2015]
***************************************************************************/
uint8_t baudrateconv(uint16_t brenum16, atbaudratedef *baudrate) {
	uint8_t			cnt;
	atbaudrateenum	tabenum;
	atbaudratedef	tabbrval;


	for (cnt = 0; cnt < (ARRAY_SIZE(UARTBaudrateTable)); cnt++) {
		tabenum = pgm_read_byte(&UARTBaudrateTable[cnt].brenum);		// Read baudrate enum from program space
		tabbrval = pgm_read_dword(&UARTBaudrateTable[cnt].baudrate);	// Read baudrate value from program space
		if (tabenum == brenum16) {
			if (baudrate) *baudrate = tabbrval;
			return EXIT_SUCCESS;
		}
		//else if (baudrate && *baudrate && (tabbrval == *baudrate)) {
		//	if (brenum16) *brenum16 = tabenum;
		//	return EXIT_SUCCESS;
		//}
	}
	return EXIT_FAILURE;	// Baudrate not found
}


/***************************************************************************
* Validate UART setting
* [19/08/2015]
***************************************************************************/
uint8_t uartsettvalidate(atmappingenum mapreg, ModData16bitDef val) {

	switch (mapreg) {
	case atmapen_baudrate:
		if (baudrateconv(val, NULL) == EXIT_FAILURE) goto failed;
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
		if (val > 1) goto failed;	// Tx delay must be less than 0x3FFFF (13.1071sec)
		break;

	case atmapen_devaddr:
		if ((val < 1) || (val > 254)) goto failed;
		break;

	default:	// Don't validate undefined settings, assume OK
		break;
	}
	return EXIT_SUCCESS;	// Setting is valid

	failed:
	return EXIT_FAILURE;	// Setting is invalid
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
				return EXIT_SUCCESS;
			else
				return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;	// Assume data is valid if not found in the table
}*/
