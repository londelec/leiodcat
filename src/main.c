/*
 ============================================================================
 Name        : main.c
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : LEIODC MCU main module

  Change log  :

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
#define	FWVERSION_MINOR			1			// Firmware version number minor
#if FWVERSION_MINOR < 10
#define	FWVERSION_10TH_ZERO		"0"
#else
#define	FWVERSION_10TH_ZERO		""
#endif
const lechar *FirmwareVersion = " FirmwareVersion="\
								STRINGIFY(FWVERSION_MAJOR)\
								"."\
								FWVERSION_10TH_ZERO\
								STRINGIFY(FWVERSION_MINOR)\
								" ";
#include "builddate.txt"


// Board hardware type
athwenum				BoardHardware = athwenundefined;
uint16_t				FwRevNumber = (FWVERSION_MAJOR * 100) + FWVERSION_MINOR;


// Base pointers
ChannelStr				*Channel0Ptr = NULL;
StatStr					*Station0ptr = NULL;
GenProtocolStr			*Gprotocol0ptr = NULL;


const hardwarenamestr hwnametable[] = {
	{"Unknown",			athwenundefined},
	{"LEIODC-MX-3100",	athwenmx3100v11},
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
***************************************************************************/
void comms_init() {
	ChannelStr			*chanptr;
	StatStr				*staptr;
	GenProtocolStr		*genprotocol;
	uint32_t			eedword;
	uint32_t 			baudrate = 115200;
	uint8_t 			parity = 'E';
	uint8_t				devaddr = 0;			// Undefined device address
	TimerConstDef 		t35 = 0;				// Undefined, default value will be used


	if (getee_data(eegren_uart0, eedten_uart_address, &eedword) == EXIT_SUCCESS) {
		devaddr = eedword;
		chanptr = channelinit();		// Initialize new channel
		staptr = stationinit();			// Initialize new station
		genprotocol = genprotinit();	// Initialize new generic protocol
		staptr->channelinst = chanptr;	// Link station to channel
		genprotocol->statptr = staptr;	// Link general protocol to station
		chanptr->serialsta = staptr;	// Select serial station
		chanptr->chtimeout = DEFAULT_TIMEOUT;	// Default value 5sec


		if (getee_data(eegren_uart0, eedten_uart_baudrate, &eedword) == EXIT_SUCCESS)	// Read baudrate
			baudrate = eedword;
		if (getee_data(eegren_uart0, eedten_uart_parity, &eedword) == EXIT_SUCCESS)		// Read parity
			parity = eedword;
		if (getee_data(eegren_uart0, eedten_uart_txdelay, &eedword) == EXIT_SUCCESS)	// Read tx delay
			chanptr->chtxdelay = eedword;
		if (getee_data(eegren_uart0, eedten_uart_timeout, &eedword) == EXIT_SUCCESS)	// Read timeout
			chanptr->chtimeout = eedword;
		if (getee_data(eegren_uart0, eedten_uart_t35, &eedword) == EXIT_SUCCESS)		// Read t35 timeout
			t35 = eedword;

		//if (baudrate > 19200) {
		//	chanptr-> tperiod = 35 * USART_FRAME35_TIMER_50US_TICKS;	/* ~1750us. */
		//}
		//else {
		//	tperiod = (( 7UL * 220000 ) / ( 2UL * baudrate )) * USART_FRAME35_TIMER_50US_TICKS;
		//}

		// TODO can migrate all mapping to EEPROM configuration


		usart_init(&chanptr->usart, baudrate, parity);						// Initialize UART
		Modbussl_preinit(genprotocol, devaddr);
		Modbussl_postinit(genprotocol, t35, boardio.mapsize);
	}
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
	SocketBufDef		txlength = 0;
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
***************************************************************************/
uint8_t gethwname(lechar **stringptr) {
	uint8_t				cnt;


	for (cnt = 0; cnt < (sizeof(hwnametable) / sizeof(hardwarenamestr)); cnt++) {
		if (hwnametable[cnt].type == BoardHardware) {
			*stringptr = hwnametable[cnt].string;
			return strlen(hwnametable[cnt].string);
		}
	}
	*stringptr = NULL;
	return 0;		// String not found in the table
}


/***************************************************************************
* Initialize protocol mapping
* [28/02/2015]
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

	case atmapen_dohld00:
	case atmapen_dohld01:
	case atmapen_dohld02:
	case atmapen_dohld03:
	case atmapen_dohld04:
	case atmapen_dohld05:
	case atmapen_dohld06:
	case atmapen_dohld07:
	case atmapen_dohld08:
	case atmapen_dohld09:
	case atmapen_dohld0A:
	case atmapen_dohld0B:
	case atmapen_dohld0C:
	case atmapen_dohld0D:
	case atmapen_dohld0E:
	case atmapen_dohld0F:
		if (boardio.doptr) {
			if ((reg - atmapen_dohld00) < boardio.doptr->count) {
				*rdptr = (leptr) &boardio.doptr->holdperiod[reg - atmapen_dohld00];
				*wrptr = *rdptr;
				return EXIT_SUCCESS;
			}
		}
		break;

	case atmapen_dopol00:
	case atmapen_dopol01:
	case atmapen_dopol02:
	case atmapen_dopol03:
	case atmapen_dopol04:
	case atmapen_dopol05:
	case atmapen_dopol06:
	case atmapen_dopol07:
	case atmapen_dopol08:
	case atmapen_dopol09:
	case atmapen_dopol0A:
	case atmapen_dopol0B:
	case atmapen_dopol0C:
	case atmapen_dopol0D:
	case atmapen_dopol0E:
	case atmapen_dopol0F:
		if (boardio.doptr) {
			if ((reg - atmapen_dopol00) < boardio.doptr->count) {
				*rdptr = (leptr) &boardio.doptr->policy[reg - atmapen_dopol00];
				*wrptr = *rdptr;
				return EXIT_SUCCESS;
			}
		}
		break;

	case atmapen_temperature:
		*rdptr = (leptr) &BOARD_TEMPCH.RES;
		return EXIT_SUCCESS;

	case atmapen_tempcal85:
		*rdptr = (leptr) &caltemp85;
		return EXIT_SUCCESS;

	case atmapen_vddio:
		*rdptr = (leptr) &BOARD_VDDIOCH.RES;
		return EXIT_SUCCESS;

	case atmapen_3v2th:
		//*rdptr = (leptr) &BOARD_ADC.CAL;
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
