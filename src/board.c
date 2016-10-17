/*
 ============================================================================
 Name        : board.c
 Author      : AK
 Version     : V1.04
 Copyright   : Property of Londelec UK Ltd
 Description : board hardware module

  Change log :

  *********V1.04 07/09/2016**************
  Fixed: Last temperature value used correctly
  Local functions marked static
  New hardwares 4000, 0400, 2200 added
  Don't initialize DI and DO structures if there are no DI/DO objects
  MX board flag created

  *********V1.03 24/08/2015**************
  LED driver main process moved from main.c

  *********V1.02 17/08/2015**************
  New hardware 3100 without MX board
  Board hardware profile table and handling functions created
  Default configuration pin created
  DI and DO mode enums now defined in modbusdef.h

  *********V1.01 12/06/2015**************
  Scaled temperature calculation created
  ADCB is used for temperature measurements
  CRC of EEPROM configuration is being verified on startup
  Configuration update request bit set introduced
  DI modes added

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */

#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "board.h"
#include "leiodcat.h"
#include "timer.h"
#include "74lv8153.h"
#include "mcueecfg.h"


#ifdef GLOBAL_DEBUG
//#define DEBUG_NOEE
//#define DEBUG_NOLEDDRV
//#define DEBUG_LEDUPDATE
#endif	// GLOBAL_DEBUG


// IO definitions
#define DI_DEFAULT_FILTER_MSEC		50 		// default 50ms
#define DO_DEFAULT_HOLD_MSEC		1500 	// default 1.5sec
//#define DO_HOLD_TIME_MIN			500
//#define DO_HOLD_TIME_MAX			5000

#define VPORT_BASE 					VPORT0


boardstr boardio;



// Offset and mask calculation for IO selection
#define	CALCULATE_REGOFFSET(mioptr, mindex) ((mioptr->bitoffset[mindex] & 0x18) >> 3)
#define	CALCULATE_VPORTPTR(mioptr, mindex)	&VPORT_BASE + CALCULATE_REGOFFSET(mioptr, mindex)
#define	CALCULATE_BITMASK(mioptr, mindex)	(1 << (mioptr->bitoffset[mindex] & 0x07))


// PIN manipulation macros
#define LED_CONTROL_PIN_ON	if (boardio.ledoepin) boardio.ctrlport->OUTCLR = boardio.ledoepin;


typedef struct boardhwtabstr_  {
	athwenum				type;
	uint8_t					dicount;
	uint8_t					docount;
	uint8_t					dioffs;
	uint8_t					dooffs;
} boardhwtabstr;

const boardhwtabstr boardhwtable[] PROGMEM = {
	{athwenundefined, 	12, 	4, 		0, 		0x0C},
	{athwenmx3100v11,	12,		4,		0, 		0x0C},
	{athwenat3100v11,	12,		4, 		0, 		0x0C},
	{athwenat2200v10,	8,		8, 		0, 		0x08},
	{athwenat4000v10,	16,		0, 		0, 		0},
	{athwenat0400v10,	0,		16, 	0, 		0},
};




/***************************************************************************
* 1ms ISR for IO processing
* [24/02/2015]
* DI and DO mode enums now defined in modbusdef.h
* [19/08/2015]
***************************************************************************/
inline void boardisr_1ms(void) {
	uint8_t				cnt;
	boardDIstr			*diptr = boardio.diptr;
	boardDOstr			*doptr = boardio.doptr;
	VPORT_t 			*vportptr;
	uint16_t			statebit;
	uint8_t 			dibit;


	if (diptr) {
		statebit = 1;
		for (cnt = 0; cnt < diptr->count; cnt++) {
			vportptr = CALCULATE_VPORTPTR(diptr, cnt);		// Calculate VPORT structure pointer
			dibit = vportptr->IN & CALCULATE_BITMASK(diptr, cnt);	// Calculate bitmask of the particular DI
			if (
					(dibit && (diptr->distates & statebit)) ||
					(!dibit && !(diptr->distates & statebit))) {	// Check bit of the particular DI
				diptr->samples[cnt] = 0;
			}
			else {
				diptr->samples[cnt]++;
				if (diptr->samples[cnt] >= diptr->filterconst[cnt]) {
					if (dibit)
						diptr->distates |= statebit;
					else
						diptr->distates &= ~statebit;
					diptr->samples[cnt] = 0;
					boardio.rflags |= BOARDRF_EVENT | BOARDRF_UPDATE_LED;
				}
			}
			statebit <<= 1;
		}
	}


	if (doptr) {
		for (cnt = 0; cnt < doptr->count; cnt++) {
			switch (doptr->mode[cnt]) {
			case modbusdomden_pulseout:
				if (doptr->dostates & (1 << cnt)) {		// Check bit of the particular DO
					doptr->samples[cnt]++;
				}
				break;

			default:	// Don't check anything if output policy is unknown
				break;
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////
// BOARD IO CHECK interrupt
//////////////////////////////////////////////////////////////////////////
ISR(TIMER1MS_ISR_VECTOR) {
	boardisr_1ms();
}


//void aaaa (void)  __attribute__ ((section (".bootloader")));
//void aaaa ()  {
//}

//DEBUG catch bad interrupt vector
#ifdef GLOBAL_DEBUG
ISR(BADISR_vect) {
	//board.usart.ledPort->OUTCLR |= board.usart.rx_led_bm | board.usart.tx_led_bm | board.usart.er_led_bm;
	//usart_led_timer_enable(board.usart.ledTimer);
}
#endif


/***************************************************************************
* Initialize digital inputs
* [24/02/2015]
* DI modes added
* [12/06/2015]
* DI and DO mode enums now defined in modbusdef.h
* [19/08/2015]
* Don't initialize structures if there are no DIs
* [08/09/2016]
***************************************************************************/
static void initboardDI(uint8_t dicount, uint8_t baseoffset) {
	uint8_t				cnt;
	uint32_t			eedword;
	uint8_t				reqeeupdate = 0;


	if (!dicount)
		return;

	boardio.diptr = calloc(1, sizeof(boardDIstr));
	boardio.diptr->count = dicount;
	boardio.diptr->samples = calloc(dicount, sizeof(uint16_t));
	boardio.diptr->mode = calloc(dicount, sizeof(modbusdimodeenum));
	boardio.diptr->filterconst = calloc(dicount, sizeof(uint16_t));
	boardio.diptr->bitoffset = calloc(dicount, sizeof(uint8_t));
	//boardio.diptr->bitoffset = (uint8_t *) boardio.diptr->filterconst + (dicount * sizeof(uint16_t));


	for (cnt = 0; cnt < dicount; cnt++) {
		boardio.diptr->bitoffset[cnt] = (baseoffset + cnt);

		if (eeconf_get(eegren_board, (eedten_board_dimode00 + cnt), &eedword, &reqeeupdate) == LE_OK) {
			boardio.diptr->mode[cnt] = eedword;
		}
		else {
			boardio.diptr->mode[cnt] = modbusdimden_spi;
		}

		if (eeconf_get(eegren_board, (eedten_board_diflt00 + cnt), &eedword, &reqeeupdate) == LE_OK) {
			boardio.diptr->filterconst[cnt] = eedword;
		}
		else {
			boardio.diptr->filterconst[cnt] = DI_DEFAULT_FILTER_MSEC;
		}
	}

	if (reqeeupdate)	// EEPROM update required
		boardio.eeupdatebs |= (1 << eegren_board);
}


/***************************************************************************
* Initialize digital outputs
* [24/02/2015]
* DI and DO mode enums now defined in modbusdef.h
* [19/08/2015]
* Don't initialize structures if there are no DOs
* [08/09/2016]
***************************************************************************/
static void initboardDO(uint8_t docount, uint8_t baseoffset) {
	uint8_t				cnt;
	uint32_t			eedword;
	uint8_t				reqeeupdate = 0;


	if (!docount)
		return;

	boardio.doptr = calloc(1, sizeof(boardDOstr));
	boardio.doptr->count = docount;
	boardio.doptr->samples = calloc(docount, sizeof(uint16_t));
	boardio.doptr->mode = calloc(docount, sizeof(modbusdomodeenum));
	boardio.doptr->pulsedur = calloc(docount, sizeof(uint16_t));
	boardio.doptr->bitoffset = calloc(docount, sizeof(uint8_t));


	for(cnt = 0; cnt < docount; cnt++) {
		boardio.doptr->bitoffset[cnt] = (baseoffset + cnt);

		if (eeconf_get(eegren_board, (eedten_board_domode00 + cnt), &eedword, &reqeeupdate) == LE_OK) {
			boardio.doptr->mode[cnt] = eedword;
		}
		else {
			boardio.doptr->mode[cnt] = modbusdomden_pulseout;
		}

		if (eeconf_get(eegren_board, (eedten_board_dopul00 + cnt), &eedword, &reqeeupdate) == LE_OK) {
			boardio.doptr->pulsedur[cnt] = eedword;
		}
		else {
			boardio.doptr->pulsedur[cnt] = DO_DEFAULT_HOLD_MSEC;
		}
	}

	if (reqeeupdate)	// EEPROM update required
		boardio.eeupdatebs |= (1 << eegren_board);
}


/***************************************************************************
* Check if any DO is active before activating requested one
* [25/02/2015]
***************************************************************************/
static uint8_t checkotherdoactive(boardDOstr *doptr, uint8_t doindex) {
	uint8_t				cnt;


	for (cnt = 0; cnt < doptr->count; cnt++) {
		if (cnt != doindex) {
			if (doptr->dostates & (1 << cnt)) {		// Check bit of the particular DO
				return LE_FAIL;	// Another DO is already active
			}
		}
	}
	return LE_OK;	// No other DOs are active at the moment
}


/***************************************************************************
* Activate output
* [25/02/2015]
***************************************************************************/
static void activateDO(boardDOstr *doptr, uint8_t doindex) {
	VPORT_t 			*vportptr;


	doptr->samples[doindex] = 0;		// Clear samples BEFORE changing state, due to interrupt awareness
	doptr->dostates |= (1 << doindex);	// Update realtime status

	vportptr = CALCULATE_VPORTPTR(doptr, doindex);		// Calculate VPORT structure pointer
	vportptr->OUT |= CALCULATE_BITMASK(doptr, doindex);	// Set bit of the particular DO
	boardio.rflags |= BOARDRF_UPDATE_LED;
}


/***************************************************************************
* Release output
* [25/02/2015]
***************************************************************************/
static void releaseDO(boardDOstr *doptr, uint8_t doindex) {
	VPORT_t 			*vportptr;


	doptr->dostates &= ~(1 << doindex);	// Update realtime status

	vportptr = CALCULATE_VPORTPTR(doptr, doindex);			// Calculate VPORT structure pointer
	vportptr->OUT &= ~(CALCULATE_BITMASK(doptr, doindex));	// Clear bit of the particular DO
	boardio.rflags |= BOARDRF_UPDATE_LED;
}


/***************************************************************************
* Calculate temperature value
* [14/06/2015]
* Fixed: Last temperature value used correctly
* [09/09/2016]
***************************************************************************/
static void calctemperature(void) {
	uint32_t	int32val;
	uint16_t 	templast = boardio.tempscaled;


	int32val = TEMP_CHAN.RESL;			// !!! Lowbyte must be read first
	int32val |= (TEMP_CHAN.RESH << 8);	// And lowbyte and highbyte reads must be kept separate

	int32val -= ATMELMCU_ADC_USIGNOFS;	// ADC offset value when using unsigned mode
	//int32val <<= 8;
	int32val <<= 10;					// Increases calculation precision so we can drop decimal part
	int32val = int32val / boardio.caltempf;	// Calculate actual temperature, it is ok to drop decimal part
	boardio.tempscaled = int32val - 2730;	// Subtract Celsius 0 degree value which is adjusted for precision

	if (!templast) {
		templast += ((boardio.tempscaled - templast) >> 1);
		boardio.tempscaled = templast;
	}
}


/***************************************************************************
* Get board hardware settings
* [18/08/2015]
* MX board flag created
* [10/09/2016]
***************************************************************************/
static void getboardhw(boardhwtabstr *hwptr) {
	uint8_t					tabcnt;
	//athwenum				hwtype;


	for (tabcnt = 0; tabcnt < (ARRAY_SIZE(boardhwtable)); tabcnt++) {
		memcpy_P(hwptr, &boardhwtable[tabcnt], sizeof(boardhwtable[0]));

		//hwtype = pgm_read_byte(&boardhwtable[tabcnt].type);	// Read hardware type from program space
		//if (hwtype == (MainLeiodc.hw & ATHW_ID_MASK)) {
		if (hwptr->type == (MainLeiodc.hw & ATHW_ID_MASK)) {
			//*((uint32_t *) &hwptr->dicount) = pgm_read_dword(&boardhwtable[tabcnt].dicount);	// Read DI count from program space
			//hwptr->docount = pgm_read_byte(&boardhwtable[tabcnt].docount);	// Read DO count from program space
			//hwptr->dioffs = pgm_read_byte(&boardhwtable[tabcnt].dioffs);	// Read DI base offset from program space
			//hwptr->dooffs = pgm_read_byte(&boardhwtable[tabcnt].dooffs);	// Read DO base offset from program space
			return;
		}
	}
	memset(hwptr, 0, sizeof(boardhwtabstr));	// Clean structure if hardware type is not found
}


/***************************************************************************
* Board IO main process
* [25/02/2015]
* DI and DO mode enums now defined in modbusdef.h
* [19/08/2015]
* LED driver main process moved from main.c
* [24/08/2015]
* LED clear function removed
* [09/09/2016]
***************************************************************************/
void board_mainproc(void) {
	uint8_t				cnt;
	boardDIstr			*diptr = boardio.diptr;
	boardDOstr			*doptr = boardio.doptr;


	if (doptr) {
		for (cnt = 0; cnt < doptr->count; cnt++) {
			switch (doptr->mode[cnt]) {
			case modbusdomden_pulseout:
				if (doptr->dostates & (1 << cnt)) {
					if (doptr->samples[cnt] >= doptr->pulsedur[cnt]) {	// Release DO if activate and samples expired
						releaseDO(doptr, cnt);
					}
				}

				if (doptr->updatereg & (1 << cnt)) {			// Request to update this DO
					if (!(doptr->updatereg & ~(1 << cnt))) {	// Only one DO requested
						if (checkotherdoactive(doptr, cnt) == LE_OK) {	// If no other DOs already active
							activateDO(doptr, cnt);
						}
					}
				}
				break;


			default:	// Don't do anything if output policy is unknown
				break;
			}
		}
		doptr->updatereg = 0;	// CLear upstream requests
	}


	// TODO add timer and disable LED OE pin when expired and
	// if there were no indication changes
#ifdef DEBUG_LEDUPDATE
	boardio.rflags |= BOARDRF_UPDATE_LED;
#endif
	if (boardio.rflags & BOARDRF_UPDATE_LED) {		// Update LEDs if flag is set
		boardio.rflags &= ~BOARDRF_UPDATE_LED;		// Must be immediately AFTER check, due to interrupt awareness
		//ledregclear();
		memset(leddriver.leddata, 0, LED_DRIVER_COUNT);
		leddriver.rflags |= LEDRF_UPDATE_LED;

		if (diptr) {
			for (cnt = 0; cnt < diptr->count; cnt++) {
				if (diptr->distates & (1 << cnt)) {
					leddriver.leddata[CALCULATE_REGOFFSET(diptr, cnt)] |= CALCULATE_BITMASK(diptr, cnt);
				}
			}
		}
		if (doptr) {
			for (cnt = 0; cnt < doptr->count; cnt++) {
				if (doptr->dostates & (1 << cnt)) {
					leddriver.leddata[CALCULATE_REGOFFSET(doptr, cnt)] |= CALCULATE_BITMASK(doptr, cnt);
				}
			}
		}
	}


	calctemperature();

#ifndef DEBUG_NOLEDDRV
	leddrv_mainproc();
#endif
}


/***************************************************************************
* Initialize board hardware
* [26/02/2015]
* ADCB is used for temperature measurements
* [14/06/2015]
* New hardware 3100 without MX board
* Changes related to board hardware profile table creation
* Default configuration pin created
* [19/08/2015]
* IO=4000, IO=0400, IO=2200 modules added
* MX board flag created
* Reset register created
* [17/10/2016]
***************************************************************************/
void board_init(void) {
	uint16_t		adcafactorycal, adcbfactorycal;
	boardhwtabstr	boardhw;


	clock_init();					// Init system clock first

#ifndef DISABLE_BOARD_AUTOID		// Disable automatic board ID identification using resistors on port PE
	PORTCFG.MPCMASK = MASK_AUTOID;	// Mask pins which are not used as ID inputs
	MCUP_AUTOID.PIN0CTRL |= PORT_OPC_PULLUP_gc;		// Enable pull-up resistors

	for (uint8_t cnt = 0; cnt < 32; cnt++) {}		// Wait a little bit before reading inputs

	MainLeiodc.hw = MCUP_AUTOID.IN & MASK_AUTOID;	// Read board ID resistors
	if (MCUP_MXAUTOID.IN & PIN_MXAUTOID)			// Read MX board identification pin
		MainLeiodc.hw |= ATHWF_MXBOARD;
#endif // DISABLE_BOARD_AUTOID

	memset(&boardio, 0, sizeof(boardio));		// Clean board structure
	if (eeconf_crc(0) == LE_FAIL) {				// Check EEPROM configuration CRC
		boardio.rflags |= BOARDRF_EECONF_CORRUPTED;
	}


	PORTCFG.MPCMASK = 0xFF;					// Apply new configuration to all pins
	MCUP_IOL.PIN0CTRL |= PORT_INVEN_bm;		// Invert all pins
	MCUP_IOL.DIR = 0;						// Set direction to input for all pins

	PORTCFG.MPCMASK = 0xFF;					// Apply new configuration to all pins
	MCUP_IOH.PIN0CTRL |= PORT_INVEN_bm;		// Invert all pins

	PORTCFG.VPCTRLA = PORTCFG_VP0MAP_PORTF_gc | PORTCFG_VP1MAP_PORTH_gc;	// Map MCU ports to virtual ports

	boardio.ctrlport = &MCUP_CTRL;			// Control port

	getboardhw(&boardhw);					// Get board hardware settings


	switch (MainLeiodc.hw & ATHW_ID_MASK) {
	/*case somerevision:
		break;*/

	case athwenat3100v11:
		MCUP_IOH.OUT = 0x00;					// Clear Outputs - logic 0, but pins driven high because of inversion
		MCUP_IOH.DIR = 0xF0;					// Pins [0..3] are inputs, pins [4..7] outputs
		boardio.ledoepin = PIN6_bm;				// LED OE pin
		boardio.defcfgpin = PIN4_bm;			// Default configuration input pin
		MainLeiodc.hw &= ~ATHWF_MXBOARD;		// Clear MX board flag for legacy resistor configuration
		break;

	case athwenmx3100v11:
		MCUP_IOH.OUT = 0x00;					// Clear Outputs - logic 0, but pins driven high because of inversion
		MCUP_IOH.DIR = 0xF0;					// Pins [0..3] are inputs, pins [4..7] outputs
		boardio.ledoepin = PIN6_bm;				// LED OE pin
		boardio.defcfgpin = PIN4_bm;			// Default configuration input pin
		break;

	case athwenat2200v10:
		MCUP_IOH.OUT = 0x00;					// Clear Outputs - logic 0, but pins driven high because of inversion
		MCUP_IOH.DIR = 0xFF;					// Pins [0..7] are outputs
		boardio.ledoepin = PIN6_bm;				// LED OE pin
		boardio.defcfgpin = PIN4_bm;			// Default configuration input pin
		break;

	case athwenat4000v10:
		MCUP_IOH.DIR = 0x00;					// Pins [0..7] are inputs
		boardio.ledoepin = PIN6_bm;				// LED OE pin
		boardio.defcfgpin = PIN4_bm;			// Default configuration input pin
		break;

	case athwenat0400v10:
		MCUP_IOH.OUT = 0x00;					// Clear Outputs - logic 0, but pins driven high because of inversion
		MCUP_IOL.OUT = 0x00;					// Clear Outputs - logic 0, but pins driven high because of inversion
		MCUP_IOH.DIR = 0xFF;					// Pins [0..7] are outputs
		MCUP_IOL.DIR = 0xFF;					// Pins [0..7] are outputs
		boardio.ledoepin = PIN6_bm;				// LED OE pin
		boardio.defcfgpin = PIN4_bm;			// Default configuration input pin
		break;

	default:
		MCUP_IOH.OUT = 0x00;					// Clear Outputs - logic 0, but pins driven high because of inversion
		MCUP_IOH.DIR = 0xF0;					// Pins [0..3] are inputs, pins [4..7] outputs
		boardio.ledoepin = 0;					// LED OE pin is not used
		MainLeiodc.hw |= ATHWF_MXBOARD;			// Force MX board flag for legacy resistor configurations
		break;
	}


	initboardDI(boardhw.dicount, boardhw.dioffs);	// Initialize all inputs
	initboardDO(boardhw.docount, boardhw.dooffs);	// Initialize all outputs
	boardio.mapsize =
			MODBUS_SYSREG_COUNT +
			(MODBUS_DIMULT * boardhw.dicount) + 1 +
			(MODBUS_DOMULT * boardhw.docount) + 1;


#ifndef DEBUG_NOLEDDRV
	leddrv_init();
#endif


	LED_CONTROL_PIN_ON			// Activate LED CONTROL (OE) pin
	if (boardio.ledoepin)
		boardio.ctrlport->DIRSET = boardio.ledoepin;	// LED OE pin is output


	if (boardio.defcfgpin) {	// Default configuration input pin
		boardio.ctrlport->DIRCLR = boardio.defcfgpin;		// Change direction to input
		PORTCFG.MPCMASK = boardio.defcfgpin;
		boardio.ctrlport->PIN0CTRL = PORT_OPC_PULLUP_gc;

		for (uint8_t cnt = 0; cnt < 32; cnt++) {}			// Wait a little bit before reading input

		if (!(boardio.ctrlport->IN & boardio.defcfgpin)) {	// Check default configuration pin
			boardio.rflags |= BOARDRF_DEFCONF;				// Set default configuration flag
		}
	}


	PMIC.CTRL |= PMIC_LOLVLEN_bm;		// Enable low level interrupts
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;		// Enable medium level interrupts


	// Read factory calibration register from program memory (ROM)
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	//adcafactorycal = pgm_read_byte(0x20);
	//adcafactorycal |= pgm_read_byte(0x21) << 8;
	adcafactorycal = pgm_read_word(0x20);
	//adcbfactorycal = pgm_read_byte(0x24);
	//adcbfactorycal |= pgm_read_byte(0x25) << 8;
	adcbfactorycal = pgm_read_word(0x24);
	//caltemp85 = pgm_read_byte(0x2E);
	//caltemp85 |= pgm_read_byte(0x2F) << 8;
	boardio.caltemp85 = pgm_read_word(0x2E);
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	ADCA.CAL = adcafactorycal;							// Set factory calibration word
	ADCB.CAL = adcbfactorycal;							// Set factory calibration word


	BOARD_ADC.CTRLA = ADC_ENABLE_bm;					// Enable ADCA
	//BOARD_ADC.CTRLB = ADC_FREERUN_bm;					// Free running mode (don't need to start conversion manually)
	BOARD_ADC.CTRLB = ADC_FREERUN_bm | ADC_CONMODE_bm;	// Free running mode, signed conversion
	BOARD_ADC.REFCTRL = ADC_REFSEL0_bm; 				// Select INTVCC as reference input
	//BOARD_ADC.REFCTRL = ADC_REFSEL0_bm | ADC_TEMPREF_bm; // Select INTVCC as reference input, enable temperature measurement
	BOARD_ADC.PRESCALER = ADC_PRESCALER_gm;				// Divide peripheral clock by 512


	TEMP_ADC.CTRLA = ADC_ENABLE_bm;						// Enable ADCB
	TEMP_ADC.CTRLB = ADC_FREERUN_bm;					// Free running mode (don't need to start conversion manually)
	//TEMP_ADC.CTRLB = ADC_FREERUN_bm | ADC_CONMODE_bm;	// Free running mode, signed conversion
	TEMP_ADC.REFCTRL = ADC_TEMPREF_bm; 					// Select INT1V (bandgap) as reference input, enable temperature measurement
	TEMP_ADC.PRESCALER = ADC_PRESCALER_gm;				// Divide peripheral clock by 512
	TEMP_ADC.EVCTRL = ADC_SWEEP_0_gc;					// Sweep ADC channel 0
	TEMP_CHAN.CTRL = ADC_CH_START_bm;					// Start conversion on Channel 0
	boardio.caltempf = boardio.caltemp85 / 3.49609375;	// Calibration value / ((85 + 273) * 10 / 1024) calibration temperature in Kelvins and multipliers for scaling


	boardio.resetreg = RST.STATUS | RESETREG_PLACEHOLDER;	// Read reset status register
	RST.STATUS = 0x3f;									// Clear bits [0..5] of reset status register
	//enable watchdog timer 256ms
	//wdt_enable(WDT_PER_256CLK_gc);

	//global interrupts enable
	sei();
}
