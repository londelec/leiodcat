/*
 ============================================================================
 Name        : board.c
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : board hardware module

  Change log  :

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
#include "main.h"
#include "timer.h"
#include "74lv8153.h"
#include "mcueecfg.h"




BOARD_t 	boardio;
uint16_t	caltemp85;			// Temperature calibration value from NVM


#ifdef GLOBAL_DEBUG
//#define DEBUG_NOEE
#endif	// GLOBAL_DEBUG


// Offset and mask calculation for IO selection
#define	CALCULATE_REGOFFSET(mioptr, mindex) ((mioptr->bitoffset[mindex] & 0x18) >> 3)
#define	CALCULATE_VPORTPTR(mioptr, mindex)	&VPORT_BASE + CALCULATE_REGOFFSET(mioptr, mindex)
#define	CALCULATE_BITMASK(mioptr, mindex)	(1 << (mioptr->bitoffset[mindex] & 0x07))


// PIN manipulation macros
#define LED_CONTROL_PIN_ON	if (boardio.ledoepin) boardio.ctrlport->OUTCLR = boardio.ledoepin;




/***************************************************************************
* Initialize board hardware
* [26/02/2015]
***************************************************************************/
void board_init() {
	uint16_t			factorycal;


#ifndef DISABLE_BOARD_AUTOID		// Disable automatic board ID identification using resistors on port PE
	PORTCFG.MPCMASK = BOARD_AUTOID_MASK;	// Mask pins which are not used as ID inputs
	BOARD_AUTOID_MCUPORT.PIN0CTRL |= PORT_OPC_PULLUP_gc;			// Enable pull-up resistors
	for (uint8_t cnt = 0; cnt < 32; cnt++) {}						// Wait a little bit before reading inputs
	BoardHardware = BOARD_AUTOID_MCUPORT.IN & BOARD_AUTOID_MASK;	// Read board ID resistors
#endif // DISABLE_BOARD_AUTOID


	switch (BoardHardware) {
	/*case somerevision:
		break;*/

	case athwenmx3100v11:
		PORTCFG.MPCMASK = 0xFF;					// Apply new configuration to all pins (no mask)
		PORTF.PIN0CTRL |= PORT_INVEN_bm;		// Invert all pins
		PORTF.DIR = 0;							// Set direction to input for all pins

		PORTCFG.MPCMASK = 0xFF;					// Apply new configuration to DI pins (mask highbyte)
		PORTH.PIN0CTRL |= PORT_INVEN_bm;		// Invert all pins
		PORTH.OUT = 0x00;						// Clear Outputs - logic 0, but pins driven high because of inversion
		PORTH.DIR = 0xF0;						// Pins [0..3] are inputs, pins [4..7] outputs

		PORTCFG.VPCTRLA = PORTCFG_VP0MAP_PORTF_gc | PORTCFG_VP1MAP_PORTH_gc;	// Map MCU ports to virtual ports

		initboardDI(12, 0);						// Initialize all inputs
		initboardDO(4, 0x0C);					// Initialize all outputs
		boardio.ctrlport = &PORTK;				// Control port
		boardio.ledoepin = PIN6_bm;				// LED OE pin
		boardio.mapsize =
				MODBUS_SYSREG_COUNT +
				(MODBUS_DIMULT * 12) + 1 +
				(MODBUS_DOMULT * 4) + 1;
		break;


	default:
		PORTCFG.MPCMASK = 0xFF;					// Apply new configuration to all pins (no mask)
		PORTF.PIN0CTRL |= PORT_INVEN_bm;		// Invert all pins
		PORTF.DIR = 0;							// Set direction to input for all pins

		PORTCFG.MPCMASK = 0xFF;					// Apply new configuration to DI pins (mask highbyte)
		PORTH.PIN0CTRL |= PORT_INVEN_bm;		// Invert all pins
		PORTH.OUT = 0x00;						// Clear Outputs - logic 0, but pins driven high because of inversion
		PORTH.DIR = 0xF0;						// Pins [0..3] are inputs, pins [4..7] outputs

		PORTCFG.VPCTRLA = PORTCFG_VP0MAP_PORTF_gc | PORTCFG_VP1MAP_PORTH_gc;	// Map MCU ports to virtual ports

		initboardDI(12, 0);						// Initialize all inputs
		initboardDO(4, 0x0C);					// Initialize all outputs
		boardio.ctrlport = &PORTK;				// Control port
		boardio.ledoepin = 0;					// LED OE pin is not used
		boardio.mapsize =
				MODBUS_SYSREG_COUNT +
				(MODBUS_DIMULT * 12) + 1 +
				(MODBUS_DOMULT * 4) + 1;
		break;
	}


	clock_init();
	leddrv_init();


	LED_CONTROL_PIN_ON							// Activate LED CONTROL (OE) pin
	if (boardio.ledoepin) boardio.ctrlport->DIRSET = boardio.ledoepin;


	PMIC.CTRL |= PMIC_LOLVLEN_bm;		// Enable low level interrupts
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;		// Enable medium level interrupts


	// Read factory calibration register from program memory (ROM)
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	factorycal = pgm_read_byte(0x20);
	factorycal |= pgm_read_byte(0x21) << 8;
	caltemp85 = pgm_read_byte(0x2E);
	caltemp85 |= pgm_read_byte(0x2F) << 8;
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	BOARD_ADC.CAL = factorycal;				// Set factory calibration word
	//BOARD_ADC.CAL = 0x0000;


	BOARD_ADC.CTRLA = ADC_ENABLE_bm;					// Enable ADCA
	//BOARD_ADC.CTRLB = ADC_FREERUN_bm;					// Free running mode (don't need to start conversion manually)
	BOARD_ADC.CTRLB = ADC_FREERUN_bm | ADC_CONMODE_bm;	// Free running mode (don't need to start conversion manually), signed conversion
	//BOARD_ADC.REFCTRL = ADC_REFSEL0_bm; 				// Select INTVCC as reference input
	BOARD_ADC.REFCTRL = ADC_REFSEL0_bm | ADC_TEMPREF_bm; // Select INTVCC as reference input, enable temperature measurement
	BOARD_ADC.PRESCALER = ADC_PRESCALER_gm;				// Divide peripheral clock by 512
	BOARD_TEMPCH.CTRL = ADC_CH_START_bm;				// Start conversion on Channel 0


	//enable watchdog timer 256ms
	//wdt_enable(WDT_PER_256CLK_gc);

	//global interrupts enable
	sei();
}


/***************************************************************************
* Initialize digital inputs
* [24/02/2015]
***************************************************************************/
void initboardDI(uint8_t dicount, uint8_t baseoffset) {
	uint8_t				cnt;
	uint32_t			eedword;


	boardio.diptr = calloc(1, sizeof(boardDIstr));
	boardio.diptr->count = dicount;

	boardio.diptr->samples = calloc(dicount, sizeof(uint16_t));
	boardio.diptr->filterconst = calloc(dicount, sizeof(uint16_t));
	boardio.diptr->bitoffset = calloc(dicount, sizeof(uint8_t));

	for (cnt = 0; cnt < dicount; cnt++) {
		boardio.diptr->bitoffset[cnt] = (baseoffset + cnt);
		if (getee_data(eegren_board, (eedten_board_diflt00 + cnt), &eedword) == EXIT_SUCCESS) {
			boardio.diptr->filterconst[cnt] = eedword;
		}
		else boardio.diptr->filterconst[cnt] = DI_DEFAULT_FILTER_MSEC;
	}
}


/***************************************************************************
* Initialize digital outputs
* [24/02/2015]
***************************************************************************/
void initboardDO(uint8_t docount, uint8_t baseoffset) {
	uint8_t				cnt;
	uint32_t			eedword;


	boardio.doptr = calloc(1, sizeof(boardDOstr));
	boardio.doptr->count = docount;
	boardio.doptr->samples = calloc(docount, sizeof(uint16_t));
	boardio.doptr->holdperiod = calloc(docount, sizeof(uint16_t));
	boardio.doptr->bitoffset = calloc(docount, sizeof(uint8_t));
	boardio.doptr->policy = calloc(docount, sizeof(dopolicyenum));
	for(cnt = 0; cnt < docount; cnt++) {
		boardio.doptr->bitoffset[cnt] = (baseoffset + cnt);
		if (getee_data(eegren_board, (eedten_board_dohld00 + cnt), &eedword) == EXIT_SUCCESS) {
			boardio.doptr->holdperiod[cnt] = eedword;
		}
		else boardio.doptr->holdperiod[cnt] = DO_DEFAULT_HOLD_MSEC;
		if (getee_data(eegren_board, (eedten_board_dopol00 + cnt), &eedword) == EXIT_SUCCESS) {
			boardio.doptr->policy[cnt] = eedword;
		}
		else boardio.doptr->policy[cnt] = dopolen_holdperiod;
	}
}


/***************************************************************************
* Board IO main process
* [25/02/2015]
***************************************************************************/
void board_mainproc() {
	uint8_t				cnt;
	boardDIstr			*diptr = boardio.diptr;
	boardDOstr			*doptr = boardio.doptr;


	if (doptr) {
		for (cnt = 0; cnt < doptr->count; cnt++) {
			switch (doptr->policy[cnt]) {
			case dopolen_holdperiod:
				if (doptr->dostates & (1 << cnt)) {
					if (doptr->samples[cnt] >= doptr->holdperiod[cnt]) {	// Release DO if activate and samples expired
						releaseDO(doptr, cnt);
					}
				}

				if (doptr->updatereg & (1 << cnt)) {			// Request to update this DO
					if (!(doptr->updatereg & ~(1 << cnt))) {	// Only one DO requested
						if (checkotherdoactive(doptr, cnt) == EXIT_SUCCESS) {	// If no other DOs already active
							activateDO(doptr, cnt);
						}
					}
				}
				break;


			default:	// Don't do anything if output policy is unknown
				break;
			}
		}
		doptr->updatereg = 0;		// CLear upstream requests
	}


	// TODO add timer and disable LED OE pin when expired and
	// if there were no indication changes
	if (boardio.rflags & BOARDRF_UPDATE_LED) {		// Update LEDs if flag is set
		boardio.rflags &= ~BOARDRF_UPDATE_LED;		// Must be immediately AFTER check, due to interrupt awareness
		ledregclear();
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
}




/***************************************************************************
* Check if any DO is active before activating requested one
* [25/02/2015]
***************************************************************************/
uint8_t checkotherdoactive(boardDOstr *doptr, uint8_t doindex) {
	uint8_t				cnt;


	for (cnt = 0; cnt < doptr->count; cnt++) {
		if (cnt != doindex) {
			if (doptr->dostates & (1 << cnt)) {		// Check bit of the particular DO
				return EXIT_FAILURE;	// Another DO is already active
			}
		}
	}
	return EXIT_SUCCESS;	// No other DOs are active at the moment
}


/***************************************************************************
* Activate output
* [25/02/2015]
***************************************************************************/
void activateDO(boardDOstr *doptr, uint8_t doindex) {
	VPORT_t 			*vportptr;


	doptr->samples[doindex] = 0;			// Clear samples BEFORE changing state, due to interrupt awareness
	doptr->dostates |= (1 << doindex);		// Update realtime status

	vportptr = CALCULATE_VPORTPTR(doptr, doindex);				// Calculate VPORT structure pointer
	vportptr->OUT |= CALCULATE_BITMASK(doptr, doindex);			// Set bit of the particular DO
	boardio.rflags |= BOARDRF_UPDATE_LED;
}


/***************************************************************************
* Release output
* [25/02/2015]
***************************************************************************/
void releaseDO(boardDOstr *doptr, uint8_t doindex) {
	VPORT_t 			*vportptr;


	doptr->dostates &= ~(1 << doindex);		// Update realtime status

	vportptr = CALCULATE_VPORTPTR(doptr, doindex);				// Calculate VPORT structure pointer
	vportptr->OUT &= ~(CALCULATE_BITMASK(doptr, doindex));		// Clear bit of the particular DO
	boardio.rflags |= BOARDRF_UPDATE_LED;

	//if (*boardio.doptr->dovport == 0) {
	//	bit_clear(boardio.status, _Busy_bp);
	//}
}




/***************************************************************************
* 1ms ISR for IO processing
* [24/02/2015]
***************************************************************************/
inline void boardisr_1ms() {
	uint8_t				cnt;
	boardDIstr			*diptr = boardio.diptr;
	boardDOstr			*doptr = boardio.doptr;
	VPORT_t 			*vportptr;
	uint16_t			statebit;


	if (diptr) {
		statebit = 1;
		for (cnt = 0; cnt < diptr->count; cnt++) {
			vportptr = CALCULATE_VPORTPTR(diptr, cnt);		// Calculate VPORT structure pointer
			uint8_t dibit = vportptr->IN & CALCULATE_BITMASK(diptr, cnt);	// Calculate bitmask of the particular DI
			if (
					(dibit && (diptr->distates & statebit)) ||
					(!dibit && !(diptr->distates & statebit))) {	// Check bit of the particular DI
				diptr->samples[cnt] = 0;
			}
			else {
				diptr->samples[cnt]++;
				if (diptr->samples[cnt] >= diptr->filterconst[cnt]) {
					if (dibit) diptr->distates |= statebit;
					else diptr->distates &= ~statebit;
					diptr->samples[cnt] = 0;
					boardio.rflags |= BOARDRF_EVENT | BOARDRF_UPDATE_LED;
				}
			}
			statebit <<= 1;
		}
	}


	if (doptr) {
		for (cnt = 0; cnt < doptr->count; cnt++) {
			switch (doptr->policy[cnt]) {
			case dopolen_holdperiod:
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



