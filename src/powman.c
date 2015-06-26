/*
 ============================================================================
 Name        : powman.c
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Power management for MX28 board

  Change log  :

  *********V1.01 16/06/2015**************
  Configuration update request bit set introduced

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */

#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>


#include "ledefs.h"
#include "main.h"
#include "powman.h"
#include "mcueecfg.h"
#include "board.h"


#ifdef GLOBAL_DEBUG
#define DEBUG_IGNORE_VDDIO
#define DEBUG_NOIDLECNT
#define DEBUG_IGNORE_HBHIGH
#endif	// GLOBAL_DEBUG


MXpowStr MXpower;



// Macros
#define POWER_GATE_3V8_ON	if (MXpower.cfg.pin3v8gate) boardio.ctrlport->OUTCLR = MXpower.cfg.pin3v8gate;
#define POWER_GATE_3V8_OFF	if (MXpower.cfg.pin3v8gate) boardio.ctrlport->OUTSET = MXpower.cfg.pin3v8gate;
#define POWER_GATE_3V3_ON	if (MXpower.cfg.pin3v3gate) boardio.ctrlport->OUTCLR = MXpower.cfg.pin3v3gate;
#define POWER_GATE_3V3_OFF	if (MXpower.cfg.pin3v3gate) boardio.ctrlport->OUTSET = MXpower.cfg.pin3v3gate;
#define POWER_SWITCH_ACT	if (MXpower.cfg.pinpowsw) boardio.ctrlport->OUTCLR = MXpower.cfg.pinpowsw;
#define POWER_SWITCH_REL	if (MXpower.cfg.pinpowsw) boardio.ctrlport->OUTSET = MXpower.cfg.pinpowsw;

#define POWMANF_ENABLE_OUTPUTS	outputpinctrl(0);
#define POWMANF_DISABLE_OUTPUTS	outputpinctrl(1);

#define POWMANF_SET_MTIMER(mtconst) timerset_fine(mtconst, &MXpower.timer);
#define POWMANF_CHECK_MTIMER timercheck_fine(&MXpower.timer)


/***************************************************************************
* Initialize power manager module
* [04/03/2015]
***************************************************************************/
void powman_init() {
	uint32_t			eedword;
	uint8_t				cnt;


	MXpower.state = powst_init;
	POWMANF_SET_MTIMER(POW_STARTUP_DELAY)					// Set 3V8 enable delay constant


	switch (BoardHardware) {
	/*case somerevision:
		break;*/

	case athwenmx3100v11:
		BOARD_VDDIOCH.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;	// PORTA pin 1 selected
		MXpower.cfg.pinhb = PIN0_bm;					// MB_SSP3_MISO
		MXpower.cfg.pin3v3gate = PIN4_bm;				// 3V3_POWER_GATE
		MXpower.cfg.pin3v8gate = PIN5_bm;				// 3V8_POWER_GATE
		MXpower.cfg.pinpowsw = PIN7_bm;					// POWERON_GATE

		break;

	default:
		BOARD_VDDIOCH.MUXCTRL = ADC_CH_MUXPOS_PIN6_gc;	// PORTA pin 6 selected
		MXpower.cfg.pinhb = 0;							// Not used
		MXpower.cfg.pin3v3gate = PIN4_bm;				// 3V3_POWER_GATE
		MXpower.cfg.pin3v8gate = PIN5_bm;				// 3V8_POWER_GATE
		MXpower.cfg.pinpowsw = PIN7_bm;					// POWERON_GATE
		break;
	}


	if (eeconf_get(eegren_powman, eedten_powman_thadc3v2, &eedword) == EXIT_SUCCESS) {
		MXpower.cfg.thadc3v2 = eedword;
	}
	else {
		MXpower.cfg.thadc3v2 = POWADC_3V2_VALUE;		// Default value
		boardio.eeupdatebs |= (1 << eegren_powman);
	}
	//BOARD_ADC.EVCTRL |= ADC_SWEEP_01_gc;							// Sweep ADC channels 0 & 1
	BOARD_ADC.EVCTRL = ADC_SWEEP_0_gc;								// Sweep ADC channel 0
	BOARD_VDDIOCH.CTRL = ADC_CH_START_bm | ADC_CH_INPUTMODE0_bm;	// Start conversion on Channel 0, single-ended positive input signal


	PORTCFG.MPCMASK =									// Set configuration of all these pins simultaneously
			MXpower.cfg.pin3v8gate |
			MXpower.cfg.pin3v3gate |
			MXpower.cfg.pinpowsw;
	boardio.ctrlport->PIN0CTRL = PORT_OPC_WIREDAND_gc;	// Wired-AND configuration
	boardio.ctrlport->OUTSET = 							// Set pins to logic 1 before setting direction
			MXpower.cfg.pin3v8gate |
			MXpower.cfg.pin3v3gate |
			MXpower.cfg.pinpowsw;
	boardio.ctrlport->DIRSET =							// Change direction to output
			MXpower.cfg.pin3v8gate |
			MXpower.cfg.pin3v3gate |
			MXpower.cfg.pinpowsw;
	boardio.ctrlport->DIRCLR = MXpower.cfg.pinhb;		// Change direction to input


	if (MXpower.cfg.pinhb) {
		for (cnt = 0; cnt < 8; cnt++) {
			if (MXpower.cfg.pinhb == (1 << cnt)) {
				(&boardio.ctrlport->PIN0CTRL)[cnt] |= PORT_ISC_FALLING_gc;	// Falling edge interrupt
				break;
			}
		}
		boardio.ctrlport->INT1MASK = MXpower.cfg.pinhb;		// Enable heartbeat pin interrupt
		boardio.ctrlport->INTCTRL = (boardio.ctrlport->INTCTRL & ~PORT_INT1LVL_gm) | PORT_INT1LVL_LO_gc;	// Enable port interrupt
	}
}




/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 * Power management processor main function
 * [04/03/2015]
 * Output enable macro added to debug mode
 * [16/06/2015]
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
uint8_t powman_mainproc() {
	uint16_t	adcvalue;


	switch (MXpower.state) {
	case powst_init:
		if (POWMANF_CHECK_MTIMER == EXIT_SUCCESS) {
			POWER_GATE_3V8_ON		// Turn on MX28 board supply 3V8
			MXpower.state = powst_actpowswitch;
			POWMANF_SET_MTIMER(POW_SWITCHON_DELAY)	// Set Power ON Switch delay
			POWMAN_RSTIDLE							// Reset MX idle counter
		}
		break;


	case powst_actpowswitch:
		if (POWMANF_CHECK_MTIMER == EXIT_SUCCESS) {
			POWER_SWITCH_ACT		// Activate MX28 power switch
			MXpower.state = powst_relpowswitch;
			POWMANF_SET_MTIMER(POW_SWITCH_DURATION)	// Set power switch duration
		}
		break;


	case powst_relpowswitch:
		if (POWMANF_CHECK_MTIMER == EXIT_SUCCESS) {
			POWER_SWITCH_REL		// Release MX28 power switch
			MXpower.state = powst_checkhb;
			POWMANF_SET_MTIMER(POW_T1MSEC)			// Set heartbeat checking interval = 1msec
			MXpower.poscnt = 0;
			MXpower.tmotcnt = 0;
		}
		break;


	case powst_checkhb:
		if (POWMANF_CHECK_MTIMER == EXIT_SUCCESS) {
			POWMANF_SET_MTIMER(POW_T1MSEC)			// Set heartbeat checking interval = 1msec
			if (boardio.ctrlport->IN & MXpower.cfg.pinhb) {	// Heartbeat pin must be high
				MXpower.poscnt++;
				if (MXpower.tmotcnt) MXpower.tmotcnt--;
			}
			else {
				MXpower.tmotcnt++;
				if (MXpower.poscnt) MXpower.poscnt--;
			}


			if (MXpower.poscnt > POW_HB_ACTIVE) {	// Heartbeat pin must be high for at least 5 msec
				MXpower.state = powst_waitmx3V3;
				POWMANF_SET_MTIMER(POW_T1MSEC)		// Set ADC checking interval = 1msec
				MXpower.poscnt = 0;
				MXpower.tmotcnt = 0;
			}
			else if (MXpower.tmotcnt > POW_HB_TIMEOUT) {	// If heartbeat didn't go high for 1 seconds
#ifdef DEBUG_IGNORE_HBHIGH
				MXpower.state = powst_waitmx3V3;
				POWMANF_SET_MTIMER(POW_T1MSEC)		// Set ADC checking interval = 1msec
				MXpower.poscnt = 0;
				MXpower.tmotcnt = 0;
#else
				POWMANF_DISABLE_OUTPUTS	// Disable other output pins
				POWER_GATE_3V3_OFF		// Turn off peripheral power 3V3
				POWER_GATE_3V8_OFF		// Turn off MX28 board supply 3V8
				POWMANF_SET_MTIMER(POW_REPOWER_DELAY);	// Delay before new power cycle
				MXpower.state = powst_init;
#endif	// DEBUG IGNORE HBHIGH
			}
		}
		break;


	case powst_waitmx3V3:
		if (POWMANF_CHECK_MTIMER == EXIT_SUCCESS) {
			POWMANF_SET_MTIMER(POW_T1MSEC)		// Set ADC checking interval = 1msec
			adcvalue = BOARD_VDDIOCH.RES;
			if (
					!(adcvalue & 0x8000) &&					// ADC integer must be positive
					(adcvalue > MXpower.cfg.thadc3v2)) {
				MXpower.poscnt++;
				if (MXpower.tmotcnt) MXpower.tmotcnt--;
			}
			else {
				MXpower.tmotcnt++;
				if (MXpower.poscnt) MXpower.poscnt--;
			}


#ifdef DEBUG_IGNORE_VDDIO
			POWER_GATE_3V3_ON
#endif
			if (MXpower.poscnt > POW_VDDIO3V3_STABLE) {	// If VDDIO was above 3.2V for at least 5 msec
				POWER_GATE_3V3_ON		// Turn on peripheral power 3V3
				POWMANF_ENABLE_OUTPUTS	// Enable other output pins
				POWMANF_SET_MTIMER(POW_T1SEC);				// VDDIO checking interval in idle state
				MXpower.state = powst_idle;
			}
			else if (MXpower.tmotcnt > POW_VDDIO3V3_TIMEOUT) {	// If VDDIO didn't go above 3.2V for 0.1 seconds
#ifdef DEBUG_IGNORE_VDDIO
				MXpower.state = powst_idle;
				POWMANF_ENABLE_OUTPUTS	// Enable other output pins
#else
				POWMANF_DISABLE_OUTPUTS	// Disable other output pins
				POWER_GATE_3V3_OFF		// Turn off peripheral power 3V3
				POWER_GATE_3V8_OFF		// Turn off MX28 board supply 3V8
				POWMANF_SET_MTIMER(POW_REPOWER_DELAY);	// Delay before new power cycle
				MXpower.state = powst_init;
#endif
			}
		}
		break;


	case powst_idle:
#ifndef DEBUG_IGNORE_VDDIO
		if (POWMANF_CHECK_MTIMER == EXIT_SUCCESS) {
			adcvalue = BOARD_VDDIOCH.RES;
			if (
					!(adcvalue & 0x8000) &&					// ADC integer must be positive
					(adcvalue > MXpower.cfg.thadc3v2)) {
				POWMANF_SET_MTIMER(POW_T1SEC);				// VDDIO checking interval in idle state
#ifndef DEBUG_NOIDLECNT
				if (MXpower.idlecnt) MXpower.idlecnt--;
				if (MXpower.idlecnt)
#endif	// DEBUG NOIDLECNT
					return EXIT_SUCCESS;

			}
			POWMANF_DISABLE_OUTPUTS	// Disable other output pins
			POWER_GATE_3V3_OFF		// Turn off peripheral power 3V3
			POWER_GATE_3V8_OFF		// Turn off MX28 board supply 3V8
			POWMANF_SET_MTIMER(POW_REPOWER_DELAY);	// Delay before new power cycle
			MXpower.state = powst_init;
			return EXIT_FAILURE;
		}
#endif	// DEBUG IGNORE VDDIO
		return EXIT_SUCCESS;
		//break;


	default:
		break;
	}
	return EXIT_FAILURE;
}




/***************************************************************************
* PORT pin interrupts
* [04/03/2015]
***************************************************************************/
ISR(CTRL_PORT_INT_VECT) {

	POWMAN_RSTIDLE							// Reset MX idle counter
}
