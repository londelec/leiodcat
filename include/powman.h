/*
============================================================================
 Name        : powman.h
 Author      : AK
 Version     : V1.02
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for power management for MX28 board

  Change log :

  *********V1.02 30/05/2023**************
  Idle timeout increased, required for OS5.0

  *********V1.01 18/08/2015**************
  Heartbeat output pin created

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */

#ifndef POWMAN_H_
#define POWMAN_H_


#include <stdint.h>

#include "ledefs.h"

/*
 * Timer constants in 100x microseconds
 */
#define POW_STARTUP_DELAY				20000		// MX28 3V8 on delay (default 2sec)
#define POW_SWITCHON_DELAY				10			// MX28 power switch assertion delay (default 1ms)
#define POW_SWITCH_DURATION				1000		// MX28 Power switch asserted duration at least 100ms as per i.MX28 Manual page 950 (100msec)
#define POW_REPOWER_DELAY				80000		// Wait before starting new power cycle (default 8sec)
#define POW_T100USEC					1			// 100usec constant
#define POW_T1MSEC						10			// 1msec constant
#define POW_T1SEC						10000		// 1sec constant
#define POW_HBLED_ON					500			// Heartbeat LED on period 0.05sec
#define POW_HBLED_OFF					10000		// Heartbeat LED off timer 1sec
/*
 * Millisecond counters
 */
#define POW_VDDIO3V3_STABLE				5			// MX28 VDDIO_3V3 have to be present for this period before peripheral 3V3 is switched on (1sec)
													// This is essential because switching on 3.3V power creates a voltage spikes which interferes with SD card read
#define POW_HB_ACTIVE					5			// Heartbeat pin must be active for this period (5msec)
#define POW_VDDIO3V3_TIMEOUT			100			// MX28 VDDIO_3V3 detection timeout after power switch release (0.1sec)
#define POW_HB_TIMEOUT					1500		// Heartbeat detection timeout after power switch release (1.5sec)
													// It may take up to 1 sec to load the uBooot from SD card and execute the code where heartbeat pin is activated
/*
 * Second counters
 */
#define POW_MXIDLE						100			// MX28 idle state counter (100sec)


#define CTRL_PORT_INT_VECT				PORTK_INT1_vect	// port interrupt vector

/*
 * Macros
 */
#define POWMAN_RSTIDLE		MXpower.idlecnt = POW_MXIDLE;


typedef enum {
	powst_init = 0,									// Initial state, after power on/reset
	powst_actpowswitch,								// Activate MX28 power switch
	powst_relpowswitch,								// Release MX28 power switch
	powst_checkhb,									// Check heartbeat pin and wait to become high
	powst_waitmx3V3,								// Read ADC and wait for VDDIO to exceed 3.2V
	powst_idle,										// Idle state, power on sequence complete
} powstate_e;


struct MXpoweecfg_s {
	uint8_t					pin3v8gate;				// MCU pin to control 3V8_POWER_GATE
	uint8_t					pin3v3gate;				// MCU pin to control 3V3_POWER_GATE
	uint8_t					pinpowsw;				// MCU pin to control POWERON_GATE
	uint8_t					pinhbin;				// MCU input pin to check MX28 heart-beat MB_SSP3_MISO
	uint8_t					pinhbout;				// MCU output pin that drives RUN LED for hardware configuration without MX
	uint16_t				thadc3v2;				// ADC threshold for 3V2 detection
};


typedef struct MXpow_s {
	powstate_e				state;					// Power state
	finetm_t				timer;					// State timer
	uint16_t				poscnt;					// ADC sampling counter, increment if VDDIO is OK (above threshold)
	uint16_t				tmotcnt;				// Timeout counter, increment if VDDIO is not present or heartbeat is not high
	struct MXpoweecfg_s		cfg;					// Pin and analog threshold configuration
	uint8_t					idlecnt;				// Idle counter, used to monitor MX activity
} MXpow_t;


extern MXpow_t MXpower;


void powman_init(void);
uint8_t powman_mainproc(void);


#endif /* POWMAN_H_ */
