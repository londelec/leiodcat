/*
============================================================================
 Name        : powman.h
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for power management for MX28 board

  Change log  :

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */

#ifndef POWMAN_H_
#define POWMAN_H_


#include <stdint.h>

#include "ledefs.h"
//#include "main.h"




// Timer constants in 100x microseconds
#define	POW_STARTUP_DELAY				20000		// MX28 3V8 on delay (default 2sec)
#define	POW_SWITCHON_DELAY				10			// MX28 power switch assertion delay (default 1ms)
#define	POW_SWITCH_DURATION				1000		// MX28 Power switch asserted duration at least 100ms as per i.MX28 Manual page 950 (100msec)
#define	POW_REPOWER_DELAY				80000		// Wait before starting new power cycle (default 8sec)
#define	POW_T100USEC					1			// 100usec constant
#define	POW_T1MSEC						10			// 1msec constant
#define	POW_T1SEC						10000		// 1sec constant
// Milisecond counters
#define	POW_VDDIO3V3_STABLE				5			// MX28 VDDIO_3V3 have to be present for this period before peripheral 3V3 is switched on (1sec)
													// This is essential because switching on 3.3V power creates a voltage spikes which interferes with SD card read
#define	POW_HB_ACTIVE					5			// Heartbeat pin must be active for this period (5msec)
#define	POW_VDDIO3V3_TIMEOUT			100			// MX28 VDDIO_3V3 detection timeout after power switch release (0.1sec)
#define	POW_HB_TIMEOUT					1500		// Heartbeat detection timeout after power switch release (1.5sec)
													// It may take up to 1 sec to load the uBooot from SD card and execute the code where heartbeat pin is activated
// Second counters
#define	POW_MXIDLE						60			// MX28 idle state counter (60sec)


// ADC constants
#define	POWADC_3V2_VALUE				1588		// 3.2V / 2 / (VCC / 1.6V) * 2047 Default 3.2V ADC value


#define CTRL_PORT_INT_VECT				PORTK_INT1_vect	// port interrupt vector

// Macros
#define	POWMAN_RSTIDLE		MXpower.idlecnt = POW_MXIDLE;


typedef enum {
	powst_init							= 0,		// Initial state, after power on/reset
	powst_actpowswitch,								// Activate MX28 power switch
	powst_relpowswitch,								// Release MX28 power switch
	powst_checkhb,									// Check heartbeat pin and wait to become high
	powst_waitmx3V3,								// Read ADC and wait for VDDIO to exceed 3.2V
	powst_idle,										// Idle state, power on sequence complete
} LEOPACK powstateenum;


typedef struct MXpoweecfgStr_ {
	uint8_t					pin3v8gate;				// MCU pin to control 3V8_POWER_GATE
	uint8_t					pin3v3gate;				// MCU pin to control 3V3_POWER_GATE
	uint8_t					pinpowsw;				// MCU pin to control POWERON_GATE
	uint8_t					pinhb;					// MCU pin to check MX28 heart-beat MB_SSP3_MISO
	uint16_t				thadc3v2;				// ADC threshold for 3V2 detection
} MXpoweecfgStr;


typedef struct MXpowStr_ {
	powstateenum			state;					// Power state
	finetmstr				timer;					// State timer
	uint16_t				poscnt;					// ADC sampling counter, increment if VDDIO is OK (above threshold)
	uint16_t				tmotcnt;				// Timeout counter, increment if VDDIO is not present or heartbeat is not high
	MXpoweecfgStr			cfg;					// Pin and analog threshold configuration
	uint8_t					idlecnt;				// Idle counter, used to monitor MX activity
} MXpowStr;




// Version string made public to allow access from main
extern MXpowStr MXpower;


void powman_init();
uint8_t powman_mainproc();


#endif /* POWMAN_H_ */
