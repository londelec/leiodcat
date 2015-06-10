/*
 ============================================================================
 Name        : 74lv8153.h
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file LED driver module

  Change log  :

  *********V1.00 25/02/2015**************
  Initial revision

 ============================================================================
 */

#include "main.h"


#define LED_PORT_INT_VECT				PORTC_INT1_vect	// port interrupt vector
#define LED_DRIVER_COUNT				2				// Number of 74LV8153 chips on board

#define LED_RESET_HOLD					10				// RESET pin asserted holding period (x100usec), default 1msec

// LED runtime flags
#define LEDRF_UPDATE_LED				0x01


typedef struct ic74lv8153str_ {
	ChannelStr				*channel;					// Channel for driver UART
	uint8_t					resetpin;					// RESET pin of the 74LV8153
	uint8_t					soutpin[LED_DRIVER_COUNT];	// SOUT pins of the 74LV8153
	uint8_t					ackflags;					// Set these flags when each 74LV8153 acknowledges data by setting SOUT pin
	uint8_t 				leddata[LED_DRIVER_COUNT];	// LED realtime data
	uint8_t					rflags;						// Runtime flags
} ic74lv8153str;


extern ic74lv8153str	leddriver;


void leddrv_init();
void leddrv_mainproc();
void ledupdate();
void ledregclear();
