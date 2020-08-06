/*
 ============================================================================
 Name        : 74lv8153.h
 Author      : AK
 Version     : V1.02
 Copyright   : Property of Londelec UK Ltd
 Description : Header file LED driver module

  Change log :

  *********V1.02 04/07/2019**************
  Station communication structure created

  *********V1.01 08/09/2016**************
  Local function prototypes removed

  *********V1.00 25/02/2015**************
  Initial revision

 ============================================================================
 */

#ifndef H74LV8153_H_
#define H74LV8153_H_


#include "leiodcat.h"


#define LED_PORT_INT_VECT				PORTC_INT1_vect	// port interrupt vector
#define LED_DRIVER_COUNT				2				// Number of 74LV8153 chips on board

#define LED_RESET_HOLD					10				// RESET pin asserted holding period (x100usec), default 1msec

// LED runtime flags
#define LEDRF_UPDATE_LED				0x01


typedef struct ic74lv8153_s {
	channel_t				*channel;					// Channel for driver UART
	stacom_t				stacoms;					// Station communication structure
	uint8_t					resetpin;					// RESET pin of the 74LV8153
	uint8_t					soutpin[LED_DRIVER_COUNT];	// SOUT pins of the 74LV8153
	uint8_t					ackflags;					// Set these flags when each 74LV8153 acknowledges data by setting SOUT pin
	uint8_t 				leddata[LED_DRIVER_COUNT];	// LED realtime data
	uint8_t					rflags;						// Runtime flags
} ic74lv8153_t;


extern ic74lv8153_t leddriver;


void leddrv_init(void);
void leddrv_mainproc(void);
//void ledregclear(void);


#endif /* H74LV8153_H_ */
