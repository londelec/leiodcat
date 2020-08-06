/*
============================================================================
 Name        : board.h
 Author      : AK
 Version     : V1.04
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for board hardware module

  Change log :

  *********V1.04 01/06/2018**************
  SPI configuration added

  *********V1.03 08/09/2016**************
  UART interface type added
  Local function prototypes removed

  *********V1.02 18/08/2015**************
  Default configuration pin and flag created
  DI and DO mode enums removed, now using ones defined in modbusdef.h
  Board UART setting structure created to enable configuration updates
  Board hardware profile table structure created

  *********V1.01 12/06/2015**************
  ADCB is used for temperature measurements and calculation function created
  EEPROM configuration read/write mask and update request bit sets added to board structure
  DI modes added

  *********V1.00 30/09/2014**************
  Initial revision

 ============================================================================
 */

#ifndef BOARD_H_
#define BOARD_H_


#include "ledefs.h"
#include "atmeldefs.h"
#include "modbusdef.h"
#include "leiodcat.h"


// ADC and channels
#define BOARD_ADC					ADCA			// Board ADC for temperature measurement, also used by powman
#define BOARD_VDDIOCH				BOARD_ADC.CH0	// ADC Channel for VDDIO measurement used by powman
#define TEMP_ADC					ADCB			// ADC for temperature measurement
#define TEMP_CHAN					TEMP_ADC.CH0	// ADC Channel for temperature measurement


// Board runtime flags
#define BOARDRF_EVENT				0x01
#define BOARDRF_UPDATE_LED			0x02
#define BOARDRF_EECONF_CORRUPTED	0x10			// EEPROM configuration is corrupted - CRC failed
#define BOARDRF_DEFCONF				0x20			// Default configuration pin is asserted


///#define bit_clear(p,m)  ((p) &= ~(1 << (m)))
//#define bit_set(p,m) 	((p) |= (1 << (m)))
//#define bit_get(p,m)	((p & (1<<(m))) && (1<<(m)))
//#define bit_toggle(p,m) (p ^= (1 << m))


typedef struct boardDI_s {
	uint16_t				*samples;					// Sampling counter of each input
	modbusdimode_e			*mode;						// Operation mode of each input
	uint16_t				*filterconst;				// Filter constant in miliseconds of each input
	uint8_t					*bitoffset;					// Bit offset of each input
	uint16_t				distates;					// Realtime DI states, mapped to MODBUS
	uint8_t					count;						// Number of DI objects
} boardDI_t;


typedef struct boardDO_s {
	uint16_t				*samples;					// Sampling counter of each output
	modbusdomode_e			*mode;						// Operation mode of each output
	uint16_t				*pulsedur;					// Output pulse duration in miliseconds
	uint8_t					*bitoffset;					// Bit offset of each output
	uint16_t				dostates;					// Realtime DO states, mapped to MODBUS
	uint16_t				updatereg;					// Mapped register for output update requests
	uint8_t					count;						// Number of DO objects
} boardDO_t;


typedef struct boardAI_s {
	modbusaimode_e			*mode;						// Operation mode of each input
	uint16_t				*uval;						// Unsigned integer value
	uint8_t					count;						// Number of DO objects
} boardAI_t;


typedef struct boarduartee_s {
	uint16_t 				bren16;						// Baudrate enum 16bit
	uint16_t				timeoutl;					// Timeout lowword ! Don't swap these, this will break EEPROM layout
	uint16_t				timeouth;					// Timeout highword ! Don't swap these, this will break EEPROM layout
	uint16_t				txdelayl;					// Tx delay lowword ! Don't swap these, this will break EEPROM layout
	uint16_t				txdelayh;					// Tx delay highword ! Don't swap these, this will break EEPROM layout
	uint16_t				t35;						// t35 timeout
	uint16_t				parity;						// Parity
	uint16_t 				devaddr;					// Device address
	uint16_t 				uartif;						// UART interface
} boarduartee_t;


typedef struct board_s {
	float 					caltempf;					// Temperature calibration float value, for scaled temperature calculation
	boardDI_t				*diptr;						// DI structure pointer, initialized if board has DIs
	boardDO_t				*doptr;						// DO structure pointer, initialized if board has DOs
	boardAI_t				*aiptr;						// AI structure pointer, initialized if board has AIs
	PORT_t					*ctrlport;					// MCU control port
	uint16_t				mapsize;					// Actual count of modbus mapped registers
	mcubitset_t				eeupdatebs;					// EEPROM configuration update bit set
	uint16_t				eesize;						// EEPROM configuration size
	uint16_t 				tempscaled; 				// Temperature value, scaled
	uint16_t 				caltemp85;					// Temperature calibration value from NVM
	uint16_t 				resetreg;					// Software reset register
	boarduartee_t	 		uartee;						// Main UART configuration
	uint8_t					rflags;						// Runtime flags
	uint8_t					ledoepin;					// Output enable (OE) pin of 74LV541
	uint8_t					defcfgpin;					// Default configuration pin
} board_t;


extern board_t boardio;


void board_mainproc(void);
void board_init(void);


#endif /* BOARD_H_ */
