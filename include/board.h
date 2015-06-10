/*
============================================================================
 Name        : board.h
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for board hardware module

  Change log  :

  *********V1.00 30/09/2014**************
  Initial revision

 ============================================================================
 */

#ifndef BOARD_H_
#define BOARD_H_


#include "ledefs.h"




// IO definitions
#define DI_DEFAULT_FILTER_MSEC		50 		// default 50ms
#define DO_DEFAULT_HOLD_MSEC		1500 	// default 1.5sec
//#define DO_HOLD_TIME_MIN			500
//#define DO_HOLD_TIME_MAX			5000

#define VPORT_BASE 					VPORT0

// Automatic hardware identification constants
#define BOARD_AUTOID_MCUPORT		PORTE	// MCU port where hardware identification resistors are
#define BOARD_AUTOID_MASK			0x3F	// Mask pins which are not used for ID resistors


// ADC and channel
#define BOARD_ADC					ADCA			// Board ADC for temperature measurement, also used by powman
#define BOARD_TEMPCH				BOARD_ADC.CH0	// ADC Channel for temperature measurement
#define BOARD_VDDIOCH				BOARD_ADC.CH1	// ADC Channel for VDDIO measurement used by powman


// Board runtime flags
#define BOARDRF_EVENT				0x01
#define BOARDRF_UPDATE_LED			0x02


///#define bit_clear(p,m)  ((p) &= ~(1 << (m)))
//#define bit_set(p,m) 	((p) |= (1 << (m)))
//#define bit_get(p,m)	((p & (1<<(m))) && (1<<(m)))
//#define bit_toggle(p,m) (p ^= (1 << m))


// Output operation policies
// Last value is 0xFFFF because policies are
// mapped to Modbus registers and need to be 2 bytes
typedef enum {
	dopolen_holdperiod				= 1,
	dopolen_undefined				= 0xFFFF
} LEOPACK dopolicyenum;


typedef struct boardDIstr_ {
	uint16_t				*samples;					// Sampling counter of each input
	uint16_t				*filterconst;				// Filter constant in miliseconds of each input
	uint8_t					*bitoffset;					// Bit offset of each input
	uint16_t				distates;					// Realtime DI states, mapped to MODBUS
	uint8_t					count;						// Number of DI objects
} boardDIstr;


typedef struct boardDOstr_ {
	uint16_t				*samples;					// Sampling counter of each output
	uint16_t				*holdperiod;				// Output activated hold period in miliseconds
	dopolicyenum			*policy;					// Operation policy of each output
	uint8_t					*bitoffset;					// Bit offset of each output
	uint16_t				dostates;					// Realtime DO states, mapped to MODBUS
	uint16_t				updatereg;					// Mapped register for output update requests
	uint8_t					count;						// Number of DO objects
} boardDOstr;


typedef struct BOARD_t {
	uint8_t					rflags;						// Runtime flags
	boardDIstr				*diptr;
	boardDOstr				*doptr;
	uint8_t					ledoepin;					// Output enable (OE) pin of 74LV541
	PORT_t					*ctrlport;					// MCU control port
	uint16_t				mapsize;					// Actual count of modbus mapped registers
} BOARD_t;


extern BOARD_t	boardio;
extern uint16_t	caltemp85;


void board_init();
void initboardDI(uint8_t dicount, uint8_t baseoffset);
void initboardDO(uint8_t docount, uint8_t baseoffset);
void board_mainproc();
uint8_t checkotherdoactive(boardDOstr *doptr, uint8_t doindex);
void activateDO(boardDOstr *doptr, uint8_t doindex);
void releaseDO(boardDOstr *doptr, uint8_t doindex);

void boardisr_1ms();

#endif /* BOARD_H_ */
