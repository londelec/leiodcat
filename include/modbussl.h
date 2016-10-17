/*
============================================================================
 Name        : Modbussl.h
 Author      : AK
 Version     : V1.03
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Modbus ASCII/RTU/TCP communication protocol slave module

  Change log :

  *********V1.03 07/09/2016**************
  Local function prototypes removed

  *********V1.02 18/08/2015**************
  Mapping write data pointer is 16bit pointer

  *********V1.01 12/06/2015**************
  New register search and count validation functions created

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */

#ifndef MODBUSSL_H_
#define MODBUSSL_H_


#include <stdint.h>

#include "ledefs.h"
#include "modbusdef.h"




// Default base addresses
// Used by Modbus Master
#define	MODBUSSL_EEBASE					0x9000			// Modbus register address for EEPROM mapping
#define	MODBUSSL_EESIZE					MCUEE_EESIZE	// Modbus EEPROM mapping size (half of the actual size because of 16bit registers)
#define	MODBUSSL_USERMAPSIZE			0x1000			// Modbus user mapping size



// Internal qualifier flags
#define	MODBSXMLF_DISABLED				0x80			// Disabled flag for internal DI, AI, DO qualifiers


// Event logfile flags ->Flags
#define	LOGMBFLAG_CMD					0x04			// Record only control commands and negative replies to Event file
#define	LOGMBFLAG_CMDINFO				0x08			// Record control command error and info messages to Event file




typedef struct ModbusRegStr_ {
	ModReg16bitDef			reg;						// Base register
	ModReg8bitDef			*rddata;					// Read data pointer 8bit
	ModReg16bitDef			*wrdata;					// Write data pointer 16bit
} ModbusRegStr;


typedef struct Modbussl_applayer_ {
	//Flags_8bit				xmlflags;					// Flags from XML configuration by bits
	//Flags_8bit				appflags;					// Communication state force flags by bits
	//IECFloatDef				commonAIdeadband;			// Common Deadband
	//IECFloatDef				commonAIpercent;			// Common Percent
	//ObjectCntDef			objcount[objtypecount];		// DI/AI/CT/DO/AO object count
	//DOMatrixDef				cmdmxsize;					// DO Matrix size
	//cmdMatrixStr			*cmdmatrix;					// DO Matrix
	//TimerConstDef			apptsec;					// Application Timeout in seconds from XML configuration
	//TimerConstDef			selecttsec;					// Select Timeout in seconds from XML configuration
	//TimerConstDef			commandtsec;				// Command expiration Timeout in seconds from XML configuration
	//Modbusma_DIprivate		*DItable;					// DI object Table
	//Modbusma_AIprivate		*AItable;					// AI object Table
	//Modbusma_CTprivate		*CTtable;					// CT object Table
	//Modbusma_DOprivate		*DOtable;					// DO private table
	//Modbusma_AOprivate		*AOtable;					// AO private table
	//GenObjectStr			*objecttable;				// Pointer to General object table
	//Logfile_structure		*eventlog;					// Event Logfile structure pointer
	//ModbusEvLogVars			evlogvar;					// Event Logger variables
	//Modbusmalinks			*links;						// ASDU only for object linking
	//ModbusmastateEnum		devstate;					// Device operation state
	//Modbusmsg				*currentreq;				// Current outgoing message
	//Modbusmsg				*generalreq;				// General outgoing message buffer
	//Modbusmsg				*initreq;					// Initialization message buffer
	ModReg16bitDef			eemapbase;					// EEPROM data mapping base register
	ModReg16bitDef			eemapsize;					// EEPROM data mapping size
	ModbusRegStr			*regmem;					// Register memory
	ModReg16bitDef			regcount;					// Register count
} Modbussl_applayer;


uint8_t Modbussl_appprocess(Modbussl_applayer *applayer, uint8_t *rxtxbuff);
Modbussl_applayer *Modbussl_preappinit(void);
void Modbussl_postappinit(Modbussl_applayer *applayer, uint8_t mapsize);


#endif /* MODBUSSL_H_ */
