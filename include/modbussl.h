/*
============================================================================
 Name        : Modbussl.h
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Modbus ASCII/RTU/TCP communication protocol slave module

  Change log  :

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
//#include "main.h"
//#include "lecommands.h"
//#include "realtime.h"
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




/*typedef struct Modbusma_objmsglink_ {
	uint16_t				bitoffset;					// Bit offset {16}
	ObjectTypeEnum			objtype;					// Object type {8}
	void					*private;					// Private object pointer {32}
	struct Modbusma_objmsglink_ *next;					// Next chain object {32}
} LEOPACK Modbusma_objmsglink;


typedef struct Modbusma_DIprivate_ {
	RealtimeDI				*rtDI;						// Realtime object pointer {32}
	Qual_8bit				qualifier;					// Internal qualifier {8}
} LEOPACK Modbusma_DIprivate;


typedef struct Modbusma_AIprivate_ {
	IECCoeffDef				coeff;						// Coefficient {32}
	AIthresholdStr			thresholds;					// Deadband {32} and Percent {32}
	AIscalingStr			*scaling;					// Scaling structure pointer {32}
	RealtimeAI				*rtAI;						// Realtime object pointer {32}
	Qual_8bit				qualifier;					// Internal qualifier {8}
} LEOPACK Modbusma_AIprivate;


typedef struct Modbusma_CTprivate_ {
	InfAddrDef				infaddr;					// Information address {32}
	IECCounterDef			deadband;					// Deadband {32}
	RealtimeCT				*rtCT;						// Realtime object pointer {32}
	Qual_8bit				qualifier;					// Internal qualifier {8}
} LEOPACK Modbusma_CTprivate;


typedef struct Modbusma_DOprivate_ {
	struct Modbusmsg_		*message;					// Request message {32}
	uint16_t				bitoffset;					// Bit offset {16}
	Qual_8bit				qualifier;					// Internal qualifier {8}
} LEOPACK Modbusma_DOprivate;


typedef struct Modbusma_AOprivate_ {
	struct Modbusmsg_		*message;					// Request message {32}
	IECCoeffDef				coeff;						// Coefficient {32}
	Qual_8bit				qualifier;					// Internal qualifier {8}
} LEOPACK Modbusma_AOprivate;


typedef struct Modbusma_control_ {
	NanoTimeDef				cmdtimer;					// DO/AO command expiration timer {64 or even 128}
	struct Modbusmsg_		*message;					// Request message {32}
	IECFloatDef				setpoint;					// Original Setpoint object value {32}
	ObjectCntDef			index;						// Object index {32}
	uint16_t				bitoffset;					// Bit offset {16}
	cmdstMasterEnum			state;						// DO command execution state {8}
} Modbusma_control;


typedef struct Modbusmalinks_ {
	Modbusma_objmsglink		*DIlinks;					// DI object XML links
	Modbusma_objmsglink		*AIlinks;					// AI object XML links
	Modbusma_objmsglink		*CTlinks;					// CT object XML links
	Modbusma_objmsglink		*DOlinks;					// DO object XML links
	Modbusma_objmsglink		*AOlinks;					// AO object XML links
} Modbusmalinks;


typedef struct Modbusmsg_ {
	//uint8_t					priority;					// Modbus message priority
	//TimerConstDef			interval;					// Message sending interval
	//NanoTimeDef				msgtimer;					// Message timer value, generate message after expiration
	ModFuncDef				func;						// Modbus function
	uint8_t					datlength;					// Outgoing data length
	uint8_t					*data;						// Outgoing data buffer
	Modbusma_objmsglink		*objlink;					// First linked object pointer
	struct Modbusmsg_		*next;						// Next Modbus message
} Modbusmsg;
*/


typedef struct ModbusRegStr_ {
	ModReg16bitDef			reg;						// Base register
	ModReg8bitDef			*rddata;					// Read data pointer
	ModReg8bitDef			*wrdata;					// Write data pointer
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





// Version string made public to allow access from main
//extern const lechar *ModbusslVersion;


Modbussl_applayer *Modbussl_preappinit(void);
void Modbussl_postappinit(Modbussl_applayer *applayer, uint8_t mapsize);
void Modbussl_regmeminit(Modbussl_applayer *applayer, uint8_t mapsize);
uint8_t Modbussl_appprocess(Modbussl_applayer *applayer, uint8_t *rxtxbuff);
uint8_t Modbussl_rdslaveid(uint8_t *txbuff);
uint8_t Modbussl_searchreg(Modbussl_applayer *applayer, ModReg16bitDef regaddr, uint16_t *memoffset);
uint8_t Modbussl_validregcnt(Modbussl_applayer *applayer, ModReg16bitDef regaddr, ModReg16bitDef regcount, uint16_t memoffset);

uint8_t Modbussl_message(uint8_t *txbuff, ModbusRegStr *regptr, uint8_t count);
uint8_t Modbussl_eeblock(uint8_t *txbuff, ModReg16bitDef reg, uint8_t count);
uint8_t Modbussl_exception(uint8_t *txbuff, ModData8bitDef excpt);


#endif /* MODBUSSL_H_ */
