/*
============================================================================
 Name        : Modbusdef.h
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Modbus communication protocol definitions

  Change log  :

  *********V1.01 24/02/2015**************
  16bit register and data definitions created
  Exception enums added

  *********V1.00 30/09/2014**************
  Initial revision

 ============================================================================
 */

#ifndef MODBUSDEF_H_
#define MODBUSDEF_H_


#include <stdint.h>

#include "ledefs.h"




// Rx/Tx buffer pointers
#define	MODBUSOFFSET_ADDR				0
#define	MODBUSOFFSET_FUNC				1
#define	MODBUSOFFSET_DATA				2
#define	MODBUSOFFSET_TCPSEQ				0
#define	MODBUSOFFSET_TCPPROTID			2
#define	MODBUSOFFSET_TCPLEN				4

#define	MODBUS_HEADER_SIZE				2				// Message header size
#define	MODBUS_SREQ_SIZE				4				// Single request message size
#define	MODBUS_CRC_SIZE					2				// Message CRC size
#define	MODBUS_TCPHEADER_SIZE			6				// Modbus TCP header size
#define	MODBUS_CRC16_POLY				0xA001			// Modbus CRC16 polynomial
#define	MODBUS_GLOBAL_ADDR				255				// Global address
#define MODBUS_MAX_REGISTER_COUNT		125				// Maximal number of registers per message

#define	MBB_EXCEPTION					0x80			// Modbus exception bit




// Size definitions for Modbus protocols
typedef	uint8_t							DevAddrDef;		/* Device address size definition */
typedef	uint8_t							ModFuncDef;		/* Modbus function size definition */
typedef	uint8_t							ModReg8bitDef;	/* Modbus register 8bit size definition */
typedef	uint16_t						ModReg16bitDef;	/* Modbus register 16bit size definition */
typedef	uint8_t							ModData8bitDef;	/* Modbus data 8bit data size definition */
typedef	uint16_t						ModData16bitDef;/* Modbus data 16bit data size definition */
typedef	uint8_t							ModMsgDef;		/* Modbus message count size definition */
typedef	uint16_t						ModCRCDef;		/* Modbus CRC size definition */




typedef enum {
	MODFUNC_01							= 1,		// Read Coil Status
	MODFUNC_02  						= 2,		// Read Input Status
	MODFUNC_03							= 3,		// Read Holding Registers
	MODFUNC_04  						= 4,		// Read Input Registers
	MODFUNC_05							= 5,		// Force Single Coil
	MODFUNC_06							= 6,		// Preset Single Register
	MODFUNC_07							= 7,		// Read Exception Status
	MODFUNC_08							= 8,		// Diagnostics

	MODFUNC_0B							= 11,		// Fetch Comm. Event Ctr
	MODFUNC_0C							= 12,		// Fetch Comm. Event Log
	MODFUNC_0D							= 13,		// Program Controller
	MODFUNC_0E							= 14,		// Poll Controller
	MODFUNC_0F							= 15,		// Force Multiple Coils
	MODFUNC_10							= 16,		// Preset Multiple Registers
	MODFUNC_11							= 17,		// Report Slave ID

	MODFUNC_13							= 19,		// Reset Comm. Link
	MODFUNC_14							= 20,		// Read General Reference
	MODFUNC_15							= 21,		// Write General Reference
	MODFUNC_16							= 22,		// Mask Write Register
	MODFUNC_17							= 23,		// Read/Write Multiple registers
	MODFUNC_18							= 24,		// Read FIFO Queue

	MODFUNC_2B							= 43,		// Encapsulated Interface Transport
} LEOPACK MODBUS_FUNC;


typedef enum {
	MODEX_ILLEGAL_FUNC					= 1,
	MODEX_ILLEGAL_DATA_ADDR 			= 2,
	MODEX_ILLEGAL_DATA_VAL				= 3,
	MODEX_SERVER_DEV_FAILURE			= 4,
	MODEX_ACKNOWLEDGE					= 5,
	MODEX_SERVER_DEV_BUSY				= 6,
	MODEX_NEGATIVE_ACK					= 7,
	MODEX_MEM_PARITY_ERR				= 8,
	MODEX_GW_PATH_UNAVAILABLE			= 10,
	MODEX_GW_TARGET_DEV_NORESP			= 11,
} LEOPACK MODBUS_EXCPT;


#endif /* MODBUSDEF_H_ */
