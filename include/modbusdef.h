/*
============================================================================
 Name        : Modbusdef.h
 Author      : AK
 Version     : V1.04
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Modbus communication protocol definitions

  Change log :

  *********V1.04 11/04/2019**************
  Fixed: Broadcast address changed to 0

  *********V1.03 03/06/2018**************
  AI modes changed

  *********V1.02 13/06/2015**************
  DI/AI/DO object processing modes added
  Application buffer byte offsets added

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
#define MODBUSOFFSET_ADDR				0
#define MODBUSOFFSET_FUNC				1
#define MODBUSOFFSET_DATA				2
#define MODBUSOFFSET_TCPSEQ				0
#define MODBUSOFFSET_TCPPROTID			2
#define MODBUSOFFSET_TCPLEN				4

#define MODBUS_HEADER_SIZE				2				// Message header size
#define MODBUS_SREQ_SIZE				4				// Single request message size
#define MODBUS_MREQ_RBCSIZE				5				// Multipe request message register value, register count and byte count size
#define MODBUS_CRC_SIZE					2				// Message CRC size
#define MODBUS_TCPHEADER_SIZE			6				// Modbus TCP header size
#define MODBUS_TCP_MAX_SIZE				260				// Maximal Modbus TCP ADU size (Header{6} + Device address{1} + PDU{253})
#define MODBUS_ADU_MAX_SIZE				256				// Maximal Modbus RTU ADU size (Device address{1} + PDU{253} + CRC{2})
#define MODBUS_CRC16_POLY				0xA001			// Modbus CRC16 polynomial
#define MODBUS_BROADCAST_ADDR			0				// Broadcast address
//#define MODBUS_UNUSED_ADDR				255				// Address used for Modbus TCP devices without Device address
#define MODBUS_MAX_REGISTER_COUNT		125				// Maximal number of registers per message

#define MBB_EXCEPTION					0x80			// Modbus exception bit


// Byte offsets in application buffer
// Byte {1}
#define MODBOFS_FUNC 			(1)
// Byte {2}
#define MODBOFS_DATA			(2)
#define MODBOFS_04BLEN			(2)
#define MODBOFS_XREGH			(2)
// Byte {3}
#define MODBOFS_XREGL			(3)
#define MODBOFS_11DATA			(3)
// Byte {4}
#define MODBOFS_04CNTH			(4)
#define MODBOFS_06DATH			(4)
#define MODBOFS_10CNTH			(4)
// Byte {5}
#define MODBOFS_04CNTL			(5)
#define MODBOFS_06DATL			(5)
#define MODBOFS_10CNTL			(5)
// Dynamic data offsets
#define MODBOFS_APPBYTES(moffset)	(2 + (moffset))
#define MODBOFS_04DATH(moffset)		(3 + (moffset))
#define MODBOFS_04DATL(moffset)		(4 + (moffset))
#define MODBOFS_10DATH(moffset)		(7 + (moffset))
#define MODBOFS_10DATL(moffset)		(8 + (moffset))


// Size definitions for Modbus protocols
typedef	uint8_t							Modaddr_t;		// Device address size definition
typedef	uint8_t							Modfunc_t;		// Modbus function size definition
typedef	uint8_t							Modreg8_t;		// Modbus register 8bit size definition
typedef	uint16_t						Modreg16_t;		// Modbus register 16bit size definition
typedef	uint8_t							Moddata8_t;		// Modbus data 8bit data size definition
typedef	uint16_t						Moddata16_t;	// Modbus data 16bit data size definition




enum {
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
} LEOPACK;


enum {
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
} LEOPACK;




// Input operation modes
// Last value is 0xFFFF because modes are
// mapped directly to Modbus registers and need to have 2 bytes
typedef enum {
	modbusdimden_spi					= 1,
	modbusdimden_undefined				= 0xFFFF
} LEOPACK modbusdimode_e;

// Input operation modes
typedef enum {
	modbusaimden_sint					= 1,
	modbusaimden_uintoffs				= 2,
	modbusaimden_undefined				= 0xFFFF
} LEOPACK modbusaimode_e;

// Output operation modes
typedef enum {
	modbusdomden_pulseout				= 1,
	modbusdomden_undefined				= 0xFFFF
} LEOPACK modbusdomode_e;


#endif /* MODBUSDEF_H_ */
