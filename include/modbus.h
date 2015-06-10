/*
============================================================================
 Name        : Modbus.h
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Modbus RTU communication protocol link layer module

  Change log  :

  *********V1.00 30/09/2014**************
  Initial revision

 ============================================================================
 */

#ifndef MODBUS_H_
#define MODBUS_H_


#include <stdint.h>

#include "main.h"
#include "modbusdef.h"
#include "modbussl.h"




#define	MODBUS_RXBUFF_SIZE				512				// Total Rx buffer size including link layer framing
#define	MODBUS_TXBUFF_SIZE				256				// Total Tx buffer size including link layer framing

#define	MODBUS_DEFAULT_NORESPCNT		5				// Default Master No Response counter value
#define	MODBUS_DEFAULT_DEGRADEDRETRIES	5				// Default Master Degraded retry counter value
#define	MODBUS_DEFAULT_DEGRADEDTIMEOUT	300				// Default Master Degraded timeout value in seconds
#define MODBUS_RXT35					100				// Default Modbus Rx idle timer in 100usec (10ms)




// Modbus frame formats
typedef enum {
	ModbusRTU							= 1,			// Modbus RTU
	ModbusTCP,											// Modbus TCP
	ModbusASCII,										// Modbus ASCII
} LEOPACK ModbusframeEnum;


// ModbusTCP Rx states
typedef enum {
	Modbusrxheader						= 0,			// Modbus TCP header
	Modbusrxpayload,									// Modbus TCP payload
} LEOPACK ModbusrxstateEnum;


// Communication states
typedef enum {
	Modbuslinkok						= 0,			// Communication link is ok
	Modbusrepeatlast									// Repeat last sent message
} LEOPACK ModbusCommsEnum;




typedef struct  Modbus_shared_linklayer_ {
	uint8_t						*rxbuff;				// Receive buffer
	uint16_t					expectedrxbytes;		// Expected data length in Rx Buffer
	ModbusrxstateEnum			rxstate;				// Message receiving state
	TxRx16bitDef				rxapplen;				// Rx Application layer buffer length in bytes
	DevAddrDef					rxdevaddr;				// Received Device address
	TxRx16bitDef				rxbytecnt;				// Rx byte count during received message decoding
	TimerConstDef				rxtimeout35;			// Start received message processing if no char is received within timeout (in 10us)
	ModbusframeEnum				fformat;				// Modbus frame format
	uint8_t						appoffset;				// Application buffer offset
	uint8_t						charmult;				// Character size multiplier
	uint16_t					crc;					// Calculated checksumm byte
}  Modbus_shared_linklayer;


typedef struct Modbus_linklayer_ {
	DevAddrDef					devaddr;				// Device address
	TxRx8bitDef					txapplen;				// Tx Application layer buffer length in bytes
	uint16_t					tcpseq;					// TCP message sequence ID
	ModbusCommsEnum				commsstate;				// Communications state
	//Flags_8bit					linkflags;				// Link layer flags
	//Flags_8bit					linkxmlflags;			// Link layer configuration flags
} Modbus_linklayer;


typedef struct Modbussl_pointers_ {
	//uint8_t						*txbuff;				// Transmission buffer
	Modbus_linklayer			*linklayer;				// Link layer pointer
	Modbus_shared_linklayer		*sharedlink;			// Pointer to shared link layer structure
	Modbussl_applayer			*applayer;				// Application layer pointer
} Modbussl_pointers;






// Version string made public to allow access from main
extern const lechar *ModbusVersion;

void Modbussl_preinit(GenProtocolStr *genprotocol, DevAddrDef devaddr);
Modbus_shared_linklayer *Modbus_initshared(StatStr *currentsta);
uint8_t Modbussl_postinit(GenProtocolStr *genprotocol, TimerConstDef t35, uint8_t mapsize);

CHStateEnum Modbus_receive(DRVARGDEF_RX);
void buildCRC16 (uint16_t *crc, uint8_t databyte);
CHStateEnum Modbussl_analyze(StatStr *staptr);

void Modbussl_commserr(DRVARGDEF_COMMSERR);
CHStateEnum Modbussl_process(DRVARGDEF_MAINPROC);

#endif /* MODBUS_H_ */
