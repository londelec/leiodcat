/*
============================================================================
 Name        : Modbus.h
 Author      : AK
 Version     : V2.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Modbus RTU communication protocol link layer module

  Change log :

  *********V2.00 29/03/2019**************
  Modbus TCP support added

  *********V1.02 18/08/2016**************
  Local function prototypes removed

  *********V1.01 20/08/2015**************
  Automatic t35 timeout calculation constant added

  *********V1.00 30/09/2014**************
  Initial revision

 ============================================================================
 */

#ifndef MODBUS_H_
#define MODBUS_H_


#include <stdint.h>

#include "modbusdef.h"


// Compile options
#define MODBUS_NONE			0		// Exclude protocol
#define MODBUS_MCU			1		// Compile for MCU
#define MODBUS_GENERIC		2		// Generic version

// Selected protocol options
#ifdef MCUTYPE
#define MODBUSSL_TYPE	MODBUS_MCU
#define MODBUSMA_TYPE	MODBUS_NONE
#include "leiodcat.h"
#else
#define MODBUSSL_TYPE	MODBUS_GENERIC
#define MODBUSMA_TYPE	MODBUS_GENERIC
#include "leandc.h"
#endif


// Modbus frame formats
typedef enum {
	ModbusRTU = 1,				// Modbus RTU
	ModbusTCP,					// Modbus TCP
	ModbusASCII,				// Modbus ASCII
} Modbusframe_e;


typedef struct Modbus_shlink_s {
	uint8_t					*rxbuff;				// Receive buffer
	uint8_t					*txbuff;				// Transmit buffer
	fddef					shfd;					// fd shared between stations
	timerconst_t			rxtimeout35;			// Start received message processing if no char is received within this timeout (in 10us)
	Modbusframe_e			fformat;				// Modbus frame format
	union {
		txrx16_t			rxbytecnt;				// Rx byte count during received message decoding for ModbusRTU and Modbus PDU size for ModbusTCP
		txrx16_t			txapplen;				// Length of the PDU + address used for Modbus Slave only
	};
	seqno_t					tcpseq;					// TCP message sequence ID
	//Modaddr_t				rxdevaddr;				// Received Device address
} Modbus_shlink_t;


#if MODBUSMA_TYPE
chret_e Modbusma_rx_check(station_t *staptr, txrx8_t *datalength);
#endif
chret_e Modbussl_rx_check(station_t *staptr, txrx8_t *datalength, int *enabledf);
void Modbus_send(station_t *staptr, txrx8_t applen);
chret_e Modbus_receive(STAARG_RX);
void Modbus_timeout(station_t *staptr);
int Modbus_create(station_t *staptr, channel_t *chanptr);
void Modbus_postinit(station_t *staptr);

#endif /* MODBUS_H_ */
