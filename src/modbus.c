/*
 ============================================================================
 Name        : Modbus.c
 Author      : AK
 Version     : V2.01
 Copyright   : Property of Londelec UK Ltd
 Description : Modbus RTU communication protocol link layer module

  Change log :

  *********V2.01 14/01/2020**************
  Compatibility with Modbus Slave improved

  *********V2.00 29/03/2019**************
  Modbus TCP support added

  *********V1.12 21/02/2019**************
  Channel station link function generalized

  *********V1.11 10/02/2019**************
  Tx buffer pointer and length moved to channel structure
  CRC16 calculate function moved from leandc.c

  *********V1.10 28/11/2017**************
  Socket initialization enabled

  *********V1.09 17/05/2017**************
  Fixed: Rx channel log fixed and invalid CRC message added to logs

  *********V1.08 04/03/2017**************
  Dynamic TxDelay created

  *********V1.07 13/08/2016**************
  Local functions marked static
  Online check function generalized
  Get information address function removed, generalized in relatime.c
  XMLmodbus.h include removed

  *********V1.06 09/04/2016**************
  Fixed: If received message analyze fails, just keep trying to analyze until timeout expires

  *********V1.05 20/08/2015**************
  t35 timeout validation created

  *********V1.04 13/06/2015**************
  Fixed: New station pointer is returned by main protocol processing function when next station is selected
  Station initialization/generic communication mode is indicated using service index -4 (servDI104started)
  Character multiplier argument removed from application layer functions
  Exported flags changed to states
  Received byte service counter added
  Poll cycle service AI calculation added

  *********V1.03 27/02/2015**************
  Outgoing modbus function 0x11 added

  *********V1.02 12/02/2015**************
  Changes related to marking flag addition to exported qualifier

  *********V1.01 30/11/2014**************
  Fixed: Reset comms state when message received

  *********V1.00 30/09/2014**************
  Initial revision

 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "modbus.h"
//#include "modbusdef.h"


const lechar ModbusVersion[] = " ModbusVersion=2.01 ";


#ifdef GLOBAL_DEBUG
#define SLAVE_T35	10000
//#define MODBUS_DEFAULT_OFFLINEDELAY		10			// Default Slave Offline delay timer value in seconds
//#define DEBUG_LOGEVERYTIMEOUT
#endif	// GLOBAL_DEBUG


// Buffer sizes
#define MODBUS_RXBUFF_SIZE				272			// Rx buffer size including link layer framing and TCP header
#define MODBUS_TXBUFF_SIZE				272			// Tx buffer size including link layer framing and TCP header

// Timing constants
#define MODBUS_DEFAULT_NORESPCNT		5			// Default Master No Response counter value
#define MODBUS_DEFAULT_OFFLINEDELAY		60			// Default Slave Offline delay timer value in seconds
#define MODBUS_DEFAULT_DEGRADEDRETRIES	5			// Default Master Degraded retry counter value
#define MODBUS_DEFAULT_DEGRADEDTIMEOUT	300			// Default Master Degraded timeout value in seconds
#define MODBUS_RXT35CONST				3500000		// Default Modbus Rx idle timeout conversion constant in 10usec


// Macros
#define MODBUS_CRC_CALC(mdata) calc_crc16(&crc, MODBUS_CRC16_POLY, mdata);

#if ((MODBUSMA_TYPE == MODBUS_GENERIC) || (MODBUSSL_TYPE == MODBUS_GENERIC))
#include "realtime.h"

static const lechar clogrxlenoverflow[] = "Received message length (%u) exceeds limit of (%u) bytes";
static const lechar clogrxlenincorrect[] = "Received message length (%u) is incorrect, (%u) bytes expected";

#define COMMS_RXLOG(mlength, moffs)\
		if ((staptr->logfile) && (staptr->logfile->lflags & CLOGXF_INCOMING))\
			commslogger(sharedlink->rxbuff + moffs, mlength, NULL, staptr->logfile, clogenRX);

#define COMMS_RXHWLOG(mlength, moffs) stacom_clog(stacoms, CLOGXF_INCOMING, sharedlink->rxbuff + moffs, mlength, clogenRX);

#define COMMS_TXLOG\
		if ((staptr->logfile) && (staptr->logfile->lflags & CLOGXF_OUTGOING))\
			commslogger(txptr, chanptr->txlen, NULL, staptr->logfile, clogenTX);\
		stacom_clog(stacoms, CLOGXF_OUTGOING, txptr, chanptr->txlen, clogenTX);

#define COMMS_RXERRLOG(...) station_rxclog(staptr, CLOGXF_ERROR, (sharedlink->fformat == ModbusTCP) ? (sharedlink->rxbuff) : (sharedlink->rxbuff + MODBUS_TCPHEADER_SIZE), sharedlink->rxbytecnt, clogenERR, __func__, __FILE__, __LINE__, __VA_ARGS__);
#define COMMS_TCPERRLOG(mlen, ...) station_rxclog(staptr, CLOGXF_ERROR, sharedlink->rxbuff, mlen, clogenERR, __func__, __FILE__, __LINE__, __VA_ARGS__);
#else
#define COMMS_RXLOG(mlength, moffs)
#define COMMS_RXHWLOG(mlength, moffs)
#define COMMS_TXLOG
#define COMMS_RXERRLOG(...)
#define COMMS_TCPERRLOG(mlen, ...)
#endif




/*
 * Check length and CRC of the received Modbus RTU message
 * [10/04/2019]
 */
static int rx_crc_check(station_t *staptr, uint8_t *rxbuff) {
	Modbus_shlink_t			*sharedlink = staptr->stacoms->priv;
	uint16_t				i;
	uint16_t 				crc = 0xffff;


	for (i = 0; i < (sharedlink->rxbytecnt - MODBUS_CRC_SIZE); i++) {
		MODBUS_CRC_CALC(rxbuff[i]);
	}


	if (
			((crc >> 8) != rxbuff[sharedlink->rxbytecnt - 1]) ||
			((crc & 0xFF) != rxbuff[sharedlink->rxbytecnt - 2])) {
		COMMS_RXERRLOG("CRC error (0x%02X 0x%02X was expected)", crc & 0xFF, crc >> 8);
		return LE_FAIL;
	}
	return LE_OK;
}


/*
 * Received PDU size check
 * [08/05/2019]
 */
static chret_e pdu_size_check(station_t *staptr, txrx8_t datalength) {
	Modbus_shlink_t			*sharedlink = staptr->stacoms->priv;
	uint8_t					*rxbuff = sharedlink->rxbuff + MODBUS_TCPHEADER_SIZE;


	switch (sharedlink->fformat) {
	case ModbusTCP:
		if ((sharedlink->rxbytecnt - MODBUS_HEADER_SIZE) != datalength) {
			COMMS_TCPERRLOG(sharedlink->rxbytecnt + MODBUS_TCPHEADER_SIZE, clogrxlenincorrect,
					(sharedlink->rxbytecnt + MODBUS_TCPHEADER_SIZE),
					datalength + MODBUS_HEADER_SIZE + MODBUS_TCPHEADER_SIZE);
			return chret_dataerror;
		}
		break;


	case ModbusASCII:
		// TODO data length validation
		break;


	case ModbusRTU:
		if ((sharedlink->rxbytecnt - (MODBUS_HEADER_SIZE + MODBUS_CRC_SIZE)) < datalength)
			return chret_empty;		// Not enough bytes received, keep waiting, applies only to Modbus RTU
		else if ((sharedlink->rxbytecnt - (MODBUS_HEADER_SIZE + MODBUS_CRC_SIZE)) > datalength) {
			COMMS_RXERRLOG(clogrxlenincorrect,
					sharedlink->rxbytecnt,
					datalength + (MODBUS_HEADER_SIZE + MODBUS_CRC_SIZE));
			return chret_dataerror;
		}

		if (rx_crc_check(staptr, rxbuff) == LE_FAIL)
			return chret_dataerror;
		break;


	default:	// Unknown frame format
		return chret_dataerror;
	}
	return chret_rxtx;
}


#if MODBUSMA_TYPE
/*
 * Analyze Modbus message
 * [07/11/2014]
 * Fixed: Reset comms state when message received
 * [30/11/2014]
 * Received byte service counter added
 * [28/06/2015]
 * Fixed: Rx channel log fixed
 * [17/05/2017]
 * Modbus TCP support added
 * [08/05/2019]
 * Station online function used
 * [13/02/2020]
 */
chret_e Modbusma_rx_check(station_t *staptr, txrx8_t *datalength) {
	stacom_t			*stacoms = staptr->stacoms;
	//channel_t			*chanptr = stacoms->chaninst;
	Modbus_shlink_t		*sharedlink = stacoms->priv;
	uint8_t				*rxbuff = sharedlink->rxbuff + MODBUS_TCPHEADER_SIZE;
	uint16_t			loglen = sharedlink->rxbytecnt;
	uint8_t				boffs = MODBUS_TCPHEADER_SIZE;


	*datalength = 0;

	switch (sharedlink->fformat) {
	case ModbusTCP:
		loglen += MODBUS_TCPHEADER_SIZE;
		boffs = 0;
		*datalength = sharedlink->rxbytecnt - MODBUS_HEADER_SIZE;		// Data length for functions without length field
		break;

	case ModbusASCII:
		// TODO need to figure out at which point to convert from ASCII to HEX
		break;

	case ModbusRTU:
		if (sharedlink->rxbytecnt < (MODBUS_HEADER_SIZE + MODBUS_CRC_SIZE))
			return chret_empty;		// Not enough bytes received, keep waiting, applies only to Modbus RTU
		*datalength = sharedlink->rxbytecnt - MODBUS_HEADER_SIZE - MODBUS_CRC_SIZE;		// Data length for functions without length field
		break;

	default:	// Unknown frame format
		goto recv_error;
	}


	if (rxbuff[MODBOFS_FUNC] & MBB_EXCEPTION) {
		*datalength = 1;		// Exception message has 1 byte
	}
	else {
		switch (rxbuff[MODBOFS_FUNC]) {
		case MODFUNC_01:
		case MODFUNC_02:
		case MODFUNC_03:
		case MODFUNC_04:
		case MODFUNC_0C:
		case MODFUNC_11:
		case MODFUNC_14:
		case MODFUNC_15:
		case MODFUNC_17:
			*datalength = rxbuff[MODBOFS_DATA] + 1;
			break;

		case MODFUNC_05:
		case MODFUNC_06:
		case MODFUNC_0B:
		case MODFUNC_0F:
		case MODFUNC_10:
			*datalength = 4;
			break;

		case MODFUNC_07:
		case MODFUNC_08:
		case MODFUNC_2B:
			break;

		case MODFUNC_16:
			*datalength = 6;
			break;

		case MODFUNC_18:
			// Data length is two bytes, however we assume the highbyte 0
			*datalength = rxbuff[MODBOFS_DATA + 1] + 2;
			break;

		default:	// Ignore unknown functions
			break;
		}
	}


	switch (pdu_size_check(staptr, *datalength)) {
	case chret_empty:
		return chret_empty;	// Not enough bytes received, keep waiting, applies only to Modbus RTU

	case chret_dataerror:
		goto recv_error;

	default:
		break;
	}


	COMMS_RXHWLOG(loglen, boffs)	// Channel comms logger


	if (rxbuff[MODBUSOFFSET_ADDR] != staptr->address) {
		COMMS_RXERRLOG("Incorrect device address (%u) received, (%u) was expected", rxbuff[MODBUSOFFSET_ADDR], staptr->address);
		goto recv_error;
	}


	if ((rxbuff[MODBOFS_FUNC] & ~MBB_EXCEPTION) != sharedlink->txbuff[MODBUS_TCPHEADER_SIZE + MODBUSOFFSET_FUNC]) {	// Unexpected function received
		COMMS_RXERRLOG("Unexpected Modbus function (%u) received, (%u) was expected",
				(rxbuff[MODBOFS_FUNC] & ~MBB_EXCEPTION),
				sharedlink->txbuff[MODBUS_TCPHEADER_SIZE + MODBUSOFFSET_FUNC]);
		goto recv_error;
	}


	if (staptr->runflags & SERVICE_COMMSDISABLED)
		goto recv_error;		// Current station disabled, change state to flush and await for timeout to occur


	COMMS_RXLOG(loglen, boffs)		// Station comms logger

	SERVF_RXCNT(sharedlink->rxbytecnt)	// Update Rx message and byte counters
	sharedlink->rxbytecnt = 0;			// Reset receive pointer after logfile function call
	mastersta_reset_cnt(staptr);

	station_online_make(staptr, 1);
	return chret_rxtx;


	recv_error:
	staptr->stacoms->serstate = ser_flush;
	return chret_dataerror;
}
#endif


/*
 * Check received Modbus Slave message
 * [08/05/2019]
 */
chret_e Modbussl_rx_check(station_t *staptr, txrx8_t *datalength, int *enabledf) {
	stacom_t			*stacoms = staptr->stacoms;
	//channel_t			*chanptr = stacoms->chaninst;
	Modbus_shlink_t		*sharedlink = stacoms->priv;
	uint8_t				*rxbuff = sharedlink->rxbuff + MODBUS_TCPHEADER_SIZE;
	uint16_t			loglen = sharedlink->rxbytecnt;
	chret_e				chstate;
#if (MODBUSSL_TYPE == MODBUS_GENERIC)
	uint8_t				boffs = MODBUS_TCPHEADER_SIZE;
#endif


	*datalength = 0;

	switch (sharedlink->fformat) {
	case ModbusTCP:
		loglen += MODBUS_TCPHEADER_SIZE;
		*datalength = sharedlink->rxbytecnt - MODBUS_HEADER_SIZE;	// Data length for functions without length field
#if (MODBUSSL_TYPE == MODBUS_GENERIC)
		boffs = 0;
#endif
		break;

	case ModbusASCII:
		// TODO need to figure out at which point to convert from ASCII to HEX
		break;

	case ModbusRTU:
		if (sharedlink->rxbytecnt < (MODBUS_HEADER_SIZE + MODBUS_CRC_SIZE))
			return chret_empty;		// Not enough bytes received, keep waiting, applies only to Modbus RTU
		*datalength = sharedlink->rxbytecnt - MODBUS_HEADER_SIZE - MODBUS_CRC_SIZE;		// Data length for functions without length field
		break;

	default:	// Unknown frame format
		goto recv_error;
	}


	switch (rxbuff[MODBOFS_FUNC]) {
	case MODFUNC_01:
	case MODFUNC_02:
	case MODFUNC_03:
	case MODFUNC_04:
	case MODFUNC_05:
	case MODFUNC_06:
		*datalength = 4;
		break;

	case MODFUNC_07:
	case MODFUNC_0B:
	case MODFUNC_0C:
	case MODFUNC_11:
		*datalength = 0;
		break;

	case MODFUNC_08:
	case MODFUNC_2B:
		break;

	case MODFUNC_0F:
	case MODFUNC_10:
		*datalength = rxbuff[MODBOFS_DATA + 4] + 5;
		break;

	case MODFUNC_14:
	case MODFUNC_15:
		*datalength = rxbuff[MODBOFS_DATA] + 1;
		break;

	case MODFUNC_16:
		*datalength = 6;
		break;

	case MODFUNC_17:
		*datalength = rxbuff[MODBOFS_DATA + 8] + 9;
		break;

	case MODFUNC_18:
		*datalength = 2;
		break;

	default:	// Ignore unknown functions
		break;
	}


	switch (pdu_size_check(staptr, *datalength)) {
	case chret_empty:
		return chret_empty;	// Not enough bytes received, keep waiting, applies only to Modbus RTU

	case chret_dataerror:
		goto recv_error;

	default:
		break;
	}


	COMMS_RXHWLOG(loglen, boffs)	// Channel comms logger
	if (rxbuff[MODBUSOFFSET_ADDR] == MODBUS_BROADCAST_ADDR) {
		chstate = chret_rxtx;
		goto rxsl_complete;
	}


#if (MODBUSSL_TYPE == MODBUS_GENERIC)
	station_t *selsta;
	if ((selsta = station_search(stacoms, rxbuff[MODBUSOFFSET_ADDR], enabledf)))
		staptr = selsta;
	else {
#else
	if (rxbuff[MODBUSOFFSET_ADDR] != staptr->address) {
#endif
		chstate = chret_disabled;
		goto rxsl_complete;		// Message received for non-existent or disabled station
	}

	COMMS_RXLOG(loglen, boffs)	// Station comms logger

#if (MODBUSSL_TYPE == MODBUS_GENERIC)
	station_online_make(staptr, 1);
#endif
	chstate = chret_rxtx;


	rxsl_complete:
	sharedlink->rxbytecnt = 0;	// Reset received byte count AFTER all log function calls
	return chstate;


	recv_error:
	station_flush(staptr);		// Flush any remaining data if error occurred
	return chret_dataerror;
}


/*
 * Receive data from serial or socket buffer
 * [04/11/2014]
 * Dynamic TxDelay created
 * [04/03/2017]
 * Modbus TCP support added
 * [08/05/2019]
 */
chret_e Modbus_receive(STAARG_RX) {
	stacom_t			*stacoms = staptr->stacoms;
	channel_t			*chanptr = stacoms->chaninst;
	Modbus_shlink_t		*sharedlink = stacoms->priv;
	rxtxsize_t			rxlength;
	chret_e				chstate = chret_empty;
	uint16_t			rxval16;


	rxbuff = sharedlink->rxbuff;	// We use our own Rx buffer

	switch (stacoms->serstate) {
	case ser_flush:
	case ser_pretxdelay:
	default:
		station_flush(staptr);		// Flush any data if not in a receive state
		break;


	case ser_readyrx:
		sharedlink->rxbytecnt = 0;	// This is initial state, reset byte count
		if (STATION_ISSLAVE(staptr)) {
			MAINF_SET_CTIMEOUT		// Set Timeout timer when first character is received
		}
		goto start_recv;


	case ser_receiving:
		start_recv:
		switch (sharedlink->fformat) {
		case ModbusTCP:
			// Note: Our NumberMaxOfSeverTransaction is 1
			// MODBUS Messaging on TCP/IP Implementation Guide V1.0b October 24, 2006 (page 28)
			// Also we don't expect ModbusTCP message to be segmented in multiple TCP/UDP datagrams, this doesn't apply to ModbusRTU
			rxlength = MODBUS_RXBUFF_SIZE;
			if ((chstate = station_receive(staptr, rxbuff, &rxlength)) != chret_rxtx)
				return chstate;		// Return if function hasn't received data


			if (rxlength < MODBUS_TCPHEADER_SIZE) {
				COMMS_TCPERRLOG(rxlength, "Received message length (%u) is too short, at least (%u) bytes expected", rxlength, MODBUS_TCPHEADER_SIZE)
				goto rxerror;
			}
			else if (rxlength > MODBUS_TCP_MAX_SIZE) {
				COMMS_TCPERRLOG(rxlength, clogrxlenoverflow, rxlength, MODBUS_TCP_MAX_SIZE)
				goto rxerror;
			}


			if (sharedlink->tcpseq) {	// Slave doesn't have sequence number, thus no need to check
				rxval16 = rxbuff[MODBUSOFFSET_TCPSEQ] << 8;
				rxval16 |= rxbuff[MODBUSOFFSET_TCPSEQ + 1];
				// We only support NumberMaxOfSeverTransaction=1, peer has to reply with the same sequence number
				if (rxval16 != sharedlink->tcpseq) {		// Sequence number error
					COMMS_TCPERRLOG(rxlength, "Incorrect sequence number (%u) received, expected (%u)", rxval16, sharedlink->tcpseq)
					goto rxerror;
				}
			}


			rxval16 = rxbuff[MODBUSOFFSET_TCPPROTID] << 8;
			rxval16 |= rxbuff[MODBUSOFFSET_TCPPROTID + 1];
			if (rxval16) {		// Protocol ID error
				COMMS_TCPERRLOG(rxlength, "Unknown protocol ID (%u) received, expected (%u)", rxval16, 0)
				goto rxerror;
			}


			sharedlink->rxbytecnt = rxbuff[MODBUSOFFSET_TCPLEN] << 8;
			sharedlink->rxbytecnt |= rxbuff[MODBUSOFFSET_TCPLEN + 1];
			if (sharedlink->rxbytecnt < MODBUS_HEADER_SIZE) {		// Minimal message length error
				COMMS_TCPERRLOG(rxlength, "Received PDU length (%u) is too small, PDU should have at least (%u) bytes",
						sharedlink->rxbytecnt, MODBUS_HEADER_SIZE)
				goto rxerror;
			}


			// Note: Don't check if received length exceeds required message length!
			// We only support NumberMaxOfSeverTransaction=1 which means if multiple messages are received in a TCP/UDP datagram
			// Only the first one will be analyzed and any leftover data will be discarded.
			if (rxlength < (sharedlink->rxbytecnt + MODBUS_TCPHEADER_SIZE)) {
				COMMS_TCPERRLOG(rxlength, "PDU size (%u) doesn't match the PDU length variable (%u)", rxlength - MODBUS_TCPHEADER_SIZE, sharedlink->rxbytecnt)
				goto rxerror;
			}
			break;


		case ModbusASCII:
			// TODO implement
			break;


		case ModbusRTU:
			// We support segmented ModbusRTU messages over UART and arriving in multiple TCP/UDP datagrams, this doesn't apply to ModbusTCP
			rxlength = MODBUS_RXBUFF_SIZE - (MODBUS_TCPHEADER_SIZE + sharedlink->rxbytecnt);
			if ((chstate = station_receive(staptr, rxbuff + MODBUS_TCPHEADER_SIZE + sharedlink->rxbytecnt, &rxlength)) != chret_rxtx)
				return chstate;		// Return if function hasn't received data

			sharedlink->rxbytecnt += rxlength;

			if (sharedlink->rxbytecnt > MODBUS_ADU_MAX_SIZE) {
				COMMS_RXERRLOG(clogrxlenoverflow, sharedlink->rxbytecnt, MODBUS_ADU_MAX_SIZE)
				goto rxerror;
			}
			TIMER_SET_10USEC(sharedlink->rxtimeout35, &stacoms->chartimer);	// Reset character timer after every received byte
			break;


		default:	// Unknown frame format
			goto rxerror;
		}
		break;
	}


	switch (stacoms->serstate) {
	case ser_readyrx:
		stacoms->serstate = ser_receiving;	// Change state if any byte is received
		break;
	default:
		break;
	}
	return chret_empty;


	rxerror:
	stacoms->serstate = ser_flush;
	station_flush(staptr);	// Flush any remaining data if error occurred
	return chret_empty;		// Do not disconnect socket, just flush data
}


/*
 * Prepare Modbus protocol data for transmission
 * [07/11/2014]
 * Outgoing modbus function 0x11 added
 * [27/02/2015]
 * Modbus TCP support added
 * [08/05/2019]
 */
void Modbus_send(station_t *staptr, txrx8_t applen) {
	stacom_t			*stacoms = staptr->stacoms;
	channel_t			*chanptr = stacoms->chaninst;
	Modbus_shlink_t		*sharedlink = stacoms->priv;
	uint8_t				i;
	uint8_t				*txptr = sharedlink->txbuff;
	uint16_t 			crc = 0xffff;


	chanptr->txlen = 0;

	switch (sharedlink->fformat) {
	case ModbusTCP:
		if (!STATION_ISSLAVE(staptr)) {	// Slave doesn't have a sequence number, it shares Rx/Tx buffer and will reply with the received sequence number and protocol ID
			sharedlink->tcpseq++;
			if (!sharedlink->tcpseq)
				sharedlink->tcpseq++;

			txptr[MODBUSOFFSET_TCPSEQ] = (sharedlink->tcpseq >> 8) & 0xff;
			txptr[MODBUSOFFSET_TCPSEQ + 1] = sharedlink->tcpseq & 0xff;
			txptr[MODBUSOFFSET_TCPPROTID] = 0;
			txptr[MODBUSOFFSET_TCPPROTID + 1] = 0;
		}
		txptr[MODBUSOFFSET_TCPLEN] = ((MODBUS_HEADER_SIZE + applen) >> 8) & 0xff;
		txptr[MODBUSOFFSET_TCPLEN + 1] = (MODBUS_HEADER_SIZE + applen) & 0xff;
		txptr[MODBUS_TCPHEADER_SIZE + MODBUSOFFSET_ADDR] = staptr->address;

		chanptr->txlen = MODBUS_TCPHEADER_SIZE + MODBUS_HEADER_SIZE + applen;
		chanptr->txptr = sharedlink->txbuff;
		//chanptr->txlen = 5;
		break;


	case ModbusASCII:
		// TODO convert address to ASCII
		break;


	case ModbusRTU:
		txptr = sharedlink->txbuff + MODBUS_TCPHEADER_SIZE;
		txptr[MODBUSOFFSET_ADDR] = staptr->address;
		MODBUS_CRC_CALC(txptr[MODBUSOFFSET_ADDR]);
		MODBUS_CRC_CALC(txptr[MODBUSOFFSET_FUNC]);

		for (i = 0; i < applen; i++) {
			MODBUS_CRC_CALC(txptr[MODBUS_HEADER_SIZE + i]);
		}
		txptr[MODBUS_HEADER_SIZE + i] = crc & 0xFF;
		txptr[MODBUS_HEADER_SIZE + i + 1] = (crc >> 8) & 0xFF;
		chanptr->txlen = MODBUS_HEADER_SIZE + applen + MODBUS_CRC_SIZE;
		chanptr->txptr = sharedlink->txbuff + MODBUS_TCPHEADER_SIZE;
		break;


	default:
		break;
	}
	COMMS_TXLOG		// Station and channel comms logger
}


#if ((MODBUSMA_TYPE == MODBUS_GENERIC) || (MODBUSSL_TYPE == MODBUS_GENERIC))
/*
 * Communication error entry point function
 * [07/11/2014]
 * Converted to timeout function
 * [09/05/2019]
 */
void Modbus_timeout(station_t *staptr) {
	stacom_t			*stacoms = staptr->stacoms;
	Modbus_shlink_t		*sharedlink = stacoms->priv;
	int					logtout = 1;


	if (!STATION_ISSLAVE(staptr)) {
		mastersta_commserr(staptr, 0);
		SERVF_RXTOCNT	// Increment Rx timeout counter
		if (stacoms->serstate != ser_receiving)
			logtout = 0;
	}


#ifdef DEBUG_LOGEVERYTIMEOUT
	logtout = 1;	// Log every Master timeout
#endif

	if (logtout) {
		COMMS_RXERRLOG(clogrxTimeout);
	}
	sharedlink->rxbytecnt = 0;	// Reset receive pointer after logfile function call
}
#endif


/*
 * Initialize Shared link layer structure
 * [30/09/2014]
 * Default t35 timeout initialization removed
 * [20/08/2015]
 * Generalized to make compatible with Modbus Slave
 * [19/04/2019]
 */
static Modbus_shlink_t *shared_init(station_t *staptr) {
	Modbus_shlink_t		*sharedlink;
	size_t				memsize = sizeof(*sharedlink) + MODBUS_TXBUFF_SIZE;


	if (staptr->stacoms->priv)
		return staptr->stacoms->priv;


	// Shared structure is not found, create new
	if (!STATION_ISSLAVE(staptr))	// Slave has a shared Rx/Tx buffer
		memsize += MODBUS_RXBUFF_SIZE;

	if (!(sharedlink = calloc(1, memsize)))
		return NULL;

	staptr->stacoms->priv = sharedlink;
	sharedlink->txbuff = (void *) (((leptr) sharedlink) + sizeof(*sharedlink));

	if (STATION_ISSLAVE(staptr))	// Slave has a shared Rx/Tx buffer
		sharedlink->rxbuff = sharedlink->txbuff;
	else
		sharedlink->rxbuff = sharedlink->txbuff + MODBUS_TXBUFF_SIZE;
	return sharedlink;
}


/*
 * Initialize memory for Modbus master protocol pointers
 * [15/11/2014]
 * Online check function generalized
 * Get information address function removed, generalized in relatime.c
 * [17/08/2016]
 * Slave intialization added
 * [04/06/2019]
 */
int Modbus_create(station_t *staptr, channel_t *chanptr) {
	Modbus_shlink_t		*sharedlink;


	if (!(sharedlink = shared_init(staptr)))
		return LE_FAIL;		// Out of memory


#if ((MODBUSMA_TYPE == MODBUS_GENERIC) || (MODBUSSL_TYPE == MODBUS_GENERIC))
	if (chanptr->uartinst) {
		if (sharedlink->rxtimeout35 < 100) 	// Ensure timeout is at least 1ms
			sharedlink->rxtimeout35 = 100;

		if (sharedlink->rxtimeout35 < (MODBUS_RXT35CONST / chanptr->uartinst->baudrate))
			sharedlink->rxtimeout35 = (MODBUS_RXT35CONST / chanptr->uartinst->baudrate);
	}
	else if (chanptr->socketinst) {
		// Ensures socket is disconnected if nothing has been received from peer
		if (chanptr->socketinst->idletsec == DEFAULT_UNUSEDT32)
			chanptr->socketinst->idletsec = DEFAULT_SOCK_IDLETIMEOUT;
	}
#endif


	if (STATION_ISSLAVE(staptr)) {
		staptr->stacoms->serstate = ser_readyrx;

#if (MODBUSSL_TYPE == MODBUS_GENERIC)
		staptr->offline1dsec = MODBUS_DEFAULT_OFFLINEDELAY;


		sharedlink->fformat = ModbusRTU;
		//sharedlink->fformat = ModbusTCP;
#ifdef SLAVE_T35
		sharedlink->rxtimeout35 = SLAVE_T35;
#endif
#endif
	}
	else {	// Master
#if (MODBUSMA_TYPE == MODBUS_GENERIC)
		staptr->norespconst = MODBUS_DEFAULT_NORESPCNT;
		staptr->degradedconst = MODBUS_DEFAULT_DEGRADEDRETRIES;
		staptr->degradedtimeout = MODBUS_DEFAULT_DEGRADEDTIMEOUT;

		staptr->stacoms->serstate = ser_pretxdelay;
#endif
	}
	return LE_OK;
}


/*
 * Finalize Modbus protocol initialization
 * after XML configuration is processed
 * [30/09/2014]
 * Slave intialization added
 * [09/04/2019]
 */
void Modbus_postinit(station_t *staptr) {

#if ((MODBUSMA_TYPE == MODBUS_GENERIC) || (MODBUSSL_TYPE == MODBUS_GENERIC))
	staptr->norespcnt = staptr->norespconst;		// Initialize no response counter
	staptr->degradedcnt = staptr->degradedconst;	// Initialize Degraded retry counter
#endif
}
