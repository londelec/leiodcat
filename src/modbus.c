/*
 ============================================================================
 Name        : Modbus.c
 Author      : AK
 Version     : V1.05
 Copyright   : Property of Londelec UK Ltd
 Description : Modbus RTU communication protocol link layer module

  Change log :

  *********V1.05 07/09/2016**************
  Local functions marked static

  *********V1.04 24/08/2015**************
  Default t35 timeout initialization removed

  *********V1.03 16/06/2015**************
  Character multiplier is no longer passed to application layer

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
#include <math.h>			// fabs


#include "modbus.h"
#include "modbusdef.h"
#include "usart.h"
//#include "XMLmodbus.h"



//const lechar *ModbusVersion = " ModbusVersion=1.02 ";


#ifdef GLOBAL_DEBUG
//#define	IEC10xma_BuildRx				// Build unique message for debugging
#endif	// GLOBAL_DEBUG






// Macros
#define MODBUS_RXLOG_MACRO(mlength) commslogger(sharedlink->rxbuff, mlength, 0, staptr->logfile, clogenRX);
#define MODBUS_RXHWLOG_MACRO(mlength) commslogger(sharedlink->rxbuff, mlength, 0, staptr->channelinst->logfile, clogenRX);
#define MODBUS_TXLOG_MACRO commslogger(*txbuffptr, txlength, 0, staptr->logfile, clogenTX);
#define MODBUS_TXHWLOG_MACRO commslogger(*txbuffptr, txlength, 0, staptr->channelinst->logfile, clogenTX);

// Function and data offset in application buffer
#define MOFFSET_FUNC (sharedlink->charmult * MODBUSOFFSET_FUNC)
#define MOFFSET_DATA (sharedlink->charmult * MODBUSOFFSET_DATA)




#ifdef GLOBAL_DEBUG

#endif	// GLOBAL_DEBUG




/***************************************************************************
* Generate Modbus CRC
* [27/10/2014]
***************************************************************************/
void buildCRC16(uint16_t *crc, uint8_t databyte) {
	uint8_t		cnt;

	*crc ^= databyte;
	for (cnt = 0; cnt < 8; cnt++) {
		if (*crc & 1) {
			*crc >>= 1;
			*crc ^= MODBUS_CRC16_POLY;
		}
		else
			*crc >>= 1;
	}
}


/***************************************************************************
* Analyze Modbus message
* [20/02/2015]
***************************************************************************/
static CHStateEnum Modbussl_analyze(StatStr *staptr) {
	Modbussl_pointers		*realprot = (Modbussl_pointers *) staptr->realprotocol;
	Modbus_linklayer		*linklayer = realprot->linklayer;
	Modbus_shared_linklayer	*sharedlink = realprot->sharedlink;
	uint8_t					*rxappbuffer;
	uint8_t					datalength = 0;
	uint16_t				cnt;
	//logflagdef				logflags = 0;
	//logflagdef				chlogflags = 0;
	//CHStateEnum				chstate = CHCommsEmpty;
	//uint8_t					resetrespcnt = 0;


	//if (staptr->runflags & SERVICE_COMMSDISABLED) {
	//	sharedlink->rxbytecnt = 0;
	//	return CHCommsRxTx;				// Pretend we have decoded message successfully
	//}


	// Read Logger flags if logfile pointer exists
	//if (staptr->logfile) logflags = staptr->logfile->lflags;
	//if (staptr->channelinst->logfile) chlogflags = staptr->channelinst->logfile->lflags;




	switch (sharedlink->fformat) {
	/*case ModbusTCP:
		if (chlogflags & CLOGXF_INCOMING) {
			MODBUS_RXHWLOG_MACRO(sharedlink->rxapplen + MODBUS_TCPHEADER_SIZE)	// Channel comms logger if enabled
		}
		rxappbuffer = sharedlink->rxbuff + MODBUS_TCPHEADER_SIZE;
		if (rxappbuffer[MODBUSOFFSET_ADDR] != MODBUS_GLOBAL_ADDR) {
			if (rxappbuffer[MODBUSOFFSET_ADDR] != linklayer->devaddr) goto Modbusma_notreceived;
		}
		if ((rxappbuffer[MOFFSET_FUNC] & 0x7F) != realprot->txbuff[MODBUS_TCPHEADER_SIZE + MODBUSOFFSET_FUNC]) goto Modbusma_notreceived;
		datalength = sharedlink->rxapplen - MODBUS_HEADER_SIZE;		// Data length for functions without length field
		break;

	case ModbusASCII:
		// TODO device address validation
		rxappbuffer = sharedlink->rxbuff;
		break;*/

	//case ModbusRTU:
	default:
		rxappbuffer = sharedlink->rxbuff;
		sharedlink->crc = 0xFFFF;		// Reset CRC
		//if (chlogflags & CLOGXF_INCOMING) {
		//	MODBUS_RXHWLOG_MACRO(sharedlink->rxbytecnt)		// Channel comms logger if enabled
		//}
		// Validate device address
		if (rxappbuffer[MODBUSOFFSET_ADDR] != linklayer->devaddr)
			goto Modbussl_notreceived;
		datalength = sharedlink->rxbytecnt - MODBUS_HEADER_SIZE - MODBUS_CRC_SIZE;		// Data length for functions without length field
		break;
	}




	switch (rxappbuffer[MOFFSET_FUNC]) {
	case MODFUNC_01:
	case MODFUNC_02:
	case MODFUNC_03:
	case MODFUNC_04:
	case MODFUNC_05:
	case MODFUNC_06:
		datalength = 4;
		break;

	case MODFUNC_07:
	case MODFUNC_0B:
	case MODFUNC_0C:
	case MODFUNC_11:
		datalength = 0;
		break;

	case MODFUNC_08:
	case MODFUNC_2B:
		break;			// Variable length

	case MODFUNC_0F:
	case MODFUNC_10:
		datalength = rxappbuffer[sharedlink->charmult * 6] + 5;
		break;

	case MODFUNC_14:
	case MODFUNC_15:
		datalength = rxappbuffer[MOFFSET_DATA] + 1;
		break;

	case MODFUNC_16:
		datalength = 6;
		break;

	case MODFUNC_17:
		datalength = rxappbuffer[sharedlink->charmult * 10] + 9;
		break;

	case MODFUNC_18:
		// FIXME length is two bytes
		datalength = rxappbuffer[MOFFSET_DATA] + 1;
		break;

	default:	// Unknown function
		goto Modbussl_notreceived;
		break;
	}




	// Check length of data
	switch (sharedlink->fformat) {
	/*case ModbusTCP:
		if ((sharedlink->rxapplen - MODBUS_HEADER_SIZE) != datalength) {
			goto Modbussl_notreceived;
		}
		//if (logflags & CLOGXF_INCOMING) {
		//	MODBUS_RXLOG_MACRO(sharedlink->rxapplen + MODBUS_TCPHEADER_SIZE)		// Station comms logger if enabled
		//}
		break;

	case ModbusASCII:
		// TODO data length validation
		break;*/

	//case ModbusRTU:
	default:
		if ((sharedlink->rxbytecnt - MODBUS_HEADER_SIZE - MODBUS_CRC_SIZE) != datalength) {
			goto Modbussl_notreceived;
		}
		for (cnt = 0; cnt < (sharedlink->rxbytecnt - MODBUS_CRC_SIZE); cnt++) {
			buildCRC16(&sharedlink->crc, rxappbuffer[cnt]);
		}
		if ((sharedlink->crc >> 8) != rxappbuffer[sharedlink->rxbytecnt - 1])
			goto Modbussl_notreceived;
		if ((sharedlink->crc & 0xFF) != rxappbuffer[sharedlink->rxbytecnt - 2])
			goto Modbussl_notreceived;
		//if (logflags & CLOGXF_INCOMING) {
		//	MODBUS_RXLOG_MACRO(sharedlink->rxbytecnt)		// Station comms logger if enabled
		//}
		break;
	}




	//servincrementCNT(staptr, servCTrx, 0);	// Increment Rx counter
	sharedlink->rxbytecnt = 0;				// Reset receive pointer after logfile function call
	realprot->linklayer->commsstate = Modbuslinkok;			// Reset comms state if message received
	linklayer->txapplen = Modbussl_appprocess(realprot->applayer, rxappbuffer);


	if (!linklayer->txapplen)
		goto Modbussl_notreceived;




	/*switch (staptr->stastate) {
	case staenonline:
		break;
	default:	// Make station online if Offline or uninitialized
		staptr->stastate = staenonline;
		servcheckgenDI(staptr, servDIonline, STATION_ONLINE_DIQ);
		CLOGINFO_MACRO(clogOnline)
		break;
	}*/


	// Start Offline delay timer if Offline delay const is defined
	//if (staptr->offline1dsec) {
	//	MAINF_SET_1SEC(staptr->offline1dsec, &staptr->offlinetimer);	// Start Offline delay Timer
	//}
	return CHCommsRxTx;


	Modbussl_notreceived:
	return CHCommsEmpty;
}


/***************************************************************************
* Receive data from serial or socket buffer
* [04/11/2014]
***************************************************************************/
static CHStateEnum Modbus_receive(DRVARGDEF_RX) {
	ChannelStr				*chanptr = staptr->channelinst;
	Modbus_linklayer		*linklayer;
	Modbus_shared_linklayer	*sharedlink;
	uint8_t					*rxbuffer;
	//rxbytesdef				rxlength;
	TxRx16bitDef			rxlength;
	CHStateEnum				chstate;
	//uint16_t				rxseq;


	//switch (staptr->realptype) {
	//case ModbusslConst:
		linklayer = ((Modbussl_pointers *) staptr->realprotocol)->linklayer;
		sharedlink = ((Modbussl_pointers *) staptr->realprotocol)->sharedlink;
		rxbuffer = sharedlink->rxbuff;
	//	break;

	//case ModbusmaConst:
	//	linklayer = ((Modbusma_pointers *) staptr->realprotocol)->linklayer;
	//	sharedlink = ((Modbusma_pointers *) staptr->realprotocol)->sharedlink;
	//	rxbuffer = sharedlink->rxbuff;
	//	break;

	//default:
	//	return CHCommsEmpty;			// Unknown protocol, no message received
	//}


	// Rx state and byte counter reset required
	//if (staptr->runflags & STARFLAG_RESET_RX_STATE) {
	//	staptr->runflags &= ~STARFLAG_RESET_RX_STATE;
	//	sharedlink->rxstate = Modbusrxheader;
	//	sharedlink->expectedrxbytes = MODBUS_TCPHEADER_SIZE;
	//}


	switch (chanptr->chserstate) {
	case enumchflush:
	case enumchpretxdelay:
		//flushrx(chanptr);			// Flush any data from the buffer if not in receive state
		break;


	//case enumchreadyrx:
	//case enumchreceiving:
	default:
		switch (sharedlink->fformat) {
		/*case ModbusTCP:
			if (chanptr->chserstate == enumchreadyrx) {		// This is initial state, reset variables
				sharedlink->rxstate = Modbusrxheader;
				sharedlink->expectedrxbytes = MODBUS_TCPHEADER_SIZE;
			}
			ContinueTCP_receive:
			switch (sharedlink->rxstate) {
			case Modbusrxpayload:
				rxlength = sharedlink->expectedrxbytes;
				chstate = channelrx(
							staptr,
							rxbuffer + MODBUS_TCPHEADER_SIZE + sharedlink->rxapplen - sharedlink->expectedrxbytes,
							&rxlength);
					if (chstate != CHCommsRxTx) return chstate;		// Return if function hasn't received data


					// Check if all data received
					if (rxlength == sharedlink->expectedrxbytes) {
						//IEC104_RXLOG_MACRO(IEC104_CLOG_INCOMING)
						sharedlink->expectedrxbytes = MODBUS_TCPHEADER_SIZE;
						sharedlink->rxstate = Modbusrxheader;
						goto Modbusma_msgreceived;
					}

					// Not enough data received
					else sharedlink->expectedrxbytes -= rxlength;
					break;


				default:
				//case Modbusrxheader:
					rxlength = sharedlink->expectedrxbytes;
					chstate = channelrx(
							staptr,
							rxbuffer + MODBUS_TCPHEADER_SIZE - sharedlink->expectedrxbytes,
							&rxlength);
					if (chstate != CHCommsRxTx) return chstate;		// Return if function hasn't received data


					// Check if all data received
					if (rxlength == sharedlink->expectedrxbytes) {

						// Validate transaction identifier
						rxseq = rxbuffer[MODBUSOFFSET_TCPSEQ] << 8;
						rxseq |= rxbuffer[MODBUSOFFSET_TCPSEQ + 1];
						if (rxseq != linklayer->tcpseq) {			// Sequence number error
							//IEC104_RXERRLOG_MACRO(clog104StartIDError, 1)
							staptr->channelinst->chserstate = enumchflush;
							sharedlink->expectedrxbytes = MODBUS_TCPHEADER_SIZE;
							return CHCommsDataError;
						}

						// Validate Protocol ID
						if (rxbuffer[MODBUSOFFSET_TCPPROTID] || rxbuffer[MODBUSOFFSET_TCPPROTID + 1]) {	// Protocol ID error
							//IEC104_RXERRLOG_MACRO(clog104StartIDError, 1)
							staptr->channelinst->chserstate = enumchflush;
							sharedlink->expectedrxbytes = MODBUS_TCPHEADER_SIZE;
							return CHCommsDataError;
						}


						sharedlink->rxapplen = rxbuffer[MODBUSOFFSET_TCPLEN] << 8;
						sharedlink->rxapplen |= rxbuffer[MODBUSOFFSET_TCPLEN + 1];
						if (sharedlink->rxapplen < MODBUS_HEADER_SIZE) {		// Minimal message length error
							staptr->channelinst->chserstate = enumchflush;
							sharedlink->expectedrxbytes = MODBUS_TCPHEADER_SIZE;
							return CHCommsDataError;
						}

						//IEC104_RXLOG_MACRO(IEC104_CLOG_TRANSPORT)
						sharedlink->expectedrxbytes = sharedlink->rxapplen;
						sharedlink->rxstate = Modbusrxpayload;
						goto ContinueTCP_receive;
					}

					// Not enough data received
					else sharedlink->expectedrxbytes -= rxlength;
					break;
				}
			break;*/


		//case ModbusRTU:
		default:
			rxlength = MODBUS_RXBUFF_SIZE;
			if (chanptr->chserstate == enumchreadyrx)
				sharedlink->rxbytecnt = 0;	// This is initial state, reset byte count
			chstate = channelrx(staptr, rxbuffer + sharedlink->rxbytecnt, &rxlength);
			if (chstate != CHCommsRxTx)
				return chstate;		// Return if function hasn't received data
			sharedlink->rxbytecnt += rxlength;
			MAINF_SET_10USEC(sharedlink->rxtimeout35, &staptr->channelinst->chchartimer);	// Reset idle timer after every received character
			break;
		}
		break;
	}




	//Modbusma_msgreceived:
	switch (chanptr->chserstate) {
	case enumchreadyrx:
		chanptr->chserstate = enumchreceiving;	// Change state to receiving if any byte is received
		break;
	default:
		break;
	}




	//switch (staptr->realptype) {
	//case ModbusslConst:
		return CHCommsRxTx;			// Message is successfully received

	//case ModbusmaConst:
	//	switch (sharedlink->fformat) {
	//	case ModbusTCP:
	//		if (Modbusma_analyze(staptr) == CHCommsRxTx) {
	//			if (MAINF_NEXT_SERMASTER == LE_OK) {	// Next station enabled
	//				staptr->channelinst->chserstate = enumchpretxdelay;
	//				MAINF_SET_CHTXDELAY						// Set pre Tx delay timer
	//			}
	//			else return CHCommsDisabled;	// All stations disabled, close socket
	//		}
	//		return CHCommsRxTx;			// Message successfully received

		//case ModbusRTU:
	//	default:
	//		return CHCommsEmpty;			// No action required
	//	}
	//	break;

	//default:
	//	break;
	//}
	return CHCommsEmpty;
}


/***************************************************************************
* Prepare Modbus protocol data for transmission
* [20/02/2015]
***************************************************************************/
static TxRx16bitDef Modbussl_send(StatStr *staptr, uint8_t **txbuffptr) {
	Modbus_linklayer		*linklayer;
	Modbus_shared_linklayer	*sharedlink;
	TxRx8bitDef				cnt;
	TxRx16bitDef			txlength = 0;
	//logflagdef				logflags = 0;
	//logflagdef				chlogflags = 0;


	linklayer = ((Modbussl_pointers *) staptr->realprotocol)->linklayer;
	sharedlink = ((Modbussl_pointers *) staptr->realprotocol)->sharedlink;
	*txbuffptr = sharedlink->rxbuff;


	// Read Logger flags if logfile pointer exists
	//if (staptr->logfile) logflags = staptr->logfile->lflags;
	//if (staptr->channelinst->logfile) chlogflags = staptr->channelinst->logfile->lflags;


	switch (sharedlink->fformat) {
	/*case ModbusTCP:
		(*txbuffptr)[MODBUS_TCPHEADER_SIZE + MODBUSOFFSET_ADDR] = linklayer->devaddr;
		break;

	case ModbusASCII:
		// TODO convert address to ASCII
		break;*/

	//case ModbusRTU:
	default:
		(*txbuffptr)[MODBUSOFFSET_ADDR] = linklayer->devaddr;
		sharedlink->crc = 0xFFFF;		// Reset CRC
		buildCRC16(&sharedlink->crc, (*txbuffptr)[MODBUSOFFSET_ADDR]);
		buildCRC16(&sharedlink->crc, (*txbuffptr)[MODBUSOFFSET_FUNC]);
		break;
	}


	switch (sharedlink->fformat) {
	/*case ModbusTCP:
		txlength = MODBUS_TCPHEADER_SIZE + MODBUS_HEADER_SIZE + MODBUS_SREQ_SIZE;
		break;

	case ModbusASCII:
		// TODO convert data to ASCII and calculate LRC
		break;*/

	//case ModbusRTU:
	default:
		for (cnt = 0; cnt < linklayer->txapplen; cnt++) {
			buildCRC16(&sharedlink->crc, (*txbuffptr)[MODBUS_HEADER_SIZE + cnt]);
		}
		(*txbuffptr)[MODBUS_HEADER_SIZE + cnt] = sharedlink->crc & 0xFF;
		(*txbuffptr)[MODBUS_HEADER_SIZE + cnt + 1] = (sharedlink->crc >> 8) & 0xFF;
		txlength = MODBUS_HEADER_SIZE + linklayer->txapplen + MODBUS_CRC_SIZE;
		break;
	}


	//if (logflags & CLOGXF_OUTGOING) {
	//	MODBUS_TXLOG_MACRO		// Station comms logger if enabled
	//}
	//if (chlogflags & CLOGXF_OUTGOING) {
	//	MODBUS_TXHWLOG_MACRO	// Channel comms logger if enabled
	//}
	return txlength;			// No outgoing message prepared
}


/***************************************************************************
* Modbus Slave protocol Offline
* [23/02/2015]
***************************************************************************/
static void Modbussl_commserr(DRVARGDEF_COMMSERR) {
	//Modbussl_pointers *realprot = (Modbussl_pointers *) staptr->realprotocol;


	if (istimeout) {	// This is comms timeout
		staptr->channelinst->chserstate = enumchreadyrx;		// Rx byte counters will be reset automatically at the beginning of receive function
		//if (realprot->sharedlink->rxbytecnt) {
			//IEC101_rxerrlog(staptr, realprot->sharedlink, clogrxTimeout);
		//}
	}
}


/***************************************************************************
* Modbus Slave protocol main process
* [19/02/2015]
***************************************************************************/
static CHStateEnum Modbussl_process(DRVARGDEF_MAINPROC) {
	ChannelStr				*chanptr = staptr->channelinst;
	Modbussl_pointers		*realprot;
	//Modbus_linklayer		*linklayer;
	Modbus_shared_linklayer	*sharedlink;


	realprot = (Modbussl_pointers *) staptr->realprotocol;
	sharedlink = realprot->sharedlink;


	// All stations disabled or different station selected for channel
	//if (sermasterprocess(staptr) == LE_FAIL) return CHCommsEmpty;	// This is not the station selected for channel or station is disabled


	switch (chanptr->chserstate) {
	case enumchflush:		// Error during rx, flush buffer
	case enumchreadyrx:		// Waiting to receive first character
		if (MAINF_CHECK_CHTIMER == LE_OK) {			// Check timeout
			Modbussl_commserr(staptr, 1, 0);
		}
		break;


	case enumchreceiving:	// We are currently receiving
		switch (((Modbussl_pointers *) staptr->realprotocol)->sharedlink->fformat) {
		/*case ModbusTCP:
			if (MAINF_CHECK_CHTIMER == LE_OK) {			// Check timeout
				Modbusma_commserr(staptr, 1, 0);
				if (MAINF_NEXT_SERMASTER == LE_OK) {		// Next station enabled
					goto Modbusma_new_message;
				}
				else return CHCommsDisabled;	// All stations disabled, close socket
			}
			break;*/

		//case ModbusRTU:
		default:
			if (MAINF_CHECK_CHCHARTIMER == LE_OK) {		// Check idle after receive
				if (Modbussl_analyze(staptr) == CHCommsRxTx) {
					chanptr->chserstate = enumchpretxdelay;
					MAINF_SET_CHTXDELAY						// Set pre Tx delay timer
				}
				else {	// Message analyze failed
					if (MAINF_CHECK_CHTIMER == LE_OK) {		// Check timeout
						Modbussl_commserr(staptr, 1, 0);
					}
					else {	// Timeout hasn't expired
						chanptr->chserstate = enumchreadyrx;		// Rx byte counters will be reset automatically at the beginning of receive function
					}
				}
			}
			break;
		}
		break;


	case enumchpretxdelay:	// Delay before transmission
		if (MAINF_CHECK_CHTIMER == LE_OK) {	// Check pre Tx delay
			goto Modbussl_new_message;
		}
		break;


	default:	// Unknown state, reset to protocol default
		chanptr->chserstate = enumchreadyrx;
		break;
	}
	return CHCommsEmpty;	// No outgoing message prepared




	Modbussl_new_message:
	//realprot = (Modbussl_pointers *) chanptr->serialsta->realprotocol;
	//sharedlink = realprot->sharedlink;
	//linklayer = realprot->linklayer;


	/*switch (linklayer->commsstate) {
	case Modbusrepeatlast:
		break;

	//case Modbuslinkok:
	default:
		linklayer->txapplen = Modbusma_build(
				realprot->applayer,
				realprot->txbuff + sharedlink->appoffset,
				sharedlink->charmult);

		if (!linklayer->txapplen) return CHCommsEmpty;	// No message prepared
		break;
	}*/


	chanptr->chserstate = enumchflush;
	MAINF_SET_CHTIMEOUT		// Activate Timeout timer
	*txlength = Modbussl_send(chanptr->serialsta, txbuffptr);
	return CHCommsRxTx;		// Transmit prepared message
}


/***************************************************************************
* Initialize Shared link layer structure
* [30/09/2014]
* Default t35 timeout initialization removed
* [24/08/2015]
***************************************************************************/
static Modbus_shared_linklayer *Modbus_initshared(StatStr *currentsta) {
	//StatStr	 				*staptr;
	Modbus_shared_linklayer	*sharedlink;


	//for (staptr = Station0ptr; staptr; staptr = staptr->next) {
		//if (
		//		(staptr->chid == currentsta->chid) &&
		//		(staptr->realptype == currentsta->realptype)) {
		//	switch (staptr->realptype) {
			//case ModbusslConst:
			//	if (staptr->realprotocol) {
			//		sharedlink = ((Modbussl_pointers *) staptr->realprotocol)->sharedlink;
			//		if (sharedlink) return sharedlink;
			//	}
			//	break;

			//case ModbusmaConst:
			//	if (staptr->realprotocol) {
			//		sharedlink = ((Modbusma_pointers *) staptr->realprotocol)->sharedlink;
			//		if (sharedlink) return sharedlink;
			//	}
			//	break;

			//default:
			//	break;
			//}
		//}
	//}


	// Existing shared structure is not found, initialize
	sharedlink = calloc(1, sizeof(Modbus_shared_linklayer));
	sharedlink->rxbuff = calloc(MODBUS_RXBUFF_SIZE, sizeof(uint8_t));
	return sharedlink;
}


/***************************************************************************
* Initialize memory for Modbus slave protocol pointers
* [19/02/2015]
***************************************************************************/
void Modbussl_preinit(GenProtocolStr *gprot, DevAddrDef devaddr) {
	Modbussl_pointers	*realprot;
	Modbussl_applayer	*applayer;


		realprot = calloc(1, sizeof(Modbussl_pointers));
		gprot->statptr->realprotocol = realprot;
		gprot->statptr->func_mainproc = Modbussl_process;
		//gprot->statptr->func_chinit = Modbusma_chinit;
		gprot->statptr->func_rx = Modbus_receive;
		//gprot->statptr->func_commserr = Modbusma_commserr;
		//gprot->statptr->func_onlinecheck = Modbus_onlinecheck;
		realprot->linklayer = calloc(1, sizeof(Modbus_linklayer));
		//realprot->txbuff = calloc(MODBUS_TXBUFF_SIZE, sizeof(uint8_t));
		realprot->sharedlink = Modbus_initshared(gprot->statptr);


		realprot->linklayer->commsstate = Modbuslinkok;
		realprot->sharedlink->rxstate = Modbusrxheader;
		realprot->linklayer->devaddr = devaddr;
		//gprot->statptr->norespconst = MODBUS_DEFAULT_NORESPCNT;
		//gprot->statptr->degradedconst = MODBUS_DEFAULT_DEGRADEDRETRIES;
		//gprot->statptr->degradedtimeout = MODBUS_DEFAULT_DEGRADEDTIMEOUT;
		applayer = Modbussl_preappinit();
		realprot->applayer = applayer;


		gprot->applayer = applayer;
}


/***************************************************************************
* Initialize Modbus Slaver protocol memory
* [23/02/2015]
***************************************************************************/
uint8_t Modbussl_postinit(GenProtocolStr *gprot, TimerConstDef t35, uint8_t mapsize) {
	Modbussl_pointers *realprot = gprot->statptr->realprotocol;


	// Populate Application layer and check Common address first
	Modbussl_postappinit(gprot->applayer, mapsize);

	switch (realprot->sharedlink->fformat) {
	/*case ModbusTCP:
		realprot->sharedlink->appoffset = MODBUS_TCPHEADER_SIZE;
		realprot->sharedlink->charmult = 1;
		break;

	case ModbusASCII:
		realprot->sharedlink->appoffset = 1;
		realprot->sharedlink->charmult = 2;
		break;*/

	//case ModbusRTU:
	default:
		realprot->sharedlink->appoffset = 0;
		realprot->sharedlink->charmult = 1;
		break;
	}


	//gprot->statptr->norespcnt = gprot->statptr->norespconst;		// Initialize no response counter
	//gprot->statptr->degradedcnt = gprot->statptr->degradedconst;	// Initialize Degraded retry counter
	//MAINF_SET_1SEC(gprot->statptr->offline1dsec, &gprot->statptr->offlinetimer); // Init Offline delay Timer
	if (t35)
		realprot->sharedlink->rxtimeout35 = t35;

	return LE_OK;
}
