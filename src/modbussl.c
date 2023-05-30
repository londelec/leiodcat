/*
 ============================================================================
 Name        : Modbussl.c
 Author      : AK
 Version     : V2.00
 Copyright   : Property of Londelec UK Ltd
 Description : Modbus ASCII/RTU/TCP communication protocol Slave module

  Change log :

  *********V2.00 09/04/2019**************
  Generalized for leandc

  *********V1.03 07/09/2016**************
  Local functions marked static

  *********V1.02 18/08/2015**************
  EEPROM configuration data validation added

  *********V1.01 12/06/2015**************
  Character multiplier is no longer passed to application layer
  Modbus function 0x10 added

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>


#include "ledefs.h"
#include "modbussl.h"
#include "modbus.h"
#if (MODBUSSL_TYPE == MODBUS_MCU)
#include <avr/io.h>
#include <avr/eeprom.h>
#include "mcueecfg.h"
#elif (MODBUSSL_TYPE == MODBUS_GENERIC)
#include "realtime.h"
#endif


#ifdef GLOBAL_DEBUG
#define DEBUG_INIT_REGMEM				// Initialize register memory
#endif	// GLOBAL_DEBUG


// Macros




/*
 * Generate exception message
 * [24/02/2015]
 * Character multiplier argument removed
 * [12/06/2015]
 */
static uint8_t Modbussl_exception(uint8_t *txbuff, Moddata8_t excpt) {

	txbuff[MODBOFS_FUNC] |= MBB_EXCEPTION;
	txbuff[MODBOFS_DATA] = excpt;
	return (1);
}


/*
 * Send Slave ID message
 * [24/02/2015]
 * Additional buffer removed, hw name copied directly to tx buffer
 * [10/09/2016]
 * Generalized for leandc
 * [10/04/2019]
 */
static txrx8_t Modbussl_rdslaveid(uint8_t *txbuff) {
	txrx8_t		length = 0;

#if (MODBUSSL_TYPE == MODBUS_MCU)
	if ((length = gethwname((lechar *) &txbuff[MODBOFS_11DATA]))) {
		txbuff[MODBOFS_DATA] = length;
	}
	else
		return Modbussl_exception(txbuff, MODEX_ILLEGAL_DATA_VAL);
#else
	lechar id[] = "LEANDC V" VERSION_STRING;
	length = strlen(id);
	txbuff[MODBOFS_DATA] = length;
	memcpy(&txbuff[MODBOFS_11DATA], id, length);
#endif
	return (length + 1);
}


/*
 * Search register address in memory map
 * [12/06/2015]
 */
static uint8_t Modbussl_searchreg(Modbussl_layer *applayer, Modreg16_t regaddr, uint16_t *memoffset) {
	uint16_t				cnt;


	for (cnt = 0; cnt < applayer->regcount; cnt++) {
		if (regaddr == applayer->regmem[cnt].reg) {		// Requested register address found in memory map
			*memoffset = cnt;
			return LE_OK;
		}
	}
	return LE_FAIL;	// Register is not found
}


/*
 * Validate number of requested registers
 * [12/06/2015]
 */
static uint8_t Modbussl_validregcnt(Modbussl_layer *applayer, Modreg16_t regaddr, Modreg16_t regcount, uint16_t memoffset) {
	uint16_t				cnt;


	for (cnt = 0; cnt < regcount; cnt++) {
		if ((memoffset + cnt) < applayer->regcount) {
			if (applayer->regmem[memoffset + cnt].reg != (regaddr + cnt)) {
				return LE_FAIL;	// Requested length exceeds sequential registers in memory
			}
		}
		else {			// End of the register memory buffer reached
			return LE_FAIL;	// Invalid data length requested
		}
	}
	return LE_OK;	// Register count is validated
}


/*
 * Prepare message from structure
 * [24/02/2015]
 * memset function used
 * [08/09/2016]
 */
static uint8_t Modbussl_message(uint8_t *txbuff, Modbusreg_t *regptr, uint8_t count) {
	uint8_t					cnt;


	txbuff[MODBOFS_DATA] = count << 1;
	for (cnt = 0; cnt < count; cnt++) {
		if (regptr[cnt].rddata) {		// Read data pointer exists
			txbuff[MODBOFS_04DATL(cnt << 1)] = regptr[cnt].rddata[0];	// Lowbyte has to be read first
			txbuff[MODBOFS_04DATH(cnt << 1)] = regptr[cnt].rddata[1];
		}
		else {		// Read data pointer is not initialized, return zeros
			memset(&txbuff[MODBOFS_04DATH(cnt << 1)], 0, 2);
		}
	}
	return ((count << 1) + 1);
}


#if (MODBUSSL_TYPE == MODBUS_MCU)
/*
 * Prepare message from eeprom block
 * [24/02/2015]
 * Character multiplier argument removed
 * [12/06/2015]
 */
static uint8_t Modbussl_eeblock(uint8_t *txbuff, Modreg16_t reg, uint8_t count) {
	uint8_t					cnt;
	uint16_t				eedata;


	txbuff[MODBOFS_DATA] = count << 1;
	for (cnt = 0; cnt < count; cnt++) {
		eedata = eeprom_read_word((uint16_t *) ((reg + cnt) << 1));
		txbuff[MODBOFS_04DATL(cnt << 1)] = eedata & 0xff;
		txbuff[MODBOFS_04DATH(cnt << 1)] = (eedata >> 8) & 0xff;
	}
	return ((count << 1) + 1);
}
#endif


/*
 * Process Modbus Slave protocol message
 * [24/02/2015]
 * Register search and validation moved to separate functions
 * Modus function 0x10 added
 * [12/06/2015]
 * EEPROM configuration data validation added
 * [20/08/2015]
 * Generalized for leandc
 * [09/04/2019]
 */
static txrx8_t Modbussl_appprocess(Modbussl_layer *applayer, uint8_t *rxtxbuff) {
	Modreg16_t			regaddr;
	Modreg16_t			regcount;
	Moddata16_t			rcvddata;
	uint16_t			memoffset;
	uint16_t			cnt;
#if (MODBUSSL_TYPE == MODBUS_MCU)
	uint8_t				eeupd = 0;
#endif


	regaddr = (rxtxbuff[MODBOFS_XREGH] << 8) | rxtxbuff[MODBOFS_XREGL];


	switch (rxtxbuff[MODBOFS_FUNC]) {
	case MODFUNC_03:
	case MODFUNC_04:
		regcount = (rxtxbuff[MODBOFS_04CNTH] << 8) | rxtxbuff[MODBOFS_04CNTL];
		if ((!regcount) || (regcount > MODBUS_MAX_REGISTER_COUNT))
			goto addrlenexception;	// Invalid data length requested

		if (Modbussl_searchreg(applayer, regaddr, &memoffset) == LE_OK) {
			if (Modbussl_validregcnt(applayer, regaddr, regcount, memoffset) == LE_FAIL) {
				goto addrlenexception;	// requested register count validation failed
			}
			// Requested data length validated, generate message
			return Modbussl_message(rxtxbuff, (applayer->regmem + memoffset), regcount);
		}


#if (MODBUSSL_TYPE == MODBUS_MCU)
		if (	// EEPROM read block
				(regaddr >= applayer->eemapbase) &&
				(regaddr < (applayer->eemapbase + (applayer->eemapsize >> 1)))) {				// If requested address less than base + size (< 0x9400)
			if ((regaddr + regcount) <= (applayer->eemapbase + (applayer->eemapsize >> 1))) {	// If requested addr + regcount is less or equal base + size (<= 0x9400)
				return Modbussl_eeblock(rxtxbuff, (regaddr - applayer->eemapbase), regcount);
			}
		}
#endif
		addrlenexception:
		return Modbussl_exception(rxtxbuff, MODEX_ILLEGAL_DATA_ADDR);
		break;


	case MODFUNC_05:
	case MODFUNC_06:
		if (Modbussl_searchreg(applayer, regaddr, &memoffset) == LE_OK) {
			if (applayer->regmem[memoffset].wrdata) {	// Check if register is writable

				//rcvddata = rxtxbuff[MODBOFS_06DATH];
				//rcvddata <<= 8;
				//rcvddata |= rxtxbuff[MODBOFS_06DATL];
				rcvddata = (rxtxbuff[MODBOFS_06DATH] << 8) | rxtxbuff[MODBOFS_06DATL];


#if (MODBUSSL_TYPE == MODBUS_MCU)
				if (*applayer->regmem[memoffset].wrdata != rcvddata) {
					if (writevalidate(regaddr, rcvddata, &eeupd) == LE_OK) {
						*applayer->regmem[memoffset].wrdata = rcvddata;		// Write received data to a mapped register


						checkeeupdate:
						if (eeupd) {	// Data was different from stored value, request eeprom update
							eeconf_update(&applayer->regmem[memoffset]);
						}
					}
					else {
						illegaldataexception:
						return Modbussl_exception(rxtxbuff, MODEX_ILLEGAL_DATA_VAL);
					}
				}
#endif

#ifdef DEBUG_INIT_REGMEM
				*applayer->regmem[memoffset].wrdata = rcvddata;
#endif
				return (4);			// Echo the received message
			}
		}
		goto addrlenexception;	// requested register count validation failed
		//break;


	case MODFUNC_10:
		regcount = (rxtxbuff[MODBOFS_10CNTH] << 8) | rxtxbuff[MODBOFS_10CNTL];
		if ((!regcount) || (regcount > MODBUS_MAX_REGISTER_COUNT))
			goto addrlenexception;	// Invalid data length requested

		if (Modbussl_searchreg(applayer, regaddr, &memoffset) == LE_OK) {
			if (Modbussl_validregcnt(applayer, regaddr, regcount, memoffset) == LE_OK) {


				for (cnt = 0; cnt < regcount; cnt++) {		// Check if all registers are writable and received data is valid
					if (!applayer->regmem[memoffset + cnt].wrdata)
						goto addrlenexception;	// current register is not writable

					rcvddata = (rxtxbuff[MODBOFS_10DATH(cnt << 1)] << 8) | rxtxbuff[MODBOFS_10DATL(cnt << 1)];


#if (MODBUSSL_TYPE == MODBUS_MCU)
					if (*applayer->regmem[memoffset + cnt].wrdata != rcvddata) {
						if (writevalidate(regaddr + cnt, rcvddata, &eeupd) == LE_FAIL)
							goto illegaldataexception;
					}
#endif
				}


				for (cnt = 0; cnt < regcount; cnt++) {
					rcvddata = (rxtxbuff[MODBOFS_10DATH(cnt << 1)] << 8) | rxtxbuff[MODBOFS_10DATL(cnt << 1)];

					*applayer->regmem[memoffset + cnt].wrdata = rcvddata;	// Write received data to a mapped register
				}
#if (MODBUSSL_TYPE == MODBUS_MCU)
				goto checkeeupdate;
#else
				return (4);		// Echo part of the received message - register address and register count
#endif
			}
		}
		goto addrlenexception;	// requested register count validation failed
		//break;


	case MODFUNC_11:	// Report Slave ID
		return Modbussl_rdslaveid(rxtxbuff);


	default:	// Unknown modbus message
		return Modbussl_exception(rxtxbuff, MODEX_ILLEGAL_FUNC);
		//break;
	}
	return 0;	// Impossible, all cases must be handled
}


#if (MODBUSSL_TYPE == MODBUS_GENERIC)
/*
 * Socket/UART closed callback
 * [15/05/2019]
 */
static void stacb_sockclose(STAARG_CB) {
	stacom_t			*stacoms = staptr->stacoms;
	channel_t			*chanptr = stacoms->chaninst;
	stacnt_t			i;
	const lechar		*clname = clogComClosed;


	if (chanptr->socketinst)
		clname = clogSockClosed;
	CLOG_HWINFO(clname)

	staptr->stacoms->serstate = ser_readyrx;

	for (i = 0; i < stacoms->stacont.size; i++) {
		staptr = stacoms->stacont.stabuff[i];
		CLOG_INFO(clname)
	}
}


/*
 * Socket/UART open callback
 * [15/05/2019]
 */
static void stacb_sockopen(STAARG_CB) {
	channel_t			*chanptr = staptr->stacoms->chaninst;
	const lechar		*opname = clogComOpen;


	if (chanptr->socketinst)
		opname = clogSockConnected;
	CLOG_INFO(opname)
}


/*
 * Slave Offline callback
 * [06/05/2019]
 */
static void stacb_offline(STAARG_CB) {

	CLOG_INFO(clogoffline1st)
}


/*
 * Station Disable/Enable callback
 * [06/05/2019]
 */
static void servcb_disable(SERVARG_DOCB) {
	stacom_t			*stacoms = staptr->stacoms;


	switch (dco & DCS_MASK) {
	case STATION_DISABLE_DCS:
		CLOG_INFO(clogDisabled)
		break;

	case STATION_ENABLE_DCS:
		CLOG_INFO(clogEnabled)
		stacoms->serstate = ser_readyrx;
		//TIMER_SET_1SEC(staptr->offline1dsec, &staptr->offlinetimer);	// Always reset offline timer
		break;

	default:	// Impossible, service command qualifiers have been validated
		break;
	}
}
#endif


/*
 * Modbus Slave protocol main process
 * [09/05/2019]
 */
static chret_e Modbussl_process(STAARG_MAINPROC) {
	stacom_t			*stacoms = staptr->stacoms;
	channel_t			*chanptr = stacoms->chaninst;
	genprot_t			*gprot = staptr->gpinst;
	Modbussl_layer		*applayer = gprot->applayer;
	Modbus_shlink_t		*sharedlink = stacoms->priv;
	chret_e				chstate = chret_empty;
	txrx8_t	 			datalength;
	int					enabledf = 1;		// Will be set to 0 if all stations are disabled


#if (MODBUSSL_TYPE == MODBUS_GENERIC)
	if (STATION_SERIAL_SEL(stacoms) != staptr)
		goto proc_complete;
#endif


	switch (stacoms->serstate) {
	case ser_readyrx:		// Waiting to receive first character
		check_enabled:
#if (MODBUSSL_TYPE == MODBUS_GENERIC)
		station_search(stacoms, 0, &enabledf);	// Need to know if there are any enabled stations
#else
		{}
#endif
		break;


	case ser_flush:			// Error during rx, discard any junk received until timeout occurs
		if (MAINF_CHECK_CTIMER == LE_OK) 	// Check timeout
			goto reset_state;
		goto check_enabled;


	case ser_receiving:		// In process of receiving Modbus RTU data or have received all Modbus TCP data
		// Note, don't check if station is disabled in this state
		// We need HW log to record received data even if all stations are disabled.
		switch (sharedlink->fformat) {
		case ModbusTCP:
			goto analyze_rcvd;

		case ModbusASCII:
			// TODO implement
			break;

		case ModbusRTU:
			if (MAINF_CHECK_CHARTIMER == LE_OK) {	// Character t35 timeout, not used for Modbus TCP
				analyze_rcvd:
				chstate = Modbussl_rx_check(staptr, &datalength, &enabledf);

#if (MODBUSSL_TYPE == MODBUS_GENERIC)
				staptr = STATION_SERIAL_SEL(stacoms);	// New station may have been selected
				applayer = staptr->gpinst->applayer;
#endif

				switch (chstate) {
				case chret_rxtx:
					if ((sharedlink->txapplen = Modbussl_appprocess(applayer, sharedlink->rxbuff + MODBUS_TCPHEADER_SIZE))) {
						stacoms->serstate = ser_pretxdelay;
						MAINF_SET_SLTXDELAY		// Set pre Tx delay timer
					}
					else
						goto reset_state;
					break;


				case chret_empty:	// Incomplete message, applies only to Modbus RTU
					if (MAINF_CHECK_CTIMER == LE_OK) {	// Check timeout
#if (MODBUSSL_TYPE == MODBUS_GENERIC)
						Modbus_timeout(staptr);
#endif
						goto reset_state;
					}
					// Keep receiving until timeout occurs
					break;


				case chret_disabled:	// Valid message received for non-existent or disabled station
					goto reset_state;


				case chret_dataerror:	// Message validation failed e.g. length or CRC error
				default:
					stacoms->serstate = ser_flush;	// Wait for timeout to occur
					break;
				}
			}
			break;


		default:	// Unknown frame format
			goto reset_state;
		}
		break;


	case ser_pretxdelay:	// Delay before transmission
#if (MODBUSSL_TYPE == MODBUS_GENERIC)
		if (staptr->runflags & SERVICE_COMMSDISABLED)
			goto reset_state;	// Need to reset state if station was disabled before it could reply
#endif


		if (MAINF_CHECK_CTIMER == LE_OK) {	// Check pre Tx delay
			Modbus_send(staptr, sharedlink->txapplen);
			stacoms->serstate = ser_readyrx;
			return chret_rxtx;	// Transmit prepared message
		}
		break;


	default:	// Unknown state, reset to protocol default
		reset_state:
		stacoms->serstate = ser_readyrx;
		goto check_enabled;
	}


#if (MODBUSSL_TYPE == MODBUS_GENERIC)
	proc_complete:
	slavesta_commserr(staptr, 0);	// Always check Offline timer of every Slave station
#endif
	if (!enabledf)
		return chret_disabled;	// All stations disabled, close socket
	return chret_empty;			// No outgoing message prepared
}


/*
 * Dummy Modbus register memory for testing
 * [12/12/2020]
 */
#ifdef DEBUG_INIT_REGMEM
static void dummy_regmem(Modbussl_layer *applayer) {
	Modreg16_t		i;
	Modreg16_t		*buf;


	applayer->regcount = 11;
	applayer->regmem = calloc(applayer->regcount, sizeof(*applayer->regmem));
	buf = calloc(applayer->regcount, sizeof(Modreg16_t));


	for (i = 0; i < applayer->regcount; i++) {
		switch (i) {
		case 0:
			//applayer->regmem[i].reg = 0x0001;
			applayer->regmem[i].reg = 40493;
			break;

		case 1:
			applayer->regmem[i].reg = 0x0100;
			//buf[i] = 0x023A;
			break;

		case 2:
		case 4:
			buf[i] = 0x0;
			//applayer->regmem[i].reg = 0x0110 + i - 2;
			applayer->regmem[i].reg = 31249 + i - 2;
			break;

		case 3:
		case 5:
			buf[i] = 116;
			//applayer->regmem[i].reg = 0x0110;
			//applayer->regmem[i].reg = 0x0110 + i - 2;
			applayer->regmem[i].reg = 31249 + i - 2;

			break;

		case 6:
			applayer->regmem[i].reg = 0x0300;
			break;

		case 7:
			buf[i] = 0xFFFF;
			applayer->regmem[i].reg = 0x0320 + i - 7;
			break;

		case 8:
			buf[i] = 0xFFFD;
			/* fallthrough */
		case 9:
		case 10:
			//applayer->regmem[i].reg = 0x0320 + i - 7;
			applayer->regmem[i].reg = 41167 + i - 9;
			break;

		default:
			break;
		}
		applayer->regmem[i].rddata = (Modreg8_t *) &buf[i];
		applayer->regmem[i].wrdata = &buf[i];
	}
}
#endif


/*
 * Initialize Modbus Slave register memory
 * [28/02/2015]
 * Generalized for leandc
 * [09/04/2019]
 */
static void Modbussl_regmeminit(Modbussl_layer *applayer, uint8_t mapsize) {
#if (MODBUSSL_TYPE == MODBUS_MCU)
	uint16_t			regoffset = 0;
	Modreg16_t			regaddr;
#endif

	if (!mapsize)
		return;	// Don't initialize if mapping size is 0

	applayer->regcount = mapsize;
	applayer->regmem = calloc(applayer->regcount, sizeof(Modbusreg_t));


#if (MODBUSSL_TYPE == MODBUS_MCU)
	for (regaddr = 0; regaddr < MODBUSSL_USERMAPSIZE; regaddr++) {
		if (mappinginit(regaddr,
				(leptr *) &applayer->regmem[regoffset].rddata,
				(leptr *) &applayer->regmem[regoffset].wrdata) == LE_OK) {
			applayer->regmem[regoffset].reg = regaddr;
			regoffset++;

			if (regoffset == mapsize)
				break;		// End of initialized register memory reached
		}
	}


	applayer->eemapbase = MODBUSSL_EEBASE;	// Base Modbus register address for accessing EEPROM contents
	applayer->eemapsize = MODBUSSL_EESIZE;	// EEPORM size in bytes
#endif
}


/*
 * Initialize memory for Modbus Slave protocol application layer variables
 * [12/12/2014]
 * Generalized for leandc
 * [19/04/2019]
 */
genprot_t *Modbussl_create(station_t *staptr, channel_t *chanptr) {
	genprot_t			*gprot;
	Modbussl_layer 		*layer;


	if (!(gprot = genprot_create(staptr, sizeof(*layer), 0)))
		goto init_fail;		// Out of memory


	if (Modbus_create(staptr, chanptr) == LE_FAIL)
		goto init_fail;		// Out of memory


	layer = gprot->applayer;
	staptr->gpinst = gprot;
	staptr->func_rx = Modbus_receive;
	staptr->func_mainproc = Modbussl_process;

#if (MODBUSSL_TYPE == MODBUS_GENERIC)
	staptr->func_sockclose = stacb_sockclose;
	staptr->func_sockopen = stacb_sockopen;
	staptr->func_offline = stacb_offline;
	staptr->servobjects->servDOfunc[SERVICE_DO_COUNT + servDOcommsdisable] = servcb_disable;
#endif
	return gprot;


	init_fail:
	GENPROT_SETFAIL(gprot)
	return NULL;
}


/*
 * Initialize Modbus slave protocol application buffer pointers
 * [24/02/2015]
 * Generalized for leandc
 * [09/04/2019]
 */
void Modbussl_postinit(genprot_t *gprot, uint8_t mapsize) {
	Modbussl_layer 		*applayer = gprot->applayer;


	Modbus_postinit(gprot->statptr);

#ifdef DEBUG_INIT_REGMEM
	dummy_regmem(applayer);
	return;
#endif

	Modbussl_regmeminit(applayer, mapsize);
}
