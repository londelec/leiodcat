/*
 ============================================================================
 Name        : Modbussl.c
 Author      : AK
 Version     : V1.03
 Copyright   : Property of Londelec UK Ltd
 Description : Modbus ASCII/RTU/TCP communication protocol application layer slave module

  Change log :

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
#include <avr/io.h>
#include <avr/eeprom.h>


#include "ledefs.h"
#include "modbussl.h"
#include "modbus.h"
#include "mcueecfg.h"




#ifdef GLOBAL_DEBUG
#endif	// GLOBAL_DEBUG





// Macros
//#define MODBUSMA_EVENTLOG(mstring, mtype) Modbusma_eventlogger(applayer->eventlog, &applayer->evlogvar, mstring, mtype, __FILE__, __LINE__);



/***************************************************************************
* Generate exception message
* [24/02/2015]
* Character multiplier argument removed
* [12/06/2015]
***************************************************************************/
static uint8_t Modbussl_exception(uint8_t *txbuff, ModData8bitDef excpt) {

	txbuff[MODBOFS_FUNC] |= MBB_EXCEPTION;
	txbuff[MODBOFS_DATA] = excpt;
	return (1);
}


/***************************************************************************
* Report Salve ID message
* [24/02/2015]
* Character multiplier argument removed
* [12/06/2015]
* Additional buffer removed, hw name copied directly to tx buffer
* [10/09/2016]
***************************************************************************/
static uint8_t Modbussl_rdslaveid(uint8_t *txbuff) {
	uint8_t 				length;


	if ((length = gethwname((lechar *) &txbuff[MODBOFS_11DATH(0)]))) {
		txbuff[MODBOFS_DATA] = length;
	}
	else
		return Modbussl_exception(txbuff, MODEX_ILLEGAL_DATA_VAL);

	return (length + 1);
}


/***************************************************************************
* Search register address in memory map
* [12/06/2015]
***************************************************************************/
static uint8_t Modbussl_searchreg(Modbussl_applayer *applayer, ModReg16bitDef regaddr, uint16_t *memoffset) {
	uint16_t				cnt;


	for (cnt = 0; cnt < applayer->regcount; cnt++) {
		if (regaddr == applayer->regmem[cnt].reg) {		// Requested register address found in memory map
			*memoffset = cnt;
			return LE_OK;
		}
	}
	return LE_FAIL;	// Register is not found
}


/***************************************************************************
* Validate number of requested registers
* [12/06/2015]
***************************************************************************/
static uint8_t Modbussl_validregcnt(Modbussl_applayer *applayer, ModReg16bitDef regaddr, ModReg16bitDef regcount, uint16_t memoffset) {
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


/***************************************************************************
* Prepare message from structure
* [24/02/2015]
* Character multiplier argument removed
* [12/06/2015]
* memset function used
* [08/09/2016]
***************************************************************************/
static uint8_t Modbussl_message(uint8_t *txbuff, ModbusRegStr *regptr, uint8_t count) {
	uint8_t					cnt;


	txbuff[MODBOFS_DATA] = count << 1;
	for (cnt = 0; cnt < count; cnt++) {
		if (regptr[cnt].rddata) {		// Read data pointer exists
			txbuff[MODBOFS_04DATL(cnt << 1)] = regptr[cnt].rddata[0];	// Lowbyte has to be read first
			txbuff[MODBOFS_04DATH(cnt << 1)] = regptr[cnt].rddata[1];
		}
		else {		// Read data pointer is not initialized, return zeros
			//txbuff[MODBOFS_04DATL(cnt << 1)] = 0;
			//txbuff[MODBOFS_04DATH(cnt << 1)] = 0;
			memset(&txbuff[MODBOFS_04DATH(cnt << 1)], 0, 2);
		}
	}
	return ((count << 1) + 1);
}


/***************************************************************************
* Prepare message from eeprom block
* [24/02/2015]
* Character multiplier argument removed
* [12/06/2015]
***************************************************************************/
static uint8_t Modbussl_eeblock(uint8_t *txbuff, ModReg16bitDef reg, uint8_t count) {
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



/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 * Process Modbus Slave protocol message
 * [24/02/2015]
 * Character multiplier argument removed
 * Register search and validation moved to separate functions
 * Modus function 0x10 added
 * [12/06/2015]
 * EEPROM configuration data validation added
 * [20/08/2015]
 * Minor register read optimization
 * [08/09/2016]
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
uint8_t Modbussl_appprocess(Modbussl_applayer *applayer, uint8_t *rxtxbuff) {
	ModReg16bitDef			regaddr;
	ModReg16bitDef			regcount;
	ModData16bitDef			rcvddata;
	uint16_t				memoffset;
	uint16_t				cnt;
	uint8_t					eeupd = 0;


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


		if (	// EEPROM read block
				(regaddr >= applayer->eemapbase) &&
				(regaddr < (applayer->eemapbase + (applayer->eemapsize >> 1)))) {				// If requested address less than base + size (< 0x9400)
			if ((regaddr + regcount) <= (applayer->eemapbase + (applayer->eemapsize >> 1))) {	// If requested addr + regcount is less or equal base + size (<= 0x9400)
				return Modbussl_eeblock(rxtxbuff, (regaddr - applayer->eemapbase), regcount);
			}
		}
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

					//rcvddata = rxtxbuff[MODBOFS_10DATH(cnt << 1)];
					//rcvddata <<= 8;
					//rcvddata |= rxtxbuff[MODBOFS_10DATL(cnt << 1)];
					rcvddata = (rxtxbuff[MODBOFS_10DATH(cnt << 1)] << 8) | rxtxbuff[MODBOFS_10DATL(cnt << 1)];

					if (*applayer->regmem[memoffset + cnt].wrdata != rcvddata) {
						if (writevalidate(regaddr + cnt, rcvddata, &eeupd) == LE_FAIL)
							goto illegaldataexception;
					}
				}


				for (cnt = 0; cnt < regcount; cnt++) {
					//rcvddata = rxtxbuff[MODBOFS_10DATH(cnt << 1)];
					//rcvddata <<= 8;
					//rcvddata |= rxtxbuff[MODBOFS_10DATL(cnt << 1)];
					rcvddata = (rxtxbuff[MODBOFS_10DATH(cnt << 1)] << 8) | rxtxbuff[MODBOFS_10DATL(cnt << 1)];

					*applayer->regmem[memoffset + cnt].wrdata = rcvddata;	// Write received data to a mapped register
				}
				goto checkeeupdate;
				//return (4);		// Echo part of the received message - register address and register count
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


/***************************************************************************
* Initialize Modbus Salve register memory
* [28/02/2015]
***************************************************************************/
static void Modbussl_regmeminit(Modbussl_applayer *applayer, uint8_t mapsize) {
	uint16_t				regoffset = 0;
	ModReg16bitDef			regaddr;


	if (!mapsize)
		return;	// Don't initialize if mapping size is 0

	applayer->regcount = mapsize;
	applayer->regmem = calloc(applayer->regcount, sizeof(ModbusRegStr));


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
}


/***************************************************************************
* Initialize memory for Modbus Slave protocol application layer variables
* [12/12/2014]
***************************************************************************/
Modbussl_applayer *Modbussl_preappinit(void) {

	Modbussl_applayer *applayer = calloc(1, sizeof(Modbussl_applayer));
	//applayer->objecttable = objecttable;
	return applayer;
}


/***************************************************************************
* Initialize Modbus slave protocol application buffer pointers
* [24/02/2015]
***************************************************************************/
void Modbussl_postappinit(Modbussl_applayer *applayer, uint8_t mapsize) {

	Modbussl_regmeminit(applayer, mapsize);
	//return LE_OK;
}
