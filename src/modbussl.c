/*
 ============================================================================
 Name        : Modbussl.c
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Modbus ASCII/RTU/TCP communication protocol application layer slave module

  Change log  :

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/eeprom.h>


#include "ledefs.h"
#include "modbussl.h"
#include "modbus.h"
#include "mcueecfg.h"



const lechar *ModbusslVersion = " ModbusslVersion=1.00 ";


#ifdef GLOBAL_DEBUG
#endif	// GLOBAL_DEBUG






// Event logfile caption
/*const lechar *elogmbCaption = "Time        \tInfo\tIndex\tFunc\tReg\tData\tValue\tDescription\r\n";
const lechar *elogmbEmpty =  "---\t---\t---\t---\t---\t";
const lechar *elogmbCommon = "%u\t0x%04X\t0x%04X\t";
const lechar *elogmbException1 = "%u\t\t0x%02X\t%s\t";
const lechar *elogmbException2 = "%u\t\t0x%02X\t";
const lechar *elogmbIndex = "%u\t";
const lechar *elogmbValue = "%s\t";
const lechar *elogmbTail1 = "\t\t\t\t";			// Without index, function and data
const lechar *elogmbTail2 = "\t\t\t\t";			// Without function and data

*/




// Macros
//#define MODBUSMA_EVENTLOG(mstring, mtype) Modbusma_eventlogger(applayer->eventlog, &applayer->evlogvar, mstring, mtype, __FILE__, __LINE__);

// Function and data offset in application buffer
#define MOFFSET_FUNC (charmult * MODBUSOFFSET_FUNC)
#define MOFFSET_DATA (charmult * MODBUSOFFSET_DATA)
#define MOFFSET_REG0 (charmult * (MODBUSOFFSET_DATA + 1))
#define MOFFSET_REGH (charmult * MODBUSOFFSET_DATA)
#define MOFFSET_REGL (charmult * (MODBUSOFFSET_DATA + 1))
#define MOFFSET_LENH (charmult * (MODBUSOFFSET_DATA + 2))
#define MOFFSET_LENL (charmult * (MODBUSOFFSET_DATA + 3))






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
	//return EXIT_SUCCESS;
}


/***************************************************************************
* Initialize Modbus Salve register memory
* [28/02/2015]
***************************************************************************/
void Modbussl_regmeminit(Modbussl_applayer *applayer, uint8_t mapsize) {
	uint16_t				regoffset = 0;
	ModReg16bitDef			regaddr;


	if (!mapsize) return;	// Don't initialize if mapping size is 0
	applayer->regcount = mapsize;
	applayer->regmem = calloc(applayer->regcount, sizeof(ModbusRegStr));


	for (regaddr = 0; regaddr < MODBUSSL_USERMAPSIZE; regaddr++) {
		if (mappinginit(regaddr,
				(leptr *) &applayer->regmem[regoffset].rddata,
				(leptr *) &applayer->regmem[regoffset].wrdata) == EXIT_SUCCESS) {
			applayer->regmem[regoffset].reg = regaddr;
			regoffset++;
			if (regoffset == mapsize) break;		// End of initialized register memory reached
		}
	}


	applayer->eemapbase = MODBUSSL_EEBASE;	// Base Modbus register address for accessing EEPROM contents
	applayer->eemapsize = MODBUSSL_EESIZE;	// EEPORM size in bytes
}





/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 * Process Modbus Slave protocol message
 * [24/02/2015]
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
uint8_t Modbussl_appprocess(Modbussl_applayer *applayer, uint8_t *rxtxbuff, uint8_t charmult) {
	ModReg16bitDef			regaddr;
	ModReg16bitDef			regcount;
	uint16_t				firstcnt, lastcnt;


	switch (rxtxbuff[MOFFSET_FUNC]) {
	case MODFUNC_03:
	case MODFUNC_04:
		regaddr = (rxtxbuff[MOFFSET_REGH] << 8);
		regaddr |= rxtxbuff[MOFFSET_REGL];
		regcount = (rxtxbuff[MOFFSET_LENH] << 8);
		regcount |= rxtxbuff[MOFFSET_LENL];
		if ((!regcount) || (regcount > MODBUS_MAX_REGISTER_COUNT)) goto addrlenexception;	// Invalid data length requested

		for (firstcnt = 0; firstcnt < applayer->regcount; firstcnt++) {
			if (regaddr == applayer->regmem[firstcnt].reg) {		// First requested register address found in memory map
				for (lastcnt = 0; lastcnt < regcount; lastcnt++) {
					if ((firstcnt + lastcnt) < applayer->regcount) {
						if (applayer->regmem[firstcnt + lastcnt].reg != (regaddr + lastcnt)) {
							goto addrlenexception;	// Requested length exceeds sequential registers in memory
						}
					}
					else {			// End of the register memory buffer reached
						goto addrlenexception;	// Invalid data length requested
					}
				}
				// Requested data length validated, generate message
				return Modbussl_message(rxtxbuff, charmult, (applayer->regmem + firstcnt), regcount);
			}
		}


		if (	// EEPROM read block
				(regaddr >= applayer->eemapbase) &&
				(regaddr < (applayer->eemapbase + (applayer->eemapsize >> 1)))) {				// If requested address less than base + size (< 0x9400)
			if ((regaddr + regcount) <= (applayer->eemapbase + (applayer->eemapsize >> 1))) {	// If requested addr + regcount is less or equal base + size (<= 0x9400)
				return Modbussl_eeblock(rxtxbuff, charmult, (regaddr - applayer->eemapbase), regcount);
			}
		}
		addrlenexception:
		return Modbussl_exception(rxtxbuff, charmult, MODEX_ILLEGAL_DATA_ADDR);
		break;


	case MODFUNC_05:
	case MODFUNC_06:
		regaddr = (rxtxbuff[MOFFSET_REGH] << 8);
		regaddr |= rxtxbuff[MOFFSET_REGL];
		for (firstcnt = 0; firstcnt < applayer->regcount; firstcnt++) {
			if (regaddr == applayer->regmem[firstcnt].reg) {		// Requested register address found in memory map
				if (applayer->regmem[firstcnt].wrdata) {
					applayer->regmem[firstcnt].wrdata[0] = rxtxbuff[MOFFSET_LENL];	// Write lowbyte to a mapped register
					applayer->regmem[firstcnt].wrdata[1] = rxtxbuff[MOFFSET_LENH];	// Write highbyte to a mapped register
					return (4 * charmult);			// Echo the received message
				}
			}
		}
		return Modbussl_exception(rxtxbuff, charmult, MODEX_ILLEGAL_DATA_ADDR);
		//break;


	case MODFUNC_11:	// Report Slave ID
		return Modbussl_rdslaveid(rxtxbuff, charmult);


	default:	// Unknown modbus message
		return Modbussl_exception(rxtxbuff, charmult, MODEX_ILLEGAL_FUNC);
		//break;
	}
	return 0;	// Impossible, all cases must be handled
}


/***************************************************************************
* Report Salve ID message
* [24/02/2015]
***************************************************************************/
uint8_t Modbussl_rdslaveid(uint8_t *txbuff, uint8_t charmult) {
	lechar 					*nameptr;
	uint8_t 				length;
	uint8_t					cnt;


	length = gethwname(&nameptr);
	if (length) {
		txbuff[MOFFSET_DATA] = length;
		for (cnt = 0; cnt < length; cnt++) {
			txbuff[MOFFSET_REG0 + (cnt * charmult)] = nameptr[cnt];
		}
	}
	else return Modbussl_exception(txbuff, charmult, MODEX_ILLEGAL_DATA_VAL);

	return (length + 1) * charmult;
}




/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 * Build Modbus Master protocol message
 * [30/09/2014]
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
//TxRx8bitDef Modbusma_build(Modbusma_applayer *applayer, uint8_t *txbuff, uint8_t charmult) {
//	TxRx8bitDef				msglength;
	//ICreq16bitDef			girequest = 0;
	//Modbusmsg				*outmsg;


//	return 0;		// No message prepared
//}


/***************************************************************************
* Prepare message from structure
* [24/02/2015]
***************************************************************************/
uint8_t Modbussl_message(uint8_t *txbuff, uint8_t charmult, ModbusRegStr *regptr, uint8_t count) {
	uint8_t					cnt;


	//txbuff[MOFFSET_FUNC] = outmsg->func;
	txbuff[MOFFSET_DATA] = count * 2;
	for (cnt = 0; cnt < count; cnt++) {
		if (regptr[cnt].rddata) {		// Read data pointer exists
			txbuff[MOFFSET_REG0 + (((cnt * 2) + 1) * charmult)] = regptr[cnt].rddata[0];	// Lowbyte has to be read first
			txbuff[MOFFSET_REG0 + (cnt * 2 * charmult)] = regptr[cnt].rddata[1];
			//txbuff[MOFFSET_REG0 + (cnt * charmult)] = ((uint16_t) dataptr) >> 8;
		}
		else {		// Read data pointer is not initialized, return zeros
			txbuff[MOFFSET_REG0 + (((cnt * 2) + 1) * charmult)] = 0;
			txbuff[MOFFSET_REG0 + (cnt * 2 * charmult)] = 0;
		}
	}
	return ((count * 2) + 1) * charmult;
}


/***************************************************************************
* Prepare message from eeprom block
* [24/02/2015]
***************************************************************************/
uint8_t Modbussl_eeblock(uint8_t *txbuff, uint8_t charmult, ModReg16bitDef reg, uint8_t count) {
	uint8_t					cnt;
	uint16_t				eedata;


	//txbuff[MOFFSET_FUNC] = outmsg->func;
	txbuff[MOFFSET_DATA] = count * 2;
	for (cnt = 0; cnt < count; cnt++) {
		eedata = eeprom_read_word((uint16_t *) ((reg + cnt) * 2));
		txbuff[MOFFSET_REG0 + (((cnt * 2) + 1) * charmult)] = eedata & 0xFF;
		txbuff[MOFFSET_REG0 + (cnt * 2 * charmult)] = (eedata >> 8) & 0xFF;
	}
	return ((count * 2) + 1) * charmult;
}


/***************************************************************************
* Generate exception message
* [24/02/2015]
***************************************************************************/
uint8_t Modbussl_exception(uint8_t *txbuff, uint8_t charmult, ModData8bitDef excpt) {

	//txbuff[MOFFSET_FUNC] = outmsg->func;
	txbuff[MOFFSET_FUNC] |= MBB_EXCEPTION;
	txbuff[MOFFSET_DATA] = excpt;
	return (1 * charmult);
}

