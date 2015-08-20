/*
 ============================================================================
 Name        : mcueecfg.c
 Author      : AK
 Version     : V2.01
 Copyright   : Property of Londelec UK Ltd
 Description : MCU EEPROM configuration parser

  Change log  :

  *********V2.01 17/08/2015**************
  Fixed: Exclude DO register when resolving EEPROM group
  Group data prepare function pointers are now stored and called from table
  Configuration validation function created
  UART setting configuration sorted out

  *********V2.00 14/06/2015**************
  EEPROM configuration writing introduced
  CRC is appended at the end of the configuration

  *********V1.00 28/12/2014**************
  Initial revision

 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
//#include <stdint.h>
//#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>



#include "main.h"
#include "ledefs.h"
#include "mcueecfg.h"
#include "board.h"
#include "powman.h"
#include "modbus.h"




#ifdef GLOBAL_DEBUG
#endif	// GLOBAL_DEBUG


#define SIZEOF_EEDTENT(mstruct)		(sizeof(mstruct) + sizeof(mcueedthead))


const mcueecbtablestr mcueecbtable[] PROGMEM = {
	{eegren_uart0, 		eegrpcreate_uart},
	{eegren_powman, 	eegrpcreate_powman},
	{eegren_board, 		eegrpcreate_board},
};


/*const eemapregTblStr eemapboardTable[] PROGMEM = {
	{atmapen_dimode00, 		eedten_board_dimode00},
	{atmapen_dimode01, 		eedten_board_dimode01},
	{atmapen_dimode02, 		eedten_board_dimode02},
	{atmapen_dimode03, 		eedten_board_dimode03},
	{atmapen_dimode04, 		eedten_board_dimode04},
	{atmapen_dimode05, 		eedten_board_dimode05},
	{atmapen_dimode06, 		eedten_board_dimode06},
	{atmapen_dimode07, 		eedten_board_dimode07},
	{atmapen_dimode08, 		eedten_board_dimode08},
	{atmapen_dimode09, 		eedten_board_dimode09},
	{atmapen_dimode0A, 		eedten_board_dimode0A},
	{atmapen_dimode0B, 		eedten_board_dimode0B},
	{atmapen_dimode0C, 		eedten_board_dimode0C},
	{atmapen_dimode0D, 		eedten_board_dimode0D},
	{atmapen_dimode0E, 		eedten_board_dimode0E},
	{atmapen_dimode0F, 		eedten_board_dimode0F},

	{atmapen_dif00, 		eedten_board_diflt00},
	{atmapen_dif01, 		eedten_board_diflt01},
	{atmapen_dif02, 		eedten_board_diflt02},
	{atmapen_dif03, 		eedten_board_diflt03},
	{atmapen_dif04, 		eedten_board_diflt04},
	{atmapen_dif05, 		eedten_board_diflt05},
	{atmapen_dif06, 		eedten_board_diflt06},
	{atmapen_dif07, 		eedten_board_diflt07},
	{atmapen_dif08, 		eedten_board_diflt08},
	{atmapen_dif09, 		eedten_board_diflt09},
	{atmapen_dif0A, 		eedten_board_diflt0A},
	{atmapen_dif0B, 		eedten_board_diflt0B},
	{atmapen_dif0C, 		eedten_board_diflt0C},
	{atmapen_dif0D, 		eedten_board_diflt0D},
	{atmapen_dif0E, 		eedten_board_diflt0E},
	{atmapen_dif0F, 		eedten_board_diflt0F},

	{atmapen_domode00, 		eedten_board_domode00},
	{atmapen_domode01, 		eedten_board_domode01},
	{atmapen_domode02, 		eedten_board_domode02},
	{atmapen_domode03, 		eedten_board_domode03},
	{atmapen_domode04, 		eedten_board_domode04},
	{atmapen_domode05, 		eedten_board_domode05},
	{atmapen_domode06, 		eedten_board_domode06},
	{atmapen_domode07, 		eedten_board_domode07},
	{atmapen_domode08, 		eedten_board_domode08},
	{atmapen_domode09, 		eedten_board_domode09},
	{atmapen_domode0A, 		eedten_board_domode0A},
	{atmapen_domode0B, 		eedten_board_domode0B},
	{atmapen_domode0C, 		eedten_board_domode0C},
	{atmapen_domode0D, 		eedten_board_domode0D},
	{atmapen_domode0E, 		eedten_board_domode0E},
	{atmapen_domode0F, 		eedten_board_domode0F},

	{atmapen_dopul00, 		eedten_board_dopul00},
	{atmapen_dopul01, 		eedten_board_dopul01},
	{atmapen_dopul02, 		eedten_board_dopul02},
	{atmapen_dopul03, 		eedten_board_dopul03},
	{atmapen_dopul04, 		eedten_board_dopul04},
	{atmapen_dopul05, 		eedten_board_dopul05},
	{atmapen_dopul06, 		eedten_board_dopul06},
	{atmapen_dopul07, 		eedten_board_dopul07},
	{atmapen_dopul08, 		eedten_board_dopul08},
	{atmapen_dopul09, 		eedten_board_dopul09},
	{atmapen_dopul0A, 		eedten_board_dopul0A},
	{atmapen_dopul0B, 		eedten_board_dopul0B},
	{atmapen_dopul0C, 		eedten_board_dopul0C},
	{atmapen_dopul0D, 		eedten_board_dopul0D},
	{atmapen_dopul0E, 		eedten_board_dopul0E},
	{atmapen_dopul0F, 		eedten_board_dopul0F},
};*/




/***************************************************************************
* Get configuration data from EEPROM
* [28/12/2014]
* EEPROM data parsing moved to separate functions
* [16/06/2015]
***************************************************************************/
uint8_t eeconf_get(mcueegrpenum groupid, uint8_t dataid, uint32_t *rddword) {
	leptr			groupptr, dataptr;
	eegrpsizeDef	groupsize;


	if (!(boardio.rflags & BOARDRF_EECONF_CORRUPTED)) {	// Ensure configuration is not corrupted
		if (!(boardio.eerdmask & (1 << groupid))) {		// Check if read is enabled for current group
			groupptr = eesearch_group(groupid, &groupsize, NULL);
			if (groupptr) {
				dataptr = eesearch_data(dataid, groupptr, groupsize, rddword);
				if (dataptr) return EXIT_SUCCESS;
			}
		}
	}
	return EXIT_FAILURE;
}


/***************************************************************************
* Check if it is possible to write received byte to EEPROM
* [18/08/2015]
***************************************************************************/
uint8_t eeconf_validate(atmappingenum mapreg, ModData16bitDef val, uint8_t *updatereq) {
	mcueegrpenum	groupid;
	leptr			groupptr;
	eegrpsizeDef	groupsize, newgrpsize;


	groupid = eegroupid_resolve(mapreg, val, updatereq);	// Search EEPROM group to be updated based on modbus register
	if (groupid) {
		if (!(boardio.eewrmask & (1 << groupid))) {		// Check if write is enabled for current group

			groupptr = eesearch_group(groupid, &groupsize, NULL);
			if (groupptr) {

				newgrpsize = eegrpfunc(groupid, NULL);	// This is just to get a size of the new group which is going to be created
				if (groupsize == newgrpsize) {			// Ensure size of the group currently stored in the EEPROM is equal to the group which is about to be created
					return EXIT_SUCCESS;				// It is possible to save received data to EEPROM
				}
			}
		}
	}
	else {
		if (updatereq && !(*updatereq))
			return EXIT_SUCCESS;	// Current register is not part not part of EEPROM configuration
	}
	return EXIT_FAILURE;	// Validation failed, impossible to save received data to EEPROM
}


/***************************************************************************
* Update configuration group
* [17/06/2015]
* Group create functions generalized and their pointers stored in a table
* [18/08/2015]
***************************************************************************/
void eeconf_update(ModbusRegStr *regmem) {
	mcueegrpenum	groupid;
	leptr			groupptr;
	eegrpsizeDef	groupsize;


	groupid = eegroupid_resolve(regmem->reg, *regmem->wrdata, NULL);	// Search EEPROM group to be updated based on modbus register
	if (groupid) {

		groupptr = eesearch_group(groupid, &groupsize, NULL);
		if (groupptr) {
			uint8_t *membuff = calloc(groupsize, 1);	// Allocate correct amount of memory
			uint8_t	*memptr = membuff;
			eegrpfunc(groupid, &memptr);			// Populate memory with group data
			eeprom_write_block(membuff, (void *) (groupptr + sizeof(mcueegrphead)), groupsize);
			eeconf_crc(1);
			free(membuff);
		}
	}
}


/***************************************************************************
* Check and restructure existing EEPROM configuration according to current firmware requirements
* Current policy is to identify the first group where changes are required
* and rebuild configuration from this group onwards. This means EEPROM contents following
* group to be modified will also be affected. This function relies on the fact all
* necessary settings have been read from EEPROM and verified by relevant functions.
* Groups which need to be updated are flagged in 'eeupdatebs' bit set.
* Under normal working conditions this function must not make any changes to EEPROM
* configuration as it is expected for the EEPROM to have correct configuration structure.
* This function is only used to restructure EEPROM in case firmware or EEPROM contents
* are upgraded which leads to mismatch between running firmware and EEPROM contents.
* [17/06/2015]
* Group create functions generalized and their pointers stored in a table
* [18/08/2015]
***************************************************************************/
void eeconf_restructure() {
	uint8_t			bitcnt;
	leptr			groupptr = 0;
	eegrpsizeDef	groupsize;
	uint16_t		newgrpsize;
	uint16_t		requiredsize = 0;
	uint8_t 		*membuff, *memptr;


	for (bitcnt = 0; bitcnt < 16; bitcnt++) {		// Identify which groups need to be updated
		if (boardio.eeupdatebs & (1 << bitcnt)) {
			newgrpsize = eegrpfunc(bitcnt, NULL);	// This is just to get a size of the new group which is going to be created
			if (newgrpsize) {
				leptr			nextavail, foundgrp;
				requiredsize += newgrpsize + sizeof(mcueegrphead);
				foundgrp = eesearch_group(bitcnt, &groupsize, &nextavail);

				if (!groupptr) {					// Initialize first group from which EEPROM update will begin
					if (foundgrp)
						groupptr = foundgrp;		// Required group already exists in EEPROM
					else
						groupptr = nextavail;		// Required group doesn't exit, it will be created at the end of current configuration
				}
			}
		}
	}


	if (requiredsize) {			// Allocate memory for the block containing one or more groups to be updated
		membuff = calloc(requiredsize, 1);
		memptr = membuff;
	}


	for (bitcnt = 0; bitcnt < 16; bitcnt++) {		// Build configuration and write to EEPROM
		if (boardio.eeupdatebs & (1 << bitcnt)) {
			boardio.eeupdatebs &= ~(1 << bitcnt);


			if (requiredsize && groupptr) {
				newgrpsize = eegrpfunc(bitcnt, NULL);	// This is just to get a size of the new group which is going to be created
				eepopoulate_grouphead(&memptr, bitcnt, newgrpsize);
				eegrpfunc(bitcnt, &memptr);
			}
		}
	}


	if (requiredsize) {
		eeprom_write_block(membuff, (void *) (groupptr), requiredsize);
		eewrite_data((sizeof(mcueeheader) - sizeof(eecfgsizeDef)), (groupptr + requiredsize), sizeof(eecfgsizeDef));		// Overall size of the configuration
		eeconf_crc(1);
		boardio.rflags &= ~BOARDRF_EECONF_CORRUPTED;
		free(membuff);	// Free allocated memory
	}
}


/***************************************************************************
* Search group in the EEPROM using identifier
* [14/06/2015]
***************************************************************************/
leptr eesearch_group(mcueegrpenum groupid, eegrpsizeDef *groupsize, leptr *nextavail) {
	leptr			groupptr;
	mcueegrphead	grouphead;


	if (nextavail) *nextavail = 0;


	// Start reading from first group after EEPROM header
	for (groupptr = sizeof(mcueeheader); groupptr < MCUEE_EESIZE; groupptr += grouphead.size + sizeof(mcueegrphead)) {
		grouphead.id = eeprom_read_byte((uint8_t *) groupptr);		// Read group identifier
		if (grouphead.id == eegren_undefined)						// Group undefined (most likely empty memory)
			break;
		grouphead.size = eeprom_read_word((uint16_t *) (groupptr + sizeof(grouphead.id)));


		if (grouphead.id == groupid) {		// Group ID found
			if (groupsize) *groupsize = grouphead.size;
			return groupptr;
		}
	}


	if (groupsize) *groupsize = 0;
	if (nextavail) *nextavail = groupptr;
	return 0;		// Group is not found
}


/***************************************************************************
* Search data in the EEPROM using identifier
* [15/06/2015]
***************************************************************************/
leptr eesearch_data(uint8_t dataid, leptr groupptr, eegrpsizeDef groupsize, uint32_t *rddword) {
	leptr			dataptr;
	mcueedthead		datahead;


	for (dataptr = groupptr + sizeof(mcueegrphead); dataptr < (groupptr + sizeof(mcueegrphead) + groupsize); dataptr += datahead.size + sizeof(mcueedthead)) {
		datahead.type = eeprom_read_byte((uint8_t *) dataptr);	// Read data type identifier
		if (datahead.type == 0xff)
			return EXIT_FAILURE;			// Data type undefined (most likely empty memory)
		datahead.size = eeprom_read_byte((uint8_t *) (dataptr + sizeof(datahead.type)));


		if (datahead.type == dataid) {	// Data ID found
			eeread_data(dataptr + sizeof(mcueedthead), rddword, datahead.size);
			return dataptr;
		}
	}
	return 0;		// Data is not found
}


/***************************************************************************
* Save data byte/word/dword to EEPROM
* [15/06/2015]
***************************************************************************/
void eeread_data(leptr eeadr, uint32_t *rddword, uint8_t size) {

	if (!rddword) return;

	switch (size) {	// Select size to read
	case 1:
		*rddword = eeprom_read_byte((uint8_t *) eeadr);
		break;

	case 2:
		*rddword = eeprom_read_word((uint16_t *) eeadr);
		break;

	case 4:
		*rddword = eeprom_read_dword((uint32_t *) eeadr);
		break;

	default:
		break;
	}
}


/***************************************************************************
* Save data byte/word/dword to EEPROM
* [15/06/2015]
***************************************************************************/
void eewrite_data(leptr eeadr, uint32_t wrdword, uint8_t size) {

	switch (size) {	// Select size to write
	case 1:
		eeprom_write_byte((uint8_t *) eeadr, wrdword);
		break;

	case 2:
		eeprom_write_word((uint16_t *) eeadr, wrdword);
		break;

	case 4:
		eeprom_write_dword((uint32_t *) eeadr, wrdword);
		break;

	default:
		break;
	}
}



/***************************************************************************
* Resolve group creation function from table and execute it
* [20/08/2015]
***************************************************************************/
eegrpsizeDef eegrpfunc(mcueegrpenum groupid, uint8_t **memptr) {
	uint8_t			cnt;
	mcueegrpenum	tabid;
	eegrpsizeDef	(*func)(MCUEEARGDEF_GRPFUNC);


	for (cnt = 0; cnt < ARRAY_SIZE(mcueecbtable); cnt++) {
		tabid = pgm_read_byte(&mcueecbtable[cnt].id);		// Read group id from program space
		if (tabid == groupid) {
			func = (void *) pgm_read_word(&mcueecbtable[cnt].createfunc);	// Read function pointer from program space
			return func(memptr);
		}
	}
	return 0;
}


/***************************************************************************
* Populate data to 'uart' group created in memory
* [19/08/2015]
***************************************************************************/
eegrpsizeDef eegrpcreate_uart(MCUEEARGDEF_GRPFUNC) {
	eegrpsizeDef	groupsize = 0;


	groupsize += SIZEOF_EEDTENT(atbaudrateenum);
	groupsize += SIZEOF_EEDTENT(atparitydef);
	groupsize += 2 * (SIZEOF_EEDTENT(atuarttodef));	// Timeout, TxDelay
	groupsize += SIZEOF_EEDTENT(uint16_t);			// t35
	groupsize += SIZEOF_EEDTENT(DevAddrDef);


	if (memptr) {
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.brenum, sizeof(atbaudrateenum), eedten_uart_baudrate, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.parity, sizeof(atparitydef), eedten_uart_parity, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.txdelayl, sizeof(atuarttodef), eedten_uart_txdelay, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.timeoutl, sizeof(atuarttodef), eedten_uart_timeout, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.t35, sizeof(uint16_t), eedten_uart_t35, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.devaddr, sizeof(DevAddrDef), eedten_uart_address, 1);
	}
	return groupsize;
}


/***************************************************************************
* Populate data to 'powman' group created in memory
* [17/06/2015]
***************************************************************************/
eegrpsizeDef eegrpcreate_powman(MCUEEARGDEF_GRPFUNC) {
	eegrpsizeDef	groupsize;


	groupsize = SIZEOF_EEDTENT(MXpower.cfg.thadc3v2);
	if (memptr) {
		eepopoulate_datamem(memptr, (uint8_t *) &MXpower.cfg.thadc3v2, sizeof(MXpower.cfg.thadc3v2), eedten_powman_thadc3v2, 1);
	}
	return groupsize;
}


/***************************************************************************
* Populate data to 'board' group created in memory
* [15/06/2015]
***************************************************************************/
eegrpsizeDef eegrpcreate_board(MCUEEARGDEF_GRPFUNC) {
	eegrpsizeDef	groupsize = 0;


	if (boardio.diptr) {
		groupsize += (boardio.diptr->count * (
				SIZEOF_EEDTENT(boardio.diptr->mode[0]) +
				SIZEOF_EEDTENT(boardio.diptr->filterconst[0])));
	}
	if (boardio.doptr) {
		groupsize += (boardio.doptr->count * (
				SIZEOF_EEDTENT(boardio.doptr->mode[0]) +
				SIZEOF_EEDTENT(boardio.doptr->pulsedur[0])));
	}
	if (groupsize && memptr) {
		if (boardio.diptr) {
			eepopoulate_datamem(memptr, (uint8_t *) boardio.diptr->mode, sizeof(boardio.diptr->mode[0]), eedten_board_dimode00, boardio.diptr->count);
			eepopoulate_datamem(memptr, (uint8_t *) boardio.diptr->filterconst, sizeof(boardio.diptr->filterconst[0]), eedten_board_diflt00, boardio.diptr->count);
		}
		if (boardio.doptr) {
			eepopoulate_datamem(memptr, (uint8_t *) boardio.doptr->mode, sizeof(boardio.doptr->mode[0]), eedten_board_domode00, boardio.doptr->count);
			eepopoulate_datamem(memptr, (uint8_t *) boardio.doptr->pulsedur, sizeof(boardio.doptr->pulsedur[0]), eedten_board_dopul00, boardio.doptr->count);
		}
	}
	return groupsize;
}


/***************************************************************************
* Populate data entry created in memory
* [15/06/2015]
***************************************************************************/
void eepopoulate_datamem(uint8_t **memptr, uint8_t *wrdata, uint8_t size, uint8_t basedten, uint8_t count) {
	uint8_t			cnt;


	for (cnt = 0; cnt < count; cnt++) {
		((mcueedthead *) *memptr)->type = basedten + cnt;
		((mcueedthead *) *memptr)->size = size;
		*memptr += sizeof(mcueedthead);


		(*memptr)[0] = wrdata[cnt * size];
		if (size > 1) (*memptr)[1] = wrdata[(cnt * size) + 1];
		if (size > 2) {
			(*memptr)[2] = wrdata[(cnt * size) + 2];
			(*memptr)[3] = wrdata[(cnt * size) + 3];
		}
		*memptr += size;
	}
}


/***************************************************************************
* Write group header to EEPROM
* [16/06/2015]
***************************************************************************/
void eepopoulate_grouphead(uint8_t **memptr, mcueegrpenum groupid, uint16_t size) {


	((mcueegrphead *) *memptr)->id = groupid;
	((mcueegrphead *) *memptr)->size = size;
	*memptr += sizeof(mcueegrphead);
}


/***************************************************************************
* Check CRC of the EEPROM configuration
* [18/06/2015]
***************************************************************************/
uint8_t eeconf_crc(uint8_t updatecrc) {
	uint16_t			cnt;
	uint8_t 			rdbyte;
	uint16_t 			calccrc = 0xFFFF;
	uint16_t 			eecrc;
	eecfgsizeDef		configsize;


	configsize = eeprom_read_word((uint16_t *) (sizeof(mcueeheader) - sizeof(eecfgsizeDef)));


	for (cnt = 0; cnt < configsize; cnt++) {
		rdbyte = eeprom_read_byte((uint8_t *) cnt);
		buildCRC16(&calccrc, rdbyte);
	}


	if (updatecrc) {
		eeprom_write_word((uint16_t *) configsize, calccrc);		// Append CRC at the end of the configuration
		return EXIT_SUCCESS;							// We just updated CRC so surely it is correct :)
	}
	else{
		eecrc = eeprom_read_word((uint16_t *) configsize);	// Read CRC from EEPROM
		if (calccrc == eecrc) return EXIT_SUCCESS;		// Correct CRC
	}
	return EXIT_FAILURE;								// Incorrect CRC
}


/***************************************************************************
* Resolve EEPROM group identifier from Modbus register
* [15/06/2015]
* Fixed: Exclude DO register when resolving EEPROM group
* Data validation added
* [17/08/2015]
***************************************************************************/
mcueegrpenum eegroupid_resolve(atmappingenum mapreg, ModData16bitDef val, uint8_t *existsincfg) {

	if (existsincfg) *existsincfg = 1;


	if (
			(mapreg >= atmapen_dimode00) &&
			(mapreg <= atmapen_dimode0F)) {
		if ((val < modbusdimden_spi) || (val > modbusdimden_spi)) goto failed;
		return eegren_board;
	}


	else if (
			(mapreg >= atmapen_domode00) &&
			(mapreg <= atmapen_domode0F)) {
		if ((val < modbusdomden_pulseout) || (val > modbusdomden_pulseout)) goto failed;
		return eegren_board;
	}


	else if (
			(mapreg >= atmapen_dif00) &&
			(mapreg <= atmapen_dif0F)) {
		if (val < 1) goto failed;
		return eegren_board;
	}


	else if (
			(mapreg >= atmapen_dopul00) &&
			(mapreg <= atmapen_dopul0F)) {
		if (val < 1) goto failed;
		return eegren_board;
	}


	else if (mapreg == atmapen_3v2th) {
		return eegren_powman;
	}


	else if (
			(mapreg >= atmapen_baudrate) &&
			(mapreg <= atmapen_devaddr)) {
		if (uartsettvalidate(mapreg, val) == EXIT_FAILURE) goto failed;
		return eegren_uart0;
	}

	// TODO resolve other groups
	else {
		if (existsincfg) *existsincfg = 0;	// Mapped register is not part of EEPROM configuration
	}


	failed:
	return 0;	//  Mapped register is not part of EEPROM configuration or data validation failed
}
