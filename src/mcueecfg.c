/*
 ============================================================================
 Name        : mcueecfg.c
 Author      : AK
 Version     : V3.00
 Copyright   : Property of Londelec UK Ltd
 Description : MCU EEPROM configuration parser

  Change log :

  *********V3.00 03/06/2018**************
  Fixed: Maximal configuration size is used when searching groups
  Fixed: Data size 1..4 limit check added
  Backup configuration created
  EEPROM header is checked and updated in runtime now
  Read and write bitsets removed
  AI configuration added

  *********V2.02 07/09/2016**************
  Fixed: eesearch data function returns 0 pointer if 0xff (empty) byte is encountered in EEPROM
  Local functions marked static
  UART interface type added

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
#include <string.h>
//#include <stdint.h>
//#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>



#include "leiodcat.h"
#include "ledefs.h"
#include "mcueecfg.h"
#include "board.h"
#include "powman.h"
#include "modbus.h"




#ifdef GLOBAL_DEBUG
//#define FORCE_WRITE				// Overwrite EEPROM contents even if they are the same, this considerably extends EEPROM writning time and if the MCU is reset during write, can get corrupted data
#endif	// GLOBAL_DEBUG


#define SIZEOF_EEDTENT(mstruct)		(sizeof(mstruct) + sizeof(mcueedthead))
#define MCUEE_MAXSIZE			(EEPROM_SIZE >> 1) - 2		// Maximal configuration size excluding CRC
#define MCUEE_BACKUP_BASE		(EEPROM_SIZE >> 1)


typedef struct mcueegrphead_s {
	mcueegrp_e			id;							// Group ID (UART, Powman, etc)
	eegrpsize_t			size;						// Group size excluding group header
} mcueegrphead;


typedef struct mcueedthead_s {							// Data header
	EEDATA_HEADER										// Data type (enum) and size - byte, word, dword
} mcueedthead;


// Macro for argument definitions
#define MCUEEARG_GRPFUNC uint8_t **memptr


static eegrpsize_t eegrpcreate_uart(MCUEEARG_GRPFUNC);
static eegrpsize_t eegrpcreate_powman(MCUEEARG_GRPFUNC);
static eegrpsize_t eegrpcreate_board(MCUEEARG_GRPFUNC);


static const struct {
	mcueegrp_e			id;							// Group ID (UART, Powman, etc)
	eegrpsize_t			(*createfunc)(MCUEEARG_GRPFUNC);	// Function that creates group and populates data
} mcueecbtable[] PROGMEM = {
	{eegren_uart0, 		eegrpcreate_uart},
	{eegren_powman, 	eegrpcreate_powman},
	{eegren_board, 		eegrpcreate_board},
};


static const mcueeheader PROGMEM eepromhdr = {
		.name = "LEIODC     ",
		.revmajor = '1',
		.delimiter = '.',
		.revminor = '1',
		.size = sizeof(mcueeheader),
};




/*
 * Populate data entry created in memory
 * [15/06/2015]
 * memcpy function used
 * [08/09/2016]
 */
static void eepopoulate_datamem(uint8_t **memptr, uint8_t *wrdata, uint8_t size, uint8_t basedten, uint8_t count) {
	uint8_t			i;


	for (i = 0; i < count; i++) {
		((mcueedthead *) *memptr)->type = basedten + i;
		((mcueedthead *) *memptr)->size = size;

		memcpy((*memptr) + sizeof(mcueedthead), &wrdata[i * size], size);
		(*memptr) += size + sizeof(mcueedthead);
	}
}


/*
 * Search group in the EEPROM using identifier
 * [14/06/2015]
 * Read EEPROM block function used
 * [10/09/2016]
 * Fixed: Maximal configuration size is used when searching groups
 * [31/07/2020]
 */
static leptr eegroup_find(mcueegrp_e groupid, eegrpsize_t *groupsize) {
	leptr			grptr;
	mcueegrphead	grouphead;


	for (grptr = sizeof(mcueeheader); grptr < boardio.eesize; grptr += grouphead.size + sizeof(mcueegrphead)) {
		eeprom_read_block(&grouphead, (const void *) grptr, sizeof(grouphead));
		if (grouphead.id == eegren_undefined)						// Group undefined (most likely empty memory)
			break;


		if (grouphead.id == groupid) {	// Group found
			if (groupsize)
				*groupsize = grouphead.size;
			return grptr;
		}
	}


	if (groupsize)
		*groupsize = 0;
	return 0;		// Group is not found
}


/*
 * Search data in the EEPROM using identifier
 * [15/06/2015]
 * Fixed: Return 0 pointer if 0xff (empty) byte is encountered in EEPROM
 * Read EEPROM block function used
 * [10/09/2016]
 * Fixed: Check if data size is within limits 1..4
 * [05/08/2020]
 */
static leptr eedata_find(uint8_t dataid, leptr grptr, eegrpsize_t groupsize, uint32_t *rddword) {
	leptr			dataptr;
	mcueedthead		datahead;


	*rddword = 0;	// Clean dword

	for (dataptr = grptr + sizeof(mcueegrphead); dataptr < (grptr + sizeof(mcueegrphead) + groupsize); dataptr += datahead.size + sizeof(mcueedthead)) {
		eeprom_read_block(&datahead, (const void *) dataptr, sizeof(datahead));
		if ((datahead.type == 0xff) || (!datahead.size) || (datahead.size > 4))
			return 0;	// Data type undefined (most likely empty memory)


		if (datahead.type == dataid) {	// Data ID found
			eeprom_read_block(rddword, (const void *) dataptr + sizeof(mcueedthead), datahead.size);
			return dataptr;
		}
	}
	return 0;	// Data is not found
}


/*
 * Resolve group creation function from table and execute it
 * [20/08/2015]
 */
static eegrpsize_t eegrpfunc(mcueegrp_e groupid, uint8_t **memptr) {
	uint8_t			i;
	mcueegrp_e		tabid;
	eegrpsize_t		(*func)(MCUEEARG_GRPFUNC);


	for (i = 0; i < ARRAY_SIZE(mcueecbtable); i++) {
		tabid = pgm_read_byte(&mcueecbtable[i].id);		// Read group id from program space
		if (tabid == groupid) {
			func = (void *) pgm_read_word(&mcueecbtable[i].createfunc);	// Read function pointer from program space
			return func(memptr);
		}
	}
	return 0;
}


/*
 * Populate data to 'uart' group created in memory
 * [19/08/2015]
 * UART interface type added
 * [08/09/2016]
 */
static eegrpsize_t eegrpcreate_uart(MCUEEARG_GRPFUNC) {
	eegrpsize_t	groupsize;


	groupsize =
			SIZEOF_EEDTENT(atbaudrate_e) +
			SIZEOF_EEDTENT(atparity_t) +
			(2 * (SIZEOF_EEDTENT(atuartto_t))) + 	// Timeout, TxDelay
			SIZEOF_EEDTENT(uint16_t) + 				// t35
			SIZEOF_EEDTENT(Modaddr_t) +
			SIZEOF_EEDTENT(uint8_t);				// Interface RS232/485/422


	if (memptr) {
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.bren16, sizeof(atbaudrate_e), eedten_uart_baudrate, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.parity, sizeof(atparity_t), eedten_uart_parity, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.txdelayl, sizeof(atuartto_t), eedten_uart_txdelay, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.timeoutl, sizeof(atuartto_t), eedten_uart_timeout, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.t35, sizeof(uint16_t), eedten_uart_t35, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.devaddr, sizeof(Modaddr_t), eedten_uart_address, 1);
		eepopoulate_datamem(memptr, (uint8_t *) &boardio.uartee.uartif, sizeof(uint8_t), eedten_uart_iface, 1);
	}
	return groupsize;
}


/*
 * Populate data to 'powman' group created in memory
 * [17/06/2015]
 */
static eegrpsize_t eegrpcreate_powman(MCUEEARG_GRPFUNC) {
	eegrpsize_t	groupsize;


	groupsize = SIZEOF_EEDTENT(MXpower.cfg.thadc3v2);
	if (memptr) {
		eepopoulate_datamem(memptr, (uint8_t *) &MXpower.cfg.thadc3v2, sizeof(MXpower.cfg.thadc3v2), eedten_powman_thadc3v2, 1);
	}
	return groupsize;
}


/*
 * Populate data to 'board' group created in memory
 * [15/06/2015]
 * AI configuration added
 * [03/06/2018]
 */
static eegrpsize_t eegrpcreate_board(MCUEEARG_GRPFUNC) {
	eegrpsize_t	groupsize = 0;


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
	if (boardio.aiptr) {
		groupsize += (boardio.aiptr->count * (
				SIZEOF_EEDTENT(boardio.aiptr->mode[0])));
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


/*
 * Write group header to EEPROM
 * [16/06/2015]
 */
static void eegroup_head(uint8_t **memptr, mcueegrp_e groupid, uint16_t size) {


	((mcueegrphead *) *memptr)->id = groupid;
	((mcueegrphead *) *memptr)->size = size;
	*memptr += sizeof(mcueegrphead);
}


/*
 * Resolve EEPROM group identifier from Modbus register
 * [15/06/2015]
 * Fixed: Exclude DO register when resolving EEPROM group
 * Data validation added
 * [17/08/2015]
 * UART interface type added
 * [10/09/2016]
 */
mcueegrp_e eedata_validate(atmapping_e mapreg, Moddata16_t val, uint8_t *valid) {
	mcueegrp_e		groupid = 0;


	if (valid)
		*valid = 0;


	if (
			(mapreg >= atmapen_dimode00) &&
			(mapreg <= atmapen_dimode0F)) {
		groupid = eegren_board;

		if ((val < modbusdimden_spi) || (val > modbusdimden_spi))
			goto failed;
	}
	else if (
			(mapreg >= atmapen_domode00) &&
			(mapreg <= atmapen_domode0F)) {
		groupid = eegren_board;

		if ((val < modbusdomden_pulseout) || (val > modbusdomden_pulseout))
			goto failed;
	}
	else if (
			(mapreg >= atmapen_aimode00) &&
			(mapreg <= atmapen_aimode07)) {
		groupid = eegren_board;

		if ((val < modbusaimden_sint) || (val > modbusaimden_uintoffs))
			goto failed;
	}
	else if (
			(mapreg >= atmapen_dif00) &&
			(mapreg <= atmapen_dif0F)) {
		groupid = eegren_board;

		if (val < 1)
			goto failed;
		//return eegren_board;
	}
	else if (
			(mapreg >= atmapen_dopul00) &&
			(mapreg <= atmapen_dopul0F)) {
		groupid = eegren_board;

		if (val < 1)
			goto failed;
		//return eegren_board;
	}
	else if (mapreg == atmapen_3v2th) {
		groupid = eegren_powman;
		//return eegren_powman;
	}
	else if (
			(mapreg >= atmapen_baudrate) &&
			(mapreg <= atmapen_uartif)) {
		groupid = eegren_uart0;

		if (uartsettvalidate(mapreg, val) == LE_FAIL)
			goto failed;
		//return eegren_uart0;
	}
	else {	// TODO resolve other groups
		goto failed;		// Mapped register is not part of EEPROM configuration
	}


	if (valid)
		*valid = 1;


	failed:
	return groupid;	// Return resolved group ID or 0 if mapped register is not part of EEPROM configuration
}


/*
 * Check CRC of the EEPROM configuration
 * [18/06/2015]
 * Configuration size variable added to board structure
 * [05/08/2020]
 */
static int eeconf_crc(int backup, int update) {
	uint16_t			i;
	uint8_t 			rdbyte;
	uint16_t 			ccrc = 0xffff;
	uint16_t 			eecrc;
	leptr				eebase = backup ? MCUEE_BACKUP_BASE : 0;


	for (i = 0; i < boardio.eesize; i++) {
		rdbyte = eeprom_read_byte((uint8_t *) eebase);
		calc_crc16(&ccrc, MODBUS_CRC16_POLY, rdbyte);
		eebase++;
	}


	if (update) {
		eeprom_update_word((uint16_t *) eebase, ccrc);	// Append CRC at the end
		return LE_OK;									// We have just updated CRC so it is defintely correct :)
	}
	else {
		eecrc = eeprom_read_word((uint16_t *) eebase);	// Read CRC from EEPROM
		if (ccrc == eecrc)
			return LE_OK;		// Correct CRC
	}
	return LE_FAIL;				// Incorrect CRC
}


/*
 * Check header and CRC of the EEPROM configuration
 * [05/08/2020]
 */
int eeconf_validate(int backup) {
	mcueeheader		eeheader;
	leptr			eebase = backup ? MCUEE_BACKUP_BASE : 0;


	eeprom_read_block(&eeheader, (void *) eebase, sizeof(eeheader));
	boardio.eesize = eeheader.size;


	if (memcmp_P(&eeheader, (const void *) &eepromhdr, sizeof(eeheader) - sizeof(eeheader.size)))
		goto failed;

	if ((eeheader.size <= sizeof(eeheader)) || (eeheader.size >= MCUEE_MAXSIZE))
		goto failed;

	if (eeconf_crc(backup, 0) == LE_OK)
		return LE_OK;
	else
		boardio.rflags |= BOARDRF_EECONF_CORRUPTED;


	failed:
	boardio.eesize = 0;		// This shows the configuration is invalid
	return LE_FAIL;
}


/*
 * Get configuration data from EEPROM
 * [28/12/2014]
 * EEPROM data parsing moved to separate functions
 * [16/06/2015]
 * Request update argument added
 * [08/09/2016]
 * Configuration size variable added to board structure
 * [05/08/2020]
 */
int eeconf_get(mcueegrp_e groupid, uint8_t dataid, uint32_t *rddword) {
	leptr			grptr, dataptr;
	eegrpsize_t		groupsize;


	if (boardio.eesize) {		// Size 0 means configuration is corrupted
		if ((grptr = eegroup_find(groupid, &groupsize))) {
			if ((dataptr = eedata_find(dataid, grptr, groupsize, rddword)))
				return LE_OK;
		}
		boardio.eesize = 0;
	}
	return LE_FAIL;
}


/*
 * Check if it is possible to write received byte to EEPROM
 * [18/08/2015]
 * Data value validation moved separate function
 * [10/09/2016]
 */
int eegroup_validate(mcueegrp_e groupid) {
	leptr			grptr;
	eegrpsize_t		groupsize, newsize;


	if ((grptr = eegroup_find(groupid, &groupsize))) {
		newsize = eegrpfunc(groupid, NULL);	// This is just to get a size of the new group which is going to be created
		if (groupsize == newsize) 			// Ensure size of the group currently stored in the EEPROM is equal to the group which is about to be created
			return LE_OK;	// It is possible to save received data to EEPROM
	}
	return LE_FAIL;		// Validation failed, impossible to save group to EEPROM
}


/*
 * Update configuration group
 * [17/06/2015]
 * Group create functions generalized and their pointers stored in a table
 * [18/08/2015]
 */
void eeconf_update(Modbusreg_t *regmem) {
	mcueegrp_e		groupid;
	leptr			grptr;
	eegrpsize_t		groupsize;
	uint8_t 		*membuff;


	if ((groupid = eedata_validate(regmem->reg, *regmem->wrdata, NULL))) {	// Search EEPROM group to be updated based on modbus register
		if ((grptr = eegroup_find(groupid, &groupsize))) {
			if (!(membuff = calloc(groupsize, 1)))
				return;

			uint8_t	*memptr = membuff;
			eegrpfunc(groupid, &memptr);			// Populate memory with group data
			eeprom_update_block(membuff, (void *) (grptr + sizeof(mcueegrphead)), groupsize);
			eeconf_crc(0, 1);
			LEF_FREE(membuff);
		}
	}
}


/*
 * Backup EEPROM configuration or restore from backup
 * [05/08/2020]
 */
void eeconf_copy(int backup) {
	mcueeheader		eeheader;
	uint8_t 		*membuff = NULL;
	leptr			srcbase = backup ? 0 : MCUEE_BACKUP_BASE;
	leptr			dstbase = backup ? MCUEE_BACKUP_BASE : 0;


	eeprom_read_block(&eeheader, (void *) srcbase, sizeof(eeheader));

	if (!(membuff = malloc(eeheader.size + 2)))
		return;

	eeprom_read_block(membuff, (void *) srcbase, eeheader.size + 2);
#ifdef FORCE_WRITE
	eeprom_write_block(membuff, (void *) dstbase, eeheader.size + 2);
#else
	eeprom_update_block(membuff, (void *) dstbase, eeheader.size + 2);
#endif
	LEF_FREE(membuff)
}


/*
 * Build new EEPROM configuration
 * [05/08/2020]
 */
void eeconf_rebuild(void) {
	uint8_t			bitcnt;
	mcueeheader		eeheader;
	eegrpsize_t		groupsize;
	uint8_t 		*memptr, *membuff = NULL;
	leptr			eeptr = sizeof(eeheader);


	if (!memcpy_P(&eeheader, (const void *) &eepromhdr, sizeof(eeheader)))
		goto fail;

	eeheader.size = sizeof(eeheader);	// Set initial size

	for (bitcnt = 0; bitcnt < 16; bitcnt++) {
		if (boardio.eeupdatebs & (1 << bitcnt)) {	// Select groups required for particular hardware
			if ((groupsize = eegrpfunc(bitcnt, NULL))) 	// Get size of the new group that is going to be created
				eeheader.size += groupsize + sizeof(mcueegrphead);
		}
	}


	boardio.eesize = eeheader.size;
#ifdef FORCE_WRITE
	eeprom_write_block(&eeheader, NULL, sizeof(eeheader));
#else
	eeprom_update_block(&eeheader, NULL, sizeof(eeheader));
#endif

	for (bitcnt = 0; bitcnt < 16; bitcnt++) {		// Build configuration and write to EEPROM
		if (boardio.eeupdatebs & (1 << bitcnt)) {
			if ((groupsize = eegrpfunc(bitcnt, NULL))) {	// Get size of the new group that is going to be created
				if (!(membuff = realloc(membuff, groupsize)))
					goto fail;

				memptr = membuff;

				eegroup_head(&memptr, bitcnt, groupsize);
				eegrpfunc(bitcnt, &memptr);
#ifdef FORCE_WRITE
				eeprom_write_block(membuff, (void *) (eeptr), memptr - membuff);
#else
				eeprom_update_block(membuff, (void *) (eeptr), memptr - membuff);
#endif
				eeptr += (memptr - membuff);
			}
		}
	}

	LEF_FREE(membuff)
	eeconf_crc(0, 1);
	return;


	fail:
	boardio.eesize = 0;
}

