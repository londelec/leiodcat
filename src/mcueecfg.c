/*
 ============================================================================
 Name        : mcueecfg.c
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : MCU EEPROM configuration parser

  Change log  :

  *********V1.00 28/12/2014**************
  Initial revision

 ============================================================================
 */

#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
//#include <avr/io.h>
#include <avr/eeprom.h>



#include "ledefs.h"
#include "mcueecfg.h"



const lechar *MCUeecfgVersion = " mcueecfgVersion=1.00 ";


#ifdef GLOBAL_DEBUG
//#define DEBUG_IGNORE_VDDIO
#endif	// GLOBAL_DEBUG



// Macros



/***************************************************************************
* Get configuration data from EEPROM
* [28/12/2014]
***************************************************************************/
uint8_t getee_data(mcueegrpenum groupid, uint8_t dataid, uint32_t *retdword) {
	leptr			groupptr, dataptr;
	mcueegrphead	grouphead;
	mcueedthead		datahead;


	groupptr = sizeof(mcueeheader);		// points to group after EEPROM header
	for (groupptr = sizeof(mcueeheader); groupptr < MCUEE_EESIZE; groupptr += grouphead.size + sizeof(mcueegrphead)) {
		grouphead.id = eeprom_read_byte((uint8_t *) groupptr);		// Read group identifier
		if (grouphead.id == eegren_undefined) return EXIT_FAILURE;	// Group undefined (most likely empty)
		grouphead.size = eeprom_read_word((uint16_t *) (groupptr + sizeof(grouphead.id)));


		if (grouphead.id == groupid) {	// Group ID matches required enum
			for (dataptr = groupptr + sizeof(mcueegrphead); dataptr < (groupptr + sizeof(mcueegrphead) + grouphead.size); dataptr += datahead.size + sizeof(mcueedthead)) {
				datahead.type = eeprom_read_byte((uint8_t *) dataptr);
				if (datahead.type == 0xFF) return EXIT_FAILURE;		// Data type undefined (most likely empty)
				datahead.size = eeprom_read_byte((uint8_t *) (dataptr + sizeof(datahead.type)));

				if (datahead.type == dataid) {	// Data ID matches required enum
					*retdword = 0;
					switch (datahead.size) {	// Select size to read
					case 1:
						*retdword = eeprom_read_byte((uint8_t *) (dataptr + sizeof(mcueedthead)));
						return EXIT_SUCCESS;
						break;

					case 2:
						*retdword = eeprom_read_word((uint16_t *) (dataptr + sizeof(mcueedthead)));
						return EXIT_SUCCESS;
						break;

					case 4:
						*retdword = eeprom_read_dword((uint32_t *) (dataptr + sizeof(mcueedthead)));
						return EXIT_SUCCESS;
						break;

					default:
						return EXIT_FAILURE;
					}
				}
			}
		}
	}
	return EXIT_FAILURE;
}

