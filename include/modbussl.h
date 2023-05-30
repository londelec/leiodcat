/*
============================================================================
 Name        : Modbussl.h
 Author      : AK
 Version     : V2.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Modbus ASCII/RTU/TCP communication protocol Slave module

  Change log :

  *********V2.00 09/04/2019**************
  Adapted from leiodcat

  *********V1.03 07/09/2016**************
  Local function prototypes removed

  *********V1.02 18/08/2015**************
  Mapping write data pointer is 16bit pointer

  *********V1.01 12/06/2015**************
  New register search and count validation functions created

  *********V1.00 12/12/2014**************
  Initial revision

 ============================================================================
 */

#ifndef MODBUSSL_H_
#define MODBUSSL_H_


#include <stdint.h>

#include "ledefs.h"
#include "modbus.h"


// Default base addresses
// Used by Modbus Master
#define MODBUSSL_USERMAPSIZE			0x1000			// Modbus user mapping size
#if (MODBUSSL_TYPE == MODBUS_MCU)
#define MODBUSSL_EEBASE					0x9000			// Modbus register address for EEPROM mapping
#define MODBUSSL_EESIZE					MCUEE_EESIZE	// Modbus EEPROM mapping size (half of the actual size because of 16bit registers)
#endif




typedef struct Modbusreg_s {
	Modreg16_t			reg;						// Base register
	Modreg8_t			*rddata;					// Read data pointer 8bit
	Modreg16_t			*wrdata;					// Write data pointer 16bit
} Modbusreg_t;


typedef struct Modbussl_layer_s {
	//Flags_8bit				xmlflags;					// Flags from XML configuration by bits
	//Flags_8bit				appflags;					// Communication state force flags by bits
	//Modbusma_DIprivate		*DItable;					// DI object Table
	//Modbusma_AIprivate		*AItable;					// AI object Table
	//Modbusma_CTprivate		*CTtable;					// CT object Table
	//Modbusma_DOprivate		*DOtable;					// DO private table
	//Modbusma_AOprivate		*AOtable;					// AO private table
	//ModbusEvLogVars			evlogvar;					// Event Logger variables
	//ModbusmastateEnum		devstate;					// Device operation state
	Modreg16_t				eemapbase;					// EEPROM data mapping base register
	Modreg16_t				eemapsize;					// EEPROM data mapping size
	Modbusreg_t				*regmem;					// Register memory
	Modreg16_t				regcount;					// Register count
} Modbussl_layer;


genprot_t *Modbussl_create(station_t *staptr, channel_t *chanptr);
void Modbussl_postinit(genprot_t *gprot, uint8_t mapsize);


#endif /* MODBUSSL_H_ */
