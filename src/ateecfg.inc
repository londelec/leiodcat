/*
============================================================================
 Name        : ateecfg.inc
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Include file for Atmel EEPROM configuraiton

  Change log  :

  *********V1.00 28/12/2014**************
  Initial revision

 ============================================================================
 */

//#ifndef EECFGINIT_H_
//#define EECFGINIT_H_


struct mcueecfgstr {									// Set the size of configration
	mcueeheader				eeheader;					// Configuration header, fixed length = 16bytes
	mcueegrphead			uart0_head;					// UART group head
	eeuartbaudrate			uart0_baudrate;				// UART baudrate structure		
	eeuartparity			uart0_parity;				// UART parity structure
	eeuarttxdelay			uart0_txdelay;				// UART tx delay structure
	eeuartaddress			uart0_address;				// UART device address
	mcueegrphead			powman_head;				// Power manager group head
	eepowmanthadc3v2		powman_thadc3v2;			// ADC threshold for 3.2V detection
	mcueegrphead			board_head;					// Board settings group head
	eeboarddiflt			board_diflt00;				// DI filter structure
	eeboarddohld			board_dohld00;				// D0 hold period structure
};


struct mcueecfgstr EEMEM eepromcfg = {					// Initialize configuration data
		.eeheader.name = "LEIODC     ",
		.eeheader.revmajor = 1,
		.eeheader.delimiter = '.',
		.eeheader.revminor = 1,
		.eeheader.size = sizeof(struct mcueecfgstr),

		.uart0_head.id = eegren_uart0,
		.uart0_head.size = (((uint16_t) &eepromcfg.powman_head) - ((uint16_t) &eepromcfg.uart0_head) - sizeof(mcueegrphead)),
		.EEDATA_ENTER(uart0_baudrate, uart, baudrate, 115200)
		.EEDATA_ENTER(uart0_parity, uart, parity, 2)
		.EEDATA_ENTER(uart0_txdelay, uart, txdelay, 500)
		.EEDATA_ENTER(uart0_address, uart, address, 1)

		.powman_head.id = eegren_powman,
		.powman_head.size = sizeof(eepowmanthadc3v2),
		.EEDATA_ENTER(powman_thadc3v2, powman, thadc3v2, POWADC_3V2_VALUE)
		
		.board_head.id = eegren_board,
		.board_head.size = sizeof(eeboarddiflt) + sizeof(eeboarddohld),
		//.EEDATA_ENTER(board_diflt00, board, diflt01, 1000)
		//.EEDATA_ENTER(board_dohld00, board, dohld02, 100)
};


//#endif /* EECFGINIT_H_ */
