/*
 ============================================================================
 Name        : main.h
 Author      : AK
 Version     : V1.01
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for LEIODC MCU main module

  Change log  :

  *********V1.01 12/06/2015**************
  Modbus mapping changes
  Hardware string table moved to flash memory

  *********V1.00 20/02/2015**************
  Initial revision

 ============================================================================
 */

#ifndef MAIN_H_
#define MAIN_H_




#include "ledefs.h"
#include "modbusdef.h"
//#include "letime.h"
#include "atmeldefs.h"
#include "timer.h"


// General constants
// Timing Constants
#define MULT10USEC						100000			// Multiplier to convert from sec to 10usec
#define MAINSLEEP						100000			// Main loop sleep in nanoseconds, default 0.1ms
#define MIN_TIMESYNC_INTERVAL			3600			// Minimal System time synchronization interval in seconds
#define SYS_REBOOT_DELAY				2				// System reboot delay in seconds


// Channel runtime flags (->chflags)
//#define	CHFLAG_SEND_DELAY				0x01			// Delay before Tx message timer is running, transmit after Timer overflow
//#define	CHFLAG_FLUSH_UART				0x02			// Flush UART on system startup
//#define	CHFLAG_SOCK_CONNECTING			0x04			// Socket Client is currently connecting to the remote Server


// Macros for argument definitions
#define CHARGDEF_CHACTFD ChannelStr *ChannelPtr, fddef *acthandler
#define CHARGDEF_CHACTFDINET CHARGDEF_CHACTFD, sockinetaddrdef *peerinetaddr
#define CHARGDEF_SENDFUNC CHARGDEF_CHACTFDINET, uint8_t *txbuffer, TxRx16bitDef txlength
#define CHARGDEF_RECVFUNC CHARGDEF_CHACTFDINET, uint8_t *rxbuffer, rxbytesdef *rxlength


#define STARGDEF_CB StatStr *staptr
#define STARGDEF_GETFIRSTAPP StatStr *staptr
#define STARGDEF_ONLINECHECK StatStr *staptr

// Protocol driver function arguments
#define DRVARGDEF_MAINPROC StatStr *staptr, uint8_t **txbuffptr, TxRx16bitDef *txlength
#define DRVARGDEF_CHINIT ChannelStr *chanptr, StatStr *staptr
#define DRVARGDEF_RX StatStr *staptr
#define DRVARGDEF_COMMSERR StatStr *staptr, uint8_t istimeout, uint8_t sockclosed
#define DRVARGDEF_REDUNINIT StatStr *staptr


// Commonly used function definitions
#define MAINF_NEXT_SERMASTER Nextserialmaster(staptr, &chanptr->serialsta)
//#define MAINF_SET_CHTIMEOUT set10ustimer(chanptr->chtimeout, &chanptr->chtimer);
//#define MAINF_SET_CHTXDELAY set10ustimer(chanptr->chtxdelay, &chanptr->chtimer);
//#define MAINF_SET_10USEC(mtconst, mtimer) set10ustimer(mtconst, mtimer);
//#define MAINF_SET_1SEC(mtconst, mtimer) set1sectimer(mtconst, mtimer);
//#define MAINF_CHECK_CHTIMER checktimer(&chanptr->chtimer)
//#define MAINF_CHECK_CHCHARTIMER checktimer(&chanptr->chchartimer)
#define MAINF_CHECK_STOFFTIMER checktimer(&staptr->offlinetimer)
//#define MAINF_CHECK_MTIMER(mtimer) checktimer(mtimer)

#define MAINF_SET_CHTIMEOUT timerset_fine(chanptr->chtimeout, &chanptr->chtimer);
#define MAINF_SET_CHTXDELAY timerset_fine(chanptr->chtxdelay, &chanptr->chtimer);
#define MAINF_SET_10USEC(mtconst, mtimer) timerset_fine(mtconst, mtimer);
#define MAINF_CHECK_CHTIMER timercheck_fine(&chanptr->chtimer)
#define MAINF_CHECK_CHCHARTIMER timercheck_fine(&chanptr->chchartimer)




#define STCOMMSERR_CBSET_INIT\
		StastatecbStr			cbset[enumstacbcount];\
		uint8_t					cnt;\
		for (cnt = 0; cnt < enumstacbcount; cnt++) {\
			cbset[cnt].cb = NULL;\
		}





// Overall project-wide type definitions
//typedef	uint16_t						gprotiddef;			/* Generic Protocol ID size definition */
//typedef	uint16_t						staiddef;			/* Station ID size definition */
//typedef	uint8_t							channeliddef;		/* Channel ID size definition */
typedef	uint16_t						TxRx16bitDef;		/* Rx/Tx buffer pointer size definition */
typedef	uint8_t							TxRx8bitDef;		/* IEC60870 Rx and Tx buffer pointer size definition */


// Channel communication states
// (function return values)
typedef enum {
	CHCommsEmpty			= 0,			// No data received or no data to send
	CHCommsRxTx,							// Data received from UART or Socket
	CHCommsDataError,						// Socket data error, socket close requested by protocol
	CHCommsDisabled,						// Station disabled by service
} LEOPACK CHStateEnum;


// Channel serial communication states
typedef enum {
	enumchreadyrx			= 0,			// Ready to receive first byte
	enumchpretxdelay,						// Delay before transmission
	enumchreceiving,						// First byte received, continuing to receive
	enumchflush,							// Message receive error, flush following junk from buffer
} LEOPACK CHSerStateEnum;




// Modbus register addresses enums
// Don't forget to update constant definitions
// when adding new enums
typedef enum {
	atmapen_fwrev				= 0x0000,
	atmapen_temperature			= 0x0001,			// Temperature, scaled
	atmapen_tempraw,								// Raw temperature value
	atmapen_tempcal85,								// Temperature calibration value at 85 degrees
	atmapen_vddio,									// VDDIO voltage measurement
	atmapen_3v2th,									// 3V2 measurement threshold
#define MODBUS_SYSREG_COUNT		6					// Number of system register to be mapped
	atmapen_direg				= 0x0100,			// DI status register
	atmapen_dimode00			= 0x0110,			// DI modes
	atmapen_dimode01,
	atmapen_dimode02,
	atmapen_dimode03,
	atmapen_dimode04,
	atmapen_dimode05,
	atmapen_dimode06,
	atmapen_dimode07,
	atmapen_dimode08,
	atmapen_dimode09,
	atmapen_dimode0A,
	atmapen_dimode0B,
	atmapen_dimode0C,
	atmapen_dimode0D,
	atmapen_dimode0E,
	atmapen_dimode0F,
	atmapen_dif00				= 0x0120,			// DI filters
	atmapen_dif01,
	atmapen_dif02,
	atmapen_dif03,
	atmapen_dif04,
	atmapen_dif05,
	atmapen_dif06,
	atmapen_dif07,
	atmapen_dif08,
	atmapen_dif09,
	atmapen_dif0A,
	atmapen_dif0B,
	atmapen_dif0C,
	atmapen_dif0D,
	atmapen_dif0E,
	atmapen_dif0F,
#define MODBUS_DIMULT			2					// Number of register for each DI (multiplier)
	atmapen_doreg				= 0x0300,			// DO control/status register
	atmapen_domode00			= 0x0310,			// DO modes
	atmapen_domode01,
	atmapen_domode02,
	atmapen_domode03,
	atmapen_domode04,
	atmapen_domode05,
	atmapen_domode06,
	atmapen_domode07,
	atmapen_domode08,
	atmapen_domode09,
	atmapen_domode0A,
	atmapen_domode0B,
	atmapen_domode0C,
	atmapen_domode0D,
	atmapen_domode0E,
	atmapen_domode0F,
	atmapen_dopul00				= 0x0320,			// DO pulse durations
	atmapen_dopul01,
	atmapen_dopul02,
	atmapen_dopul03,
	atmapen_dopul04,
	atmapen_dopul05,
	atmapen_dopul06,
	atmapen_dopul07,
	atmapen_dopul08,
	atmapen_dopul09,
	atmapen_dopul0A,
	atmapen_dopul0B,
	atmapen_dopul0C,
	atmapen_dopul0D,
	atmapen_dopul0E,
	atmapen_dopul0F,
#define MODBUS_DOMULT			2				// Number of register for each DO (multiplier)
} LEOPACK atmappingenum;




typedef struct ChannelStr_ {
	//channeliddef			chindex;					// Channel index
	//fddef					chfd;						// TCP Socket Handler for own socket
	struct ChannelStr_		*next;						// Next Channel Structure
	struct StatStr_			*serialsta;					// Currently selected station, only used for Serial stations
	//struct StatChainStr_	*chain;						// Station chain, only used for Packet stations
	//struct Supervisor_		*supervinst;				// Supervisor instance pointer
	uartatstr	 			usart;						// Atmel application specific UART structure
	//IPv4_Settings			*socketinst;				// Socket instance pointer
	//TimerConstDef			gtimesyncsec;				// Global time synchronization Constant in seconds from XML configuration
	//nanotimedef				gtimesynctimer;				// Global time synchronization Timer value, generate Time sync message after expiration
	//leunixsec				lastrealgsync;				// Last real time, when Global time synchronization command was sent
	//leflags8bit				chflags;					// Timer and socket connection flags
	TimerConstDef			chtxdelay;					// Delay before transmit in 10's us
	TimerConstDef			chtimeout;					// Timeout if no comms of reconnect socket in seconds
	//nanotimedef				chtimer;					// Timeout or delay timer for comparison to the monotonic time
	//nanotimedef				chchartimer;				// Timer for message end detection by measuring idle time after each received character
	finetmstr				chtimer;					// Timeout or delay timer for comparison to the monotonic time
	finetmstr				chchartimer;				// Timer for message end detection by measuring idle time after each received character
	CHSerStateEnum			chserstate;					// Serial communication state of the channel
	//struct ControlLockStr_	chcmdlock;					// Command lock structure
	//struct LogfileStr_		*logfile;					// Logfile structure pointer
} ChannelStr;


typedef struct StatStr_ {
	//staiddef				staid;						// Station Identifier
	//channeliddef			chid;						// Linked channel id
	//ProtocolTypeEnum		realptype;					// Real Protocol type
	//fddef					*fdptr;						// Socket or UART fd (TCP Server = remote client's socket handler; TCP Client >0 if connection established)
	//leflags8bit				runflags;					// Runtime flags, various indications
	//leflags8bit				xmlflags;					// XML configuration flags
	//StaCommsEnum			stastate;					// Station Online/Offline state
	//cfilterdef				clientfilterid;				// TCP Server mode allowed Clients filter
	//struct StatStr_			*redundmainptr;				// Main Station in a redundancy group pointer
	//uint8_t 				norespcnt;					// Missing reply counter
	//uint8_t 				norespconst;				// No response count from XML
	//uint8_t 				degradedconst;				// Degraded message count from XML
	//uint8_t 				degradedcnt;				// Degraded message counter
	//TimerConstDef			degradedtimeout;			// Degraded timer constant from XML
	//nanotimedef				degradedtimer;				// Degraded timer
	//TimerConstDef			offline1dsec;				// Offline 1 delay in seconds from XML
	//TimerConstDef			offline2dsec;				// Offline 2 delay in seconds from XML
	//nanotimedef				offlinetimer;				// Offline delay timer
	ChannelStr				*channelinst;				// Channel instance pointer
	//sockinetaddrdef			*peerinetaddr;				// Peer Socket INET address structure
	struct StatStr_			*next;						// Next Station pointer
	void					*realprotocol;				// Real Protocol unique structure
	//struct ServiceObjStr_	*servobjects;				// Service object structure
	void					*func_mainproc;				// Main processing function
	//void					*func_chinit;				// Channel initialization function
	void					*func_rx;					// Generic receive function
	//void					*func_commserr;				// Communication error (timeout, sock disconnect) handling function
	//void					*func_getfirstapp;			// Resolve first application layer function
	//void					*func_onlinecheck;			// Check is station is online function
	//void					*func_reduninit;			// Redundant station initialization function
	//struct LogfileStr_		*logfile;					// Logfile structure
} StatStr;


typedef struct GenProtocolStr_ {
	//gprotiddef				gpid;						// Generic Protocol identifier
	//ProtocolTypeEnum		realptype;					// Real Protocol type
	//gprotiddef	 			sourcegp;					// Source Generic protocol index (from leandc.xml)
	struct GenProtocolStr_	*next;						// Next Generic Protocol Structure pointer
	struct StatStr_			*statptr;					// Parent Station pointer
	//GenObjectStr			*objecttable;				// General information object tables
	void					*applayer;					// Real Protocol Application structure
	//void					*func_xmlslinit;			// Individual XML initialization function (for Slave protocols)
	//void					*func_xmlmainit;			// Individual XML initialization function (for Master protocols)
	//void					*func_resloveobj;			// Resolve objects function
	//void					*func_getinfaddr;			// Function to get information address of the source object
	//void					*func_commands;				// General command processing function
	//void					*func_evloginit;			// Event logger initialization function
	//void					*func_objlinkinit;			// Slave protocol object link initialization function
	//void					*func_legacyaiev;			// Legacy AI event initialization function
	//lechar					*xmlpath; 					// Generic Protocol XML configuration file name pointer (including path)
} GenProtocolStr;


typedef struct hardwarenamestr_  {
	athwenum				type;
	leptr					strptr;
} hardwarenamestr;




// Always define global variables in C source file
extern ChannelStr		*Channel0Ptr;
extern StatStr			*Station0ptr;
extern athwenum			BoardHardware;

extern uint8_t irqasmenum;



void comms_init();
void protocolrxproc();
void protocolmainproc();

ChannelStr *channelinit();
StatStr *stationinit();
GenProtocolStr *genprotinit();

uint8_t gethwname(lechar *hwnamebuff, uint8_t bufflen);
uint8_t mappinginit(ModReg16bitDef reg, leptr *rdptr, leptr *wrptr);
void outputpinctrl(uint8_t disable);
void zerofill(void *ptr, uint8_t size);


#endif /* MAIN_H_ */
