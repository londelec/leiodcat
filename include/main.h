/*
 ============================================================================
 Name        : main.h
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Data Concentrator main module

  Change log  :

  *********V1.00 20/02/2015**************
  Initial revision

 ============================================================================
 */

#ifndef MAIN_H_
#define MAIN_H_


//#include <sys/time.h>		// Unix time and timers; capability manipulation
//#include <netdb.h>			// Socket structures
//#include <termios.h>		// Termios structures

#include "ledefs.h"
#include "modbusdef.h"
//#include "letime.h"
#include "atmeldefs.h"
#include "timer.h"


// General constants
#define	ABSOLUTE_PATH_LENGTH			500				// Absolute path including filename length
#define LOGFILE_PATH_LENGTH				200				// Logfile path length
#define XML_FILENAME_LENGTH				100				// XML configuration file path length
#define OUTPUTSTR_LENGTH				512				// Output string length
#define SYSSTATSTR_LENGTH				200				// System status diagnostic file string length


// Timing Constants
#define MULT10USEC						100000			// Multiplier to convert from sec to 10usec
#define MAINSLEEP						100000			// Main loop sleep in nanoseconds, default 0.1ms
#define MIN_TIMESYNC_INTERVAL			3600			// Minimal System time synchronization interval in seconds
#define SYS_REBOOT_DELAY				2				// System reboot delay in seconds


// UART constants
#define UARTPathLen						100				// Device hardware path length
#define UARTParityLen					1				// UART parity char length
#define DEFAULT_UARTIO_DEBOUNCE			8				// Debounce counter for reading UART IO
#define DEFAULT_UART_DATABITS			8				// UART Data bits
#define DEFAULT_UART_STOPBITS			1				// UART Stop bits
#define DEFAULT_TXDELAY_MULT			44				// UART Tx delay multiplier (# of bits)


// IPv4 Socket constants
//#define InterfaceLen					10				// ETH interface name length
#define IPv4AddrLen						15				// IPv4 Address length ('x.x.x.x')
#define HostnameLen						200				// Internet host name length
#define DEFAULT_IPv4MASK				32				// Default IPv4 Client IP address mask
#define	DEFAULT_IPv4QUEUESIZE			4				// Default IPv4 TCP Server mode queue size
#define DEFAULT_SOCK_CONNECTTIMEOUT		5				// Socket connection timeout in seconds, applies to Client mode only
#define DEFAULT_SOCK_TIMEOUT			2				// Socket Timeout for Serial protocols, default 2sec
#define DEFAULT_SOCK_TXDELAY			0.1				// Socket Tx delay for Serial protocols, default 100ms
#define DEFAULT_SOCK_IDLETIMEOUT		120				// Socket Idle Timeout for Serial protocols, default 120sec


// Channel runtime flags (->chflags)
//#define	CHFLAG_SEND_DELAY				0x01			// Delay before Tx message timer is running, transmit after Timer overflow
#define	CHFLAG_FLUSH_UART				0x02			// Flush UART on system startup
#define	CHFLAG_SOCK_CONNECTING			0x04			// Socket Client is currently connecting to the remote Server


// Station runtime flags (->runflags)
#define	STARFLAG_NEW_SOCKET				0x01			// Indicates newly created TCP client connection to the socket
#define	STARFLAG_RESET_RX_STATE			0x02			// Reset receive state after UART timeout or new socket connection
#define	SERVICE_COMMSDISABLED			0x10			// Station communication disabled
#define	SERVICE_104STOPPED				0x20			// IEC104 protocol in Stopped mode (don't send STARTDT_act)
#define	STARFLAG_GLOBAL_TIMESYNC		0x40			// Global Time synchronization request for Serial stations


// Station configuration flags (->xmlflags)
#define	STAXF_TCPSERV_ACCEPT			0x01			// Always accept new connection request to TCP server and disconnect old socket


// Firmware Capability flags
#define	CAPF_SETTIME					0x01			// System Time setting capability
//#define	CAPF_RAWIO					0x02			// System Raw IO accessing capability


// Command line argument flags
#define	ARGF_QUIET						0x01			// Suppress all output to console
#define	ARGF_LICINFO					0x02			// Display license information and exit
#define	ARGF_IOCOUNT					0x04			// Display total IO count and exit
#define	ARGF_XMLVALID					0x08			// Validate XML and exit



// System Service commands (SYS_Service)
#define	SYS_FWReset						0x01			// Firmware reset request
#define	SYS_Reboot						0x02			// System reboot request
#define	SYS_ResetDelay					0x04			// Firmware reset delay active


// Service indication values
#define	STATION_ONLINE_DIQ				0x02			// Master Station Online (for application layer, service indications)
#define	STATION_OFFLINE_DIQ				0x01			// Master Station Offline (for application layer, service indications)
#define	STATION_ENABLED_DIQ				0x02			// Master Station Enabled (for application layer, service indications)
#define	STATION_DISABLED_DIQ			0x01			// Master Station Disabled (for application layer, service indications)
#define	STATION_104STARTED_DIQ			0x02			// 104 Master Station Active (for application layer, service indications)
#define	STATION_104STOPPED_DIQ			0x01			// 104 Master Station Standby (for application layer, service indications)
#define	UART_RIOFF_DIQ					0x01			// UART RI pin is currently -12V (for service indications)
#define	UART_RION_DIQ					0x02			// UART RI pin is currently +12V (for service indications)



// Station initialization XML flags
// (virtual, only used in XMLproc.c)
#define	XMLFLAG_TCPSERV_ACCEPT			0x08			// Always accept new connection request to TCP server and disconnect old socket
#define	XMLFLAG_SERVICE_104STOPPED		0x10			// IEC104 protocol Stopped mode (don't send startdt)
#define	XMLFLAG_SERVICE_DISABLED		0x80			// Virtual protocol communication disabled


#define MODBUS_SYSREG_COUNT				5				// Number of system register to be mapped
#define MODBUS_DIMULT					1				// Number of register for each DI (multiplier)
#define MODBUS_DOMULT					2				// Number of register for each DO (multiplier)



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
typedef	uint16_t						gprotiddef;			/* Generic Protocol ID size definition */
typedef	uint16_t						staiddef;			/* Station ID size definition */
typedef	uint8_t							cfilterdef;			/* Client filter ID size definition */
typedef	uint8_t							channeliddef;		/* Channel ID size definition */
typedef	uint32_t						ObjectCntDef;		/* Object count or counter size definition */
typedef	uint8_t							Qual_8bit;			/* Internal Qualifier size definition */
typedef	uint16_t						Qual_16bit;			/* Internal Qualifier size definition */
typedef	uint32_t						Qual_32bit;			/* Internal Qualifier size definition */
typedef	uint16_t						TxRx16bitDef;		/* Rx/Tx buffer pointer size definition */
typedef	uint32_t						BaudrateDef;		/* UART Baudrate size definition */
//typedef	uint32_t						TimerConstDef;		/* 32bit Timer Constant size definition */
typedef	uint8_t							Fifo_8bit;			/* FIFO buffer size definition */
typedef	uint8_t							DOMatrixDef;		/* DO Matrix size definition */
typedef	uint32_t						XMLValidDef;		/* XML validation variable size definition */
typedef	uint32_t						XMLVarDef;			/* XML read integer variable size definition */
typedef	float							XMLFloatDef;		/* XML read float variable size definition */
typedef	XMLFloatDef						XMLVerDef;			/* XML file Version size definition */

typedef	int32_t							SockTypeDef;		/* Socket type (SOCK_STREAM or SOCK_DGRAM) definition */
typedef	uint8_t							SockQueueDef;		/* Socket queue size definition */
typedef	uint16_t						SocketBufDef;		/* Socket Rx and Tx buffer pointer size definition */
typedef	uint8_t							State_8bit;			/* internal state size definition */
typedef	uint8_t							Status_8bit;		/* communication status size definition */
typedef	uint8_t							exportedfdef;		/* Exported flags size definition */
typedef	uint8_t							TxRx8bitDef;		/* IEC60870 Rx and Tx buffer pointer size definition */


// Defined protocol constants, values 0 and 254
// are not allowed. used for licensed IO validation
typedef enum {
	IEC101slConst 			= 1,
	IEC104slConst,
	IEC104slRConst,
	IEC104slCConst,
	IEC101maConst			= 0x41,
	IEC104maConst,
	IEC103maConst,
	IEC61850clConst,
	ModbusmaConst,
	IECDebugConst
} LEOPACK ProtocolTypeEnum;


// Socket types
// Don't change enumeration as these are used to cast
// string constant table in main.c
typedef enum {
	SockUDPNormal 			= 0,
	SockTCPServer 			= 1,
	SockTCPClient			= 2
} LEOPACK SockModeEnum;


// Don't change enumeration as these are used to cast tables
typedef enum {
	DItype					= 1,
	AItype					= 2,
	CTtype					= 3,
	DOtype					= 4,
	AOtype					= 5,
	objtypecount							// This is the count of objects, it MSUT be last enum
} LEOPACK ObjectTypeEnum;


// UART interface types
// Don't change enumeration as these are used to cast
// string constant table in main.c
typedef enum {
	RS232					= 0,
	RS485		 			= 1,
	RS422					= 2
} LEOPACK UartIntEnum;


// Command line argument specification
typedef enum {
	argquiet				= 1,
	arghelp,
	argversion,
	argmodules,
	arglicense,
	argiocount,
	argxmlvalidate
} LEOPACK ArgEnum;


// Channel communication states
// (function return values)
typedef enum {
	CHCommsEmpty			= 0,			// No data received or no data to send
	CHCommsRxTx,							// Data received from UART or Socket
	CHCommsDataError,						// Socket data error, socket close requested by protocol
	CHCommsDisabled,						// Station disabled by service
} LEOPACK CHStateEnum;


// Reason for Socket close
typedef enum {
	CHCloseEmpty			= 0,			// Unknown reason for socket closure
	CHCloseTCPdisconn,						// TCP socket which has been closed by peer
	CHCloseTCPcommslost,					// Connection to remote TCP peer is lost and socket Tx buffer overflowed
	CHCloseUDPdown,							// UDP only Eth cable is disconnected
	CHCloseUDPreject,						// Rejection received from UDP destination peer
	CHCloseRxerr,							// Unknown receive error
	CHCloseTxerr,							// Unknown send error
	CHCloseDataErr,							// Data error detected by linked protocol
	CHCloseDisabled,						// Station disabled by service
	CHCloseIdleT0,							// Idle timer expired
	CHCloseServNewconn,						// New incoming connection accepted
} LEOPACK CHSockCloseEnum;


// Station comms states
typedef enum {
	staenuninit				= 0,			// Station state is not initialized (after power on)
	staenonline,							// Station is online
	staenuninitoffltimer,					// Offline timer 1 is running after uninitialized state (station was never online)
	staenoffltimer1,						// Offline timer 1 is running after station has been online
	staenoffltimer2,						// Offline timer 2 is running
	staenoffline,							// Station is offline
	staendisabled,							// Station is disabled
} LEOPACK StaCommsEnum;


// Station comms callback function names
typedef enum {
	enumstacbdisable		= 1,			// Station is disabled, mark objects offline
	enumstacbsockclose,						// Socket used by station is closed
	enumstacboffline,						// Station must be marked offline (or not topical)
	enumstacbnoreply,						// Reply from outstation is missing, used only for serial master protocols
	enumstacbresetlink,						// No response counter is expired, need to reset link, used only for serial master protocols
	enumstacbcount							// This is the count of objects, it MSUT be last enum
} LEOPACK TempStaCbEnum;


// Channel serial communication states
typedef enum {
	enumchreadyrx			= 0,			// Ready to receive first byte
	enumchpretxdelay,						// Delay before transmission
	enumchreceiving,						// First byte received, continuing to receive
	enumchflush,							// Message receive error, flush following junk from buffer
} LEOPACK CHSerStateEnum;


// UART test or specific operating mode enums
typedef enum {
	enemuartecho				= 1,
} LEOPACK UartTestEnum;


// Modbus register addresses enums
typedef enum {
	atmapen_fwrev				= 0x0000,
	atmapen_temperature			= 0x0001,			// Temperature
	atmapen_tempcal85,								// Temperature calibration value at 85 degrees
	atmapen_vddio,									// VDDIO voltage measurement
	atmapen_3v2th,									// 3V2 measurement threshold
	atmapen_direg				= 0x0100,			// DI status register
	atmapen_dif00				= 0x0110,			// DI filters
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
	atmapen_doreg				= 0x0300,			// DO control/status register
	atmapen_dohld00				= 0x0310,			// DO hold periods
	atmapen_dohld01,
	atmapen_dohld02,
	atmapen_dohld03,
	atmapen_dohld04,
	atmapen_dohld05,
	atmapen_dohld06,
	atmapen_dohld07,
	atmapen_dohld08,
	atmapen_dohld09,
	atmapen_dohld0A,
	atmapen_dohld0B,
	atmapen_dohld0C,
	atmapen_dohld0D,
	atmapen_dohld0E,
	atmapen_dohld0F,
	atmapen_dopol00				= 0x0320,			// DO policies
	atmapen_dopol01,
	atmapen_dopol02,
	atmapen_dopol03,
	atmapen_dopol04,
	atmapen_dopol05,
	atmapen_dopol06,
	atmapen_dopol07,
	atmapen_dopol08,
	atmapen_dopol09,
	atmapen_dopol0A,
	atmapen_dopol0B,
	atmapen_dopol0C,
	atmapen_dopol0D,
	atmapen_dopol0E,
	atmapen_dopol0F,
} LEOPACK atmappingenum;



/*typedef struct ControlLockStr_ {
	struct StatStr_			*lockedsta;					// Station which is currently locked for polling
	struct StatStr_			*savedsta;					// Station pointer which was superseded by Control Lock activation
	nanotimedef				locktimer;					// Control Locking Timer
} ControlLockStr;*/


/*typedef struct UART_Settings_ {
	uint8_t					databits;					// UART Data bits count
	uint8_t					stopbits;					// UART Stop bits count
	BaudrateDef				baudrate;					// UART Baudrate
	uint8_t					parity;						// UART Parity (ASCII)
	UartIntEnum				interface;					// UART Interface 232/485/422
	//lechar					*devpath;					// UART Device Path
	//TimerConstDef			iordinterval;				// Hardware IO reading interval in seconds from XML configuration
	//nanotimedef				iordtimer;					// Hardware IO reading Timer value
	//uint8_t					iodebounceconst;			// Status read Debounce constant from XML configuration
	//uint8_t					iodebouncecnt;				// Status read Debounce counter
	//tcflag_t				lflag;						// Termios local flag
} UARTStr;*/


/*typedef struct IPv4_Settings_ {
	sockinetaddrdef			servinetaddr;				// Socket INET address structure (always Server address regardless of mode)
	sockinetaddrdef			localinetaddr;				// Socket INET address structure (for UDP sockets)
	SockTypeDef				socktype;					// SOCK_STREAM or SOCK_DGRAM
	SockModeEnum			sockmode;					// Server or Client
	SockQueueDef			sockqueue;					// Socket queue size for Socket Server mode
	TimerConstDef			reconntimeout;				// Reconnection timeout in sec for TCP Client and UDP modes
	nanotimedef				reconntimer;				// TCP Client reconnection timer for comparison to the monotonic time
	TimerConstDef			idletimeout;				// TCP mode idle monitoring timeout in sec for serial protocols and supervision
	nanotimedef				idletimer;					// TCP mode idle timer for comparison to the monotonic time
	lechar					*hostname;					// Internet host name
} IPv4_Settings;*/


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


/*typedef struct IPv4CliFilterStr_ {
	cfilterdef				filterid;					// IPv4 Client filter ID
	struct IPv4CliFilterStr_ *next;						// Next Client filter structure pointer
	sockinetaddrdef			cliinetaddr;				// Socket INET address structure (expected Client IP address and port)
	uint8_t					climask;					// Mask for verifying Client IP address
} IPv4CliFilterStr;


typedef struct GenObjectStr_ {
	ObjectCntDef			objcount[objtypecount];		// DI/AI/CT/DO/AO object count
	struct cmdMatrixStr_	*cmdmatrix;					// DO Matrix (populated only by Master protocols)
	DOMatrixDef				cmdmxsize;					// DO Matrix size (populated only by Master protocols)
	TimerConstDef			lockingsec;					// Control Locking Timer constant
} GenObjectStr;*/




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


/*typedef struct StatChainStr_ {
	struct StatChainStr_	*next;						// Next chain element
	StatStr					*statptr;					// Station structure pointer
} StatChainStr;




// Table structures
typedef struct ArgumentTableStr_ {
	lechar					*string;
	ArgEnum					entry;
} ArgumentTableStr;


typedef struct ProtocolIOStr_ {
	char					*string;
	ProtocolTypeEnum		entry;
	ObjectCntDef			IOs;
} ProtocolIOStr;


typedef struct UARTBaudrateStr_ {
	BaudrateDef				octalflag;
	BaudrateDef				baudrate;
} UARTBaudrateStr;


typedef struct ScriptPathStr_ {
	lechar					*string;
	uint8_t					absolute;
} ScriptPathStr;


typedef struct ProtNameStr_ {
	lechar					*string;
	ProtocolTypeEnum		type;
} ProtNameStr;


typedef struct StastatecbStr_ {
	void					*cb;
} StastatecbStr;*/


typedef struct hardwarenamestr_ {
	lechar					*string;
	athwenum				type;
} hardwarenamestr;





// Always define global variables in C source file
/*extern uint8_t			fwcapabilities;
extern leflags8bit		SYS_Service;*/
extern ChannelStr		*Channel0Ptr;
extern StatStr			*Station0ptr;
/*extern GenProtocolStr	*Gprotocol0ptr;
extern uint8_t			fwarguments;
extern uint8_t			SYS_RebootCause;
extern nanotimedef		ClockTime;
extern const lechar		*MainXMLfilename;
extern const lechar		*LoggerXMLfilename;

extern lechar 			Output_string[OUTPUTSTR_LENGTH];
extern lechar			*StdErrorPtr;*/
extern athwenum			BoardHardware;

extern uint8_t irqasmenum;



void comms_init();
void protocolrxproc();
void protocolmainproc();

ChannelStr *channelinit();
StatStr *stationinit();
GenProtocolStr *genprotinit();

uint8_t gethwname(lechar **stringptr);
uint8_t mappinginit(ModReg16bitDef reg, leptr *rdptr, leptr *wrptr);
void outputpinctrl(uint8_t disable);


#endif /* MAIN_H_ */
