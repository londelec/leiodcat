/*
 ============================================================================
 Name        : ledefs.h
 Author      : AK
 Version     : V1.18
 Copyright   : Property of Londelec UK Ltd
 Description : Global header file for Londelec C/C++ projects

  Change log :

  *********V1.18 16/09/2019**************
  Pointer container structure created

  *********V1.17 12/04/2019**************
  IPv6 address definitions added

  *********V1.16 06/09/2018**************
  String buffer structure added

  *********V1.15 27/03/2018**************
  Londelec version number definition created

  *********V1.14 10/11/2017**************
  Don't define ARRAY_SIZE and BOOL_CHECK if already defined

  *********V1.13 10/11/2017**************
  Bitset macros added

  *********V1.12 28/07/2016**************
  Internet port and file mode definition added

  *********V1.11 25/07/2016**************
  Clock monotonic is defined here only if not already defined in time.h

  *********V1.10 11/05/2016**************
  leserialdef, direntdef, dirdef, filedef, leuid, ledid definitions added

  *********V1.09 04/10/2015**************
  lseek offset definition added
  lefloat definition added
  aligned attribute added

  *********V1.08 22/06/2015**************
  Londelec Unix second definition added
  Boolean check macro added

  *********V1.07 16/02/2015**************
  EXIT_SUCCESS and EXIT_FAILURE need to be defined for Atmel
  Ethernet structures are not available for Atmel
  Some generic macros added

  *********V1.06 07/02/2015**************
  16bit flag definition created

  *********V1.05 02/02/2015**************
  Structures must be aligned when building for IMX287, don't use optional packed attribute

  *********V1.04 30/07/2013**************
  Compilation date expansion macros moved from main.h and commented out

  *********V1.03 13/06/2013**************
  Multiple packed attribute definitions required due to ARM compile compatibility

  *********V1.02 23/04/2013**************
  Filename changed to ledefs.h

  *********V1.01 13/05/2013**************
  Info header changed to bring in line with other files

  *********V1.00 19/10/2011**************
  Initial revision

 ============================================================================
 */

#ifndef LE_DEFS_H_
#define LE_DEFS_H_


#include <stdint.h>
#ifndef MCUTYPE
#include <sys/un.h>			// Unix socket
#include <netdb.h>			// Socket structures
#include <dirent.h>			// Directory structures
#include <time.h>			// localtime
#endif	// MCUTYPE


#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 0
#endif
#ifndef EXIT_FAILURE
#define EXIT_FAILURE 1
#endif


// Generic definitions
#define HOURINSEC						3600			// Hour in seconds
#define MININSEC						60				// Minute in seconds
#define DAYINSEC						86400			// Day in seconds
#define SECINNSEC						1000000000		// Second in nanoseconds
#define MSECINNSEC						1000000			// Milisecond in nanoseconds

// Function return status
#define LE_OK 							0
#define LE_FAIL 						1

// Boolean
#define LE_FALSE						0
#define LE_TRUE							1


// Force compiler to use min number of bytes for
// structure or enum members
#define LEHWPACK					__attribute__ ((packed))	// Pack hardware structures, mandatory
#if defined MOXA_W3X5A | defined IMX287
#define LEOPACK													// Don't pack structures where packing is optional
#else
#define LEOPACK						__attribute__ ((packed))	// Pack in order to save memory, optional
#endif // MOXA W3X5A
#define LEALIGN4					__attribute__ ((aligned (4)))	// Minimum alignment 4 bytes
#define LEWEAK						__attribute__ ((weak))			// Function initialized as weak, normally replaced by driver
#define LELIBCONSTRUCTOR			__attribute__ ((constructor))	// Library initialization constructor, executed before library loads


// This is normally defined in 'bits/time.h',
// but the header is not getting indexed properly due to an Eclipse bug.
// Delete this definition if different IDE indexes 'bits/time.h'
// correctly and throws redefinition error
/* Monotonic system-wide clock.  */
#ifdef MOXA_W3X5A
#define CLOCK_MONOTONIC					0		// Moxa's W325A kernel doesn't have Monotonic clock
#else
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC					1
#endif
#endif




// Generic definitions
typedef	struct sigaction			sigaction_t;		// Signal action structure
typedef	struct termios				termios_t;			// Terminal structure
typedef	int							sockflags_t;		// Socket flags size definition
typedef struct sockaddr				sockaddr_t;			// Socket address definition
typedef struct sockaddr_in 			sockinaddr_t;		// Socket AF_INET address definition
typedef struct sockaddr_un 			sockunaddr_t;		// Socket Unix address definition
typedef	enum __socket_type			socktype_t;			/* Socket type (SOCK_STREAM or SOCK_DGRAM) definition */
typedef struct in_addr				inaddr_t;			// Internet IPv4 address definition
typedef	struct if_nameindex			ifnameix_t;			// Interface name index definition
typedef	struct ifreq				ifreq_t;			// Interface IO parameters definition
typedef	unsigned int				ifix_t;				// Interface index size definition
typedef	int							fddef;				// File descriptor size definition
typedef	int							errn_t;				// Error Number definition
typedef	char						lechar;				// Character size definition
typedef	uintptr_t					leptr;				// Pointer size definition
typedef	float						lefloat;			// Short floating point size definition
typedef	struct timeval				unixtime_t;			// Unix time (sec and microsecs) structure definition
typedef uint32_t					lever_t;			// Londelec version number
typedef	uint32_t					timerconst_t;		// 32bit Timer Constant size definition
typedef	uint16_t					seqno_t;			// X25 sequence number size definition
typedef	uint8_t						fifo8_t;			// FIFO buffer size definition
typedef	uint16_t					fifo16_t;			// FIFO buffer size definition
typedef	uint16_t					txrx16_t;			// Rx/Tx buffer pointer size definition
typedef	uint16_t					llevel_t;			// Syslog logger level
#ifndef MCUTYPE
typedef	ssize_t						rxtxsize_t;			// recv/read/send function return size definition
typedef struct in6_addr				in6addr_t;			// IPv6 address definition
typedef	in_addr_t					ipv4addr_t;			// IPv4 address definition
typedef	in_port_t					inport_t;			// Internet address port size definition
typedef struct sockaddr_in6			sockin6addr_t;		// Socket AF_INET6 address definition
typedef struct addrinfo				addrinfo_t;			// Address of a service provider
typedef	__pid_t						lepid;				// Process identifier size definition
typedef	__uid_t						leuid;				// User identifier size definition
typedef	__gid_t						legid;				// Group identifier size definition
typedef	pthread_t					lethread;			// Thread identifier size definition
typedef fd_set						fdset_t;			// Directly accessible bit set
typedef __time_t					leunixsec_t;		// Unix second definition
typedef uint32_t					leepochsec_t;		// Londelec defined 32bit unsigned seconds since epoch
typedef uint64_t					leepoch64sec_t;		// Londelec defined 64bit seconds since epoch
typedef uint16_t					lemsec_t;			// Milisecond size definition
typedef __off_t						leofft;				// lseek offset size definition
typedef	DIR							dirdef;				// directory stream objects definition
typedef	FILE						filedef;			// IO file definition
typedef	struct stat 				filestat_t;			// file status structure definition
typedef	struct statvfs				vfsstatdef;			// Virtual file system status structure definition
typedef	struct pcap_pkthdr			pcaphdr_t;			// pcap header structure definition
typedef	struct serial_struct		leserialdef;		// linux kernel serial structure definition
typedef	struct dirent				direntdef;			// directory entry structure definition
#endif	// MCUTYPE


// !!! Following structures are not indexed properly,
// Eclipse bug fixed by inserting 'time.h' in indexer
// (Files to index up-front list) right after entry 'ctime'
typedef struct tm					localtime_t;		// Local time structure definition
typedef struct timespec				nanotime_t;			// Nanosecond time structure for Monotonic clock


// Application specific definitions
typedef	uint8_t						leflags8_t;			// Internal flag size definition
typedef	uint16_t					leflags16_t;		// Internal flag size definition


// Constant string container
typedef struct sbuff_s {
	lechar 					*buf;
	uint32_t 				size;
} sbuff_t;


// Dynamic string container
typedef struct scont_s {
	lechar					*buf;
	lechar					*tail;
	uint32_t				size;
} scont_t;


// Pointer container
typedef struct ptrcont_s {
	void 					**buf;
	uint32_t 				size;
} ptrcont_t;


// Unsigned integer container
typedef struct u32buff_s {
	uint8_t 				*buf;
	uint8_t					*head;
	uint32_t 				size;
} u32buff_t;


// Generic Macros
#define STRINGIFY_(s) 				#s
#define STRINGIFY(s)				STRINGIFY_(s)
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)				(sizeof(x) / sizeof((x)[0]))
#endif
#ifndef BOOL_CHECK
#define BOOL_CHECK(b)				((b) ? 1 : 0)
#endif

#define BYTE_ISNUM(mbyte) 			(((mbyte) >= 0x30) && ((mbyte) <= 0x39))
#define BYTE_ISCRLF(mbyte) 			(((mbyte) == '\r') || ((mbyte) == '\n'))
// Convert ASCII to decimal
#define ASCIIDEC_MACRO(mdec, mascii)\
		mdec *= 10;\
		mdec += (mascii & 0x0F);

#define BITSET_SET(mbs, mbit)\
		if (((mbit) >> 3) < sizeof(mbs)) {\
			mbs[(mbit) >> 3] |= 1 << ((mbit) & 0x07);\
		}

#define BITSET_TEST(mbs, mbit)		(((mbit) >> 3) < sizeof(mbs)) ? (mbs[(mbit) >> 3] & (1 << ((mbit) & 0x07))) : 0




/*// Macros for expanding the date string
#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')

#define BUILD_MONTH_CH1 \
    ((BUILD_MONTH_IS_OCT || BUILD_MONTH_IS_NOV || BUILD_MONTH_IS_DEC) ? '1' : '0')


#define BUILD_MONTH_CH0 \
    ( \
        (BUILD_MONTH_IS_JAN) ? '1' : \
        (BUILD_MONTH_IS_FEB) ? '2' : \
        (BUILD_MONTH_IS_MAR) ? '3' : \
        (BUILD_MONTH_IS_APR) ? '4' : \
        (BUILD_MONTH_IS_MAY) ? '5' : \
        (BUILD_MONTH_IS_JUN) ? '6' : \
        (BUILD_MONTH_IS_JUL) ? '7' : \
        (BUILD_MONTH_IS_AUG) ? '8' : \
        (BUILD_MONTH_IS_SEP) ? '9' : \
        (BUILD_MONTH_IS_OCT) ? '0' : \
        (BUILD_MONTH_IS_NOV) ? '1' : \
        (BUILD_MONTH_IS_DEC) ? '2' : \
        // error default //    '?' \
    )

#define BUILD_DAY_CH1 ((__DATE__[4] >= '0') ? (__DATE__[4]) : '0')
#define BUILD_DAY_CH0 (__DATE__[5])
*/


#endif /* LE_DEFS_H_ */
