/*
 ============================================================================
 Name        : ledefs.h
 Author      : AK
 Version     : V1.07
 Copyright   : Property of Londelec UK Ltd
 Description : Global header file for Londelec C/C++ projects

  Change log  :

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
#include <time.h>			// localtime
#endif	// MCUTYPE


#define FALSE 0
#define TRUE 1


#ifndef EXIT_SUCCESS
#define EXIT_SUCCESS 0
#endif
#ifndef EXIT_FAILURE
#define EXIT_FAILURE 1
#endif


// Generic definitions
#define	HOURINSEC						3600			// Hour in seconds
//#define	DAYINSEC						86400			// Day in seconds
#define SECINNSEC						1000000000		// Second in nanoseconds


// Force compiler to use min number of bytes for
// structure or enum members
#define LEHWPACK					__attribute__ ((packed))	// Pack hardware structures, mandatory
#if defined MOXA_W3X5A | defined IMX287
#define LEOPACK													// Don't pack structures where packing is optional
#else
#define LEOPACK						__attribute__ ((packed))	// Pack in order to save memory, optional
#endif // MOXA W3X5A
#define LEWEAK						__attribute__((weak))		// Function initialized as weak, normally replaced by driver


// This is normally defined in 'bits/time.h',
// but the header is not getting indexed properly due to an Eclipse bug.
// Delete this definition if different IDE indexes 'bits/time.h'
// correctly and throws redefinition error
/* Monotonic system-wide clock.  */
#ifdef MOXA_W3X5A
#define CLOCK_MONOTONIC					0		// Moxa's W325A kernel doesn't have Monotonic clock
#else
#define CLOCK_MONOTONIC					1
#endif




// Generic definitions
typedef	struct sigaction			lesigaction;		/* Signal action structure */
typedef	struct termios				letermios;			/* Terminal structure */
typedef	int							sockflagsdef;		/* Socket flags size definition */
typedef struct sockaddr				sockaddrdef;		/* Socket address definition */
typedef struct sockaddr_in 			sockinetaddrdef;	/* Socket INET address definition */
typedef struct sockaddr_un 			sockunixaddrdef;	/* Socket Unix address definition */
typedef struct in_addr				inaddrdef;			/* Internet IPv4 address definition */
typedef	struct if_nameindex			ifnameixdef;		/* Interface name index definition */
typedef	struct ifreq				ifreqdef;			/* Interface IO parameters definition */
typedef	int							fddef;				/* File descriptor size definition */
typedef	int							errnodef;			/* Error Number definition */
typedef	char						lechar;				/* Character size definition */
typedef	uintptr_t					leptr;				/* Pointer size definition */
typedef	struct stat 				filestatdef;		/* file status structure definition */
typedef	struct timeval				unixtimedef;		/* Unix time (sec and microsecs) structure definition */
#ifndef MCUTYPE
typedef	ssize_t						rxbytesdef;			/* recv/read/send function return size definition */
typedef	ssize_t						retssizedef;		/* function return size definition */
typedef	in_addr_t					ipv4addrdef;		/* IPv4 address definition */
typedef	__pid_t						lepid;				/* Process identifier size definition */
typedef	pthread_t					lethread;			/* Thread identifier size definition */
typedef fd_set						lefdset;			/* Directly accessible bit set */
typedef __time_t					leunixsec;			/* Unix second definition */
#endif	// MCUTYPE


// !!! Following structures are not indexed properly,
// Eclipse bug fixed by inserting 'time.h' in indexer
// (Files to index up-front list) right after entry 'ctime'
typedef struct tm					localtimedef;		// Local time structure definition
typedef struct timespec				nanotimedef;		/* Nanosecond time structure for Monotonic clock */


// Application specific definitions
typedef	uint8_t						leflags8bit;		/* Internal flag size definition */
typedef	uint16_t					leflags16bit;		/* Internal flag size definition */


// Generic Macros
#define STRINGIFY_(s) 				#s
#define STRINGIFY(s)				STRINGIFY_(s)
#define ARRAY_SIZE(x)				(sizeof(x) / sizeof((x)[0]))


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
