/*
============================================================================
 Name        : timer.h
 Author      : AK
 Version     : V1.00
 Copyright   : Property of Londelec UK Ltd
 Description : Header file for Atmel timers

  Change log  :

  *********V1.00 19/02/2015**************
  Initial revision

 ============================================================================
 */

#ifndef TIMER_H_
#define TIMER_H_


#include "atmeldefs.h"

//#define INTERNAL_32M_CLOCK

#define CLKSYS_Enable( _oscSel ) ( OSC.CTRL |= (_oscSel) )

#define F_TIMER								F_CPU
#define FINETIMER_MCUTMR					TCC0				// Timer for 100us ticks
#define FINETIMER_ISR_VECTOR				TCC0_OVF_vect		// 100us timer vector
#define TIMER1MS_MCUTMR						TCC1
#define TIMER1MS_ISR_VECTOR					TCC1_OVF_vect		// 1ms timer vector

#define TIMERPER_1MSEC						(uint16_t)(F_TIMER / 1000UL)	// 1msec rollover value
#define TIMERPER_100USEC					(uint16_t)(F_TIMER / 10000UL)	// 100usec rollover value
#define SEC_100USEC							10000				// Second in 100us seconds


/*! \brief Sets the timer period.
 *
 *  This macro sets a new timer period. The period buffer register is not
 *  used, so the new period will be valid immediately after the 16-bit write
 *  is finished.
 *
 *  \param _tc               Timer/Counter module instance.
 *  \param _period           New Timer/Counter period.
 */
#define TC_SetPeriod( _tc, _period ) ( (_tc)->PER = (_period) )

/*! \brief This macro check if selected oscillator is ready.
 *
 *  This macro will return non-zero if is is running, regardless if it is
 *  used as a main clock source or not.
 *
 *  \param _oscSel Bitmask of selected clock. Can be one of the following
 *                 OSC_RC2MEN_bm, OSC_RC32MEN_bm, OSC_RC32KEN_bm, OSC_XOSCEN_bm,
 *                 OSC_PLLEN_bm.
 *
 *  \return  Non-zero if oscillator is ready and running.
 */
#define CLKSYS_IsReady( _oscSel ) ( OSC.STATUS & (_oscSel) )

/* Definition of watchdog macros *//*! \brief Check if Synchronization busy flag is set. */#define WDT_IsSyncBusy() ( WDT.STATUS & WDT_SYNCBUSY_bm )
/*! \brief This macro resets the Watchdog Timer. */#define wdt_reset() __asm__ __volatile__ ("wdr")

typedef struct finetm_s {
	timerconst_t			sec;					// Seconds
	uint16_t				usec100;				// 100x Microseconds
} finetm_t;


void clock_init();
void timer_init();

uint8_t CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_t clockSource );

#ifdef INTERNAL_32M_CLOCK
	void CLKSYS_AutoCalibration_Enable( uint8_t clkSource, uint8_t extReference );
#endif


void CCPWrite( volatile uint8_t * address, uint8_t value );

void timerset_fine(timerconst_t tconst, finetm_t *tptr);
uint8_t timercheck_fine(finetm_t *tptr);

void wdt_enable( WDT_PER_t period );

#endif /* TIMER_H_ */
